use arrayvec::ArrayVec;
use core::any::{Any, TypeId};
use core::cell::RefCell;
use cortex_m::singleton;
use critical_section::Mutex;
use defmt::Format;
use fugit::HertzU32;
use pio::Program;
use pio_proc::pio_file;
use rp2040_hal::clocks::SystemClock;
use rp2040_hal::dma::{
    Channel, ChannelIndex, DoubleBuffering, DoubleBufferingConfig, ReadNext, ReadTarget,
    SingleBuffering, SingleChannel,
};
use rp2040_hal::gpio::DynPin;
use rp2040_hal::pio::{
    PIOExt, PinDir, Running, StateMachine, StateMachineIndex, Tx, UninitStateMachine, PIO,
};
use rp2040_hal::{self, Clock};

use crate::initialize_array;

type PinData = &'static mut Sequence;
#[allow(dead_code)]
type SingleTxTransfer<C, P, SM> = SingleBuffering<Channel<C>, PinData, Tx<(P, SM)>>;
type TxTransfer<C1, C2, P, SM> =
    DoubleBuffering<Channel<C1>, Channel<C2>, PinData, Tx<(P, SM)>, ReadNext<PinData>>;
type WaitingTxTransfer<C1, C2, P, SM> =
    DoubleBuffering<Channel<C1>, Channel<C2>, PinData, Tx<(P, SM)>, ()>;

const NUM_BUFFERS: usize = 3;
// Set to 64, the maximum number of single rises and falls for 32 channels within a looping time period
const BUFFER_SIZE: usize = 64;
// The number of dummy transitions to insert into the data to delay the DMA interrupt (if zero then
// no zone is used)
const LOADING_ZONE_SIZE: u32 = 3;
// The number of levels before the top to insert the load zone.
// Smaller values will make the DMA interrupt trigger closer to the time the data is needed,
// but risks stalling the PIO if the interrupt takes longer due to other processes.
const LOADING_ZONE_POSITION: u32 = 55;
const MAX_PWM_CLUSTER_WRAP: u64 = u16::MAX as u64;
// KEEP IN SYNC WITH pwm.pio!
const PWM_CLUSTER_CYCLES: u64 = 5;

pub struct GlobalState<C1, C2, P, SM>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt,
    SM: StateMachineIndex,
{
    handler: Option<InterruptHandler<C1, C2, P, SM>>,
    indices: Mutex<RefCell<Indices>>,
    sequences: Mutex<RefCell<SequenceList>>,
    sequence1: Sequence,
    sequence2: Sequence,
    sequence3: Sequence,
    loop_sequences: Mutex<RefCell<SequenceList>>,
    loop_sequence1: Sequence,
    loop_sequence2: Sequence,
    loop_sequence3: Sequence,
}

/// A trait to abstract over [GlobalState] structs with different generic parameters.
pub trait Handler: Any {
    fn try_next_dma_sequence(&mut self);
}

// Downcast hack since conversion between dyn Handler and dyn Any not yet supported.
// Copy of dyn Any handler (but only for mut variants).
impl dyn Handler {
    #[inline]
    pub(crate) fn downcast_mut<T: Any>(&mut self) -> Option<&mut T> {
        if self.is::<T>() {
            // SAFETY: just checked whether we are pointing to the correct type, and we can rely on
            // that check for memory safety because we have implemented Any for all types; no other
            // impls can exist as they would conflict with our impl.
            unsafe { Some(self.downcast_mut_unchecked()) }
        } else {
            None
        }
    }

    #[inline]
    /// # Safety
    /// caller guarantees that T is the correct type
    pub(crate) unsafe fn downcast_mut_unchecked<T: Any>(&mut self) -> &mut T {
        debug_assert!(self.is::<T>());
        // SAFETY: caller guarantees that T is the correct type
        unsafe { &mut *(self as *mut dyn Handler as *mut T) }
    }

    #[inline]
    pub(crate) fn is<T: Any>(&self) -> bool {
        // Get `TypeId` of the type this function is instantiated with.
        let t = TypeId::of::<T>();

        // Get `TypeId` of the type in the trait object (`self`).
        let concrete = self.type_id();

        // Compare both `TypeId`s on equality.
        t == concrete
    }
}

impl<C1, C2, P, SM> Handler for GlobalState<C1, C2, P, SM>
where
    C1: ChannelIndex + 'static,
    C2: ChannelIndex + 'static,
    P: PIOExt + 'static,
    SM: StateMachineIndex + 'static,
{
    fn try_next_dma_sequence(&mut self) {
        if let Some(ref mut handler) = self.handler {
            handler.try_next_dma_sequence();
        }
    }
}

/// A struct that can be used to store multiple [GlobalState]s.
pub struct GlobalStates<const NUM_CHANNELS: usize> {
    pub states: [Option<&'static mut dyn Handler>; NUM_CHANNELS],
}

impl<const NUM_CHANNELS: usize> GlobalStates<NUM_CHANNELS> {
    /// Returns global state if types match.
    pub(crate) fn get_mut<C1, C2, P, SM, F>(
        &mut self,
        _channels: &mut (Channel<C1>, Channel<C2>),
        ctor: F,
    ) -> Option<*mut GlobalState<C1, C2, P, SM>>
    where
        C1: ChannelIndex + 'static,
        C2: ChannelIndex + 'static,
        P: PIOExt + 'static,
        SM: StateMachineIndex + 'static,
        F: FnOnce() -> &'static mut GlobalState<C1, C2, P, SM>,
    {
        let entry = &mut self.states[<C1 as ChannelIndex>::id() as usize];
        if entry.is_none() {
            *entry = Some(ctor())
        }

        let state: *mut &'static mut dyn Handler = entry.as_mut().unwrap() as *mut _;
        // Safety: Already reference to static mut, which is already unsafe.
        let state: &'static mut dyn Handler = unsafe { *state };
        <dyn Handler>::downcast_mut::<GlobalState<C1, C2, P, SM>>(state).map(|d| d as *mut _)
    }
}

/// Indices for the current sequence to write to.
struct Indices {
    /// The last written index.
    last_written_index: usize,
    /// The current read index.
    read_index: usize,
}

impl Indices {
    fn new() -> Self {
        Self {
            last_written_index: 0,
            read_index: 0,
        }
    }
}

/// Builder for array of [Sequence]s.
struct SequenceListBuilder;
type SequenceList = [Option<*mut Sequence>; NUM_BUFFERS];

impl SequenceListBuilder {
    /// Construct a list of sequences with the first transition in each defaulted to a delay of 10.
    fn build() -> SequenceList {
        initialize_array::<Option<*mut Sequence>, NUM_BUFFERS>(|| None)
    }
}

pub struct InterruptHandler<C1, C2, P, SM>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt,
    SM: StateMachineIndex,
{
    sequences: &'static Mutex<RefCell<SequenceList>>,
    loop_sequences: &'static Mutex<RefCell<SequenceList>>,
    indices: &'static Mutex<RefCell<Indices>>,
    tx_transfer: Option<TxTransfer<C1, C2, P, SM>>,
    /// Track where the last sequence came from so we can put it back.
    /// None means it came from the singletons, so don't worry
    last_was_loop: Option<bool>,
}

/// DMA interrupt handler. Call directly from DMA_IRQ_0 or DMA_IRQ_1.
#[inline]
pub fn dma_interrupt<const NUM_PINS: usize>(global_state: &'static mut GlobalStates<NUM_PINS>) {
    for state in global_state.states.iter_mut().flatten() {
        state.try_next_dma_sequence();
    }
}

impl<C1, C2, P, SM> InterruptHandler<C1, C2, P, SM>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt,
    SM: StateMachineIndex,
{
    /// Try to setup the next dma sequence..
    fn try_next_dma_sequence(&mut self) {
        let (tx_buf, tx) = {
            if let Some(mut tx_transfer) = self.tx_transfer.take() {
                // Check the interrupt to clear it if this is the transfer that's ready.
                if tx_transfer.check_irq0() && tx_transfer.is_done() {
                    tx_transfer.wait()
                } else {
                    // Either this wasn't the transfer that triggered the interrupt, or it did, but
                    // for some reason it's not ready. Place it back so we can try again next time.
                    self.tx_transfer = Some(tx_transfer);
                    return;
                }
            } else {
                // This interrupt handler has not been configured with a handler yet.
                return;
            }
        };
        self.next_dma_sequence(tx_buf, tx);
    }

    fn next_dma_sequence(
        &mut self,
        tx_buf: &'static mut Sequence,
        tx: WaitingTxTransfer<C1, C2, P, SM>,
    ) {
        critical_section::with(|cs| {
            let mut indices = self.indices.borrow(cs).borrow_mut();
            // If there was a write since the last read...
            let next_buf = if indices.last_written_index != indices.read_index {
                if let Some(last_was_loop) = self.last_was_loop {
                    // Put the sequence back before updating.
                    if last_was_loop {
                        self.loop_sequences.borrow(cs).borrow_mut()[indices.read_index] =
                            Some(tx_buf);
                    } else {
                        self.sequences.borrow(cs).borrow_mut()[indices.read_index] = Some(tx_buf);
                    }
                }

                // Update the read index and use sequences.
                indices.read_index = indices.last_written_index;
                self.last_was_loop = Some(false);
                self.sequences.borrow(cs).borrow_mut()[indices.read_index]
                    .take()
                    .unwrap()
            } else {
                if let Some(false) = self.last_was_loop {
                    // Put the sequence back before updating.
                    self.sequences.borrow(cs).borrow_mut()[indices.read_index] = Some(tx_buf);
                }

                // Otherwise just use the loop sequences.
                if let Some(sequence) =
                    self.loop_sequences.borrow(cs).borrow_mut()[indices.read_index].take()
                {
                    self.last_was_loop = Some(true);
                    // We took ownership from the sequence.
                    sequence
                } else {
                    // We already have the buffer, so just re-use it.
                    tx_buf as *mut _
                }
            };

            // Start the next transfer.
            // Safety: We took ownership from the sequence list, so we're the only
            // location that has access to this unique reference.
            self.tx_transfer = Some(tx.read_next(unsafe { &mut *next_buf }));
        });
    }
}

/// A type to manage a cluster of PWM signals.
pub struct PwmCluster<const NUM_PINS: usize, P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    sm: Option<StateMachine<(P, SM), Running>>,
    channel_to_pin_map: [u8; NUM_PINS],
    channels: [ChannelState; NUM_PINS],
    sequences: &'static Mutex<RefCell<SequenceList>>,
    loop_sequences: &'static Mutex<RefCell<SequenceList>>,
    indices: &'static Mutex<RefCell<Indices>>,
    transitions: [TransitionData; BUFFER_SIZE],
    looping_transitions: [TransitionData; BUFFER_SIZE],
    loading_zone: bool,
    top: u32,
}

/// A type to build a [PwmCluster]
pub struct PwmClusterBuilder<const NUM_PINS: usize> {
    pin_mask: u32,
    #[cfg(deature = "debug_pio")]
    side_set_pin: u8,
    channel_to_pin_map: [u8; NUM_PINS],
}

impl<const NUM_PINS: usize> PwmClusterBuilder<NUM_PINS> {
    /// Construct a new [PwmClusterBuilder].
    pub fn new() -> Self {
        Self {
            pin_mask: 0,
            #[cfg(deature = "debug_pio")]
            side_set_pin: 0,
            channel_to_pin_map: [0; NUM_PINS],
        }
    }

    /// Set the pin_base for this cluster. NUM_PINS will be used to determine the pin count.
    pub fn pin_base(mut self, base_pin: DynPin) -> Self {
        let base_pin = base_pin.id().num;
        for (i, pin_map) in self.channel_to_pin_map.iter_mut().enumerate() {
            let pin_id = base_pin + i as u8;
            self.pin_mask |= 1 << pin_id;
            *pin_map = pin_id;
        }
        self
    }

    /// Set the pins directly from the `pins` parameter.
    pub fn pins(mut self, pins: &[DynPin; NUM_PINS]) -> Self {
        for (pin, pin_map) in pins.iter().zip(self.channel_to_pin_map.iter_mut()) {
            let pin_id = pin.id().num;
            self.pin_mask |= 1 << pin_id;
            *pin_map = pin_id;
        }

        self
    }

    /// Set the side pin for debugging.
    ///
    /// This method can be enabled with the "debug_pio" feature.
    #[cfg(feature = "debug_pio")]
    pub fn side_pin(mut self, side_set_pin: &DynPin) -> Self {
        self.side_set_pin = side_set_pin.id().num;
        self
    }

    /// Initialize [GlobalState]
    pub(crate) fn prep_global_state<C1, C2, P, SM>(
        global_state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
    ) -> &'static mut GlobalState<C1, C2, P, SM>
    where
        C1: ChannelIndex,
        C2: ChannelIndex,
        P: PIOExt,
        SM: StateMachineIndex,
    {
        if global_state.is_none() {
            *global_state = Some(GlobalState {
                handler: None,
                indices: Mutex::new(RefCell::new(Indices::new())),
                sequences: Mutex::new(RefCell::new(SequenceListBuilder::build())),
                sequence1: Sequence::new_for_list(),
                sequence2: Sequence::new_for_list(),
                sequence3: Sequence::new_for_list(),
                loop_sequences: Mutex::new(RefCell::new(SequenceListBuilder::build())),
                loop_sequence1: Sequence::new_for_list(),
                loop_sequence2: Sequence::new_for_list(),
                loop_sequence3: Sequence::new_for_list(),
            });
            {
                let state = (*global_state).as_mut().unwrap();
                critical_section::with(|cs| {
                    // Safety: These self-referential fields are ok because the global state is
                    // guaranteed to be stored in a static.
                    let mut sequences = state.sequences.borrow(cs).borrow_mut();
                    sequences[0] = Some(&mut state.sequence1 as *mut _);
                    sequences[1] = Some(&mut state.sequence2 as *mut _);
                    sequences[2] = Some(&mut state.sequence3 as *mut _);
                    let mut loop_sequences = state.loop_sequences.borrow(cs).borrow_mut();
                    loop_sequences[0] = Some(&mut state.loop_sequence1 as *mut _);
                    loop_sequences[1] = Some(&mut state.loop_sequence2 as *mut _);
                    loop_sequences[2] = Some(&mut state.loop_sequence3 as *mut _);
                });
            }
        }
        global_state.as_mut().unwrap()
    }

    /// Build a PwmCluster.
    ///
    /// # Safety
    /// Caller must ensure that global_state is not being read/mutated anywhere else.
    #[allow(clippy::too_many_arguments)]
    pub unsafe fn build<C1, C2, P, SM>(
        self,
        servo_pins: [DynPin; NUM_PINS],
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        mut dma_channels: (Channel<C1>, Channel<C2>),
        sys_clock: &SystemClock,
        global_state: *mut GlobalState<C1, C2, P, SM>,
    ) -> PwmCluster<NUM_PINS, P, SM>
    where
        C1: ChannelIndex,
        C2: ChannelIndex,
        P: PIOExt,
        SM: StateMachineIndex,
    {
        let program = Self::pio_program();
        let installed = pio.install(&program).unwrap();
        const DESIRED_CLOCK_HZ: u32 = 500_000;
        let sys_hz = sys_clock.freq().to_Hz();
        let (int, frac) = (
            (sys_hz / DESIRED_CLOCK_HZ) as u16,
            (sys_hz as u64 * 256 / DESIRED_CLOCK_HZ as u64) as u8,
        );
        let (mut sm, _, tx) = {
            let mut builder = rp2040_hal::pio::PIOBuilder::from_program(installed);
            #[cfg(not(feature = "debug_pio"))]
            {
                builder = builder.out_pins(0, 32)
            }
            #[cfg(deature = "debug_pio")]
            {
                builder = builder
                    .out_pins(0, self.side_set_pin)
                    .side_set_pin_base(self.side_set_pin)
            }
            builder.clock_divisor_fixed_point(int, frac).build(sm)
        };
        sm.set_pindirs({
            #[allow(unused_mut)]
            let mut iter = servo_pins.into_iter().map(|pin| pin.id().num);
            #[cfg(feature = "debug_pio")]
            {
                iter = iter.chain(Some(self.side_set_pin).into_iter())
            }
            iter.map(|id| (id, PinDir::Output))
        });

        let sequence = Sequence::new_for_list();

        // Safety: caller guarantees that global_state is not being read/mutated anywhere else.
        let mut interrupt_handler = unsafe {
            InterruptHandler {
                sequences: &(*global_state).sequences,
                loop_sequences: &(*global_state).loop_sequences,
                indices: &(*global_state).indices,
                tx_transfer: None,
                last_was_loop: None,
            }
        };

        let tx_buf = singleton!(: Sequence = sequence.clone()).unwrap();
        dma_channels.0.listen_irq0();
        dma_channels.1.listen_irq0();
        let tx = DoubleBufferingConfig::new(dma_channels, tx_buf, tx).start();
        let tx_buf = singleton!(: Sequence = sequence).unwrap();
        interrupt_handler.next_dma_sequence(tx_buf, tx);

        // Safety: caller guarantees that global_state is not being read/mutated anywhere else.
        unsafe { (*global_state).handler = Some(interrupt_handler) };

        let sm = sm.start();

        // Safety: caller guarantees that global_state is not being read/mutated anywhere else.
        unsafe {
            PwmCluster::new(
                sm,
                self.channel_to_pin_map,
                &(*global_state).indices,
                &(*global_state).sequences,
                &(*global_state).loop_sequences,
            )
        }
    }

    /// Get PIO program data.
    #[cfg(feature = "debug_pio")]
    fn pio_program() -> Program<32> {
        let pwm_program = pio_file!("./src/pwm.pio", select_program("debug_pwm_cluster"));
        pwm_program.program
    }

    /// Get PIO program data.
    #[cfg(not(feature = "debug_pio"))]
    fn pio_program() -> Program<32> {
        let pwm_program = pio_file!("./src/pwm.pio", select_program("pwm_cluster"));
        pwm_program.program
    }
}

impl<const NUM_PINS: usize> Default for PwmClusterBuilder<NUM_PINS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const NUM_PINS: usize, P, SM> PwmCluster<NUM_PINS, P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    /// The number of channels used by this cluster.
    pub const CHANNEL_COUNT: usize = NUM_PINS;
    /// The number of channel pairs used by this cluster.
    pub const CHANNEL_PAIR_COUNT: usize = NUM_PINS / 2;

    /// Get a [PwmClusterBuilder].
    pub fn builder() -> PwmClusterBuilder<NUM_PINS> {
        PwmClusterBuilder::new()
    }

    /// Construct the cluster.
    fn new(
        sm: StateMachine<(P, SM), Running>,
        channel_to_pin_map: [u8; NUM_PINS],
        indices: &'static Mutex<RefCell<Indices>>,
        sequences: &'static Mutex<RefCell<SequenceList>>,
        loop_sequences: &'static Mutex<RefCell<SequenceList>>,
    ) -> Self {
        let channels = [ChannelState::new(); NUM_PINS];
        let transitions = [TransitionData::default(); BUFFER_SIZE];
        let looping_transitions = [TransitionData::default(); BUFFER_SIZE];

        Self {
            sm: Some(sm),
            channel_to_pin_map,
            channels,
            sequences,
            loop_sequences,
            loading_zone: false,
            top: 0,
            indices,
            transitions,
            looping_transitions,
        }
    }

    /// Calculate the pwm factors (div and frac) from the system clock and desired frequency.
    pub fn calculate_pwm_factors(system_clock_hz: HertzU32, freq: f32) -> Option<(u32, u32)> {
        let source_hz = system_clock_hz.to_Hz() as u64 / PWM_CLUSTER_CYCLES;

        // Check the provided frequency is valid
        if (freq >= 0.01) && (freq <= (source_hz >> 1) as f32) {
            let mut div256_top = ((source_hz << 8) as f32 / freq) as u64;
            let mut top: u64 = 1;

            loop {
                // Try a few small prime factors to get close to the desired frequency.
                if (div256_top >= (11 << 8))
                    && (div256_top % 11 == 0)
                    && (top * 11 <= MAX_PWM_CLUSTER_WRAP)
                {
                    div256_top /= 11;
                    top *= 11;
                } else if (div256_top >= (7 << 8))
                    && (div256_top % 7 == 0)
                    && (top * 7 <= MAX_PWM_CLUSTER_WRAP)
                {
                    div256_top /= 7;
                    top *= 7;
                } else if (div256_top >= (5 << 8))
                    && (div256_top % 5 == 0)
                    && (top * 5 <= MAX_PWM_CLUSTER_WRAP)
                {
                    div256_top /= 5;
                    top *= 5;
                } else if (div256_top >= (3 << 8))
                    && (div256_top % 3 == 0)
                    && (top * 3 <= MAX_PWM_CLUSTER_WRAP)
                {
                    div256_top /= 3;
                    top *= 3;
                } else if (div256_top >= (2 << 8)) && (top * 2 <= MAX_PWM_CLUSTER_WRAP) {
                    div256_top /= 2;
                    top *= 2;
                } else {
                    break;
                }
            }

            // Only return valid factors if the divisor is actually achievable.
            if div256_top >= 256 && div256_top <= ((u8::MAX as u64) << 8) {
                Some((top as u32, div256_top as u32))
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Set the clock divisor for this cluster.
    pub fn clock_divisor_fixed_point(&mut self, div: u16, frac: u8) {
        if let Some(sm) = self.sm.take() {
            let mut sm = sm.stop();
            sm.clock_divisor_fixed_point(div, frac);
            self.sm = Some(sm.start());
        }
    }

    /// The configured pwm level for the supplied `channel`.
    pub fn channel_level(&self, channel: u8) -> Result<u32, PwmError> {
        self.channels
            .get(channel as usize)
            .map(|c| c.level)
            .ok_or(PwmError::MissingChannel)
    }

    /// Set the pwm level for the supplied `channel`. If `load` is true, update
    /// the cluster's data in the PIO state machine.
    pub fn set_channel_level(
        &mut self,
        channel: u8,
        level: u32,
        load: bool,
    ) -> Result<(), PwmError> {
        let res = self
            .channels
            .get_mut(channel as usize)
            .map(|c| c.level = level)
            .ok_or(PwmError::MissingChannel);
        if load {
            self.load_pwm();
        }
        res
    }

    pub fn channel_offset(&self, channel: u8) -> Result<u32, PwmError> {
        self.channels
            .get(channel as usize)
            .map(|c| c.offset)
            .ok_or(PwmError::MissingChannel)
    }

    /// Set the start offset for the supplied `channel`. If `load` is true,
    /// update the cluster's data in the PIO state machine.
    pub fn set_channel_offset(
        &mut self,
        channel: u8,
        offset: u32,
        load: bool,
    ) -> Result<(), PwmError> {
        self.channels
            .get_mut(channel as usize)
            .map(|c| c.offset = offset)
            .ok_or(PwmError::MissingChannel)?;
        if load {
            self.load_pwm();
        }
        Ok(())
    }

    pub fn channel_polarity(&self, channel: u8) -> Result<bool, PwmError> {
        self.channels
            .get(channel as usize)
            .map(|c| c.polarity)
            .ok_or(PwmError::MissingChannel)
    }

    /// Set the polarity for the supplied `channel`. If `load` is true, update
    /// the cluster's data in the PIO state machine.
    pub fn set_channel_polarity(
        &mut self,
        channel: u8,
        polarity: bool,
        load: bool,
    ) -> Result<(), PwmError> {
        self.channels
            .get_mut(channel as usize)
            .map(|c| c.polarity = polarity)
            .ok_or(PwmError::MissingChannel)?;
        if load {
            self.load_pwm()
        }
        Ok(())
    }

    /// Gets the top value for the pwm counter. This is equivalent to [`Slice::get_top`] when using
    /// [pac::PWM].
    ///
    /// [`Slice::get_top`]: rp2040_hal::pwm::Slice
    pub fn top(&self) -> u32 {
        self.top
    }

    /// Set the top value for the supplied `channel`. If `load` is true, update
    /// the cluster's data in the PIO state machine.
    pub fn set_top(&mut self, top: u32, load_pwm: bool) {
        self.top = top.max(1); // Cannot have a wrap of zero.
        if load_pwm {
            self.load_pwm();
        }
    }

    /// Load the pwm data into the PIO state machine.
    pub fn load_pwm(&mut self) {
        let mut data_size = 0;
        let mut looping_data_size = 0;

        // Start with all pins low.
        let mut pin_states = 0;

        // Check if the data we last wrote has been picked up by the DMA yet.
        let read_since_last_write = critical_section::with(|cs| {
            let indices = self.indices.borrow(cs).borrow();
            indices.read_index == indices.last_written_index
        });

        // Go through each channel that we are assigned to.
        for (channel_idx, state) in self.channels.iter_mut().enumerate() {
            // Invert this channel's initial state of its polarity invert is set.
            if state.polarity {
                pin_states |= 1 << self.channel_to_pin_map[channel_idx];
            }

            let channel_start = state.offset;
            let channel_end = state.offset + state.level;
            let channel_wrap_end = channel_end % self.top;

            // If the data as been read, copy the channel overruns from that sequence.
            // Otherwise, keep the ones we previously stored.
            if read_since_last_write {
                // This condition was added to deal with cases of load_pwm() being called multiple
                // times between DMA reads, and thus losing memory of the previous sequence's
                // overruns.
                state.overrun = state.next_overrun;
            }

            // Always clear the next channel overruns, as we are loading new data.
            state.next_overrun = 0;

            // Did the previous sequence overrun the top level?
            if state.overrun > 0 {
                // Flip the initial state so the pin starts "high" (or "low" if polarity inverted).
                pin_states ^= 1 << self.channel_to_pin_map[channel_idx];

                // Is our end level before our start level?
                if channel_wrap_end < channel_start {
                    // Yes, so add a transition to "low" (or "high" if polarity inverted) at the end
                    // level, rather than the overrun (so our pulse takes effect earlier).
                    Self::sorted_insert(
                        &mut self.transitions,
                        &mut data_size,
                        TransitionData::new(channel_idx as u8, channel_wrap_end, state.polarity),
                    )
                } else if state.overrun < channel_start {
                    // No, so add a transition to "low" (or "high" if polarity inverted) at the
                    // overrun instead.
                    Self::sorted_insert(
                        &mut self.transitions,
                        &mut data_size,
                        TransitionData::new(channel_idx as u8, state.overrun, state.polarity),
                    )
                }
            }

            // Is the state level greater than zero, and the start level within the top?
            if state.level > 0 && channel_start < self.top {
                // Add a transition to "high" (or "low" if polarity inverted) at the start level.
                Self::sorted_insert(
                    &mut self.transitions,
                    &mut data_size,
                    TransitionData {
                        channel: channel_idx as u8,
                        level: channel_start,
                        state: !state.polarity,
                        dummy: false,
                    },
                );
                Self::sorted_insert(
                    &mut self.looping_transitions,
                    &mut looping_data_size,
                    TransitionData {
                        channel: channel_idx as u8,
                        level: channel_start,
                        state: !state.polarity,
                        dummy: false,
                    },
                );

                // If the channel has overrun the top level, record by how much.
                if channel_wrap_end < channel_start {
                    state.next_overrun = channel_wrap_end;
                }
            }

            // Is the state level within the top?
            if state.level < self.top {
                // Is the end level within the wrap too?
                if channel_end < self.top {
                    // Add a transition to "low" (or "high" if the polarity inverted) at the end level.
                    Self::sorted_insert(
                        &mut self.transitions,
                        &mut data_size,
                        TransitionData::new(channel_idx as u8, channel_end, state.polarity),
                    );
                }

                // Add a transition to "low" (or "high" if polarity inverted) at the top level.
                Self::sorted_insert(
                    &mut self.looping_transitions,
                    &mut looping_data_size,
                    TransitionData::new(channel_idx as u8, channel_wrap_end, state.polarity),
                );
            }
        }

        if self.loading_zone {
            // Introduce "Loading Zone" transitions to the end of the sequence to
            // prevent the DMA interrupt firing many milliseconds before the sequence ends.
            let zone_inserts = LOADING_ZONE_SIZE.min(self.top - LOADING_ZONE_POSITION);
            for i in (zone_inserts + LOADING_ZONE_POSITION)..LOADING_ZONE_POSITION {
                Self::sorted_insert(
                    &mut self.transitions,
                    &mut data_size,
                    TransitionData::with_level(self.top - i),
                );
                Self::sorted_insert(
                    &mut self.looping_transitions,
                    &mut looping_data_size,
                    TransitionData::with_level(self.top - i),
                );
            }
        }

        // Read | Last W = Write
        // 0    | 0      = 1 (or 2)
        // 0    | 1      = 2
        // 0    | 2      = 1
        // 1    | 0      = 2
        // 1    | 1      = 2 (or 0)
        // 1    | 2      = 0
        // 2    | 0      = 1
        // 2    | 1      = 0
        // 2    | 2      = 0 (or 1)

        // Choose the write index based on the read and last written indices (using the above table).
        let write_index = critical_section::with(|cs| {
            let indices = self.indices.borrow(cs).borrow();
            let write_index = (indices.read_index + 1) % NUM_BUFFERS;
            if write_index == indices.last_written_index {
                (write_index + 1) % NUM_BUFFERS
            } else {
                write_index
            }
        });

        self.populate_sequence(
            TransitionType::Normal,
            data_size,
            write_index,
            &mut pin_states,
        );
        self.populate_sequence(
            TransitionType::Loop,
            looping_data_size,
            write_index,
            &mut pin_states,
        );

        critical_section::with(|cs| {
            let mut indices = self.indices.borrow(cs).borrow_mut();
            indices.last_written_index = write_index;
        });
    }

    /// Insert `data` into `transitions` in sorted order.
    fn sorted_insert(transitions: &mut [TransitionData], size: &mut usize, data: TransitionData) {
        let mut i = *size;
        while i > 0 && transitions[i - 1].level > data.level {
            transitions[i] = transitions[i - 1];
            i -= 1;
        }
        transitions[i] = data;
        *size += 1;
    }

    /// Populate the sequence of kind `TransitionType` with `transition_size` number of elements
    /// into sequence at `sequence_id`. Updates `pin_states` with the current states for each pin.
    fn populate_sequence(
        &mut self,
        transition_type: TransitionType,
        transition_size: usize,
        sequence_id: usize,
        pin_states: &mut u32,
    ) {
        critical_section::with(|cs| {
            let mut sequences;
            let mut loop_sequences;
            let (transitions, sequence) = match transition_type {
                TransitionType::Normal => {
                    sequences = self.sequences.borrow(cs).borrow_mut();
                    (
                        &self.transitions[..transition_size],
                        sequences[sequence_id].as_mut().unwrap(),
                    )
                }
                TransitionType::Loop => {
                    loop_sequences = self.loop_sequences.borrow(cs).borrow_mut();
                    (
                        &self.looping_transitions[..transition_size],
                        loop_sequences[sequence_id].as_mut().unwrap(),
                    )
                }
            };

            // Reset the sequence, otherwise we end up appending and weird thing happen.
            // Safety: We only mutate this if we can get a reference to it. The interrupt takes
            // ownership of the sequence when it's in use by the DMA controller.
            (unsafe { &mut **sequence }).data.clear();

            let mut data_index = 0;
            let mut current_level = 0;

            // Populate the selected write sequence with pin states and delays.
            while data_index < transition_size {
                // Set the next level to be the top, initially.
                let mut next_level = self.top;

                loop {
                    // Is the level of this transition at the current level being checked?
                    let transition = &transitions[data_index];
                    if transition.level <= current_level {
                        // Yes, so add the transition state to the pin states mask, if it's not a
                        // dummy transition.
                        if !transition.dummy {
                            if transition.state {
                                *pin_states |=
                                    1 << self.channel_to_pin_map[transition.channel as usize];
                            } else {
                                *pin_states &=
                                    !(1 << self.channel_to_pin_map[transition.channel as usize]);
                            }
                        }

                        // Move on to the next transition.
                        data_index += 1;
                    } else {
                        // No, it is higher, so set it as our next level and break out of the loop.
                        next_level = transition.level;
                        break;
                    }

                    if data_index >= transition_size {
                        break;
                    }
                }

                // Add the transition to the sequence.
                let transition = Transition {
                    mask: *pin_states,
                    delay: (next_level - current_level) - 1,
                };
                // Safety: We only mutate this if we can get a reference to it. The interrupt takes
                // ownership of the sequence when it's in use by the DMA controller.
                (unsafe { &mut **sequence }).data.push(transition);
                current_level = next_level;
            }
        });
    }
}

/// Error for PwnCluster methods.
pub enum PwmError {
    /// The supplied channel was not part of the PwmCluster.
    MissingChannel,
}

/// Kinds of transitions.
#[derive(Copy, Clone, Format)]
enum TransitionType {
    /// Normal types are used when a new update comes in.
    Normal,
    /// Loop types are used to repeat the currently setup structure of updates.
    Loop,
}

/// State used for each channel.
#[derive(Copy, Clone, Format)]
struct ChannelState {
    /// The level the channel is currently set to.
    level: u32,
    /// The offset the channel should use when starting a new high signal.
    offset: u32,
    /// Whether to invert the high/low signals.
    polarity: bool,
    /// Track when level wraps around the `top` value of the cluster.
    overrun: u32,
    /// Helper storage for overrun to account for multiple loads between DMA reads.
    next_overrun: u32,
}

impl ChannelState {
    fn new() -> Self {
        Self {
            level: 0,
            offset: 0,
            polarity: false,
            overrun: 0,
            next_overrun: 0,
        }
    }
}

/// A Sequence of [Transition]s
#[derive(Clone)]
pub struct Sequence {
    /// Inner array of transitions.
    data: ArrayVec<Transition, BUFFER_SIZE>,
}

impl Sequence {
    /// Constructor for a Sequence.
    pub fn new() -> Self {
        let mut data = ArrayVec::default();
        data.push(Transition::new());
        Self { data }
    }

    fn new_for_list() -> Self {
        let mut this = Self::new();
        this.data[0].delay = 10;
        this
    }
}

impl Default for Sequence {
    fn default() -> Self {
        Self::new()
    }
}

// ReadTarget allows Sequence to be used directly by the DMA.
impl ReadTarget for &mut Sequence {
    type ReceivedWord = u32;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        (self.data.as_ptr() as u32, self.data.len() as u32 * 2)
    }

    fn rx_increment(&self) -> bool {
        true
    }
}

/// Data to be sent to the PIO program.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Transition {
    /// Mask for pin states. All low bits turn off the signal for an output pin,
    /// and all high bits turn on the signal for an output pin.
    mask: u32,
    /// The number of cycles to wait before activating the next [Transition].
    delay: u32,
}

impl Format for Transition {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Transition {{ mask: {:#032b}, delay: {} }}",
            self.mask,
            self.delay
        )
    }
}

impl Transition {
    /// Constructor for a [Transition].
    fn new() -> Self {
        Self { mask: 0, delay: 0 }
    }
}

/// Data for the PwmCluster to track transitions.
#[derive(Copy, Clone, Default, Format)]
struct TransitionData {
    /// The channel that this transition applies to.
    channel: u8,
    /// The level when this transition should occur.
    level: u32,
    /// The state tracks whether to emit a high or low signal.
    state: bool,
    /// Dummy states just keep the same state but allow delaying the DMA interrupt.
    dummy: bool,
}

impl TransitionData {
    /// Construct a transition for `channel` at `level` and set it to `state`.
    fn new(channel: u8, level: u32, state: bool) -> Self {
        Self {
            channel,
            level,
            state,
            dummy: false,
        }
    }

    /// Construct a dummy transition at `level`.
    fn with_level(level: u32) -> Self {
        Self {
            channel: 0,
            level,
            state: false,
            dummy: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transition_must_be_exact() {
        assert_eq!(core::mem::size_of::<Transition>(), 2);
        assert_eq!(core::mem::size_of::<MaybeUninit<Transition>>(), 2);
    }
}
