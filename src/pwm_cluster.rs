use core::any::Any;
use core::cell::RefCell;
use core::mem::MaybeUninit;
use cortex_m::singleton;
use critical_section::Mutex;
use fugit::HertzU32;
use pimoroni_servo2040::hal;
use pimoroni_servo2040::hal::dma::{
    Channel, ChannelIndex, ReadTarget, SingleBuffering, SingleBufferingConfig, SingleChannel,
};
use pimoroni_servo2040::hal::gpio::DynPin;
use pimoroni_servo2040::hal::pio::{
    PIOExt, PinDir, Running, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine,
    ValidStateMachine, PIO,
};
use pio_proc::pio_file;

type PinData = &'static mut Sequence;
type TxTransfer<C, P, SM> = SingleBuffering<Channel<C>, PinData, Tx<(P, SM)>>;

const NUM_BUFFERS: usize = 2;
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

pub struct GlobalState<C, P, SM>
where
    C: ChannelIndex,
    P: PIOExt,
    SM: StateMachineIndex,
{
    handler: Option<InterruptHandler<C, P, SM>>,
    indices: Mutex<RefCell<Indices>>,
    sequences: Mutex<RefCell<SequenceList>>,
    loop_sequences: Mutex<RefCell<SequenceList>>,
}

pub trait Handler: Any {
    fn try_next_dma_sequence(&mut self);
}

impl Handler for () {
    fn try_next_dma_sequence(&mut self) {
        // no-op
    }
}

enum PioStateMachine<VSM>
where
    VSM: ValidStateMachine,
{
    Stopped(StateMachine<VSM, Stopped>),
    Transitioning,
    Running(StateMachine<VSM, Running>),
}

impl<C, P, SM> Handler for GlobalState<C, P, SM>
where
    C: ChannelIndex + 'static,
    P: PIOExt + 'static,
    SM: StateMachineIndex + 'static,
{
    fn try_next_dma_sequence(&mut self) {
        if let Some(ref mut handler) = self.handler {
            handler.try_next_dma_sequence();
        }
    }
}

pub struct GlobalStates<const NUM_CHANNELS: usize> {
    pub states: [Option<&'static mut dyn Handler>; NUM_CHANNELS],
}

impl<const NUM_CHANNELS: usize> GlobalStates<NUM_CHANNELS> {
    pub(crate) fn get_mut<C, P, SM, F>(
        &mut self,
        channel: &mut Channel<C>,
        ctor: F,
    ) -> &'static mut GlobalState<C, P, SM>
    where
        C: ChannelIndex + 'static,
        P: PIOExt + 'static,
        SM: StateMachineIndex + 'static,
        F: FnOnce() -> &'static mut GlobalState<C, P, SM>,
    {
        let entry = &mut self.states[channel.id() as usize];
        if entry.is_none() {
            *entry = Some(ctor())
        }
        let state = self.states[channel.id() as usize]
            .as_mut()
            .and_then(|a| (a as &mut dyn Any).downcast_mut::<GlobalState<C, P, SM>>())
            .unwrap();
        unsafe { &mut *(state as *mut GlobalState<C, P, SM>) }
    }
}

struct Indices {
    pub last_written_index: usize,
    pub read_index: usize,
}

impl Indices {
    fn new() -> Self {
        Self {
            last_written_index: 0,
            read_index: 0,
        }
    }
}

struct SequenceListBuilder;
type SequenceList = [Sequence; NUM_BUFFERS];

impl SequenceListBuilder {
    fn build() -> SequenceList {
        let mut sequences: [MaybeUninit<Sequence>; NUM_BUFFERS] =
            unsafe { MaybeUninit::uninit().assume_init() };
        for sequence in &mut sequences {
            sequence.write(Sequence::new());
        }
        let mut sequences =
            unsafe { core::mem::transmute::<_, [Sequence; NUM_BUFFERS]>(sequences) };

        // Safety: First index of Sequence.data is always initialized.
        unsafe {
            // Need to set a delay otherwise a lockup occurs when first changing frequency
            for sequence in &mut sequences {
                sequence.data[0].assume_init_mut().delay = 10;
            }
        }

        sequences
    }
}

pub struct InterruptHandler<C, P, SM>
where
    C: ChannelIndex,
    P: PIOExt,
    SM: StateMachineIndex,
{
    sequences: &'static Mutex<RefCell<SequenceList>>,
    loop_sequences: &'static Mutex<RefCell<SequenceList>>,
    indices: &'static Mutex<RefCell<Indices>>,
    tx_transfer: Option<TxTransfer<C, P, SM>>,
}

#[inline]
pub fn dma_interrupt<const NUM_PINS: usize>(global_state: &'static mut GlobalStates<NUM_PINS>) {
    for state in global_state.states.iter_mut().flatten() {
        state.try_next_dma_sequence();
    }
}

impl<C, P, SM> InterruptHandler<C, P, SM>
where
    C: ChannelIndex,
    P: PIOExt,
    SM: StateMachineIndex,
{
    pub fn try_next_dma_sequence(&mut self) {
        critical_section::with(|cs| {
            let (channel, tx_buf, tx) = {
                let mut tx_transfer = self.tx_transfer.take().unwrap();
                if tx_transfer.check_irq0() && tx_transfer.is_done() {
                    tx_transfer.wait()
                } else {
                    return;
                }
            };

            let mut indices = self.indices.borrow(cs).borrow_mut();
            if indices.last_written_index == indices.read_index {
                indices.read_index = indices.last_written_index;
                core::mem::swap(
                    tx_buf,
                    &mut self.sequences.borrow(cs).borrow_mut()[indices.read_index],
                );
            } else {
                core::mem::swap(
                    tx_buf,
                    &mut self.loop_sequences.borrow(cs).borrow_mut()[indices.read_index],
                );
            }

            self.tx_transfer = Some(SingleBufferingConfig::new(channel, tx_buf, tx).start());
        });
    }
}

pub struct PwmCluster<const NUM_PINS: usize, P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    sm: PioStateMachine<(P, SM)>,
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

pub struct PwmClusterBuilder<const NUM_PINS: usize> {
    pin_mask: u32,
    side_set_pin: u8,
    channel_to_pin_map: [u8; NUM_PINS],
}

impl<const NUM_PINS: usize> PwmClusterBuilder<NUM_PINS> {
    pub fn new() -> Self {
        Self {
            pin_mask: 0,
            side_set_pin: 0,
            channel_to_pin_map: [0; NUM_PINS],
        }
    }

    pub fn pin_base(mut self, base_pin: DynPin) -> Self {
        let base_pin = base_pin.id().num;
        for (i, pin_map) in self.channel_to_pin_map.iter_mut().enumerate() {
            let pin_id = base_pin + i as u8;
            self.pin_mask |= 1 << pin_id;
            *pin_map = pin_id;
        }
        self
    }

    pub fn pins(mut self, pins: &[DynPin; NUM_PINS]) -> Self {
        for (pin, pin_map) in pins.iter().zip(self.channel_to_pin_map.iter_mut()) {
            let pin_id = pin.id().num;
            self.pin_mask |= 1 << pin_id;
            *pin_map = pin_id;
        }

        self
    }

    pub fn side_pin(mut self, side_set_pin: &DynPin) -> Self {
        self.side_set_pin = side_set_pin.id().num;
        self
    }

    pub fn prep_global_state<C, P, SM>(
        global_state: &'static mut Option<GlobalState<C, P, SM>>,
    ) -> &'static mut GlobalState<C, P, SM>
    where
        C: ChannelIndex,
        P: PIOExt,
        SM: StateMachineIndex,
    {
        if global_state.is_none() {
            *global_state = Some(GlobalState {
                handler: None,
                indices: Mutex::new(RefCell::new(Indices::new())),
                sequences: Mutex::new(RefCell::new(SequenceListBuilder::build())),
                loop_sequences: Mutex::new(RefCell::new(SequenceListBuilder::build())),
            })
        }
        global_state.as_mut().unwrap()
    }

    pub fn build<C, P, SM>(
        self,
        servo_pins: [DynPin; NUM_PINS],
        side_pin: DynPin,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        mut dma: Channel<C>,
        global_state: &'static mut GlobalState<C, P, SM>,
    ) -> PwmCluster<NUM_PINS, P, SM>
    where
        C: ChannelIndex,
        P: PIOExt,
        SM: StateMachineIndex,
    {
        let pwm_program = pio_file!("./src/pwm.pio", select_program("debug_pwm_cluster"),);
        let program = pwm_program.program;
        let installed = pio.install(&program).unwrap();
        let (int, frac) = (0, 0); // as slow as possible, 0 interpreted as 65,536.
        let trailing_zeros: u8 = self.pin_mask.trailing_zeros() as u8;
        let num_pins = (32 - trailing_zeros).saturating_sub(self.pin_mask.leading_zeros() as u8);
        let (mut sm, _rx, tx) = hal::pio::PIOBuilder::from_program(installed)
            .out_pins(trailing_zeros, num_pins)
            .side_set_pin_base(side_pin.id().num)
            .clock_divisor_fixed_point(int, frac)
            .build(sm);
        sm.set_pindirs(
            servo_pins
                .into_iter()
                .chain(Some(side_pin).into_iter())
                .map(|pin| (pin.id().num, PinDir::Output)),
        );
        let sm = sm.start();

        // Transfer a single message via DMA.
        let sequence = Sequence::new();
        let tx_buf = singleton!(: Sequence = sequence).unwrap();
        // Enable DMA_IRQ_0 for this channel, but interrupt not triggered until unmasked
        // below.
        dma.listen_irq0();
        let tx_transfer = SingleBufferingConfig::new(dma, tx_buf, tx).start();
        global_state.handler = Some(InterruptHandler {
            sequences: &global_state.sequences,
            loop_sequences: &global_state.loop_sequences,
            indices: &global_state.indices,
            tx_transfer: Some(tx_transfer),
        });

        PwmCluster::new(
            sm,
            self.channel_to_pin_map,
            &global_state.indices,
            &global_state.sequences,
            &global_state.loop_sequences,
        )
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
    pub fn builder() -> PwmClusterBuilder<NUM_PINS> {
        PwmClusterBuilder::new()
    }

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
            sm: PioStateMachine::Running(sm),
            channel_to_pin_map,
            channels,
            sequences,
            loop_sequences,
            loading_zone: true,
            top: 0,
            indices,
            transitions,
            looping_transitions,
        }
    }

    pub fn calculate_pwm_factors(system_clock_hz: HertzU32, freq: f32) -> Option<(u32, u32)> {
        let source_hz = system_clock_hz.to_Hz() as u64 / PWM_CLUSTER_CYCLES;

        // Check the provided frequency is valid
        if (freq >= 0.01) && (freq <= (source_hz >> 1) as f32) {
            let mut div256_top = (((source_hz as u64) << 8) as f32 / freq) as u64;
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

            // Only return valid factors if the divisor is actually achievable
            if div256_top >= 256 && div256_top <= ((u8::MAX as u64) << 8) {
                Some((top as u32, div256_top as u32))
            } else {
                None
            }
        } else {
            None
        }
    }
    pub fn clock_divisor_fixed_point(&mut self, div: u16, frac: u8) {
        match &mut self.sm {
            PioStateMachine::Stopped(sm) => sm.clock_divisor_fixed_point(div, frac),
            PioStateMachine::Running(_) => {
                let sm = core::mem::replace(&mut self.sm, PioStateMachine::Transitioning);
                if let PioStateMachine::Running(sm) = sm {
                    let mut sm = sm.stop();
                    sm.clock_divisor_fixed_point(div, frac);
                    let _ = core::mem::replace(&mut self.sm, PioStateMachine::Stopped(sm));
                }
            }
            PioStateMachine::Transitioning => {
                unreachable!()
            }
        }
    }

    pub const fn channel_count(&self) -> u8 {
        NUM_PINS as u8
    }

    pub const fn channel_pair_count(&self) -> u8 {
        NUM_PINS as u8 / 2
    }

    pub fn channel_level(&self, channel: u8) -> Result<u32, PwmError> {
        self.channels
            .get(channel as usize)
            .map(|c| c.level)
            .ok_or(PwmError::MissingChannel)
    }

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

    pub fn top(&self) -> u32 {
        self.top
    }

    pub fn set_top(&mut self, top: u32, load_pwm: bool) {
        self.top = top.max(1); // Cannot have a wrap of zero.
        if load_pwm {
            self.load_pwm();
        }
    }

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
                    TransitionData {
                        channel: channel_idx as u8,
                        level: channel_wrap_end,
                        state: state.polarity,
                        dummy: false,
                    },
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
            let mut write_index = (indices.read_index + 1) % NUM_BUFFERS;
            if write_index == indices.last_written_index {
                write_index = (write_index + 1) % NUM_BUFFERS;
            }
            write_index
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

    fn sorted_insert(transitions: &mut [TransitionData], size: &mut usize, data: TransitionData) {
        let mut i = *size;
        while i > 0 && transitions[i - 1].level > data.level {
            transitions[i] = transitions[i - 1];
            i -= 1;
        }
        transitions[i] = data;
        *size += 1;
    }

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
                        &mut sequences[sequence_id],
                    )
                }
                TransitionType::Loop => {
                    loop_sequences = self.loop_sequences.borrow(cs).borrow_mut();
                    (
                        &self.looping_transitions[..transition_size],
                        &mut loop_sequences[sequence_id],
                    )
                }
            };

            let mut data_index = 0;
            let mut current_level = 0;

            // Populate the selected write sequence with pin states and delays.
            while data_index < transitions.len() {
                // Set the next level to be the top, initially.
                let mut next_level = self.top;
                let mut should_break = false;

                while {
                    // Is the level of this transition at the current level being checked?
                    let transition = &transitions[data_index];
                    if transition.level < current_level {
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
                        should_break = true;
                    }

                    // while
                    data_index < transitions.len()
                } {
                    if should_break {
                        break;
                    }
                }

                // Add the transition to the sequence.
                sequence.data[sequence.size as usize].write(Transition {
                    mask: *pin_states,
                    delay: (next_level - current_level) - 1,
                });
                sequence.size += 1;

                current_level = next_level;
            }
        });
    }
}

pub enum PwmError {
    MissingChannel,
}

enum TransitionType {
    Normal,
    Loop,
}

#[derive(Copy, Clone)]
struct ChannelState {
    level: u32,
    offset: u32,
    polarity: bool,
    overrun: u32,
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

#[derive(Clone)]
pub struct Sequence {
    size: u32,
    data: [MaybeUninit<Transition>; BUFFER_SIZE],
}

impl Sequence {
    pub fn new() -> Self {
        // Safety: Arrays are always init.
        let mut data: [MaybeUninit<Transition>; BUFFER_SIZE] =
            unsafe { MaybeUninit::uninit().assume_init() };
        data[0].write(Transition::new());
        Self { size: 1, data }
    }
}

impl Default for Sequence {
    fn default() -> Self {
        Self::new()
    }
}

impl ReadTarget for &mut Sequence {
    type ReceivedWord = u32;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        (&self.data as *const _ as u32, self.size * 2_u32)
    }

    fn rx_increment(&self) -> bool {
        true
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Transition {
    mask: u32,
    delay: u32,
}

impl Transition {
    fn new() -> Self {
        Self { mask: 0, delay: 0 }
    }
}

#[derive(Copy, Clone, Default)]
struct TransitionData {
    channel: u8,
    level: u32,
    state: bool,
    dummy: bool,
}

impl TransitionData {
    fn new(channel: u8, level: u32, state: bool) -> Self {
        Self {
            channel,
            level,
            state,
            dummy: false,
        }
    }

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
