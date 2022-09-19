use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::ptr::{read_volatile, write_volatile};
use cortex_m::singleton;
use critical_section::Mutex;
use pimoroni_servo2040::hal;
use pimoroni_servo2040::hal::dma::{
    Channel, ChannelIndex, ReadTarget, SingleBuffering, SingleBufferingConfig, SingleChannel,
};
use pimoroni_servo2040::hal::gpio::bank0::BankPinId;
use pimoroni_servo2040::hal::gpio::{DynPin, FunctionPio0, Pin, PinId, PinMode, ValidPinMode};
use pimoroni_servo2040::hal::pio::{
    PIOExt, PinDir, StateMachineIndex, Tx, UninitStateMachine, PIO,
};
use pimoroni_servo2040::pac::interrupt;
use pio_proc::pio_file;

use crate::{GLOBALS, NUM_SERVOS};

type PinData = &'static mut Sequence;
type TxTransfer<C, P, SM> = SingleBuffering<Channel<C>, PinData, Tx<(P, SM)>>;

const NUM_PINS: usize = 30;
const NUM_BUFFERS: usize = 3;
const BUFFER_SIZE: usize = 64;

pub struct GlobalState<C, P, SM>
where
    C: ChannelIndex,
    P: PIOExt,
    SM: StateMachineIndex,
{
    clusters: [Option<PwmCluster>; 2],
    tx_transfer: Mutex<RefCell<Option<TxTransfer<C, P, SM>>>>,
}

pub struct PwmCluster {
    pin_mask: u32,
    channel_to_pin_map: [u8; NUM_PINS],
    channel_count: u8,
    channels: [Option<ChannelState>; NUM_PINS],
    sequences: [Sequence; NUM_BUFFERS],
    loop_sequences: [Sequence; NUM_BUFFERS],
    loading_zone: bool,
    wrap_level: u32,
    last_written_index: u32,
    read_index: u32,
    initialized: bool,
}

impl PwmCluster {
    pub fn new<C, P, SM>(
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        mut dma: Channel<C>,
        servo_pins: [DynPin; NUM_SERVOS as usize],
        side_pin: DynPin,
        global_state: &'static mut Option<GlobalState<C, P, SM>>,
    ) -> Result<Self, ()>
    where
        C: ChannelIndex,
        P: PIOExt,
        SM: StateMachineIndex,
    {
        let mut pin_mask = 0;
        let mut channels = [None; NUM_PINS];
        let mut channel_to_pin_map = [0; NUM_PINS];
        let channel_count = servo_pins.len() + 1;
        for ((pin, pin_map), channel) in servo_pins
            .iter()
            .chain(Some(&side_pin).into_iter())
            .zip(channel_to_pin_map.iter_mut())
            .zip(channels.iter_mut())
        {
            let pin_id = pin.id().num;
            pin_mask |= 1 << pin_id;
            *pin_map = pin_id;
            *channel = Some(ChannelState::new());
        }

        let mut sequences =
            unsafe { MaybeUninit::<[MaybeUninit<Sequence>; NUM_BUFFERS]>::uninit().assume_init() };
        let mut loop_sequences =
            unsafe { MaybeUninit::<[MaybeUninit<Sequence>; NUM_BUFFERS]>::uninit().assume_init() };
        for sequence in &mut sequences {
            sequence.write(Sequence::new());
        }
        for sequence in &mut loop_sequences {
            sequence.write(Sequence::new());
        }
        let mut sequences =
            unsafe { core::mem::transmute::<_, [Sequence; NUM_BUFFERS]>(sequences) };
        let mut loop_sequences =
            unsafe { core::mem::transmute::<_, [Sequence; NUM_BUFFERS]>(loop_sequences) };

        // Need to set a delay otherwise a lockup occurs when first changing frequency
        for sequence in &mut sequences {
            sequence.data[0].as_mut().unwrap().delay = 10;
        }
        for sequence in &mut loop_sequences {
            sequence.data[0].as_mut().unwrap().delay = 10;
        }

        let mut channels = [None; NUM_PINS];
        for channel in channels.iter_mut().take(channel_count) {
            *channel = Some(ChannelState::new());
        }

        let pwm_program = pio_file!("./src/pwm.pio", select_program("pwm_debug"),);
        let program = pwm_program.program;
        let installed = pio.install(&program).unwrap();
        let (int, frac) = (0, 0); // as slow as possible, 0 interpreted as 65,536.
        let (mut sm, _rx, tx) = hal::pio::PIOBuilder::from_program(installed)
            .out_pins(servo_pins[0].id().num, NUM_SERVOS as u8)
            .side_set_pin_base(side_pin.id().num)
            .clock_divisor_fixed_point(int, frac)
            .build(sm);
        sm.set_pindirs(
            servo_pins
                .into_iter()
                .chain(Some(side_pin).into_iter())
                .map(|pin| (pin.id().num, PinDir::Output)),
        );
        sm.start();

        if global_state.is_none() {
            // Transfer a single message via DMA.
            let sequence = Sequence::new();
            let tx_buf = singleton!(: Sequence = sequence).unwrap();
            // Enable DMA_IRQ_0 for this channel, but interrupt not triggered until unmasked below.
            dma.listen_irq0();
            let tx_transfer = SingleBufferingConfig::new(dma, tx_buf, tx).start();

            *global_state = Some(GlobalState {
                clusters: [None, None],
                tx_transfer: Mutex::new(RefCell::new(Some(tx_transfer))),
            });
        }

        Ok(Self {
            pin_mask,
            channel_to_pin_map,
            channel_count: channel_count as u8,
            channels,
            sequences,
            loop_sequences,
            loading_zone: true,
            wrap_level: 0,
            read_index: 0,
            last_written_index: 0,
            initialized: false,
        })
    }

    pub fn next_dma_sequence<C, P, SM>(
        &'static mut self,
        ch0: Channel<C>,
        tx: Tx<(P, SM)>,
    ) -> TxTransfer<C, P, SM>
    where
        C: ChannelIndex,
        P: PIOExt,
        SM: StateMachineIndex,
    {
        let sequence = unsafe {
            if read_volatile(&self.last_written_index as *const _)
                != read_volatile(&self.read_index as *const _)
            {
                write_volatile(
                    &mut self.read_index as *const _ as *mut _,
                    &self.last_written_index as *const _,
                );
                &mut self.sequences[read_volatile(&self.read_index as *const _) as usize]
            } else {
                &mut self.loop_sequences[read_volatile(&self.read_index as *const _) as usize]
            }
        };

        SingleBufferingConfig::new(ch0, sequence, tx).start()
    }

    pub fn check_irq0<C, P, SM>(&mut self, tx_transfer: &mut TxTransfer<C, P, SM>) -> bool
    where
        C: ChannelIndex,
        P: PIOExt,
        SM: StateMachineIndex,
    {
        tx_transfer.check_irq0()
    }
}

struct PioPin(DynPin);

impl PioPin {
    fn new<I, M>(p: Pin<I, M>) -> Self
    where
        I: PinId + BankPinId,
        M: PinMode + ValidPinMode<I>,
    {
        let p: Pin<I, FunctionPio0> = p.into_mode();
        Self(p.into())
    }
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|cs| {
        // Unwrap ok because interrupt only enabled after variable initialized.
        // Safety: we're within a critical section, so nothing else will modify global_state.
        let global_state = unsafe { GLOBALS.as_mut().unwrap() };
        let mut tx_transfer = global_state.tx_transfer.borrow_ref_mut(cs);
        let tx_transfer = tx_transfer.as_mut().unwrap();
        for cluster in global_state.clusters.iter_mut().flatten() {
            if cluster.check_irq0(tx_transfer) {
                break;
            }
        }

        if tx_transfer.check_irq0() && tx_transfer.is_done() {
            let tx_transfer = global_state.tx_transfer.borrow_ref_mut(cs).take().unwrap();
            let (ch0, _, tx) = tx_transfer.wait();
            if let Some(cluster) = global_state.clusters[ch0.id() as usize].as_mut() {
                *global_state.tx_transfer.borrow_ref_mut(cs) =
                    Some(cluster.next_dma_sequence(ch0, tx));
            }

            // global_state.tx_transfer = Some(SingleBufferingConfig::new(ch0, tx_buf, tx).start());
        }
    })
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
    data: [Option<Transition>; BUFFER_SIZE],
}

impl Sequence {
    pub fn new() -> Self {
        let mut data = [None; BUFFER_SIZE];
        data[0] = Some(Transition::new());
        Self { size: 1, data }
    }
}

impl ReadTarget for &mut Sequence {
    type ReceivedWord = u32;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        (&self.data as *const _ as u32, self.size * 2 as u32)
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
