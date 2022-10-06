//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt::Format;
use defmt_rtt as _;
use embedded_hal::timer::CountDown;
use fugit::ExtU64;
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::{DynPin, Error as GpioError, FunctionConfig, FunctionPio0};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::{self, pac, Clock};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{AngularCalibration, Calibration};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{ServoCluster, ServoClusterBuilderError};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812Direct;

const LED_BRIGHTNESS: u8 = 16;
const NUM_SERVOS: usize = 2;
const NUM_CHANNELS: usize = 12;
static mut STATE1: Option<GlobalState<CH0, PIO0, SM0>> = {
    const NONE_HACK: Option<GlobalState<CH0, PIO0, SM0>> = None;
    NONE_HACK
};
#[allow(dead_code)]
static mut STATE2: Option<GlobalState<CH1, PIO0, SM0>> = {
    const NONE_HACK: Option<GlobalState<CH1, PIO0, SM0>> = None;
    NONE_HACK
};
static mut GLOBALS: GlobalStates<NUM_CHANNELS> = {
    const NONE_HACK: Option<&'static mut dyn Handler> = None;
    GlobalStates {
        states: [NONE_HACK; NUM_CHANNELS],
    }
};

#[pimoroni_servo2040::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let sio = hal::Sio::new(pac.SIO);

    let clocks = hal::clocks::init_clocks_and_plls(
        pimoroni_servo2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = pimoroni_servo2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let servo_pins: [_; NUM_SERVOS] = [
        pins.servo3.into_mode::<FunctionPio0>().into(),
        pins.servo4.into_mode::<FunctionPio0>().into(),
    ];
    let side_set_pin = pins.scl.into_mode::<FunctionPio0>().into();

    let (pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let (mut pio1, sm10, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    // Configure the Timer peripheral in count-down mode
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    let mut ws = Ws2812Direct::new(
        pins.led_data.into_mode(),
        &mut pio1,
        sm10,
        clocks.peripheral_clock.freq(),
    );

    let mut servo_cluster = match build_servo_cluster(
        pio0,
        sm0,
        dma.ch0,
        servo_pins,
        side_set_pin,
        clocks.system_clock,
        unsafe { &mut STATE1 },
    ) {
        Ok(cluster) => cluster,
        Err(e) => {
            defmt::error!("Failed to build servo cluster: {:?}", e);
            #[allow(clippy::empty_loop)]
            loop {}
        }
    };

    // Unmask the DMA interrupt so the handler can start running.
    defmt::trace!("Unmasking dma");
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    const MIN_PULSE: f32 = 1000.0;
    const MID_PULSE: f32 = 1500.0;
    const MAX_PULSE: f32 = 2000.0;
    let movement_delay = 1000.millis();
    servo_cluster.set_pulse(0, MAX_PULSE, true);
    count_down.start(movement_delay * 5);

    #[allow(clippy::empty_loop)]
    loop {
        // defmt::trace!("to 0");
        // let _ = ws.write(brightness(
        //     Some(RGB8 { r: 0, g: 255, b: 0 }).into_iter(),
        //     LED_BRIGHTNESS,
        // ));
        // // move to 0°
        // servo_cluster.set_pulse(0, MID_PULSE, true);
        // count_down.start(movement_delay);
        // let _ = nb::block!(count_down.wait());

        // defmt::trace!("to 90");
        // // 0° to 90°
        // let _ = ws.write(brightness(
        //     Some(RGB8 { r: 0, g: 0, b: 0 }).into_iter(),
        //     LED_BRIGHTNESS,
        // ));
        // servo_cluster.set_pulse(0, MAX_PULSE, true);
        // count_down.start(movement_delay);
        // let _ = nb::block!(count_down.wait());

        // defmt::trace!("to 0");
        // // 90° to 0°
        // let _ = ws.write(brightness(
        //     Some(RGB8 { r: 0, g: 0, b: 255 }).into_iter(),
        //     LED_BRIGHTNESS,
        // ));
        // servo_cluster.set_pulse(0, MID_PULSE, true);
        // count_down.start(movement_delay);
        // let _ = nb::block!(count_down.wait());

        // defmt::trace!("to -90");
        // // 0° to -90°
        // let _ = ws.write(brightness(
        //     Some(RGB8 { r: 0, g: 0, b: 0 }).into_iter(),
        //     LED_BRIGHTNESS,
        // ));
        // servo_cluster.set_pulse(0, MIN_PULSE, true);
        // count_down.start(movement_delay);
        // let _ = nb::block!(count_down.wait());
    }
}

#[derive(Format)]
enum BuildError {
    Gpio(GpioError),
    Build(ServoClusterBuilderError),
}

fn build_servo_cluster<C, P, SM>(
    mut pio: PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channel: Channel<C>,
    servo_pins: [DynPin; NUM_SERVOS],
    side_set_pin: DynPin,
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C, P, SM>>,
) -> Result<ServoCluster<NUM_SERVOS, P, SM, AngularCalibration>, BuildError>
where
    C: ChannelIndex,
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    ServoCluster::builder(&mut pio, sm, dma_channel, unsafe { &mut GLOBALS })
        .pins(servo_pins)
        .map_err(BuildError::Gpio)?
        .side_set_pin(side_set_pin)
        .map_err(BuildError::Gpio)?
        .pwm_frequency(50.0)
        .calibration(Calibration::new())
        .build(&system_clock, state)
        .map_err(BuildError::Build)
}

#[interrupt]
fn DMA_IRQ_0() {
    defmt::warn!("DMA_IRQ_0");
    critical_section::with(|_| {
        // Safety: we're within a critical section, so nothing else will modify global_state.
        dma_interrupt(unsafe { &mut GLOBALS });
    });
}
