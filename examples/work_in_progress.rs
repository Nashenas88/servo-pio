//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::iter::once;
use defmt::Format;
use defmt_rtt as _;
use embedded_hal::timer::CountDown;
use fugit::ExtU64;
#[cfg(feature = "debug_pio")]
use hal::gpio::{DynPinId, Pin};
use hal::gpio::{FunctionPio1, PullNone};
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::FunctionPio0;
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::{self, pac, Clock};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{AngularCalibration, Calibration};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{
    ServoCluster, ServoClusterBuilder, ServoClusterBuilderError, ServoData,
};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812Direct;

const LED_BRIGHTNESS: u8 = 16;
const NUM_SERVOS: usize = 4;
const NUM_CHANNELS: usize = 12;
static mut STATE1: Option<GlobalState<CH0, CH1, PIO0, SM0>> = {
    const NONE_HACK: Option<GlobalState<CH0, CH1, PIO0, SM0>> = None;
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
        ServoData {
            pin: pins
                .servo3
                .reconfigure::<FunctionPio0, PullNone>()
                .into_dyn_pin(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
        ServoData {
            pin: pins
                .servo4
                .reconfigure::<FunctionPio0, PullNone>()
                .into_dyn_pin(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
        ServoData {
            pin: pins
                .servo5
                .reconfigure::<FunctionPio0, PullNone>()
                .into_dyn_pin(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
        ServoData {
            pin: pins
                .servo6
                .reconfigure::<FunctionPio0, PullNone>()
                .into_dyn_pin(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
    ];

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    // Use a different pio for the leds because they run at a different
    // clock speed.
    let (mut pio1, sm10, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    // Configure the Timer peripheral in count-down mode.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut count_down = timer.count_down();

    let mut ws = Ws2812Direct::new(
        pins.led_data.reconfigure::<FunctionPio1, PullNone>(),
        &mut pio1,
        sm10,
        clocks.peripheral_clock.freq(),
    );

    // Build the servo cluster
    let mut servo_cluster = match build_servo_cluster(
        &mut pio0,
        sm0,
        (dma.ch0, dma.ch1),
        servo_pins,
        #[cfg(feature = "debug_pio")]
        pins.scl
            .reconfigure::<FunctionPio0, PullNone>()
            .into_dyn_pin(),
        clocks.system_clock,
        unsafe { &mut STATE1 },
    ) {
        Ok(cluster) => cluster,
        Err(e) => {
            defmt::error!("Failed to build servo cluster: {:?}", e);
            let _ = ws.write(brightness(
                once(RGB8 { r: 255, g: 0, b: 0 }),
                LED_BRIGHTNESS,
            ));
            #[allow(clippy::empty_loop)]
            loop {}
        }
    };

    // Unmask the DMA interrupt so the handler can start running. This can only
    // be done after the servo cluster has been built.
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    const MIN_PULSE: f32 = 1500.0;
    const MID_PULSE: f32 = 2000.0;
    const MAX_PULSE: f32 = 2500.0;
    let movement_delay = 20.millis();

    // We need to use the indices provided by the cluster because the servo pin
    // numbers do not line up with the indices in the clusters and PIO.
    let [servo1, servo2, servo3, servo4] = servo_cluster.servos();

    servo_cluster.set_pulse(servo1, MAX_PULSE, false);
    servo_cluster.set_pulse(servo2, MID_PULSE, false);
    servo_cluster.set_pulse(servo3, MIN_PULSE, false);
    servo_cluster.set_value(servo4, 35.0, true);
    count_down.start(movement_delay * 5);

    let mut velocity1: f32 = 10.0;
    let mut velocity2: f32 = 15.0;
    let mut velocity3: f32 = 25.0;
    let mut velocity4: f32 = 50.0;
    #[allow(clippy::empty_loop)]
    loop {
        let _ = ws.write(brightness(
            Some(RGB8 { r: 0, g: 255, b: 0 }).into_iter(),
            LED_BRIGHTNESS,
        ));

        for (servo, velocity) in [
            (servo1, &mut velocity1),
            (servo2, &mut velocity2),
            (servo3, &mut velocity3),
            (servo4, &mut velocity4),
        ] {
            let mut pulse = servo_cluster.pulse(servo).unwrap();
            pulse += *velocity;
            if !(MIN_PULSE..=MAX_PULSE).contains(&pulse) {
                *velocity *= -1.0;
                pulse = pulse.clamp(MIN_PULSE, MAX_PULSE);
            }
            // Assign pulses to each servo, but passing `false` to prevent
            // immediate usage of the pulse.
            servo_cluster.set_pulse(servo, pulse, false);
        }
        // Load to trigger a simultaneous of the values to the servos. Phasing
        // of the PWM signals will be used to prevent voltage spikes.
        servo_cluster.load();
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());
    }
}

#[derive(Format)]
enum BuildError {
    Build(ServoClusterBuilderError),
}

fn build_servo_cluster<C1, C2, P, SM>(
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channels: (Channel<C1>, Channel<C2>),
    servo_data: [ServoData<AngularCalibration, FunctionPio0>; NUM_SERVOS],
    #[cfg(feature = "debug_pio")] side_set_pin: Pin<DynPinId, FunctionPio0, PullNone>,
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
) -> Result<ServoCluster<NUM_SERVOS, P, SM, AngularCalibration>, BuildError>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt<PinFunction = FunctionPio0>,
    SM: StateMachineIndex,
{
    #[allow(unused_mut)]
    let mut builder: ServoClusterBuilder<
        '_,
        AngularCalibration,
        C1,
        C2,
        P,
        SM,
        FunctionPio0,
        NUM_SERVOS,
        NUM_CHANNELS,
    > = ServoCluster::<NUM_SERVOS, P, SM, AngularCalibration>::builder(
        pio,
        sm,
        dma_channels,
        unsafe { &mut GLOBALS },
    )
    .pins_and_calibration(servo_data);
    #[cfg(feature = "debug_pio")]
    {
        builder = builder.side_set_pin(side_set_pin);
    }
    builder
        .pwm_frequency(50.0)
        .build(&system_clock, state)
        .map_err(BuildError::Build)
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|_| {
        // Safety: we're within a critical section, so nothing else will modify global_state.
        dma_interrupt(unsafe { &mut GLOBALS });
    });
}
