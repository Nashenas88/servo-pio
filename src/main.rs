//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use embedded_hal::timer::CountDown;
use fugit::ExtU64;
use panic_halt as _;
use pimoroni_servo2040::hal::dma::{DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::pio::{PIOExt, SM0};
use pimoroni_servo2040::hal::{self, pac, Clock};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{AngularCalibration, Calibration};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::ServoCluster;

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
    clocks.system_clock.freq();

    let pins = pimoroni_servo2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);
    let servo_pins: [_; NUM_SERVOS] = [pins.servo1.into(), pins.servo2.into()];
    let side_set_pin = pins.scl.into();

    let mut servo_cluster = {
        let result = ServoCluster::<NUM_SERVOS, _, _, AngularCalibration>::builder(
            &mut pio,
            sm0,
            dma.ch0,
            unsafe { &mut GLOBALS },
        )
        .pins(servo_pins)
        .side_set_pin(side_set_pin)
        .pwm_frequency(50.0)
        .calibration(Calibration::<AngularCalibration>::new())
        .build(clocks.system_clock.freq(), unsafe { &mut STATE1 });
        match result {
            Ok(servo_cluster) => servo_cluster,
            // TODO handle error
            Err(_) =>
            {
                #[allow(clippy::empty_loop)]
                loop {}
            }
        }
    };

    // Unmask the DMA interrupt so the handler can start running.
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    // // Chain some buffers together.
    // let tx_buf2 = singleton!(: [u32; 1] = [2]).unwrap();
    // let tx_transfer = DoubleBufferingConfig::new((ch0, dma.ch1), tx_buf, tx).start();
    // let mut tx_transfer = tx_transfer.read_next(tx_buf2);
    // loop {
    //     // We simply immediately enqueue the buffers again.
    //     if tx_transfer.is_done() {
    //         let (tx_buf, next_tx_transfer) = tx_transfer.wait();
    //         tx_transfer = next_tx_transfer.read_next(tx_buf);
    //     }
    // }

    // let mut pwm: hal::pwm::Slice<_, _> = pwm_slices.pwm0;
    // pwm.set_ph_correct();
    // // pwm.set_div_int(38);
    // // pwm.set_div_frac(3);
    // pwm.set_div_int(20u8); // 50Hz
    // pwm.enable();

    // const MIN_PULSE: u16 = 1000;
    // const MID_PULSE: u16 = 1500;
    // const MAX_PULSE: u16 = 2000;
    // let movement_delay = 400.millis();

    // // Infinite loop, moving micro servo from one position to another.
    // // You may need to adjust the pulse width since several servos from
    // // different manufacturers respond differently.
    // loop {
    //     // move to 0°
    //     channel_a.set_duty(MID_PULSE);
    //     count_down.start(movement_delay);
    //     let _ = nb::block!(count_down.wait());

    //     // 0° to 90°
    //     channel_a.set_duty(MAX_PULSE);
    //     count_down.start(movement_delay);
    //     let _ = nb::block!(count_down.wait());

    //     // 90° to 0°
    //     channel_a.set_duty(MID_PULSE);
    //     count_down.start(movement_delay);
    //     let _ = nb::block!(count_down.wait());

    //     // 0° to -90°
    //     channel_a.set_duty(MIN_PULSE);
    //     count_down.start(movement_delay);
    //     let _ = nb::block!(count_down.wait());
    // }

    // Configure the Timer peripheral in count-down mode
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    const MIN_PULSE: f32 = 1000.0;
    const MID_PULSE: f32 = 1500.0;
    const MAX_PULSE: f32 = 2000.0;
    let movement_delay = 400.millis();

    loop {
        // move to 0°
        servo_cluster.set_pulse(0, MID_PULSE, true);
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        // 0° to 90°
        servo_cluster.set_pulse(0, MAX_PULSE, true);
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        // 90° to 0°
        servo_cluster.set_pulse(0, MID_PULSE, true);
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        // 0° to -90°
        servo_cluster.set_pulse(0, MIN_PULSE, true);
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());
    }
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|_| {
        // Safety: we're within a critical section, so nothing else will modify global_state.
        dma_interrupt(unsafe { &mut GLOBALS });
    });
}
