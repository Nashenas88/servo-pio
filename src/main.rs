//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use panic_halt as _;
use pimoroni_servo2040::hal::dma::{DMAExt, CH0};
use pimoroni_servo2040::hal::pio::{PIOExt, SM0};
use pimoroni_servo2040::hal::{self, pac};
use pimoroni_servo2040::pac::PIO0;
use pwm_cluster::{GlobalState, PwmCluster};

mod pwm_cluster;
mod servo_cluster;

/// Static to prevent pins used in PIO from being used elsewhere.

/// Number of servos to control on board.
const NUM_SERVOS: u8 = 2;
static mut GLOBALS: Option<GlobalState<CH0, PIO0, SM0>> = None;

#[pimoroni_servo2040::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let sio = hal::Sio::new(pac.SIO);

    let _clocks = hal::clocks::init_clocks_and_plls(
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

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);
    let _pwm_cluster = PwmCluster::new(
        &mut pio,
        sm0,
        dma.ch0,
        [pins.servo1.into(), pins.servo2.into()],
        pins.scl.into(),
        // Safety: No interrupts are able to run yet, so only this function is modifying GLOBALS.
        unsafe { &mut GLOBALS },
    )
    .unwrap_or_else(|_| loop {});

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

    #[allow(clippy::empty_loop)]
    loop {}
}

// End of file
