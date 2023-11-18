#![no_std]
#![no_main]

use defmt_rtt as _;
use hal::gpio::{Function, FunctionPio0, Pin, PinId, PullNone, PullType};
use hal::pio::{PIOExt, PinDir};
use hal::Clock;
use panic_probe as _;
use pimoroni_servo2040::hal;
use pimoroni_servo2040::pac;
use pio_proc::pio_file;

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
    let servo7 = pins.servo7.reconfigure::<FunctionPio0, PullNone>();
    let servo4 = pins.servo4.reconfigure::<FunctionPio0, PullNone>();
    let scl = pins.scl.reconfigure::<FunctionPio0, PullNone>();

    let (mut pio, sm, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let pwm_program = pio_file!("./src/pwm.pio", select_program("debug_pwm_cluster"));
    let program = pwm_program.program;
    let installed = pio.install(&program).unwrap();
    const DESIRED_CLOCK_HZ: u32 = 500_000;
    let sys_hz = clocks.system_clock.freq().to_Hz();
    let (int, frac) = (
        sys_hz / DESIRED_CLOCK_HZ,
        (sys_hz as u64 * 256 / DESIRED_CLOCK_HZ as u64) as u8,
    );
    let (mut sm, _, mut tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .out_pins(0, id(&scl))
        .side_set_pin_base(id(&scl))
        .clock_divisor_fixed_point(int as u16, frac)
        .build(sm);
    sm.set_pindirs(
        [(id(&scl), PinDir::Output)].into_iter().chain(
            [id(&servo4), id(&servo7)]
                .into_iter()
                .map(|pin| (pin, PinDir::Output)),
        ),
    );

    let _sm = sm.start();
    let transitions = [
        Transition {
            mask: 1 << id(&servo7),
            delay: 4293,
        },
        Transition {
            mask: 0,
            delay: 26_955,
        },
        Transition {
            mask: 0,
            delay: 31_249,
        },
    ];
    let mut idx = 0;
    let mut is_mask = true;
    loop {
        if !tx.is_full() {
            let value = if is_mask {
                is_mask = !is_mask;
                transitions[idx].mask
            } else {
                let ret = transitions[idx].delay;
                is_mask = true;
                idx = (idx + 1) % transitions.len();
                ret
            };
            tx.write(value);
        } else {
            cortex_m::asm::delay(100);
        }
    }
}

struct Transition {
    mask: u32,
    delay: u32,
}

fn id<I, F, P>(pin: &Pin<I, F, P>) -> u8
where
    I: PinId,
    F: Function,
    P: PullType,
{
    pin.id().num
}
