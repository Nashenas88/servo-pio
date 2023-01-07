# servo-pio

Control servo motors using the RP2040's PIO peripheral.

## Example

See [work_in_progress.rs] for a complete example. Below you'll find the details relevant for this crate.

[work_in_progress.rs]: examples/work_in_progress.rs

```rust
fn main() -> ! {
    // ...
    let pins = pimoroni_servo2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Assign calibration data for each servo.
    let servo_pins: [_; NUM_SERVOS] = [
        ServoData {
            pin: pins.servo3.into_mode::<FunctionPio0>().into(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
        ServoData {
            pin: pins.servo4.into_mode::<FunctionPio0>().into(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
        ServoData {
            pin: pins.servo5.into_mode::<FunctionPio0>().into(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
        ServoData {
            pin: pins.servo6.into_mode::<FunctionPio0>().into(),
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
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    // Build the servo cluster
    let mut servo_cluster = match build_servo_cluster(
        &mut pio0,
        sm0,
        (dma.ch0, dma.ch1),
        servo_pins,
        #[cfg(feature = "debug_pio")]
        pins.scl.into_mode::<FunctionPio0>().into(),
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

fn build_servo_cluster<C1, C2, P, SM>(
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channels: (Channel<C1>, Channel<C2>),
    servo_data: [ServoData<AngularCalibration>; NUM_SERVOS],
    #[cfg(feature = "debug_pio")] side_set_pin: DynPin,
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
) -> Result<ServoCluster<NUM_SERVOS, P, SM, AngularCalibration>, BuildError>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt + FunctionConfig,
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
        NUM_SERVOS,
        NUM_CHANNELS,
    > = ServoCluster::<NUM_SERVOS, P, SM, AngularCalibration>::builder(
        pio,
        sm,
        dma_channels,
        unsafe { &mut GLOBALS },
    )
    .pins_and_calibration(servo_data)
    .map_err(BuildError::Gpio)?;
    #[cfg(feature = "debug_pio")]
    {
        builder = builder
            .side_set_pin(side_set_pin)
            .map_err(BuildError::Gpio)?;
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
```

## License

The contents of this repository are dual-licensed under the _MIT OR Apache
2.0_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See [`MIT`] or [`APACHE2.0`] for more
information on each specific licence.

Note that this code is a derivative of https://github.com/pimoroni/pimoroni-pico's
servo_cluster and pwm_cluster drivers. pwm.pio comes directly from pwm_cluster.pio in Pimoroni's
project. Their cose is licensed under the _MIT_ License. See [`Pimoroni.MIT`] for more information on
their license.

[`MIT`]: ./MIT
[`APACHE2.0`]: ./APACHE2.0
[`Pimoroni.MIT`]: ./Pimoroni.MIT
