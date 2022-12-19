use crate::calibration::{Calibration, CalibrationData, NoCustom, Point};
use crate::pwm_cluster::{GlobalState, GlobalStates, PwmCluster, PwmClusterBuilder};
use crate::servo_state::{self, ServoState, DEFAULT_FREQUENCY};
use crate::{initialize_array_by_index, initialize_array_from, initialize_arrays_from};
use defmt::Format;
use fugit::HertzU32;
use rp2040_hal::clocks::SystemClock;
use rp2040_hal::dma::{Channel, ChannelIndex};
use rp2040_hal::gpio::{DynPin, Error as GpioError, Function, FunctionConfig, PinMode};
use rp2040_hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO};
use rp2040_hal::Clock;

/// Use to index the servos in [ServoCluster methods].
#[derive(Copy, Clone)]
pub struct ServoIdx(u8);

/// A type to manage a cluster of servos all run from the same PIO state machine.
pub struct ServoCluster<const NUM_SERVOS: usize, P, SM, Cal = NoCustom>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    /// The [PwmCluster] used to control the pwm signals for the servos.
    pwms: PwmCluster<NUM_SERVOS, P, SM>,
    /// The pwm period shared by all servos in this cluster.
    pwm_period: u32,
    /// The pwm frequency shared by all servos in this cluster.
    pwm_frequency: f32,
    /// The individual servo states.
    states: [ServoState<Cal>; NUM_SERVOS],
    /// The phases for each servo.
    ///
    /// The phases are needed to prevent voltage spikes when attempting to move many servos at once.
    /// A phase shift (or offset) is applied so there is not a sudden demand for current in the
    /// system.
    servo_phases: [f32; NUM_SERVOS],
}

/// A builder for [ServoCluster].
pub struct ServoClusterBuilder<
    'a,
    Cal,
    C1,
    C2,
    P,
    SM,
    const NUM_SERVOS: usize,
    const NUM_CHANNELS: usize,
> where
    C1: ChannelIndex + 'static,
    C2: ChannelIndex + 'static,
    P: PIOExt + 'static,
    SM: StateMachineIndex + 'static,
{
    /// The [PIO] instance to use.
    pio: &'a mut PIO<P>,
    /// The specific [UninitStateMachine] to run this cluster on.
    sm: UninitStateMachine<(P, SM)>,
    /// The DMA [Channel]s to manage the transitions.
    dma_channels: (Channel<C1>, Channel<C2>),
    /// The [GlobalStates] to manage the interrupt handler for this cluster.
    global_states: &'static mut GlobalStates<NUM_CHANNELS>,
    /// The output pins this cluster will use to communicate with the servos.
    pins: Option<[DynPin; NUM_SERVOS]>,
    /// The side set pin used for debugging the pio program.
    #[cfg(feature = "debug_pio")]
    side_set_pin: Option<DynPin>,
    /// The pwm frequency the servos will share.
    pwm_frequency: Option<f32>,
    /// The calibration for each servo.
    calibrations: Option<[Calibration<Cal>; NUM_SERVOS]>,
    /// Whether or not to enable a phase shift for each servo. The default is true.
    auto_phase: Option<bool>,
}

pub struct ServoData<C> {
    pub pin: DynPin,
    pub calibration: Calibration<C>,
}

impl<'a, Cal, C1, C2, P, SM, const NUM_SERVOS: usize, const NUM_CHANNELS: usize>
    ServoClusterBuilder<'a, Cal, C1, C2, P, SM, NUM_SERVOS, NUM_CHANNELS>
where
    Cal: CalibrationData + Default + Clone,
    for<'i> <Cal as CalibrationData>::Iterator<'i>: Iterator<Item = (Point, Point)>,
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt + FunctionConfig,
    Function<P>: PinMode,
    SM: StateMachineIndex,
{
    /// Set the calibration for each servos.
    pub fn calibrations(mut self, calibrations: [Calibration<Cal>; NUM_SERVOS]) -> Self {
        self.calibrations = Some(calibrations);
        self
    }

    /// Configure whether or not to enable phase shifts for the servos.
    pub fn auto_phase(mut self, auto_phase: bool) -> Self {
        self.auto_phase = Some(auto_phase);
        self
    }

    // pub fn pin_mask(mut self, pin_mask: u32) -> Self {
    //     if true {
    //         // self.pins is [DynPin;]
    //         todo!()
    //     }
    //     // Safety: MaybeUninit<[T; N]> is always initialized.
    //     let mut pins: [MaybeUninit<u8>; NUM_SERVOS] =
    //         unsafe { MaybeUninit::uninit().assume_init() };
    //     for (pin, bit) in pins.iter_mut().zip(0..32) {
    //         pin.write(if pin_mask & (1 << bit) == 1 << bit {
    //             bit
    //         } else {
    //             0
    //         });
    //     }

    //     // Safety: all entries initialized above.
    //     self.pins = Some(unsafe {
    //         (*(&MaybeUninit::new(pins) as *const _ as *const MaybeUninit<_>)).assume_init_read()
    //     });

    //     self
    // }

    // pub fn pin_base(mut self, pin_base: u8) -> Self {
    //     if true {
    //         // self.pins is [DynPin;]
    //         todo!()
    //     }
    //     // Safety: MaybeUninit<[T; N]> is always initialized.
    //     let mut pins: [MaybeUninit<u8>; NUM_SERVOS] =
    //         unsafe { MaybeUninit::uninit().assume_init() };
    //     for (pin, id) in pins.iter_mut().zip(pin_base..(pin_base + NUM_SERVOS as u8)) {
    //         pin.write(id);
    //     }

    //     // Safety: all entries initialized above.
    //     self.pins = Some(unsafe {
    //         (*(&MaybeUninit::new(pins) as *const _ as *const MaybeUninit<_>)).assume_init_read()
    //     });

    //     self
    // }

    /// Set the output pins to correspond to the servos. Note that the order they are passed in here
    /// will map how the servos are accessed in [ServoCluster] when specifying the servo index.
    pub fn pins_and_calibration(
        mut self,
        pin_data: [ServoData<Cal>; NUM_SERVOS],
    ) -> Result<Self, GpioError> {
        for data in &pin_data {
            if data.pin.mode() != <Function<P> as PinMode>::DYN {
                return Err(GpioError::InvalidPinMode);
            }
        }
        let (dyn_pins, calibrations) =
            initialize_arrays_from(pin_data, |data| (data.pin, data.calibration));
        self.pins = Some(dyn_pins);
        self.calibrations = Some(calibrations);

        Ok(self)
    }

    /// Set the sideset pin to use for debugging the PIO program.
    #[cfg(feature = "debug_pio")]
    pub fn side_set_pin(mut self, side_set_pin: DynPin) -> Result<Self, GpioError> {
        if side_set_pin.mode() == <Function<P> as PinMode>::DYN {
            self.side_set_pin = Some(side_set_pin);
            Ok(self)
        } else {
            Err(GpioError::InvalidPinMode)
        }
    }

    /// Set the frequency for the pwm signal for the cluster.
    pub fn pwm_frequency(mut self, pwm_frequency: f32) -> Self {
        self.pwm_frequency = Some(pwm_frequency);
        self
    }

    /// Build the [ServoCluster]. Note that the global state passed in here should not have been
    /// used to initialize another [ServoCluster]. If so, this function will return an error.
    pub fn build(
        mut self,
        system_clock: &SystemClock,
        maybe_global_state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
    ) -> Result<ServoCluster<NUM_SERVOS, P, SM, Cal>, ServoClusterBuilderError> {
        let pins = self.pins.ok_or(ServoClusterBuilderError::MissingPins)?;
        let calibrations = self
            .calibrations
            .ok_or(ServoClusterBuilderError::MissingCalibrations)?;
        let (states, servo_phases) = ServoCluster::<NUM_SERVOS, P, SM, _>::create_servo_states(
            calibrations,
            self.auto_phase.unwrap_or(true),
        );
        let global_state = self.global_states.get_mut(&mut self.dma_channels, move || {
            PwmClusterBuilder::<NUM_SERVOS>::prep_global_state(maybe_global_state)
        });
        let mut pwms = {
            {
                unsafe {
                    #[allow(unused_mut)]
                    let mut cluster = PwmCluster::<NUM_SERVOS, P, SM>::builder().pins(&pins);
                    #[cfg(feature = "debug_pio")]
                    {
                        let side_set_pin = self
                            .side_set_pin
                            .ok_or(ServoClusterBuilderError::MissingSideSet)?;
                        cluster = cluster.side_pin(&side_set_pin);
                    }
                    cluster.build(
                        pins,
                        self.pio,
                        self.sm,
                        self.dma_channels,
                        system_clock,
                        global_state.ok_or(ServoClusterBuilderError::MismatchingGlobalState)?,
                    )
                }
            }
        };

        // Calculate a suitable pwm top period for this frequency
        let pwm_frequency = self.pwm_frequency.unwrap_or(DEFAULT_FREQUENCY);
        let pwm_period = if let Some((pwm_period, div256)) =
            PwmCluster::<NUM_SERVOS, P, SM>::calculate_pwm_factors(
                system_clock.freq(),
                pwm_frequency,
            ) {
            // Update the pwm before setting the new top
            for servo in 0..NUM_SERVOS as u8 {
                let _ = pwms.set_channel_level(servo, 0, false);
                let _ = pwms.set_channel_offset(
                    servo,
                    (servo_phases[servo as usize] * pwm_period as f32) as u32,
                    false,
                );
            }

            // Set the new top (should be 1 less than the period to get full 0 to 100%)
            pwms.set_top(pwm_period, true); // NOTE Minus 1 not needed here. Maybe should change Wrap behaviour so it is needed, for consistency with hardware pwm?

            // Apply the new divider
            // This is done after loading new PWM values to avoid a lockup condition
            let div: u16 = (div256 >> 8) as u16;
            let modulus: u8 = (div256 % 256) as u8;
            pwms.clock_divisor_fixed_point(div, modulus);
            pwm_period
        } else {
            0
        };

        Ok(ServoCluster {
            pwms,
            pwm_period,
            pwm_frequency,
            states,
            servo_phases,
        })
    }
}

/// Possible errors when attempting to build a [ServoCluster].
#[derive(Format)]
pub enum ServoClusterBuilderError {
    /// The GlobalState passed in does not match one of the DMA channels, PIO
    /// instance or PIO State Machine number.
    MismatchingGlobalState,
    /// The output pins are missing.
    MissingPins,
    MissingCalibrations,
    /// The side set pin is missing.
    #[cfg(feature = "debug_pio")]
    MissingSideSet,
}

impl<'a, const NUM_SERVOS: usize, P, SM, Cal> ServoCluster<NUM_SERVOS, P, SM, Cal>
where
    Cal: Default + CalibrationData + Clone,
    for<'i> <Cal as CalibrationData>::Iterator<'i>: Iterator<Item = (Point, Point)>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    /// Get a builder to help construct a `ServoCluster`.
    pub fn builder<C1, C2, const NUM_CHANNELS: usize>(
        pio: &'a mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        dma_channels: (Channel<C1>, Channel<C2>),
        global_states: &'static mut GlobalStates<NUM_CHANNELS>,
    ) -> ServoClusterBuilder<'a, Cal, C1, C2, P, SM, NUM_SERVOS, NUM_CHANNELS>
    where
        C1: ChannelIndex + 'static,
        C2: ChannelIndex + 'static,
        P: PIOExt + 'static,
    {
        ServoClusterBuilder {
            pio,
            sm,
            dma_channels,
            global_states,
            pins: None,
            #[cfg(feature = "debug_pio")]
            side_set_pin: None,
            pwm_frequency: None,
            calibrations: None,
            auto_phase: None,
        }
    }

    /// Create the servo states.
    fn create_servo_states(
        calibrations: [Calibration<Cal>; NUM_SERVOS],
        auto_phase: bool,
    ) -> ([ServoState<Cal>; NUM_SERVOS], [f32; NUM_SERVOS]) {
        (
            initialize_array_from::<Calibration<Cal>, ServoState<Cal>, NUM_SERVOS>(
                calibrations,
                move |calibration| ServoState::with_calibration(calibration),
            ),
            initialize_array_by_index::<f32, NUM_SERVOS>(|i| {
                if auto_phase {
                    i as f32 / NUM_SERVOS as f32
                } else {
                    0.0
                }
            }),
        )
    }

    pub fn servos(&self) -> [ServoIdx; NUM_SERVOS] {
        initialize_array_by_index(|i| ServoIdx(i as u8))
    }

    /// Return whether or not this `ServoCluster` is enabled.
    pub fn enabled(&self, servo: ServoIdx) -> bool {
        self.states[servo.0 as usize].enabled()
    }

    /// Control whether the `ServoCluster` is enabled or not. If `load` is true, immeditely update
    /// the servo pwm signal.
    pub fn set_enabled(&mut self, servo: ServoIdx, enable: bool, load: bool) {
        let state = &mut self.states[servo.0 as usize];
        if enable {
            let new_pulse = state.enable_with_return();
            self.apply_pulse(servo, new_pulse, load);
        } else {
            state.disable();
        }
    }

    /// Get the pulse for a particular servo.
    pub fn pulse(&self, servo: ServoIdx) -> Option<f32> {
        self.states[servo.0 as usize].pulse()
    }

    /// Set the pulse for a particular servo. If `load` is true, immeditely update the servo pwm
    /// signal.
    pub fn set_pulse(&mut self, servo: ServoIdx, pulse: f32, load: bool) {
        let new_pulse = self.states[servo.0 as usize].set_pulse_with_return(pulse);
        if let Some(new_pulse) = new_pulse {
            self.apply_pulse(servo, new_pulse, load);
        }
    }

    /// Get the value for a particular servo.
    pub fn value(&self, servo: ServoIdx) -> f32 {
        self.states[servo.0 as usize].value()
    }

    /// Set the value for a particular servo. If `load` is true, immeditely update the servo pwm
    /// signal.
    pub fn set_value(&mut self, servo: ServoIdx, value: f32, load: bool) {
        let new_pulse = self.states[servo.0 as usize].set_value_with_return(value);
        self.apply_pulse(servo, new_pulse, load);
    }

    /// Get the phase for a particular servo.
    pub fn phase(&self, servo: ServoIdx) -> f32 {
        self.servo_phases[servo.0 as usize]
    }

    /// Set the phase for a particular servo. If `load` is true, immeditely update the servo pwm
    /// signal.
    pub fn set_phase(&mut self, servo: ServoIdx, phase: f32, load: bool) {
        self.servo_phases[servo.0 as usize] = phase.clamp(0.0, 1.0);
        // servo count already checked above
        let _ = self.pwms.set_channel_offset(
            servo.0,
            (self.servo_phases[servo.0 as usize] * self.pwms.top() as f32) as u32,
            load,
        );
    }

    /// Get the pwm frequency being used for this `ServoCluster`.
    pub fn frequency(&self) -> f32 {
        self.pwm_frequency
    }

    /// Set the pwm frequency being used for this `ServoCluster`.
    pub fn set_frequency(&mut self, system_clock_hz: HertzU32, frequency: f32) {
        if (servo_state::MIN_FREQUENCY..=servo_state::MAX_FREQUENCY).contains(&frequency) {
            if let Some((period, div256)) =
                PwmCluster::<NUM_SERVOS, P, SM>::calculate_pwm_factors(system_clock_hz, frequency)
            {
                self.pwm_period = period;
                self.pwm_frequency = frequency;

                for servo in 0..NUM_SERVOS {
                    if self.states[servo].enabled() {
                        // Unwrap ok because state is enabled
                        self.apply_pulse(
                            ServoIdx(servo as u8),
                            self.states[servo].pulse().unwrap(),
                            false,
                        );
                    }
                    // Won't error because iterationg over NUM_SERVOS.
                    let _ = self.pwms.set_channel_offset(
                        servo as u8,
                        (self.servo_phases[servo] * self.pwm_period as f32) as u32,
                        false,
                    );
                }

                self.pwms.set_top(self.pwm_period, true);
                let div = (div256 >> 8) as u16;
                let frac = (div256 % 256) as u8;
                self.pwms.clock_divisor_fixed_point(div, frac);
            }
        }
    }

    /// Get the min value for a particular servo.
    pub fn min_value(&self, servo: ServoIdx) -> f32 {
        self.states[servo.0 as usize].min_value()
    }

    /// Get the mid value for a particular servo.
    pub fn mid_value(&self, servo: ServoIdx) -> f32 {
        self.states[servo.0 as usize].mid_value()
    }

    /// Get the max value for a particular servo.
    pub fn max_value(&self, servo: ServoIdx) -> f32 {
        self.states[servo.0 as usize].max_value()
    }

    /// Set the min value for a particular servo. If `load` is true, immediately update the servo
    /// pwm signal.
    pub fn to_min(&mut self, servo: ServoIdx, load: bool) {
        let new_pulse = self.states[servo.0 as usize].to_min_with_return();
        self.apply_pulse(servo, new_pulse, load);
    }

    /// Set the mid value for a particular servo. If `load` is true, immediately update the servo
    /// pwm signal.
    pub fn to_mid(&mut self, servo: ServoIdx, load: bool) {
        let new_pulse = self.states[servo.0 as usize].to_mid_with_return();
        self.apply_pulse(servo, new_pulse, load);
    }

    /// Set the max value for a particular servo. If `load` is true, immediately update the servo
    /// pwm signal.
    pub fn to_max(&mut self, servo: ServoIdx, load: bool) {
        let new_pulse = self.states[servo.0 as usize].to_max_with_return();
        self.apply_pulse(servo, new_pulse, load);
    }

    /// Set a particular servo to a percentage of its range. 0% maps to the servo's minimum value,
    /// 100% maps to the servo's maximum value.
    pub fn to_percent(&mut self, servo: ServoIdx, percent: f32, load: bool) {
        let new_pulse = self.states[servo.0 as usize].to_percent_with_return(percent);
        self.apply_pulse(servo, new_pulse, load);
    }

    /// Get a shared reference to a particular servo's calibration.
    pub fn calibration(&self, servo: ServoIdx) -> &Calibration<Cal> {
        self.states[servo.0 as usize].calibration()
    }

    /// Get a unique reference to a particular servo's calibration.
    pub fn calibration_mut(&mut self, servo: ServoIdx) -> &mut Calibration<Cal> {
        self.states[servo.0 as usize].calibration_mut()
    }

    /// Immediately update the pwm state for all servos.
    pub fn load(&mut self) {
        self.pwms.load_pwm()
    }

    /// Apply a pulse to a particular servo. If `load` is true, immeditely update the servo pwm
    /// signal.
    fn apply_pulse(&mut self, servo: ServoIdx, pulse: f32, load: bool) {
        // unwrap ok because servo checked at call sites.
        let level = ServoState::<Cal>::pulse_to_level(pulse, self.pwm_period, self.pwm_frequency);
        let _ = self.pwms.set_channel_level(servo.0, level, load);
    }
}
