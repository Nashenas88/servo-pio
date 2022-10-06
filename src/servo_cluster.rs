use crate::calibration::{Calibration, CalibrationData, NoCustom, Point};
use crate::pwm_cluster::{GlobalState, GlobalStates, PwmCluster, PwmClusterBuilder};
use crate::servo_state::{self, ServoState, DEFAULT_FREQUENCY};
use crate::{alloc_array, alloc_array_by_index};
use defmt::Format;
use fugit::HertzU32;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex};
use pimoroni_servo2040::hal::gpio::{
    DynPin, Error as GpioError, Function, FunctionConfig, PinMode,
};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO};
use pimoroni_servo2040::hal::Clock;

pub struct ServoCluster<const NUM_SERVOS: usize, P, SM, Cal = NoCustom>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    pwms: PwmCluster<NUM_SERVOS, P, SM>,
    pwm_period: u32,
    pwm_frequency: f32,
    states: [ServoState<Cal>; NUM_SERVOS],
    servo_phases: [f32; NUM_SERVOS],
}

pub struct ServoClusterBuilder<
    'a,
    Cal,
    C,
    P,
    SM,
    const NUM_SERVOS: usize,
    const NUM_CHANNELS: usize,
> where
    C: ChannelIndex + 'static,
    P: PIOExt + 'static,
    SM: StateMachineIndex + 'static,
{
    pio: &'a mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channel: Channel<C>,
    global_states: &'static mut GlobalStates<NUM_CHANNELS>,
    pins: Option<[DynPin; NUM_SERVOS]>,
    side_set_pin: Option<DynPin>,
    pwm_frequency: Option<f32>,
    calibration: Option<Calibration<Cal>>,
    auto_phase: Option<bool>,
}

impl<'a, Cal, C, P, SM, const NUM_SERVOS: usize, const NUM_CHANNELS: usize>
    ServoClusterBuilder<'a, Cal, C, P, SM, NUM_SERVOS, NUM_CHANNELS>
where
    Cal: CalibrationData + Default + Clone,
    <Cal as CalibrationData>::Custom: Iterator<Item = (Point, Point)>,
    C: ChannelIndex,
    P: PIOExt + FunctionConfig,
    Function<P>: PinMode,
    SM: StateMachineIndex,
{
    pub fn calibration(mut self, calibration: Calibration<Cal>) -> Self {
        self.calibration = Some(calibration);
        self
    }

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

    pub fn pins(mut self, dyn_pins: [DynPin; NUM_SERVOS]) -> Result<Self, GpioError> {
        for pin in &dyn_pins {
            if pin.mode() != <Function<P> as PinMode>::DYN {
                return Err(GpioError::InvalidPinMode);
            }
        }

        self.pins = Some(dyn_pins);

        Ok(self)
    }

    pub fn side_set_pin(mut self, side_set_pin: DynPin) -> Result<Self, GpioError> {
        if side_set_pin.mode() == <Function<P> as PinMode>::DYN {
            self.side_set_pin = Some(side_set_pin);
            Ok(self)
        } else {
            Err(GpioError::InvalidPinMode)
        }
    }

    pub fn pwm_frequency(mut self, pwm_frequency: f32) -> Self {
        self.pwm_frequency = Some(pwm_frequency);
        self
    }

    pub fn build(
        mut self,
        system_clock: &SystemClock,
        maybe_global_state: &'static mut Option<GlobalState<C, P, SM>>,
    ) -> Result<ServoCluster<NUM_SERVOS, P, SM, Cal>, ServoClusterBuilderError> {
        let pins = self.pins.ok_or(ServoClusterBuilderError::MissingPins)?;
        let side_set_pin = self
            .side_set_pin
            .ok_or(ServoClusterBuilderError::MissingSideSet)?;
        let (states, servo_phases) = ServoCluster::<NUM_SERVOS, P, SM, _>::create_servo_states(
            self.calibration,
            self.auto_phase.unwrap_or(true),
        );
        let global_state = self.global_states.get_mut(&mut self.dma_channel, move || {
            PwmClusterBuilder::<NUM_SERVOS>::prep_global_state(maybe_global_state)
        });
        let mut pwms = {
            {
                PwmCluster::<NUM_SERVOS, P, SM>::builder()
                    .pins(&pins)
                    .side_pin(&side_set_pin)
                    .build(
                        pins,
                        side_set_pin,
                        self.pio,
                        self.sm,
                        self.dma_channel,
                        system_clock,
                        global_state.ok_or(ServoClusterBuilderError::MismatchingGlobalState)?,
                    )
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

#[derive(Format)]
pub enum ServoClusterBuilderError {
    MismatchingGlobalState,
    MissingPins,
    MissingSideSet,
}

impl<'a, const NUM_SERVOS: usize, P, SM, Ca> ServoCluster<NUM_SERVOS, P, SM, Ca>
where
    Ca: Default + CalibrationData + Clone,
    <Ca as CalibrationData>::Custom: Iterator<Item = (Point, Point)>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    pub fn builder<C, const NUM_CHANNELS: usize>(
        pio: &'a mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        dma_channel: Channel<C>,
        global_states: &'static mut GlobalStates<NUM_CHANNELS>,
    ) -> ServoClusterBuilder<'a, Ca, C, P, SM, NUM_SERVOS, NUM_CHANNELS>
    where
        C: ChannelIndex + 'static,
        P: PIOExt + 'static,
    {
        ServoClusterBuilder {
            pio,
            sm,
            dma_channel,
            global_states,
            pins: None,
            side_set_pin: None,
            pwm_frequency: None,
            calibration: None,
            auto_phase: None,
        }
    }

    fn create_servo_states(
        calibration: Option<Calibration<Ca>>,
        auto_phase: bool,
    ) -> ([ServoState<Ca>; NUM_SERVOS], [f32; NUM_SERVOS]) {
        (
            match calibration {
                Some(calibration) => alloc_array::<ServoState<Ca>, NUM_SERVOS>(move || {
                    ServoState::with_calibration(calibration.clone())
                }),
                None => alloc_array::<ServoState<Ca>, NUM_SERVOS>(|| ServoState::new()),
            },
            alloc_array_by_index::<f32, NUM_SERVOS>(|i| {
                if auto_phase {
                    i as f32 / NUM_SERVOS as f32
                } else {
                    0.0
                }
            }),
        )
    }

    pub fn enabled(&self, servo: u8) -> bool {
        self.states[servo as usize].enabled()
    }

    pub fn set_enabled(&mut self, servo: u8, enable: bool, load: bool) {
        let state = &mut self.states[servo as usize];
        let new_pulse = if enable {
            state.enable_with_return()
        } else {
            state.disable_with_return()
        };
        self.apply_pulse(servo, new_pulse, load);
    }

    pub fn pulse(&self, servo: u8) -> f32 {
        self.states[servo as usize].pulse()
    }

    pub fn set_pulse(&mut self, servo: u8, pulse: f32, load: bool) {
        let new_pulse = self.states[servo as usize].set_pulse_with_return(pulse);
        self.apply_pulse(servo, new_pulse, load);
    }

    pub fn value(&self, servo: u8) -> f32 {
        self.states[servo as usize].value()
    }

    pub fn set_value(&mut self, servo: u8, value: f32, load: bool) {
        let new_pulse = self.states[servo as usize].set_value_with_return(value);
        self.apply_pulse(servo, new_pulse, load);
    }

    pub fn phase(&self, servo: u8) -> f32 {
        self.servo_phases[servo as usize]
    }

    pub fn set_phase(&mut self, servo: u8, phase: f32, load: bool) {
        self.servo_phases[servo as usize] = phase.max(0.0).min(1.0);
        // servo count already checked above
        let _ = self.pwms.set_channel_offset(
            servo,
            (self.servo_phases[servo as usize] * self.pwms.top() as f32) as u32,
            load,
        );
    }

    pub fn frequency(&self) -> f32 {
        self.pwm_frequency
    }

    pub fn set_frequency(&mut self, system_clock_hz: HertzU32, frequency: f32) {
        if (servo_state::MIN_FREQUENCY..=servo_state::MAX_FREQUENCY).contains(&frequency) {
            if let Some((period, div256)) =
                PwmCluster::<NUM_SERVOS, P, SM>::calculate_pwm_factors(system_clock_hz, frequency)
            {
                self.pwm_period = period;
                self.pwm_frequency = frequency;

                for servo in 0..NUM_SERVOS {
                    if self.states[servo].enabled() {
                        self.apply_pulse(servo as u8, self.states[servo].pulse(), false);
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

    pub fn min_value(&self, servo: u8) -> f32 {
        self.states[servo as usize].min_value()
    }

    pub fn mid_value(&self, servo: u8) -> f32 {
        self.states[servo as usize].mid_value()
    }

    pub fn max_value(&self, servo: u8) -> f32 {
        self.states[servo as usize].max_value()
    }

    pub fn to_min(&mut self, servo: u8, load: bool) {
        let new_pulse = self.states[servo as usize].to_min_with_return();
        self.apply_pulse(servo, new_pulse, load);
    }

    pub fn to_mid(&mut self, servo: u8, load: bool) {
        let new_pulse = self.states[servo as usize].to_mid_with_return();
        self.apply_pulse(servo, new_pulse, load);
    }

    pub fn to_max(&mut self, servo: u8, load: bool) {
        let new_pulse = self.states[servo as usize].to_max_with_return();
        self.apply_pulse(servo, new_pulse, load);
    }

    pub fn to_percent(&mut self, servo: u8, percent: f32, load: bool) {
        let new_pulse = self.states[servo as usize].to_percent_with_return(percent);
        self.apply_pulse(servo, new_pulse, load);
    }

    pub fn calibration(&self, servo: u8) -> &Calibration<Ca> {
        self.states[servo as usize].calibration()
    }

    pub fn calibration_mut(&mut self, servo: u8) -> &mut Calibration<Ca> {
        self.states[servo as usize].calibration_mut()
    }

    pub fn load(&mut self) {
        self.pwms.load_pwm()
    }

    fn apply_pulse(&mut self, servo: u8, pulse: f32, load: bool) {
        // unwrap ok because servo checked at call sites.
        let level = ServoState::<Ca>::pulse_to_level(pulse, self.pwm_period, self.pwm_frequency);
        defmt::trace!("Pulse to level is {}", level);
        let _ = self.pwms.set_channel_level(servo, level, load);
    }
}
