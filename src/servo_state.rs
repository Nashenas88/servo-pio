use crate::calibration::{self, Calibration, CalibrationData, Point};

pub const DEFAULT_FREQUENCY: f32 = 50.0;
pub const MIN_FREQUENCY: f32 = 10.0;
pub const MAX_FREQUENCY: f32 = 350.0;
const MIN_VALID_PULSE: f32 = 1.0;

/// Type to manage state for a single servo.
pub struct ServoState<C> {
    /// The current value for the servo. The unit is derived from the calibration.
    servo_value: f32,
    /// The most recent pulse value for this servo while it was enabled.
    last_enabled_pulse: Option<f32>,
    /// Whether or not the servo is enabled.
    enabled: bool,
    /// The calibration for this servo.
    calibration: Calibration<C>,
}

impl<C> ServoState<C>
where
    C: Default + Clone + CalibrationData,
{
    /// Constructs a new `ServoState` if the calibration type has 2 or more calibration points.
    pub fn new() -> Option<Self> {
        Some(Self {
            servo_value: 0.0,
            last_enabled_pulse: None,
            enabled: false,
            calibration: Calibration::new()?,
        })
    }
}

impl<C> ServoState<C>
where
    C: CalibrationData,
    <C as CalibrationData>::Custom: Iterator<Item = (Point, Point)>,
{
    /// Construct a `ServoState` based on some [Calibration].
    pub fn with_calibration(calibration: Calibration<C>) -> Self {
        Self {
            servo_value: 0.0,
            last_enabled_pulse: None,
            enabled: false,
            calibration,
        }
    }

    /// Enable the servo and return its pulse.
    pub fn enable_with_return(&mut self) -> f32 {
        if let Some(pulse) = self.last_enabled_pulse {
            self.enabled = true;
            pulse
        } else {
            self.go_to_mid_with_return()
        }
    }

    /// Set the value to the mid value and return the pulse.
    #[inline]
    fn go_to_mid_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.calibration.mid_value())
    }

    /// # Panic Safety
    /// Caller ensures last_enabled_pulse is occupied.
    #[inline]
    fn inner_enable_with_return(&mut self) -> Option<f32> {
        self.enabled = true;
        self.last_enabled_pulse
    }

    /// Set the value and return the pulse.
    pub fn set_value_with_return(&mut self, value: f32) -> f32 {
        let point = self.calibration.value_to_pulse(value);
        self.last_enabled_pulse = Some(point.pulse);
        self.servo_value = point.value;
        self.enabled = true;
        point.pulse
    }

    /// Disable the servo.
    #[inline]
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Return whether the servo is enabled.
    #[inline]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Convert the pulse, given a resolution and frequency, into a level for use by pwm signals.
    pub fn pulse_to_level(pulse: f32, resolution: u32, frequency: f32) -> u32 {
        if pulse >= MIN_VALID_PULSE {
            ((pulse * resolution as f32 * frequency) as u64 / 1_000_000) as u32
        } else {
            0
        }
    }

    /// Get the pulse if available.
    #[inline]
    pub fn pulse(&self) -> Option<f32> {
        self.last_enabled_pulse
    }

    /// Set the pulse and return the new pulse value. Can return `None` when pulse is not larger
    /// than the minimum valid pulse.
    pub fn set_pulse_with_return(&mut self, pulse: f32) -> Option<f32> {
        if pulse >= MIN_VALID_PULSE {
            if let Some(point) = self.calibration.pulse_to_value(pulse) {
                self.servo_value = point.value;
                self.last_enabled_pulse = Some(point.pulse);
                return self.inner_enable_with_return();
            }
        }
        self.disable();
        None
    }

    /// Get the current value of the servo.
    #[inline]
    pub fn value(&self) -> f32 {
        self.servo_value
    }

    /// Get the minimum value the servo can move to.
    #[inline]
    pub(crate) fn min_value(&self) -> f32 {
        self.calibration.first().value
    }

    /// Get the mid-point value the servo can move to.
    #[inline]
    pub(crate) fn mid_value(&self) -> f32 {
        self.calibration.mid_value()
    }

    /// Get the max value the servo can move to.
    #[inline]
    pub(crate) fn max_value(&self) -> f32 {
        self.calibration.last().value
    }

    /// Move the servo to the minimum value and return the pulse.
    #[inline]
    pub fn to_min_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.min_value())
    }

    /// Move the servo to the mid-point value and return the pulse.
    #[inline]
    pub fn to_mid_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.mid_value())
    }

    /// Move the servo to the maximum value and return the pulse.
    #[inline]
    pub fn to_max_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.max_value())
    }

    /// Move the servo to a percentage of its movement range.
    ///
    /// 0% corresponds to the servo's minimum value, and 100% corresponds to the servo's maximum value.
    pub fn to_percent_with_return(&mut self, percent: f32) -> f32 {
        let value = calibration::map_float(percent, 0.0, 100.0, self.min_value(), self.max_value());
        self.set_value_with_return(value)
    }

    /// Get a shared reference of the servo's calibration.
    #[inline]
    pub fn calibration(&self) -> &Calibration<C> {
        &self.calibration
    }

    /// Get a unique reference to the servos' calibration.
    #[inline]
    pub fn calibration_mut(&mut self) -> &mut Calibration<C> {
        &mut self.calibration
    }
}
