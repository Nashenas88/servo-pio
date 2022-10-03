use crate::calibration::{self, Calibration, CalibrationData, Point};

pub const DEFAULT_FREQUENCY: f32 = 50.0;
pub const MIN_FREQUENCY: f32 = 10.0;
pub const MAX_FREQUENCY: f32 = 350.0;
const MIN_VALID_PULSE: f32 = 1.0;

pub struct ServoState<C> {
    servo_value: f32,
    last_enabled_pulse: f32,
    enabled: bool,
    calibration: Calibration<C>,
}

impl<C> ServoState<C>
where
    C: Default + Clone,
{
    pub fn new() -> Self {
        Self {
            servo_value: 0.0,
            last_enabled_pulse: 0.0,
            enabled: false,
            calibration: Calibration::new(),
        }
    }
}

impl<C> Default for ServoState<C>
where
    C: Default + Clone,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<C> ServoState<C>
where
    C: CalibrationData,
    <C as CalibrationData>::Custom: Iterator<Item = (Point, Point)>,
{
    pub fn with_calibration(calibration: Calibration<C>) -> Self {
        Self {
            servo_value: 0.0,
            last_enabled_pulse: 0.0,
            enabled: false,
            calibration,
        }
    }

    pub fn enable_with_return(&mut self) -> f32 {
        if self.last_enabled_pulse < MIN_VALID_PULSE {
            self.go_to_mid_with_return()
        } else {
            self.inner_enable_with_return()
        }
    }

    fn go_to_mid_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.calibration.mid_value())
    }

    fn inner_enable_with_return(&mut self) -> f32 {
        self.enabled = true;
        self.last_enabled_pulse
    }

    pub fn set_value_with_return(&mut self, value: f32) -> f32 {
        if let Some(point) = self.calibration.value_to_pulse(value) {
            self.last_enabled_pulse = point.pulse;
            self.servo_value = point.value;
            self.inner_enable_with_return()
        } else {
            self.disable_with_return()
        }
    }

    pub fn disable_with_return(&mut self) -> f32 {
        self.enabled = false;
        0.0 // A zero pulse.
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn pulse_to_level(pulse: f32, resolution: u32, frequency: f32) -> u32 {
        if pulse >= MIN_VALID_PULSE {
            (pulse * resolution as f32 * frequency) as u32 / 1_000_000
        } else {
            0
        }
    }

    pub fn pulse(&self) -> f32 {
        self.last_enabled_pulse
    }

    pub fn set_pulse_with_return(&mut self, pulse: f32) -> f32 {
        if pulse >= MIN_VALID_PULSE {
            if let Some(point) = self.calibration.pulse_to_value(pulse) {
                self.servo_value = point.value;
                self.last_enabled_pulse = point.pulse;
                return self.inner_enable_with_return();
            }
        }
        self.disable_with_return()
    }

    pub fn value(&self) -> f32 {
        self.servo_value
    }

    pub(crate) fn min_value(&self) -> f32 {
        if <C as CalibrationData>::LEN > 0 {
            self.calibration.first().value
        } else {
            0.0
        }
    }

    pub(crate) fn mid_value(&self) -> f32 {
        if <C as CalibrationData>::LEN > 0 {
            self.calibration.mid_value()
        } else {
            0.0
        }
    }

    pub(crate) fn max_value(&self) -> f32 {
        if <C as CalibrationData>::LEN > 0 {
            self.calibration.last().value
        } else {
            0.0
        }
    }

    pub fn to_min_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.min_value())
    }

    pub fn to_mid_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.mid_value())
    }

    pub fn to_max_with_return(&mut self) -> f32 {
        self.set_value_with_return(self.max_value())
    }

    pub fn to_percent_with_return(&mut self, percent: f32) -> f32 {
        let value = calibration::map_float(percent, 0.0, 100.0, self.min_value(), self.max_value());
        self.set_value_with_return(value)
    }

    pub fn calibration(&self) -> &Calibration<C> {
        &self.calibration
    }

    pub fn calibration_mut(&mut self) -> &mut Calibration<C> {
        &mut self.calibration
    }
}
