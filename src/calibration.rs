pub const DEFAULT_MIN_PULSE: f32 = 500.0;
pub const DEFAULT_MID_PULSE: f32 = 1500.0;
pub const DEFAULT_MAX_PULSE: f32 = 2500.0;

const LOWER_HARD_LIMIT: f32 = 400.0;
const UPPER_HARD_LIMIT: f32 = 2600.0;

#[derive(Copy, Clone)]
pub enum CalibrationType {
    Angular,
    Linear,
    Continuous,
}

#[derive(Copy, Clone)]
pub struct Point {
    pub pulse: f32,
    pub value: f32,
}

impl Point {
    fn new() -> Self {
        Self {
            pulse: 0.0,
            value: 0.0,
        }
    }
}

pub enum CalibrationIters<'a, C: 'a> {
    Angular(AngularCalibrationIter<'a>),
    Linear(LinearCalibrationIter<'a>),
    Continuous(ContinuousCalibrationIter<'a>),
    Custom(C),
}

impl<'a, C> Iterator for CalibrationIters<'a, C>
where
    C: Iterator<Item = (Point, Point)>,
{
    type Item = (Point, Point);

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            CalibrationIters::Angular(a) => a.next(),
            CalibrationIters::Linear(l) => l.next(),
            CalibrationIters::Continuous(c) => c.next(),
            CalibrationIters::Custom(c) => c.next(),
        }
    }
}

pub trait CalibrationData {
    const LEN: usize;
    type Custom;
    fn first(&self) -> Point;
    fn second(&self) -> Point;
    fn penultimate(&self) -> Point;
    fn last(&self) -> Point;
    fn windows(&self) -> CalibrationIters<'_, Self::Custom>;
}

#[derive(Default, Copy, Clone)]
pub struct NoCustom;

impl CalibrationData for NoCustom {
    const LEN: usize = 0;
    type Custom = Self;
    fn first(&self) -> Point {
        Point::new()
    }

    fn second(&self) -> Point {
        Point::new()
    }

    fn penultimate(&self) -> Point {
        Point::new()
    }

    fn last(&self) -> Point {
        Point::new()
    }

    fn windows(&self) -> CalibrationIters<'_, Self::Custom> {
        CalibrationIters::Custom(NoCustom)
    }
}

impl Iterator for NoCustom {
    type Item = (Point, Point);

    fn next(&mut self) -> Option<Self::Item> {
        None
    }
}

#[derive(Copy, Clone)]
pub struct AngularCalibration {
    min: Point,
    mid: Point,
    max: Point,
}

impl Default for AngularCalibration {
    fn default() -> Self {
        Self {
            mid: Point {
                pulse: DEFAULT_MIN_PULSE,
                value: -90.0,
            },
            min: Point {
                pulse: DEFAULT_MID_PULSE,
                value: 0.0,
            },
            max: Point {
                pulse: DEFAULT_MAX_PULSE,
                value: 90.0,
            },
        }
    }
}

pub struct AngularCalibrationIter<'a> {
    calibration: &'a AngularCalibration,
    count: u8,
}

impl<'a> AngularCalibrationIter<'a> {
    fn new(calibration: &'a AngularCalibration) -> Self {
        Self {
            calibration,
            count: 0,
        }
    }
}

impl<'a> Iterator for AngularCalibrationIter<'a> {
    type Item = (Point, Point);

    fn next(&mut self) -> Option<Self::Item> {
        match self.count {
            0 => Some((self.calibration.min, self.calibration.mid)),
            1 => Some((self.calibration.mid, self.calibration.max)),
            _ => None,
        }
    }
}

impl CalibrationData for AngularCalibration {
    const LEN: usize = 3;
    type Custom = NoCustom;

    fn first(&self) -> Point {
        self.min
    }

    fn second(&self) -> Point {
        self.mid
    }

    fn penultimate(&self) -> Point {
        self.mid
    }

    fn last(&self) -> Point {
        self.max
    }

    fn windows(&self) -> CalibrationIters<'_, NoCustom> {
        CalibrationIters::Angular(AngularCalibrationIter::new(self))
    }
}

pub struct LinearCalibration {
    min: Point,
    max: Point,
}

impl Default for LinearCalibration {
    fn default() -> Self {
        Self {
            min: Point {
                pulse: DEFAULT_MIN_PULSE,
                value: 0.0,
            },
            max: Point {
                pulse: DEFAULT_MAX_PULSE,
                value: 1.0,
            },
        }
    }
}

pub struct LinearCalibrationIter<'a> {
    calibration: &'a LinearCalibration,
    count: u8,
}

impl<'a> LinearCalibrationIter<'a> {
    fn new(calibration: &'a LinearCalibration) -> Self {
        Self {
            calibration,
            count: 0,
        }
    }
}

impl<'a> Iterator for LinearCalibrationIter<'a> {
    type Item = (Point, Point);

    fn next(&mut self) -> Option<Self::Item> {
        match self.count {
            0 => Some((self.calibration.min, self.calibration.max)),
            _ => None,
        }
    }
}

impl CalibrationData for LinearCalibration {
    const LEN: usize = 3;
    type Custom = ();

    fn first(&self) -> Point {
        self.min
    }

    fn second(&self) -> Point {
        self.max
    }

    fn penultimate(&self) -> Point {
        self.min
    }

    fn last(&self) -> Point {
        self.max
    }

    fn windows(&self) -> CalibrationIters<'_, ()> {
        CalibrationIters::Linear(LinearCalibrationIter::new(self))
    }
}

pub struct ContinuousCalibration {
    min: Point,
    mid: Point,
    max: Point,
}

impl Default for ContinuousCalibration {
    fn default() -> Self {
        Self {
            mid: Point {
                pulse: DEFAULT_MIN_PULSE,
                value: -1.0,
            },
            min: Point {
                pulse: DEFAULT_MID_PULSE,
                value: 0.0,
            },
            max: Point {
                pulse: DEFAULT_MAX_PULSE,
                value: 1.0,
            },
        }
    }
}

pub struct ContinuousCalibrationIter<'a> {
    calibration: &'a ContinuousCalibration,
    count: u8,
}

impl<'a> ContinuousCalibrationIter<'a> {
    fn new(calibration: &'a ContinuousCalibration) -> Self {
        Self {
            calibration,
            count: 0,
        }
    }
}

impl<'a> Iterator for ContinuousCalibrationIter<'a> {
    type Item = (Point, Point);

    fn next(&mut self) -> Option<Self::Item> {
        match self.count {
            0 => Some((self.calibration.min, self.calibration.max)),
            _ => None,
        }
    }
}

impl CalibrationData for ContinuousCalibration {
    const LEN: usize = 3;
    type Custom = ();

    fn first(&self) -> Point {
        self.min
    }

    fn second(&self) -> Point {
        self.mid
    }

    fn penultimate(&self) -> Point {
        self.mid
    }

    fn last(&self) -> Point {
        self.max
    }

    fn windows(&self) -> CalibrationIters<'_, ()> {
        CalibrationIters::Continuous(ContinuousCalibrationIter::new(self))
    }
}

#[derive(Clone)]
pub struct Calibration<C> {
    calibration: C,
    limit_lower: bool,
    limit_upper: bool,
}

impl<C> Calibration<C>
where
    C: Default + Clone,
{
    pub fn new() -> Self {
        Self {
            calibration: Default::default(),
            limit_lower: true,
            limit_upper: true,
        }
    }
}

impl<C> Default for Calibration<C>
where
    C: Default + Clone,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<C> Calibration<C>
where
    C: CalibrationData,
    <C as CalibrationData>::Custom: Iterator<Item = (Point, Point)>,
{
    pub(crate) fn value_to_pulse(&self, value: f32) -> Option<Point> {
        if <C as CalibrationData>::LEN < 2 {
            return None;
        }

        let first = self.calibration.first();
        let last = self.calibration.last();

        // Is the vale below the bottom-most calibration pair?
        let mut point = if value < first.value {
            // Should the value be limited to the calibration or projected below it?
            if self.limit_lower {
                Some(first)
            } else {
                let second = self.calibration.second();
                Some(Point {
                    pulse: map_float(value, first.value, second.value, first.pulse, second.pulse),
                    value,
                })
            }
            // Is the value above the top-most calibration pair?
        } else if value > last.value {
            // Should the value be limited to the calibration or be projected above it?
            if self.limit_upper {
                Some(last)
            } else {
                let penultimate = self.calibration.penultimate();
                Some(Point {
                    pulse: map_float(
                        value,
                        penultimate.value,
                        last.value,
                        penultimate.pulse,
                        last.pulse,
                    ),
                    value,
                })
            }
        } else {
            // The value must be between two calibration points, so iterate through them to find which ones.
            let mut point = None;
            for (smaller, larger) in self.calibration.windows() {
                if value < larger.value {
                    point = Some(Point {
                        pulse: map_float(
                            value,
                            smaller.value,
                            larger.value,
                            smaller.pulse,
                            larger.pulse,
                        ),
                        value,
                    });
                    break;
                }
            }
            point
        }?;

        // Clamp the pulse between the hard limits.
        if point.pulse < LOWER_HARD_LIMIT || point.pulse > UPPER_HARD_LIMIT {
            point.pulse = point.pulse.max(LOWER_HARD_LIMIT).min(UPPER_HARD_LIMIT);

            // Is the pulse below the bottom-most calibration pair?
            if point.pulse < first.pulse {
                let second = self.calibration.second();
                point.value = map_float(
                    point.pulse,
                    first.pulse,
                    second.pulse,
                    first.value,
                    second.value,
                );
                // Is the pulse above the top-most calibration pair?
            } else if point.pulse > last.pulse {
                let penultimate = self.calibration.penultimate();
                point.value = map_float(
                    point.pulse,
                    penultimate.pulse,
                    last.pulse,
                    penultimate.value,
                    last.value,
                );
            } else {
                // The pulse must be between two calibration points, so iterate through them to find which ones.
                for (smaller, larger) in self.calibration.windows() {
                    if point.pulse < larger.pulse {
                        point.value = map_float(
                            point.pulse,
                            smaller.pulse,
                            larger.pulse,
                            smaller.value,
                            larger.value,
                        );
                        break;
                    }
                }
            }
        }

        Some(point)
    }

    pub fn pulse_to_value(&self, pulse: f32) -> Option<Point> {
        if <C as CalibrationData>::LEN < 2 {
            return None;
        }

        // Clamp the pulse between the hard limits
        let mut pulse_out = pulse.max(LOWER_HARD_LIMIT).min(UPPER_HARD_LIMIT);
        let mut value_out = 0.0;

        // Is the pulse below the bottom most calibration pair?
        if pulse_out < self.calibration.first().pulse {
            // Should the pulse be limited to the calibration or projected below it?
            if self.limit_lower {
                value_out = self.calibration.first().value;
                pulse_out = self.calibration.first().pulse;
            } else {
                value_out = map_float(
                    pulse,
                    self.calibration.first().pulse,
                    self.calibration.second().pulse,
                    self.calibration.first().value,
                    self.calibration.second().value,
                );
            }
        }
        // Is the pulse above the top most calibration pair?
        else if pulse > self.calibration.last().pulse {
            // Should the pulse be limited to the calibration or projected above it?
            if self.limit_upper {
                value_out = self.calibration.last().value;
                pulse_out = self.calibration.last().pulse;
            } else {
                value_out = map_float(
                    pulse,
                    self.calibration.penultimate().pulse,
                    self.calibration.last().pulse,
                    self.calibration.penultimate().value,
                    self.calibration.last().value,
                );
            }
        } else {
            // The pulse must between two calibration pairs, so iterate through them to find which ones
            for (left, right) in self.calibration.windows() {
                if pulse < right.pulse {
                    value_out = map_float(pulse, left.pulse, right.pulse, left.value, right.value);
                    break; // No need to continue checking so break out of the loop
                }
            }
        }

        Some(Point {
            value: value_out,
            pulse: pulse_out,
        })
    }

    pub fn first(&self) -> Point {
        self.calibration.first()
    }

    pub fn mid_value(&self) -> f32 {
        (self.calibration.first().value + self.calibration.last().value) / 2.0
    }

    pub fn last(&self) -> Point {
        self.calibration.last()
    }
}

pub fn map_float(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    ((value - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min
}
