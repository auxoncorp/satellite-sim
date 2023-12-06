use crate::{
    satellite::{SatelliteEnvironment, SatelliteSharedState},
    units::{Ratio, Temperature, TemperatureInterval, TemperatureIntervalRate, Time},
    SimulationComponent,
};
use oorandom::Rand64;

#[derive(Debug)]
pub struct TemperatureSensor {
    config: TemperatureSensorConfig,
    prng: Rand64,
    time_base: Time,
    using_day_params: bool,
    temperature: Temperature,
}

#[derive(Debug, Copy, Clone)]
pub struct TemperatureSensorConfig {
    pub model: TemperatureSensorModel,
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum TemperatureSensorModel {
    /// Temperature is fixed at a constant value
    Constant(Temperature),

    /// Temperature randomly increases or decreases within the interval over time
    RandomInterval(TemperatureSensorRandomIntervalModelParams),

    /// Temperature increases or decreases linearly over time
    Linear(TemperatureSensorLinearModelParams),

    ///Temperature increases or decreases exponentially over time
    Exponential(TemperatureSensorExponentialModelParams),
}

pub type TemperatureSensorRandomIntervalModelParams =
    TemperatureSensorModelParams<TemperatureInterval>;
pub type TemperatureSensorLinearModelParams = TemperatureSensorModelParams<TemperatureIntervalRate>;
pub type TemperatureSensorExponentialModelParams =
    TemperatureSensorModelParams<TemperatureSensorExponential>;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TemperatureSensorModelParams<T> {
    /// Initial temperature    
    pub initial: Temperature,

    /// The min temperature to clamp to
    pub min: Temperature,

    /// The max temperature to clamp to
    pub max: Temperature,

    /// The parameters to use when the sun is visible
    pub day: T,

    /// The parameters to use when eclipsed
    pub night: T,
}

/// Generalized exponential function for temperature.
///
/// temperature = k·bˢᵗ + o
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TemperatureSensorExponential {
    /// The base `b`
    pub base: f64,
    /// The vertical scaling factor `k`
    pub vertical_scaling_factor: f64,
    /// The horizontal scaling factor `s`
    pub horizontal_scaling_factor: f64,
    /// The offset `o`
    pub offset: f64,
}

impl TemperatureSensorExponential {
    fn temp_at_t(&self, t: Time) -> Temperature {
        let t = t.as_secs();
        let y = self.vertical_scaling_factor * self.base.powf(self.horizontal_scaling_factor * t)
            + self.offset;
        Temperature::from_degrees_celsius(y)
    }
}

impl TemperatureSensorRandomIntervalModelParams {
    fn interval(&self, sun_visible: bool) -> TemperatureInterval {
        if sun_visible {
            self.day
        } else {
            self.night
        }
    }
}

impl TemperatureSensorLinearModelParams {
    fn rate(&self, sun_visible: bool) -> TemperatureIntervalRate {
        if sun_visible {
            self.day
        } else {
            self.night
        }
    }
}

impl TemperatureSensorExponentialModelParams {
    fn exponential(&self, sun_visible: bool) -> TemperatureSensorExponential {
        if sun_visible {
            self.day
        } else {
            self.night
        }
    }
}

impl TemperatureSensor {
    pub fn new(config: TemperatureSensorConfig) -> Self {
        if let TemperatureSensorModel::Linear(params) = config.model {
            assert!(params.initial >= params.min);
            assert!(params.initial <= params.max);
            assert!(params.max > params.min);
        }

        let temperature = config.model.initial_temperature();
        Self {
            config,
            prng: Rand64::new(0), // NOTE: seed is fixed, could be provided if we need it
            time_base: Time::from_secs(0.0),
            using_day_params: true,
            temperature,
        }
    }

    pub fn temperature(&self) -> Temperature {
        self.temperature
    }

    pub fn reset(&mut self) {
        self.time_base = Time::from_secs(0.0);
        self.temperature = self.config.model.initial_temperature();
    }

    /// Returns the temperature normalized in [-1, 1] with the model's min and max.
    /// Returns `None` if the model variant is `TemperatureSensorModel::Constant`.
    pub fn normalized_temperature(&self) -> Option<Ratio> {
        use TemperatureSensorModel::*;
        match self.config.model {
            Constant(_) => None,
            RandomInterval(params) => Some(normalize(self.temperature, params.min, params.max)),
            Linear(params) => Some(normalize(self.temperature, params.min, params.max)),
            Exponential(params) => Some(normalize(self.temperature, params.min, params.max)),
        }
    }

    pub fn use_day_params(&mut self, use_day_params: bool) {
        // Reset time (x) when switching parameter sets
        if self.using_day_params != use_day_params {
            self.time_base = Time::from_secs(0.0);
        }
        self.using_day_params = use_day_params;
    }

    /// Set the temperature model to constant, fixed to the provided temperature
    pub fn convert_to_constant_model(&mut self, temperature: Temperature) {
        self.config.model = TemperatureSensorModel::Constant(temperature);
        self.reset();
    }

    // NOTE: you shouldn't call this directly.
    // It's only pub so that the temperature_profile bin can use it.
    pub fn manual_step(&mut self, dt: Time, sun_visible: bool) {
        use TemperatureSensorModel::*;

        self.use_day_params(sun_visible);

        match &self.config.model {
            Constant(t) => {
                self.temperature = *t;
            }
            RandomInterval(params) => {
                let interval = params.interval(sun_visible).as_degrees_celsius().abs();
                let scale = (self.prng.rand_float() * 2.0) - 1.0; // [-1.0, 1.0)
                let dtemp = TemperatureInterval::from_degrees_celsius(interval * scale);
                let temp = self.temperature + dtemp;
                self.temperature = temp.clamp(params.min, params.max);
            }
            Linear(params) => {
                // Linear change in temperature
                let rate = params.rate(sun_visible);
                let dtemp = rate * dt;
                let temp = self.temperature + dtemp;
                self.temperature = temp.clamp(params.min, params.max);
            }
            Exponential(params) => {
                let exp = params.exponential(sun_visible);

                let y0 = exp.temp_at_t(self.time_base - dt);
                let y1 = exp.temp_at_t(self.time_base);
                let dtemp = y1 - y0;
                let temp = self.temperature + dtemp;
                self.temperature = temp.clamp(params.min, params.max);

                self.time_base += dt;
            }
        }
    }
}

impl TemperatureSensorModel {
    fn initial_temperature(&self) -> Temperature {
        use TemperatureSensorModel::*;
        match self {
            Constant(t) => *t,
            RandomInterval(params) => params.initial,
            Linear(params) => params.initial,
            Exponential(params) => params.initial,
        }
    }
}

/// Normalize to [-1, 1]
fn normalize(t: Temperature, min: Temperature, max: Temperature) -> Ratio {
    Ratio::from_f64(map_range(
        (min.as_degrees_celsius(), max.as_degrees_celsius()),
        (-1.0, 1.0),
        t.as_degrees_celsius(),
    ))
}

// https://rosettacode.org/wiki/Map_range#Rust
fn map_range(from_range: (f64, f64), to_range: (f64, f64), s: f64) -> f64 {
    to_range.0 + (s - from_range.0) * (to_range.1 - to_range.0) / (from_range.1 - from_range.0)
}

impl<'a> SimulationComponent<'a> for TemperatureSensor {
    type SharedState = SatelliteSharedState;
    type Environment = SatelliteEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, _common: &mut Self::SharedState) {
        let css = env.fsw_data.css.get(&0).expect("Missing CSS at index 0");
        self.use_day_params(css.valid);
    }

    fn step(&mut self, dt: Time, env: &'a Self::Environment, _common: &mut Self::SharedState) {
        let css = env.fsw_data.css.get(&0).expect("Missing CSS at index 0");
        self.manual_step(dt, css.valid);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_constant_model() {
        let config = TemperatureSensorConfig {
            model: TemperatureSensorModel::Constant(Temperature::from_degrees_celsius(21.0)),
        };
        let mut sensor = TemperatureSensor::new(config);

        // Initial temp
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 21.0);

        let dt = Time::from_secs(1.0);
        for i in 0..10 {
            sensor.manual_step(dt, i % 3 == 0);
            let temp = sensor.temperature();
            assert_relative_eq!(temp.as_degrees_celsius(), 21.0);
            assert!(sensor.normalized_temperature().is_none());
        }
    }

    #[test]
    fn test_rand_interval_model() {
        let config = TemperatureSensorConfig {
            model: TemperatureSensorModel::RandomInterval(
                TemperatureSensorRandomIntervalModelParams {
                    initial: Temperature::from_degrees_celsius(0.0),
                    day: TemperatureInterval::from_degrees_celsius(1.0),
                    night: TemperatureInterval::from_degrees_celsius(1.0),
                    min: Temperature::from_degrees_celsius(-10.0),
                    max: Temperature::from_degrees_celsius(10.0),
                },
            ),
        };
        let mut sensor = TemperatureSensor::new(config);

        // Initial temp
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 0.0);

        let dt = Time::from_secs(1.0);
        for i in 0..10 {
            sensor.manual_step(dt, i % 3 == 0);
            let _temp = sensor.temperature();
            assert!(sensor.normalized_temperature().is_some());
        }
    }

    #[test]
    fn test_linear_model() {
        let config = TemperatureSensorConfig {
            model: TemperatureSensorModel::Linear(TemperatureSensorLinearModelParams {
                initial: Temperature::from_degrees_celsius(0.0),
                day: TemperatureIntervalRate::from_degrees_celsius_per_second(1.0),
                night: TemperatureIntervalRate::from_degrees_celsius_per_second(-1.0),
                min: Temperature::from_degrees_celsius(-10.0),
                max: Temperature::from_degrees_celsius(10.0),
            }),
        };
        let mut sensor = TemperatureSensor::new(config);

        // Initial temp
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 0.0);
        assert_relative_eq!(sensor.normalized_temperature().unwrap().as_f64(), 0.0);

        let dt = Time::from_secs(1.0);

        // Increases while sun is visible
        for _ in 0..10 {
            sensor.manual_step(dt, true);
        }

        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 10.0);

        // Clamps max
        sensor.manual_step(dt, true);
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 10.0);
        assert_relative_eq!(sensor.normalized_temperature().unwrap().as_f64(), 1.0);

        // Decreases while sun is not visible
        for _ in 0..20 {
            sensor.manual_step(dt, false);
        }

        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), -10.0);

        // Clamps min
        sensor.manual_step(dt, false);
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), -10.0);
        assert_relative_eq!(sensor.normalized_temperature().unwrap().as_f64(), -1.0);

        // Resets
        sensor.reset();
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 0.0);

        // Switch model
        sensor.convert_to_constant_model(Temperature::from_degrees_celsius(5.0));
        sensor.manual_step(dt, false);
        assert_relative_eq!(sensor.temperature().as_degrees_celsius(), 5.0);
    }

    #[test]
    fn test_exponential_model() {
        let config = TemperatureSensorConfig {
            model: TemperatureSensorModel::Exponential(
                // https://www.desmos.com/calculator/aregvq5rhi
                TemperatureSensorExponentialModelParams {
                    initial: Temperature::from_degrees_celsius(2.2),
                    min: Temperature::from_degrees_celsius(-26.6),
                    max: Temperature::from_degrees_celsius(31.0),
                    // y\ =0.2e^{\left(0.5x\right)}+2
                    day: TemperatureSensorExponential {
                        base: std::f64::consts::E,
                        vertical_scaling_factor: 0.2,
                        horizontal_scaling_factor: 0.5,
                        offset: 2.0,
                    },
                    // y\ =-0.2e^{\left(0.5x\right)}+2
                    night: TemperatureSensorExponential {
                        base: std::f64::consts::E,
                        vertical_scaling_factor: -0.2,
                        horizontal_scaling_factor: 0.5,
                        offset: 2.0,
                    },
                },
            ),
        };
        let mut sensor = TemperatureSensor::new(config);

        // Initial temp
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 2.2);
        assert_relative_eq!(sensor.normalized_temperature().unwrap().as_f64(), 0.0);

        let dt = Time::from_secs(1.0);

        // Increases while sun is visible
        for _ in 0..20 {
            sensor.manual_step(dt, true);
        }

        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 31.0);

        // Clamps max
        sensor.manual_step(dt, true);
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 31.0);
        assert_relative_eq!(sensor.normalized_temperature().unwrap().as_f64(), 1.0);

        // Decreases while sun is not visible
        sensor.manual_step(dt, false);
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 30.921306131942526);
        for _ in 0..20 {
            sensor.manual_step(dt, false);
        }

        // Clamps min
        sensor.manual_step(dt, false);
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), -26.6);
        assert_relative_eq!(sensor.normalized_temperature().unwrap().as_f64(), -1.0);

        // Resets
        sensor.reset();
        let temp = sensor.temperature();
        assert_relative_eq!(temp.as_degrees_celsius(), 2.2);

        // Switch model
        sensor.convert_to_constant_model(Temperature::from_degrees_celsius(-5.0));
        sensor.manual_step(dt, false);
        assert_relative_eq!(sensor.temperature().as_degrees_celsius(), -5.0);
    }
}
