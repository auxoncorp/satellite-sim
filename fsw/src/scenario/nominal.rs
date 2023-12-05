use oorandom::Rand64;
use types42::prelude::SpacecraftIndex;

use crate::{
    satellite::{
        CommsConfig, ComputeConfig, ImuConfig, PowerConfig, SatelliteConfig,
        TemperatureSensorConfig, TemperatureSensorExponential,
        TemperatureSensorExponentialModelParams, TemperatureSensorLinearModelParams,
        TemperatureSensorModel, VisionConfig, SATELLITE_IDS,
    },
    units::{
        Angle, ElectricCharge, ElectricCurrent, ElectricPotential, Temperature,
        TemperatureIntervalRate, Time,
    },
};

pub fn satellite_config(spacecraft_index: SpacecraftIndex) -> SatelliteConfig {
    // So that each satellite has a slightly different initial condition
    let mut prng = Rand64::new(spacecraft_index.into());

    let id = &SATELLITE_IDS[spacecraft_index as usize];

    SatelliteConfig {
        id,
        power_config: PowerConfig {
            battery_max_charge: ElectricCharge::from_amp_hours(40.0),
            battery_max_voltage: ElectricPotential::from_volts(13.1),
            battery_discharge_factor: ElectricPotential::from_volts(2.0)
                / ElectricCharge::from_amp_hours(40.0),
            solar_panel_charge_rate: if id.is_goes() {
                ElectricCurrent::from_milliamps(1200.0 - (prng.rand_float() * 10.0))
            } else {
                ElectricCurrent::from_milliamps(1200.0)
            },
            system_load: if id.is_goes() {
                ElectricCurrent::from_milliamps(600.0 - (prng.rand_float() * 10.0))
            } else {
                ElectricCurrent::from_milliamps(600.0)
            },
            temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::Linear(TemperatureSensorLinearModelParams {
                    initial: Temperature::from_degrees_celsius(prng.rand_float() * 5.0),
                    min: Temperature::from_degrees_celsius(-100.0),
                    max: Temperature::from_degrees_celsius(100.0),
                    day: TemperatureIntervalRate::from_degrees_celsius_per_second(0.0005),
                    night: TemperatureIntervalRate::from_degrees_celsius_per_second(-0.0005),
                }),
            },
            fault_config: Default::default(),
        },
        compute_config: ComputeConfig {
            telemetry_rate: Time::from_secs(1.0),
            collect_timeout: Time::from_millis(500.0),
            temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::Linear(TemperatureSensorLinearModelParams {
                    initial: Temperature::from_degrees_celsius(prng.rand_float() * 5.0),
                    min: Temperature::from_degrees_celsius(-100.0),
                    max: Temperature::from_degrees_celsius(100.0),
                    day: TemperatureIntervalRate::from_degrees_celsius_per_second(0.0001),
                    night: TemperatureIntervalRate::from_degrees_celsius_per_second(-0.0001),
                }),
            },
            fault_config: None,
        },
        comms_config: CommsConfig {
            temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::Linear(TemperatureSensorLinearModelParams {
                    initial: Temperature::from_degrees_celsius(prng.rand_float() * 5.0),
                    min: Temperature::from_degrees_celsius(-100.0),
                    max: Temperature::from_degrees_celsius(100.0),
                    day: TemperatureIntervalRate::from_degrees_celsius_per_second(0.0001),
                    night: TemperatureIntervalRate::from_degrees_celsius_per_second(-0.0001),
                }),
            },
            fault_config: Default::default(),
        },
        vision_config: VisionConfig {
            scanner_field_of_view_angle: if id.is_goes() {
                Angle::from_degrees(6.0)
            } else {
                Angle::from_degrees(12.0)
            },
            focus_field_of_view_angle: Angle::from_degrees(2.0),
            update_interval: Time::from_secs(1.0),
            scanner_camera_temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::Exponential(
                    // https://www.desmos.com/calculator/4xqzocrefv
                    TemperatureSensorExponentialModelParams {
                        initial: Temperature::from_degrees_celsius(prng.rand_float() * 5.0),
                        min: Temperature::from_degrees_celsius(-100.0),
                        max: Temperature::from_degrees_celsius(100.0),
                        // y\ =0.1e^{\left(0.0001x\right)}+0
                        day: TemperatureSensorExponential {
                            base: std::f64::consts::E,
                            vertical_scaling_factor: 0.1,
                            horizontal_scaling_factor: 0.0001,
                            offset: 0.0,
                        },
                        // y\ =-0.1e^{\left(0.0001x\right)}+0
                        night: TemperatureSensorExponential {
                            base: std::f64::consts::E,
                            vertical_scaling_factor: -0.1,
                            horizontal_scaling_factor: 0.0001,
                            offset: 0.0,
                        },
                    },
                ),
            },
            focus_camera_temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::Exponential(
                    // https://www.desmos.com/calculator/4xqzocrefv
                    TemperatureSensorExponentialModelParams {
                        initial: Temperature::from_degrees_celsius(prng.rand_float() * 5.0),
                        min: Temperature::from_degrees_celsius(-100.0),
                        max: Temperature::from_degrees_celsius(100.0),
                        // y\ =0.1e^{\left(0.0001x\right)}+0
                        day: TemperatureSensorExponential {
                            base: std::f64::consts::E,
                            vertical_scaling_factor: 0.1,
                            horizontal_scaling_factor: 0.0001,
                            offset: 0.0,
                        },
                        // y\ =-0.1e^{\left(0.0001x\right)}+0
                        night: TemperatureSensorExponential {
                            base: std::f64::consts::E,
                            vertical_scaling_factor: -0.1,
                            horizontal_scaling_factor: 0.0001,
                            offset: 0.0,
                        },
                    },
                ),
            },
            focus_camera_disabled: id.is_goes(),
            fault_config: None,
        },
        imu_config: ImuConfig {
            temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::Linear(TemperatureSensorLinearModelParams {
                    initial: Temperature::from_degrees_celsius(prng.rand_float() * 5.0),
                    min: Temperature::from_degrees_celsius(-100.0),
                    max: Temperature::from_degrees_celsius(100.0),
                    day: TemperatureIntervalRate::from_degrees_celsius_per_second(0.0005),
                    night: TemperatureIntervalRate::from_degrees_celsius_per_second(-0.0005),
                }),
            },
            fault_config: Default::default(),
        },
    }
}
