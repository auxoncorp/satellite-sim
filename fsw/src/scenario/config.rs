use crate::{
    point_failure,
    satellite::{comms, compute, imu, power, temperature_sensor, vision, SatCatId, SATELLITE_IDS},
    units::{
        Angle, ElectricCharge, ElectricCurrent, ElectricPotential, PotentialOverCharge, Ratio,
        Temperature, TemperatureInterval, TemperatureIntervalRate, Time,
    },
};
use serde::Deserialize;
use std::{collections::HashSet, fs, path::Path};

#[derive(Clone, PartialEq, Debug, Default, Deserialize)]
#[serde(default, rename_all = "kebab-case")]
pub struct Config {
    pub name: Option<String>,
    #[serde(alias = "temperature-sensor")]
    pub temperature_sensors: Vec<TemperatureSensor>,
    #[serde(alias = "point-failure")]
    pub point_failures: Vec<PointFailure>,
    #[serde(alias = "mutator")]
    pub mutators: Vec<Mutator>,
    #[serde(alias = "power-subsystem")]
    pub power_subsystems: Vec<PowerSubsystem>,
    #[serde(alias = "compute-subsystem")]
    pub compute_subsystems: Vec<ComputeSubsystem>,
    #[serde(alias = "comms-subsystem")]
    pub comms_subsystems: Vec<CommsSubsystem>,
    #[serde(alias = "vision-subsystem")]
    pub vision_subsystems: Vec<VisionSubsystem>,
    #[serde(alias = "imu-subsystem")]
    pub imu_subsystems: Vec<ImuSubsystem>,
    #[serde(alias = "satellite")]
    pub satellites: Vec<Satellite>,
    #[serde(alias = "relay-ground-station")]
    pub relay_ground_stations: Vec<RelayGroundStation>,
    #[serde(alias = "ir-event")]
    pub ir_events: Vec<IREvent>,
}

impl Config {
    pub fn load<P: AsRef<Path>>(path: P) -> Self {
        let content = fs::read_to_string(path).expect("Failed to read config file");
        Self::from_str_checked(&content)
    }

    pub fn from_str_checked(s: &str) -> Self {
        let cfg: Config = toml::from_str(s).expect("Failed to parse config file");

        let mut names = HashSet::new();
        for name in cfg.temperature_sensors.iter().map(|t| &t.name) {
            if !names.insert(name) {
                panic!("Duplicate configuration entry for temperature sensor '{name}'");
            }
        }

        names.clear();
        for name in cfg.point_failures.iter().map(|t| &t.name) {
            if !names.insert(name) {
                panic!("Duplicate configuration entry for point failure '{name}'");
            }
        }

        names.clear();
        for name in cfg.mutators.iter().map(|t| &t.name) {
            if !names.insert(name) {
                panic!("Duplicate configuration entry for mutator '{name}'");
            }
        }

        let mut ids = HashSet::new();
        for sat in cfg.satellites.iter() {
            let sat_idx = sat.sat_idx();
            let sat_name = SATELLITE_IDS[sat_idx].name;
            if !ids.insert(sat_idx) {
                panic!("Duplicate configuration entry for satellite '{sat_name}'");
            }
        }

        ids.clear();
        for rgs in cfg.relay_ground_stations.iter() {
            if !ids.insert(rgs.id as usize) {
                panic!(
                    "Duplicate configuration entry for relay ground station '{}'",
                    rgs.name
                );
            }
        }

        ids.clear();
        for ire in cfg.ir_events.iter() {
            if !ids.insert(ire.ground_truth_id as usize) {
                panic!(
                    "Duplicate configuration entry for IR event '{}'",
                    ire.ground_truth_id
                );
            }
        }

        cfg
    }

    pub(crate) fn satellite(&self, id: SatCatId) -> Option<&Satellite> {
        self.satellites.iter().find(|s| s.satcat_id() == id)
    }

    fn temperature_sensor(&self, name: &str) -> Option<&TemperatureSensor> {
        self.temperature_sensors.iter().find(|t| t.name == name)
    }

    fn point_failure(&self, name: &str) -> Option<&PointFailure> {
        self.point_failures.iter().find(|t| t.name == name)
    }

    fn mutator(&self, name: &str) -> Option<&Mutator> {
        self.mutators.iter().find(|t| t.name == name)
    }

    pub(crate) fn power_config(&self, id: SatCatId) -> Option<power::PowerConfig> {
        let sat = self.satellite(id)?;
        let cfg = sat.power.as_ref().map(|n| {
            self.power_subsystems
                .iter()
                .find(|p| &p.name == n)
                .expect("Missing power subsystem config")
        })?;
        let temperature_sensor_config = self
            .temperature_sensor(&cfg.temperature_sensor)
            .unwrap()
            .into();
        let fault_config = cfg
            .fault
            .as_ref()
            .map(|f| power::PowerFaultConfig {
                solar_panel_degraded: f.solar_panel_degraded.as_ref().map(|f| {
                    (
                        self.point_failure(&f.name).map(|fc| fc.into()).unwrap(),
                        ElectricCurrent::from_amps(f.value),
                    )
                }),
                battery_degraded: f.solar_panel_degraded.as_ref().map(|f| {
                    (
                        self.point_failure(&f.name).map(|fc| fc.into()).unwrap(),
                        PotentialOverCharge::from_volts_per_coulomb(f.value),
                    )
                }),
                watchdog_out_of_sync: f
                    .watchdog_out_of_sync
                    .as_ref()
                    .map(|m| self.mutator(m).map(|m| m.enabled).unwrap())
                    .unwrap_or(false),
            })
            .unwrap_or_default();
        Some(power::PowerConfig {
            battery_max_charge: ElectricCharge::from_amp_hours(cfg.battery_max_charge),
            battery_max_voltage: ElectricPotential::from_volts(cfg.battery_max_voltage),
            battery_discharge_factor: PotentialOverCharge::from_volts_per_coulomb(
                cfg.battery_discharge_factor,
            ),
            solar_panel_charge_rate: ElectricCurrent::from_amps(cfg.solar_panel_charge_rate),
            system_load: ElectricCurrent::from_amps(cfg.system_load),
            temperature_sensor_config,
            fault_config,
        })
    }

    pub(crate) fn compute_config(&self, id: SatCatId) -> Option<compute::ComputeConfig> {
        let sat = self.satellite(id)?;
        let cfg = sat.compute.as_ref().map(|n| {
            self.compute_subsystems
                .iter()
                .find(|p| &p.name == n)
                .expect("Missing imu subsystem config")
        })?;
        let temperature_sensor_config = self
            .temperature_sensor(&cfg.temperature_sensor)
            .unwrap()
            .into();
        let fault_config = cfg
            .fault
            .as_ref()
            .map(|f| compute::ComputeFaultConfig {
                watchdog_out_of_sync: f
                    .watchdog_out_of_sync
                    .as_ref()
                    .map(|m| self.mutator(m).map(|m| m.enabled).unwrap())
                    .unwrap_or(false),
            })
            .unwrap_or_default();
        Some(compute::ComputeConfig {
            telemetry_rate: Time::from_secs(cfg.telemetry_rate),
            collect_timeout: Time::from_secs(cfg.collect_timeout),
            temperature_sensor_config,
            fault_config,
        })
    }

    pub(crate) fn comms_config(&self, id: SatCatId) -> Option<comms::CommsConfig> {
        let sat = self.satellite(id)?;
        let cfg = sat.comms.as_ref().map(|n| {
            self.comms_subsystems
                .iter()
                .find(|p| &p.name == n)
                .expect("Missing comms subsystem config")
        })?;
        let temperature_sensor_config = self
            .temperature_sensor(&cfg.temperature_sensor)
            .unwrap()
            .into();
        let fault_config = cfg
            .fault
            .as_ref()
            .map(|f| comms::CommsFaultConfig {
                rng_seed: f.rng_seed.unwrap_or(0),
                gps_offline_rtc_drift: f.gps_offline_rtc_drift.as_ref().map(|f| {
                    (
                        self.point_failure(&f.name).map(|fc| fc.into()).unwrap(),
                        Ratio::from_f64(f.value),
                    )
                }),
                gps_offline: f
                    .gps_offline
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                ground_transceiver_failure: f
                    .ground_transceiver_failure
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                ground_transceiver_partial_failure: f
                    .ground_transceiver_partial_failure
                    .as_ref()
                    .map(|f| {
                        (
                            self.point_failure(&f.name).map(|fc| fc.into()).unwrap(),
                            Ratio::from_f64(f.value),
                        )
                    }),
                rtc_degraded: f
                    .rtc_degraded
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                watchdog_out_of_sync: f
                    .watchdog_out_of_sync
                    .as_ref()
                    .map(|m| self.mutator(m).map(|m| m.enabled).unwrap())
                    .unwrap_or(false),
            })
            .unwrap_or_default();
        Some(comms::CommsConfig {
            temperature_sensor_config,
            fault_config,
        })
    }

    pub(crate) fn vision_config(&self, id: SatCatId) -> Option<vision::VisionConfig> {
        let sat = self.satellite(id)?;
        let cfg = sat.vision.as_ref().map(|n| {
            self.vision_subsystems
                .iter()
                .find(|p| &p.name == n)
                .expect("Missing vision subsystem config")
        })?;
        let focus_camera_temperature_sensor_config = self
            .temperature_sensor(&cfg.focus_camera_temperature_sensor)
            .unwrap()
            .into();
        let scanner_camera_temperature_sensor_config = self
            .temperature_sensor(&cfg.scanner_camera_temperature_sensor)
            .unwrap()
            .into();
        let fault_config = cfg
            .fault
            .as_ref()
            .map(|f| vision::VisionFaultConfig {
                active_cooling: f
                    .active_cooling
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                scanner_camera_offline: f
                    .scanner_camera_offline
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                focus_camera_offline: f
                    .focus_camera_offline
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                focus_camera_gimbal: f
                    .focus_camera_gimbal
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                focus_camera_constant_temperature_after_reset: f
                    .focus_camera_constant_temperature_after_reset
                    .map(Temperature::from_degrees_celsius),
                scanner_camera_constant_temperature_after_reset: f
                    .scanner_camera_constant_temperature_after_reset
                    .map(Temperature::from_degrees_celsius),
                watchdog_out_of_sync: f
                    .watchdog_out_of_sync
                    .as_ref()
                    .map(|m| self.mutator(m).map(|m| m.enabled).unwrap())
                    .unwrap_or(false),
            })
            .unwrap_or_default();

        Some(vision::VisionConfig {
            scanner_field_of_view_angle: Angle::from_degrees(cfg.scanner_field_of_view),
            focus_field_of_view_angle: Angle::from_degrees(cfg.focus_field_of_view),
            update_interval: Time::from_secs(cfg.update_interval),
            focus_camera_temperature_sensor_config,
            scanner_camera_temperature_sensor_config,
            focus_camera_disabled: cfg.focus_camera_disabled.unwrap_or(false),
            fault_config,
        })
    }

    pub(crate) fn imu_config(&self, id: SatCatId) -> Option<imu::ImuConfig> {
        let sat = self.satellite(id)?;
        let cfg = sat.imu.as_ref().map(|n| {
            self.imu_subsystems
                .iter()
                .find(|p| &p.name == n)
                .expect("Missing imu subsystem config")
        })?;
        let temperature_sensor_config = self
            .temperature_sensor(&cfg.temperature_sensor)
            .unwrap()
            .into();
        let fault_config = cfg
            .fault
            .as_ref()
            .map(|f| imu::ImuFaultConfig {
                degraded_state: f
                    .degraded_state
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                data_inconsistency: f
                    .data_inconsistency
                    .as_ref()
                    .map(|f| self.point_failure(f).map(|fc| fc.into()).unwrap()),
                constant_temperature_after_reset: f
                    .constant_temperature_after_reset
                    .map(Temperature::from_degrees_celsius),
                watchdog_out_of_sync: f
                    .watchdog_out_of_sync
                    .as_ref()
                    .map(|m| self.mutator(m).map(|m| m.enabled).unwrap())
                    .unwrap_or(false),
            })
            .unwrap_or_default();

        Some(imu::ImuConfig {
            temperature_sensor_config,
            fault_config,
        })
    }
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct Satellite {
    pub id: toml::Value,
    pub power: Option<String>,
    pub compute: Option<String>,
    pub comms: Option<String>,
    pub vision: Option<String>,
    pub imu: Option<String>,
}

impl Satellite {
    pub(crate) fn sat_idx(&self) -> usize {
        for (idx, sat) in SATELLITE_IDS.iter().enumerate() {
            if let Some(s) = self.id.as_str() {
                if s == sat.name {
                    return idx;
                }
            } else if let Some(n) = self.id.as_integer() {
                if n as u64 == u64::from(sat.satcat_id) {
                    return idx;
                }
            } else {
                panic!(
                    "Invalid satellite configuration, id must be either a string name or satcat ID"
                );
            }
        }
        panic!("Invalid satellite configuration, id must be a value string name or satcat ID");
    }

    pub(crate) fn satcat_id(&self) -> SatCatId {
        SATELLITE_IDS[self.sat_idx()].satcat_id
    }
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct PowerSubsystem {
    pub name: String,
    pub battery_max_charge: f64,
    pub battery_max_voltage: f64,
    pub battery_discharge_factor: f64,
    pub solar_panel_charge_rate: f64,
    pub system_load: f64,
    pub temperature_sensor: String,
    pub fault: Option<PowerFault>,
}

#[derive(Clone, PartialEq, Debug, Default, Deserialize)]
#[serde(default, rename_all = "kebab-case")]
pub struct PowerFault {
    pub watchdog_out_of_sync: Option<String>,
    pub solar_panel_degraded: Option<PointFailureActivatedValue>,
    pub battery_degraded: Option<PointFailureActivatedValue>,
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct ComputeSubsystem {
    pub name: String,
    pub telemetry_rate: f64,
    pub collect_timeout: f64,
    pub temperature_sensor: String,
    pub fault: Option<ComputeFault>,
}

#[derive(Clone, PartialEq, Debug, Default, Deserialize)]
#[serde(default, rename_all = "kebab-case")]
pub struct ComputeFault {
    pub watchdog_out_of_sync: Option<String>,
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct CommsSubsystem {
    pub name: String,
    pub temperature_sensor: String,
    pub fault: Option<CommsFault>,
}

#[derive(Clone, PartialEq, Debug, Default, Deserialize)]
#[serde(default, rename_all = "kebab-case")]
pub struct CommsFault {
    pub rng_seed: Option<u32>,
    pub gps_offline: Option<String>,
    pub rtc_degraded: Option<String>,
    pub watchdog_out_of_sync: Option<String>,
    pub ground_transceiver_failure: Option<String>,
    pub gps_offline_rtc_drift: Option<PointFailureActivatedValue>,
    pub ground_transceiver_partial_failure: Option<PointFailureActivatedValue>,
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct VisionSubsystem {
    pub name: String,
    pub scanner_field_of_view: f64,
    pub focus_field_of_view: f64,
    pub update_interval: f64,
    pub focus_camera_temperature_sensor: String,
    pub scanner_camera_temperature_sensor: String,
    pub focus_camera_disabled: Option<bool>,
    pub fault: Option<VisionFault>,
}

#[derive(Clone, PartialEq, Debug, Default, Deserialize)]
#[serde(default, rename_all = "kebab-case")]
pub struct VisionFault {
    pub active_cooling: Option<String>,
    pub scanner_camera_offline: Option<String>,
    pub focus_camera_offline: Option<String>,
    pub focus_camera_gimbal: Option<String>,
    pub focus_camera_constant_temperature_after_reset: Option<f64>,
    pub scanner_camera_constant_temperature_after_reset: Option<f64>,
    pub watchdog_out_of_sync: Option<String>,
    pub watchdog_out_of_sync_recurring: Option<bool>,
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct ImuSubsystem {
    pub name: String,
    pub temperature_sensor: String,
    pub fault: Option<ImuFault>,
}

#[derive(Clone, PartialEq, Debug, Default, Deserialize)]
#[serde(default, rename_all = "kebab-case")]
pub struct ImuFault {
    pub degraded_state: Option<String>,
    pub data_inconsistency: Option<String>,
    pub constant_temperature_after_reset: Option<f64>,
    pub watchdog_out_of_sync: Option<String>,
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct TemperatureSensor {
    pub name: String,
    pub model: TemperatureSensorModel,
    pub temperature: Option<f64>,
    pub initial: Option<f64>,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub day: Option<toml::Value>,
    pub night: Option<toml::Value>,
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TemperatureSensorModel {
    Constant,
    RandomInterval,
    Linear,
    Exponential,
}

impl From<&TemperatureSensor> for temperature_sensor::TemperatureSensorConfig {
    fn from(value: &TemperatureSensor) -> Self {
        let get_fields = || {
            Some((
                value.initial?,
                value.min?,
                value.max?,
                value.day.clone()?,
                value.night.clone()?,
            ))
        };

        let model = match value.model {
            TemperatureSensorModel::Constant => {
                temperature_sensor::TemperatureSensorModel::Constant(
                    Temperature::from_degrees_celsius(value.temperature.expect(
                        "temperature-sensor config constant model requires the 'temperature' field",
                    )),
                )
            }
            TemperatureSensorModel::RandomInterval => {
                let (initial, min, max, day, night) = get_fields().expect(
                    "temperature-sensor config random-interval model requires the 'initial', 'min', 'max', 'day', and 'night' fields");
                let day = day.as_float().unwrap();
                let night = night.as_float().unwrap();
                let params = temperature_sensor::TemperatureSensorRandomIntervalModelParams {
                    initial: Temperature::from_degrees_celsius(initial),
                    min: Temperature::from_degrees_celsius(min),
                    max: Temperature::from_degrees_celsius(max),
                    day: TemperatureInterval::from_degrees_celsius(day),
                    night: TemperatureInterval::from_degrees_celsius(night),
                };
                temperature_sensor::TemperatureSensorModel::RandomInterval(params)
            }
            TemperatureSensorModel::Linear => {
                let (initial, min, max, day, night) = get_fields().expect(
                    "temperature-sensor config linear model requires the 'initial', 'min', 'max', 'day', and 'night' fields");
                let day = day.as_float().unwrap();
                let night = night.as_float().unwrap();
                let params = temperature_sensor::TemperatureSensorLinearModelParams {
                    initial: Temperature::from_degrees_celsius(initial),
                    min: Temperature::from_degrees_celsius(min),
                    max: Temperature::from_degrees_celsius(max),
                    day: TemperatureIntervalRate::from_degrees_celsius_per_second(day),
                    night: TemperatureIntervalRate::from_degrees_celsius_per_second(night),
                };
                temperature_sensor::TemperatureSensorModel::Linear(params)
            }
            TemperatureSensorModel::Exponential => {
                let (initial, min, max, day, night) = get_fields().expect(
                    "temperature-sensor config linear model requires the 'initial', 'min', 'max', 'day', and 'night' fields");
                let day = temp_senor_exp_from_str(day.as_str().unwrap());
                let night = temp_senor_exp_from_str(night.as_str().unwrap());
                let params = temperature_sensor::TemperatureSensorExponentialModelParams {
                    initial: Temperature::from_degrees_celsius(initial),
                    min: Temperature::from_degrees_celsius(min),
                    max: Temperature::from_degrees_celsius(max),
                    day,
                    night,
                };
                temperature_sensor::TemperatureSensorModel::Exponential(params)
            }
        };
        temperature_sensor::TemperatureSensorConfig { model }
    }
}

impl From<TemperatureSensor> for temperature_sensor::TemperatureSensorConfig {
    fn from(value: TemperatureSensor) -> Self {
        temperature_sensor::TemperatureSensorConfig::from(&value)
    }
}

fn temp_senor_exp_from_str(val: &str) -> temperature_sensor::TemperatureSensorExponential {
    let params: Vec<&str> = val.trim().split(' ').collect();
    if params.len() != 4 {
        panic!("temperature-sensor config exponential model uses 'base vertical-scaling-factor horizontal-scaling-factor offset' strings for the day and night fields");
    }
    let base = match params[0].trim() {
        // Support some common constants
        "E" | "e" => std::f64::consts::E,
        "PI" | "pi" => std::f64::consts::PI,
        b => b.parse::<f64>().unwrap(),
    };
    let vertical_scaling_factor = params[1].trim().parse::<f64>().unwrap();
    let horizontal_scaling_factor = params[2].trim().parse::<f64>().unwrap();
    let offset = params[3].trim().parse::<f64>().unwrap();
    temperature_sensor::TemperatureSensorExponential {
        base,
        vertical_scaling_factor,
        horizontal_scaling_factor,
        offset,
    }
}

#[derive(Clone, PartialEq, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct PointFailureActivatedValue {
    pub name: String,
    pub value: f64,
}

#[derive(Clone, PartialEq, PartialOrd, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct PointFailure {
    pub name: String,
    pub threshold: String,
    pub hold_period: f64,
}

macro_rules! pf_from_impl {
    ($unit_type:ty, $unit_from:ident) => {
        impl From<&PointFailure> for point_failure::PointFailureConfig<$unit_type> {
            fn from(pf: &PointFailure) -> Self {
                const LE: &str = "<=";
                const GE: &str = ">=";
                assert!(
                    !pf.name.is_empty(),
                    "point-failure config name cannot be empty"
                );
                let threshold = if pf.threshold.starts_with(GE) {
                    let t = pf
                        .threshold
                        .trim_start_matches(GE)
                        .trim()
                        .parse::<f64>()
                        .unwrap()
                        .into();
                    point_failure::PointFailureThresholdOperator::GreaterThanEqual(
                        <$unit_type>::$unit_from(t),
                    )
                } else if pf.threshold.starts_with(LE) {
                    let t = pf
                        .threshold
                        .trim_start_matches(LE)
                        .trim()
                        .parse::<f64>()
                        .unwrap()
                        .into();
                    point_failure::PointFailureThresholdOperator::LessThanEqual(
                        <$unit_type>::$unit_from(t),
                    )
                } else {
                    panic!("point-failure config '{}' threshold is invalid", pf.name);
                };
                point_failure::PointFailureConfig {
                    threshold,
                    hold_period: Time::from_secs(pf.hold_period),
                }
            }
        }
    };
}
pf_from_impl!(Temperature, from_degrees_celsius);
pf_from_impl!(Time, from_secs);
pf_from_impl!(ElectricPotential, from_volts);

#[derive(Clone, PartialEq, PartialOrd, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct Mutator {
    pub name: String,
    pub enabled: bool,
}

#[derive(Clone, PartialEq, PartialOrd, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct RelayGroundStation {
    pub name: String,
    pub id: u32,
    pub latitude: f64,
    pub longitude: f64,
    pub enable_time_sync: bool,
    pub rtc_drift: f64,
}

#[derive(Clone, PartialEq, PartialOrd, Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub struct IREvent {
    pub ground_truth_id: u32,
    pub activate_at: f64,
    pub deactivate_at: Option<f64>,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub velocity_east: f64,
    pub velocity_north: f64,
    pub intensity: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use indoc::indoc;

    const FULL_CONFIG_TOML: &str = indoc! {r#"
        name = 'my scenario'

        [[temperature-sensor]]
        name = 'ts0'
        model = 'constant'
        temperature = 24.2

        [[temperature-sensor]]
        name = 'ts1'
        model = 'random-interval'
        initial = 40.0
        min = 10.0
        max = 100.0
        day = 2.0
        night = -2.0

        [[point-failure]]
        name = 'pf0'
        threshold = '>= 60.0'
        hold-period = 60.0

        [[point-failure]]
        name = 'pf1'
        threshold = '>= 10.0'
        hold-period = 20.0

        [[mutator]]
        name = 'm0'
        enabled = true

        [[power-subsystem]]
        name = 'power0'
        battery-max-charge = 2.2
        battery-max-voltage = 4.4
        battery-discharge-factor = 5.5
        solar-panel-charge-rate = 6.6
        system-load = 5.3
        temperature-sensor = 'ts1'
            [power-subsystem.fault]
            watchdog-out-of-sync = 'm0'
                [power-subsystem.fault.solar-panel-degraded]
                name = 'pf1'
                value = 2.1
                [power-subsystem.fault.battery-degraded]
                name = 'pf0'
                value = 1.1

        [[compute-subsystem]]
        name = 'compute0'
        telemetry-rate = 4.2
        collect-timeout = 4.8
        temperature-sensor = 'ts1'
            [compute-subsystem.fault]
            watchdog-out-of-sync = 'm0'

        [[comms-subsystem]]
        name = 'comms0'
        temperature-sensor = 'ts0'
            [comms-subsystem.fault]
            rng-seed = 11
            gps-offline = 'pf1'
            rtc-degraded = 'pf1'
            watchdog-out-of-sync = 'm0'
            ground-transceiver-failure = 'pf0'
                [comms-subsystem.fault.ground-transceiver-partial-failure]
                name = 'pf1'
                value = 2.1
                [comms-subsystem.fault.gps-offline-rtc-drift]
                name = 'pf0'
                value = 1.1

        [[vision-subsystem]]
        name = 'vision0'
        scanner-field-of-view = 20.0
        focus-field-of-view = 20.0
        update-interval = 2.0
        focus-camera-temperature-sensor = 'ts0'
        scanner-camera-temperature-sensor = 'ts1'
        focus-camera-disabled = false
            [vision-subsystem.fault]
            active-cooling = 'pf0'
            scanner-camera-offline = 'pf1'
            focus-camera-offline = 'pf1'
            focus-camera-gimbal = 'pf1'
            focus-camera-constant-temperature-after-reset = 22.0
            scanner-camera-constant-temperature-after-reset = 33.0
            watchdog-out-of-sync = 'm0'

        [[imu-subsystem]]
        name = 'imu0'
        temperature-sensor = 'ts0'
            [imu-subsystem.fault]
            degraded-state = 'pf0'
            data-inconsistency = 'pf1'
            constant-temperature-after-reset = 33.0
            watchdog-out-of-sync = 'm0'

        [[satellite]]
        id = 'GALAXY-1'
        power = 'power0'
        compute = 'compute0'
        comms = 'comms0'
        vision = 'vision0'
        imu = 'imu0'

        [[relay-ground-station]]
        id = 1
        name = 'RGS'
        latitude = 0.23
        longitude = 0.44
        enable-time-sync = true
        rtc-drift = 0.02

        [[ir-event]]
        ground-truth-id = 1
        activate-at = 10.1
        deactivate-at = 100.0
        latitude = 0.23
        longitude = 0.44
        altitude = 100.0
        velocity-east = 100.0
        velocity-north = -100.0
        intensity = 10.0
    "#};

    #[test]
    fn full_config() {
        let cfg = Config::from_str_checked(FULL_CONFIG_TOML);
        dbg!(&cfg);
        assert_eq!(cfg.name.as_deref(), Some("my scenario"));
        assert_eq!(cfg.temperature_sensors.len(), 2);
        assert_eq!(cfg.point_failures.len(), 2);
        assert_eq!(cfg.mutators.len(), 1);
        assert_eq!(cfg.satellites.len(), 1);
        assert_eq!(cfg.power_subsystems.len(), 1);
        assert_eq!(cfg.compute_subsystems.len(), 1);
        assert_eq!(cfg.comms_subsystems.len(), 1);
        assert_eq!(cfg.vision_subsystems.len(), 1);
        assert_eq!(cfg.imu_subsystems.len(), 1);
        assert_eq!(cfg.relay_ground_stations.len(), 1);
        assert_eq!(cfg.ir_events.len(), 1);

        let id = SatCatId::from(45026);
        assert_eq!(cfg.satellites[0].satcat_id(), id);
        assert!(cfg.satellite(id).is_some());
        assert!(cfg.power_config(id).is_some());
        assert!(cfg.compute_config(id).is_some());
        assert!(cfg.comms_config(id).is_some());
        assert!(cfg.vision_config(id).is_some());
        assert!(cfg.imu_config(id).is_some());
    }

    #[test]
    fn temp_sensor_constant() {
        const TOML: &str = indoc! {r#"
            name = 't c'
            model = 'constant'
            temperature = 1.2
        "#};
        let ts: TemperatureSensor = toml::from_str(TOML).unwrap();
        assert_eq!(ts.name.as_str(), "t c");
        let tsc: temperature_sensor::TemperatureSensorConfig = ts.into();
        if let temperature_sensor::TemperatureSensorModel::Constant(t) = tsc.model {
            assert_relative_eq!(t.as_degrees_celsius(), 1.2);
        } else {
            panic!();
        }
    }

    #[test]
    fn temp_sensor_rand_ival() {
        const TOML: &str = indoc! {r#"
            name = 't r i'
            model = 'random-interval'
            initial = 10.0
            min = 0.0
            max = 100.0
            day = 1.0
            night = -1.0
        "#};
        let ts: TemperatureSensor = toml::from_str(TOML).unwrap();
        assert_eq!(ts.name.as_str(), "t r i");
        let tsc: temperature_sensor::TemperatureSensorConfig = ts.into();
        if let temperature_sensor::TemperatureSensorModel::RandomInterval(p) = tsc.model {
            assert_relative_eq!(p.initial.as_degrees_celsius(), 10.0);
            assert_relative_eq!(p.min.as_degrees_celsius(), 0.0);
            assert_relative_eq!(p.max.as_degrees_celsius(), 100.0);
            assert_relative_eq!(p.day.as_degrees_celsius(), 1.0);
            assert_relative_eq!(p.night.as_degrees_celsius(), -1.0);
        } else {
            panic!();
        }
    }

    #[test]
    fn temp_sensor_linear() {
        const TOML: &str = indoc! {r#"
            name = 't l'
            model = 'linear'
            initial = 10.0
            min = 0.0
            max = 100.0
            day = 1.0
            night = -1.0
        "#};
        let ts: TemperatureSensor = toml::from_str(TOML).unwrap();
        assert_eq!(ts.name.as_str(), "t l");
        let tsc: temperature_sensor::TemperatureSensorConfig = ts.into();
        if let temperature_sensor::TemperatureSensorModel::Linear(p) = tsc.model {
            assert_relative_eq!(p.initial.as_degrees_celsius(), 10.0);
            assert_relative_eq!(p.min.as_degrees_celsius(), 0.0);
            assert_relative_eq!(p.max.as_degrees_celsius(), 100.0);
            assert_relative_eq!(p.day.as_degrees_celsius_per_second(), 1.0);
            assert_relative_eq!(p.night.as_degrees_celsius_per_second(), -1.0);
        } else {
            panic!();
        }
    }

    #[test]
    fn temp_sensor_exp() {
        const TOML: &str = indoc! {r#"
            name = 't e'
            model = 'exponential'
            initial = 10.0
            min = 0.0
            max = 100.0
            day = 'E 0.2 0.5 2.0'
            night = 'pi -0.2 0.6 4.0'
        "#};
        let ts: TemperatureSensor = toml::from_str(TOML).unwrap();
        assert_eq!(ts.name.as_str(), "t e");
        let tsc: temperature_sensor::TemperatureSensorConfig = ts.into();
        if let temperature_sensor::TemperatureSensorModel::Exponential(p) = tsc.model {
            assert_relative_eq!(p.initial.as_degrees_celsius(), 10.0);
            assert_relative_eq!(p.min.as_degrees_celsius(), 0.0);
            assert_relative_eq!(p.max.as_degrees_celsius(), 100.0);
            assert_relative_eq!(p.day.base, std::f64::consts::E);
            assert_relative_eq!(p.day.vertical_scaling_factor, 0.2);
            assert_relative_eq!(p.day.horizontal_scaling_factor, 0.5);
            assert_relative_eq!(p.day.offset, 2.0);
            assert_relative_eq!(p.night.base, std::f64::consts::PI);
            assert_relative_eq!(p.night.vertical_scaling_factor, -0.2);
            assert_relative_eq!(p.night.horizontal_scaling_factor, 0.6);
            assert_relative_eq!(p.night.offset, 4.0);
        } else {
            panic!();
        }
    }

    #[test]
    fn point_failure() {
        const TOML: &str = indoc! {r#"
            name = 'foo bar'
            threshold = '>= 60.0'
            hold-period = 10.0
        "#};
        let pf: PointFailure = toml::from_str(TOML).unwrap();
        assert_eq!(pf.name.as_str(), "foo bar");
        let pfc: point_failure::PointFailureConfig<Temperature> = (&pf).into();
        assert_relative_eq!(pfc.hold_period.as_secs(), 10.0);
        if let point_failure::PointFailureThresholdOperator::GreaterThanEqual(t) = pfc.threshold {
            assert_relative_eq!(t.as_degrees_celsius(), 60.0);
        } else {
            panic!();
        }
    }
}
