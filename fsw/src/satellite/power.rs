use modality_api::{AttrType, TimelineId};
use modality_mutator_protocol::descriptor::owned::*;
use oorandom::Rand64;
use serde::Serialize;
use tracing::warn;

use crate::{
    channel::{Receiver, Sender, TracedMessage},
    event,
    modality::{kv, AttrsBuilder, MODALITY},
    mutator::{
        constant_temperature_descriptor, watchdog_out_of_sync_descriptor, GenericBooleanMutator,
        GenericSetFloatMutator, MutatorActuatorDescriptor,
    },
    point_failure::{PointFailure, PointFailureConfig},
    recv,
    satellite::temperature_sensor::{
        TemperatureSensor, TemperatureSensorConfig, TemperatureSensorModel,
        TemperatureSensorRandomIntervalModelParams,
    },
    satellite::{SatelliteEnvironment, SatelliteId, SatelliteSharedState},
    try_send,
    units::{
        ElectricCharge, ElectricCurrent, ElectricPotential, PotentialOverCharge, Ratio,
        Temperature, TemperatureInterval, Time,
    },
    SimulationComponent,
};

/// The power subsystem models a battery that discharges over time,
/// along with a solar panel that charges it whenever it's in the
/// sunlight.
pub struct PowerSubsystem {
    config: PowerConfig,
    cmd_rx: Receiver<PowerCommand>,
    res_tx: Sender<PowerResponse>,
    timeline: TimelineId,

    battery_charge: ElectricCharge,
    battery_voltage: ElectricPotential,
    temp_sensor: TemperatureSensor,
    error_register: PowerErrorRegister,

    solar_panel_degraded: Option<GenericSetFloatMutator>,
    battery_degraded: Option<(PointFailure<Temperature>, PotentialOverCharge)>,
    watchdog_out_of_sync: Option<GenericBooleanMutator>,
    constant_temperature: Option<GenericSetFloatMutator>,
}

impl std::fmt::Debug for PowerSubsystem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PowerSubsystem")
            .field("config", &self.config)
            .field("battery_charge", &self.battery_charge)
            .field("battery_voltage", &self.battery_voltage)
            .finish()
    }
}

impl PowerSubsystem {
    pub const COMPONENT_NAME: &'static str = "power_subsystem";

    pub fn new(config: PowerConfig, rx: Receiver<PowerCommand>, tx: Sender<PowerResponse>) -> Self {
        assert!(
            config.battery_initial_charge.as_amp_hours()
                <= config.battery_max_charge.as_amp_hours()
        );
        let battery_charge = config.battery_initial_charge;
        let battery_voltage = config.battery_max_voltage;
        let timeline = TimelineId::allocate();
        let temp_sensor = TemperatureSensor::new(config.temperature_sensor_config);
        let battery_degraded = config
            .fault_config
            .battery_degraded
            .as_ref()
            .map(|(pfc, val)| (PointFailure::new(pfc.clone()), *val));

        if let Some(bdf) = config
            .fault_config
            .battery_degraded
            .as_ref()
            .map(|(_, bdf)| *bdf)
        {
            assert!(
                bdf > config.battery_discharge_factor,
                "Battery degraded fault battery discharge factor should be greater than the config amount");
        }

        Self {
            cmd_rx: rx,
            res_tx: tx,
            config,
            timeline,
            battery_charge,
            battery_voltage,
            temp_sensor,
            error_register: PowerErrorRegister { out_of_sync: false },
            battery_degraded,
            // Mutators are initialized in init_fault_models at sim-init time
            solar_panel_degraded: None,
            watchdog_out_of_sync: None,
            constant_temperature: None,
        }
    }

    pub fn has_error_bits_set(&self) -> bool {
        self.error_register.out_of_sync
    }

    fn solar_panel_charge_rate(&self) -> ElectricCurrent {
        if let Some(active_mutation) = self
            .solar_panel_degraded
            .as_ref()
            .and_then(|m| m.active_mutation())
        {
            return ElectricCurrent::from_amps(active_mutation);
        }
        self.config.solar_panel_charge_rate
    }

    fn battery_discharge_factor(&self) -> PotentialOverCharge {
        if let Some((pf, value_when_active)) = self.battery_degraded.as_ref() {
            if pf.is_active() {
                return *value_when_active;
            }
        }
        self.config.battery_discharge_factor
    }

    fn init_fault_models(&mut self, id: &SatelliteId) {
        let base_ctx = [
            ("satellite_name", id.name),
            ("component_name", Self::COMPONENT_NAME),
        ];

        if let Some((pf, _)) = self.battery_degraded.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "battery_degraded");
        }

        // Construct mutators at sim-init time so we have access to SatelliteId/env details
        // to qualify the mutator descriptors
        self.solar_panel_degraded =
            self.config
                .fault_config
                .solar_panel_degraded
                .then_some(GenericSetFloatMutator::new(OwnedMutatorDescriptor {
                    name: format!("{} solar panel degraded charge rate", Self::COMPONENT_NAME)
                        .into(),
                    description: "Sets the solar panel charge rate".to_owned().into(),
                    layer: MutatorLayer::Implementational.into(),
                    group: Self::COMPONENT_NAME.to_owned().into(),
                    operation: MutatorOperation::SetToValue.into(),
                    statefulness: MutatorStatefulness::Permanent.into(),
                    organization_custom_metadata: OrganizationCustomMetadata::new(
                        "satellite".to_string(),
                        [
                            ("id".to_string(), id.satcat_id.into()),
                            ("name".to_string(), id.name.into()),
                            ("component_name".to_string(), Self::COMPONENT_NAME.into()),
                        ]
                        .into_iter()
                        .collect(),
                    ),
                    params: vec![OwnedMutatorParamDescriptor::new(
                        AttrType::Float,
                        "electric_current".to_owned(),
                    )
                    .unwrap()
                    .with_description("Solar panel charge rate electric current [amps]")
                    .with_value_min(0.0)
                    .with_value_max(self.config.solar_panel_charge_rate.as_amps())
                    .with_least_effect_value(self.config.solar_panel_charge_rate.as_amps())],
                }));

        if let Some(m) = &self.solar_panel_degraded {
            MODALITY.register_mutator(m);
        }

        self.watchdog_out_of_sync =
            self.config
                .fault_config
                .watchdog_out_of_sync
                .then_some(GenericBooleanMutator::new(watchdog_out_of_sync_descriptor(
                    Self::COMPONENT_NAME,
                    id,
                )));

        if let Some(m) = &self.watchdog_out_of_sync {
            MODALITY.register_mutator(m);
        }

        self.constant_temperature =
            self.config
                .fault_config
                .constant_temperature
                .then_some(GenericSetFloatMutator::new(
                    constant_temperature_descriptor(Self::COMPONENT_NAME, id),
                ));

        if let Some(m) = &self.constant_temperature {
            MODALITY.register_mutator(m);
        }
    }

    fn update_fault_models(&mut self, dt: Time) {
        if let Some((pf, _)) = self.battery_degraded.as_mut() {
            pf.update(dt, self.temp_sensor.temperature());
        }

        self.error_register.out_of_sync = self
            .watchdog_out_of_sync
            .as_ref()
            .map(|m| m.is_active())
            .unwrap_or(false);

        if let Some(m) = self.constant_temperature.as_mut() {
            if let Some(active_mutation) = m.active_mutation() {
                if !m.was_applied {
                    m.was_applied = true;
                    let temp = Temperature::from_degrees_celsius(active_mutation);
                    self.temp_sensor.convert_to_constant_model(temp);
                }
            }
        }
    }

    fn hard_reset(&mut self, rel_time: Time) {
        warn!(?rel_time, "Power hard reset");

        // Clear error register
        self.error_register.out_of_sync = false;

        // Reset point failures
        if let Some((pf, _)) = self.battery_degraded.as_mut() {
            pf.reset();
        }

        if let Some(m) = self.watchdog_out_of_sync.as_mut() {
            MODALITY.clear_mutation(m);
        }

        if let Some(m) = self.constant_temperature.as_mut() {
            MODALITY.clear_mutation(m);

            // Re-initial temperature sensor to original model
            self.temp_sensor = TemperatureSensor::new(self.config.temperature_sensor_config);
        }

        // Reset to initial temperature
        self.temp_sensor.reset();

        // Drop message buffers
        self.cmd_rx.clear();
        self.res_tx.clear();
    }
}

#[derive(Debug, Clone)]
pub struct PowerConfig {
    pub battery_initial_charge: ElectricCharge,
    pub battery_max_charge: ElectricCharge,
    pub battery_max_voltage: ElectricPotential,

    /// How does the voltage change as the battery discharges? We use a simple linear
    /// model for this; measured in Volts / Coulomb
    pub battery_discharge_factor: PotentialOverCharge,

    pub solar_panel_charge_rate: ElectricCurrent,
    pub system_load: ElectricCurrent,

    pub temperature_sensor_config: TemperatureSensorConfig,

    pub fault_config: PowerFaultConfig,
}

/// Parameters for the power subsystem point failures.
/// See the requirements doc, sections 1.3.4.5 and 1.3.4.6.
#[derive(Debug, Clone, Default)]
pub struct PowerFaultConfig {
    /// Enable the solar panel degraded mutator.
    /// See sections 1.3.4.5 of the requirements doc.
    pub solar_panel_degraded: bool,

    /// Parameters for the battery degraded point failure.
    /// See sections 1.3.4.6 of the requirements doc.
    /// This point failure is motivated by temperature.
    /// It uses the provided battery discharge factor when active.
    pub battery_degraded: Option<(PointFailureConfig<Temperature>, PotentialOverCharge)>,

    /// Enable the data watchdog execution out-of-sync mutator.
    /// See sections 1.3.4.2 of the requirements doc.
    pub watchdog_out_of_sync: bool,

    /// Enable the constant temperature mutator.
    pub constant_temperature: bool,
}

impl PowerConfig {
    pub fn nominal(sat: &SatelliteId, mut with_variance: Option<&mut Rand64>) -> Self {
        let mut variance = |scale| {
            with_variance
                .as_mut()
                .map(|prng| ((prng.rand_float() * 2.0) - 1.0) * scale)
                .unwrap_or(0.0)
        };

        let max_charge = 40.0 - variance(5.0);
        let initial_charge = (max_charge * 0.9) - variance(3.0).abs();
        Self {
            battery_initial_charge: ElectricCharge::from_amp_hours(initial_charge),
            battery_max_charge: ElectricCharge::from_amp_hours(max_charge),
            battery_max_voltage: ElectricPotential::from_volts(13.1),
            battery_discharge_factor: ElectricPotential::from_volts(2.0)
                / ElectricCharge::from_amp_hours(40.0),
            solar_panel_charge_rate: if sat.is_goes() {
                ElectricCurrent::from_milliamps(1180.0 - variance(20.0))
            } else {
                ElectricCurrent::from_milliamps(1190.0 + variance(20.0))
            },
            system_load: if sat.is_goes() {
                ElectricCurrent::from_milliamps(550.0 - variance(10.0))
            } else {
                ElectricCurrent::from_milliamps(600.0 + variance(10.0))
            },
            temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::RandomInterval(
                    TemperatureSensorRandomIntervalModelParams {
                        initial: Temperature::from_degrees_celsius(variance(5.0)),
                        min: Temperature::from_degrees_celsius(-50.0),
                        max: Temperature::from_degrees_celsius(50.0),
                        day: TemperatureInterval::from_degrees_celsius(0.01),
                        night: TemperatureInterval::from_degrees_celsius(0.01),
                    },
                ),
            },
            fault_config: Default::default(),
        }
    }

    pub fn with_fault_config(mut self, fault_config: PowerFaultConfig) -> Self {
        self.fault_config = fault_config;
        self
    }

    pub fn with_all_mutators_enabled(mut self, enable_all_mutators: Option<bool>) -> Self {
        if enable_all_mutators.unwrap_or(false) {
            self.fault_config.solar_panel_degraded = true;
            self.fault_config.watchdog_out_of_sync = true;
            self.fault_config.constant_temperature = true;
        }
        self
    }
}

#[derive(Debug, Clone)]
pub enum PowerCommand {
    GetStatus,
}

impl TracedMessage for PowerCommand {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            PowerCommand::GetStatus => {
                vec![
                    kv("event.name", "get_status"),
                    kv("event.component", PowerSubsystem::COMPONENT_NAME),
                ]
            }
        }
    }
}

#[derive(Debug, Clone)]
pub enum PowerResponse {
    Status(PowerStatus),
}

impl TracedMessage for PowerResponse {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            PowerResponse::Status(power_status) => {
                let mut b = AttrsBuilder::new();
                b.kv("event.name", "power_status");
                b.kv("event.component", PowerSubsystem::COMPONENT_NAME);
                b.with_prefix("event.power", |b| power_status.to_attrs(b));
                b.build()
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct PowerStatus {
    pub battery_charge: ElectricCharge,

    /// Ranges from 0.0 to 1.0
    pub battery_charge_ratio: Ratio,

    /// Ranges from 0.0 to 1.0.
    pub solar_panel_illumination: Ratio,

    pub temperature: Temperature,

    pub error_register: PowerErrorRegister,
}

impl PowerStatus {
    pub fn to_attrs(&self, b: &mut AttrsBuilder) {
        b.kv("battery_charge", self.battery_charge.as_amp_hours());
        b.kv("battery_charge_ratio", self.battery_charge_ratio.as_f64());
        b.kv(
            "solar_panel_illumination",
            self.solar_panel_illumination.as_f64(),
        );
        b.kv("error.out_of_sync", self.error_register.out_of_sync);
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct PowerErrorRegister {
    /// Watchdog detected out-of-sync execution, subsystem is halted
    pub out_of_sync: bool,
}

impl<'a> SimulationComponent<'a> for PowerSubsystem {
    type SharedState = SatelliteSharedState;
    type Environment = SatelliteEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, sat: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        MODALITY.emit_sat_timeline_attrs(Self::COMPONENT_NAME, sat.id);
        event!("init");

        self.temp_sensor.init(env, sat);
        self.init_fault_models(sat.id);
    }

    fn reset(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        event!("reset");
        self.hard_reset(env.sim_info.relative_time);
    }

    fn step(&mut self, dt: Time, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);

        let mutators = [
            self.watchdog_out_of_sync.as_mut().map(|m| m.as_dyn()),
            self.solar_panel_degraded.as_mut().map(|m| m.as_dyn()),
            self.constant_temperature.as_mut().map(|m| m.as_dyn()),
        ];
        MODALITY.process_mutation_plane_messages(mutators.into_iter());

        let css = env
            .fsw_data
            .css
            .get(&0)
            .expect("Missing sun sensor observation at index 0");

        self.temp_sensor.step(dt, env, sat);

        self.update_fault_models(dt);

        let illumination = if css.valid && !self.error_register.out_of_sync {
            let illum = css.illum.abs();
            debug_assert!(css.illum <= 1.0);
            Ratio::from_f64(illum)
        } else {
            Ratio::from_f64(0.0)
        };

        if !self.error_register.out_of_sync {
            // Charge / discharge the battery
            self.battery_charge += (self.solar_panel_charge_rate() * illumination) * dt;
            self.battery_charge -= self.config.system_load * dt;

            if self.battery_charge > self.config.battery_max_charge {
                self.battery_charge = self.config.battery_max_charge;
            }

            // Voltage depends on how much the battery has discharged
            let discharge_amount = self.config.battery_max_charge - self.battery_charge;
            self.battery_voltage = self.config.battery_max_voltage
                - (self.battery_discharge_factor() * discharge_amount);

            // Write voltage to common state
            sat.power_supply_voltage = self.battery_voltage;
        }

        if !self.error_register.out_of_sync {
            if let Some(cmd) = recv!(&mut self.cmd_rx) {
                match cmd {
                    PowerCommand::GetStatus => {
                        let _ = try_send!(
                            &mut self.res_tx,
                            PowerResponse::Status(PowerStatus {
                                battery_charge: self.battery_charge,
                                battery_charge_ratio: self.battery_charge
                                    / self.config.battery_max_charge,
                                solar_panel_illumination: illumination,
                                temperature: self.temp_sensor.temperature(),
                                error_register: self.error_register,
                            })
                        );
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use types42::{
        prelude::{CoarseSunSensor, FswData},
        time::UtcTimestamp,
    };

    use super::*;
    use crate::{
        channel::{Step, StepChannel},
        satellite::{TemperatureSensorLinearModelParams, TemperatureSensorModel, SATELLITE_IDS},
        sim_info::SimulationInfo,
        units::{TemperatureIntervalRate, Timestamp},
    };

    #[test]
    fn test_power_subsystem() {
        let mut cmd_ch = StepChannel::new();
        let mut status_ch = StepChannel::new();

        let config = PowerConfig {
            battery_initial_charge: ElectricCharge::from_amp_hours(40.0),
            battery_max_charge: ElectricCharge::from_amp_hours(40.0),
            battery_max_voltage: ElectricPotential::from_volts(13.1),
            battery_discharge_factor: ElectricPotential::from_volts(2.0)
                / ElectricCharge::from_amp_hours(40.0),
            solar_panel_charge_rate: ElectricCurrent::from_milliamps(1200.0),
            system_load: ElectricCurrent::from_milliamps(600.0),
            temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::Linear(TemperatureSensorLinearModelParams {
                    initial: Temperature::from_degrees_celsius(0.0),
                    min: Temperature::from_degrees_celsius(-100.0),
                    max: Temperature::from_degrees_celsius(100.0),
                    day: TemperatureIntervalRate::from_degrees_celsius_per_second(0.0001),
                    night: TemperatureIntervalRate::from_degrees_celsius_per_second(-0.0001),
                }),
            },
            fault_config: Default::default(),
        };

        let mut power_system =
            PowerSubsystem::new(config, cmd_ch.receiver(None), status_ch.sender(None));

        let dt = Time::from_millis(1000.0);
        let mut last_voltage = ElectricPotential::from_volts(13.2);
        let mut last_charge = ElectricCharge::from_amp_hours(40.1);

        // It's dark! Battery should drain
        let mut fsw_data = FswData::default();
        fsw_data.css.insert(
            0,
            CoarseSunSensor {
                index: 0,
                valid: true,
                illum: 0.0,
            },
        );

        let timestamp = UtcTimestamp::default();

        let mut common_state = SatelliteSharedState {
            gui: None,
            id: &SATELLITE_IDS[0],
            power_supply_voltage: power_system.config.battery_max_voltage,
            rtc: Timestamp::epoch(),
            reset_flags: Default::default(),
        };

        let sim_info = SimulationInfo::new();
        let ground_truth_ir_events = vec![];

        {
            let env = SatelliteEnvironment {
                sim_info: &sim_info,
                ground_truth_ir_events: &ground_truth_ir_events,
                timestamp: &timestamp,
                fsw_data: &fsw_data,
            };

            for _ in 0..1000 {
                power_system.step(dt, &env, &mut common_state);

                assert!(power_system.battery_charge < last_charge);
                assert!(power_system.battery_voltage < last_voltage);

                last_charge = power_system.battery_charge;
                last_voltage = power_system.battery_voltage;
            }

            // Ask the power system its status
            let mut cmd_tx = cmd_ch.sender(None);
            let mut status_rx = status_ch.receiver(None);
            try_send!(&mut cmd_tx, PowerCommand::GetStatus).unwrap();
            cmd_ch.step().unwrap();
            power_system.step(dt, &env, &mut common_state);
            status_ch.step().unwrap();
            let status = recv!(&mut status_rx).unwrap();

            match status {
                PowerResponse::Status(status) => {
                    assert_eq!(status.battery_charge, power_system.battery_charge);
                    assert!(status.battery_charge_ratio.as_f64() > 0.0);
                    assert!(status.battery_charge_ratio.as_f64() < 1.0);
                }
            }
        }

        // Turn on the sun! Battery should charge.
        if let Some(css) = fsw_data.css.get_mut(&0) {
            css.illum = 1.0;
        }
        {
            let env = SatelliteEnvironment {
                sim_info: &sim_info,
                ground_truth_ir_events: &ground_truth_ir_events,
                timestamp: &timestamp,
                fsw_data: &fsw_data,
            };

            last_charge = power_system.battery_charge;
            last_voltage = power_system.battery_voltage;
            for _ in 0..1000 {
                power_system.step(dt, &env, &mut common_state);

                assert!(power_system.battery_charge > last_charge);
                assert!(power_system.battery_voltage > last_voltage);

                last_charge = power_system.battery_charge;
                last_voltage = power_system.battery_voltage;
            }
        }
    }
}
