use modality_api::TimelineId;
use na::Vector3;
use oorandom::Rand64;
use serde::Serialize;
use tracing::warn;

use crate::{
    channel::{Receiver, Sender, TracedMessage},
    event,
    modality::{kv, AttrsBuilder, MODALITY},
    mutator::{
        constant_temperature_descriptor, degrade_orbit_maintenance_descriptor,
        watchdog_out_of_sync_descriptor, GenericBooleanMutator, GenericSetFloatMutator,
        MutatorActuatorDescriptor,
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
        Acceleration, AngularVelocity, MagneticFluxDensity, Temperature, TemperatureInterval, Time,
        Timestamp,
    },
    SimulationComponent,
};

// Sensor axis indices are fixed, set in the 42 spacecraft config file
const INDEX_X: u64 = 0;
const INDEX_Y: u64 = 1;
const INDEX_Z: u64 = 2;

#[derive(Debug)]
pub struct ImuSubsystem {
    config: ImuConfig,

    temp_sensor: TemperatureSensor,
    error_register: ImuErrorRegister,

    degraded_state: Option<PointFailure<Temperature>>,
    data_inconsistency: Option<PointFailure<Temperature>>,
    watchdog_out_of_sync: Option<GenericBooleanMutator>,
    constant_temperature: Option<GenericSetFloatMutator>,
    orbit_maintenance: Option<GenericSetFloatMutator>,

    timeline: TimelineId,

    sample_tx: Sender<ImuSample>,
    cmd_rx: Receiver<ImuCommand>,
    res_tx: Sender<ImuResponse>,
}

#[derive(Debug, Clone)]
pub struct ImuConfig {
    pub temperature_sensor_config: TemperatureSensorConfig,
    pub fault_config: ImuFaultConfig,
}

/// Parameters for the IMU point failures and mutators.
/// See the requirements doc, sections 1.3.4.15 and 1.3.4.16.
#[derive(Debug, Clone, Default)]
pub struct ImuFaultConfig {
    /// Parameters for the degraded state point failure.
    /// See sections 1.3.4.15 of the requirements doc.
    ///
    /// This failure point may be cleared by an `ImuCommand::Reset`
    /// and thus can also be recurring.
    /// The current temperature is also reset to its initial value on `ImuCommand::Reset`.
    pub degraded_state: Option<PointFailureConfig<Temperature>>,

    /// Parameters for the data inconsistency point failure.
    /// See sections 1.3.4.16 of the requirements doc.
    pub data_inconsistency: Option<PointFailureConfig<Temperature>>,

    /// Enable the data watchdog execution out-of-sync mutator.
    /// See sections 1.3.4.2 of the requirements doc.
    pub watchdog_out_of_sync: bool,

    /// Enable the constant temperature mutator.
    pub constant_temperature: bool,

    /// Enable the orbit maintenance mutator.
    pub orbit_maintenance: bool,
}

impl ImuConfig {
    pub fn nominal(_sat: &SatelliteId, mut with_variance: Option<&mut Rand64>) -> Self {
        let mut variance = |scale| {
            with_variance
                .as_mut()
                .map(|prng| ((prng.rand_float() * 2.0) - 1.0) * scale)
                .unwrap_or(0.0)
        };

        Self {
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

    pub fn with_fault_config(mut self, fault_config: ImuFaultConfig) -> Self {
        self.fault_config = fault_config;
        self
    }

    pub fn with_all_mutators_enabled(mut self, enable_all_mutators: Option<bool>) -> Self {
        if enable_all_mutators.unwrap_or(false) {
            self.fault_config.watchdog_out_of_sync = true;
            self.fault_config.constant_temperature = true;
            self.fault_config.orbit_maintenance = true;
        }
        self
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ImuSample {
    pub timestamp: Timestamp,
    pub angular_velocity: Vector3<AngularVelocity>,
    pub acceleration: Vector3<Acceleration>,
    pub magnetic_flux_density: Vector3<MagneticFluxDensity>,
}

impl TracedMessage for ImuSample {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        let mut b = AttrsBuilder::new();
        b.kv("event.name", "imu_sample");
        b.with_prefix("event", |b| self.to_attrs(b));
        b.build()
    }
}

impl ImuSample {
    pub fn to_attrs(&self, b: &mut AttrsBuilder) {
        b.kv(
            "angular_velocity.x",
            self.angular_velocity.x.as_degrees_per_second(),
        );
        b.kv(
            "angular_velocity.y",
            self.angular_velocity.y.as_degrees_per_second(),
        );
        b.kv(
            "angular_velocity.z",
            self.angular_velocity.z.as_degrees_per_second(),
        );

        b.kv(
            "acceleration.x",
            self.acceleration.x.as_meters_per_second_squared(),
        );
        b.kv(
            "acceleration.y",
            self.acceleration.y.as_meters_per_second_squared(),
        );
        b.kv(
            "acceleration.z",
            self.acceleration.z.as_meters_per_second_squared(),
        );

        b.kv(
            "magnetic_flux_density.x",
            self.magnetic_flux_density.x.as_teslas(),
        );
        b.kv(
            "magnetic_flux_density.y",
            self.magnetic_flux_density.y.as_teslas(),
        );
        b.kv(
            "magnetic_flux_density.z",
            self.magnetic_flux_density.z.as_teslas(),
        );
    }
}

#[derive(Debug, Clone)]
pub enum ImuCommand {
    GetStatus,

    /// Reset the IMU and its temperature sensor back to initial value.
    /// Clears the `ImuErrorRegister.degraded` flag.
    /// Replies with `ImuResponse::Status`.
    Reset,

    /// Clears the `ImuErrorRegister.data_inconsistency` flag.
    /// Replies with `ImuResponse::Status`.
    ClearDataInconsistency,
}

impl TracedMessage for ImuCommand {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            ImuCommand::GetStatus => vec![
                kv("event.name", "get_status"),
                kv("event.component", ImuSubsystem::COMPONENT_NAME),
            ],
            ImuCommand::Reset => vec![
                kv("event.name", "reset_imu"),
                kv("event.component", ImuSubsystem::COMPONENT_NAME),
            ],
            ImuCommand::ClearDataInconsistency => {
                vec![
                    kv("event.name", "clear_data_inconsistency"),
                    kv("event.component", ImuSubsystem::COMPONENT_NAME),
                ]
            }
        }
    }
}

#[derive(Debug, Clone)]
pub enum ImuResponse {
    Status(ImuStatus),
}

impl TracedMessage for ImuResponse {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            ImuResponse::Status(imu_status) => {
                let mut b = AttrsBuilder::new();
                b.kv("event.name", "imu_status");
                b.kv("event.component", ImuSubsystem::COMPONENT_NAME);
                b.with_prefix("event", |b| imu_status.to_attrs(b));
                b.build()
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct ImuStatus {
    pub temperature: Temperature,
    pub error_register: ImuErrorRegister,
}

impl ImuStatus {
    pub fn to_attrs(&self, b: &mut AttrsBuilder) {
        b.kv("temperature", self.temperature.as_degrees_celsius());
        b.kv("error.out_of_sync", self.error_register.out_of_sync);
        b.kv("error.degraded", self.error_register.degraded);
        b.kv(
            "error.data_inconsistency",
            self.error_register.data_inconsistency,
        );
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct ImuErrorRegister {
    /// Watchdog detected out-of-sync execution, subsystem is halted
    pub out_of_sync: bool,
    /// Running on the backup IMU, in a degraded state.
    pub degraded: bool,
    /// There's an inconsistency between the primary and backup IMU.
    /// No IMU data will be sent.
    pub data_inconsistency: bool,
}

impl ImuSubsystem {
    pub const COMPONENT_NAME: &'static str = "imu_subsystem";

    pub fn new(
        config: ImuConfig,
        sample_tx: Sender<ImuSample>,
        cmd_rx: Receiver<ImuCommand>,
        res_tx: Sender<ImuResponse>,
    ) -> Self {
        let temp_sensor = TemperatureSensor::new(config.temperature_sensor_config);

        let degraded_state = config
            .fault_config
            .degraded_state
            .as_ref()
            .map(|pf_config| PointFailure::new(pf_config.clone()));

        let data_inconsistency = config
            .fault_config
            .data_inconsistency
            .as_ref()
            .map(|pf_config| PointFailure::new(pf_config.clone()));

        Self {
            config,
            temp_sensor,
            error_register: ImuErrorRegister {
                out_of_sync: false,
                degraded: false,
                data_inconsistency: false,
            },
            degraded_state,
            data_inconsistency,
            timeline: TimelineId::allocate(),
            sample_tx,
            cmd_rx,
            res_tx,
            // Mutators are initialized in init_fault_models at sim-init time
            watchdog_out_of_sync: None,
            constant_temperature: None,
            orbit_maintenance: None,
        }
    }

    pub fn has_error_bits_set(&self) -> bool {
        self.error_register.out_of_sync
            || self.error_register.degraded
            || self.error_register.data_inconsistency
    }

    /// Get the orbit maintenance mutator's error_rate_ratio mutation parameter.
    /// This is used out-of-band in the System to modify the GPS's reported position.
    pub(crate) fn orbit_maintenance_error_rate_ratio(&self) -> Option<f64> {
        self.orbit_maintenance
            .as_ref()
            .and_then(|m| m.active_mutation())
    }

    fn init_fault_models(&mut self, id: &SatelliteId) {
        let base_ctx = [
            ("satellite_name", id.name),
            ("component_name", Self::COMPONENT_NAME),
        ];

        if let Some(pf) = self.degraded_state.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "degraded_state");
        }
        if let Some(pf) = self.data_inconsistency.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "data_inconsistency");
        }

        // Construct mutators at sim-init time so we have access to SatelliteId/env details
        // to qualify the mutator descriptors
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

        self.orbit_maintenance =
            self.config
                .fault_config
                .orbit_maintenance
                .then_some(GenericSetFloatMutator::new(
                    degrade_orbit_maintenance_descriptor(Self::COMPONENT_NAME, id),
                ));

        if let Some(m) = &self.orbit_maintenance {
            MODALITY.register_mutator(m);
        }
    }

    fn update_fault_models(&mut self, dt: Time) {
        if let Some(pf) = self.degraded_state.as_mut() {
            let was_active = pf.is_active();
            let fault_active = pf.update(dt, self.temp_sensor.temperature());
            // This point failure can be cleared, so it's only applied when it transitions
            if !was_active && fault_active {
                self.error_register.degraded = fault_active;
            }
        }

        if let Some(pf) = self.data_inconsistency.as_mut() {
            let fault_active = pf.update(dt, self.temp_sensor.temperature());
            self.error_register.data_inconsistency = fault_active;
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

        if self
            .orbit_maintenance
            .as_ref()
            .map(|m| m.active_mutation().is_some())
            .unwrap_or(false)
        {
            self.error_register.degraded = true;
        }
    }

    fn soft_reset(&mut self, rel_time: Time) {
        warn!(?rel_time, "IMU soft reset");

        // Reset the degraded state point failure
        if let Some(pf) = self.degraded_state.as_mut() {
            pf.reset();
        }

        // Reset to initial temperature in the original model
        self.temp_sensor = TemperatureSensor::new(self.config.temperature_sensor_config);
        self.temp_sensor.reset();

        // Clear the degraded state point failure
        self.error_register.degraded = false;
    }

    fn hard_reset(&mut self, rel_time: Time) {
        warn!(?rel_time, "IMU hard reset");

        // Clear error register
        self.error_register.out_of_sync = false;
        self.error_register.degraded = false;
        self.error_register.data_inconsistency = false;

        // Reset point failures
        if let Some(pf) = self.degraded_state.as_mut() {
            pf.reset();
        }
        if let Some(pf) = self.data_inconsistency.as_mut() {
            pf.reset();
        }

        if let Some(m) = self.watchdog_out_of_sync.as_mut() {
            MODALITY.clear_mutation(m);

            // Re-initial temperature sensor to original model
            self.temp_sensor = TemperatureSensor::new(self.config.temperature_sensor_config);
        }

        // Reset to initial temperature in the original model
        self.temp_sensor.reset();

        // Drop message buffers
        self.sample_tx.clear();
        self.cmd_rx.clear();
        self.res_tx.clear();
    }
}

impl<'a> SimulationComponent<'a> for ImuSubsystem {
    type SharedState = SatelliteSharedState;
    type Environment = SatelliteEnvironment<'a>;

    fn init(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
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
            self.constant_temperature.as_mut().map(|m| m.as_dyn()),
            self.orbit_maintenance.as_mut().map(|m| m.as_dyn()),
        ];
        MODALITY.process_mutation_plane_messages(mutators.into_iter());

        self.temp_sensor.step(dt, env, sat);

        self.update_fault_models(dt);

        // We don't send data if the data inconsistency or watchdog point failure is active
        if !self.error_register.data_inconsistency && !self.error_register.out_of_sync {
            let mut sample = ImuSample {
                timestamp: sat.rtc,
                angular_velocity: Vector3::from_element(AngularVelocity::from_radians_per_second(
                    0.0,
                )),
                acceleration: Vector3::from_element(Acceleration::from_meters_per_second_squared(
                    0.0,
                )),
                magnetic_flux_density: Vector3::from_element(MagneticFluxDensity::from_teslas(0.0)),
            };

            for idx in [INDEX_X, INDEX_Y, INDEX_Z] {
                let gyro = env.fsw_data.gyro.get(&idx).expect("Missing gyro index");
                let magf = env.fsw_data.mag.get(&idx).expect("Missing magf index");
                let accel = env.fsw_data.accel.get(&idx).expect("Bad accel index");

                sample.angular_velocity[idx as usize] =
                    AngularVelocity::from_radians_per_second(gyro.rate);
                sample.acceleration[idx as usize] =
                    Acceleration::from_meters_per_second_squared(accel.acc);
                sample.magnetic_flux_density[idx as usize] =
                    MagneticFluxDensity::from_teslas(magf.field);
            }

            let _ = try_send!(&mut self.sample_tx, sample);
        }

        if !self.error_register.out_of_sync {
            if let Some(cmd) = recv!(&mut self.cmd_rx) {
                match cmd {
                    ImuCommand::GetStatus => {
                        let _ = try_send!(
                            &mut self.res_tx,
                            ImuResponse::Status(ImuStatus {
                                temperature: self.temp_sensor.temperature(),
                                error_register: self.error_register,
                            })
                        );
                    }
                    ImuCommand::Reset => {
                        self.soft_reset(env.sim_info.relative_time);

                        let _ = try_send!(
                            &mut self.res_tx,
                            ImuResponse::Status(ImuStatus {
                                temperature: self.temp_sensor.temperature(),
                                error_register: self.error_register,
                            })
                        );
                    }
                    ImuCommand::ClearDataInconsistency => {
                        self.error_register.data_inconsistency = false;

                        if let Some(pf) = self.data_inconsistency.as_mut() {
                            pf.reset();
                        }

                        let _ = try_send!(
                            &mut self.res_tx,
                            ImuResponse::Status(ImuStatus {
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
