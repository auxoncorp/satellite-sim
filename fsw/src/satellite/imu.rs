use modality_api::TimelineId;
use na::Vector3;
use serde::Serialize;
use tracing::warn;

use crate::{
    channel::{Receiver, Sender, TracedMessage},
    modality::{kv, AttrsBuilder, MODALITY},
    point_failure::{PointFailure, PointFailureConfig},
    satellite::{SatelliteEnvironment, SatelliteId, SatelliteSharedState},
    units::{Acceleration, AngularVelocity, MagneticFluxDensity, Temperature, Time, Timestamp},
    SimulationComponent,
};

use super::temperature_sensor::{TemperatureSensor, TemperatureSensorConfig};

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
    watchdog_out_of_sync: Option<PointFailure<Time>>,
    watchdog_out_of_sync_threshold: Option<Time>,

    timeline: TimelineId,

    sample_tx: Sender<ImuSample>,
    cmd_rx: Receiver<ImuCommand>,
    res_tx: Sender<ImuResponse>,
}

#[derive(Debug, Clone)]
pub struct ImuConfig {
    pub temperature_sensor_config: TemperatureSensorConfig,
    pub fault_config: Option<ImuFaultConfig>,
}

/// Parameters for the IMU point failures.
/// See the requirements doc, sections 1.3.4.15 and 1.3.4.16.
///
/// Both point point failures are motivated by temperature.
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

    /// Switch the temperature sensor model to constant after
    /// a `ImuCommand::Reset` is received, or a hard reset occurs.
    /// This can be used to prevent the temperature-motivated
    /// point failures from recurring.
    pub constant_temperature_after_reset: Option<Temperature>,

    /// Parameters for the data watchdog execution out-of-sync point failure.
    /// See sections 1.3.4.2 of the requirements doc.
    pub watchdog_out_of_sync: Option<PointFailureConfig<Time>>,

    /// Whether the `watchdog_out_of_sync` point failure
    /// is recurring or not. Defaults to false.
    pub watchdog_out_of_sync_recurring: Option<bool>,
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
                kv("event.name", "reset"),
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
            .as_ref()
            .and_then(|c| c.degraded_state.as_ref())
            .map(|pf_config| PointFailure::new(pf_config.clone()));

        let data_inconsistency = config
            .fault_config
            .as_ref()
            .and_then(|c| c.data_inconsistency.as_ref())
            .map(|pf_config| PointFailure::new(pf_config.clone()));

        let watchdog_out_of_sync = config
            .fault_config
            .as_ref()
            .and_then(|c| c.watchdog_out_of_sync.as_ref())
            .map(|pf_config| PointFailure::new(pf_config.clone()));
        let watchdog_out_of_sync_threshold = watchdog_out_of_sync.as_ref().map(|pf| pf.threshold());

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
            watchdog_out_of_sync,
            watchdog_out_of_sync_threshold,
            timeline: TimelineId::allocate(),
            sample_tx,
            cmd_rx,
            res_tx,
        }
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
        if let Some(pf) = self.watchdog_out_of_sync.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "watchdog_out_of_sync");
        }
    }

    fn update_fault_models(&mut self, dt: Time, rel_time: Time) {
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

        if let Some(pf) = self.watchdog_out_of_sync.as_mut() {
            let fault_active = pf.update(dt, rel_time);
            self.error_register.out_of_sync = fault_active;
        }
    }

    fn soft_reset(&mut self, rel_time: Time) {
        warn!(?rel_time, "IMU soft reset");

        // Reset the degraded state point failure
        if let Some(pf) = self.degraded_state.as_mut() {
            pf.reset();
        }

        if let Some(constant_temperature_after_reset) = self
            .config
            .fault_config
            .as_ref()
            .and_then(|c| c.constant_temperature_after_reset)
        {
            // Use a constant temperature model
            self.temp_sensor
                .convert_to_constant_model(constant_temperature_after_reset);
        } else {
            // Otherwise reset to initial temperature in the original model
            self.temp_sensor.reset();
        }

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
        if let Some(pf) = self.watchdog_out_of_sync.as_mut() {
            let watchdog_out_of_sync_recurring = self
                .config
                .fault_config
                .as_ref()
                .and_then(|c| c.watchdog_out_of_sync_recurring)
                .unwrap_or(false);

            pf.reset();

            if watchdog_out_of_sync_recurring {
                // Update the time threshold to now + dur
                pf.set_threshold(self.watchdog_out_of_sync_threshold.unwrap() + rel_time);
            } else {
                // One-shot, disable further activations
                pf.set_disabled(true);
            }
        }

        if let Some(constant_temperature_after_reset) = self
            .config
            .fault_config
            .as_ref()
            .and_then(|c| c.constant_temperature_after_reset)
        {
            // Use a constant temperature model
            self.temp_sensor
                .convert_to_constant_model(constant_temperature_after_reset);
        } else {
            // Otherwise reset to initial temperature in the original model
            self.temp_sensor.reset();
        }

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

        self.temp_sensor.init(env, sat);
        self.init_fault_models(sat.id);
    }

    fn reset(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        MODALITY.quick_event("reset");
        self.hard_reset(env.sim_info.relative_time);
    }

    fn step(&mut self, dt: Time, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);

        self.temp_sensor.step(dt, env, sat);

        self.update_fault_models(dt, env.sim_info.relative_time);

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

            let _ = self.sample_tx.try_send(sample);
        }

        if let Some(cmd) = self.cmd_rx.recv() {
            match cmd {
                ImuCommand::GetStatus => {
                    let _ = self.res_tx.try_send(ImuResponse::Status(ImuStatus {
                        temperature: self.temp_sensor.temperature(),
                        error_register: self.error_register,
                    }));
                }
                ImuCommand::Reset => {
                    self.soft_reset(env.sim_info.relative_time);

                    let _ = self.res_tx.try_send(ImuResponse::Status(ImuStatus {
                        temperature: self.temp_sensor.temperature(),
                        error_register: self.error_register,
                    }));
                }
                ImuCommand::ClearDataInconsistency => {
                    self.error_register.data_inconsistency = false;

                    if let Some(pf) = self.data_inconsistency.as_mut() {
                        pf.reset();
                    }

                    let _ = self.res_tx.try_send(ImuResponse::Status(ImuStatus {
                        temperature: self.temp_sensor.temperature(),
                        error_register: self.error_register,
                    }));
                }
            }
        }
    }
}
