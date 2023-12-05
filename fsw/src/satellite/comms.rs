use modality_api::TimelineId;
use nav_types::{NVector, WGS84};
use oorandom::Rand32;
use serde::Serialize;
use tracing::warn;

use crate::{
    channel::{Receiver, Sender, TracedMessage},
    ground_station::Relayed,
    modality::{kv, AttrsBuilder, MODALITY},
    mutator::{watchdog_out_of_sync_descriptor, GenericBooleanMutator},
    point_failure::{PointFailure, PointFailureConfig, PointFailureThresholdOperator},
    satellite::temperature_sensor::{
        TemperatureSensor, TemperatureSensorConfig, TemperatureSensorModel,
    },
    satellite::{SatelliteEnvironment, SatelliteId, SatelliteSharedState},
    system::{GroundToSatMessage, SatToGroundMessage},
    units::{Ratio, Temperature, Time, Timestamp},
    SimulationComponent,
};

pub struct CommsSubsystem {
    config: CommsConfig,
    cmd_rx: Receiver<CommsCommand>,
    res_tx: Sender<CommsResponse>,
    sat_to_ground: Sender<SatToGroundMessage>,
    ground_to_sat: Receiver<Relayed<GroundToSatMessage>>,
    timeline: TimelineId,
    temp_sensor: TemperatureSensor,

    prng: Rand32,
    enable_time_sync: bool,
    gps_offline_rtc_drift_ratio: Ratio,
    rtc_degraded_drift_ratio: Ratio,
    last_known_gps_nvec: NVector<f64>,
    error_register: CommsErrorRegister,

    gps_offline_rtc_drift: Option<(PointFailure<Time>, Ratio)>,
    gps_offline: Option<PointFailure<Time>>,
    ground_transceiver_failure: Option<PointFailure<Time>>,
    ground_transceiver_partial_failure: Option<(PointFailure<Time>, Ratio)>,
    rtc_degraded: Option<PointFailure<Temperature>>,
    watchdog_out_of_sync: Option<GenericBooleanMutator>,
}

impl CommsSubsystem {
    pub const COMPONENT_NAME: &'static str = "comms_subsystem";

    pub fn new(
        config: CommsConfig,
        cmd_rx: Receiver<CommsCommand>,
        res_tx: Sender<CommsResponse>,
        sat_to_ground: Sender<SatToGroundMessage>,
        ground_to_sat: Receiver<Relayed<GroundToSatMessage>>,
    ) -> Self {
        let timeline = TimelineId::allocate();
        let temp_sensor = TemperatureSensor::new(config.temperature_sensor_config);
        let gps_offline_rtc_drift = config
            .fault_config
            .gps_offline_rtc_drift
            .as_ref()
            .map(|(pf_config, val)| (PointFailure::new(pf_config.clone()), *val));
        let gps_offline = config
            .fault_config
            .gps_offline
            .as_ref()
            .map(|pf_config| PointFailure::new(pf_config.clone()));
        let ground_transceiver_failure = config
            .fault_config
            .ground_transceiver_failure
            .as_ref()
            .map(|pf_config| PointFailure::new(pf_config.clone()));
        let ground_transceiver_partial_failure = config
            .fault_config
            .ground_transceiver_partial_failure
            .as_ref()
            .map(|(pf_config, val)| (PointFailure::new(pf_config.clone()), *val));
        let rtc_degraded = config
            .fault_config
            .rtc_degraded
            .as_ref()
            .map(|pf_config| PointFailure::new(pf_config.clone()));

        if let (Some(partial_failure), Some(complete_failure)) = (
            ground_transceiver_partial_failure
                .as_ref()
                .map(|(pf, _)| pf.config()),
            ground_transceiver_failure.as_ref().map(|pf| pf.config()),
        ) {
            let err_msg = "ground_transceiver_partial_failure must happen before ground_transceiver_failure if both are enabled";
            let pf_t = if let PointFailureThresholdOperator::GreaterThanEqual(t) =
                partial_failure.threshold
            {
                t
            } else {
                panic!("{err_msg}");
            };
            let cf_t = if let PointFailureThresholdOperator::GreaterThanEqual(t) =
                complete_failure.threshold
            {
                t
            } else {
                panic!("{err_msg}");
            };
            if pf_t >= cf_t {
                panic!("{err_msg}");
            }
        }

        if rtc_degraded.is_some() {
            if let TemperatureSensorModel::Linear(_) = config.temperature_sensor_config.model {
                panic!("rtc_degraded requires a non-linear temperature sensor model");
            }
        }

        let rng_seed = config.fault_config.rng_seed;
        Self {
            cmd_rx,
            res_tx,
            config,
            sat_to_ground,
            ground_to_sat,
            timeline,
            temp_sensor,
            prng: Rand32::new(rng_seed.into()),
            enable_time_sync: true,
            gps_offline_rtc_drift_ratio: Ratio::from_f64(0.0),
            rtc_degraded_drift_ratio: Ratio::from_f64(0.0),
            // Gets re-initialized in init
            last_known_gps_nvec: NVector::new(Default::default(), 0.0),
            error_register: CommsErrorRegister { out_of_sync: false },
            gps_offline_rtc_drift,
            gps_offline,
            ground_transceiver_failure,
            ground_transceiver_partial_failure,
            rtc_degraded,
            // Mutators are initialized in init_fault_models at sim-init time
            watchdog_out_of_sync: None,
        }
    }

    fn init_fault_models(&mut self, id: &SatelliteId) {
        let base_ctx = [
            ("satellite_name", id.name),
            ("component_name", Self::COMPONENT_NAME),
        ];

        if let Some((pf, _)) = self.gps_offline_rtc_drift.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "gps_offline_rtc_drift");
        }
        if let Some(pf) = self.gps_offline.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "gps_offline");
        }
        if let Some(pf) = self.ground_transceiver_failure.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "ground_transceiver_failure");
        }
        if let Some((pf, _)) = self.ground_transceiver_partial_failure.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "ground_transceiver_partial_failure");
        }
        if let Some(pf) = self.rtc_degraded.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "rtc_degraded");
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
    }

    fn update_fault_models(&mut self, dt: Time, rel_time: Time) {
        if let Some((pf, drift_ratio)) = self.gps_offline_rtc_drift.as_mut() {
            let fault_active = pf.update(dt, rel_time);
            if fault_active {
                self.enable_time_sync = false;
                self.gps_offline_rtc_drift_ratio = *drift_ratio;
            }
        }

        if let Some(pf) = self.gps_offline.as_mut() {
            pf.update(dt, rel_time);
        }

        if let Some(pf) = self.ground_transceiver_failure.as_mut() {
            pf.update(dt, rel_time);
        }

        if let Some((pf, _)) = self.ground_transceiver_partial_failure.as_mut() {
            pf.update(dt, rel_time);
        }

        if let Some(pf) = self.rtc_degraded.as_mut() {
            let fault_active = pf.update(dt, self.temp_sensor.temperature());
            if fault_active {
                self.enable_time_sync = false;
                // already checked config for non-constant temperature model
                self.rtc_degraded_drift_ratio = self
                    .temp_sensor
                    .normalized_temperature()
                    .unwrap_or_else(|| Ratio::from_f64(0.0));
            }
        }

        self.error_register.out_of_sync = self
            .watchdog_out_of_sync
            .as_ref()
            .map(|m| m.is_active())
            .unwrap_or(false);
    }

    fn hard_reset(&mut self, rel_time: Time) {
        warn!(?rel_time, "Comms hard reset");

        // Clear error register
        self.error_register.out_of_sync = false;

        // Reset point failures
        if let Some((pf, _)) = self.gps_offline_rtc_drift.as_mut() {
            pf.reset();
        }

        if let Some(pf) = self.gps_offline.as_mut() {
            pf.reset();
        }

        if let Some(pf) = self.ground_transceiver_failure.as_mut() {
            pf.reset();
        }

        if let Some((pf, _)) = self.ground_transceiver_partial_failure.as_mut() {
            pf.reset();
        }

        if let Some(pf) = self.rtc_degraded.as_mut() {
            pf.reset();
        }

        if let Some(m) = self.watchdog_out_of_sync.as_mut() {
            MODALITY.clear_mutation(m);
        }

        // Reset to initial temperature
        self.temp_sensor.reset();

        // Drop message buffers
        self.cmd_rx.clear();
        self.res_tx.clear();
        self.sat_to_ground.clear();
        self.ground_to_sat.clear();
    }

    fn maybe_corrupt_msg(&mut self, chance: Ratio) -> bool {
        let pct = (100.0 * chance.as_f64()) as u32;
        self.prng.rand_u32() % 100 < pct
    }
}

#[derive(Debug, Clone)]
pub enum CommsCommand {
    SendGroundMessage(Box<SatToGroundMessage>),
    GetGps,
    GetStatus,
}

impl TracedMessage for CommsCommand {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            CommsCommand::SendGroundMessage(_) => vec![kv("event.name", "send_ground_message")],
            CommsCommand::GetGps => vec![kv("event.name", "get_gps")],
            CommsCommand::GetStatus => vec![kv("event.name", "get_status")],
        }
    }
}

#[derive(Debug, Clone)]
pub enum CommsResponse {
    #[allow(dead_code)]
    RecvGroundMessage(GroundToSatMessage),
    GpsPosition(nav_types::NVector<f64>),
    LastKnownGpsPosition(nav_types::NVector<f64>),
    Status(CommsStatus),
}

impl TracedMessage for CommsResponse {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        let mut b = AttrsBuilder::new();
        match self {
            CommsResponse::RecvGroundMessage(msg) => {
                b.kv("event.name", "recv_ground_message");
                b.kv("event.message_name", msg.name());
            }
            CommsResponse::GpsPosition(gps) => {
                b.kv("event.name", "gps_position");
                let pos = WGS84::<f64>::from(*gps);
                b.kv("event.latitude", pos.latitude_degrees());
                b.kv("event.longitude", pos.longitude_degrees());
                b.kv("event.altitude", pos.altitude());
            }
            CommsResponse::LastKnownGpsPosition(gps) => {
                b.kv("event.name", "last_known_gps_position");
                let pos = WGS84::<f64>::from(*gps);
                b.kv("event.latitude", pos.latitude_degrees());
                b.kv("event.longitude", pos.longitude_degrees());
                b.kv("event.altitude", pos.altitude());
            }
            CommsResponse::Status(comms_status) => {
                b.kv("event.name", "comms_status");
                b.with_prefix("event.comms", |b| comms_status.to_attrs(b));
            }
        }

        b.build()
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct CommsStatus {
    pub temperature: Temperature,
    pub error_register: CommsErrorRegister,
}

impl CommsStatus {
    pub fn to_attrs(&self, b: &mut AttrsBuilder) {
        b.kv("temperature", self.temperature.as_degrees_celsius());
        b.kv("error.out_of_sync", self.error_register.out_of_sync);
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct CommsErrorRegister {
    /// Watchdog detected out-of-sync execution, subsystem is halted
    pub out_of_sync: bool,
}

#[derive(Debug, Clone)]
pub struct CommsConfig {
    pub temperature_sensor_config: TemperatureSensorConfig,
    pub fault_config: CommsFaultConfig,
}

/// Parameters for the comms subsystem point failures.
/// See the requirements doc, sections 1.3.4.7-1.3.4.11.
#[derive(Debug, Clone, Default)]
pub struct CommsFaultConfig {
    /// Seed for the PRNG.
    pub rng_seed: u32,

    /// Parameters for the GPS offline RTC drift point failure.
    /// See sections 1.3.4.7 of the requirements doc.
    /// This point failure is motivated by relative time.
    /// It uses the provided drift ratio when active.
    /// Note that multiple point failures can accumulate RTC drift.
    pub gps_offline_rtc_drift: Option<(PointFailureConfig<Time>, Ratio)>,

    /// Parameters for the GPS offline point failure.
    /// See sections 1.3.4.8 of the requirements doc.
    /// This point failure is motivated by relative time.
    /// Current GPS position is unknown when this point failure
    /// is active.
    pub gps_offline: Option<PointFailureConfig<Time>>,

    /// Parameters for the ground transceiver failure point failure.
    /// See sections 1.3.4.9 of the requirements doc.
    /// This point failure is motivated by relative time.
    pub ground_transceiver_failure: Option<PointFailureConfig<Time>>,

    /// Parameters for the ground transceiver partial failure point failure.
    /// See sections 1.3.4.10 of the requirements doc.
    /// This point failure is motivated by relative time.
    /// It uses the provided corrupted message chance to drop messages.
    pub ground_transceiver_partial_failure: Option<(PointFailureConfig<Time>, Ratio)>,

    /// Parameters for the RTC degraded point failure.
    /// See sections 1.3.4.11 of the requirements doc.
    /// This point failure is motivated by temperature.
    /// The RTC drift ratio is set to the normalized temperature
    /// when active.
    /// Note that multiple point failures can accumulate RTC drift.
    pub rtc_degraded: Option<PointFailureConfig<Temperature>>,

    /// Enable the data watchdog execution out-of-sync mutator.
    /// See sections 1.3.4.2 of the requirements doc.
    pub watchdog_out_of_sync: bool,
}

impl<'a> SimulationComponent<'a> for CommsSubsystem {
    type SharedState = SatelliteSharedState;
    type Environment = SatelliteEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        MODALITY.emit_sat_timeline_attrs(Self::COMPONENT_NAME, sat.id);
        MODALITY.quick_event("init");

        self.temp_sensor.init(env, sat);
        self.init_fault_models(sat.id);
    }

    fn reset(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        MODALITY.quick_event("reset");
        self.hard_reset(env.sim_info.relative_time);
    }

    fn step(&mut self, dt: Time, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        self.temp_sensor.step(dt, env, sat);

        if let Some(m) = &mut self.watchdog_out_of_sync {
            MODALITY.process_mutation_plane_messages(std::iter::once(m));
        }

        // NOTE: This may need to move down, once we pull in mutation support, to occur after we tick the clock.
        self.update_fault_models(dt, env.sim_info.relative_time);

        if !self.error_register.out_of_sync {
            let timestamp = Timestamp::from_utc(*env.timestamp);
            if sat.rtc < timestamp && self.enable_time_sync {
                // This is 'gps' time
                sat.rtc = timestamp;
            } else {
                let total_drift = (self.gps_offline_rtc_drift_ratio.as_f64()
                    + self.rtc_degraded_drift_ratio.as_f64())
                .clamp(-1.0, 1.0);
                let drift = Ratio::from_f64(total_drift);
                sat.rtc += dt + (dt * drift);
            }
        }

        // NB: This happens down here a little bit because this component is responsible for
        // ticking the clock, and we need that to happen first. It's okay because none of
        // the above code (presently) logs any events.
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);

        let gps = env.fsw_data.gps.get(&0).unwrap();
        let wgs84 =
            WGS84::from_radians_and_meters(gps.wgs_latitude, gps.wgs_longitude, gps.wgs_altitude);
        let gps_nvec = NVector::from(wgs84);

        let gps_offline = self
            .gps_offline
            .as_ref()
            .map(|pf| pf.is_active())
            .unwrap_or(false);
        let ground_trx_offline = self
            .ground_transceiver_failure
            .as_ref()
            .map(|pf| pf.is_active())
            .unwrap_or(false);
        let (ground_trx_partially_offline, corrupt_chance) = self
            .ground_transceiver_partial_failure
            .as_ref()
            .map(|(pf, v)| (pf.is_active(), *v))
            .unwrap_or((false, Ratio::from_f64(0.0)));

        if !gps_offline {
            self.last_known_gps_nvec = gps_nvec;
        }

        self.sat_to_ground.set_location(gps_nvec);
        self.ground_to_sat.set_location(gps_nvec);

        if let Some(msg) = self.cmd_rx.recv() {
            match msg {
                CommsCommand::SendGroundMessage(msg) => {
                    if !self.error_register.out_of_sync {
                        if ground_trx_partially_offline {
                            if self.maybe_corrupt_msg(corrupt_chance) {
                                warn!(seq = msg.seq, "Dropping corrupt message");
                            } else {
                                let _ = self.sat_to_ground.try_send(*msg);
                            }
                        } else if !ground_trx_offline {
                            let _ = self.sat_to_ground.try_send(*msg);
                        }
                    }
                }
                CommsCommand::GetGps => {
                    if !self.error_register.out_of_sync {
                        if gps_offline {
                            let _ = self.res_tx.try_send(CommsResponse::LastKnownGpsPosition(
                                self.last_known_gps_nvec,
                            ));
                        } else {
                            let _ = self.res_tx.try_send(CommsResponse::GpsPosition(gps_nvec));
                        }
                    }
                }
                CommsCommand::GetStatus => {
                    let _ = self.res_tx.try_send(CommsResponse::Status(CommsStatus {
                        temperature: self.temp_sensor.temperature(),
                        error_register: self.error_register,
                    }));
                }
            }
        }

        while let Some(msg) = self.ground_to_sat.recv() {
            if msg.inner.destination() == sat.id.satcat_id {
                let _ = self
                    .res_tx
                    .try_send(CommsResponse::RecvGroundMessage(msg.inner));
            }
        }
    }
}
