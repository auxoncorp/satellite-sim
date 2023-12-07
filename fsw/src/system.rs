use serde::Serialize;
use std::{collections::HashMap, net::TcpStream};

use modality_ingest_client::IngestError;
use na::Vector3;
use nav_types::{NVector, WGS84};
use types42::spacecraft::SpacecraftIndex;

use crate::{
    channel::{Step, StepChannel, TracedMessage},
    ground_station::{ConsolidatedGroundStation, RelayGroundStation, Relayed},
    ground_truth_ir_events::{GroundTruthIrEventsManager, GroundTruthIrEventsManagerConfig},
    gui::SharedGuiState,
    modality::{kv, AttrsBuilder},
    satellite::{
        CommsStatus, ComputeStatus, ImuSample, ImuStatus, PowerStatus, SatCatId, Satellite,
        SatelliteEnvironment, VisionStatus,
    },
    scenario::Scenario,
    sim_info::SimulationInfo,
    units::{LuminousIntensity, Time, Timestamp},
    SimulationComponent,
};

/// The whole system, with all the satellites and ground stations.
pub struct System {
    scenario: Scenario,

    satellites: HashMap<SpacecraftIndex, Satellite>,
    ground_truth_events_manager: GroundTruthIrEventsManager,
    consolidated_ground_station: ConsolidatedGroundStation,
    relay_stations: Vec<RelayGroundStation>,

    cgs_to_relay_ch: StepChannel<GroundToSatMessage>,
    relay_to_cgs_ch: StepChannel<Relayed<SatToGroundMessage>>,
    relay_to_sat_ch: StepChannel<Relayed<GroundToSatMessage>>,
    sat_to_relay_ch: StepChannel<SatToGroundMessage>,
}

impl System {
    pub fn new(scenario: Scenario, external_mission_control: Option<TcpStream>) -> Self {
        let mut cgs_to_relay_ch = StepChannel::new();
        let mut relay_to_cgs_ch = StepChannel::new();

        let mut relay_to_sat_ch = StepChannel::new();
        let mut sat_to_relay_ch = StepChannel::new();

        let consolidated_ground_station = ConsolidatedGroundStation::new(
            scenario.consolidated_ground_station_config.clone(),
            || relay_to_cgs_ch.receiver(None),
            cgs_to_relay_ch.sender(None),
            external_mission_control.as_ref().map(|s| {
                s.try_clone()
                    .expect("Failed to connect to external mission control")
            }),
        );

        let mut relay_stations = vec![];
        for (_, relay_config) in scenario.ground_stations.clone().into_iter() {
            let pos = relay_config.position;
            let relay = RelayGroundStation::new(
                relay_config,
                sat_to_relay_ch.receiver(None).with_location(pos),
                relay_to_sat_ch.sender(None).with_location(pos),
                cgs_to_relay_ch.receiver(None),
                relay_to_cgs_ch.sender(None),
            );
            relay_stations.push(relay);
        }

        let scheduled_events = scenario.ir_events.clone();

        Self {
            scenario,
            satellites: HashMap::new(),
            ground_truth_events_manager: GroundTruthIrEventsManager::new(
                GroundTruthIrEventsManagerConfig { scheduled_events },
                external_mission_control,
            ),
            consolidated_ground_station,
            relay_stations,
            cgs_to_relay_ch,
            relay_to_cgs_ch,
            relay_to_sat_ch,
            sat_to_relay_ch,
        }
    }
}
impl<'a> SimulationComponent<'a> for System {
    type SharedState = SystemSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &SystemEnvironment, shared_state: &mut Self::SharedState) {
        if let Some(mut gui) = shared_state.gui.as_ref().map(|gui| gui.borrow_mut()) {
            gui.update_celestial_bodies(&env.telemetry.worlds);
        }

        self.ground_truth_events_manager.init(env, shared_state);

        for relay in self.relay_stations.iter_mut() {
            relay.init(env, shared_state);
        }
        self.consolidated_ground_station.init(env, &mut ());

        for (sat_idx, spacecraft) in env.telemetry.spacecrafts.iter() {
            let mut sat = Satellite::new(
                self.scenario
                    .satellite_configs
                    .get(sat_idx)
                    .expect("Bad satellite index")
                    .clone(),
                self.sat_to_relay_ch.sender(None),
                self.relay_to_sat_ch.receiver(None),
            );
            let env = SatelliteEnvironment {
                sim_info: env.sim_info,
                timestamp: &env.telemetry.timestamp,
                fsw_data: &spacecraft.ac,
                ground_truth_ir_events: self.ground_truth_events_manager.active_events(),
            };
            sat.init(&env, shared_state);
            self.satellites.insert(*sat_idx, sat);
        }
    }

    fn reset(&mut self, env: &'a Self::Environment, shared_state: &mut Self::SharedState) {
        for relay in self.relay_stations.iter_mut() {
            relay.reset(env, shared_state);
        }
        self.consolidated_ground_station.reset(env, &mut ());

        for (sat_idx, sat) in self.satellites.iter_mut() {
            let spacecraft = env
                .telemetry
                .spacecrafts
                .get(sat_idx)
                .expect("Missing satellite from environment");
            let env = SatelliteEnvironment {
                sim_info: env.sim_info,
                timestamp: &env.telemetry.timestamp,
                fsw_data: &spacecraft.ac,
                ground_truth_ir_events: self.ground_truth_events_manager.active_events(),
            };
            sat.reset(&env, shared_state);
        }
    }

    fn step(&mut self, dt: Time, env: &SystemEnvironment, shared_state: &mut Self::SharedState) {
        if let Some(mut gui) = shared_state.gui.as_ref().map(|gui| gui.borrow_mut()) {
            gui.update_celestial_bodies(&env.telemetry.worlds);
        }

        self.ground_truth_events_manager.step(dt, env, shared_state);

        for (sat_idx, sat) in self.satellites.iter_mut() {
            let spacecraft = env
                .telemetry
                .spacecrafts
                .get(sat_idx)
                .expect("Missing satellite from environment");
            let env = SatelliteEnvironment {
                sim_info: env.sim_info,
                timestamp: &env.telemetry.timestamp,
                fsw_data: &spacecraft.ac,
                ground_truth_ir_events: self.ground_truth_events_manager.active_events(),
            };
            sat.step(dt, &env, shared_state);
        }

        for relay in self.relay_stations.iter_mut() {
            relay.step(dt, env, shared_state);
        }

        self.consolidated_ground_station.step(dt, env, &mut ());

        let _ = self.cgs_to_relay_ch.step();
        let _ = self.relay_to_cgs_ch.step();
        let _ = self.relay_to_sat_ch.step();
        let _ = self.sat_to_relay_ch.step();
    }
}

#[derive(Debug, Clone)]
pub struct SystemEnvironment<'a> {
    pub sim_info: &'a SimulationInfo,
    pub telemetry: &'a protocol42::Telemetry,
}

pub struct SystemSharedState {
    pub gui: Option<SharedGuiState>,
}

impl SystemSharedState {
    pub fn new(gui: Option<SharedGuiState>) -> Result<Self, IngestError> {
        Ok(SystemSharedState { gui })
    }
}

#[derive(Debug, Clone)]
pub struct SatToGroundMessage {
    /// Incremented by one for each telemetry message, by the satellite. Starts at 1.
    /// Ignored by the ground stations, so timing errors are easier to induce.
    pub seq: u64,
    pub satellite_id: SatCatId,
    pub satellite_name: &'static str,
    pub sat_send_timestamp: Timestamp,

    pub body: SatToGroundMessageBody,
}

#[derive(Debug, Clone)]
pub enum SatToGroundMessageBody {
    SatelliteTelemetry(Box<SatelliteTelemetry>),
    Detections(Detections),
}

#[derive(Debug, Clone, Default)]
pub struct SatelliteTelemetry {
    pub power: Option<PowerStatus>,
    pub gps: Option<NVector<f64>>,
    pub imu: Option<ImuSample>,
    pub imu_status: Option<ImuStatus>,
    pub vision_status: Option<VisionStatus>,
    pub comms_status: Option<CommsStatus>,
    pub compute_status: Option<ComputeStatus>,
}

impl SatelliteTelemetry {
    pub fn new() -> Self {
        SatelliteTelemetry {
            power: None,
            gps: None,
            imu: None,
            imu_status: None,
            vision_status: None,
            comms_status: None,
            compute_status: None,
        }
    }

    pub fn is_complete(&self) -> bool {
        let SatelliteTelemetry {
            power,
            gps,
            imu,
            imu_status,
            vision_status,
            comms_status,
            compute_status,
        } = self;

        power.is_some()
            && gps.is_some()
            && imu.is_some()
            && imu_status.is_some()
            && vision_status.is_some()
            && comms_status.is_some()
            && compute_status.is_some()
    }
}

impl TracedMessage for SatToGroundMessage {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        let mut b = AttrsBuilder::new();
        b.kv("event.satellite.id", self.satellite_id);
        b.kv("event.satellite.name", self.satellite_name);

        match &self.body {
            SatToGroundMessageBody::SatelliteTelemetry(telem) => {
                b.push_prefix("event");
                b.kv("name", "telemetry");

                if let Some(power) = telem.power {
                    b.with_prefix("power", |b| power.to_attrs(b));
                }

                if let Some(nvec) = telem.gps {
                    let pos: WGS84<f64> = nvec.into();
                    b.kv("gps.latitude", pos.latitude_degrees());
                    b.kv("gps.longitude", pos.longitude_degrees());
                    b.kv("gps.altitude", pos.altitude());
                }

                if let Some(imu) = &telem.imu {
                    b.with_prefix("imu", |b| imu.to_attrs(b));
                }

                if let Some(imu_status) = telem.imu_status {
                    b.with_prefix("imu", |b| imu_status.to_attrs(b));
                }

                if let Some(vision_status) = telem.vision_status {
                    b.with_prefix("vision", |b| vision_status.to_attrs(b));
                }

                if let Some(comms_status) = telem.comms_status {
                    b.with_prefix("comms", |b| comms_status.to_attrs(b));
                }

                if let Some(compute_status) = telem.compute_status {
                    b.with_prefix("compute", |b| compute_status.to_attrs(b));
                }
            }

            SatToGroundMessageBody::Detections(det) => {
                b.kv("event.name", "detections");
                b.kv("event.count", det.events.len() as i64);
            }
        }

        b.build()
    }
}

#[derive(Debug, Clone)]
pub enum GroundToSatMessage {
    PrioritizeIrEvent {
        sat: SatCatId,
        source_id: CameraSourceId,
    },
    ClearSatelliteErrorFlag {
        sat: SatCatId,
        flag: SatErrorFlag,
    },
}

impl GroundToSatMessage {
    pub fn name(&self) -> &str {
        match self {
            GroundToSatMessage::PrioritizeIrEvent { .. } => "prioritize_ir_event",
            GroundToSatMessage::ClearSatelliteErrorFlag { .. } => "clear_satellite_error_flag",
        }
    }

    pub fn destination(&self) -> SatCatId {
        match self {
            GroundToSatMessage::PrioritizeIrEvent { sat, .. } => *sat,
            GroundToSatMessage::ClearSatelliteErrorFlag { sat, .. } => *sat,
        }
    }
}

impl TracedMessage for GroundToSatMessage {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            GroundToSatMessage::PrioritizeIrEvent { sat, source_id } => {
                let mut kvs = vec![
                    kv("event.name", "prioritize_ir_event"),
                    kv("event.satellite.id", *sat),
                    kv("event.source_type", source_id.source_type()),
                ];
                if let Some(source_id) = source_id.source_id() {
                    kvs.push(kv("event.source_id", source_id));
                }
                kvs
            }
            GroundToSatMessage::ClearSatelliteErrorFlag { sat, flag } => vec![
                kv("event.name", "clear_satellite_error_flag"),
                kv("event.satellite.id", *sat),
                kv("event.error_flag", flag.name()),
            ],
        }
    }
}

#[derive(Debug, Clone)]
pub struct Detections {
    pub sat_timestamp: Timestamp,
    pub events: Vec<IREvent>,
}

impl TracedMessage for Detections {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        vec![
            kv("event.name", "detections"),
            kv("event.event_count", self.events.len() as i64),
        ]
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct IREvent {
    /// Fixed id statically assigned by the simulation scenario
    pub ground_truth_id: i64,
    /// ID generated by the camera source the originating object was detected by
    #[serde(skip)]
    pub source_id: CameraSourceId,
    /// Location
    #[serde(skip)]
    pub location: nav_types::NVector<f64>,
    /// Position in the world frame [m]
    pub position: Vector3<f64>,
    /// Linear velocity in the world frame [m/s]
    pub velocity: Vector3<f64>,
    /// Eastward velocity [m/s]
    #[serde(skip)]
    pub velocity_east: f64,
    /// Northward velocity [m/s]
    #[serde(skip)]
    pub velocity_north: f64,
    /// Luminous intensity
    pub intensity: LuminousIntensity,
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Serialize)]
pub enum CameraSourceId {
    /// Not assigned
    Unassigned,

    /// Wide-angle scanner camera
    Scanner(i64),

    /// Narrow-angle focus camera
    Focus(i64),
}

impl CameraSourceId {
    pub fn source_type(&self) -> &'static str {
        match self {
            CameraSourceId::Unassigned => "unassigned",
            CameraSourceId::Scanner(_) => "scanner",
            CameraSourceId::Focus(_) => "focus",
        }
    }

    pub fn source_id(&self) -> Option<i64> {
        match self {
            CameraSourceId::Unassigned => None,
            CameraSourceId::Scanner(n) => Some(*n),
            CameraSourceId::Focus(n) => Some(*n),
        }
    }
}

#[derive(Debug, Eq, PartialEq, Hash, Copy, Clone)]
pub enum SatErrorFlag {
    /// Watchdog detected out-of-sync execution, subsystem is halted
    CommsOutOfSync,

    /// Watchdog detected out-of-sync execution, subsystem is halted
    VisionOutOfSync,

    /// Watchdog detected out-of-sync execution, subsystem is halted
    PowerOutOfSync,

    /// Watchdog detected out-of-sync execution, subsystem is halted
    ImuOutOfSync,

    /// Running on the backup IMU, in a degraded state.
    ImuDegraded,

    /// There's an inconsistency between the primary and backup IMU.
    /// No IMU data will be sent.
    ImuDataInconsistency,
}

impl SatErrorFlag {
    pub fn name(&self) -> &'static str {
        match self {
            SatErrorFlag::CommsOutOfSync => "CommsOutOfSync",
            SatErrorFlag::VisionOutOfSync => "VisionOutOfSync",
            SatErrorFlag::PowerOutOfSync => "PowerOutOfSync",
            SatErrorFlag::ImuOutOfSync => "ImuOutOfSync",
            SatErrorFlag::ImuDegraded => "ImuDegraded",
            SatErrorFlag::ImuDataInconsistency => "ImuDataInconsistency",
        }
    }
}
