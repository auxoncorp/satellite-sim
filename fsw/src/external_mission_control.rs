use nav_types::WGS84;
use protocol42::parse_unstructured_tle_set;
use serde::Serialize;
use std::{fs, io, path::Path};

use crate::{
    ground_station::{GroundStationId, Relayed},
    satellite::{
        CommsStatus, ComputeStatus, ImuSample, ImuStatus, PowerStatus, VisionStatus, SATELLITE_IDS,
    },
    system::{IREvent, SatToGroundMessage, SatToGroundMessageBody},
    units::Timestamp,
};

pub trait MissionControlMessageType: Serialize {
    fn msg_type(&self) -> u8;
}

/// Length-and-type-prefixed JSON
/// Framing is a u32 le total length followed by u8 message type and JSON body
pub struct Message {}

impl Message {
    pub fn send<T: MissionControlMessageType, O: io::Write>(body: &T, stream: &mut O) {
        #[derive(Serialize)]
        struct Msg<'a, T> {
            msg_type: u8,
            #[serde(flatten)]
            body: &'a T,
        }

        let msg = Msg {
            msg_type: body.msg_type(),
            body,
        };
        let m = serde_json::to_vec(&msg).expect("JSON serialize");
        let msg_len = m.len() as u32 + 4;
        stream.write_all(&msg_len.to_le_bytes()).unwrap();
        stream.write_all(&m).expect("Mission control message send");
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct SecAndUSecTimestamp {
    pub sec: i64,
    pub usec: u32,
}

impl From<Timestamp> for SecAndUSecTimestamp {
    fn from(value: Timestamp) -> Self {
        let t = value.as_utc();
        SecAndUSecTimestamp {
            sec: t.timestamp(),
            usec: t.timestamp_subsec_micros(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Telemetry {
    pub seqnum: u64,
    pub sat_id: u64,
    pub sat_name: &'static str,
    pub sat_send_timestamp: SecAndUSecTimestamp,

    pub relay_ground_station_id: GroundStationId,
    pub relay_timestamp: SecAndUSecTimestamp,

    pub gps: Option<WGS84<f64>>,
    pub imu: Option<ImuSample>,

    pub power_status: Option<PowerStatus>,
    pub imu_status: Option<ImuStatus>,
    pub vision_status: Option<VisionStatus>,
    pub comms_status: Option<CommsStatus>,
    pub compute_status: Option<ComputeStatus>,
}

impl MissionControlMessageType for Telemetry {
    fn msg_type(&self) -> u8 {
        1
    }
}

impl Telemetry {
    pub fn from_sat_to_ground_message(m: &Relayed<SatToGroundMessage>) -> Option<Self> {
        if let SatToGroundMessageBody::SatelliteTelemetry(telem) = &m.inner.body {
            Some(Telemetry {
                seqnum: m.inner.seq,
                sat_id: m.inner.satellite_id.into(),
                sat_name: m.inner.satellite_name,
                sat_send_timestamp: m.inner.sat_send_timestamp.into(),
                relay_ground_station_id: m.relay_ground_station_id,
                relay_timestamp: m.relay_timestamp.into(),
                power_status: telem.power,
                gps: telem.gps.map(|nvec| nvec.into()),
                imu: telem.imu.clone(),
                imu_status: telem.imu_status,
                vision_status: telem.vision_status,
                comms_status: telem.comms_status,
                compute_status: telem.compute_status,
            })
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Tle<'a> {
    pub sat_id: u64,
    pub sat_name: &'static str,
    pub line1: &'a str,
    pub line2: &'a str,
}

impl<'a> MissionControlMessageType for Tle<'a> {
    fn msg_type(&self) -> u8 {
        2
    }
}

// We only expect to send this once on startup, so it does all the parsing and IO upfront
pub fn send_tle_set<P: AsRef<Path>, O: io::Write>(tle_entries_path: P, stream: &mut O) {
    let tle_entries =
        fs::read_to_string(tle_entries_path).expect("Failed to read TLE entries file");
    let (_, tle_set) =
        parse_unstructured_tle_set(&tle_entries).expect("Failed to parse TLE entries file");
    for tle in tle_set.into_iter() {
        let id = SATELLITE_IDS
            .iter()
            .find(|id| id.name.contains(&tle.satellite_name))
            .expect("Failed to find TLE by name");
        Message::send(
            &Tle {
                sat_id: id.satcat_id.into(),
                sat_name: id.name,
                line1: &tle.line1,
                line2: &tle.line2,
            },
            stream,
        );
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct GroundTruthIREvent<'a> {
    pub seqnum: u64,
    pub timestamp: SecAndUSecTimestamp,
    #[serde(flatten)]
    pub event: &'a IREvent,
}

impl<'a> MissionControlMessageType for GroundTruthIREvent<'a> {
    fn msg_type(&self) -> u8 {
        3
    }
}
