use modality_api::TimelineId;
use na::Vector3;
use nav_types::{NVector, ECEF, WGS84};

use crate::{
    channel::{Receiver, Sender, TracedMessage},
    modality::{kv, MODALITY},
    system::{GroundToSatMessage, SatToGroundMessage, SystemEnvironment, SystemSharedState},
    units::{Angle, Ratio, Time, Timestamp},
    SimulationComponent,
};

use super::Relayed;

pub type GroundStationId = u32;

/// A ground station on the earth
#[derive(Debug, Clone, PartialEq)]
pub struct RelayGroundStationConfig {
    pub id: GroundStationId,
    pub name: String,
    pub position: NVector<f64>,

    pub enable_time_sync: bool,
    pub rtc_drift: Ratio,
}

impl RelayGroundStationConfig {
    pub fn from_wgs84_degrees(
        id: GroundStationId,
        name: String,
        latitude: Angle,
        longitude: Angle,
    ) -> Self {
        Self {
            id,
            name,
            position: WGS84::from_degrees_and_meters(
                latitude.as_degrees(),
                longitude.as_degrees(),
                0.0, // MSL
            )
            .into(),
            enable_time_sync: true,
            rtc_drift: Ratio::from_f64(0.02),
        }
    }
}

pub struct RelayGroundStation {
    pub config: RelayGroundStationConfig,
    sat_rx: Receiver<SatToGroundMessage>,
    sat_tx: Sender<Relayed<GroundToSatMessage>>,
    cgs_rx: Receiver<GroundToSatMessage>,
    cgs_tx: Sender<Relayed<SatToGroundMessage>>,

    rtc: Timestamp,
    timeline: TimelineId,
}

impl RelayGroundStation {
    pub fn new(
        config: RelayGroundStationConfig,
        sat_rx: Receiver<SatToGroundMessage>,
        sat_tx: Sender<Relayed<GroundToSatMessage>>,
        cgs_rx: Receiver<GroundToSatMessage>,
        cgs_tx: Sender<Relayed<SatToGroundMessage>>,
    ) -> Self {
        Self {
            config,
            sat_rx,
            sat_tx,
            cgs_rx,
            cgs_tx,
            rtc: Timestamp::epoch(),
            timeline: TimelineId::allocate(),
        }
    }
}

impl<'a> SimulationComponent<'a> for RelayGroundStation {
    type Environment = SystemEnvironment<'a>;
    type SharedState = SystemSharedState;

    fn init(&mut self, env: &'a Self::Environment, common_state: &mut Self::SharedState) {
        self.rtc = env.sim_info.timestamp;

        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.rtc);
        let location = WGS84::from(self.config.position);
        let lat: f64 = location.latitude_degrees();
        let long: f64 = location.longitude_degrees();

        MODALITY.emit_timeline_attrs([
            kv(
                "timeline.name",
                format!("{} Relay Ground Station", self.config.name),
            ),
            kv("timeline.ground_station.name", self.config.name.as_str()),
            kv("timeline.latitude", lat),
            kv("timeline.longitude", long),
        ]);

        MODALITY.quick_event("init");

        if let Some(mut gui) = common_state.gui.as_ref().map(|gui| gui.borrow_mut()) {
            let pos = ECEF::from(self.config.position);
            gui.update_relay_ground_station(
                self.config.id,
                &Vector3::new(pos.x(), pos.y(), pos.z()),
            );
        }
    }

    fn reset(&mut self, _env: &'a Self::Environment, _shared_state: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.rtc);
        MODALITY.quick_event("reset");
    }

    fn step(
        &mut self,
        dt: Time,
        env: &'a Self::Environment,
        _common_state: &mut Self::SharedState,
    ) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.rtc);
        let timestamp = Timestamp::from_utc(env.telemetry.timestamp);
        if self.rtc < timestamp && self.config.enable_time_sync {
            // This is 'gps' time
            self.rtc = timestamp;
        } else {
            self.rtc += dt + (dt * self.config.rtc_drift);
        }

        // TODO consider adding relay station delays as a mutation
        relay(&mut self.sat_rx, &mut self.cgs_tx, self.config.id, self.rtc);
        relay(&mut self.cgs_rx, &mut self.sat_tx, self.config.id, self.rtc);
    }
}

fn relay<T: TracedMessage>(
    from: &mut Receiver<T>,
    to: &mut Sender<Relayed<T>>,
    id: GroundStationId,
    timestamp: Timestamp,
) {
    while let Some(msg) = from.recv() {
        let _ = to.try_send(Relayed {
            relay_ground_station_id: id,
            relay_timestamp: timestamp,
            inner: msg,
        });
    }
}
