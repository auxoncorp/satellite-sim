use modality_api::{AttrType, TimelineId};
use modality_mutator_protocol::descriptor::owned::*;
use na::Vector3;
use nav_types::{NVector, ECEF, WGS84};

use crate::{
    channel::{Receiver, Sender, TracedMessage},
    modality::{kv, MODALITY},
    mutator::{GenericSetFloatMutator, MutatorActuatorDescriptor, SimTimer},
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
    pub fault_config: RelayGroundStationFaultConfig,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct RelayGroundStationFaultConfig {
    /// Enable the RTC drift mutator.
    pub rtc_drift: bool,

    /// Enable the satellite-to-CGS communications delay mutator.
    pub satellite_to_cgs_delay: bool,

    /// Enable the CGS-to-satellite communications delay mutator.
    pub cgs_to_satellite_delay: bool,
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
            fault_config: Default::default(),
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
    enable_time_sync: bool,
    rtc_drift_ratio: Ratio,
    satellite_to_cgs_delay_timer: SimTimer,
    cgs_to_satellite_delay_timer: SimTimer,

    rtc_drift: Option<GenericSetFloatMutator>,
    satellite_to_cgs_delay: Option<GenericSetFloatMutator>,
    cgs_to_satellite_delay: Option<GenericSetFloatMutator>,
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
            enable_time_sync: true,
            rtc_drift_ratio: Ratio::from_f64(0.0),
            satellite_to_cgs_delay_timer: Default::default(),
            cgs_to_satellite_delay_timer: Default::default(),
            // Mutators are initialized in init_fault_models at sim-init time
            rtc_drift: None,
            satellite_to_cgs_delay: None,
            cgs_to_satellite_delay: None,
        }
    }

    fn init_fault_models(&mut self) {
        self.rtc_drift = self
            .config
            .fault_config
            .rtc_drift
            .then_some(GenericSetFloatMutator::new(OwnedMutatorDescriptor {
                name: "Relay Ground Station RTC drift".to_owned().into(),
                description: "Sets RTC drift ratio".to_owned().into(),
                layer: MutatorLayer::Implementational.into(),
                group: "relay_ground_station".to_owned().into(),
                operation: MutatorOperation::SetToValue.into(),
                statefulness: MutatorStatefulness::Permanent.into(),
                organization_custom_metadata: OrganizationCustomMetadata::new(
                    "relay_ground_station".to_string(),
                    [
                        ("id".to_string(), self.config.id.into()),
                        ("name".to_string(), self.config.name.clone().into()),
                    ]
                    .into_iter()
                    .collect(),
                ),
                params: vec![OwnedMutatorParamDescriptor::new(
                    AttrType::Float,
                    "drift_ratio".to_owned(),
                )
                .unwrap()
                .with_description("RTC drift ratio [normalized]")
                .with_value_min(0.0)
                .with_value_max(1.0)
                .with_least_effect_value(0.0)],
            }));

        if let Some(m) = &self.rtc_drift {
            MODALITY.register_mutator(m);
        }

        self.satellite_to_cgs_delay =
            self.config
                .fault_config
                .satellite_to_cgs_delay
                .then_some(GenericSetFloatMutator::new(OwnedMutatorDescriptor {
                    name: "Satellite-to-CGS communications delay".to_owned().into(),
                    description: "Applies a delay in satellite-to-CGS communications"
                        .to_owned()
                        .into(),
                    layer: MutatorLayer::Implementational.into(),
                    group: "relay_ground_station".to_owned().into(),
                    operation: MutatorOperation::Delay.into(),
                    statefulness: MutatorStatefulness::Permanent.into(),
                    organization_custom_metadata: OrganizationCustomMetadata::new(
                        "relay_ground_station".to_string(),
                        [
                            ("id".to_string(), self.config.id.into()),
                            ("name".to_string(), self.config.name.clone().into()),
                        ]
                        .into_iter()
                        .collect(),
                    ),
                    params: vec![OwnedMutatorParamDescriptor::new(
                        AttrType::Float,
                        "delay".to_owned(),
                    )
                    .unwrap()
                    .with_description("Amount of simulation time to delay [seconds]")
                    .with_value_min(0.0)
                    .with_least_effect_value(0.0)],
                }));

        if let Some(m) = &self.satellite_to_cgs_delay {
            MODALITY.register_mutator(m);
        }

        self.cgs_to_satellite_delay =
            self.config
                .fault_config
                .cgs_to_satellite_delay
                .then_some(GenericSetFloatMutator::new(OwnedMutatorDescriptor {
                    name: "CGS-to-satellite communications delay".to_owned().into(),
                    description: "Applies a delay in CGS-to-satellite communications"
                        .to_owned()
                        .into(),
                    layer: MutatorLayer::Implementational.into(),
                    group: "relay_ground_station".to_owned().into(),
                    operation: MutatorOperation::Delay.into(),
                    statefulness: MutatorStatefulness::Permanent.into(),
                    organization_custom_metadata: OrganizationCustomMetadata::new(
                        "relay_ground_station".to_string(),
                        [
                            ("id".to_string(), self.config.id.into()),
                            ("name".to_string(), self.config.name.clone().into()),
                        ]
                        .into_iter()
                        .collect(),
                    ),
                    params: vec![OwnedMutatorParamDescriptor::new(
                        AttrType::Float,
                        "delay".to_owned(),
                    )
                    .unwrap()
                    .with_description("Amount of simulation time to delay [seconds]")
                    .with_value_min(0.0)
                    .with_least_effect_value(0.0)],
                }));

        if let Some(m) = &self.cgs_to_satellite_delay {
            MODALITY.register_mutator(m);
        }
    }

    fn update_fault_models(&mut self, rel_time: Time) {
        self.enable_time_sync = true;

        if let Some(active_mutation) = self.rtc_drift.as_ref().and_then(|m| m.active_mutation()) {
            self.enable_time_sync = false;
            self.rtc_drift_ratio = Ratio::from_f64(active_mutation);
        } else {
            self.rtc_drift_ratio = Ratio::from_f64(0.0);
        }

        if let Some(active_mutation) = self
            .satellite_to_cgs_delay
            .as_ref()
            .and_then(|m| m.active_mutation())
        {
            if !self.satellite_to_cgs_delay_timer.is_started() {
                let duration_to_delay = Time::from_secs(active_mutation);
                self.satellite_to_cgs_delay_timer
                    .start(rel_time, duration_to_delay);
            }
        } else {
            self.satellite_to_cgs_delay_timer.stop();
        }

        if let Some(active_mutation) = self
            .cgs_to_satellite_delay
            .as_ref()
            .and_then(|m| m.active_mutation())
        {
            if !self.cgs_to_satellite_delay_timer.is_started() {
                let duration_to_delay = Time::from_secs(active_mutation);
                self.cgs_to_satellite_delay_timer
                    .start(rel_time, duration_to_delay);
            }
        } else {
            self.cgs_to_satellite_delay_timer.stop();
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

        self.init_fault_models();

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

        let mutators = [
            self.rtc_drift.as_mut().map(|m| m.as_dyn()),
            self.satellite_to_cgs_delay.as_mut().map(|m| m.as_dyn()),
            self.cgs_to_satellite_delay.as_mut().map(|m| m.as_dyn()),
        ];

        MODALITY.process_mutation_plane_messages(mutators.into_iter());

        self.update_fault_models(env.sim_info.relative_time);

        let timestamp = Timestamp::from_utc(env.telemetry.timestamp);

        if self.rtc < timestamp && self.enable_time_sync {
            // This is 'gps' time
            self.rtc = timestamp;
        } else {
            self.rtc += dt + (dt * self.rtc_drift_ratio);
        }

        let mut delay_relay_comms = self.satellite_to_cgs_delay_timer.is_started();
        if self
            .satellite_to_cgs_delay_timer
            .is_expired(env.sim_info.relative_time)
        {
            self.satellite_to_cgs_delay_timer
                .restart(env.sim_info.relative_time);
            // Send when we've buffered each timer expiration
            delay_relay_comms = false;
        }

        if !delay_relay_comms {
            relay(&mut self.sat_rx, &mut self.cgs_tx, self.config.id, self.rtc);
        }

        let mut delay_relay_comms = self.cgs_to_satellite_delay_timer.is_started();
        if self
            .cgs_to_satellite_delay_timer
            .is_expired(env.sim_info.relative_time)
        {
            self.cgs_to_satellite_delay_timer
                .restart(env.sim_info.relative_time);
            // Send when we've buffered each timer expiration
            delay_relay_comms = false;
        }

        if !delay_relay_comms {
            relay(&mut self.cgs_rx, &mut self.sat_tx, self.config.id, self.rtc);
        }
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
