use modality_api::{AttrType, TimelineId};
use modality_mutator_protocol::descriptor::owned::*;

use super::{RackId, RackSharedState};
use crate::{
    modality::{kv, MODALITY},
    mutator::GenericSetFloatMutator,
    system::SystemEnvironment,
    units::{Ratio, Time},
    SimulationComponent,
};

pub struct TimeSourceSubsystem {
    config: TimeSourceConfig,

    enable_time_sync: bool,
    have_gps_lock: bool,
    rtc_drift_ratio: Ratio,
    timeline: TimelineId,

    rtc_drift: Option<GenericSetFloatMutator>,
}

impl TimeSourceSubsystem {
    pub fn new(config: TimeSourceConfig) -> Self {
        Self {
            config,
            enable_time_sync: true,
            have_gps_lock: false,
            rtc_drift_ratio: Ratio::from_f64(0.0),
            timeline: TimelineId::allocate(),
            // Mutators are initialized in init_fault_models at sim-init time
            rtc_drift: None,
        }
    }

    fn init_fault_models(&mut self, id: RackId) {
        self.rtc_drift = self
            .config
            .fault_config
            .rtc_drift
            .then_some(GenericSetFloatMutator::new(OwnedMutatorDescriptor {
                name: "Consolidated ground station rack RTC drift"
                    .to_owned()
                    .into(),
                description: "Sets RTC drift ratio".to_owned().into(),
                layer: MutatorLayer::Implementational.into(),
                group: "consolidated_ground_station".to_owned().into(),
                operation: MutatorOperation::SetToValue.into(),
                statefulness: MutatorStatefulness::Permanent.into(),
                organization_custom_metadata: OrganizationCustomMetadata::new(
                    "consolidated_ground_station".to_string(),
                    [("rack.id".to_string(), (id as i64).into())]
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
    }

    fn update_fault_models(&mut self) {
        if let Some(active_mutation) = self.rtc_drift.as_ref().and_then(|m| m.active_mutation()) {
            self.enable_time_sync = false;
            self.rtc_drift_ratio = Ratio::from_f64(active_mutation);
        } else {
            self.enable_time_sync = true;
            self.rtc_drift_ratio = Ratio::from_f64(0.0);
        }
    }
}

#[derive(Debug, Clone)]
pub struct TimeSourceConfig {
    pub fault_config: TimeSourceFaultConfig,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct TimeSourceFaultConfig {
    /// Enable the RTC drift mutator.
    pub rtc_drift: bool,
}

impl<'a> SimulationComponent<'a> for TimeSourceSubsystem {
    type SharedState = RackSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, rack: &mut Self::SharedState) {
        if self.enable_time_sync {
            rack.rtc = env.sim_info.timestamp;
        }
        self.have_gps_lock = self.enable_time_sync;

        // NB: we intentionally do this after initializing the rtc
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.emit_rack_timeline_attrs("time_source", rack.id);
        MODALITY.quick_event("init");

        self.init_fault_models(rack.id);

        MODALITY.quick_event_attrs("gps_time_sync", [kv("event.gps_lock", self.have_gps_lock)]);
    }

    fn reset(&mut self, env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.quick_event("reset");

        if self.enable_time_sync {
            rack.rtc = env.sim_info.timestamp;
        }

        self.have_gps_lock = self.enable_time_sync;
        MODALITY.quick_event_attrs("gps_time_sync", [kv("event.gps_lock", self.have_gps_lock)]);
    }

    fn step(&mut self, dt: Time, env: &SystemEnvironment<'a>, rack: &mut RackSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);

        MODALITY.process_mutation_plane_messages([self.rtc_drift.as_mut()].into_iter());

        self.update_fault_models();

        match (self.enable_time_sync, self.have_gps_lock) {
            (false, true) => {
                self.have_gps_lock = true;
                MODALITY.quick_event_attrs("gps_time_sync", [kv("event.gps_lock", true)]);
            }

            (true, false) => {
                self.have_gps_lock = false;
                MODALITY.quick_event_attrs("gps_time_sync", [kv("event.gps_lock", false)]);
            }

            _ => (),
        }

        let timestamp = env.sim_info.timestamp;
        if rack.rtc < timestamp && self.enable_time_sync {
            // This is 'gps' time
            rack.rtc = timestamp;
        } else {
            rack.rtc += dt + (dt * self.rtc_drift_ratio);
        }
    }
}
