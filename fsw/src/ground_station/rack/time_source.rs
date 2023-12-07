use modality_api::TimelineId;

use super::RackSharedState;
use crate::{
    modality::{kv, MODALITY},
    system::SystemEnvironment,
    units::{Ratio, Time},
    SimulationComponent,
};

pub struct TimeSourceSubsystem {
    config: TimeSourceConfig,

    have_gps_lock: bool,
    timeline: TimelineId,
}

impl TimeSourceSubsystem {
    pub fn new(config: TimeSourceConfig) -> Self {
        Self {
            config,
            have_gps_lock: false,
            timeline: TimelineId::allocate(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TimeSourceConfig {
    pub enable_time_sync: bool,
    pub rtc_drift: Ratio,
}

impl<'a> SimulationComponent<'a> for TimeSourceSubsystem {
    type SharedState = RackSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, rack: &mut Self::SharedState) {
        if self.config.enable_time_sync {
            rack.rtc = env.sim_info.timestamp;
        }
        self.have_gps_lock = self.config.enable_time_sync;

        // NB: we intentionally do this after initializing the rtc
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.emit_rack_timeline_attrs("time_source", rack.id);
        MODALITY.quick_event("init");

        MODALITY.quick_event_attrs("gps_time_sync", [kv("event.gps_lock", self.have_gps_lock)]);
    }

    fn reset(&mut self, env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.quick_event("reset");

        if self.config.enable_time_sync {
            rack.rtc = env.sim_info.timestamp;
        }

        self.have_gps_lock = self.config.enable_time_sync;
        MODALITY.quick_event_attrs("gps_time_sync", [kv("event.gps_lock", self.have_gps_lock)]);
    }

    fn step(&mut self, dt: Time, env: &SystemEnvironment<'a>, rack: &mut RackSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);

        match (self.config.enable_time_sync, self.have_gps_lock) {
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
        if rack.rtc < timestamp && self.config.enable_time_sync {
            // This is 'gps' time
            rack.rtc = timestamp;
        } else {
            rack.rtc += dt + (dt * self.config.rtc_drift);
        }
    }
}
