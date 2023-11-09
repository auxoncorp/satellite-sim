use crate::units::{Time, Timestamp};

#[derive(Debug, Clone)]
pub struct SimulationInfo {
    pub timestamp: Timestamp,
    pub sim_iteration: u64,
    pub fsw_iteration: u64,
    pub relative_time: Time,
}

impl Default for SimulationInfo {
    fn default() -> Self {
        Self::new()
    }
}

impl SimulationInfo {
    pub fn new() -> Self {
        SimulationInfo {
            timestamp: Timestamp::epoch(),
            sim_iteration: 0,
            fsw_iteration: 0,
            relative_time: Time::from_secs(0.0),
        }
    }

    pub fn sim_step(&mut self, timestamp: Timestamp) {
        self.sim_iteration += 1;
        self.timestamp = timestamp;
    }

    pub fn fsw_step(&mut self, dt: Time) {
        self.fsw_iteration += 1;
        self.relative_time += dt;
    }
}
