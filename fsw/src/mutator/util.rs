use crate::{satellite::SatelliteId, units::Time};
use modality_api::AttrType;
use modality_mutator_protocol::descriptor::owned::*;
use std::collections::HashMap;

pub fn watchdog_out_of_sync_descriptor(
    component_name: &str,
    id: &SatelliteId,
) -> OwnedMutatorDescriptor {
    OwnedMutatorDescriptor {
        name: "Watchdog execution out-of-sync error".to_owned().into(),
        description: "Sets the watchdog execution out-of-sync error register bit"
            .to_owned()
            .into(),
        layer: MutatorLayer::Implementational.into(),
        group: component_name.to_owned().into(),
        operation: MutatorOperation::Enable.into(),
        statefulness: MutatorStatefulness::Transient.into(),
        organization_custom_metadata: OrganizationCustomMetadata::new(
            "satellite".to_string(),
            HashMap::from([
                ("id".to_string(), id.satcat_id.into()),
                ("name".to_string(), id.name.into()),
                ("component_name".to_string(), component_name.into()),
            ]),
        ),
        params: Default::default(),
    }
}

pub fn constant_temperature_descriptor(
    component_name: &str,
    id: &SatelliteId,
) -> OwnedMutatorDescriptor {
    OwnedMutatorDescriptor {
        name: "Set temperature".to_owned().into(),
        description: "Set the temperature sensor reading to a contant value"
            .to_owned()
            .into(),
        layer: MutatorLayer::Implementational.into(),
        group: component_name.to_owned().into(),
        operation: MutatorOperation::SetToValue.into(),
        statefulness: MutatorStatefulness::Transient.into(),
        organization_custom_metadata: OrganizationCustomMetadata::new(
            "satellite".to_string(),
            HashMap::from([
                ("id".to_string(), id.satcat_id.into()),
                ("name".to_string(), id.name.into()),
                ("component_name".to_string(), component_name.into()),
            ]),
        ),
        params: vec![
            OwnedMutatorParamDescriptor::new(AttrType::Float, "temperature".to_owned())
                .unwrap()
                .with_description("Constant temperature [degrees Celsius]")
                .with_value_min(-500.0)
                .with_value_max(500.0),
            // NOTE: could surface theses in config
        ],
    }
}

pub fn disable_vision_stabilizer_descriptor(
    component_name: &str,
    id: &SatelliteId,
) -> OwnedMutatorDescriptor {
    OwnedMutatorDescriptor {
        name: "Disable vision stabilizer".to_owned().into(),
        description: "Disables the vision subsystem orientation stabilizer"
            .to_owned()
            .into(),
        layer: MutatorLayer::Implementational.into(),
        group: component_name.to_owned().into(),
        operation: MutatorOperation::Enable.into(),
        statefulness: MutatorStatefulness::Transient.into(),
        organization_custom_metadata: OrganizationCustomMetadata::new(
            "satellite".to_string(),
            HashMap::from([
                ("id".to_string(), id.satcat_id.into()),
                ("name".to_string(), id.name.into()),
                ("component_name".to_string(), component_name.into()),
            ]),
        ),
        params: Default::default(),
    }
}

/// A simulation-relative-time timer
#[derive(Debug)]
pub struct SimTimer {
    /// Simulation relative time the timer was started at
    started_at: Option<Time>,
    duration: Time,
}

impl Default for SimTimer {
    fn default() -> Self {
        Self {
            started_at: None,
            duration: Time::from_secs(0.0),
        }
    }
}

impl SimTimer {
    pub fn start(&mut self, now: Time, duration: Time) {
        assert!(now.as_secs() >= 0.0);
        assert!(duration.as_secs() > 0.0);
        self.started_at = Some(now);
        self.duration = duration;
    }

    pub fn stop(&mut self) {
        self.started_at = None;
    }

    pub fn restart(&mut self, now: Time) {
        self.start(now, self.duration);
    }

    pub fn is_started(&self) -> bool {
        self.started_at.is_some()
    }

    pub fn is_expired(&self, now: Time) -> bool {
        if let Some(started_at) = self.started_at {
            assert!(now >= started_at);
            (now - started_at) >= self.duration
        } else {
            // Not started
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_timer() {
        let mut timer = SimTimer::default();

        assert!(!timer.is_started());
        assert!(!timer.is_expired(Time::from_secs(0.0)));

        for base in 0..5 {
            let t0 = Time::from_secs(base as f64);
            timer.start(t0, Time::from_secs(10.0));
            timer.restart(t0);
            assert!(timer.is_started());
            assert!(!timer.is_expired(t0 + Time::from_secs(0.0)));
            assert!(!timer.is_expired(t0 + Time::from_secs(5.0)));
            assert!(timer.is_expired(t0 + Time::from_secs(10.0)));
        }

        timer.stop();
        assert!(!timer.is_started());
        assert!(!timer.is_expired(Time::from_secs(20.0)));
    }
}
