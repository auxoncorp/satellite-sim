use modality_api::TimelineId;
use modality_mutator_protocol::descriptor::owned::*;
use na::Vector3;
use nav_types::NVector;
use std::collections::VecDeque;

use super::{AnalyzedIREvents, IREventClass, RackId, RackSharedState};
use crate::{
    channel::{Receiver, Sender},
    event,
    ground_station::{
        consolidated::{GlobalIRView, TrackedEventId, TrackedIREvent},
        OperationalStatus,
    },
    modality::{kv, MODALITY},
    mutator::GenericBooleanMutator,
    recv,
    system::{IREvent, SystemEnvironment},
    try_send,
    units::{Length, LuminousIntensity, Time, Timestamp, Velocity},
    SimulationComponent,
};

pub struct SynthesisSubsystem {
    config: SynthesisConfig,
    analyzed_rx: Receiver<AnalyzedIREvents>,
    synthesized_tx: Sender<GlobalIRView>,

    tracked_events: Vec<TrackedIREvent>,
    next_event_id: TrackedEventId,
    last_report_time: Timestamp,
    timeline: TimelineId,

    rack_offline: Option<GenericBooleanMutator>,
}

#[derive(Debug, Clone)]
pub struct SynthesisConfig {
    /// How frequently should the synthesis module send out full reports?
    pub report_interval: Time,

    /// How long just observations stick around in the 'recent' list,
    /// until one of them has been seen again?
    pub observation_timeout: Time,

    pub collapse_instensity_threshold: LuminousIntensity,
    pub collapse_velocity_threshold: Velocity,
    pub collapse_position_threshold: Length,

    pub fault_config: SynthesisFaultConfig,
}

#[derive(Debug, Clone, Default)]
pub struct SynthesisFaultConfig {
    /// Enable the rack offline mutator.
    pub rack_offline: bool,
}

impl SynthesisSubsystem {
    pub fn new(
        config: SynthesisConfig,
        analyzed_rx: Receiver<AnalyzedIREvents>,
        synthesized_tx: Sender<GlobalIRView>,
    ) -> Self {
        Self {
            config,
            analyzed_rx,
            synthesized_tx,
            tracked_events: vec![],
            next_event_id: 0,
            last_report_time: Timestamp::epoch(),
            timeline: TimelineId::allocate(),
            // Mutators are initialized in init_fault_models at sim-init time
            rack_offline: None,
        }
    }

    fn init_fault_models(&mut self, id: RackId) {
        // Construct mutators at sim-init time so we have access to SatelliteId/env details
        // to qualify the mutator descriptors
        self.rack_offline =
            self.config
                .fault_config
                .rack_offline
                .then_some(GenericBooleanMutator::new(OwnedMutatorDescriptor {
                    name: "Consolidated ground station rack offline".to_owned().into(),
                    description: "Takes a rack offline".to_owned().into(),
                    layer: MutatorLayer::Implementational.into(),
                    group: "consolidated_ground_station".to_owned().into(),
                    operation: MutatorOperation::Disable.into(),
                    statefulness: MutatorStatefulness::Transient.into(),
                    organization_custom_metadata: OrganizationCustomMetadata::new(
                        "consolidated_ground_station".to_string(),
                        [("rack.id".to_string(), (id as i64).into())]
                            .into_iter()
                            .collect(),
                    ),
                    params: Default::default(),
                }));

        if let Some(m) = &self.rack_offline {
            MODALITY.register_mutator(m);
        }
    }
}

fn recency_biased_average<T: Into<f64> + From<f64>>(points: impl Iterator<Item = T>) -> T {
    let mut next_bias = 1.0;
    let mut sum = 0.0;
    let mut count = 0.0;
    for p in points {
        sum += p.into() * next_bias;
        count += next_bias;
        next_bias /= 2.0;
    }

    T::from(sum / count)
}

impl TrackedIREvent {
    /// Based on the observations that went into this event, how trustworthy is it on an operational basis?
    #[allow(dead_code)]
    pub fn operational_status(&self) -> OperationalStatus {
        let observation_count = self.observations.len();

        #[allow(clippy::comparison_chain)]
        if observation_count > 2 {
            OperationalStatus::Nominal
        } else if observation_count == 2 {
            OperationalStatus::Degraded
        } else {
            OperationalStatus::NotTrustworthy
        }
    }

    pub fn classification(&self) -> IREventClass {
        // just take the first one, I guess?
        self.observations.front().unwrap().classification
    }

    pub fn confidence(&self) -> f64 {
        recency_biased_average(self.observations.iter().map(|obs| obs.confidence))
    }

    pub fn position(&self) -> Vector3<f64> {
        let x = recency_biased_average(self.observations.iter().map(|obs| obs.event.position[0]));
        let y = recency_biased_average(self.observations.iter().map(|obs| obs.event.position[1]));
        let z = recency_biased_average(self.observations.iter().map(|obs| obs.event.position[2]));
        Vector3::new(x, y, z)
    }

    #[allow(dead_code)]
    pub fn location(&self) -> NVector<f64> {
        let x = recency_biased_average(
            self.observations
                .iter()
                .map(|obs| obs.event.location.vector()[0]),
        );
        let y = recency_biased_average(
            self.observations
                .iter()
                .map(|obs| obs.event.location.vector()[1]),
        );
        let z = recency_biased_average(
            self.observations
                .iter()
                .map(|obs| obs.event.location.vector()[2]),
        );
        let a = recency_biased_average(
            self.observations
                .iter()
                .map(|obs| obs.event.location.altitude()),
        );
        NVector::new(Vector3::new(x, y, z), a)
    }

    pub fn velocity(&self) -> Vector3<f64> {
        let x = recency_biased_average(self.observations.iter().map(|obs| obs.event.velocity[0]));
        let y = recency_biased_average(self.observations.iter().map(|obs| obs.event.velocity[1]));
        let z = recency_biased_average(self.observations.iter().map(|obs| obs.event.velocity[2]));
        Vector3::new(x, y, z)
    }

    #[allow(dead_code)]
    pub fn velocity_east(&self) -> f64 {
        recency_biased_average(self.observations.iter().map(|obs| obs.event.velocity_east))
    }

    #[allow(dead_code)]
    pub fn velocity_north(&self) -> f64 {
        recency_biased_average(self.observations.iter().map(|obs| obs.event.velocity_north))
    }

    #[allow(dead_code)]
    pub fn intensity(&self) -> LuminousIntensity {
        recency_biased_average(self.observations.iter().map(|obs| obs.event.intensity))
    }

    fn should_include_event_based_on_parameters(
        &self,
        ev: &IREvent,
        config: &SynthesisConfig,
    ) -> bool {
        // only consider the most recent observation
        let Some(newest_observation) = self.observations.front() else {
            return false;
        };

        let intensity_difference = (newest_observation.event.intensity - ev.intensity).abs();

        let velocity_difference = Velocity::from_meters_per_second(
            (newest_observation.event.velocity - ev.velocity)
                .norm()
                .abs(),
        );

        let position_difference = Length::from_meters(
            (newest_observation.event.position - ev.position)
                .norm()
                .abs(),
        );

        intensity_difference < config.collapse_instensity_threshold
            && velocity_difference < config.collapse_velocity_threshold
            && position_difference < config.collapse_position_threshold
    }

    fn time_out_old_observations(&mut self, observation_timeout: Time, rtc: Timestamp) {
        loop {
            let remove_last = if let Some(oldest_observation) = self.observations.back() {
                rtc - oldest_observation.rack_timestamp > observation_timeout
            } else {
                false
            };

            if remove_last {
                if let Some(old) = self.observations.pop_back() {
                    let mut kvs = vec![
                        kv("event.tracked_event_id", self.id as i64),
                        kv("event.satellite.id", old.based_on_sat_camera.0),
                        kv("event.camera.type", old.based_on_sat_camera.1.source_type()),
                    ];

                    if let Some(source_id) = old.based_on_sat_camera.1.source_id() {
                        kvs.push(kv("event.camera.id", source_id));
                    }

                    event!("time_out_old_observation", kvs);
                }
            } else {
                return;
            }
        }
    }
}

impl<'a> SimulationComponent<'a> for SynthesisSubsystem {
    type SharedState = RackSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.emit_rack_timeline_attrs("synthesis", rack.id);
        event!("init");
        self.init_fault_models(rack.id);
    }

    fn reset(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        event!("reset");

        if let Some(m) = self.rack_offline.as_mut() {
            MODALITY.clear_mutation(m);
        }
    }

    fn step(&mut self, _dt: Time, _env: &SystemEnvironment<'a>, rack: &mut RackSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);

        MODALITY.process_mutation_plane_messages([self.rack_offline.as_mut()].into_iter());

        let rack_offline = self
            .rack_offline
            .as_ref()
            .map(|m| m.is_active())
            .unwrap_or(false);

        while let Some(msg) = recv!(&mut self.analyzed_rx) {
            if rack_offline {
                // Drop ingress
                // NOTE: we could ignore and let the channel fill up
                continue;
            }

            for ev in msg.events.into_iter() {
                // If this new observation has parameters consistent with an existing tracked event...
                if let Some(tracked) = self.tracked_events.iter_mut().find(|tracked| {
                    tracked.should_include_event_based_on_parameters(&ev.event, &self.config)
                }) {
                    // ... then add it as another supporting observation for the same event
                    tracked.observations.push_front(ev);
                } else {
                    event!(
                        "new_tracked_event",
                        [kv("event.tracked_event_id", self.next_event_id as i64)],
                    );

                    // Otherwise, we're creating a new tracked event.
                    let mut observations = VecDeque::new();
                    observations.push_front(ev);
                    self.tracked_events.push(TrackedIREvent {
                        id: self.next_event_id,
                        observations,
                    });
                    self.next_event_id += 1;
                }
            }
        }

        if !rack_offline {
            for tracked in self.tracked_events.iter_mut() {
                tracked.time_out_old_observations(self.config.observation_timeout, rack.rtc);
            }

            // Remove any events that no longer have any supporting observations
            self.tracked_events.retain(|tracked| {
                let should_keep = !tracked.observations.is_empty();
                if !should_keep {
                    event!(
                        "expire_tracked_event",
                        [kv("event.tracked_event_id", tracked.id as i64)],
                    );
                }
                should_keep
            });

            let next_report_time = self.last_report_time + self.config.report_interval;
            if next_report_time < rack.rtc {
                let _ = try_send!(
                    &mut self.synthesized_tx,
                    GlobalIRView {
                        rack_id: rack.id,
                        rack_timestamp: rack.rtc,
                        events: self.tracked_events.clone(),
                    }
                );
                self.last_report_time = rack.rtc;
            }
        }
    }
}
