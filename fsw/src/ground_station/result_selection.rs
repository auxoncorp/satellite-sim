use std::{
    collections::{hash_map::RandomState, HashMap},
    hash::{BuildHasher, Hash, Hasher},
};

use modality_api::TimelineId;
use na::Vector3;
use ordered_float::OrderedFloat;

use super::{
    consolidated::{GlobalIRView, SelectedGlobalIRView},
    rack::IREventClass,
    OperationalStatus,
};
use crate::{
    channel::{Receiver, Sender},
    modality::{kv, MODALITY},
    system::SystemEnvironment,
    units::{Time, Timestamp},
    SimulationComponent,
};

/// Predict IR event class based on velocity
pub struct ResultSelectionSubsystem {
    config: ResultSelectionConfig,
    racks_rx: Receiver<GlobalIRView>,
    selected_tx: Sender<SelectedGlobalIRView>,

    timeline: TimelineId,

    /// Just to track the relative passage of time, for timeouts
    relative_rtc: Timestamp,

    // None if there is nothing currently pending
    pending_batch_start_time: Option<Timestamp>,

    pending_batch: Vec<GlobalIRView>,
}

pub struct ResultSelectionConfig {
    /// After receiving data from a rack, wait this long for candidates from the others.
    pub selection_rx_window: Time,
}

impl ResultSelectionSubsystem {
    pub fn new(
        config: ResultSelectionConfig,
        racks_rx: Receiver<GlobalIRView>,
        selected_tx: Sender<SelectedGlobalIRView>,
    ) -> Self {
        Self {
            config,
            racks_rx,
            selected_tx,
            timeline: TimelineId::allocate(),
            relative_rtc: Timestamp::epoch(),
            pending_batch_start_time: None,
            pending_batch: vec![],
        }
    }
}

impl<'a> SimulationComponent<'a> for ResultSelectionSubsystem {
    type SharedState = ();
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, _: &mut Self::SharedState) {
        self.relative_rtc = env.sim_info.timestamp;
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.relative_rtc);
        MODALITY.emit_timeline_attrs([
            kv("timeline.name", "result_selection"),
            kv("timeline.ground_station.name", "consolidated"),
        ]);

        MODALITY.quick_event("init");
    }

    fn reset(&mut self, _env: &'a Self::Environment, _: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.relative_rtc);
        MODALITY.quick_event("reset");
    }

    fn step(&mut self, dt: Time, _env: &SystemEnvironment<'a>, _: &mut ()) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.relative_rtc);

        self.relative_rtc += dt;
        while let Some(msg) = self.racks_rx.recv() {
            if self
                .pending_batch
                .iter()
                .any(|view| view.rack_id == msg.rack_id)
            {
                // drop multiple messages from the same rack inside the selection window
                continue;
            }

            if self.pending_batch_start_time.is_none() {
                self.pending_batch_start_time = Some(self.relative_rtc);
            }

            self.pending_batch.push(msg);
        }

        if let Some(batch_start_time) = self.pending_batch_start_time {
            let batch_end = batch_start_time + self.config.selection_rx_window;
            if batch_end < self.relative_rtc {
                self.pending_batch_start_time = None;

                // group the global views by equivilancy
                let mut views_by_fingerprint = HashMap::new();
                let s = RandomState::new();
                for view in self.pending_batch.drain(..) {
                    let fp = global_ir_view_fingerprint(&s, &view);
                    views_by_fingerprint
                        .entry(fp)
                        .or_insert_with(Vec::new)
                        .push(view);
                }

                // Now sort them so the ones that occur most frequently come last
                let mut frequency_and_view_vec = views_by_fingerprint
                    .into_values()
                    .map(|mut view_vec| (view_vec.len(), view_vec.pop().unwrap()))
                    .collect::<Vec<_>>();
                frequency_and_view_vec.sort_by_key(|(count, _view)| *count);

                let (largest_freq, view) = frequency_and_view_vec.pop().unwrap();
                let status = {
                    if largest_freq == 3 {
                        // All 3 racks agree
                        OperationalStatus::Nominal
                    } else if largest_freq == 2 {
                        // One of the racks is different from the other two
                        OperationalStatus::Degraded
                    } else {
                        // Each rack says something different
                        OperationalStatus::NotTrustworthy
                    }
                };

                let selection = SelectedGlobalIRView { view, status };

                let _ = self.selected_tx.try_send(selection);
            }
        }
    }
}

// RandomState is externalized so we can get consistent hashes between the different views
fn global_ir_view_fingerprint(s: &RandomState, view: &GlobalIRView) -> u64 {
    let mut hasher = s.build_hasher();
    for ev in view.events.iter() {
        let fp = EventFingerprint {
            position: ev.position().map(OrderedFloat::from),
            location_vec: ev.location().vector().map(OrderedFloat::from),
            location_alt: OrderedFloat::from(ev.location().altitude()),
            velocity: ev.velocity().map(OrderedFloat::from),
            intensity: OrderedFloat::from(ev.intensity().as_candelas()),
            class: ev.classification(),
            confidence: OrderedFloat::from(ev.confidence()),
        };
        fp.hash(&mut hasher);
    }

    hasher.finish()
}

#[derive(PartialEq, Eq, Hash)]
struct EventFingerprint {
    position: Vector3<OrderedFloat<f64>>,
    location_vec: Vector3<OrderedFloat<f64>>,
    location_alt: OrderedFloat<f64>,
    velocity: Vector3<OrderedFloat<f64>>,
    intensity: OrderedFloat<f64>,
    class: IREventClass,
    confidence: OrderedFloat<f64>,
}
