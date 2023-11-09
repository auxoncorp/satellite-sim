use modality_api::TimelineId;
use ordered_float::OrderedFloat;

use super::{AnalyzedIREvent, AnalyzedIREvents, CorrelatedIrEvents, IREventClass, RackSharedState};
use crate::{
    channel::{Receiver, Sender},
    modality::MODALITY,
    satellite::SatCatId,
    system::{IREvent, SystemEnvironment},
    units::{Ratio, Time, Timestamp, Velocity},
    SimulationComponent,
};

/// Predict IR event class based on velocity
pub struct VelocityAnalysisSubsystem {
    #[allow(unused)]
    config: VelocityAnalysisConfig,
    correlation_rx: Receiver<CorrelatedIrEvents>,
    analyzed_tx: Sender<AnalyzedIREvents>,

    event_classes: Vec<VelocityEventClassification>,
    timeline: TimelineId,
}

#[derive(Clone)]
pub struct VelocityAnalysisConfig {}

impl VelocityAnalysisSubsystem {
    pub fn new(
        config: VelocityAnalysisConfig,
        correlation_rx: Receiver<CorrelatedIrEvents>,
        analyzed_tx: Sender<AnalyzedIREvents>,
    ) -> Self {
        let event_classes = vec![
            VelocityEventClassification::new(IREventClass::Fire, 0.0, 3.0),
            VelocityEventClassification::new(IREventClass::Meteor, 8000.0, 10000.0),
            VelocityEventClassification::new(IREventClass::Superman, 300.0, 500.0),
        ];

        Self {
            config,
            correlation_rx,
            analyzed_tx,
            event_classes,
            timeline: TimelineId::allocate(),
        }
    }

    fn classify_event(
        &self,
        event: IREvent,
        sat_id: SatCatId,
        rack_timestamp: Timestamp,
    ) -> AnalyzedIREvent {
        let velocity = Velocity::from_meters_per_second(event.velocity.norm());
        let ec = self
            .event_classes
            .iter()
            .max_by_key(|ec| OrderedFloat(ec.confidence_for_velocity(velocity).as_f64()))
            .unwrap();

        let mut confidence = ec.confidence_for_velocity(velocity).as_f64();
        let classification = if confidence == 0.0 {
            confidence = 1.0;
            IREventClass::Unknown
        } else {
            ec.classification
        };

        AnalyzedIREvent {
            rack_timestamp,
            based_on_sat_camera: (sat_id, event.source_id),
            event,
            classification,
            confidence,
        }
    }
}

impl<'a> SimulationComponent<'a> for VelocityAnalysisSubsystem {
    type SharedState = RackSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.emit_rack_timeline_attrs("velocity_analysis", rack.id);
        MODALITY.quick_event("init");
    }

    fn reset(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.quick_event("reset");
    }

    fn step(&mut self, _dt: Time, _env: &SystemEnvironment<'a>, rack: &mut RackSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        while let Some(msg) = self.correlation_rx.recv() {
            let mut events = vec![];
            for ev in msg.events.into_iter() {
                events.push(self.classify_event(ev, msg.satellite_id, msg.rack_timestamp));
            }

            let _ = self.analyzed_tx.try_send(AnalyzedIREvents {
                rack_timestamp: msg.rack_timestamp,
                events,
            });
        }
    }
}

struct VelocityEventClassification {
    classification: IREventClass,
    min: Velocity,
    max: Velocity,
}

impl VelocityEventClassification {
    fn new(class: IREventClass, min: f64, max: f64) -> Self {
        Self {
            classification: class,
            min: Velocity::from_meters_per_second(min),
            max: Velocity::from_meters_per_second(max),
        }
    }
}

impl VelocityEventClassification {
    fn midpoint(&self) -> Velocity {
        self.min + (self.max - self.min)
    }

    fn confidence_for_velocity(&self, v: Velocity) -> Ratio {
        if v < self.min || v > self.max {
            return Ratio::from_f64(0.0);
        }

        let midpoint = self.midpoint();
        if v < midpoint {
            (v - self.min) / (midpoint - self.min)
        } else if v > self.midpoint() {
            (self.max - v) / (self.max - midpoint)
        } else {
            Ratio::from_f64(1.0)
        }
    }
}
