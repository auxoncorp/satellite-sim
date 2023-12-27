use modality_api::TimelineId;
use ordered_float::OrderedFloat;

use super::{AnalyzedIREvent, AnalyzedIREvents, CorrelatedIrEvents, IREventClass, RackSharedState};
use crate::{
    channel::{Receiver, Sender},
    event,
    modality::MODALITY,
    recv,
    satellite::SatCatId,
    system::{IREvent, SystemEnvironment},
    try_send,
    units::{LuminousIntensity, Ratio, Time, Timestamp},
    SimulationComponent,
};

/// Predict IR event class based on intensity
pub struct IntensityAnalysisSubsystem {
    _config: IntensityAnalysisConfig,
    correlation_rx: Receiver<CorrelatedIrEvents>,
    analyzed_tx: Sender<AnalyzedIREvents>,

    event_classes: Vec<IntensityEventClassification>,
    timeline: TimelineId,
}

#[derive(Debug, Clone, Default)]
pub struct IntensityAnalysisConfig {}

impl IntensityAnalysisSubsystem {
    pub fn new(
        config: IntensityAnalysisConfig,
        correlation_rx: Receiver<CorrelatedIrEvents>,
        analyzed_tx: Sender<AnalyzedIREvents>,
    ) -> Self {
        let event_classes = vec![
            IntensityEventClassification::new(IREventClass::Fire, 5.0, 10.0),
            IntensityEventClassification::new(IREventClass::Meteor, 20.0, 40.0),
            IntensityEventClassification::new(IREventClass::Superman, 100.0, 500.0),
        ];

        Self {
            _config: config,
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
        let intensity = event.intensity;
        let ec = self
            .event_classes
            .iter()
            .max_by_key(|ec| OrderedFloat(ec.confidence_for_intensity(intensity).as_f64()))
            .unwrap();

        AnalyzedIREvent {
            rack_timestamp,
            based_on_sat_camera: (sat_id, event.source_id),
            event,
            classification: ec.class,
            confidence: ec.confidence_for_intensity(intensity).as_f64(),
        }
    }
}

impl<'a> SimulationComponent<'a> for IntensityAnalysisSubsystem {
    type SharedState = RackSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.emit_rack_timeline_attrs("intensity_analysis", rack.id);
        event!("init");
    }

    fn reset(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        event!("reset");
    }

    fn step(&mut self, _dt: Time, _env: &SystemEnvironment<'a>, rack: &mut RackSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);

        while let Some(msg) = recv!(&mut self.correlation_rx) {
            let mut events = vec![];
            for ev in msg.events.into_iter() {
                events.push(self.classify_event(ev, msg.satellite_id, msg.rack_timestamp));
            }

            let _ = try_send!(
                &mut self.analyzed_tx,
                AnalyzedIREvents {
                    rack_timestamp: msg.rack_timestamp,
                    events,
                }
            );
        }
    }
}

struct IntensityEventClassification {
    class: IREventClass,
    min: LuminousIntensity,
    max: LuminousIntensity,
}

impl IntensityEventClassification {
    fn new(class: IREventClass, min: f64, max: f64) -> Self {
        Self {
            class,
            min: LuminousIntensity::from_candelas(min),
            max: LuminousIntensity::from_candelas(max),
        }
    }
}

impl IntensityEventClassification {
    fn midpoint(&self) -> LuminousIntensity {
        self.min + (self.max - self.min)
    }

    fn confidence_for_intensity(&self, l: LuminousIntensity) -> Ratio {
        if l < self.min || l > self.max {
            return Ratio::from_f64(0.0);
        }

        let midpoint = self.midpoint();
        if l < midpoint {
            (l - self.min) / (midpoint - self.min)
        } else if l > self.midpoint() {
            (self.max - l) / (self.max - midpoint)
        } else {
            Ratio::from_f64(1.0)
        }
    }
}
