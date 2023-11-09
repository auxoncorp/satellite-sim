use crate::{
    channel::{self, Receiver, Sender, StepChannel, TracedMessage},
    modality::kv,
    satellite::SatCatId,
    system::{CameraSourceId, IREvent, SatToGroundMessage, SystemEnvironment},
    units::{Time, Timestamp},
    SimulationComponent,
};

mod correlation;
mod intensity_analysis;
mod synthesis;
mod time_source;
mod velocity_analysis;

pub use correlation::CorrelationConfig;
pub use intensity_analysis::IntensityAnalysisConfig;
pub use synthesis::SynthesisConfig;
pub use time_source::TimeSourceConfig;
pub use velocity_analysis::VelocityAnalysisConfig;

use correlation::CorrelationSubsystem;
use intensity_analysis::IntensityAnalysisSubsystem;
use synthesis::SynthesisSubsystem;
use time_source::TimeSourceSubsystem;
use velocity_analysis::VelocityAnalysisSubsystem;

use super::{consolidated::GlobalIRView, Relayed};

pub struct RackSharedState {
    pub id: RackId,
    pub rtc: Timestamp,
}

pub struct Rack {
    time_source: TimeSourceSubsystem,
    correlation: CorrelationSubsystem,
    velocity_analysis: VelocityAnalysisSubsystem,
    intensity_analysis: IntensityAnalysisSubsystem,
    synthesis: SynthesisSubsystem,

    state: RackSharedState,
    channels: Vec<Box<dyn channel::Step>>,
}

pub type RackId = usize;

#[derive(Clone)]
pub struct RackConfig {
    pub id: RackId,
    pub time_source_config: TimeSourceConfig,
    pub correlation_config: CorrelationConfig,
    pub velocity_analysis_config: VelocityAnalysisConfig,
    pub intensity_analysis_config: IntensityAnalysisConfig,
    pub synthesis_config: SynthesisConfig,
}

impl Rack {
    pub fn new(
        config: RackConfig,
        relay_rx: Receiver<Relayed<SatToGroundMessage>>,
        global_ir_tx: Sender<GlobalIRView>,
    ) -> Self {
        let mut correlated_ir_events = StepChannel::new();
        let mut analyzed_ir_events = StepChannel::new();

        let time_source = TimeSourceSubsystem::new(config.time_source_config);

        let correlation = CorrelationSubsystem::new(
            config.correlation_config,
            relay_rx,
            correlated_ir_events.sender(None),
        );

        let velocity_analysis = VelocityAnalysisSubsystem::new(
            config.velocity_analysis_config,
            correlated_ir_events.receiver(None),
            analyzed_ir_events.sender(None),
        );

        let intensity_analysis = IntensityAnalysisSubsystem::new(
            config.intensity_analysis_config,
            correlated_ir_events.receiver(None),
            analyzed_ir_events.sender(None),
        );

        let synthesis = SynthesisSubsystem::new(
            config.synthesis_config,
            analyzed_ir_events.receiver(None),
            global_ir_tx,
        );

        let state = RackSharedState {
            id: config.id,
            rtc: Timestamp::epoch(),
        };

        Self {
            time_source,
            correlation,
            velocity_analysis,
            intensity_analysis,
            synthesis,

            state,
            channels: vec![Box::new(correlated_ir_events), Box::new(analyzed_ir_events)],
        }
    }
}

impl<'a> SimulationComponent<'a> for Rack {
    type SharedState = ();
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &SystemEnvironment<'a>, _: &mut ()) {
        self.state.rtc = env.sim_info.timestamp;
        self.time_source.init(env, &mut self.state);
        self.correlation.init(env, &mut self.state);
        self.velocity_analysis.init(env, &mut self.state);
        self.intensity_analysis.init(env, &mut self.state);
        self.synthesis.init(env, &mut self.state);
    }

    fn reset(&mut self, env: &'a Self::Environment, _: &mut Self::SharedState) {
        self.state.rtc = env.sim_info.timestamp;
        self.time_source.reset(env, &mut self.state);
        self.correlation.reset(env, &mut self.state);
        self.velocity_analysis.reset(env, &mut self.state);
        self.intensity_analysis.reset(env, &mut self.state);
        self.synthesis.reset(env, &mut self.state);
    }

    fn step(&mut self, dt: Time, env: &SystemEnvironment<'a>, _: &mut ()) {
        self.time_source.step(dt, env, &mut self.state);
        self.correlation.step(dt, env, &mut self.state);
        self.velocity_analysis.step(dt, env, &mut self.state);
        self.intensity_analysis.step(dt, env, &mut self.state);
        self.synthesis.step(dt, env, &mut self.state);

        for ch in self.channels.iter_mut() {
            let _ = ch.step();
        }
    }
}

/// Messages from multiple relays (but reflecting the same satellite
/// source message) are reconciled to make this one. This is output from
/// the 'correlation' stage.
#[derive(Debug, Clone)]
pub struct CorrelatedIrEvents {
    pub rack_timestamp: Timestamp,
    pub satellite_id: SatCatId,
    pub satellite_name: &'static str,
    pub events: Vec<IREvent>,
}

impl TracedMessage for CorrelatedIrEvents {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        vec![
            kv("event.name", "correlated_ir_events"),
            kv("event.rack.timestamp", self.rack_timestamp),
            kv("event.satellite.id", self.satellite_id),
            kv("event.satellite.name", self.satellite_name),
            kv("event.event_count", self.events.len() as i64),
        ]
    }
}

/// This is output from from the 'analysis' stages.
#[derive(Debug, Clone)]
pub struct AnalyzedIREvents {
    pub rack_timestamp: Timestamp,
    pub events: Vec<AnalyzedIREvent>,
}

impl TracedMessage for AnalyzedIREvents {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        vec![
            kv("event.name", "analyzed_ir_events"),
            kv("event.rack.timestamp", self.rack_timestamp),
            kv("event.event_count", self.events.len() as i64),
        ]
    }
}

#[derive(Debug, Clone)]
pub struct AnalyzedIREvent {
    pub rack_timestamp: Timestamp,

    /// The actual underlying event, from the satellite
    pub event: IREvent,

    /// What did the source event come from?
    pub based_on_sat_camera: (SatCatId, CameraSourceId),

    /// What class did the analysis stage choose for this event?
    pub classification: IREventClass,

    /// from 0.0 to 1.0
    pub confidence: f64,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum IREventClass {
    Fire,
    Meteor,
    Superman,
    Unknown,
}

impl IREventClass {
    #[allow(dead_code)]
    pub fn name(&self) -> &'static str {
        match self {
            IREventClass::Fire => "Fire",
            IREventClass::Meteor => "Meteor",
            IREventClass::Superman => "Superman",
            IREventClass::Unknown => "Unknown",
        }
    }
}
