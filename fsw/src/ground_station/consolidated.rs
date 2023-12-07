use std::{
    collections::{HashSet, VecDeque},
    net::TcpStream,
};

use crate::{
    channel::{self, Receiver, Sender, StepChannel, TracedMessage},
    modality::kv,
    satellite::SatCatId,
    system::{GroundToSatMessage, SatErrorFlag, SatToGroundMessage, SystemEnvironment},
    units::{Time, Timestamp},
    SimulationComponent,
};

use super::{
    mcui::{MissionControlUIConfig, MissionControlUISubsystem},
    rack::{AnalyzedIREvent, Rack, RackConfig, RackId},
    result_selection::{ResultSelectionConfig, ResultSelectionSubsystem},
    IROperator, IROperatorConfig, OperationalStatus, Relayed, SatOperator, SatOperatorConfig,
};

pub struct ConsolidatedGroundStation {
    analysis_racks: Vec<Rack>,
    result_selection: ResultSelectionSubsystem,
    mcui: MissionControlUISubsystem,
    ir_operator: IROperator,
    sat_operator: SatOperator,
    channels: Vec<Box<dyn channel::Step>>,
}

#[derive(Debug, Clone)]
pub struct ConsolidatedGroundStationConfig {
    pub base_rack_config: RackConfig,
    pub rack_count: usize,
    pub result_selection_config: ResultSelectionConfig,
    pub mcui_config: MissionControlUIConfig,
    pub ir_operator_config: IROperatorConfig,
    pub sat_operator_config: SatOperatorConfig,
}

impl ConsolidatedGroundStation {
    pub fn new(
        config: ConsolidatedGroundStationConfig,
        mut make_relay_rx: impl FnMut() -> Receiver<Relayed<SatToGroundMessage>>,
        sat_tx: Sender<GroundToSatMessage>,
        external_mission_control: Option<TcpStream>,
    ) -> Self {
        let mut global_ir_view_from_rack = StepChannel::new();
        let mut selected_global_ir_view = StepChannel::new();
        let mut operator_action = StepChannel::new();
        let mut ir_operator_notification = StepChannel::new();
        let mut sat_operator_notification = StepChannel::new();

        let mut analysis_racks = vec![];
        for i in 0..config.rack_count {
            let mut cfg = config.base_rack_config.clone();
            cfg.id = i;
            analysis_racks.push(Rack::new(
                cfg,
                make_relay_rx(),
                global_ir_view_from_rack.sender(None),
            ));
        }

        let result_selection = ResultSelectionSubsystem::new(
            config.result_selection_config,
            global_ir_view_from_rack.receiver(None),
            selected_global_ir_view.sender(None),
        );

        let mcui = MissionControlUISubsystem::new(
            config.mcui_config,
            make_relay_rx(),
            selected_global_ir_view.receiver(None),
            operator_action.receiver(None),
            sat_tx,
            ir_operator_notification.sender(None),
            sat_operator_notification.sender(None),
            external_mission_control,
        );

        let ir_operator = IROperator::new(
            config.ir_operator_config,
            operator_action.sender(None),
            ir_operator_notification.receiver(None),
        );

        let sat_operator = SatOperator::new(
            config.sat_operator_config,
            operator_action.sender(None),
            sat_operator_notification.receiver(None),
        );

        Self {
            analysis_racks,
            result_selection,
            mcui,
            ir_operator,
            sat_operator,
            channels: vec![
                Box::new(global_ir_view_from_rack),
                Box::new(selected_global_ir_view),
                Box::new(operator_action),
                Box::new(ir_operator_notification),
                Box::new(sat_operator_notification),
            ],
        }
    }
}

impl<'a> SimulationComponent<'a> for ConsolidatedGroundStation {
    type SharedState = ();
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, _: &mut Self::SharedState) {
        for rack in self.analysis_racks.iter_mut() {
            rack.init(env, &mut ());
        }

        self.result_selection.init(env, &mut ());
        self.mcui.init(env, &mut ());
        self.ir_operator.init(env, &mut ());
        self.sat_operator.init(env, &mut ());
    }

    fn reset(&mut self, env: &'a Self::Environment, _: &mut Self::SharedState) {
        for rack in self.analysis_racks.iter_mut() {
            rack.reset(env, &mut ());
        }

        self.result_selection.reset(env, &mut ());
        self.mcui.reset(env, &mut ());
        self.ir_operator.reset(env, &mut ());
        self.sat_operator.reset(env, &mut ());
    }

    fn step(&mut self, dt: Time, env: &'a Self::Environment, _: &mut Self::SharedState) {
        for rack in self.analysis_racks.iter_mut() {
            rack.step(dt, env, &mut ());
        }

        self.result_selection.step(dt, env, &mut ());
        self.mcui.step(dt, env, &mut ());
        self.ir_operator.step(dt, env, &mut ());
        self.sat_operator.step(dt, env, &mut ());

        for ch in self.channels.iter_mut() {
            let _ = ch.step();
        }
    }
}

#[derive(Debug, Clone)]
pub struct SelectedGlobalIRView {
    pub view: GlobalIRView,
    pub status: OperationalStatus,
}

impl TracedMessage for SelectedGlobalIRView {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        let mut kvs = self.view.attrs();
        kvs.push(kv("event.status", self.status.name()));
        kvs
    }
}

/// The output of a synthesis stage; contains all current events in the whole globe
#[derive(Debug, Clone)]
pub struct GlobalIRView {
    pub rack_timestamp: Timestamp,
    pub rack_id: RackId,
    pub events: Vec<TrackedIREvent>,
}

impl TracedMessage for GlobalIRView {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        vec![
            kv("event.name", "global_ir_view"),
            kv("event.rack.timestamp", self.rack_timestamp),
            kv("event.rack.id", self.rack_id as i64),
            kv("event.event_count", self.events.len() as i64),
        ]
    }
}

/// A single coherent event on the globe, based on observations from
/// many cameras on many satellites
#[derive(Debug, Clone)]
pub struct TrackedIREvent {
    pub id: TrackedEventId,
    pub observations: VecDeque<AnalyzedIREvent>,
}

pub type TrackedEventId = usize;

#[derive(Debug, Clone)]
pub enum OperatorNotification {
    GlobalIRViewStateChange(GlobalIRViewStateChange),
    SatErrorFlagStateChange(SatErrorFlagStateChange),
}

impl TracedMessage for OperatorNotification {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            OperatorNotification::GlobalIRViewStateChange(sc) => {
                vec![
                    kv("event.name", "global_ir_view_change"),
                    kv("event.new_event_count", sc.new_events.len() as i64),
                    kv(
                        "event.disappeared_event_count",
                        sc.disappeared_events.len() as i64,
                    ),
                ]
            }
            OperatorNotification::SatErrorFlagStateChange(sc) => {
                let mut kvs = vec![
                    kv("event.name", "satellite_error_flag_change"),
                    kv("event.satellite.id", sc.sat),
                    kv("event.satellite.name", sc.sat_name),
                ];

                for set_flag in sc.set_flags.iter() {
                    kvs.push(kv(&format!("event.flag.{}", set_flag.name()), true));
                }

                for set_flag in sc.cleared_flags.iter() {
                    kvs.push(kv(&format!("event.flag.{}", set_flag.name()), false));
                }

                kvs
            }
        }
    }
}

#[derive(Default, Debug, Clone)]
pub struct GlobalIRViewStateChange {
    pub new_events: Vec<TrackedEventId>,
    pub disappeared_events: Vec<TrackedEventId>,
}

impl GlobalIRViewStateChange {
    pub fn has_action(&self) -> bool {
        !self.new_events.is_empty() || !self.disappeared_events.is_empty()
    }
}

#[derive(Debug, Clone)]
pub struct SatErrorFlagStateChange {
    pub sat: SatCatId,
    pub sat_name: &'static str,
    pub set_flags: Vec<SatErrorFlag>,
    pub cleared_flags: Vec<SatErrorFlag>,
}

impl SatErrorFlagStateChange {
    pub fn has_action(&self) -> bool {
        !self.set_flags.is_empty() || !self.cleared_flags.is_empty()
    }

    pub fn add_flag(
        &mut self,
        flag: SatErrorFlag,
        flag_is_set: bool,
        sat_flag_set: &mut HashSet<SatErrorFlag>,
    ) {
        if flag_is_set && sat_flag_set.insert(flag) {
            self.set_flags.push(flag);
        } else if !flag_is_set && sat_flag_set.remove(&flag) {
            self.cleared_flags.push(flag);
        }
    }
}

#[derive(Debug, Clone)]
pub enum OperatorAction {
    PrioritizeIrEvent { id: TrackedEventId },
    ClearSatelliteErrorFlag { sat: SatCatId, flag: SatErrorFlag },
}

impl TracedMessage for OperatorAction {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            OperatorAction::PrioritizeIrEvent { id } => {
                vec![
                    kv("event.name", "prioritize_ir_event"),
                    kv("event.ir_event_id", *id as i64),
                ]
            }
            OperatorAction::ClearSatelliteErrorFlag { sat, flag } => {
                vec![
                    kv("event.name", "clear_satellite_error_flag"),
                    kv("event.satellite.id", *sat),
                    kv("event.error_flag", flag.name()),
                ]
            }
        }
    }
}
