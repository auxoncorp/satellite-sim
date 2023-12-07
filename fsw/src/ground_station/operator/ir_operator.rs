use tracing::warn;

use modality_api::TimelineId;

use crate::{
    channel::{Receiver, Sender},
    ground_station::consolidated::{OperatorAction, OperatorNotification},
    modality::{kv, MODALITY},
    system::SystemEnvironment,
    units::Timestamp,
    SimulationComponent,
};

/// A pretend person that knows how to reset error flags.
pub struct IROperator {
    #[allow(dead_code)]
    config: IROperatorConfig,

    /// Send out actions
    operator_action_tx: Sender<OperatorAction>,

    /// Get notified by the UI
    operator_notification_rx: Receiver<OperatorNotification>,

    /// What time is it, according to Casio?
    wristwatch: Timestamp,

    timeline: TimelineId,
}

impl IROperator {
    pub fn new(
        config: IROperatorConfig,
        operator_action_tx: Sender<OperatorAction>,
        operator_notification_rx: Receiver<OperatorNotification>,
    ) -> Self {
        Self {
            config,
            operator_action_tx,
            operator_notification_rx,
            wristwatch: Timestamp::epoch(),
            timeline: TimelineId::allocate(),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct IROperatorConfig {}

impl<'a> SimulationComponent<'a> for IROperator {
    type SharedState = ();
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, _: &mut Self::SharedState) {
        self.wristwatch = env.sim_info.timestamp;
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.wristwatch);
        MODALITY.emit_timeline_attrs([
            kv("timeline.name", "ir_operator"),
            kv("timeline.ground_station.name", "consolidated"),
        ]);

        MODALITY.quick_event("init");
    }

    fn reset(&mut self, _env: &'a Self::Environment, _: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.wristwatch);
        MODALITY.quick_event("reset");
    }

    fn step(
        &mut self,
        dt: crate::units::Time,
        _env: &'a Self::Environment,
        _common_state: &mut Self::SharedState,
    ) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.wristwatch);
        self.wristwatch += dt;

        // TODO print shit so we know it's happening
        while let Some(msg) = self.operator_notification_rx.recv() {
            match msg {
                OperatorNotification::GlobalIRViewStateChange(sc) => {
                    // This operator is a squirrel who always focuses on the new, shiny event.
                    if let Some(new_ev) = sc.new_events.first() {
                        let _ = self
                            .operator_action_tx
                            .try_send(OperatorAction::PrioritizeIrEvent { id: *new_ev });
                    }
                }
                _ => warn!(msg = ?msg, "Unexpected IR operator notification"),
            }
        }
    }
}
