use std::collections::HashSet;
use tracing::warn;

use modality_api::TimelineId;

use crate::{
    channel::{Receiver, Sender},
    ground_station::consolidated::{OperatorAction, OperatorNotification},
    modality::{kv, MODALITY},
    satellite::SatCatId,
    system::{SatErrorFlag, SystemEnvironment},
    units::{Time, Timestamp},
    SimulationComponent,
};

/// A pretend person that knows how to focus cameras on events.
pub struct SatOperator {
    #[allow(dead_code)]
    config: SatOperatorConfig,

    /// Send out actions
    operator_action_tx: Sender<OperatorAction>,

    /// Get notified by the UI
    operator_notification_rx: Receiver<OperatorNotification>,

    /// After this time passes, we'll clear all the flags we have queued up.
    next_flag_clearing: Option<Timestamp>,

    /// All the flags we're going to clear, when we get around to it.
    flags_to_clear: HashSet<(SatCatId, SatErrorFlag)>,

    /// What time is it, according to Casio?
    wristwatch: Timestamp,

    timeline: TimelineId,
}

impl SatOperator {
    pub fn new(
        config: SatOperatorConfig,
        operator_action_tx: Sender<OperatorAction>,
        operator_notification_rx: Receiver<OperatorNotification>,
    ) -> Self {
        Self {
            config,
            operator_action_tx,
            operator_notification_rx,
            next_flag_clearing: None,
            flags_to_clear: HashSet::new(),
            wristwatch: Timestamp::epoch(),
            timeline: TimelineId::allocate(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct SatOperatorConfig {
    /// The operator will eventually clear flags, after waiting this long
    pub clear_flags_after: Time,
}

impl<'a> SimulationComponent<'a> for SatOperator {
    type SharedState = ();
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, _: &mut Self::SharedState) {
        self.wristwatch = env.sim_info.timestamp;
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.wristwatch);
        MODALITY.emit_timeline_attrs([
            kv("timeline.name", "sat_operator"),
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
                OperatorNotification::SatErrorFlagStateChange(sc) => {
                    if !sc.set_flags.is_empty() && self.next_flag_clearing.is_none() {
                        self.next_flag_clearing = Some(self.wristwatch);
                    }

                    for flag in sc.set_flags.into_iter() {
                        MODALITY.quick_event_attrs(
                            "remembering_error_flag_to_clear_in_a_moment",
                            [
                                kv("event.satellite.id", sc.sat),
                                kv("event.flag", flag.name()),
                            ],
                        );
                        self.flags_to_clear.insert((sc.sat, flag));
                    }
                }
                _ => warn!(msg = ?msg, "Unexpected satellite operator notification"),
            }
        }

        if let Some(clear_time) = self.next_flag_clearing {
            if self.wristwatch > clear_time {
                for (sat, flag) in self.flags_to_clear.drain() {
                    let _ = self
                        .operator_action_tx
                        .try_send(dbg!(OperatorAction::ClearSatelliteErrorFlag { sat, flag }));
                }
                self.next_flag_clearing = None;
            }
        }
    }
}
