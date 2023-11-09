use std::collections::BTreeSet;

use modality_api::TimelineId;

use super::{CorrelatedIrEvents, RackSharedState, Relayed};
use crate::{
    channel::{Receiver, Sender},
    modality::{kv, MODALITY},
    satellite::SatCatId,
    system::{Detections, SatToGroundMessage, SatToGroundMessageBody, SystemEnvironment},
    units::{Time, Timestamp},
    SimulationComponent,
};

pub struct CorrelationSubsystem {
    config: CorrelationConfig,
    relay_rx: Receiver<Relayed<SatToGroundMessage>>,
    analysis_tx: Sender<CorrelatedIrEvents>,

    recently_processed_detections: BTreeSet<(SatCatId, Timestamp)>,
    timeline: TimelineId,
}

#[derive(Clone)]
pub struct CorrelationConfig {
    pub correlation_window: Time,
    pub prune_window: Time,
}

impl CorrelationSubsystem {
    pub fn new(
        config: CorrelationConfig,
        relay_rx: Receiver<Relayed<SatToGroundMessage>>,
        analysis_tx: Sender<CorrelatedIrEvents>,
    ) -> Self {
        Self {
            config,
            relay_rx,
            analysis_tx,
            recently_processed_detections: BTreeSet::new(),
            timeline: TimelineId::allocate(),
        }
    }

    fn process_detections_msg(&mut self, msg: Relayed<SatToGroundMessage>, rack: &RackSharedState) {
        // See if we recently processed a detections msg at t += correlation_window/2
        let min_ts = msg.relay_timestamp - self.config.correlation_window / 2.0;
        let max_ts = msg.relay_timestamp + self.config.correlation_window / 2.0;

        let detections = body_into_detections(msg.inner.body);

        let mut detections_in_window = self
            .recently_processed_detections
            .range((msg.inner.satellite_id, min_ts)..(msg.inner.satellite_id, max_ts));

        if detections_in_window.next().is_none() {
            MODALITY.quick_event_attrs(
                "novel_detections_payload",
                [
                    kv("event.satellite.id", msg.inner.satellite_id),
                    kv("event.satellite.name", msg.inner.satellite_name),
                    kv("event.rack_timestamp", rack.rtc),
                ],
            );

            // we haven't seen this message before, based on time windowing. Send it on for analysis,
            // add add it to the 'recent' set.
            self.recently_processed_detections
                .insert((msg.inner.satellite_id, msg.relay_timestamp));
            let _ = self.analysis_tx.try_send(CorrelatedIrEvents {
                satellite_id: msg.inner.satellite_id,
                satellite_name: msg.inner.satellite_name,
                rack_timestamp: rack.rtc,
                events: detections.events,
            });
        } else {
            MODALITY.quick_event_attrs(
                "drop_duplicate_detections_payload",
                [
                    kv("event.satellite.id", msg.inner.satellite_id),
                    kv("event.satellite.name", msg.inner.satellite_name),
                    kv("event.rack_timestamp", rack.rtc),
                ],
            );
        }
    }

    fn prune_old_detections(&mut self, rack: &RackSharedState) {
        let cutoff = rack.rtc - self.config.prune_window;
        let count_before = self.recently_processed_detections.len();
        self.recently_processed_detections
            .retain(|(_id, ts)| ts > &cutoff);

        let count_after = self.recently_processed_detections.len();
        let prune_count = count_before - count_after;
        if prune_count > 0 {
            MODALITY.quick_event_attrs(
                "prune_old_detections",
                [kv("event.count", prune_count as i64)],
            );
        }
    }
}

fn body_into_detections(body: SatToGroundMessageBody) -> Detections {
    match body {
        SatToGroundMessageBody::SatelliteTelemetry(_) => panic!(),
        SatToGroundMessageBody::Detections(detections) => detections,
    }
}

impl<'a> SimulationComponent<'a> for CorrelationSubsystem {
    type SharedState = RackSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.emit_rack_timeline_attrs("correlation", rack.id);
        MODALITY.quick_event("init");
    }

    fn reset(&mut self, _env: &'a Self::Environment, rack: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);
        MODALITY.quick_event("reset");
    }

    fn step(&mut self, _dt: Time, _env: &SystemEnvironment<'a>, rack: &mut RackSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, rack.rtc);

        while let Some(msg) = self.relay_rx.recv() {
            if let SatToGroundMessageBody::Detections(_) = &msg.inner.body {
                self.process_detections_msg(msg, rack);
            }

            self.prune_old_detections(rack);
        }
    }
}
