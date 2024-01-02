// what do we need?
// inputs: results from result selection, commands from the operator
// outputs: notifications to the operator, commands to realys

use std::{
    collections::{HashMap, HashSet},
    net::TcpStream,
};

use modality_api::TimelineId;

use crate::{
    channel::{Receiver, Sender},
    event,
    external_mission_control::{Message, Telemetry},
    modality::{kv, MODALITY},
    recv,
    satellite::SatCatId,
    system::{
        GroundToSatMessage, SatErrorFlag, SatToGroundMessage, SatToGroundMessageBody,
        SatelliteTelemetry, SystemEnvironment,
    },
    try_send,
    units::Timestamp,
    SimulationComponent,
};

use super::{
    consolidated::{
        GlobalIRViewStateChange, OperatorAction, OperatorNotification, SatErrorFlagStateChange,
        SelectedGlobalIRView,
    },
    Relayed,
};

pub struct MissionControlUISubsystem {
    #[allow(dead_code)]
    config: MissionControlUIConfig,

    /// Receive telemetry messages directly from the satellites (via relays)
    sat_telemetry_rx: Receiver<Relayed<SatToGroundMessage>>,

    /// Receive selected global IR view periodically from the Result Selection stage
    selected_global_view_rx: Receiver<SelectedGlobalIRView>,

    /// Receive actions from the operators, as they come
    operator_action_rx: Receiver<OperatorAction>,

    /// Messages sent to satellites, via the relay network
    sat_tx: Sender<GroundToSatMessage>,

    /// Noficiations to the IR operator that something has happened
    ir_operator_notification_tx: Sender<OperatorNotification>,

    /// Noficiations to the satellite operator that something has happened
    sat_operator_notification_tx: Sender<OperatorNotification>,

    /// The most recently received global ir view
    current_global_ir_view: Option<SelectedGlobalIRView>,

    /// The error flags known to be set on each satellite
    current_sat_error_flags: HashMap<SatCatId, HashSet<SatErrorFlag>>,

    /// A connection to the external mission control software stack
    external_mission_control: Option<TcpStream>,

    timeline: TimelineId,
    relative_rtc: Timestamp,
}

#[derive(Debug, Clone, Default)]
pub struct MissionControlUIConfig {}

impl MissionControlUISubsystem {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        config: MissionControlUIConfig,
        sat_telemetry_rx: Receiver<Relayed<SatToGroundMessage>>,
        selected_global_view_rx: Receiver<SelectedGlobalIRView>,
        operator_action_rx: Receiver<OperatorAction>,
        sat_tx: Sender<GroundToSatMessage>,
        ir_operator_notification_tx: Sender<OperatorNotification>,
        sat_operator_notification_tx: Sender<OperatorNotification>,
        external_mission_control: Option<TcpStream>,
    ) -> Self {
        Self {
            config,
            sat_telemetry_rx,
            selected_global_view_rx,
            operator_action_rx,
            sat_tx,
            ir_operator_notification_tx,
            sat_operator_notification_tx,
            current_global_ir_view: None,
            current_sat_error_flags: HashMap::new(),
            external_mission_control,
            timeline: TimelineId::allocate(),
            relative_rtc: Timestamp::epoch(),
        }
    }

    fn process_telemetry_error_flags(
        &mut self,
        sat: SatCatId,
        sat_name: &'static str,
        telem: Box<SatelliteTelemetry>,
    ) -> SatErrorFlagStateChange {
        let current_flags = self.current_sat_error_flags.entry(sat).or_default();

        let mut sc = SatErrorFlagStateChange {
            sat,
            sat_name,
            set_flags: vec![],
            cleared_flags: vec![],
        };

        if let Some(power_status) = telem.power {
            sc.add_flag(
                SatErrorFlag::PowerOutOfSync,
                power_status.error_register.out_of_sync,
                current_flags,
            );
        }

        if let Some(imu_status) = telem.imu_status {
            sc.add_flag(
                SatErrorFlag::ImuOutOfSync,
                imu_status.error_register.out_of_sync,
                current_flags,
            );
            sc.add_flag(
                SatErrorFlag::ImuDataInconsistency,
                imu_status.error_register.data_inconsistency,
                current_flags,
            );
            sc.add_flag(
                SatErrorFlag::ImuDegraded,
                imu_status.error_register.degraded,
                current_flags,
            );
        }

        if let Some(vision_status) = telem.vision_status {
            sc.add_flag(
                SatErrorFlag::VisionOutOfSync,
                vision_status.error_register.out_of_sync,
                current_flags,
            );
        }

        if let Some(comms_status) = telem.comms_status {
            sc.add_flag(
                SatErrorFlag::CommsOutOfSync,
                comms_status.error_register.out_of_sync,
                current_flags,
            );
        }

        sc
    }

    fn process_new_global_ir_view(
        &mut self,
        new_view: SelectedGlobalIRView,
    ) -> GlobalIRViewStateChange {
        let sc = if let Some(old_view) = &self.current_global_ir_view {
            let mut sc = GlobalIRViewStateChange::default();
            let new_events = &new_view.view.events;
            let old_events = &old_view.view.events;

            for new_ev in new_events.iter() {
                if !old_events.iter().any(|old_ev| old_ev.id == new_ev.id) {
                    sc.new_events.push(new_ev.id);
                }
            }

            for old_ev in old_events.iter() {
                if !new_events.iter().any(|new_ev| new_ev.id == old_ev.id) {
                    sc.disappeared_events.push(old_ev.id);
                }
            }

            sc
        } else {
            GlobalIRViewStateChange {
                new_events: new_view.view.events.iter().map(|ev| ev.id).collect(),
                disappeared_events: vec![],
            }
        };

        self.current_global_ir_view = Some(new_view);
        sc
    }
}

impl<'a> SimulationComponent<'a> for MissionControlUISubsystem {
    type SharedState = ();
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, _: &mut Self::SharedState) {
        self.relative_rtc = env.sim_info.timestamp;
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.relative_rtc);
        MODALITY.emit_timeline_attrs([
            kv("timeline.name", "mission_control_ui"),
            kv("timeline.ground_station.name", "consolidated"),
        ]);

        event!("init");
    }

    fn reset(&mut self, _env: &'a Self::Environment, _: &mut Self::SharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.relative_rtc);
        event!("reset");
    }

    fn step(
        &mut self,
        dt: crate::units::Time,
        env: &'a Self::Environment,
        _common_state: &mut Self::SharedState,
    ) {
        self.relative_rtc += dt;
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, self.relative_rtc);

        while let Some(msg) = recv!(&mut self.sat_telemetry_rx) {
            // Forward a copy to the external mission control stack
            // periodically based on sim iters
            if env.sim_info.sim_iteration == 0 || (env.sim_info.sim_iteration % 50 == 0) {
                if let Some(stream) = self.external_mission_control.as_mut() {
                    if let Some(ext_msg) = Telemetry::from_sat_to_ground_message(&msg) {
                        Message::send(&ext_msg, stream);
                    }
                }
            }

            // Non-telemetry messages are handled by the analysis system; we get the results
            // of the analysis via the selected_global_view channel.
            if let SatToGroundMessageBody::SatelliteTelemetry(telem) = msg.inner.body {
                let state_change = self.process_telemetry_error_flags(
                    msg.inner.satellite_id,
                    msg.inner.satellite_name,
                    telem,
                );
                if state_change.has_action() {
                    let _ = try_send!(
                        &mut self.sat_operator_notification_tx,
                        OperatorNotification::SatErrorFlagStateChange(state_change)
                    );
                }
            }
        }

        while let Some(new_view) = recv!(&mut self.selected_global_view_rx) {
            let state_change = self.process_new_global_ir_view(new_view);
            if state_change.has_action() {
                let tx_msg = OperatorNotification::GlobalIRViewStateChange(state_change);
                let _ = try_send!(&mut self.ir_operator_notification_tx, tx_msg);
            }
        }

        while let Some(action) = recv!(&mut self.operator_action_rx) {
            match action {
                OperatorAction::PrioritizeIrEvent { id } => {
                    if let Some(view) = &self.current_global_ir_view {
                        let events = &view.view.events;
                        if let Some(ev) = events.iter().find(|ev| ev.id == id) {
                            // Send one message per source satellite that we know to have seen the event
                            for obs in ev.observations.iter() {
                                let (sat, source_id) = obs.based_on_sat_camera;
                                let tx_msg =
                                    GroundToSatMessage::PrioritizeIrEvent { sat, source_id };
                                let _ = try_send!(&mut self.sat_tx, tx_msg);
                            }
                        }
                    }
                }
                OperatorAction::ClearSatelliteErrorFlag { sat, flag } => {
                    let tx_msg = GroundToSatMessage::ClearSatelliteErrorFlag { sat, flag };
                    let _ = try_send!(&mut self.sat_tx, tx_msg);
                }
            }
        }
    }
}
