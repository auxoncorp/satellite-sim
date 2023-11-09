use na::{Rotation3, Vector3};
use nav_types::WGS84;
use std::net::TcpStream;
use tracing::debug;
use types42::prelude::WorldKind;

use crate::{
    external_mission_control::{GroundTruthIREvent, Message},
    scenario::ScheduledIREvent,
    system::{CameraSourceId, SystemEnvironment, SystemSharedState},
    units::Time,
    SimulationComponent,
};

pub type GroundTruthIrEvents = Vec<ScheduledIREvent>;

#[derive(Debug)]
pub struct GroundTruthIrEventsManagerConfig {
    pub scheduled_events: Vec<ScheduledIREvent>,
}

#[derive(Debug)]
pub struct GroundTruthIrEventsManager {
    config: GroundTruthIrEventsManagerConfig,

    pending_events: Vec<ScheduledIREvent>,
    active_events: Vec<ScheduledIREvent>,
    retired_events: Vec<ScheduledIREvent>,

    /// A connection to the external mission control software stack
    external_mission_control: Option<TcpStream>,
}

impl GroundTruthIrEventsManager {
    pub fn new(
        config: GroundTruthIrEventsManagerConfig,
        external_mission_control: Option<TcpStream>,
    ) -> Self {
        let pending_events = Vec::with_capacity(config.scheduled_events.len());
        let active_events = Vec::with_capacity(config.scheduled_events.len());
        let retired_events = Vec::with_capacity(config.scheduled_events.len());

        Self {
            config,
            pending_events,
            active_events,
            retired_events,
            external_mission_control,
        }
    }

    pub fn active_events(&self) -> &GroundTruthIrEvents {
        &self.active_events
    }

    /// Activate scheduled events
    fn manage_pending_events(&mut self, relative_sim_time: Time) {
        let mut idx = 0;
        while idx < self.pending_events.len() {
            if relative_sim_time >= self.pending_events[idx].activate_at {
                let mut activated_ev = self.pending_events.remove(idx);
                activated_ev.event.source_id = CameraSourceId::Unassigned;
                debug!(
                    ground_truth_id = activated_ev.event.ground_truth_id,
                    rel_time = relative_sim_time.as_secs(),
                    "Activating pending IR event"
                );
                self.active_events.push(activated_ev);

                continue;
            }
            idx += 1;
        }
    }

    /// Retire events that deactivate
    fn manage_active_events(&mut self, relative_sim_time: Time) {
        let mut idx = 0;
        while idx < self.active_events.len() {
            if let Some(deactivat_at) = self.active_events[idx].deactivat_at {
                if relative_sim_time >= deactivat_at {
                    let retired_ev = self.active_events.remove(idx);
                    debug!(
                        ground_truth_id = retired_ev.event.ground_truth_id,
                        rel_time = relative_sim_time.as_secs(),
                        "Retiring scheduled IR event"
                    );
                    self.retired_events.push(retired_ev);
                    continue;
                }
            }
            idx += 1;
        }
    }
}

impl<'a> SimulationComponent<'a> for GroundTruthIrEventsManager {
    type SharedState = SystemSharedState;
    type Environment = SystemEnvironment<'a>;

    fn init(&mut self, env: &SystemEnvironment, common: &mut SystemSharedState) {
        // Stage events that start at T0, otherwise put them in pending
        for mut ev in self.config.scheduled_events.iter().cloned() {
            if ev.activate_at.as_secs() == 0.0 {
                ev.event.source_id = CameraSourceId::Unassigned;
                debug!(
                    ground_truth_id = ev.event.ground_truth_id,
                    rel_time = env.sim_info.relative_time.as_secs(),
                    "Activating scheduled IR event"
                );
                self.active_events.push(ev);
            } else {
                self.pending_events.push(ev);
            }
        }

        if let Some(mut gui) = common.gui.as_ref().map(|gui| gui.borrow_mut()) {
            for ev in self.active_events.iter() {
                gui.update_ir_event(
                    ev.event.ground_truth_id,
                    &ev.event.position,
                    &ev.event.velocity,
                );
            }
        }
    }

    fn step(&mut self, dt: Time, env: &SystemEnvironment, common: &mut SystemSharedState) {
        let dt = dt.as_secs();

        // Update active event kinematics
        for ev in self.active_events.iter_mut() {
            let p = &ev.event.position;
            let p_wgs = WGS84::from(ev.event.location);

            // Angular velocity from east-north velocity
            // w = v / r
            let omega_east = ev.event.velocity_east / (WorldKind::EARTH_RADIUS + p_wgs.altitude());
            let omega_north =
                ev.event.velocity_north / (WorldKind::EARTH_RADIUS + p_wgs.altitude());

            // Map ENU to zyx in the world frame
            let omega_enu = Vector3::new(0.0, -omega_north * dt, omega_east * dt);

            // Rotation to convert relative ENU to ECEF world frame
            let latlat_rot = Rotation3::from_scaled_axis(Vector3::new(
                0.0,
                p_wgs.latitude_radians(),
                p_wgs.longitude_radians(),
            ));

            let omega_ecef = latlat_rot.inverse() * omega_enu;

            // This rotation describes the change in angle given rate and dt
            let v_rot = Rotation3::from_scaled_axis(omega_ecef);

            // Step the ECEF position
            let p_prime = v_rot * p;

            ev.event.velocity = (p_prime - p) / dt;
            ev.event.position = p_prime;
        }

        // Activate scheduled events
        self.manage_pending_events(env.sim_info.relative_time);

        // Retire events that deactivate
        self.manage_active_events(env.sim_info.relative_time);

        // Forward active events to the external mission control stack
        // periodically based on sim iters
        if env.sim_info.sim_iteration == 0 || (env.sim_info.sim_iteration % 25 == 0) {
            if let Some(stream) = self.external_mission_control.as_mut() {
                for ev in self.active_events.iter() {
                    Message::send(
                        &GroundTruthIREvent {
                            seqnum: env.sim_info.sim_iteration,
                            timestamp: env.sim_info.timestamp.into(),
                            event: &ev.event,
                        },
                        stream,
                    );
                }
            }
        }

        // Update GUI
        if let Some(mut gui) = common.gui.as_ref().map(|gui| gui.borrow_mut()) {
            for ev in self.active_events.iter() {
                gui.update_ir_event(
                    ev.event.ground_truth_id,
                    &ev.event.position,
                    &ev.event.velocity,
                );
            }

            for ev in self.retired_events.iter() {
                gui.remove_ir_event(ev.event.ground_truth_id);
            }
        }
    }
}
