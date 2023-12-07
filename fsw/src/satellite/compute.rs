use modality_api::TimelineId;
use serde::Serialize;
use tracing::warn;

use crate::{
    channel::{Receiver, Sender},
    modality::{AttrsBuilder, MODALITY},
    mutator::{watchdog_out_of_sync_descriptor, GenericBooleanMutator},
    satellite::{SatelliteEnvironment, SatelliteId, SatelliteSharedState},
    system::{
        Detections, GroundToSatMessage, SatErrorFlag, SatToGroundMessage, SatToGroundMessageBody,
        SatelliteTelemetry,
    },
    units::{Temperature, Time, Timestamp},
    SimulationComponent,
};

use super::{
    comms::{CommsCommand, CommsResponse},
    imu::{ImuCommand, ImuResponse, ImuSample},
    power::{PowerCommand, PowerResponse},
    temperature_sensor::{TemperatureSensor, TemperatureSensorConfig},
    vision::{VisionCommand, VisionResponse},
};

pub struct ComputeSubsystem {
    config: ComputeConfig,
    channels: ComputeChannels,

    last_sequence_number: u64,
    telemetry_state: TelemetryState,
    error_register: ComputeErrorRegister,

    temp_sensor: TemperatureSensor,
    watchdog_out_of_sync: Option<GenericBooleanMutator>,

    timeline: TimelineId,
}

#[derive(Debug, Clone)]
pub struct ComputeConfig {
    pub telemetry_rate: Time,
    pub collect_timeout: Time,
    pub temperature_sensor_config: TemperatureSensorConfig,
    pub fault_config: ComputeFaultConfig,
}

/// Parameters for the compute point failures.
/// See the requirements doc, section 1.3.4.2.
#[derive(Debug, Clone, Default)]
pub struct ComputeFaultConfig {
    /// Enable the data watchdog execution out-of-sync mutator.
    /// See sections 1.3.4.2 of the requirements doc.
    pub watchdog_out_of_sync: bool,
}

pub struct ComputeChannels {
    pub power_cmd_tx: Sender<PowerCommand>,
    pub power_res_rx: Receiver<PowerResponse>,

    pub comms_cmd_tx: Sender<CommsCommand>,
    pub comms_res_rx: Receiver<CommsResponse>,

    pub vision_detections_rx: Receiver<Detections>,
    pub vision_cmd_tx: Sender<VisionCommand>,
    pub vision_res_rx: Receiver<VisionResponse>,

    pub imu_sample_rx: Receiver<ImuSample>,
    pub imu_cmd_tx: Sender<ImuCommand>,
    pub imu_res_rx: Receiver<ImuResponse>,
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct ComputeStatus {
    pub temperature: Temperature,
    pub error_register: ComputeErrorRegister,
}

impl ComputeStatus {
    pub fn to_attrs(&self, b: &mut AttrsBuilder) {
        b.kv("temperature", self.temperature.as_degrees_celsius());
        b.kv("error.out_of_sync", self.error_register.out_of_sync);
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct ComputeErrorRegister {
    /// Watchdog detected out-of-sync execution, subsystem is halted
    pub out_of_sync: bool,
}

enum TelemetryState {
    Idle {
        collect_time: Timestamp,
    },
    Collecting {
        pending_telemetry: Box<SatelliteTelemetry>,
        timeout_send_time: Timestamp,
        next_collect_time: Timestamp,
    },
}

impl ComputeSubsystem {
    pub const COMPONENT_NAME: &'static str = "compute_subsystem";

    pub fn new(config: ComputeConfig, channels: ComputeChannels) -> Self {
        let timeline = TimelineId::allocate();
        let temp_sensor = TemperatureSensor::new(config.temperature_sensor_config);
        Self {
            config,
            channels,

            last_sequence_number: 1,
            telemetry_state: TelemetryState::Idle {
                // real initial value comes in init
                collect_time: Timestamp::epoch(),
            },
            error_register: ComputeErrorRegister { out_of_sync: false },

            temp_sensor,
            // Mutators are initialized in init_fault_models at sim-init time
            watchdog_out_of_sync: None,

            timeline,
        }
    }

    fn init_fault_models(&mut self, id: &SatelliteId) {
        // Construct mutators at sim-init time so we have access to SatelliteId/env details
        // to qualify the mutator descriptors
        self.watchdog_out_of_sync =
            self.config
                .fault_config
                .watchdog_out_of_sync
                .then_some(GenericBooleanMutator::new(watchdog_out_of_sync_descriptor(
                    Self::COMPONENT_NAME,
                    id,
                )));

        if let Some(m) = &self.watchdog_out_of_sync {
            MODALITY.register_mutator(m);
        }
    }

    fn update_fault_models(&mut self) {
        self.error_register.out_of_sync = self
            .watchdog_out_of_sync
            .as_ref()
            .map(|m| m.is_active())
            .unwrap_or(false);
    }

    fn hard_reset(&mut self, rel_time: Time, shared_state: &mut SatelliteSharedState) {
        warn!(?rel_time, "Compute hard reset");

        // Clear error register
        self.error_register.out_of_sync = false;

        // Reset state
        self.last_sequence_number = 1;
        self.telemetry_state = TelemetryState::Idle {
            collect_time: shared_state.rtc + self.config.telemetry_rate,
        };

        if let Some(m) = self.watchdog_out_of_sync.as_mut() {
            MODALITY.clear_mutation(m);
        }

        // Drop message buffers
        self.channels.power_cmd_tx.clear();
        self.channels.power_res_rx.clear();
        self.channels.comms_cmd_tx.clear();
        self.channels.comms_res_rx.clear();
        self.channels.vision_detections_rx.clear();
        self.channels.vision_cmd_tx.clear();
        self.channels.vision_res_rx.clear();
        self.channels.imu_sample_rx.clear();
        self.channels.imu_cmd_tx.clear();
        self.channels.imu_res_rx.clear();
    }
}

impl<'a> SimulationComponent<'a> for ComputeSubsystem {
    type SharedState = SatelliteSharedState;
    type Environment = SatelliteEnvironment<'a>;

    fn init(&mut self, env: &'a Self::Environment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        MODALITY.emit_sat_timeline_attrs("cpu", sat.id);
        MODALITY.quick_event("init");

        self.temp_sensor.init(env, sat);

        self.telemetry_state = TelemetryState::Idle {
            collect_time: sat.rtc + self.config.telemetry_rate,
        };

        self.init_fault_models(sat.id);
    }

    fn reset(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        MODALITY.quick_event("reset");
        self.hard_reset(env.sim_info.relative_time, sat);
    }

    fn step(&mut self, dt: Time, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);

        MODALITY
            .process_mutation_plane_messages(std::iter::once(self.watchdog_out_of_sync.as_mut()));

        self.temp_sensor.step(dt, env, sat);

        self.update_fault_models();

        let mut power_res = self.channels.power_res_rx.recv();
        let mut comms_res = self.channels.comms_res_rx.recv();
        let mut vision_detections = self.channels.vision_detections_rx.recv();
        let mut vision_res = self.channels.vision_res_rx.recv();
        let mut imu_sample = self.channels.imu_sample_rx.recv();
        let mut imu_res = self.channels.imu_res_rx.recv();

        if self.error_register.out_of_sync {
            warn!("Detected compute watchdog, triggering hard reset");
            sat.reset_flags.reset_all();
        }

        if let Some(vision_detections) = vision_detections.take() {
            self.last_sequence_number += 1;
            let _ = self
                .channels
                .comms_cmd_tx
                .try_send(CommsCommand::SendGroundMessage(Box::new(
                    SatToGroundMessage {
                        seq: self.last_sequence_number,
                        satellite_id: sat.id.satcat_id,
                        satellite_name: sat.id.name,
                        sat_send_timestamp: sat.rtc,
                        body: SatToGroundMessageBody::Detections(vision_detections),
                    },
                )));
        }

        match &mut self.telemetry_state {
            TelemetryState::Idle { collect_time } => {
                if &sat.rtc >= collect_time {
                    let _ = self.channels.power_cmd_tx.try_send(PowerCommand::GetStatus);
                    let _ = self.channels.comms_cmd_tx.try_send(CommsCommand::GetStatus);
                    let _ = self.channels.comms_cmd_tx.try_send(CommsCommand::GetGps);
                    let _ = self
                        .channels
                        .vision_cmd_tx
                        .try_send(VisionCommand::GetStatus);
                    let _ = self.channels.imu_cmd_tx.try_send(ImuCommand::GetStatus);

                    self.telemetry_state = TelemetryState::Collecting {
                        pending_telemetry: Box::new(SatelliteTelemetry::new()),
                        timeout_send_time: sat.rtc + self.config.collect_timeout,
                        next_collect_time: sat.rtc + self.config.telemetry_rate,
                    };
                }
            }
            TelemetryState::Collecting {
                pending_telemetry,
                timeout_send_time,
                next_collect_time,
            } => {
                if let Some(PowerResponse::Status(power_status)) = power_res.take() {
                    if power_status.error_register.out_of_sync {
                        warn!("Detected power watchdog, triggering hard reset");
                        sat.reset_flags.reset_power();
                    }
                    pending_telemetry.power = power_status.into();
                }

                if let Some(comms_resp) = comms_res.take() {
                    match comms_resp {
                        CommsResponse::GpsPosition(gps) => {
                            pending_telemetry.gps = gps.into();
                        }
                        CommsResponse::Status(comms_status) => {
                            if comms_status.error_register.out_of_sync {
                                warn!("Detected comms watchdog, triggering hard reset");
                                sat.reset_flags.reset_comms();
                            }
                            pending_telemetry.comms_status = comms_status.into();
                        }
                        CommsResponse::RecvGroundMessage(msg) => {
                            process_ground_to_sat_msg(msg, &mut self.channels);
                        }
                        // TODO - handle CommsResponse::LastKnownGpsPosition
                        _ => (),
                    }
                }

                if let Some(imu_sample) = imu_sample.take() {
                    pending_telemetry.imu = imu_sample.into();
                }
                if let Some(ImuResponse::Status(imu_status)) = imu_res.take() {
                    if imu_status.error_register.out_of_sync {
                        warn!("Detected IMU watchdog, triggering hard reset");
                        sat.reset_flags.reset_imu();
                    }
                    pending_telemetry.imu_status = imu_status.into();
                }

                if let Some(VisionResponse::Status(vision_status)) = vision_res.take() {
                    if vision_status.error_register.out_of_sync {
                        warn!("Detected vision watchdog, triggering hard reset");
                        sat.reset_flags.reset_vision();
                    }
                    pending_telemetry.vision_status = vision_status.into();
                }

                pending_telemetry.compute_status = ComputeStatus {
                    temperature: self.temp_sensor.temperature(),
                    error_register: self.error_register,
                }
                .into();

                if pending_telemetry.is_complete() || &sat.rtc >= timeout_send_time {
                    self.last_sequence_number += 1;
                    let _ = self
                        .channels
                        .comms_cmd_tx
                        .try_send(CommsCommand::SendGroundMessage(Box::new(
                            SatToGroundMessage {
                                seq: self.last_sequence_number,
                                satellite_id: sat.id.satcat_id,
                                satellite_name: sat.id.name,
                                sat_send_timestamp: sat.rtc,
                                body: SatToGroundMessageBody::SatelliteTelemetry(Box::new(
                                    pending_telemetry.as_ref().clone(),
                                )),
                            },
                        )));
                    self.telemetry_state = TelemetryState::Idle {
                        collect_time: *next_collect_time,
                    };
                }
            }
        }

        #[allow(clippy::single_match)]
        match power_res {
            Some(PowerResponse::Status { .. }) => {
                warn!("PowerResponse::Status message received outside the telemetry collection state; dropping");
            }
            None => {}
        }

        match comms_res {
            Some(CommsResponse::GpsPosition(_)) => {
                warn!("CommsResponse::GpsPosition message received outside the telemetry collection state; dropping");
            }
            Some(CommsResponse::LastKnownGpsPosition(_)) => {
                warn!("CommsResponse::LastKnownGpsPosition message received outside the telemetry collection state; dropping");
            }
            Some(CommsResponse::Status(_)) => {
                warn!("CommsResponse::Status message received outside the telemetry collection state; dropping");
            }
            Some(CommsResponse::RecvGroundMessage(msg)) => {
                process_ground_to_sat_msg(msg, &mut self.channels);
            }

            None => {}
        }
    }
}

fn process_ground_to_sat_msg(msg: GroundToSatMessage, channels: &mut ComputeChannels) {
    match msg {
        GroundToSatMessage::PrioritizeIrEvent { sat: _, source_id } => {
            let _ = channels
                .vision_cmd_tx
                .try_send(VisionCommand::PrioritizeIrEvent(source_id));
        }
        GroundToSatMessage::ClearSatelliteErrorFlag { sat: _, flag } => match flag {
            SatErrorFlag::ImuDegraded => {
                let _ = channels.imu_cmd_tx.try_send(ImuCommand::Reset);
            }
            SatErrorFlag::ImuDataInconsistency => {
                let _ = channels
                    .imu_cmd_tx
                    .try_send(ImuCommand::ClearDataInconsistency);
            } // NOTE: the rest are watchdog assertions, which trigger component hard-resets and auto
            // auto-clear
            _ => (),
        },
    }
}
