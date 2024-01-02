use modality_api::TimelineId;
use oorandom::Rand64;
use serde::Serialize;
use tracing::warn;

use crate::{
    channel::{Receiver, Sender},
    event,
    modality::{AttrsBuilder, MODALITY},
    mutator::{watchdog_out_of_sync_descriptor, GenericBooleanMutator},
    point_failure::{PointFailure, PointFailureConfig},
    recv,
    satellite::{
        SatelliteEnvironment, SatelliteId, SatelliteSharedState, TemperatureSensorModel,
        TemperatureSensorRandomIntervalModelParams,
    },
    system::{
        Detections, GroundToSatMessage, SatErrorFlag, SatToGroundMessage, SatToGroundMessageBody,
        SatelliteTelemetry,
    },
    try_send,
    units::{Ratio, Temperature, TemperatureInterval, Time, Timestamp},
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
    telemetry_timer_degraded_drift_ratio: Ratio,

    temp_sensor: TemperatureSensor,
    telemetry_timer_degraded: Option<PointFailure<Temperature>>,
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

impl ComputeConfig {
    pub fn nominal(_sat: &SatelliteId, mut with_variance: Option<&mut Rand64>) -> Self {
        let mut variance = |scale| {
            with_variance
                .as_mut()
                .map(|prng| ((prng.rand_float() * 2.0) - 1.0) * scale)
                .unwrap_or(0.0)
        };

        Self {
            telemetry_rate: Time::from_secs(1.0),
            collect_timeout: Time::from_millis(500.0),
            temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::RandomInterval(
                    TemperatureSensorRandomIntervalModelParams {
                        initial: Temperature::from_degrees_celsius(variance(5.0)),
                        min: Temperature::from_degrees_celsius(-50.0),
                        max: Temperature::from_degrees_celsius(50.0),
                        day: TemperatureInterval::from_degrees_celsius(0.01),
                        night: TemperatureInterval::from_degrees_celsius(0.01),
                    },
                ),
            },
            fault_config: Default::default(),
        }
    }

    pub fn with_fault_config(mut self, fault_config: ComputeFaultConfig) -> Self {
        self.fault_config = fault_config;
        self
    }
}

/// Parameters for the compute point failures.
/// See the requirements doc, section 1.3.4.2.
#[derive(Debug, Clone, Default)]
pub struct ComputeFaultConfig {
    /// Enable the data watchdog execution out-of-sync mutator.
    /// See sections 1.3.4.2 of the requirements doc.
    pub watchdog_out_of_sync: bool,

    /// Parameters for the telemetry timer degraded point failure.
    /// This point failure is motivated by temperature.
    /// The telemetry rate is decreased/scaled by the normalized
    /// temperature when active.
    pub telemetry_timer_degraded: Option<PointFailureConfig<Temperature>>,
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
        b.kv("event.component", ComputeSubsystem::COMPONENT_NAME);
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
        let telemetry_timer_degraded = config
            .fault_config
            .telemetry_timer_degraded
            .as_ref()
            .map(|pf_config| PointFailure::new(pf_config.clone()));

        if telemetry_timer_degraded.is_some() {
            if let TemperatureSensorModel::Linear(_) = config.temperature_sensor_config.model {
                panic!("telemetry_timer_degraded requires a non-linear temperature sensor model");
            }
        }
        Self {
            config,
            channels,

            last_sequence_number: 1,
            telemetry_state: TelemetryState::Idle {
                // real initial value comes in init
                collect_time: Timestamp::epoch(),
            },
            error_register: ComputeErrorRegister { out_of_sync: false },
            telemetry_timer_degraded_drift_ratio: Ratio::from_f64(0.0),

            temp_sensor,
            telemetry_timer_degraded,
            // Mutators are initialized in init_fault_models at sim-init time
            watchdog_out_of_sync: None,

            timeline,
        }
    }

    fn init_fault_models(&mut self, id: &SatelliteId) {
        let base_ctx = [
            ("satellite_name", id.name),
            ("component_name", Self::COMPONENT_NAME),
        ];

        if let Some(pf) = self.telemetry_timer_degraded.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "telemetry_timer_degraded");
        }

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

    fn update_fault_models(&mut self, dt: Time) {
        self.telemetry_timer_degraded_drift_ratio = Ratio::from_f64(0.0);
        if let Some(pf) = self.telemetry_timer_degraded.as_mut() {
            let fault_active = pf.update(dt, self.temp_sensor.temperature());
            if fault_active {
                // already checked config for non-constant temperature model
                self.telemetry_timer_degraded_drift_ratio = self
                    .temp_sensor
                    .normalized_temperature()
                    .unwrap_or_else(|| Ratio::from_f64(0.0));
            }
        }

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

        // Reset point failures
        if let Some(pf) = self.telemetry_timer_degraded.as_mut() {
            pf.reset();
        }

        // Reset state
        self.telemetry_timer_degraded_drift_ratio = Ratio::from_f64(0.0);
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
        event!("init");

        self.temp_sensor.init(env, sat);

        self.telemetry_state = TelemetryState::Idle {
            collect_time: sat.rtc + self.config.telemetry_rate,
        };

        self.init_fault_models(sat.id);
    }

    fn reset(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        event!("reset");
        self.hard_reset(env.sim_info.relative_time, sat);
    }

    fn step(&mut self, dt: Time, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);

        MODALITY
            .process_mutation_plane_messages(std::iter::once(self.watchdog_out_of_sync.as_mut()));

        self.temp_sensor.step(dt, env, sat);

        self.update_fault_models(dt);

        let mut power_res = recv!(&mut self.channels.power_res_rx);
        let mut comms_res = recv!(&mut self.channels.comms_res_rx);
        let mut vision_detections = recv!(&mut self.channels.vision_detections_rx);
        let mut vision_res = recv!(&mut self.channels.vision_res_rx);
        let mut imu_sample = recv!(&mut self.channels.imu_sample_rx);
        let mut imu_res = recv!(&mut self.channels.imu_res_rx);

        if self.error_register.out_of_sync {
            warn!("Detected compute watchdog, triggering hard reset");
            sat.reset_flags.reset_all();
            return;
        }

        if let Some(vision_detections) = vision_detections.take() {
            self.last_sequence_number += 1;
            let _ = try_send!(
                &mut self.channels.comms_cmd_tx,
                CommsCommand::SendGroundMessage(Box::new(SatToGroundMessage {
                    seq: self.last_sequence_number,
                    satellite_id: sat.id.satcat_id,
                    satellite_name: sat.id.name,
                    sat_send_timestamp: sat.rtc,
                    body: SatToGroundMessageBody::Detections(vision_detections),
                },))
            );
        }

        match &mut self.telemetry_state {
            TelemetryState::Idle { collect_time } => {
                if &sat.rtc >= collect_time {
                    let _ = try_send!(&mut self.channels.power_cmd_tx, PowerCommand::GetStatus);
                    let _ = try_send!(&mut self.channels.comms_cmd_tx, CommsCommand::GetStatus);
                    let _ = try_send!(&mut self.channels.comms_cmd_tx, CommsCommand::GetGps);
                    let _ = try_send!(&mut self.channels.vision_cmd_tx, VisionCommand::GetStatus);
                    let _ = try_send!(&mut self.channels.imu_cmd_tx, ImuCommand::GetStatus);

                    let telem_rate = self.config.telemetry_rate
                        + (self.config.telemetry_rate * self.telemetry_timer_degraded_drift_ratio);

                    self.telemetry_state = TelemetryState::Collecting {
                        pending_telemetry: Box::new(SatelliteTelemetry::new()),
                        timeout_send_time: sat.rtc + self.config.collect_timeout,
                        next_collect_time: sat.rtc + telem_rate,
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
                    let _ = try_send!(
                        &mut self.channels.comms_cmd_tx,
                        CommsCommand::SendGroundMessage(Box::new(SatToGroundMessage {
                            seq: self.last_sequence_number,
                            satellite_id: sat.id.satcat_id,
                            satellite_name: sat.id.name,
                            sat_send_timestamp: sat.rtc,
                            body: SatToGroundMessageBody::SatelliteTelemetry(Box::new(
                                pending_telemetry.as_ref().clone(),
                            )),
                        },))
                    );
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
            let _ = try_send!(
                &mut channels.vision_cmd_tx,
                VisionCommand::PrioritizeIrEvent(source_id)
            );
        }
        GroundToSatMessage::ClearSatelliteErrorFlag { sat: _, flag } => match flag {
            SatErrorFlag::ImuDegraded => {
                let _ = try_send!(&mut channels.imu_cmd_tx, ImuCommand::Reset);
            }
            SatErrorFlag::ImuDataInconsistency => {
                let _ = try_send!(&mut channels.imu_cmd_tx, ImuCommand::ClearDataInconsistency);
            } // NOTE: the rest are watchdog assertions, which trigger component hard-resets and auto
            // auto-clear
            _ => (),
        },
    }
}
