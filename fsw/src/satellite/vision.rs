use modality_api::TimelineId;
use na::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use oorandom::Rand64;
use parry3d_f64::{query::PointQuery, shape::Cone};
use serde::Serialize;
use std::collections::HashMap;
use tracing::{debug, warn};
use types42::prelude::GpsIndex;

use crate::{
    channel::{Receiver, Sender, TracedMessage},
    event,
    modality::{kv, AttrsBuilder, MODALITY},
    mutator::{
        disable_vision_stabilizer_descriptor, watchdog_out_of_sync_descriptor,
        GenericBooleanMutator, MutatorActuatorDescriptor,
    },
    point_failure::{PointFailure, PointFailureConfig},
    recv,
    satellite::temperature_sensor::{
        TemperatureSensor, TemperatureSensorConfig, TemperatureSensorModel,
        TemperatureSensorRandomIntervalModelParams,
    },
    satellite::{SatelliteEnvironment, SatelliteId, SatelliteSharedState},
    system::{CameraSourceId, Detections, IREvent},
    try_send,
    units::{
        Angle, AngularVelocity, ElectricPotential, LuminousIntensity, Temperature,
        TemperatureInterval, Time,
    },
    SimulationComponent,
};

const PRIMARY_GPS: GpsIndex = 0;

/// The vision subsystem models a wide-angle "scanner" camera
/// and a narrow-angle "focus" camera using conical
/// viewing volumes.
///
/// The focus camera automatically targets the most intense object in the
/// field of view of the scanner camera.
///
/// It reports IREvent's when objects are within the
/// field of view.
///
/// Simplifications:
/// * Conical FOV: lets us ignore the spacecraft attitude
/// * Optical axis originates at the host spacecraft's body frame: requires no additional extrinsics transformations
/// * Working distance is from spacecraft origin to center of the Earth: so we can ignore objects on the
///   other side of the Earth
#[derive(Debug)]
pub struct VisionSubsystem {
    config: VisionConfig,
    timeline: TimelineId,

    sim_iters_per_interval: u64,
    sim_iters_in_cur_interval: u64,

    scanner_tracker: ObjectTracker,
    focus_tracker: ObjectTracker,

    in_focus_object_ids: Option<(i64, CameraSourceId)>,

    scanner_cam_temp_sensor: TemperatureSensor,
    focus_cam_temp_sensor: TemperatureSensor,

    error_register: VisionErrorRegister,

    active_cooling: Option<PointFailure<Temperature>>,
    scanner_camera_offline: Option<PointFailure<ElectricPotential>>,
    focus_camera_offline: Option<PointFailure<ElectricPotential>>,
    focus_camera_gimbal: Option<PointFailure<Temperature>>,

    watchdog_out_of_sync: Option<GenericBooleanMutator>,
    disable_stabilizer: Option<GenericBooleanMutator>,

    stabilizer_state: DisableVisionStabilizerMutatorState,

    event_tx: Sender<Detections>,
    cmd_rx: Receiver<VisionCommand>,
    res_tx: Sender<VisionResponse>,
}

#[derive(Debug, Clone)]
pub struct VisionConfig {
    pub scanner_field_of_view_angle: Angle,
    pub focus_field_of_view_angle: Angle,
    pub update_interval: Time,
    pub focus_camera_temperature_sensor_config: TemperatureSensorConfig,
    pub scanner_camera_temperature_sensor_config: TemperatureSensorConfig,
    pub focus_camera_disabled: bool,
    pub fault_config: VisionFaultConfig,
}

/// Parameters for the vision subsystem point failures.
/// See the requirements doc, sections 1.3.4.12-1.3.4.14.
#[derive(Debug, Clone, Default)]
pub struct VisionFaultConfig {
    /// Parameters for the active cooling point failure.
    /// See sections 1.3.4.12 of the requirements doc.
    /// This point failure is motivated by the scanner camera temperature.
    pub active_cooling: Option<PointFailureConfig<Temperature>>,

    /// Parameters for the scanner camera outright failure point failure.
    /// See sections 1.3.4.13 of the requirements doc.
    /// This point failure is motivated by power supply voltage.
    pub scanner_camera_offline: Option<PointFailureConfig<ElectricPotential>>,

    /// Parameters for the focus camera outright failure point failure.
    /// See sections 1.3.4.13 of the requirements doc.
    /// This point failure is motivated by power supply voltage.
    pub focus_camera_offline: Option<PointFailureConfig<ElectricPotential>>,

    /// Parameters for the focus camera gimbal point failure.
    /// See sections 1.3.4.14 of the requirements doc.
    /// This point failure is motivated by the focus camera temperature.
    pub focus_camera_gimbal: Option<PointFailureConfig<Temperature>>,

    /// Enable the data watchdog execution out-of-sync mutator.
    /// See sections 1.3.4.2 of the requirements doc.
    pub watchdog_out_of_sync: bool,

    /// Enable the vision orientation stabilizer disable mutator.
    pub disable_stabilizer: bool,
}

impl VisionConfig {
    pub fn nominal(sat: &SatelliteId, mut with_variance: Option<&mut Rand64>) -> Self {
        let mut variance = |scale| {
            with_variance
                .as_mut()
                .map(|prng| ((prng.rand_float() * 2.0) - 1.0) * scale)
                .unwrap_or(0.0)
        };

        Self {
            scanner_field_of_view_angle: if sat.is_goes() {
                Angle::from_degrees(6.0)
            } else {
                Angle::from_degrees(12.0)
            },
            focus_field_of_view_angle: Angle::from_degrees(2.0),
            update_interval: Time::from_secs(1.0),
            scanner_camera_temperature_sensor_config: TemperatureSensorConfig {
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
            focus_camera_temperature_sensor_config: TemperatureSensorConfig {
                model: TemperatureSensorModel::RandomInterval(
                    TemperatureSensorRandomIntervalModelParams {
                        initial: Temperature::from_degrees_celsius(variance(5.0)),
                        min: Temperature::from_degrees_celsius(-40.0),
                        max: Temperature::from_degrees_celsius(40.0),
                        day: TemperatureInterval::from_degrees_celsius(1.0),
                        night: TemperatureInterval::from_degrees_celsius(1.0),
                    },
                ),
            },
            focus_camera_disabled: sat.is_goes(),
            fault_config: Default::default(),
        }
    }

    pub fn with_fault_config(mut self, fault_config: VisionFaultConfig) -> Self {
        self.fault_config = fault_config;
        self
    }

    pub fn with_all_mutators_enabled(mut self, enable_all_mutators: Option<bool>) -> Self {
        if enable_all_mutators.unwrap_or(false) {
            self.fault_config.watchdog_out_of_sync = true;
            self.fault_config.disable_stabilizer = true;
        }
        self
    }
}

#[derive(Debug, Clone)]
pub enum VisionCommand {
    GetStatus,
    PrioritizeIrEvent(CameraSourceId),
}

impl TracedMessage for VisionCommand {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            VisionCommand::GetStatus => vec![
                kv("event.name", "get_status"),
                kv("event.component", VisionSubsystem::COMPONENT_NAME),
            ],
            VisionCommand::PrioritizeIrEvent(source_id) => {
                let mut kvs = vec![
                    kv("event.component", VisionSubsystem::COMPONENT_NAME),
                    kv("event.name", "prioritize_ir_event"),
                    kv("event.source_type", source_id.source_type()),
                ];
                if let Some(source_id) = source_id.source_id() {
                    kvs.push(kv("event.source_id", source_id));
                }
                kvs
            }
        }
    }
}

#[derive(Debug, Clone)]
pub enum VisionResponse {
    Status(VisionStatus),
}

impl TracedMessage for VisionResponse {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        match self {
            VisionResponse::Status(vision_status) => {
                let mut b = AttrsBuilder::new();
                b.kv("event.name", "vision_status");
                b.kv("event.component", VisionSubsystem::COMPONENT_NAME);
                b.with_prefix("event", |b| vision_status.to_attrs(b));
                b.build()
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct VisionStatus {
    pub scanner_camera_temperature: Temperature,
    pub focus_camera_temperature: Temperature,
    pub error_register: VisionErrorRegister,
}

impl VisionStatus {
    pub fn to_attrs(&self, b: &mut AttrsBuilder) {
        b.kv(
            "scanner_camera.temperature",
            self.scanner_camera_temperature.as_degrees_celsius(),
        );
        b.kv(
            "focus_camera.temperature",
            self.focus_camera_temperature.as_degrees_celsius(),
        );
        b.kv("error.out_of_sync", self.error_register.out_of_sync);
        b.kv("error.stabilizer", self.error_register.stabilizer);
    }
}

#[derive(Debug, Copy, Clone, Serialize)]
pub struct VisionErrorRegister {
    /// Watchdog detected out-of-sync execution, subsystem is halted                                                           
    pub out_of_sync: bool,

    /// Stabilizer is malfunctioning, detections may be degraded
    pub stabilizer: bool,
}

impl VisionSubsystem {
    pub const COMPONENT_NAME: &'static str = "vision_subsystem";

    pub fn new(
        config: VisionConfig,
        event_tx: Sender<Detections>,
        cmd_rx: Receiver<VisionCommand>,
        res_tx: Sender<VisionResponse>,
    ) -> Self {
        debug_assert!(config.update_interval.as_secs() > 0.0);

        let scanner_cam_temp_sensor =
            TemperatureSensor::new(config.scanner_camera_temperature_sensor_config);
        let focus_cam_temp_sensor =
            TemperatureSensor::new(config.focus_camera_temperature_sensor_config);

        let active_cooling = config
            .fault_config
            .active_cooling
            .as_ref()
            .map(|pfc| PointFailure::new(pfc.clone()));
        let scanner_camera_offline = config
            .fault_config
            .scanner_camera_offline
            .as_ref()
            .map(|pfc| PointFailure::new(pfc.clone()));
        let focus_camera_offline = config
            .fault_config
            .focus_camera_offline
            .as_ref()
            .map(|pfc| PointFailure::new(pfc.clone()));
        let focus_camera_gimbal = config
            .fault_config
            .focus_camera_gimbal
            .as_ref()
            .map(|pfc| PointFailure::new(pfc.clone()));

        Self {
            config,
            timeline: TimelineId::allocate(),
            sim_iters_per_interval: 0, // Set on first call to step using dt
            sim_iters_in_cur_interval: 0,
            scanner_tracker: ObjectTracker::new(CameraSourceIdTracker::new_scanner()),
            focus_tracker: ObjectTracker::new(CameraSourceIdTracker::new_focus()),
            in_focus_object_ids: None,
            scanner_cam_temp_sensor,
            focus_cam_temp_sensor,
            error_register: VisionErrorRegister {
                out_of_sync: false,
                stabilizer: false,
            },
            active_cooling,
            scanner_camera_offline,
            focus_camera_offline,
            focus_camera_gimbal,
            // Mutators are initialized in init_fault_models at sim-init time
            watchdog_out_of_sync: None,
            disable_stabilizer: None,
            stabilizer_state: DisableVisionStabilizerMutatorState::default(),
            event_tx,
            cmd_rx,
            res_tx,
        }
    }

    /// Returns true if the step loop should be run for the given sim iteration
    fn downsample_step(&mut self, dt: Time) -> bool {
        if self.sim_iters_per_interval == 0 {
            debug_assert!(dt < self.config.update_interval);
            self.sim_iters_per_interval = (self.config.update_interval / dt).as_f64() as u64;
            debug_assert!(self.sim_iters_per_interval != 0);
        }

        // Downsample to update_interval
        self.sim_iters_in_cur_interval += 1;
        if self.sim_iters_in_cur_interval != self.sim_iters_per_interval {
            false
        } else {
            self.sim_iters_in_cur_interval = 0;
            true
        }
    }

    fn init_fault_models(&mut self, id: &SatelliteId) {
        let base_ctx = [
            ("satellite_name", id.name),
            ("component_name", Self::COMPONENT_NAME),
        ];

        if let Some(pf) = self.active_cooling.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "active_cooling");
        }
        if let Some(pf) = self.scanner_camera_offline.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "scanner_camera_offline");
        }
        if let Some(pf) = self.focus_camera_offline.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "focus_camera_offline");
        }
        if let Some(pf) = self.focus_camera_gimbal.as_mut() {
            pf.set_context(base_ctx);
            pf.add_context("name", "focus_camera_gimbal");
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

        self.disable_stabilizer =
            self.config
                .fault_config
                .disable_stabilizer
                .then_some(GenericBooleanMutator::new(
                    disable_vision_stabilizer_descriptor(Self::COMPONENT_NAME, id),
                ));

        if let Some(m) = &self.disable_stabilizer {
            MODALITY.register_mutator(m);
        }
    }

    fn update_fault_models(&mut self, dt: Time, power_supply_voltage: ElectricPotential) {
        if let Some(pf) = self.active_cooling.as_mut() {
            pf.update(dt, self.scanner_cam_temp_sensor.temperature());
        }

        if let Some(pf) = self.scanner_camera_offline.as_mut() {
            pf.update(dt, power_supply_voltage);
        }

        if let Some(pf) = self.focus_camera_offline.as_mut() {
            pf.update(dt, power_supply_voltage);
        }

        if let Some(pf) = self.focus_camera_gimbal.as_mut() {
            pf.update(dt, self.focus_cam_temp_sensor.temperature());
        }

        self.error_register.out_of_sync = self
            .watchdog_out_of_sync
            .as_ref()
            .map(|m| m.is_active())
            .unwrap_or(false);

        let is_active = self
            .disable_stabilizer
            .as_ref()
            .map(|m| m.is_active())
            .unwrap_or(false);
        if is_active != self.stabilizer_state.active {
            self.stabilizer_state.reset(is_active);
        }
        self.error_register.stabilizer = self.stabilizer_state.active;
    }

    fn hard_reset(&mut self, rel_time: Time) {
        warn!(?rel_time, "Vision hard reset");

        // Clear error register
        self.error_register.out_of_sync = false;
        self.error_register.stabilizer = false;

        // Clear tracking state
        self.in_focus_object_ids = None;
        self.scanner_tracker.reset();
        self.focus_tracker.reset();

        // Reset point failures
        if let Some(pf) = self.active_cooling.as_mut() {
            pf.reset();
        }
        if let Some(pf) = self.scanner_camera_offline.as_mut() {
            pf.reset();
        }
        if let Some(pf) = self.focus_camera_offline.as_mut() {
            pf.reset();
        }
        if let Some(pf) = self.focus_camera_gimbal.as_mut() {
            pf.reset();
        }

        if let Some(m) = self.watchdog_out_of_sync.as_mut() {
            MODALITY.clear_mutation(m);
        }

        if let Some(m) = self.disable_stabilizer.as_mut() {
            MODALITY.clear_mutation(m);
        }

        self.stabilizer_state.reset(false);

        // Reset to initial temperature
        self.focus_cam_temp_sensor.reset();

        // Reset to initial temperature
        self.scanner_cam_temp_sensor.reset();

        // Drop message buffers
        self.event_tx.clear();
        self.cmd_rx.clear();
        self.res_tx.clear();
    }
}

impl<'a> SimulationComponent<'a> for VisionSubsystem {
    type SharedState = SatelliteSharedState;
    type Environment = SatelliteEnvironment<'a>;

    fn init(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        MODALITY.emit_sat_timeline_attrs(Self::COMPONENT_NAME, sat.id);
        event!("init");

        self.scanner_cam_temp_sensor.init(env, sat);
        self.focus_cam_temp_sensor.init(env, sat);

        self.init_fault_models(sat.id);

        if let Some(mut gui) = sat.gui.as_ref().map(|gui| gui.borrow_mut()) {
            let gps = env
                .fsw_data
                .gps
                .get(&PRIMARY_GPS)
                .expect("Missing primary GPS");

            let scanner_camera = Camera::new(self.config.scanner_field_of_view_angle, &gps.pos_w);
            gui.update_satellite_scanner_camera(
                sat.id.satcat_id,
                scanner_camera.viewing_volume_radius,
                scanner_camera.viewing_volume_half_height * 2.0,
                &scanner_camera.isometry,
            );

            let focus_camera = Camera::new(self.config.focus_field_of_view_angle, &gps.pos_w);
            gui.update_satellite_focus_camera(
                sat.id.satcat_id,
                focus_camera.viewing_volume_radius,
                focus_camera.viewing_volume_half_height * 2.0,
                &focus_camera.isometry,
            );
        }
    }

    fn reset(&mut self, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);
        event!("reset");
        self.hard_reset(env.sim_info.relative_time);
    }

    fn step(&mut self, dt: Time, env: &SatelliteEnvironment, sat: &mut SatelliteSharedState) {
        let _timeline_guard = MODALITY.set_current_timeline(self.timeline, sat.rtc);

        let mutators = [
            self.watchdog_out_of_sync.as_mut().map(|m| m.as_dyn()),
            self.disable_stabilizer.as_mut().map(|m| m.as_dyn()),
        ];
        MODALITY.process_mutation_plane_messages(mutators.into_iter());

        self.scanner_cam_temp_sensor.step(dt, env, sat);
        self.focus_cam_temp_sensor.step(dt, env, sat);

        self.update_fault_models(dt, sat.power_supply_voltage);

        if let Some(cmd) = recv!(&mut self.cmd_rx) {
            match cmd {
                VisionCommand::GetStatus => {
                    let _ = try_send!(
                        &mut self.res_tx,
                        VisionResponse::Status(VisionStatus {
                            scanner_camera_temperature: self.scanner_cam_temp_sensor.temperature(),
                            focus_camera_temperature: self.focus_cam_temp_sensor.temperature(),
                            error_register: self.error_register,
                        })
                    );
                }
                VisionCommand::PrioritizeIrEvent(source_id) => {
                    let mut tracked_objects = self
                        .focus_tracker
                        .grount_truth_id_to_sid
                        .iter()
                        .chain(self.scanner_tracker.grount_truth_id_to_sid.iter());

                    if let Some(gt_id) =
                        tracked_objects.find_map(
                            |(gt_id, &sid)| if sid == source_id { Some(gt_id) } else { None },
                        )
                    {
                        // Either the scanner or the focus tracker knows about this ID
                        self.in_focus_object_ids = (*gt_id, source_id).into();

                        debug!(
                            sat = sat.id.name,
                            ground_truth_id = gt_id,
                            source_id = ?source_id,
                            rel_time = env.sim_info.relative_time.as_secs(),
                            "Prioritized IR event"
                        );
                        event!(
                            "prioritize_ir_event",
                            gt_and_src_id_attrs(*gt_id, source_id),
                        );
                    } else {
                        debug!(
                            sat = sat.id.name,
                            source_id = ?source_id,
                            rel_time = env.sim_info.relative_time.as_secs(),
                            "Requested prioritize IR event not tracked"
                        );
                        event!("prioritize_ir_event_invalid");
                    }
                }
            }
        }

        // If we're halted, don't run camera models
        if self.error_register.out_of_sync {
            if let Some(mut gui) = sat.gui.as_ref().map(|gui| gui.borrow_mut()) {
                gui.remove_satellite_scanner_camera(sat.id.satcat_id);
                gui.remove_satellite_focus_camera(sat.id.satcat_id);
            }
            return;
        }

        // Downsample to update_interval for IR event tracking/updates
        if !self.downsample_step(dt) {
            return;
        }

        let gps = env
            .fsw_data
            .gps
            .get(&PRIMARY_GPS)
            .expect("Missing primary GPS");

        let focus_camera_disabled = self
            .focus_camera_offline
            .as_ref()
            .map(|pf| pf.is_active())
            .unwrap_or(false)
            || self.config.focus_camera_disabled;

        let mut scanner_camera = Camera::new(self.config.scanner_field_of_view_angle, &gps.pos_w);
        let mut focus_camera = Camera::new(self.config.focus_field_of_view_angle, &gps.pos_w);

        // Run containment test on the scanner camera, if not offline
        let mut detection_events = Vec::new();
        if !self
            .scanner_camera_offline
            .as_ref()
            .map(|pf| pf.is_active())
            .unwrap_or(false)
        {
            for gt in env.ground_truth_ir_events.iter() {
                // Test if object is contained in the viewing volume
                if let Some(tracked_event) = self.scanner_tracker.process_event(
                    env.sim_info.relative_time,
                    sat.id,
                    &scanner_camera,
                    &gt.event,
                ) {
                    detection_events.push(tracked_event);
                }
            }
        }

        // The active cooling point failure simulates an extreme reduction
        // in scanner camera operation.
        // No longer have velocity information and we halve the intensity.
        if self
            .active_cooling
            .as_ref()
            .map(|pf| pf.is_active())
            .unwrap_or(false)
        {
            for detection in detection_events.iter_mut() {
                detection.velocity = Vector3::zeros();
                detection.velocity_east = 0.0;
                detection.velocity_north = 0.0;
                detection.intensity =
                    LuminousIntensity::from_candelas(detection.intensity.as_candelas() / 2.0);
            }
        }

        // Remove the in-focus object if no longer in the scanner FOV, focus camera will
        // orient back to origin
        if let Some(in_focus_object_ids) = self.in_focus_object_ids {
            if !detection_events
                .iter()
                .any(|ev| ev.ground_truth_id == in_focus_object_ids.0)
            {
                let _ = self.in_focus_object_ids.take();
                debug!(
                    sat = sat.id.name,
                    ground_truth_id = in_focus_object_ids.0,
                    source_id = ?in_focus_object_ids.1,
                    rel_time = env.sim_info.relative_time.as_secs(),
                    "Lost focus of IR event"
                );
                event!(
                    "lost_focus",
                    gt_and_src_id_attrs(in_focus_object_ids.0, in_focus_object_ids.1),
                );
            }
        }

        // Sort the events detected by the scanner camera by intensity, most intense first
        detection_events.sort_by(|a, b| b.intensity.total_cmp(&a.intensity));

        let mut in_focus_object = self
            .in_focus_object_ids
            .and_then(|(gt, _)| detection_events.iter().find(|ev| ev.ground_truth_id == gt));

        // Set the focus camera to track the most intense object in the scanner FOV (if one exists)
        if let Some(most_intense_scanner_event) = detection_events.first() {
            // If the gimbal failure is not active
            if !self
                .focus_camera_gimbal
                .as_ref()
                .map(|pf| pf.is_active())
                .unwrap_or(false)
            {
                let is_more_intense_than_currently_in_focus = in_focus_object
                    .as_ref()
                    .map(|ev| most_intense_scanner_event.intensity > ev.intensity)
                    .unwrap_or(true);

                if is_more_intense_than_currently_in_focus {
                    debug!(
                        sat = sat.id.name,
                        ground_truth_id = most_intense_scanner_event.ground_truth_id,
                        source_id = ?most_intense_scanner_event.source_id,
                        rel_time = env.sim_info.relative_time.as_secs(),
                        "Focusing on IR event"
                    );
                    event!(
                        "acquired_focus",
                        gt_and_src_id_attrs(
                            most_intense_scanner_event.ground_truth_id,
                            most_intense_scanner_event.source_id,
                        )
                    );

                    self.in_focus_object_ids = (
                        most_intense_scanner_event.ground_truth_id,
                        most_intense_scanner_event.source_id,
                    )
                        .into();
                }
            }
        }

        // Clear out the in-focus object if gimbal failure is active
        if self
            .focus_camera_gimbal
            .as_ref()
            .map(|pf| pf.is_active())
            .unwrap_or(false)
        {
            let _ = in_focus_object.take();
            let _ = self.in_focus_object_ids.take();
        }

        // Align the focus camera with the target, otherwise it stays at the origin
        if let Some(in_focus_object) = in_focus_object.as_ref() {
            focus_camera.set_focal_point_target(&in_focus_object.position);
        }

        // Run through the same containment test with the focus camera, if not offline
        if !focus_camera_disabled {
            for gt in env.ground_truth_ir_events.iter() {
                // Test if object is contained in the viewing volume
                if let Some(tracked_event) = self.focus_tracker.process_event(
                    env.sim_info.relative_time,
                    sat.id,
                    &focus_camera,
                    &gt.event,
                ) {
                    detection_events.push(tracked_event);
                }
            }
        }

        // Handle vision stabilizer mutator
        if self.stabilizer_state.active {
            detection_events.clear();

            self.stabilizer_state.step(dt);

            let fp = self
                .stabilizer_state
                .focal_point_target(&scanner_camera.pos);
            scanner_camera.set_focal_point_target(&fp);

            if !focus_camera_disabled {
                let fp = self.stabilizer_state.focal_point_target(&focus_camera.pos);
                focus_camera.set_focal_point_target(&fp);
            }
        }

        if !detection_events.is_empty() {
            let _ = try_send!(
                &mut self.event_tx,
                Detections {
                    sat_timestamp: sat.rtc,
                    events: detection_events,
                }
            );
        }

        // Update GUI
        if let Some(mut gui) = sat.gui.as_ref().map(|gui| gui.borrow_mut()) {
            if self
                .scanner_camera_offline
                .as_ref()
                .map(|pf| pf.is_active())
                .unwrap_or(false)
            {
                gui.remove_satellite_scanner_camera(sat.id.satcat_id);
            } else {
                gui.update_satellite_scanner_camera(
                    sat.id.satcat_id,
                    scanner_camera.viewing_volume_radius,
                    scanner_camera.viewing_volume_half_height * 2.0,
                    &scanner_camera.isometry,
                );
            }

            if focus_camera_disabled {
                gui.remove_satellite_focus_camera(sat.id.satcat_id);
            } else {
                gui.update_satellite_focus_camera(
                    sat.id.satcat_id,
                    focus_camera.viewing_volume_radius,
                    focus_camera.viewing_volume_half_height * 2.0,
                    &focus_camera.isometry,
                );
            }
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
enum CameraSourceIdTracker {
    /// Wide-angle scanner camera
    Scanner(i64),

    /// Narrow-angle focus camera
    Focus(i64),
}

impl CameraSourceIdTracker {
    pub fn new_scanner() -> Self {
        CameraSourceIdTracker::Scanner(0)
    }

    pub fn new_focus() -> Self {
        CameraSourceIdTracker::Focus(0)
    }

    #[allow(dead_code)]
    pub fn reset(&mut self) {
        match self {
            CameraSourceIdTracker::Scanner(id) => {
                *id = 0;
            }
            CameraSourceIdTracker::Focus(id) => {
                *id = 0;
            }
        }
    }

    pub fn next_id(&mut self) -> CameraSourceId {
        match self {
            CameraSourceIdTracker::Scanner(id) => {
                let next = *id;
                *id += 1;
                CameraSourceId::Scanner(next)
            }
            CameraSourceIdTracker::Focus(id) => {
                let next = *id;
                *id += 1;
                CameraSourceId::Focus(next)
            }
        }
    }
}

#[derive(Debug)]
struct ObjectTracker {
    /// Ground truth IDs to CameraSourceId's
    grount_truth_id_to_sid: HashMap<i64, CameraSourceId>,

    /// Next CameraSourceId to assign to an event
    next_id: CameraSourceIdTracker,
}

impl ObjectTracker {
    pub fn new(base: CameraSourceIdTracker) -> Self {
        Self {
            grount_truth_id_to_sid: Default::default(),
            next_id: base,
        }
    }

    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.grount_truth_id_to_sid.clear();
        self.next_id.reset();
    }

    pub fn process_event(
        &mut self,
        rel_time: Time,
        sat_id: &SatelliteId,
        camera: &Camera,
        ground_truth_event: &IREvent,
    ) -> Option<IREvent> {
        let p = Point3::new(
            ground_truth_event.position.x,
            ground_truth_event.position.y,
            ground_truth_event.position.z,
        );

        if camera.is_point_in_view(&p) {
            // In FOV
            let mut ev = ground_truth_event.clone();
            let source_id = self
                .grount_truth_id_to_sid
                .entry(ev.ground_truth_id)
                .and_modify(|sid| {
                    // We've been tracking it, use our assigned ID
                    debug!(
                        sat = sat_id.name,
                        ground_truth_id = ev.ground_truth_id,
                        source_id = ?sid,
                        rel_time = rel_time.as_secs(),
                        "Tracking IR event"
                    );
                    event!(
                        "tracking_ir_event",
                        gt_and_src_id_attrs(ev.ground_truth_id, *sid),
                    );
                })
                .or_insert_with(|| {
                    // Newly detected, assign it an ID
                    let sid = self.next_id.next_id();
                    debug!(
                        sat = sat_id.name,
                        ground_truth_id = ev.ground_truth_id,
                        source_id = ?sid,
                        rel_time = rel_time.as_secs(),
                        "Detected IR event"
                    );
                    event!(
                        "detected_ir_event",
                        gt_and_src_id_attrs(ev.ground_truth_id, sid),
                    );
                    sid
                });
            ev.source_id = *source_id;
            Some(ev)
        } else {
            // No longer tracking this object or never were
            let maybe_was_tracking = self
                .grount_truth_id_to_sid
                .remove(&ground_truth_event.ground_truth_id);
            if let Some(sid) = maybe_was_tracking {
                debug!(
                    sat = sat_id.name,
                    ground_truth_id = ground_truth_event.ground_truth_id,
                    source_id = ?sid,
                    rel_time = rel_time.as_secs(),
                    "Lost IR event"
                );
                event!(
                    "lost_ir_event",
                    gt_and_src_id_attrs(ground_truth_event.ground_truth_id, sid),
                );
            }
            None
        }
    }
}

struct Camera {
    /// Position of the camera in the world frame [m]
    pos: Vector3<f64>,
    /// Viewing volume half-height along the principal axis [m]
    viewing_volume_half_height: f64,
    /// Radius of the viewing_volume cone [m]
    viewing_volume_radius: f64,
    /// Translation of the midpoint of the viewing volumne relative to Earth
    /// and rototation relative to the Earth in the world frame
    isometry: Isometry3<f64>,
    /// Viewing volume
    viewing_volume: Cone,
}

impl Camera {
    /// Create a new camera model with the given FOV
    /// and location in the world frame
    pub fn new(fov: Angle, pos_w: &Vector3<f64>) -> Self {
        // Distance from ECEF origin to satellite cm, meters
        let viewing_volume_height = pos_w.magnitude();
        let viewing_volume_half_height = viewing_volume_height / 2.0;

        // Angle from the principal axis, radians
        let theta = fov.as_radians() / 2.0;

        // Radius of the viewing_volume cone
        let viewing_volume_radius = theta.tan() * viewing_volume_height;

        // Construct a cone, from satellite origin in the W frame
        // to the center of the Earth
        // The principal axis is aligned with the Y axis
        let viewing_volume = Cone::new(viewing_volume_half_height, viewing_volume_radius);

        // Setup the isometry to transform the viewing volume
        // to our GPS/satellite position, orientated towards the Earth
        let ref_point = Vector3::new(0.0, viewing_volume_half_height, 0.0);
        let rotation =
            UnitQuaternion::rotation_between(&ref_point, pos_w).expect("Bad GPS rotation");

        // Half the distance since the viewing volume origin needs to be offset (half-height)
        let translation = Translation3::from(rotation * ref_point);

        let isometry = Isometry3 {
            rotation,
            translation,
        };

        Self {
            pos: *pos_w,
            viewing_volume_half_height,
            viewing_volume_radius,
            isometry,
            viewing_volume,
        }
    }

    pub fn set_focal_point_target(&mut self, fp: &Vector3<f64>) {
        // Vector between the camera position and the target
        let pos_rel_to_fp = self.pos - fp;

        // Principal axis of the viewing volume cone
        let ref_point = Vector3::new(0.0, self.viewing_volume_half_height, 0.0);

        let rotation =
            UnitQuaternion::rotation_between(&ref_point, &pos_rel_to_fp).expect("Bad GPS rotation");

        let translation = Translation3::from(self.pos - (rotation * ref_point));

        self.isometry = Isometry3 {
            rotation,
            translation,
        };
    }

    pub fn is_point_in_view(&self, p: &Point3<f64>) -> bool {
        self.viewing_volume.contains_point(&self.isometry, p)
    }
}

fn gt_and_src_id_attrs(
    gt_id: i64,
    src_id: CameraSourceId,
) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
    let mut b = AttrsBuilder::new();
    b.kv("event.ground_truth_id", gt_id);
    b.kv("event.source_type", src_id.source_type());
    if let Some(id) = src_id.source_id() {
        b.kv("event.source_id", id);
    }
    b.build()
}

#[derive(Debug)]
struct DisableVisionStabilizerMutatorState {
    active: bool,
    angular_rate: AngularVelocity,
    angle: Angle,
}

impl Default for DisableVisionStabilizerMutatorState {
    fn default() -> Self {
        Self {
            active: false,
            angular_rate: AngularVelocity::from_degrees_per_second(1.0),
            angle: Angle::from_degrees(0.0),
        }
    }
}

impl DisableVisionStabilizerMutatorState {
    const MAX_ANGLE_DEG: f64 = 85.0;

    // TODO - it'd be nice for this to ramp back to normal
    fn reset(&mut self, active: bool) {
        self.active = active;
        self.angular_rate = AngularVelocity::from_degrees_per_second(1.0);
        self.angle = Angle::from_degrees(0.0);
    }

    fn step(&mut self, dt: Time) {
        let next_angle = self.angle + (self.angular_rate * dt);
        self.angle = Angle::from_degrees(next_angle.as_degrees().clamp(0.0, Self::MAX_ANGLE_DEG));
    }

    fn focal_point_target(&self, cam_pos: &Vector3<f64>) -> Vector3<f64> {
        // Focal point starts at the ECEF origin
        let fp = Vector3::zeros();

        let pos_rel_to_fp = fp - cam_pos;

        let rot = UnitQuaternion::from_scaled_axis(Vector3::x() * self.angle.as_radians());

        rot * pos_rel_to_fp
    }
}
