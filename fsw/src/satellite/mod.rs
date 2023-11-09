pub mod comms;
pub mod compute;
pub mod imu;
pub mod power;
pub mod temperature_sensor;
pub mod vision;

pub use comms::{CommsConfig, CommsStatus};
pub use compute::{ComputeConfig, ComputeStatus};
pub use imu::{ImuConfig, ImuFaultConfig, ImuSample, ImuStatus};
pub use power::{PowerConfig, PowerFaultConfig, PowerStatus};
pub use temperature_sensor::{
    TemperatureSensorConfig, TemperatureSensorExponential, TemperatureSensorExponentialModelParams,
    TemperatureSensorLinearModelParams, TemperatureSensorModel,
};
pub use vision::{VisionConfig, VisionFaultConfig, VisionStatus};

use channel::Sender;
use comms::CommsSubsystem;
use compute::{ComputeChannels, ComputeSubsystem};
use imu::ImuSubsystem;
use modality_api::AttrVal;
use power::PowerSubsystem;
use vision::VisionSubsystem;

use crate::{
    channel::{self, Receiver, StepChannel},
    ground_station::Relayed,
    ground_truth_ir_events::GroundTruthIrEvents,
    gui::SharedGuiState,
    sim_info::SimulationInfo,
    system::{GroundToSatMessage, SatToGroundMessage, SystemSharedState},
    units::{ElectricPotential, Time, Timestamp},
    SimulationComponent,
};

pub struct Satellite {
    shared_state: SatelliteSharedState,
    channels: Vec<Box<dyn channel::Step>>,

    power: PowerSubsystem,
    comms: CommsSubsystem,
    compute: ComputeSubsystem,
    vision: VisionSubsystem,
    imu: ImuSubsystem,
}

impl Satellite {
    pub fn new(
        config: SatelliteConfig,
        sat_to_ground: Sender<SatToGroundMessage>,
        ground_to_sat: Receiver<Relayed<GroundToSatMessage>>,
    ) -> Self {
        let mut power_cmd = StepChannel::new();
        let mut power_res = StepChannel::new();
        let mut comms_cmd = StepChannel::new();
        let mut comms_res = StepChannel::new();

        let mut vision_detections = StepChannel::new();
        let mut vision_cmd = StepChannel::new();
        let mut vision_res = StepChannel::new();

        let mut imu_sample = StepChannel::new();
        let mut imu_cmd = StepChannel::new();
        let mut imu_res = StepChannel::new();

        let comms = CommsSubsystem::new(
            config.comms_config,
            comms_cmd.receiver(None),
            comms_res.sender(None),
            sat_to_ground,
            ground_to_sat,
        );

        let power = PowerSubsystem::new(
            config.power_config,
            power_cmd.receiver(None),
            power_res.sender(None),
        );

        let vision = VisionSubsystem::new(
            config.vision_config,
            vision_detections.sender(None),
            vision_cmd.receiver(None),
            vision_res.sender(None),
        );

        let imu = ImuSubsystem::new(
            config.imu_config,
            imu_sample.sender(None),
            imu_cmd.receiver(None),
            imu_res.sender(None),
        );

        let compute = ComputeSubsystem::new(
            config.compute_config,
            ComputeChannels {
                power_cmd_tx: power_cmd.sender(None),
                power_res_rx: power_res.receiver(None),
                comms_cmd_tx: comms_cmd.sender(None),
                comms_res_rx: comms_res.receiver(None),
                vision_detections_rx: vision_detections.receiver(None),
                vision_cmd_tx: vision_cmd.sender(None),
                vision_res_rx: vision_res.receiver(None),
                imu_sample_rx: imu_sample.receiver(None),
                imu_cmd_tx: imu_cmd.sender(None),
                imu_res_rx: imu_res.receiver(None),
            },
        );

        // These will be filled in sanely by init()
        let shared_state = SatelliteSharedState {
            gui: None,
            id: config.id,
            power_supply_voltage: ElectricPotential::from_volts(0.0),
            rtc: Timestamp::epoch(),
            reset_flags: Default::default(),
        };

        Satellite {
            shared_state,
            channels: vec![
                Box::new(power_cmd),
                Box::new(power_res),
                Box::new(comms_cmd),
                Box::new(comms_res),
                Box::new(vision_detections),
                Box::new(vision_cmd),
                Box::new(vision_res),
                Box::new(imu_sample),
                Box::new(imu_cmd),
                Box::new(imu_res),
            ],
            power,
            comms,
            compute,
            vision,
            imu,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SatelliteConfig {
    pub id: &'static SatelliteId,
    pub power_config: PowerConfig,
    pub compute_config: ComputeConfig,
    pub comms_config: CommsConfig,
    pub vision_config: VisionConfig,
    pub imu_config: ImuConfig,
}

// These are indexed to match Inp_Sim.txt
#[rustfmt::skip]
pub const SATELLITE_IDS: &[SatelliteId] = &[
    SatelliteId { satcat_id: SatCatId(45026), name: "GALAXY-1"  },
    SatelliteId { satcat_id: SatCatId(46114), name: "GALAXY-2"  },
    SatelliteId { satcat_id: SatCatId(54243), name: "GALAXY-3"  },
    SatelliteId { satcat_id: SatCatId(54244), name: "GALAXY-4"  },
    SatelliteId { satcat_id: SatCatId(54741), name: "GALAXY-5"  },
    SatelliteId { satcat_id: SatCatId(54742), name: "GALAXY-6"  },
    SatelliteId { satcat_id: SatCatId(26410), name: "CLUSTER-1" },
    SatelliteId { satcat_id: SatCatId(26411), name: "CLUSTER-2" },
    SatelliteId { satcat_id: SatCatId(26463), name: "CLUSTER-3" },
    SatelliteId { satcat_id: SatCatId(23051), name: "GOES-1" },
    SatelliteId { satcat_id: SatCatId(23581), name: "GOES-2" },
    SatelliteId { satcat_id: SatCatId(24786), name: "GOES-3" },
    SatelliteId { satcat_id: SatCatId(26352), name: "GOES-4" },
    SatelliteId { satcat_id: SatCatId(26871), name: "GOES-5"  },
];

#[derive(Debug, Hash, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub struct SatCatId(usize);

impl From<usize> for SatCatId {
    fn from(value: usize) -> Self {
        SatCatId(value)
    }
}

impl From<SatCatId> for u64 {
    fn from(val: SatCatId) -> Self {
        val.0 as _
    }
}

impl From<SatCatId> for AttrVal {
    fn from(val: SatCatId) -> Self {
        AttrVal::Integer(val.0 as i64)
    }
}

#[derive(Debug)]
pub struct SatelliteId {
    /// https://en.wikipedia.org/wiki/Satellite_Catalog_Number
    pub satcat_id: SatCatId,

    /// e.g. GALAXY-1
    pub name: &'static str,
}

impl SatelliteId {
    pub fn is_goes(&self) -> bool {
        matches!(self.satcat_id.0, 23051 | 23581 | 24786 | 26352 | 26871)
    }
}

impl<'a> SimulationComponent<'a> for Satellite {
    type SharedState = SystemSharedState;
    type Environment = SatelliteEnvironment<'a>;

    fn init(&mut self, env: &SatelliteEnvironment, sys: &mut Self::SharedState) {
        if let Some(gui) = &sys.gui {
            if self.shared_state.gui.is_none() {
                self.shared_state.gui = Some(gui.clone());
                let gps = env.fsw_data.gps.get(&0).expect("Missing primary GPS");
                gui.borrow_mut().update_satellite(
                    self.shared_state.id.satcat_id,
                    &gps.pos_w,
                    &gps.vel_w,
                );
            }
        }

        self.power.init(env, &mut self.shared_state);
        self.comms.init(env, &mut self.shared_state);
        self.compute.init(env, &mut self.shared_state);
        self.vision.init(env, &mut self.shared_state);
        self.imu.init(env, &mut self.shared_state);
    }

    fn step(
        &mut self,
        dt: Time,
        env: &SatelliteEnvironment,
        _common_state: &mut Self::SharedState,
    ) {
        if let Some(gui) = &self.shared_state.gui {
            let gps = env.fsw_data.gps.get(&0).expect("Missing primary GPS");
            gui.borrow_mut().update_satellite(
                self.shared_state.id.satcat_id,
                &gps.pos_w,
                &gps.vel_w,
            );
        }

        self.comms.step(dt, env, &mut self.shared_state);
        self.power.step(dt, env, &mut self.shared_state);
        self.vision.step(dt, env, &mut self.shared_state);
        self.imu.step(dt, env, &mut self.shared_state);
        self.compute.step(dt, env, &mut self.shared_state);

        for ch in self.channels.iter_mut() {
            ch.step().expect("Error stepping channel");
        }

        // Manage out-of-band hard resets
        if self.shared_state.reset_flags.fetch_clear_comms_reset() {
            self.comms.reset(env, &mut self.shared_state);
        }
        if self.shared_state.reset_flags.fetch_clear_power_reset() {
            self.power.reset(env, &mut self.shared_state);
        }
        if self.shared_state.reset_flags.fetch_clear_vision_reset() {
            self.vision.reset(env, &mut self.shared_state);
        }
        if self.shared_state.reset_flags.fetch_clear_imu_reset() {
            self.imu.reset(env, &mut self.shared_state);
        }
        if self.shared_state.reset_flags.fetch_clear_compute_reset() {
            self.compute.reset(env, &mut self.shared_state);
        }
    }
}

/// Information that all components have (mutable) access to during
/// their step function.
pub struct SatelliteSharedState {
    pub gui: Option<SharedGuiState>,

    pub id: &'static SatelliteId,
    pub power_supply_voltage: ElectricPotential,
    pub rtc: Timestamp,

    /// Flags managed by the compute subsystem to control hard resets
    /// of each subsystem
    pub reset_flags: ResetFlags,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ResetFlags {
    reset_imu: bool,
    reset_vision: bool,
    reset_power: bool,
    reset_comms: bool,
    reset_compute: bool,
}

impl ResetFlags {
    pub fn reset_imu(&mut self) {
        self.reset_imu = true;
    }
    pub fn reset_vision(&mut self) {
        self.reset_vision = true;
    }
    pub fn reset_power(&mut self) {
        self.reset_power = true;
    }
    pub fn reset_comms(&mut self) {
        self.reset_comms = true;
    }
    pub fn reset_all(&mut self) {
        self.reset_imu = true;
        self.reset_vision = true;
        self.reset_power = true;
        self.reset_comms = true;
        self.reset_compute = true;
    }

    fn fetch_clear_imu_reset(&mut self) -> bool {
        let reset = self.reset_imu;
        self.reset_imu = false;
        reset
    }
    fn fetch_clear_vision_reset(&mut self) -> bool {
        let reset = self.reset_vision;
        self.reset_vision = false;
        reset
    }
    fn fetch_clear_power_reset(&mut self) -> bool {
        let reset = self.reset_power;
        self.reset_power = false;
        reset
    }
    fn fetch_clear_comms_reset(&mut self) -> bool {
        let reset = self.reset_comms;
        self.reset_comms = false;
        reset
    }
    fn fetch_clear_compute_reset(&mut self) -> bool {
        let reset = self.reset_compute;
        self.reset_compute = false;
        reset
    }
}

/// Information derived from the simulation environment.
pub struct SatelliteEnvironment<'a> {
    pub sim_info: &'a SimulationInfo,
    pub timestamp: &'a types42::time::UtcTimestamp,
    pub fsw_data: &'a types42::prelude::FswData,
    /// Ground truth events that are currently active
    pub ground_truth_ir_events: &'a GroundTruthIrEvents,
}
