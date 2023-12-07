use na::Vector3;
use nav_types::{ECEF, WGS84};
use oorandom::Rand64;
use std::{collections::HashMap, path::Path};
use tracing::info;
use types42::prelude::SpacecraftIndex;

use self::config::Config;
use crate::{
    ground_station::{
        ConsolidatedGroundStationConfig, CorrelationConfig, GroundStationId, IROperatorConfig,
        IntensityAnalysisConfig, MissionControlUIConfig, RackConfig, RelayGroundStationConfig,
        ResultSelectionConfig, SatOperatorConfig, SynthesisConfig, TimeSourceConfig,
        VelocityAnalysisConfig,
    },
    satellite::{SatelliteConfig, SATELLITE_IDS},
    system::{CameraSourceId, IREvent},
    units::{Angle, Length, LuminousIntensity, Time, Velocity},
};

pub mod config;
pub mod nominal;

#[derive(Debug, Clone)]
pub struct Scenario {
    pub consolidated_ground_station_config: ConsolidatedGroundStationConfig,
    pub ground_stations: HashMap<GroundStationId, RelayGroundStationConfig>,
    pub ir_events: Vec<ScheduledIREvent>,
    pub satellite_configs: HashMap<SpacecraftIndex, SatelliteConfig>,
}

#[derive(Debug, Copy, Clone, Default)]
pub struct ScenarioOptions {
    pub enable_all_mutators: bool,
}

impl Scenario {
    pub fn load_with_options<P: AsRef<Path>>(opts: ScenarioOptions, config: Option<P>) -> Self {
        let mut cfg = Self::load_inner(config);

        if opts.enable_all_mutators {
            for sat in cfg.satellite_configs.values_mut() {
                sat.power_config.fault_config.solar_panel_degraded = true;
                sat.power_config.fault_config.watchdog_out_of_sync = true;
                sat.power_config.fault_config.constant_temperature = true;

                sat.compute_config.fault_config.watchdog_out_of_sync = true;

                sat.comms_config.fault_config.gps_offline_rtc_drift = true;
                sat.comms_config.fault_config.gps_offline = true;
                sat.comms_config.fault_config.ground_transceiver_failure = true;
                sat.comms_config
                    .fault_config
                    .ground_transceiver_partial_failure = true;
                sat.comms_config.fault_config.watchdog_out_of_sync = true;

                sat.vision_config.fault_config.watchdog_out_of_sync = true;

                sat.imu_config.fault_config.watchdog_out_of_sync = true;
                sat.imu_config.fault_config.constant_temperature = true;
            }

            for rgs in cfg.ground_stations.values_mut() {
                rgs.fault_config.rtc_drift = true;
                rgs.fault_config.satellite_to_cgs_delay = true;
                rgs.fault_config.cgs_to_satellite_delay = true;
            }

            cfg.consolidated_ground_station_config
                .base_rack_config
                .time_source_config
                .fault_config
                .rtc_drift = true;
            cfg.consolidated_ground_station_config
                .base_rack_config
                .synthesis_config
                .fault_config
                .rack_offline = true;
        }

        cfg
    }

    pub fn load<P: AsRef<Path>>(config: Option<P>) -> Self {
        Self::load_inner(config)
    }

    fn load_inner<P: AsRef<Path>>(config: Option<P>) -> Self {
        // Start with the default satellite configs
        let mut satellite_configs: HashMap<SpacecraftIndex, SatelliteConfig> = (0..SATELLITE_IDS
            .len())
            .map(|idx| {
                let sat_idx = idx as SpacecraftIndex;
                let cfg = nominal::satellite_config(sat_idx);
                (sat_idx, cfg)
            })
            .collect();

        if let Some(cfg_path) = config.as_ref() {
            info!(
                config = %cfg_path.as_ref().display(),
                "Loading scenario from config file",
            );
            let cfg = Config::load(cfg_path);

            // Apply user configs on a per-satellite per-subsystem basis
            for sat_cfg in satellite_configs.values_mut() {
                if cfg.satellite(sat_cfg.id.satcat_id).is_some() {
                    if let Some(c) = cfg.power_config(sat_cfg.id.satcat_id) {
                        sat_cfg.power_config = c;
                    }
                    if let Some(c) = cfg.compute_config(sat_cfg.id.satcat_id) {
                        sat_cfg.compute_config = c;
                    }
                    if let Some(c) = cfg.comms_config(sat_cfg.id.satcat_id) {
                        sat_cfg.comms_config = c;
                    }
                    if let Some(c) = cfg.vision_config(sat_cfg.id.satcat_id) {
                        sat_cfg.vision_config = c;
                    }
                    if let Some(c) = cfg.imu_config(sat_cfg.id.satcat_id) {
                        sat_cfg.imu_config = c;
                    }
                }
            }

            let relays = cfg.relay_ground_station_configs();
            let ground_stations = if relays.is_empty() {
                all_known_ground_stations()
            } else {
                relays.into_iter().map(|rgs| (rgs.id, rgs)).collect()
            };

            let ir_events = if cfg.ir_events.is_empty() {
                basic_scheduled_ir_events()
            } else {
                cfg.ir_events
                    .iter()
                    .cloned()
                    .map(|ire| {
                        ScheduledIREvent::from_degrees_and_meters(
                            Time::from_secs(ire.activate_at),
                            ire.deactivate_at.map(Time::from_secs),
                            ire.ground_truth_id.into(),
                            Angle::from_degrees(ire.latitude),
                            Angle::from_degrees(ire.longitude),
                            Length::from_meters(ire.altitude),
                            Velocity::from_meters_per_second(ire.velocity_east),
                            Velocity::from_meters_per_second(ire.velocity_north),
                            LuminousIntensity::from_candelas(ire.intensity),
                        )
                    })
                    .collect()
            };

            let consolidated_ground_station_config = cfg
                .consolidated_ground_station_config()
                .unwrap_or_else(default_consolidated_ground_station_config);

            Self {
                consolidated_ground_station_config,
                ground_stations,
                ir_events,
                satellite_configs,
            }
        } else {
            info!("Loading default nominal scenario");
            Self {
                consolidated_ground_station_config: default_consolidated_ground_station_config(),
                ground_stations: all_known_ground_stations(),
                ir_events: basic_scheduled_ir_events(),
                satellite_configs,
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct ScheduledIREvent {
    /// Relative time to activate the event
    pub activate_at: Time,
    /// Relative time to deactivate the event
    pub deactivat_at: Option<Time>,
    pub event: IREvent,
}

impl ScheduledIREvent {
    #[allow(clippy::too_many_arguments)]
    pub fn from_degrees_and_meters(
        activate_at: Time,
        deactivate_at: Option<Time>,
        ground_truth_id: i64,
        latitude: Angle,
        longitude: Angle,
        altitude: Length,
        velocity_east: Velocity,
        velocity_north: Velocity,
        intensity: LuminousIntensity,
    ) -> Self {
        if let Some(t1) = deactivate_at.as_ref() {
            debug_assert!(t1 > &activate_at);
        }
        let pos_wgs = WGS84::from_degrees_and_meters(
            latitude.as_degrees(),
            longitude.as_degrees(),
            altitude.as_meters(),
        );
        let pos_ecef = ECEF::from(pos_wgs);

        Self {
            activate_at,
            deactivat_at: deactivate_at,
            event: IREvent {
                ground_truth_id,
                source_id: CameraSourceId::Unassigned,
                location: pos_wgs.into(),
                position: Vector3::new(pos_ecef.x(), pos_ecef.y(), pos_ecef.z()),
                velocity: Vector3::zeros(), // Gets computed at runtime
                velocity_east: velocity_east.as_meters_per_second(),
                velocity_north: velocity_north.as_meters_per_second(),
                intensity,
            },
        }
    }
}

struct GroundTruthIdGen(i64);
impl GroundTruthIdGen {
    fn next(&mut self) -> i64 {
        let id = self.0;
        self.0 += 1;
        id
    }
}

pub fn basic_scheduled_ir_events() -> Vec<ScheduledIREvent> {
    let mut prng = Rand64::new(0);
    let mut ground_truth_id = GroundTruthIdGen(0);

    let mut events = Vec::new();

    {
        let activate_at = Time::from_secs(0.0);
        let active_duration = Time::from_secs(10_000.0);
        for _ in 0..prng.rand_range(2..8) {
            events.push(ScheduledIREvent::from_degrees_and_meters(
                activate_at,
                Some(activate_at + active_duration),
                ground_truth_id.next(),
                Angle::from_radians(3.586730104944e-02 + (prng.rand_float() * 0.1)),
                Angle::from_radians(1.152404170706e+00 + (prng.rand_float() * 0.1)),
                Length::from_kilometers(1000.0 + (prng.rand_float() * 1000.0)),
                Velocity::from_meters_per_second(4000.0 + (prng.rand_float() * 2000.0)),
                Velocity::from_meters_per_second(4000.0 + (prng.rand_float() * 2000.0)),
                LuminousIntensity::from_candelas(prng.rand_float() * 100_000.0),
            ));
        }
    }

    {
        let activate_at = Time::from_secs(100.0 + (prng.rand_float() * 10_000.0));
        let active_duration = Time::from_secs(10_000.0);
        for _ in 0..prng.rand_range(2..12) {
            events.push(ScheduledIREvent::from_degrees_and_meters(
                activate_at,
                Some(activate_at + active_duration),
                ground_truth_id.next(),
                Angle::from_radians(1.0762695642393085 + (prng.rand_float() * 0.1)),
                Angle::from_radians(0.2215893423601145 + (prng.rand_float() * 0.1)),
                Length::from_kilometers(100.0 + (prng.rand_float() * 1000.0)),
                Velocity::from_meters_per_second(2000.0 + (prng.rand_float() * 3000.0)),
                Velocity::from_meters_per_second(-2000.0 + (prng.rand_float() * 3000.0)),
                LuminousIntensity::from_candelas(prng.rand_float() * 100_000.0),
            ));
        }
    }

    for _ in 0..prng.rand_range(2..12) {
        let activate_at = Time::from_secs(500.0 + (prng.rand_float() * 10_000.0));
        let active_duration = Time::from_secs(prng.rand_float() * 10_000.0);
        events.push(ScheduledIREvent::from_degrees_and_meters(
            activate_at,
            Some(activate_at + active_duration),
            ground_truth_id.next(),
            Angle::from_degrees(41.5240 + (prng.rand_float() * 10.0)),
            Angle::from_degrees(105.3188 + (prng.rand_float() * 10.0)),
            Length::from_kilometers(100.0 + (prng.rand_float() * 1000.0)),
            Velocity::from_meters_per_second(-2000.0 + (prng.rand_float() * 3000.0)),
            Velocity::from_meters_per_second(2000.0 + (prng.rand_float() * 3000.0)),
            LuminousIntensity::from_candelas(prng.rand_float() * 100_000.0),
        ));
    }

    events
}

pub fn all_known_ground_stations() -> HashMap<GroundStationId, RelayGroundStationConfig> {
    vec![
        RelayGroundStationConfig::from_wgs84_degrees(
            1,
            "GSFC".into(),
            Angle::from_degrees(37.0),
            Angle::from_degrees(-77.0),
        ),
        RelayGroundStationConfig::from_wgs84_degrees(
            2,
            "South Point".into(),
            Angle::from_degrees(19.0),
            Angle::from_degrees(-155.6),
        ),
        RelayGroundStationConfig::from_wgs84_degrees(
            3,
            "Dongara".into(),
            Angle::from_degrees(-29.0),
            Angle::from_degrees(115.4),
        ),
        RelayGroundStationConfig::from_wgs84_degrees(
            4,
            "Santiago".into(),
            Angle::from_degrees(-33.0),
            Angle::from_degrees(-71.0),
        ),
        RelayGroundStationConfig::from_wgs84_degrees(
            5,
            "SANSA EO".into(),
            Angle::from_degrees(-25.887152),
            Angle::from_degrees(27.707600),
        ),
    ]
    .into_iter()
    .map(|gs| (gs.id, gs))
    .collect()
}

fn default_consolidated_ground_station_config() -> ConsolidatedGroundStationConfig {
    ConsolidatedGroundStationConfig {
        rack_count: 3,
        base_rack_config: RackConfig {
            id: 0,
            time_source_config: TimeSourceConfig {
                fault_config: Default::default(),
            },
            correlation_config: CorrelationConfig {
                correlation_window: Time::from_secs(3.0),
                prune_window: Time::from_secs(10.0),
            },
            velocity_analysis_config: VelocityAnalysisConfig {},
            intensity_analysis_config: IntensityAnalysisConfig {},
            synthesis_config: SynthesisConfig {
                observation_timeout: Time::from_secs(10.0),
                collapse_instensity_threshold: LuminousIntensity::from_candelas(0.02), // TODO ?????
                collapse_velocity_threshold: Velocity::from_meters_per_second(0.1),    // TODO ?????
                collapse_position_threshold: Length::from_meters(100.0),               // TODO ?????
                report_interval: Time::from_secs(5.0),
                fault_config: Default::default(),
            },
        },
        result_selection_config: ResultSelectionConfig {
            selection_rx_window: Time::from_secs(2.0),
        },
        mcui_config: MissionControlUIConfig {},
        ir_operator_config: IROperatorConfig {},
        sat_operator_config: SatOperatorConfig {
            clear_flags_after: Time::from_secs(60.0),
        },
    }
}
