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
    ground_truth_ir_events::ScheduledIREvent,
    ir_event_generator::IREventGenerator,
    satellite::{
        CommsConfig, ComputeConfig, ImuConfig, PowerConfig, SatelliteConfig, VisionConfig,
        SATELLITE_IDS,
    },
    units::{Angle, Length, LuminousIntensity, Time, Velocity},
};

pub mod config;

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
    pub config_prng_seed: Option<u64>,
    pub random_ir_events: Option<usize>,
}

impl Scenario {
    pub fn load_with_options<P: AsRef<Path>>(opts: ScenarioOptions, config: Option<P>) -> Self {
        Self::load_inner(opts, config)
    }

    pub fn load<P: AsRef<Path>>(config: Option<P>) -> Self {
        Self::load_inner(Default::default(), config)
    }

    fn load_inner<P: AsRef<Path>>(opts: ScenarioOptions, config: Option<P>) -> Self {
        let cfg = if let Some(cfg_path) = config.as_ref() {
            info!(
                config = %cfg_path.as_ref().display(),
                "Loading scenario from config file",
            );
            Config::load(cfg_path)
        } else {
            info!("Loading default nominal scenario");
            Config::default()
        };

        // NOTE: the SATELLITE_IDS array is ordered to match what we expect from 42 exactly
        let mut satellite_configs: HashMap<SpacecraftIndex, SatelliteConfig> = Default::default();
        for (idx, sat) in SATELLITE_IDS.iter().enumerate() {
            let sat_idx = idx as SpacecraftIndex;
            // So that each satellite has slightly different initial conditions
            let mut prng = Rand64::new((opts.config_prng_seed.unwrap_or(0) + sat_idx).into());
            let apply_variance = cfg.apply_variance.unwrap_or(true);
            let enable_all_mutators = cfg.enable_all_mutators(sat);

            let sat_cfg = SatelliteConfig {
                id: sat,
                power_config: cfg.power_config(sat, &mut prng).unwrap_or_else(|| {
                    PowerConfig::nominal(sat, apply_variance.then_some(&mut prng))
                        .with_all_mutators_enabled(enable_all_mutators)
                }),
                compute_config: cfg.compute_config(sat, &mut prng).unwrap_or_else(|| {
                    ComputeConfig::nominal(sat, apply_variance.then_some(&mut prng))
                        .with_all_mutators_enabled(enable_all_mutators)
                }),
                comms_config: cfg.comms_config(sat, &mut prng).unwrap_or_else(|| {
                    CommsConfig::nominal(sat, apply_variance.then_some(&mut prng))
                        .with_all_mutators_enabled(enable_all_mutators)
                }),
                vision_config: cfg.vision_config(sat, &mut prng).unwrap_or_else(|| {
                    VisionConfig::nominal(sat, apply_variance.then_some(&mut prng))
                        .with_all_mutators_enabled(enable_all_mutators)
                }),
                imu_config: cfg.imu_config(sat, &mut prng).unwrap_or_else(|| {
                    ImuConfig::nominal(sat, apply_variance.then_some(&mut prng))
                        .with_all_mutators_enabled(enable_all_mutators)
                }),
            };
            satellite_configs.insert(sat_idx, sat_cfg);
        }

        let relays = cfg.relay_ground_station_configs();
        let mut ground_stations = if relays.is_empty() {
            all_known_ground_stations()
        } else {
            relays.into_iter().map(|rgs| (rgs.id, rgs)).collect()
        };

        let ir_events = if let Some(num_events) = opts.random_ir_events {
            let gen = IREventGenerator {
                num_events,
                ..Default::default()
            };
            gen.generate()
        } else if let Some(gen) = cfg.ir_event_generator() {
            gen.generate()
        } else if cfg.ir_events.is_empty() {
            IREventGenerator::default().generate()
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

        let mut consolidated_ground_station_config = cfg
            .consolidated_ground_station_config()
            .unwrap_or_else(default_consolidated_ground_station_config);

        if opts.enable_all_mutators || cfg.enable_all_mutators.unwrap_or(false) {
            for sat in satellite_configs.values_mut() {
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

            for rgs in ground_stations.values_mut() {
                rgs.fault_config.rtc_drift = true;
                rgs.fault_config.satellite_to_cgs_delay = true;
                rgs.fault_config.cgs_to_satellite_delay = true;
            }

            consolidated_ground_station_config
                .base_rack_config
                .time_source_config
                .fault_config
                .rtc_drift = true;
            consolidated_ground_station_config
                .base_rack_config
                .synthesis_config
                .fault_config
                .rack_offline = true;
        }

        Self {
            consolidated_ground_station_config,
            ground_stations,
            ir_events,
            satellite_configs,
        }
    }
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
