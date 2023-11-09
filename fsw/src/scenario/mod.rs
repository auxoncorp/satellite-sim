use na::Vector3;
use nav_types::{ECEF, WGS84};
use oorandom::Rand64;
use std::{collections::HashMap, path::Path};
use tracing::info;
use types42::prelude::SpacecraftIndex;

use self::config::Config;
use crate::{
    ground_station::{GroundStationId, RelayGroundStationConfig},
    satellite::{SatelliteConfig, SATELLITE_IDS},
    system::{CameraSourceId, IREvent},
    units::{Angle, Length, LuminousIntensity, Ratio, Time, Velocity},
};

pub mod config;
pub mod nominal;

#[derive(Debug, Clone)]
pub struct Scenario {
    pub ground_stations: HashMap<GroundStationId, RelayGroundStationConfig>,
    pub ir_events: Vec<ScheduledIREvent>,
    pub satellite_configs: HashMap<SpacecraftIndex, SatelliteConfig>,
}

impl Scenario {
    pub fn load<P: AsRef<Path>>(config: Option<P>) -> Self {
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

            let ground_stations = if cfg.relay_ground_stations.is_empty() {
                all_known_ground_stations()
            } else {
                cfg.relay_ground_stations
                    .into_iter()
                    .map(|rgs| {
                        let id = rgs.id;
                        (
                            id,
                            RelayGroundStationConfig {
                                id,
                                name: rgs.name,
                                position: WGS84::from_degrees_and_meters(
                                    rgs.latitude,
                                    rgs.longitude,
                                    0.0, // MSL
                                )
                                .into(),
                                enable_time_sync: rgs.enable_time_sync,
                                rtc_drift: Ratio::from_f64(rgs.rtc_drift),
                            },
                        )
                    })
                    .collect()
            };

            let ir_events = if cfg.ir_events.is_empty() {
                basic_scheduled_ir_events()
            } else {
                cfg.ir_events
                    .into_iter()
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

            Self {
                ground_stations,
                ir_events,
                satellite_configs,
            }
        } else {
            info!("Loading default nominal scenario");
            Self {
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
