// cargo run --bin temperature-profile --release -- --satcat-id 45026 --sensor focus-cam --duration 345600 --dt 1.0 --toggle-sun-vis-every 43200 /tmp/temp_data.txt

use clap::Parser;
use std::io::prelude::*;
use std::path::PathBuf;
use std::{fs::File, str::FromStr};

use fsw_lib::{
    satellite::{temperature_sensor::TemperatureSensor, SATELLITE_IDS},
    scenario::Scenario,
    units::Time,
};

/// Print a temperature profile over time
#[derive(Parser, Debug)]
#[command(version)]
struct Opts {
    /// Scenario configuration toml file.
    ///
    /// The default nominal scenario is used when not provided.
    #[arg(long)]
    scenario: Option<PathBuf>,

    /// Satcat ID
    #[arg(short = 'i', long)]
    satcat_id: usize,

    /// Which subsystem's sensor to use (imu, scanner-cam, focus-cam, power, compute, comms)
    #[arg(short = 's', long)]
    sensor: Sensor,

    /// Toggle sun visibility every t (starts true)
    #[arg(short = 'v', long)]
    toggle_sun_vis_every: Option<f64>,

    /// Duration in seconds
    #[arg(short = 'd', long)]
    duration: f64,

    /// Time step (dt)
    #[arg(short = 't', long)]
    dt: f64,

    /// Output file path to write
    output: PathBuf,
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
enum Sensor {
    Imu,
    ScannerCam,
    FocusCam,
    Power,
    Compute,
    Comms,
}

impl FromStr for Sensor {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(match s.trim().to_lowercase().as_str() {
            "imu" => Sensor::Imu,
            "scanner-cam" => Sensor::ScannerCam,
            "focus-cam" => Sensor::FocusCam,
            "power" => Sensor::Power,
            "compute" => Sensor::Compute,
            "comms" => Sensor::Comms,
            _ => return Err("Bad sensor type".to_string()),
        })
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let opts = Opts::parse();

    let satcat_id = opts.satcat_id.into();
    let sc_index = SATELLITE_IDS
        .iter()
        .position(|s| s.satcat_id == satcat_id)
        .expect("Satellite with provided satcat_id doesn't exist") as u64;

    let scenario = Scenario::load(opts.scenario);
    let satellite_config = scenario
        .satellite_configs
        .get(&sc_index)
        .expect("Bad satellite index");

    let config = match opts.sensor {
        Sensor::Imu => satellite_config.imu_config.temperature_sensor_config,
        Sensor::ScannerCam => {
            satellite_config
                .vision_config
                .scanner_camera_temperature_sensor_config
        }
        Sensor::FocusCam => {
            satellite_config
                .vision_config
                .focus_camera_temperature_sensor_config
        }
        Sensor::Power => satellite_config.power_config.temperature_sensor_config,
        Sensor::Compute => satellite_config.compute_config.temperature_sensor_config,
        Sensor::Comms => satellite_config.comms_config.temperature_sensor_config,
    };

    let mut sensor = TemperatureSensor::new(config);

    let mut output = File::create(opts.output)?;

    let mut sun_visible = true;
    let mut time = Time::from_secs(0.0);
    let mut sun_vis_timer = Time::from_secs(0.0);
    let dt = Time::from_secs(opts.dt);

    loop {
        if time.as_secs() >= opts.duration {
            break;
        }

        sensor.manual_step(dt, sun_visible);
        let temp = sensor.temperature();

        writeln!(
            &mut output,
            "{} {}",
            time.as_secs(),
            temp.as_degrees_celsius()
        )?;

        time += dt;
        sun_vis_timer += dt;

        if let Some(toggle_vis_interval) = opts.toggle_sun_vis_every {
            if sun_vis_timer.as_secs() >= toggle_vis_interval {
                sun_visible = !sun_visible;
                sun_vis_timer = Time::from_secs(0.0);
            }
        }
    }

    Ok(())
}
