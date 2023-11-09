// cargo run --bin power-profile --release -- --satcat-id 37481 --duration 345600 --dt 1.0 --toggle-sun-vis-every 43200 /tmp/power_data.txt

use clap::Parser;
use std::fs::File;
use std::io::prelude::*;
use std::path::PathBuf;

use fsw_lib::{
    channel::{Step, StepChannel},
    satellite::{
        power::{PowerCommand, PowerResponse, PowerSubsystem},
        SatelliteEnvironment, SatelliteSharedState, SATELLITE_IDS,
    },
    scenario::Scenario,
    sim_info::SimulationInfo,
    units::{Time, Timestamp},
    SimulationComponent,
};
use types42::prelude::{CoarseSunSensor, FswData, UtcTimestamp};

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

    let mut cmd_ch = StepChannel::new();
    let mut status_ch = StepChannel::new();

    let mut power_system = PowerSubsystem::new(
        satellite_config.power_config.clone(),
        cmd_ch.receiver(None),
        status_ch.sender(None),
    );

    let mut cmd_tx = cmd_ch.sender(None);
    let mut status_rx = status_ch.receiver(None);

    let mut common_state = SatelliteSharedState {
        gui: None,
        id: &SATELLITE_IDS[0],
        power_supply_voltage: satellite_config.power_config.battery_max_voltage,
        rtc: Timestamp::epoch(),
        reset_flags: Default::default(),
    };

    let ground_truth_ir_events = vec![];
    let sim_info = SimulationInfo::new();
    let timestamp = UtcTimestamp::default();
    let mut fsw_data: FswData = FswData::default();
    fsw_data.css.insert(
        0,
        CoarseSunSensor {
            index: 0,
            valid: true,
            illum: 1.0,
        },
    );

    let mut output = File::create(opts.output)?;

    let mut sun_visible = true;
    let mut time = Time::from_secs(0.0);
    let mut sun_vis_timer = Time::from_secs(0.0);
    let dt = Time::from_secs(opts.dt);

    {
        let env = SatelliteEnvironment {
            sim_info: &sim_info,
            ground_truth_ir_events: &ground_truth_ir_events,
            timestamp: &timestamp,
            fsw_data: &fsw_data,
        };

        power_system.init(&env, &mut common_state);
    }

    loop {
        if time.as_secs() >= opts.duration {
            break;
        }

        let env = SatelliteEnvironment {
            sim_info: &sim_info,
            ground_truth_ir_events: &ground_truth_ir_events,
            timestamp: &timestamp,
            fsw_data: &fsw_data,
        };

        cmd_tx.try_send(PowerCommand::GetStatus).unwrap();
        cmd_ch.step().unwrap();

        power_system.step(dt, &env, &mut common_state);

        status_ch.step().unwrap();
        let PowerResponse::Status(status) = status_rx.recv().unwrap();

        writeln!(
            &mut output,
            "{} {} {}",
            time.as_secs(),
            common_state.power_supply_voltage.as_volts(),
            status.battery_charge.as_coulombs(),
        )?;

        time += dt;
        sun_vis_timer += dt;

        if let Some(toggle_vis_interval) = opts.toggle_sun_vis_every {
            if sun_vis_timer.as_secs() >= toggle_vis_interval {
                sun_visible = !sun_visible;
                sun_vis_timer = Time::from_secs(0.0);
                fsw_data.css.insert(
                    0,
                    CoarseSunSensor {
                        index: 0,
                        valid: sun_visible,
                        illum: 1.0,
                    },
                );
            }
        }
    }

    Ok(())
}
