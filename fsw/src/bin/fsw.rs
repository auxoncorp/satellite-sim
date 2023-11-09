use clap::Parser;
use protocol42::{parse_telemetry, ParseErrorExt, ACK_MSG, EOF_TOKEN};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::{fs::File, io};
use std::{
    io::{prelude::*, BufReader},
    time::Duration,
};

use fsw_lib::{
    external_mission_control::send_tle_set,
    gui::{GuiState, RenderContext, SharedGuiState},
    modality::MODALITY,
    scenario::Scenario,
    sim_info::SimulationInfo,
    system::{System, SystemEnvironment, SystemSharedState},
    units::Timestamp,
    SimulationComponent,
};

#[derive(Parser, Debug)]
#[command(version)]
struct Opts {
    /// Enable the dev GUI
    #[arg(long, group = "gui")]
    dev_gui: bool,

    /// Pause the simulation after the first telemetry message
    #[arg(long, requires = "gui")]
    pause: bool,

    /// Scenario configuration toml file.
    ///
    /// The default nominal scenario is used when not provided.
    #[arg(long)]
    scenario: Option<PathBuf>,

    /// The optional address:port of the mission control TCP server to connect to
    #[arg(long, requires = "mc")]
    mission_control: Option<String>,

    /// When connected to an external mission control server, supply the
    /// satellite orbit TLE entries file (i.e. SAT_TLE.txt).
    #[arg(long = "tle", group = "mc")]
    tle_entries_path: Option<PathBuf>,

    /// The 42 data source. This can either be an address:port combination
    /// or a file path for import mode
    #[arg(default_value = "127.0.0.1:10001")]
    data_source: String,
}

const STEP_SUBSAMPLING: usize = 100;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();
    let opts = Opts::parse();

    let intr = interruptor::Interruptor::new();
    let intr_clone = intr.clone();
    ctrlc::set_handler(move || {
        if intr_clone.is_set() {
            let exit_code = if cfg!(target_family = "unix") {
                // 128 (fatal error signal "n") + 2 (control-c is fatal error signal 2)
                130
            } else {
                // Windows code 3221225786
                // -1073741510 == C000013A
                -1073741510
            };
            std::process::exit(exit_code);
        } else {
            intr_clone.set();
        }
    })?;

    MODALITY.connect();

    let external_mission_control = if let Some(mc_sock_addr) = &opts.mission_control {
        let tle_entries_path = opts
            .tle_entries_path
            .as_ref()
            .expect("Missing TLE entries file option");
        let mut s = TcpStream::connect(mc_sock_addr)?;
        send_tle_set(tle_entries_path, &mut s);
        Some(s)
    } else {
        None
    };

    let mut sys_state = SystemSharedState::new(if opts.dev_gui {
        Some(GuiState::new_shared("Dev GUI", opts.pause))
    } else {
        None
    })?;

    let mut line_buf = String::with_capacity(1024);
    let mut msg_buf = String::with_capacity(4096);
    let maybe_file_path = Path::new(&opts.data_source);
    let data_source = if maybe_file_path.exists() {
        println!("Opening '{}'", opts.data_source);
        DatSource::File(File::open(maybe_file_path)?)
    } else {
        println!("Connecting to '{}'", opts.data_source);
        loop {
            if let Ok(stream) = TcpStream::connect(opts.data_source.clone()) {
                break DatSource::TcpStream(stream);
            } else {
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    };

    let mut msg_reader = BufReader::new(data_source);

    let mut system = System::new(Scenario::load(opts.scenario), external_mission_control);
    let mut sim_info = SimulationInfo::new();
    let mut last_t: Option<Timestamp> = None;

    'outer: loop {
        if intr.is_set() {
            break 'outer;
        }

        MODALITY.poll_mutation_plane();

        // Buffer lines until we see EOF_TOKEN before running the msg parser
        'io: loop {
            line_buf.clear();
            let num_bytes = msg_reader.read_line(&mut line_buf)?;
            if num_bytes == 0 {
                // 0 means EOF, sim must be dead
                break 'outer;
            }

            msg_buf.push_str(&line_buf);

            if line_buf.starts_with(EOF_TOKEN) {
                break 'io;
            }
        }

        let telem = match parse_telemetry(&msg_buf) {
            Ok((_, telem)) => telem,
            Err(e) => {
                if !e.is_failure() {
                    continue;
                } else {
                    // Elide some borrow issues with the msg_buf and nom error variant by turning
                    // it into a string
                    return Err(e.to_string().into());
                }
            }
        };

        msg_buf.clear();

        sim_info.sim_step(Timestamp::from_utc(telem.timestamp));

        if sim_info.sim_iteration % 10 == 0 {
            println!("TIME: {}", telem.timestamp);
        }

        // Just a sanity check in case the msg reader misses a msg
        let new_t = Timestamp::from_utc(telem.timestamp);
        MODALITY.set_sim_time(new_t);
        if let Some(t) = last_t {
            let dt = new_t - t;
            if dt.as_millis() as i64 != 10_000 {
                println!(
                    "TIME: {} ({})",
                    telem.timestamp,
                    telem.timestamp.format(protocol42::TIMESTAMP_UTC_FORMAT)
                );
                println!("     ms: {}", new_t.as_millis());
                println!("  dt_ms: {}", dt.as_millis());
                println!("Last ms: {}", t.as_millis());
                panic!("You must have changed dt in config/Inp_Sim.txt, expected 10.0s.");
            }
        }

        match last_t {
            Some(t) => {
                let dt = (new_t - t) / STEP_SUBSAMPLING;
                for subsample_idx in 0..(STEP_SUBSAMPLING - 1) {
                    sim_info.fsw_step(dt);

                    let env = SystemEnvironment {
                        sim_info: &sim_info,
                        telemetry: &telem,
                    };
                    system.step(dt, &env, &mut sys_state);

                    let ctx = if subsample_idx == 0 {
                        RenderContext::Sim
                    } else {
                        RenderContext::Fsm
                    };

                    if !render_gui(&sys_state.gui, ctx, &sim_info) {
                        break 'outer;
                    }
                }
            }
            None => {
                sim_info.timestamp = Timestamp::from_utc(telem.timestamp);
                let env = SystemEnvironment {
                    sim_info: &sim_info,
                    telemetry: &telem,
                };
                system.init(&env, &mut sys_state);

                if !render_gui(&sys_state.gui, RenderContext::Sim, &sim_info) {
                    break 'outer;
                }
            }
        }
        last_t = Some(new_t);

        // Reply with ACK
        if let DatSource::TcpStream(s) = msg_reader.get_mut() {
            s.write_all(ACK_MSG.as_bytes())?;
        }
    }

    println!("Stopped at\n{sim_info:#?}");

    Ok(())
}

/// Returns false if the window should be closed
fn render_gui(gui: &Option<SharedGuiState>, ctx: RenderContext, sim_info: &SimulationInfo) -> bool {
    gui.as_ref()
        .map(|gui| gui.borrow_mut().render(ctx, sim_info))
        .unwrap_or(true)
}

enum DatSource {
    File(File),
    TcpStream(TcpStream),
}

impl io::Read for DatSource {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            DatSource::File(f) => io::Read::read(f, buf),
            DatSource::TcpStream(s) => io::Read::read(s, buf),
        }
    }
}

impl io::Write for DatSource {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            DatSource::File(f) => io::Write::write(f, buf),
            DatSource::TcpStream(s) => io::Write::write(s, buf),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match self {
            DatSource::File(f) => io::Write::flush(f),
            DatSource::TcpStream(s) => io::Write::flush(s),
        }
    }
}

mod interruptor {
    use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
    use std::sync::Arc;

    #[derive(Clone, Debug)]
    #[repr(transparent)]
    pub struct Interruptor(Arc<AtomicBool>);

    impl Interruptor {
        pub fn new() -> Self {
            Interruptor(Arc::new(AtomicBool::new(false)))
        }

        pub fn set(&self) {
            self.0.store(true, SeqCst);
        }

        pub fn is_set(&self) -> bool {
            self.0.load(SeqCst)
        }
    }

    impl Default for Interruptor {
        fn default() -> Self {
            Self::new()
        }
    }
}
