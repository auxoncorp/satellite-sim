use clap::Parser;
use protocol42::{parse_telemetry, ParseErrorExt, EOF_TOKEN};
use std::fs::File;
use std::io::{prelude::*, BufReader};
use std::path::PathBuf;

/// Print CSS data over time
#[derive(Parser, Debug)]
#[command(version)]
struct Opts {
    /// Spacecraft index
    #[arg(short = 's', long)]
    spacecraft: u64,

    /// CSS index
    #[arg(short = 'c', long)]
    sensor: u64,

    /// Use absolute UTC timestamp seconds instead of relative
    #[arg(short = 'a', long)]
    abs_time: bool,

    /// Output file path to write
    #[arg(short = 'o', long)]
    output: PathBuf,

    /// 42 sim log file to read
    input: PathBuf,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let opts = Opts::parse();

    let mut output = File::create(opts.output)?;

    let mut line_buf = String::with_capacity(1024);
    let mut msg_buf = String::with_capacity(4096);
    let input = File::open(opts.input)?;

    let mut msg_reader = BufReader::new(input);

    let mut t0 = None;

    'outer: loop {
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

        let msg_timestamp = telem.timestamp.timestamp();

        let timestamp_sec = if opts.abs_time {
            msg_timestamp
        } else {
            match t0 {
                Some(first_t) => msg_timestamp - first_t,
                None => {
                    t0 = Some(msg_timestamp);
                    0
                }
            }
        };

        let sc = telem
            .spacecrafts
            .get(&opts.spacecraft)
            .expect("Invalid spacecraft index");
        let css = sc.ac.css.get(&opts.sensor).expect("Invalid CSS index");

        writeln!(
            &mut output,
            "{} {} {}",
            timestamp_sec, css.valid as usize, css.illum
        )?;
    }

    Ok(())
}
