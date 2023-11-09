extern crate nalgebra as na;

pub use crate::parser::{parse_telemetry, parse_unstructured_tle_set, ParseError, ParseErrorExt};
pub use crate::telemetry::Telemetry;

pub mod parser;
pub mod telemetry;

pub const SOF_TOKEN: &str = "TIME";
pub const EOF_TOKEN: &str = "[EOF]";
pub const FRAME_DELIMETER: &str = "\n";
pub const ACK_MSG: &str = "ACK\n";
pub const TIMESTAMP_UTC_FORMAT: &str = "%Y-%j-%H:%M:%S%.9f";
