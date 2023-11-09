//! A very simplistic parser for 42's IPC text-based protocol

use crate::{Telemetry, EOF_TOKEN, FRAME_DELIMETER, SOF_TOKEN, TIMESTAMP_UTC_FORMAT};
use chrono::prelude::*;
use nom::{
    branch::alt,
    bytes::complete::tag,
    character::complete::{line_ending, not_line_ending, space1, u64},
    combinator::{map, opt},
    error::ErrorKind,
    multi::fold_many0,
    number::complete::double,
    sequence::delimited,
    Err::Error,
};
use std::collections::HashSet;
use tracing::debug;
use types42::prelude::*;

pub type Result<I, O, E = ParseError<I>> = std::result::Result<(I, O), nom::Err<E>>;

#[derive(Debug, PartialEq, thiserror::Error)]
pub enum ParseError<I> {
    #[error("Invalid SOF TIME format")]
    Timestamp,
    #[error("Missing EOF")]
    MissingEof,
    #[error("Parse error")]
    Nom(I, ErrorKind),
}

pub trait ParseErrorExt {
    fn is_failure(&self) -> bool;
}

impl<I> ParseErrorExt for nom::Err<ParseError<I>> {
    fn is_failure(&self) -> bool {
        matches!(
            self,
            nom::Err::Error(ParseError::Timestamp) | nom::Err::Failure(_)
        )
    }
}

pub fn parse_unstructured_tle_set(set: &str) -> Result<&str, HashSet<UnstructuredTle>> {
    let (s, tle_set) = fold_many0(
        tle,
        HashSet::new,
        |mut tle_set: HashSet<UnstructuredTle>, tle| {
            tle_set.insert(tle);
            tle_set
        },
    )(set)?;
    Ok((s, tle_set))
}

fn tle(s: &str) -> Result<&str, UnstructuredTle> {
    let (s, name) = not_line_ending(s)?;
    let (s, _) = line_ending(s)?;
    let (s, line1) = not_line_ending(s)?;
    let (s, _) = line_ending(s)?;
    let (s, line2) = not_line_ending(s)?;
    let (s, _) = line_ending(s)?;
    let (s, _) = opt(line_ending)(s)?;
    Ok((
        s,
        UnstructuredTle {
            satellite_name: name.to_string(),
            line1: line1.to_string(),
            line2: line2.to_string(),
        },
    ))
}

pub fn parse_telemetry(msg: &str) -> Result<&str, Telemetry> {
    use IpcTextLine::*;

    // Frame delimiter can be optionally leading or trailing
    let (s, _) = opt(tag(FRAME_DELIMETER))(msg)?;
    let (s, timestamp) = timestamp(s)?;
    let (s, _) = line_ending(s)?;

    let telem_default = || Telemetry {
        timestamp,
        spacecrafts: Default::default(),
        orbits: Default::default(),
        worlds: Default::default(),
        eof_reached: false,
    };

    let (s, telem) = fold_many0(read_line, telem_default, |mut telem, line| {
        match line {
            ScPosR(sc, v) => {
                telem.spacecraft_mut(sc).pos_r = v;
            }
            ScVelR(sc, v) => {
                telem.spacecraft_mut(sc).vel_r = v;
            }
            ScSvb(sc, v) => {
                telem.spacecraft_mut(sc).svb = v;
            }
            ScBvb(sc, v) => {
                telem.spacecraft_mut(sc).bvb = v;
            }
            ScHvb(sc, v) => {
                telem.spacecraft_mut(sc).hvb = v;
            }

            AcParmLoad(sc, v) => {
                telem.spacecraft_ac_mut(sc).param_load_enabled = v;
            }
            AcParmDump(sc, v) => {
                telem.spacecraft_ac_mut(sc).param_dump_enabled = v;
            }

            AcGyroRate(sc, idx, v) => {
                telem.spacecraft_ac_gyro_mut(sc, idx).rate = v;
            }
            AcMagField(sc, idx, v) => {
                telem.spacecraft_ac_mag_mut(sc, idx).field = v;
            }
            AcCssValid(sc, idx, v) => {
                telem.spacecraft_ac_css_mut(sc, idx).valid = v;
            }
            AcCssIllum(sc, idx, v) => {
                telem.spacecraft_ac_css_mut(sc, idx).illum = v;
            }

            AcGpsValid(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).valid = v;
            }
            AcGpsRollover(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).rollover = v;
            }
            AcGpsWeek(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).week = v;
            }
            AcGpsSec(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).sec = v;
            }
            AcGpsPosN(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).pos_n = v;
            }
            AcGpsVelN(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).vel_n = v;
            }
            AcGpsPosW(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).pos_w = v;
            }
            AcGpsVelW(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).vel_w = v;
            }
            AcGpsLong(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).longitude = v;
            }
            AcGpsLat(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).latitude = v;
            }
            AcGpsAlt(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).altitude = v;
            }
            AcGpsWgsLong(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).wgs_longitude = v;
            }
            AcGpsWgsLat(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).wgs_latitude = v;
            }
            AcGpsWgsAlt(sc, idx, v) => {
                telem.spacecraft_ac_gps_mut(sc, idx).wgs_altitude = v;
            }
            AcAccelAcc(sc, idx, v) => {
                telem.spacecraft_ac_accel_mut(sc, idx).acc = v;
            }

            ScBodyWn(sc, idx, v) => {
                telem.spacecraft_body_mut(sc, idx).wn = v;
            }
            ScBodyQn(sc, idx, v) => {
                telem.spacecraft_body_mut(sc, idx).qn = v;
            }

            ScGnPos(sc, v) => {
                telem.spacecraft_mut(sc).gn.pos = v;
            }
            ScGnPosRate(sc, v) => {
                telem.spacecraft_mut(sc).gn.pos_rate = v;
            }
            ScGnAng(sc, v) => {
                telem.spacecraft_mut(sc).gn.ang = v;
            }
            ScGnAngRate(sc, v) => {
                telem.spacecraft_mut(sc).gn.ang_rate = v;
            }

            OrbPosN(orb, v) => {
                telem.orbit_mut(orb).central.pos_n = v;
            }
            OrbVelN(orb, v) => {
                telem.orbit_mut(orb).central.vel_n = v;
            }

            WorldPosH(kind, v) => {
                telem.world_mut(kind).pos_h = v;
            }
            WorldEphPosN(kind, v) => {
                telem.world_mut(kind).eph.pos_n = v;
            }
            WorldEphVelN(kind, v) => {
                telem.world_mut(kind).eph.vel_n = v;
            }

            Eof => {
                telem.eof_reached = true;
            }

            Unsupported(line) => {
                debug!(line = line, "Unsupported 42 protocol line");
            }
        }
        telem
    })(s)?;

    if telem.eof_reached {
        let (s, _) = opt(tag(FRAME_DELIMETER))(s)?;
        Ok((s, telem))
    } else {
        Err(Error(ParseError::MissingEof))
    }
}

type Vector3 = na::Vector3<f64>;
type Quaternion = na::Quaternion<f64>;

/// A line in the IPC text-based protocol
#[derive(Clone, Debug)]
enum IpcTextLine<'a> {
    // Excludes the SOF token (UtcTimestamp)

    // Spacecraft/SCType
    ScPosR(SpacecraftIndex, Vector3),
    ScVelR(SpacecraftIndex, Vector3),
    ScSvb(SpacecraftIndex, Vector3),
    ScBvb(SpacecraftIndex, Vector3),
    ScHvb(SpacecraftIndex, Vector3),

    // FswData/AcType
    AcParmLoad(SpacecraftIndex, bool),
    AcParmDump(SpacecraftIndex, bool),
    AcGyroRate(SpacecraftIndex, GyroscopeIndex, f64),
    AcMagField(SpacecraftIndex, MagnetometerIndex, f64),
    AcCssValid(SpacecraftIndex, CoarseSunSensorIndex, bool),
    AcCssIllum(SpacecraftIndex, CoarseSunSensorIndex, f64),
    AcGpsValid(SpacecraftIndex, GpsIndex, bool),
    AcGpsRollover(SpacecraftIndex, GpsIndex, u64),
    AcGpsWeek(SpacecraftIndex, GpsIndex, u64),
    AcGpsSec(SpacecraftIndex, GpsIndex, f64),
    AcGpsPosN(SpacecraftIndex, GpsIndex, Vector3),
    AcGpsVelN(SpacecraftIndex, GpsIndex, Vector3),
    AcGpsPosW(SpacecraftIndex, GpsIndex, Vector3),
    AcGpsVelW(SpacecraftIndex, GpsIndex, Vector3),
    AcGpsLong(SpacecraftIndex, GpsIndex, f64),
    AcGpsLat(SpacecraftIndex, GpsIndex, f64),
    AcGpsAlt(SpacecraftIndex, GpsIndex, f64),
    AcGpsWgsLong(SpacecraftIndex, GpsIndex, f64),
    AcGpsWgsLat(SpacecraftIndex, GpsIndex, f64),
    AcGpsWgsAlt(SpacecraftIndex, GpsIndex, f64),
    AcAccelAcc(SpacecraftIndex, AccelerometerIndex, f64),

    // Body
    ScBodyWn(SpacecraftIndex, BodyIndex, Vector3),
    ScBodyQn(SpacecraftIndex, BodyIndex, Quaternion),

    // Joint
    ScGnPos(SpacecraftIndex, Vector3),
    ScGnPosRate(SpacecraftIndex, Vector3),
    ScGnAng(SpacecraftIndex, Vector3),
    ScGnAngRate(SpacecraftIndex, Vector3),

    // Orbit
    OrbPosN(OrbitIndex, Vector3),
    OrbVelN(OrbitIndex, Vector3),

    // World
    WorldPosH(WorldKind, Vector3),
    WorldEphPosN(WorldKind, Vector3),
    WorldEphVelN(WorldKind, Vector3),

    Eof,

    Unsupported(&'a str),
}

fn read_line(s: &str) -> Result<&str, IpcTextLine> {
    let (s, line) = alt((
        read_sc_prologue_state,
        map(ac_parm_load_en, |(sc_idx, v)| {
            IpcTextLine::AcParmLoad(sc_idx, v)
        }),
        map(ac_parm_dump_en, |(sc_idx, v)| {
            IpcTextLine::AcParmDump(sc_idx, v)
        }),
        map(ac_gyro_rate, |(sc_idx, idx, v)| {
            IpcTextLine::AcGyroRate(sc_idx, idx, v)
        }),
        map(ac_magf_field, |(sc_idx, idx, v)| {
            IpcTextLine::AcMagField(sc_idx, idx, v)
        }),
        map(ac_css_valid, |(sc_idx, idx, v)| {
            IpcTextLine::AcCssValid(sc_idx, idx, v)
        }),
        map(ac_css_illum, |(sc_idx, idx, v)| {
            IpcTextLine::AcCssIllum(sc_idx, idx, v)
        }),
        read_ac_gps_line,
        map(ac_accel_acc, |(sc_idx, idx, v)| {
            IpcTextLine::AcAccelAcc(sc_idx, idx, v)
        }),
        map(sc_body_wn, |(sc_idx, idx, v)| {
            IpcTextLine::ScBodyWn(sc_idx, idx, v)
        }),
        map(sc_body_qn, |(sc_idx, idx, v)| {
            IpcTextLine::ScBodyQn(sc_idx, idx, v)
        }),
        read_sc_gn,
        map(orb_pos_n, |(orb_idx, v)| IpcTextLine::OrbPosN(orb_idx, v)),
        map(orb_vel_n, |(orb_idx, v)| IpcTextLine::OrbVelN(orb_idx, v)),
        map(world_pos_h, |(kind, v)| IpcTextLine::WorldPosH(kind, v)),
        map(world_eph_pos_n, |(kind, v)| {
            IpcTextLine::WorldEphPosN(kind, v)
        }),
        map(world_eph_vel_n, |(kind, v)| {
            IpcTextLine::WorldEphVelN(kind, v)
        }),
        map(tag(EOF_TOKEN), |_| IpcTextLine::Eof),
        map(not_line_ending, IpcTextLine::Unsupported),
    ))(s)?;
    let (s, _) = line_ending(s)?;
    Ok((s, line))
}

fn read_sc_prologue_state(s: &str) -> Result<&str, IpcTextLine> {
    alt((
        map(sc_pos_r, |(sc_idx, v)| IpcTextLine::ScPosR(sc_idx, v)),
        map(sc_vel_r, |(sc_idx, v)| IpcTextLine::ScVelR(sc_idx, v)),
        map(sc_svb, |(sc_idx, v)| IpcTextLine::ScSvb(sc_idx, v)),
        map(sc_bvb, |(sc_idx, v)| IpcTextLine::ScBvb(sc_idx, v)),
        map(sc_hvb, |(sc_idx, v)| IpcTextLine::ScHvb(sc_idx, v)),
    ))(s)
}

fn read_ac_gps_line(s: &str) -> Result<&str, IpcTextLine> {
    alt((
        map(ac_gps_valid, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsValid(sc_idx, idx, v)
        }),
        map(ac_gps_rollover, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsRollover(sc_idx, idx, v)
        }),
        map(ac_gps_week, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsWeek(sc_idx, idx, v)
        }),
        map(ac_gps_sec, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsSec(sc_idx, idx, v)
        }),
        map(ac_gps_pos_n, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsPosN(sc_idx, idx, v)
        }),
        map(ac_gps_vel_n, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsVelN(sc_idx, idx, v)
        }),
        map(ac_gps_pos_w, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsPosW(sc_idx, idx, v)
        }),
        map(ac_gps_vel_w, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsVelW(sc_idx, idx, v)
        }),
        map(ac_gps_long, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsLong(sc_idx, idx, v)
        }),
        map(ac_gps_lat, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsLat(sc_idx, idx, v)
        }),
        map(ac_gps_alt, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsAlt(sc_idx, idx, v)
        }),
        map(ac_gps_wgs_long, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsWgsLong(sc_idx, idx, v)
        }),
        map(ac_gps_wgs_lat, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsWgsLat(sc_idx, idx, v)
        }),
        map(ac_gps_wgs_alt, |(sc_idx, idx, v)| {
            IpcTextLine::AcGpsWgsAlt(sc_idx, idx, v)
        }),
    ))(s)
}

fn read_sc_gn(s: &str) -> Result<&str, IpcTextLine> {
    alt((
        map(sc_gn_pos, |(sc_idx, v)| IpcTextLine::ScGnPos(sc_idx, v)),
        map(sc_gn_pos_rate, |(sc_idx, v)| {
            IpcTextLine::ScGnPosRate(sc_idx, v)
        }),
        map(sc_gn_ang, |(sc_idx, v)| IpcTextLine::ScGnAng(sc_idx, v)),
        map(sc_gn_ang_rate, |(sc_idx, v)| {
            IpcTextLine::ScGnAngRate(sc_idx, v)
        }),
    ))(s)
}

fn timestamp(s: &str) -> Result<&str, UtcTimestamp> {
    let (s, _) = tag(SOF_TOKEN)(s)?;
    let (s, _) = space1(s)?;
    let (s, datetime) = not_line_ending(s)?;
    let timestamp = NaiveDateTime::parse_from_str(datetime, TIMESTAMP_UTC_FORMAT)
        .map_err(|_| Error(ParseError::Timestamp))?;
    Ok((s, timestamp.and_utc()))
}

fn ac_parm_load_en(s: &str) -> Result<&str, (SpacecraftIndex, bool)> {
    let (s, sc_idx) = sc(s)?;
    let (s, _) = tag(".AC.ParmLoadEnabled")(s)?;
    let (s, _) = eq(s)?;
    let (s, val) = intbool(s)?;
    Ok((s, (sc_idx, val)))
}

fn ac_parm_dump_en(s: &str) -> Result<&str, (SpacecraftIndex, bool)> {
    let (s, sc_idx) = sc(s)?;
    let (s, _) = tag(".AC.ParmDumpEnabled")(s)?;
    let (s, _) = eq(s)?;
    let (s, val) = intbool(s)?;
    Ok((s, (sc_idx, val)))
}

fn world(s: &str) -> Result<&str, OrbitIndex> {
    let (s, _) = tag("World")(s)?;
    index(s)
}

fn orb(s: &str) -> Result<&str, OrbitIndex> {
    let (s, _) = tag("Orb")(s)?;
    index(s)
}

fn sc(s: &str) -> Result<&str, SpacecraftIndex> {
    let (s, _) = tag("SC")(s)?;
    index(s)
}

fn index(s: &str) -> Result<&str, u64> {
    let (s, idx) = delimited(tag("["), u64, tag("]"))(s)?;
    Ok((s, idx))
}

fn eq(s: &str) -> Result<&str, &str> {
    tag(" = ")(s)
}

fn intbool(s: &str) -> Result<&str, bool> {
    let (s, int) = u64(s)?;
    Ok((s, int != 0))
}

fn v3(s: &str) -> Result<&str, Vector3> {
    let (s, v0) = double(s)?;
    let (s, _) = space1(s)?;
    let (s, v1) = double(s)?;
    let (s, _) = space1(s)?;
    let (s, v2) = double(s)?;
    Ok((s, Vector3::new(v0, v1, v2)))
}

fn quat(s: &str) -> Result<&str, Quaternion> {
    let (s, v0) = double(s)?;
    let (s, _) = space1(s)?;
    let (s, v1) = double(s)?;
    let (s, _) = space1(s)?;
    let (s, v2) = double(s)?;
    let (s, _) = space1(s)?;
    let (s, v3) = double(s)?;
    Ok((s, Quaternion::new(v0, v1, v2, v3)))
}

macro_rules! create_sc_v3 {
    ($func_name:ident, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (SpacecraftIndex, Vector3)> {
            let (s, sc_idx) = sc(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = v3(s)?;
            Ok((s, (sc_idx, val)))
        }
    };
}

macro_rules! create_sc_sub_arr_intbool {
    ($func_name:ident, $arr:literal, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (SpacecraftIndex, u64, bool)> {
            let (s, sc_idx) = sc(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($arr)(s)?;
            let (s, idx) = index(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = intbool(s)?;
            Ok((s, (sc_idx, idx, val)))
        }
    };
}

macro_rules! create_sc_sub_arr_f64 {
    ($func_name:ident, $arr:literal, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (SpacecraftIndex, u64, f64)> {
            let (s, sc_idx) = sc(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($arr)(s)?;
            let (s, idx) = index(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = double(s)?;
            Ok((s, (sc_idx, idx, val)))
        }
    };
}

macro_rules! create_sc_sub_arr_u64 {
    ($func_name:ident, $arr:literal, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (SpacecraftIndex, u64, u64)> {
            let (s, sc_idx) = sc(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($arr)(s)?;
            let (s, idx) = index(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = u64(s)?;
            Ok((s, (sc_idx, idx, val)))
        }
    };
}

macro_rules! create_sc_sub_arr_v3 {
    ($func_name:ident, $arr:literal, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (SpacecraftIndex, u64, Vector3)> {
            let (s, sc_idx) = sc(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($arr)(s)?;
            let (s, idx) = index(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = v3(s)?;
            Ok((s, (sc_idx, idx, val)))
        }
    };
}

macro_rules! create_sc_sub_arr_q {
    ($func_name:ident, $arr:literal, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (SpacecraftIndex, u64, Quaternion)> {
            let (s, sc_idx) = sc(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($arr)(s)?;
            let (s, idx) = index(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = quat(s)?;
            Ok((s, (sc_idx, idx, val)))
        }
    };
}

macro_rules! create_orb_v3 {
    ($func_name:ident, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (OrbitIndex, Vector3)> {
            let (s, idx) = orb(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = v3(s)?;
            Ok((s, (idx, val)))
        }
    };
}

macro_rules! create_world_v3 {
    ($func_name:ident, $field:literal) => {
        fn $func_name(s: &str) -> Result<&str, (WorldKind, Vector3)> {
            let (s, kind) = world(s)?;
            let (s, _) = tag(".")(s)?;
            let (s, _) = tag($field)(s)?;
            let (s, _) = eq(s)?;
            let (s, val) = v3(s)?;
            Ok((s, (WorldKind::from(kind), val)))
        }
    };
}

create_sc_v3!(sc_pos_r, "PosR");
create_sc_v3!(sc_vel_r, "VelR");
create_sc_v3!(sc_svb, "svb");
create_sc_v3!(sc_bvb, "bvb");
create_sc_v3!(sc_hvb, "Hvb");
create_sc_sub_arr_v3!(sc_body_wn, "B", "wn");
create_sc_sub_arr_q!(sc_body_qn, "B", "qn");
create_sc_v3!(sc_gn_pos, "GN.Pos");
create_sc_v3!(sc_gn_pos_rate, "GN.PosRate");
create_sc_v3!(sc_gn_ang, "GN.Ang");
create_sc_v3!(sc_gn_ang_rate, "GN.AngRate");

create_sc_sub_arr_intbool!(ac_css_valid, "AC.CSS", "Valid");
create_sc_sub_arr_f64!(ac_css_illum, "AC.CSS", "Illum");

create_sc_sub_arr_f64!(ac_gyro_rate, "AC.Gyro", "Rate");
create_sc_sub_arr_f64!(ac_magf_field, "AC.MAG", "Field");
create_sc_sub_arr_f64!(ac_accel_acc, "AC.Accel", "Acc");

create_sc_sub_arr_intbool!(ac_gps_valid, "AC.GPS", "Valid");
create_sc_sub_arr_u64!(ac_gps_rollover, "AC.GPS", "Rollover");
create_sc_sub_arr_u64!(ac_gps_week, "AC.GPS", "Week");
create_sc_sub_arr_f64!(ac_gps_sec, "AC.GPS", "Sec");
create_sc_sub_arr_v3!(ac_gps_pos_n, "AC.GPS", "PosN");
create_sc_sub_arr_v3!(ac_gps_vel_n, "AC.GPS", "VelN");
create_sc_sub_arr_v3!(ac_gps_pos_w, "AC.GPS", "PosW");
create_sc_sub_arr_v3!(ac_gps_vel_w, "AC.GPS", "VelW");
create_sc_sub_arr_f64!(ac_gps_long, "AC.GPS", "Lng");
create_sc_sub_arr_f64!(ac_gps_lat, "AC.GPS", "Lat");
create_sc_sub_arr_f64!(ac_gps_alt, "AC.GPS", "Alt");
create_sc_sub_arr_f64!(ac_gps_wgs_long, "AC.GPS", "WgsLng");
create_sc_sub_arr_f64!(ac_gps_wgs_lat, "AC.GPS", "WgsLat");
create_sc_sub_arr_f64!(ac_gps_wgs_alt, "AC.GPS", "WgsAlt");

create_orb_v3!(orb_pos_n, "PosN");
create_orb_v3!(orb_vel_n, "VelN");

create_world_v3!(world_pos_h, "PosH");
create_world_v3!(world_eph_pos_n, "eph.PosN");
create_world_v3!(world_eph_vel_n, "eph.VelN");

impl<I> nom::error::ParseError<I> for ParseError<I> {
    fn from_error_kind(s: I, kind: ErrorKind) -> Self {
        ParseError::Nom(s, kind)
    }

    fn append(_: I, _: ErrorKind, other: Self) -> Self {
        other
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use indoc::indoc;

    const TELEM_MSG: &str = indoc! {r#"TIME 2028-192-08:30:45.500000000
        SC[4].PosR = 2.761593233762e-09 -2.052612283404e-09 -9.101141695539e-09
        SC[4].VelR = 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00
        SC[4].svb = -9.332740448479e-01 1.236536344317e-01 -3.372081492271e-01
        SC[4].bvb = -2.101750629716e-05 -2.410215214612e-05 8.171113829394e-06
        SC[4].Hvb = -3.319477714866e+00 1.027565593909e+00 -2.431153296488e+01
        SC[4].AC.ParmLoadEnabled = 0
        SC[4].AC.ParmDumpEnabled = 0
        SC[4].AC.Gyro[0].Rate = -3.311277441978e-02
        SC[4].AC.MAG[0].Field = -2.100000000000e-05
        SC[4].AC.CSS[0].Valid = 1
        SC[4].AC.CSS[0].Illum = -6.610000000000e-01
        SC[4].AC.GPS[0].Valid = 1
        SC[4].AC.GPS[0].Rollover = 2
        SC[4].AC.GPS[0].Week = 483
        SC[4].AC.GPS[0].Sec = 1.171145839959e+05
        SC[4].AC.GPS[0].PosN = -6.772709496839e+06 -1.670934436068e+05 2.138644687279e+05
        SC[4].AC.GPS[0].VelN = 3.070495250338e+02 -4.717413159861e+03 6.038067448434e+03
        SC[4].AC.GPS[0].PosW = -3.915385854507e+06 5.529469304609e+06 1.948716967005e+05
        SC[4].AC.GPS[0].VelW = -3.351078198606e+03 -2.585700621439e+03 6.038868252894e+03
        SC[4].AC.GPS[0].Lng = 2.186934093933e+00
        SC[4].AC.GPS[0].Lat = 2.875396542320e-02
        SC[4].AC.GPS[0].Alt = 4.000001857023e+05
        SC[4].AC.GPS[0].WgsLng = 2.186934093933e+00
        SC[4].AC.GPS[0].WgsLat = 2.893614208344e-02
        SC[4].AC.GPS[0].WgsAlt = 4.000259436487e+05
        SC[4].AC.Accel[0].Acc = 5.000000000000e-01
        SC[4].B[0].wn = -3.319477714866e-02 5.137827969545e-03 -8.103844321627e-02
        SC[4].B[0].qn = -1.448532591491e-01 5.249724712983e-01 -4.282234860125e-01 7.211422076957e-01
        SC[4].GN.Pos = 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00
        SC[4].GN.PosRate = 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00
        SC[4].GN.Ang = 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00
        SC[4].GN.AngRate = 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00
        SC[4].Gyro[0].TrueRate = -3.315780981007e-02
        Orb[0].PosN = -6.772678814706e+06 -1.675587079216e+05 2.144653661054e+05
        Orb[0].VelN = 3.079127031786e+02 -4.717425402638e+03 6.038029169602e+03
        World[3].PosH = 4.856639914829e+10 -1.441222124336e+11 0.000000000000e+00
        World[3].eph.PosN = 4.856639914829e+10 -1.441222124336e+11 0.000000000000e+00
        World[3].eph.VelN = 2.774561571481e+04 9.397259584099e+03 0.000000000000e+00
        World[10].PosH = 4.891197548546e+10 -1.443298361456e+11 2.185098725053e+07
        World[10].eph.PosN = 3.455763371709e+08 -1.991828549520e+08 -6.254008076517e+07
        World[10].eph.VelN = 5.107354322344e+02 7.282302007882e+02 3.958409879577e+02
        [EOF]
        "#};

    const TLE_SET: &str = indoc! {r#"GEO1
        1 37481U 11019A   23190.45078927 -.00000009  00000-0  00000+0 0  9991
        2 37481   2.3847  40.6385 0001640  70.7486  43.7146  1.00272292 44578

        GEO2
        1 39120U 13011A   23190.50177227 -.00000262  00000-0  00000+0 0  9997
        2 39120   2.3950  38.7964 0001772  68.0002 323.0070  1.00271163 37822
        "#};

    #[test]
    fn parse_timestamp() {
        assert_eq!(
            timestamp("TIME 2028-192-08:30:45.500000000"),
            Ok((
                "",
                "2028-07-10 08:30:45.500 UTC"
                    .parse::<UtcTimestamp>()
                    .unwrap()
            ))
        );

        assert_eq!(
            timestamp("TIME ABBC-192-08:30:45.500000000"),
            Err(Error(ParseError::Timestamp))
        );
    }

    #[test]
    fn parse_intbool() {
        assert_eq!(
            ac_gps_valid("SC[4].AC.GPS[0].Valid = 1"),
            Ok(("", (4, 0, true)))
        );
        assert_eq!(
            ac_css_valid("SC[4].AC.CSS[2].Valid = 0"),
            Ok(("", (4, 2, false)))
        );
    }

    #[test]
    fn parse_f64() {
        assert_eq!(
            ac_gyro_rate("SC[0].AC.Gyro[0].Rate = -3.311277441978e-02"),
            Ok(("", (0, 0, -3.311277441978e-02)))
        );
    }

    #[test]
    fn parse_u64() {
        assert_eq!(
            ac_gps_week("SC[4].AC.GPS[1].Week = 483"),
            Ok(("", (4, 1, 483)))
        );
    }

    #[test]
    fn parse_v3() {
        assert_eq!(
            ac_gps_pos_n(
                "SC[4].AC.GPS[2].PosN = -6.772709496839e+06 -1.670934436068e+05 2.138644687279e+05"
            ),
            Ok((
                "",
                (
                    4,
                    2,
                    na::vector![-6.772709496839e+06, -1.670934436068e+05, 2.138644687279e+05],
                )
            ))
        );
    }

    #[test]
    fn parse_quat() {
        assert_eq!(
            sc_body_qn(
                "SC[0].B[0].qn = -1.448532591491e-01 5.249724712983e-01 -4.282234860125e-01 7.211422076957e-01"
            ),
            Ok((
                "",
                (
                    0,
                    0,
                    Quaternion::new(
                    -1.448532591491e-01, 5.249724712983e-01 ,-4.282234860125e-01 ,7.211422076957e-01),
                )
            ))
        );
    }

    #[test]
    fn parse_message() {
        let (s, t) = parse_telemetry(TELEM_MSG).unwrap();
        assert_eq!(s, "");
        assert!(t.eof_reached);
        assert_eq!(t.spacecrafts.len(), 1);
        assert_eq!(t.orbits.len(), 1);
        assert_eq!(t.worlds.len(), 2);

        assert!(t.spacecrafts.get(&4).is_some());
        let sc = t.spacecrafts.get(&4).unwrap();
        assert_eq!(
            sc.pos_r,
            na::vector![2.761593233762e-09, -2.052612283404e-09, -9.101141695539e-09]
        );

        assert!(t.orbits.get(&0).is_some());

        assert!(t.worlds.get(&WorldKind::Earth).is_some());
        assert!(t.worlds.get(&WorldKind::Luna).is_some());
    }

    #[test]
    fn missing_eof() {
        let msg = TELEM_MSG.trim_end().trim_end_matches(EOF_TOKEN);
        assert_eq!(parse_telemetry(msg), Err(Error(ParseError::MissingEof)));
    }

    #[test]
    fn parse_tle() {
        let (s, tle_set) = parse_unstructured_tle_set(TLE_SET).unwrap();
        assert!(s.is_empty());
        assert_eq!(tle_set.len(), 2);
        dbg!(tle_set);
    }
}
