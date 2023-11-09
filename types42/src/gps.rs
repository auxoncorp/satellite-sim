use derive_more::Display;

pub type GpsIndex = u64;

/// AcGpsType
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Display)]
#[display(
    fmt = "{{index: {}, valid: {}, lat: {}, lon: {}, alt: {}, ...}}",
    "index",
    "valid",
    "latitude",
    "longitude",
    "altitude"
)]
pub struct Gps {
    pub index: GpsIndex,

    /// Ignore if false
    pub valid: bool,

    pub rollover: u64,
    pub week: u64,
    pub sec: f64,

    /// Position vector in N frame [m]
    pub pos_n: na::Vector3<f64>,
    /// Velocity vector in N frame [m/s]
    pub vel_n: na::Vector3<f64>,
    /// Position vector in world frame [m]
    pub pos_w: na::Vector3<f64>,
    /// Velocity vector in world frame [m/s]
    pub vel_w: na::Vector3<f64>,

    /// Geocentric coordinates [rad]
    pub longitude: f64,
    /// Geocentric coordinates [rad]
    pub latitude: f64,
    /// Geocentric coordinates [m]
    pub altitude: f64,

    /// Geodetic, WGS-84 coordinates [rad]
    pub wgs_longitude: f64,
    /// Geodetic, WGS-84 coordinates [rad]
    pub wgs_latitude: f64,
    /// Geodetic, WGS-84 coordinates [m]
    pub wgs_altitude: f64,
}

impl Gps {
    pub fn new(index: GpsIndex, valid: bool) -> Self {
        Self {
            index,
            valid,
            rollover: 0,
            week: 0,
            sec: 0.0,
            pos_n: Default::default(),
            vel_n: Default::default(),
            pos_w: Default::default(),
            vel_w: Default::default(),
            longitude: Default::default(),
            latitude: Default::default(),
            altitude: Default::default(),
            wgs_longitude: Default::default(),
            wgs_latitude: Default::default(),
            wgs_altitude: Default::default(),
        }
    }
}
