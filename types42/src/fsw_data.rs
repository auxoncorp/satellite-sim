use crate::{
    accelerometer::{Accelerometer, AccelerometerIndex},
    css::{CoarseSunSensor, CoarseSunSensorIndex},
    gps::{Gps, GpsIndex},
    gyroscope::{Gyroscope, GyroscopeIndex},
    magnetometer::{Magnetometer, MagnetometerIndex},
};
use std::collections::HashMap;

/// AKA AcType
#[derive(Clone, PartialEq, Debug, Default)]
pub struct FswData {
    pub param_load_enabled: bool,
    pub param_dump_enabled: bool,
    pub gyro: HashMap<GyroscopeIndex, Gyroscope>,
    pub mag: HashMap<MagnetometerIndex, Magnetometer>,
    pub css: HashMap<CoarseSunSensorIndex, CoarseSunSensor>,
    pub gps: HashMap<GpsIndex, Gps>,
    pub accel: HashMap<AccelerometerIndex, Accelerometer>,
}
