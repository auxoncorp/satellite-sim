use std::collections::HashMap;
pub use types42::prelude::*;

#[derive(Clone, Debug, PartialEq)]
pub struct Telemetry {
    pub timestamp: UtcTimestamp,
    pub spacecrafts: HashMap<SpacecraftIndex, Spacecraft>,
    pub orbits: HashMap<OrbitIndex, Orbit>,
    pub worlds: HashMap<WorldKind, World>,
    pub(crate) eof_reached: bool,
}

impl Telemetry {
    pub(crate) fn spacecraft_mut(&mut self, sc: SpacecraftIndex) -> &mut Spacecraft {
        self.spacecrafts.entry(sc).or_insert(Spacecraft::new(sc))
    }

    pub(crate) fn spacecraft_ac_mut(&mut self, sc: SpacecraftIndex) -> &mut FswData {
        &mut self.spacecrafts.entry(sc).or_insert(Spacecraft::new(sc)).ac
    }

    pub(crate) fn spacecraft_ac_gyro_mut(
        &mut self,
        sc: SpacecraftIndex,
        idx: GyroscopeIndex,
    ) -> &mut Gyroscope {
        self.spacecrafts
            .entry(sc)
            .or_insert(Spacecraft::new(sc))
            .ac
            .gyro
            .entry(idx)
            .or_insert(Gyroscope::new(idx, 0.0))
    }

    pub(crate) fn spacecraft_ac_mag_mut(
        &mut self,
        sc: SpacecraftIndex,
        idx: MagnetometerIndex,
    ) -> &mut Magnetometer {
        self.spacecrafts
            .entry(sc)
            .or_insert(Spacecraft::new(sc))
            .ac
            .mag
            .entry(idx)
            .or_insert(Magnetometer::new(idx, 0.0))
    }

    pub(crate) fn spacecraft_ac_css_mut(
        &mut self,
        sc: SpacecraftIndex,
        idx: CoarseSunSensorIndex,
    ) -> &mut CoarseSunSensor {
        self.spacecrafts
            .entry(sc)
            .or_insert(Spacecraft::new(sc))
            .ac
            .css
            .entry(idx)
            .or_insert(CoarseSunSensor::new(idx, false, 0.0))
    }

    pub(crate) fn spacecraft_ac_gps_mut(&mut self, sc: SpacecraftIndex, idx: GpsIndex) -> &mut Gps {
        self.spacecrafts
            .entry(sc)
            .or_insert(Spacecraft::new(sc))
            .ac
            .gps
            .entry(idx)
            .or_insert(Gps::new(idx, false))
    }

    pub(crate) fn spacecraft_ac_accel_mut(
        &mut self,
        sc: SpacecraftIndex,
        idx: AccelerometerIndex,
    ) -> &mut Accelerometer {
        self.spacecrafts
            .entry(sc)
            .or_insert(Spacecraft::new(sc))
            .ac
            .accel
            .entry(idx)
            .or_insert(Accelerometer::new(idx, 0.0))
    }

    pub(crate) fn spacecraft_body_mut(&mut self, sc: SpacecraftIndex, idx: BodyIndex) -> &mut Body {
        self.spacecrafts
            .entry(sc)
            .or_insert(Spacecraft::new(sc))
            .bodies
            .entry(idx)
            .or_insert(Body::new(idx))
    }

    pub(crate) fn orbit_mut(&mut self, orb: OrbitIndex) -> &mut Orbit {
        self.orbits.entry(orb).or_insert(Orbit::new(orb))
    }

    pub(crate) fn world_mut(&mut self, kind: WorldKind) -> &mut World {
        self.worlds.entry(kind).or_insert(World::new(kind))
    }
}
