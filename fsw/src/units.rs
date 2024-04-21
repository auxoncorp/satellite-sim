//! A lightweight uom-ish library. The real thing breaks rust-analyzer.
#![allow(dead_code)]

use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

use modality_api::{AttrVal, Nanoseconds};
use protocol42::telemetry::UtcTimestamp;
use serde::Serialize;

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct Length {
    meters: f64,
}

impl std::fmt::Debug for Length {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} m", self.meters)
    }
}

impl Length {
    pub fn from_meters(meters: f64) -> Length {
        Length { meters }
    }

    pub fn from_kilometers(km: f64) -> Length {
        Length {
            meters: km * 1000.0,
        }
    }

    pub fn as_meters(&self) -> f64 {
        self.meters
    }
}

impl Mul<Length> for Length {
    type Output = Area;

    fn mul(self, rhs: Length) -> Self::Output {
        Area::from_square_meters(self.as_meters() * rhs.as_meters())
    }
}

impl Mul<Length> for f64 {
    type Output = Length;

    fn mul(self, rhs: Length) -> Self::Output {
        Length::from_meters(self * rhs.as_meters())
    }
}

impl Add<Length> for Length {
    type Output = Length;

    fn add(self, rhs: Length) -> Self::Output {
        Length::from_meters(self.as_meters() + rhs.as_meters())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct Area {
    square_meters: f64,
}

impl std::fmt::Debug for Area {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} m²", self.square_meters)
    }
}

impl Area {
    pub fn from_square_meters(square_meters: f64) -> Area {
        Area { square_meters }
    }

    pub fn as_square_meters(&self) -> f64 {
        self.square_meters
    }

    pub fn sqrt(&self) -> Length {
        Length::from_meters(self.as_square_meters().sqrt())
    }
}

impl Add<Area> for Area {
    type Output = Area;

    fn add(self, rhs: Area) -> Self::Output {
        Area::from_square_meters(self.as_square_meters() + rhs.as_square_meters())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Serialize)]
pub struct Ratio {
    ratio: f64,
}

impl std::fmt::Debug for Ratio {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.ratio)
    }
}

impl Ratio {
    pub fn from_f64(ratio: f64) -> Ratio {
        Ratio { ratio }
    }

    pub fn as_f64(&self) -> f64 {
        self.ratio
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Serialize)]
pub struct Timestamp {
    utc: UtcTimestamp,
}

impl std::fmt::Debug for Timestamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.utc)
    }
}

impl std::fmt::Display for Timestamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.utc)
    }
}

impl From<Timestamp> for AttrVal {
    fn from(value: Timestamp) -> Self {
        AttrVal::Timestamp(Nanoseconds::from(value.as_nanos()))
    }
}

impl Timestamp {
    pub fn epoch() -> Timestamp {
        Timestamp::from_utc(UtcTimestamp::default())
    }

    pub fn from_utc(utc: UtcTimestamp) -> Timestamp {
        Timestamp { utc }
    }

    pub fn as_utc(&self) -> &UtcTimestamp {
        &self.utc
    }

    pub fn as_millis(&self) -> i64 {
        self.utc.timestamp_millis()
    }

    pub fn as_nanos(&self) -> u64 {
        self.utc.timestamp_nanos_opt().unwrap() as u64
    }
}

impl Sub<Timestamp> for Timestamp {
    type Output = Time;

    fn sub(self, rhs: Timestamp) -> Self::Output {
        Time::from_chrono_duration(*self.as_utc() - *rhs.as_utc())
    }
}

impl Add<Time> for Timestamp {
    type Output = Timestamp;

    fn add(self, rhs: Time) -> Self::Output {
        let mut ts = self;
        ts += rhs;
        ts
    }
}

impl AddAssign<Time> for Timestamp {
    fn add_assign(&mut self, rhs: Time) {
        self.utc += chrono::Duration::nanoseconds(rhs.as_nanos());
    }
}

impl Sub<Time> for Timestamp {
    type Output = Timestamp;

    fn sub(self, rhs: Time) -> Self::Output {
        let mut ts = self;
        ts -= rhs;
        ts
    }
}

impl SubAssign<Time> for Timestamp {
    fn sub_assign(&mut self, rhs: Time) {
        self.utc -= chrono::Duration::nanoseconds(rhs.as_nanos());
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct Time {
    seconds: f64,
}

impl std::fmt::Debug for Time {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} s", self.seconds)
    }
}

impl Time {
    pub fn from_chrono_duration(duration: chrono::Duration) -> Time {
        Time {
            seconds: duration.num_nanoseconds().unwrap() as f64 / 1_000_000_000.0,
        }
    }

    pub fn from_hours(hours: f64) -> Time {
        Self::from_minutes(hours * 60.0)
    }

    pub fn from_minutes(minutes: f64) -> Time {
        Self::from_secs(minutes * 60.0)
    }

    pub fn from_secs(seconds: f64) -> Time {
        Time { seconds }
    }

    pub fn from_millis(millis: f64) -> Time {
        Time {
            seconds: millis / 1000.0,
        }
    }

    pub fn from_nanos(nanos: i64) -> Time {
        Time {
            seconds: (nanos as f64) * 1_000_000_000.0,
        }
    }

    pub fn as_secs(&self) -> f64 {
        self.seconds
    }

    pub fn as_millis(&self) -> f64 {
        self.seconds * 1_000.0
    }

    pub fn as_nanos(&self) -> i64 {
        (self.seconds * 1_000_000_000.0) as i64
    }

    pub fn abs(&self) -> Time {
        Time {
            seconds: self.seconds.abs(),
        }
    }
}

impl Add<Time> for Time {
    type Output = Time;

    fn add(self, rhs: Time) -> Self::Output {
        Time::from_secs(self.as_secs() + rhs.as_secs())
    }
}

impl AddAssign<Time> for Time {
    fn add_assign(&mut self, rhs: Time) {
        self.seconds += rhs.as_secs()
    }
}

impl Sub<Time> for Time {
    type Output = Time;

    fn sub(self, rhs: Time) -> Self::Output {
        Time::from_secs(self.as_secs() - rhs.as_secs())
    }
}

impl Div<Time> for Time {
    type Output = Ratio;

    fn div(self, rhs: Time) -> Self::Output {
        Ratio::from_f64(self.as_secs() / rhs.as_secs())
    }
}

impl Div<usize> for Time {
    type Output = Time;

    fn div(self, rhs: usize) -> Self::Output {
        Time::from_secs(self.as_secs() / (rhs as f64))
    }
}

impl Div<f64> for Time {
    type Output = Time;

    fn div(self, rhs: f64) -> Self::Output {
        Time::from_secs(self.as_secs() / rhs)
    }
}

impl Mul<Ratio> for Time {
    type Output = Time;

    fn mul(self, rhs: Ratio) -> Self::Output {
        Time::from_secs(self.as_secs() * rhs.as_f64())
    }
}

#[derive(Copy, Clone)]
pub struct Angle {
    degrees: f64,
}

impl std::fmt::Debug for Angle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}°", self.degrees)
    }
}

impl Angle {
    pub fn from_degrees(degrees: f64) -> Angle {
        Angle { degrees }
    }

    pub fn from_radians(radians: f64) -> Angle {
        Angle {
            degrees: radians.to_degrees(),
        }
    }

    pub fn as_degrees(&self) -> f64 {
        self.degrees
    }

    pub fn as_radians(&self) -> f64 {
        self.degrees.to_radians()
    }
}

impl Add<Angle> for Angle {
    type Output = Angle;

    fn add(self, rhs: Angle) -> Self::Output {
        Angle::from_degrees(self.as_degrees() + rhs.as_degrees())
    }
}

impl Sub<Angle> for Angle {
    type Output = Angle;

    fn sub(self, rhs: Angle) -> Self::Output {
        Angle::from_degrees(self.as_degrees() - rhs.as_degrees())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct ElectricPotential {
    volts: f64,
}

impl std::fmt::Debug for ElectricPotential {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} V", self.volts)
    }
}

impl ElectricPotential {
    pub fn from_volts(volts: f64) -> ElectricPotential {
        ElectricPotential { volts }
    }

    pub fn as_volts(&self) -> f64 {
        self.volts
    }
}

impl Div<ElectricCharge> for ElectricPotential {
    type Output = PotentialOverCharge;

    fn div(self, rhs: ElectricCharge) -> Self::Output {
        PotentialOverCharge::from_volts_per_coulomb(self.as_volts() / rhs.as_coulombs())
    }
}

impl Sub<ElectricPotential> for ElectricPotential {
    type Output = ElectricPotential;

    fn sub(self, rhs: ElectricPotential) -> Self::Output {
        ElectricPotential::from_volts(self.as_volts() - rhs.as_volts())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Serialize)]
pub struct ElectricCharge {
    coulombs: f64,
}

impl std::fmt::Debug for ElectricCharge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} C", self.coulombs)
    }
}

impl ElectricCharge {
    pub fn from_coulombs(coulombs: f64) -> ElectricCharge {
        ElectricCharge { coulombs }
    }

    pub fn from_amp_hours(amp_hours: f64) -> ElectricCharge {
        ElectricCharge {
            coulombs: amp_hours * 3600.0,
        }
    }

    pub fn as_coulombs(&self) -> f64 {
        self.coulombs
    }

    pub fn as_amp_hours(&self) -> f64 {
        self.coulombs / 3600.0
    }
}

impl Sub<ElectricCharge> for ElectricCharge {
    type Output = ElectricCharge;

    fn sub(self, rhs: ElectricCharge) -> Self::Output {
        ElectricCharge::from_coulombs(self.as_coulombs() - rhs.as_coulombs())
    }
}

impl AddAssign<ElectricCharge> for ElectricCharge {
    fn add_assign(&mut self, rhs: ElectricCharge) {
        self.coulombs += rhs.as_coulombs()
    }
}

impl SubAssign<ElectricCharge> for ElectricCharge {
    fn sub_assign(&mut self, rhs: ElectricCharge) {
        self.coulombs -= rhs.as_coulombs()
    }
}

impl Div<ElectricCharge> for ElectricCharge {
    type Output = Ratio;

    fn div(self, rhs: ElectricCharge) -> Self::Output {
        Ratio::from_f64(self.as_coulombs() / rhs.as_coulombs())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct ElectricCurrent {
    amps: f64,
}

impl std::fmt::Debug for ElectricCurrent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} A", self.amps)
    }
}

impl ElectricCurrent {
    pub fn from_amps(amps: f64) -> ElectricCurrent {
        ElectricCurrent { amps }
    }

    pub fn from_milliamps(milliamps: f64) -> ElectricCurrent {
        ElectricCurrent {
            amps: milliamps / 1000.0,
        }
    }

    pub fn as_amps(&self) -> f64 {
        self.amps
    }
}

impl Mul<ElectricCurrent> for f64 {
    type Output = ElectricCurrent;

    fn mul(self, rhs: ElectricCurrent) -> Self::Output {
        ElectricCurrent::from_amps(rhs.as_amps() * self)
    }
}

impl Mul<f64> for ElectricCurrent {
    type Output = ElectricCurrent;

    fn mul(self, rhs: f64) -> Self::Output {
        ElectricCurrent::from_amps(self.as_amps() * rhs)
    }
}

impl Mul<Ratio> for ElectricCurrent {
    type Output = ElectricCurrent;

    fn mul(self, rhs: Ratio) -> Self::Output {
        ElectricCurrent::from_amps(self.as_amps() * rhs.as_f64())
    }
}

impl Mul<Time> for ElectricCurrent {
    type Output = ElectricCharge;

    fn mul(self, rhs: Time) -> Self::Output {
        ElectricCharge::from_coulombs(self.amps * rhs.as_secs())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct PotentialOverCharge {
    volts_per_coulomb: f64,
}

impl std::fmt::Debug for PotentialOverCharge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} V/C", self.volts_per_coulomb)
    }
}

impl PotentialOverCharge {
    pub fn from_volts_per_coulomb(volts_per_coulomb: f64) -> PotentialOverCharge {
        PotentialOverCharge { volts_per_coulomb }
    }

    pub fn as_volts_per_coulomb(&self) -> f64 {
        self.volts_per_coulomb
    }
}

impl Mul<ElectricCharge> for PotentialOverCharge {
    type Output = ElectricPotential;

    fn mul(self, rhs: ElectricCharge) -> Self::Output {
        ElectricPotential::from_volts(self.as_volts_per_coulomb() * rhs.as_coulombs())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct Velocity {
    meters_per_second: f64,
}

impl std::fmt::Debug for Velocity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} m·s⁻¹", self.meters_per_second)
    }
}

impl Velocity {
    pub const fn from_meters_per_second(meters_per_second: f64) -> Velocity {
        Velocity { meters_per_second }
    }

    pub fn as_meters_per_second(&self) -> f64 {
        self.meters_per_second
    }

    pub fn abs(&self) -> Self {
        Velocity::from_meters_per_second(self.as_meters_per_second().abs())
    }
}

impl Add<Velocity> for Velocity {
    type Output = Velocity;

    fn add(self, rhs: Velocity) -> Self::Output {
        Velocity::from_meters_per_second(self.as_meters_per_second() + rhs.as_meters_per_second())
    }
}

impl Sub<Velocity> for Velocity {
    type Output = Velocity;

    fn sub(self, rhs: Velocity) -> Self::Output {
        Velocity::from_meters_per_second(self.as_meters_per_second() - rhs.as_meters_per_second())
    }
}

impl Div<Velocity> for Velocity {
    type Output = Ratio;

    fn div(self, rhs: Velocity) -> Self::Output {
        Ratio::from_f64(self.as_meters_per_second() / rhs.as_meters_per_second())
    }
}

impl Mul<Time> for Velocity {
    type Output = Length;

    fn mul(self, rhs: Time) -> Self::Output {
        Length::from_meters(self.as_meters_per_second() * rhs.as_secs())
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Serialize)]
pub struct LuminousIntensity {
    candelas: f64,
}

impl std::fmt::Debug for LuminousIntensity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} cd", self.candelas)
    }
}

impl LuminousIntensity {
    pub fn from_candelas(candelas: f64) -> LuminousIntensity {
        LuminousIntensity { candelas }
    }

    pub fn as_candelas(&self) -> f64 {
        self.candelas
    }

    pub fn total_cmp(&self, other: &LuminousIntensity) -> std::cmp::Ordering {
        self.as_candelas().total_cmp(&other.as_candelas())
    }

    pub fn abs(&self) -> LuminousIntensity {
        LuminousIntensity::from_candelas(self.as_candelas().abs())
    }
}

impl From<f64> for LuminousIntensity {
    fn from(value: f64) -> Self {
        LuminousIntensity::from_candelas(value)
    }
}

impl From<LuminousIntensity> for f64 {
    fn from(value: LuminousIntensity) -> Self {
        value.as_candelas()
    }
}

impl Add<LuminousIntensity> for LuminousIntensity {
    type Output = LuminousIntensity;

    fn add(self, rhs: LuminousIntensity) -> Self::Output {
        LuminousIntensity::from_candelas(self.as_candelas() + rhs.as_candelas())
    }
}

impl Sub<LuminousIntensity> for LuminousIntensity {
    type Output = LuminousIntensity;

    fn sub(self, rhs: LuminousIntensity) -> Self::Output {
        LuminousIntensity::from_candelas(self.as_candelas() - rhs.as_candelas())
    }
}

impl Div<LuminousIntensity> for LuminousIntensity {
    type Output = Ratio;

    fn div(self, rhs: LuminousIntensity) -> Self::Output {
        Ratio::from_f64(self.as_candelas() / rhs.as_candelas())
    }
}

#[derive(Copy, Clone, PartialEq, Serialize)]
pub struct AngularVelocity {
    radians_per_second: f64,
}

impl std::fmt::Debug for AngularVelocity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} °·s⁻¹", self.as_degrees_per_second())
    }
}

impl AngularVelocity {
    pub fn from_degrees_per_second(degrees_per_second: f64) -> AngularVelocity {
        AngularVelocity {
            radians_per_second: degrees_per_second.to_radians(),
        }
    }

    pub fn from_radians_per_second(radians_per_second: f64) -> AngularVelocity {
        AngularVelocity { radians_per_second }
    }

    pub fn as_degrees_per_second(&self) -> f64 {
        self.radians_per_second.to_degrees()
    }

    pub fn as_radians_per_second(&self) -> f64 {
        self.radians_per_second
    }
}

impl Mul<Time> for AngularVelocity {
    type Output = Angle;

    fn mul(self, rhs: Time) -> Self::Output {
        Angle::from_radians(self.as_radians_per_second() * rhs.as_secs())
    }
}

#[derive(Copy, Clone, PartialEq, Serialize)]
pub struct Acceleration {
    meters_per_second_squared: f64,
}

impl std::fmt::Debug for Acceleration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} m·s⁻²", self.meters_per_second_squared)
    }
}

impl Acceleration {
    pub fn from_meters_per_second_squared(meters_per_second_squared: f64) -> Acceleration {
        Acceleration {
            meters_per_second_squared,
        }
    }

    pub fn as_meters_per_second_squared(&self) -> f64 {
        self.meters_per_second_squared
    }
}

#[derive(Copy, Clone, PartialEq, Serialize)]
pub struct MagneticFluxDensity {
    teslas: f64,
}

impl std::fmt::Debug for MagneticFluxDensity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} kg·s⁻²·A⁻¹", self.teslas)
    }
}

impl MagneticFluxDensity {
    pub fn from_teslas(teslas: f64) -> MagneticFluxDensity {
        MagneticFluxDensity { teslas }
    }

    pub fn as_teslas(&self) -> f64 {
        self.teslas
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Serialize)]
pub struct Temperature {
    degrees_celsius: f64,
}

impl std::fmt::Debug for Temperature {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} °C", self.degrees_celsius)
    }
}

impl Temperature {
    pub fn from_degrees_celsius(degrees_celsius: f64) -> Self {
        Self { degrees_celsius }
    }

    pub fn as_degrees_celsius(&self) -> f64 {
        self.degrees_celsius
    }

    pub fn clamp(self, min: Temperature, max: Temperature) -> Self {
        Self::from_degrees_celsius(
            self.as_degrees_celsius()
                .clamp(min.as_degrees_celsius(), max.as_degrees_celsius()),
        )
    }
}

impl Add<TemperatureInterval> for Temperature {
    type Output = Temperature;

    fn add(self, rhs: TemperatureInterval) -> Self::Output {
        Temperature::from_degrees_celsius(self.as_degrees_celsius() + rhs.as_degrees_celsius())
    }
}

impl Sub<Temperature> for Temperature {
    type Output = TemperatureInterval;

    fn sub(self, rhs: Temperature) -> Self::Output {
        TemperatureInterval::from_degrees_celsius(
            self.as_degrees_celsius() - rhs.as_degrees_celsius(),
        )
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct TemperatureInterval {
    degrees_celsius: f64,
}

impl std::fmt::Debug for TemperatureInterval {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} °C", self.degrees_celsius)
    }
}

impl TemperatureInterval {
    pub fn from_degrees_celsius(degrees_celsius: f64) -> Self {
        Self { degrees_celsius }
    }

    pub fn as_degrees_celsius(&self) -> f64 {
        self.degrees_celsius
    }
}

impl Div<Time> for TemperatureInterval {
    type Output = TemperatureIntervalRate;

    fn div(self, rhs: Time) -> Self::Output {
        TemperatureIntervalRate::from_degrees_celsius_per_second(
            self.as_degrees_celsius() / rhs.as_secs(),
        )
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub struct TemperatureIntervalRate {
    degrees_celsius_per_second: f64,
}

impl std::fmt::Debug for TemperatureIntervalRate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} °C·s⁻¹", self.degrees_celsius_per_second)
    }
}

impl TemperatureIntervalRate {
    pub fn from_degrees_celsius_per_second(degrees_celsius_per_second: f64) -> Self {
        Self {
            degrees_celsius_per_second,
        }
    }

    pub fn as_degrees_celsius_per_second(&self) -> f64 {
        self.degrees_celsius_per_second
    }
}

impl Mul<Time> for TemperatureIntervalRate {
    type Output = TemperatureInterval;

    fn mul(self, rhs: Time) -> Self::Output {
        TemperatureInterval::from_degrees_celsius(
            self.as_degrees_celsius_per_second() * rhs.as_secs(),
        )
    }
}
