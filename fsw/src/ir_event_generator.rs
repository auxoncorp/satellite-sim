use crate::{
    ground_truth_ir_events::ScheduledIREvent,
    units::{Angle, Length, LuminousIntensity, Time, Velocity},
};
use oorandom::Rand64;
use std::ops::RangeInclusive;

#[derive(Debug)]
pub struct IREventGenerator {
    pub prng_seed: u64,
    pub num_events: usize,

    pub initial_start_time: Time,
    pub max_time_between_activations: Time,
    pub max_active_duration: Option<Time>,

    pub latitude_range: RangeInclusive<Angle>,
    pub longitude_range: RangeInclusive<Angle>,
    pub altitude_range: RangeInclusive<Length>,
    pub velocity_east_range: RangeInclusive<Velocity>,
    pub velocity_north_range: RangeInclusive<Velocity>,
    pub intensity_range: RangeInclusive<LuminousIntensity>,
}

impl Default for IREventGenerator {
    fn default() -> Self {
        IREventGenerator {
            prng_seed: 0,
            num_events: 15,
            initial_start_time: Time::from_secs(0.0),
            max_time_between_activations: Time::from_minutes(5.0),
            max_active_duration: Time::from_minutes(60.0).into(),
            latitude_range: Angle::from_degrees(-60.0)..=Angle::from_degrees(60.0),
            longitude_range: Angle::from_degrees(-140.0)..=Angle::from_degrees(140.0),
            altitude_range: Length::from_kilometers(100.0)..=Length::from_kilometers(1500.0),
            velocity_east_range: Velocity::from_meters_per_second(-6000.0)
                ..=Velocity::from_meters_per_second(6000.0),
            velocity_north_range: Velocity::from_meters_per_second(-6000.0)
                ..=Velocity::from_meters_per_second(6000.0),
            intensity_range: LuminousIntensity::from_candelas(1000.0)
                ..=LuminousIntensity::from_candelas(1_000_000.0),
        }
    }
}

impl IREventGenerator {
    pub fn generate(self) -> Vec<ScheduledIREvent> {
        let mut rand = Rand64::new(self.prng_seed.into());
        let mut id = GroundTruthIdGen::default();
        let mut events = Vec::with_capacity(self.num_events);

        let prng = &mut rand;
        let mut last_activation_time = self.initial_start_time;
        for idx in 0..self.num_events {
            let activate_at = if idx == 0 {
                self.initial_start_time
            } else {
                last_activation_time + gen_time(prng, self.max_time_between_activations)
            };
            last_activation_time = activate_at;

            let deactivate_at = self
                .max_active_duration
                .map(|t| activate_at + gen_time(prng, t));

            let latitude = gen_value_in_range(
                prng,
                &self.latitude_range,
                Angle::from_degrees,
                Angle::as_degrees,
            );
            let longitude = gen_value_in_range(
                prng,
                &self.longitude_range,
                Angle::from_degrees,
                Angle::as_degrees,
            );
            let altitude = gen_value_in_range(
                prng,
                &self.altitude_range,
                Length::from_meters,
                Length::as_meters,
            );
            let velocity_east = gen_value_in_range(
                prng,
                &self.velocity_east_range,
                Velocity::from_meters_per_second,
                Velocity::as_meters_per_second,
            );
            let velocity_north = gen_value_in_range(
                prng,
                &self.velocity_north_range,
                Velocity::from_meters_per_second,
                Velocity::as_meters_per_second,
            );
            let intensity = gen_value_in_range(
                prng,
                &self.intensity_range,
                LuminousIntensity::from_candelas,
                LuminousIntensity::as_candelas,
            );

            events.push(ScheduledIREvent::from_degrees_and_meters(
                activate_at,
                deactivate_at,
                id.next(),
                latitude,
                longitude,
                altitude,
                velocity_east,
                velocity_north,
                intensity,
            ));
        }

        events
    }
}

fn gen_time(prng: &mut Rand64, max: Time) -> Time {
    Time::from_secs(prng.rand_float() * max.as_secs())
}

fn gen_value_in_range<T, F, R>(prng: &mut Rand64, range: &RangeInclusive<T>, f: F, r: R) -> T
where
    F: Fn(f64) -> T,
    R: Fn(&T) -> f64,
{
    let val = prng.rand_float();
    let from = RangeInclusive::new(0.0, 1.0);
    let to = RangeInclusive::new(r(range.start()), r(range.end()));
    let raw = map_range(val, &from, &to);
    f(raw)
}

fn map_range(value: f64, from: &RangeInclusive<f64>, to: &RangeInclusive<f64>) -> f64 {
    to.start() + (value - from.start()) * (to.end() - to.start()) / (from.end() - from.start())
}

#[derive(Debug, Default)]
struct GroundTruthIdGen(i64);

impl GroundTruthIdGen {
    fn next(&mut self) -> i64 {
        let id = self.0;
        self.0 += 1;
        id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn range_mapping() {
        let from = 0.0..=1.0;
        let to = -10.0..=10.0;
        assert_relative_eq!(map_range(0.0, &from, &to), -10.0);
        assert_relative_eq!(map_range(0.5, &from, &to), 0.0);
        assert_relative_eq!(map_range(1.0, &from, &to), 10.0);
    }
}
