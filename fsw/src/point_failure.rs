//! Utilities for managing local point failure

use std::collections::HashMap;
use tracing::debug;

use crate::{
    modality::{AttrsBuilder, MODALITY},
    units::Time,
};

#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub enum PointFailureThresholdOperator<T: PartialOrd> {
    /// Measurement is greater than or equal to the provided value
    GreaterThanEqual(T),
    /// Measurement is less than or equal to the provided value
    LessThanEqual(T),
}

#[derive(Debug, Clone)]
pub struct PointFailureConfig<T: PartialOrd> {
    /// The hold period timer is started when the
    /// threshold conditions are satisfied
    pub threshold: PointFailureThresholdOperator<T>,

    /// Amount of time required to elapse while the
    /// threshold conditions are satisfied before activating
    /// the failure. Zero means immediately
    pub hold_period: Time,
}

/// Implements a generic point failure motivated by an
/// excessive measurement value over time.
///
/// Events are logged to the caller's current timeline.
#[derive(Debug)]
pub struct PointFailure<T: PartialOrd> {
    /// Config
    config: PointFailureConfig<T>,

    /// Key/value context
    context: HashMap<String, String>,

    /// Number of instances where the threshold conditions were triggered
    triggered: usize,

    /// Timer started when the threshold conditions are met
    hold_timer: Time,

    /// Timer incremented with each update by `dt` until active, for logging purposes
    sim_clock: Time,

    /// Set the true when the failure is active
    active: bool,

    /// Explicitly disabled by the caller
    disabled: bool,
}

impl<T: Copy + PartialOrd + std::fmt::Debug> PointFailure<T> {
    pub fn new(config: PointFailureConfig<T>) -> Self {
        Self {
            config,
            context: Default::default(),
            triggered: 0,
            hold_timer: Time::from_secs(0.0),
            sim_clock: Time::from_secs(0.0),
            active: false,
            disabled: false,
        }
    }

    pub fn set_context<'a, I>(&mut self, context: I)
    where
        I: IntoIterator<Item = (&'a str, &'a str)>,
    {
        self.context = context
            .into_iter()
            .map(|(k, v)| {
                let normalized_key = if k == "name" { "point_failure.name" } else { k };
                (normalized_key.to_string(), v.to_string())
            })
            .collect();
    }

    pub fn add_context(&mut self, k: &str, v: &str) {
        let normalized_key = if k == "name" { "point_failure.name" } else { k };
        self.context
            .insert(normalized_key.to_string(), v.to_string());
    }

    pub fn config(&self) -> &PointFailureConfig<T> {
        &self.config
    }

    pub fn triggered(&self) -> usize {
        self.triggered
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn threshold(&self) -> T {
        match &self.config.threshold {
            PointFailureThresholdOperator::LessThanEqual(t) => *t,
            PointFailureThresholdOperator::GreaterThanEqual(t) => *t,
        }
    }

    /// Sets the threshold and resets
    pub fn set_threshold(&mut self, threshold: T) {
        match &mut self.config.threshold {
            PointFailureThresholdOperator::LessThanEqual(t) => *t = threshold,
            PointFailureThresholdOperator::GreaterThanEqual(t) => *t = threshold,
        }
        self.reset();
    }

    pub fn set_disabled(&mut self, disabled: bool) {
        self.disabled = disabled;
        if disabled {
            self.active = false;
            debug!(
                sim_clock = ?self.sim_clock,
                context = ?self.context,
                "Point failure disabled");
        }
    }

    pub fn reset(&mut self) {
        self.triggered = 0;
        self.hold_timer = Time::from_secs(0.0);
        self.active = false;
        self.disabled = false;
        debug!(
            sim_clock = ?self.sim_clock,
            context = ?self.context,
            "Point failure reset");
    }

    /// Returns true if the point failure is active
    pub fn update(&mut self, dt: Time, measurement: T) -> bool {
        if !self.active && !self.disabled {
            self.sim_clock += dt;

            if self.config.threshold.compare(measurement) {
                // Measurement threshold reached
                // Increment the counter and update the timer
                self.triggered = self.triggered.saturating_add(1);
                self.hold_timer += dt;

                MODALITY.quick_event_attrs("point_failure_triggered", self.attrs(measurement));

                if self.hold_timer >= self.config.hold_period {
                    // Conditions are met, now active
                    self.active = true;

                    debug!(
                        ?measurement,
                        triggered = self.triggered,
                        hold_timer = ?self.hold_timer,
                        sim_clock = ?self.sim_clock,
                        context = ?self.context,
                        "Point failure activated");

                    MODALITY.quick_event_attrs("point_failure_activated", self.attrs(measurement));
                }
            } else {
                // Measurement is below the threshold and we haven't exceeded
                // the hold period
                //
                // Reset the hold timer
                self.hold_timer = Time::from_secs(0.0);
            }
        }

        self.active
    }

    fn attrs(&self, measurement: T) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        let mut b = AttrsBuilder::new();
        b.with_prefix("event", |b| {
            self.context
                .iter()
                .for_each(|(k, v)| b.kv(k.as_str(), v.as_str()));
        });
        b.kv("event.triggered", self.triggered as i64);
        b.kv("event.hold_timer", self.hold_timer.as_secs());
        // TODO - make a Measurement trait if this stuff sticks around
        // has attrs like
        // .measurement.value
        // .measurement.unit
        b.kv("event.measurement", format!("{measurement:?}"));
        b.build()
    }
}

impl<T: PartialOrd> PointFailureThresholdOperator<T> {
    fn compare(&self, measurement: T) -> bool {
        use PointFailureThresholdOperator::*;
        match self {
            GreaterThanEqual(rhs) => measurement >= *rhs,
            LessThanEqual(rhs) => measurement <= *rhs,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Temperature;
    use approx::assert_relative_eq;

    #[test]
    fn immediate() {
        let dt = Time::from_secs(1.0);

        // Used as a timer
        let mut pf = PointFailure::new(PointFailureConfig {
            threshold: PointFailureThresholdOperator::GreaterThanEqual(Time::from_secs(1.0)),
            hold_period: Time::from_secs(0.0),
        });
        assert_eq!(pf.triggered(), 0);
        assert!(!pf.is_active());
        pf.update(dt, Time::from_secs(1.0));
        assert_eq!(pf.triggered(), 1);
        assert!(pf.is_active());
    }

    #[test]
    fn sustained_above_threshold() {
        let mut pf = PointFailure::new(PointFailureConfig {
            threshold: PointFailureThresholdOperator::GreaterThanEqual(
                Temperature::from_degrees_celsius(25.0),
            ),
            hold_period: Time::from_secs(10.0),
        });

        let dt = Time::from_secs(1.0);

        // Initial conditions
        assert_eq!(pf.triggered(), 0);
        assert!(!pf.is_active());

        // Threshold not met
        for _ in 0..20 {
            pf.update(dt, Temperature::from_degrees_celsius(24.0));
        }
        assert_eq!(pf.triggered(), 0);
        assert!(!pf.is_active());

        // Threshold triggered
        for _ in 0..9 {
            pf.update(dt, Temperature::from_degrees_celsius(25.0));
        }
        assert_eq!(pf.triggered(), 9);
        assert!(!pf.is_active());

        // Drop below threshold, resets hold timer
        pf.update(dt, Temperature::from_degrees_celsius(0.0));
        assert_eq!(pf.triggered(), 9);
        assert!(!pf.is_active());
        assert_relative_eq!(pf.hold_timer.as_secs(), 0.0);

        // Threshold triggered
        for _ in 0..9 {
            pf.update(dt, Temperature::from_degrees_celsius(25.0));
        }
        assert_eq!(pf.triggered(), 9 + 9);
        assert!(!pf.is_active());

        // Conditions are satisfied
        pf.update(dt, Temperature::from_degrees_celsius(25.0));
        assert_eq!(pf.triggered(), 10 + 9);
        assert!(pf.is_active());
    }

    #[test]
    fn sustained_below_threshold() {
        let mut pf = PointFailure::new(PointFailureConfig {
            threshold: PointFailureThresholdOperator::LessThanEqual(
                Temperature::from_degrees_celsius(-25.0),
            ),
            hold_period: Time::from_secs(10.0),
        });

        let dt = Time::from_secs(1.0);

        // Initial conditions
        assert_eq!(pf.triggered(), 0);
        assert!(!pf.is_active());

        // Threshold not met
        for _ in 0..20 {
            pf.update(dt, Temperature::from_degrees_celsius(-24.0));
        }
        assert_eq!(pf.triggered(), 0);
        assert!(!pf.is_active());

        // Threshold triggered
        for _ in 0..9 {
            pf.update(dt, Temperature::from_degrees_celsius(-25.0));
        }
        assert_eq!(pf.triggered(), 9);
        assert!(!pf.is_active());

        // Drop below threshold, resets hold timer
        pf.update(dt, Temperature::from_degrees_celsius(0.0));
        assert_eq!(pf.triggered(), 9);
        assert!(!pf.is_active());
        assert_relative_eq!(pf.hold_timer.as_secs(), 0.0);

        // Threshold triggered
        for _ in 0..9 {
            pf.update(dt, Temperature::from_degrees_celsius(-25.0));
        }
        assert_eq!(pf.triggered(), 9 + 9);
        assert!(!pf.is_active());

        // Conditions are satisfied
        pf.update(dt, Temperature::from_degrees_celsius(-25.0));
        assert_eq!(pf.triggered(), 10 + 9);
        assert!(pf.is_active());
    }
}
