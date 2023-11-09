use derive_more::Display;

pub type AccelerometerIndex = u64;

/// AcAccelType
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Display)]
#[display(fmt = "{{ndex: {}, acc: {}}}", "index", "acc")]
pub struct Accelerometer {
    pub index: AccelerometerIndex,

    /// Acceleration [m/s^2]
    pub acc: f64,
}

impl Accelerometer {
    pub fn new(index: AccelerometerIndex, acc: f64) -> Self {
        Self { index, acc }
    }
}
