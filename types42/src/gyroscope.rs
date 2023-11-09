use derive_more::Display;

pub type GyroscopeIndex = u64;

/// AcGyroType
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Display)]
#[display(fmt = "{{index: {}, rate: {}}}", "index", "rate")]
pub struct Gyroscope {
    pub index: GyroscopeIndex,

    /// Angular rate [rad/s]
    pub rate: f64,
}

impl Gyroscope {
    pub fn new(index: GyroscopeIndex, rate: f64) -> Self {
        Self { index, rate }
    }
}
