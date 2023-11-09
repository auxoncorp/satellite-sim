use derive_more::Display;

pub type CoarseSunSensorIndex = u64;

/// AcCssType
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Display)]
#[display(fmt = "{{index: {}, valid: {}, illum: {}}}", "index", "valid", "illum")]
pub struct CoarseSunSensor {
    pub index: CoarseSunSensorIndex,

    /// Ignore if false
    pub valid: bool,

    /// Illumincation value.
    /// Typically in the range of [-1, +1] (based on the scale config).
    /// Positive when the CSS axis is pointing towards the sun, negative when pointing away.
    pub illum: f64,
}

impl CoarseSunSensor {
    pub fn new(index: CoarseSunSensorIndex, valid: bool, illum: f64) -> Self {
        Self {
            index,
            valid,
            illum,
        }
    }
}
