use derive_more::Display;

pub type MagnetometerIndex = u64;

/// AcMagnetometerType
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Display)]
#[display(fmt = "{{index: {}, field: {}}}", "index", "field")]
pub struct Magnetometer {
    pub index: MagnetometerIndex,

    /// [Tesla]
    pub field: f64,
}

impl Magnetometer {
    pub fn new(index: MagnetometerIndex, field: f64) -> Self {
        Self { index, field }
    }
}
