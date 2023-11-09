use derive_more::Display;

pub type OrbitIndex = u64;

/// OrbitType
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Display)]
#[display(fmt = "{{index: {}, cental: {}}}", "index", "central")]
pub struct Orbit {
    /// AKA OrbitType.Tag
    pub index: OrbitIndex,

    /// Central orbit description
    pub central: OrbitDescriptor,
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default, Display)]
#[display(fmt = "{{pos_n: {}, vel_n: {}}}", "pos_n", "vel_n")]
pub struct OrbitDescriptor {
    /// Position, [m], expressed in N
    pub pos_n: na::Vector3<f64>,

    /// Velocity, [m/s], expressed in N
    pub vel_n: na::Vector3<f64>,
}

impl Orbit {
    pub fn new(index: OrbitIndex) -> Self {
        Self {
            index,
            central: Default::default(),
        }
    }
}
