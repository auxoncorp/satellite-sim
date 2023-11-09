use derive_more::Display;

/// JointType
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Default, Display)]
#[display(
    fmt = "{{pos: {}, pos_rate: {}, ang: {}, ang_rate: {}}}",
    "pos",
    "pos_rate",
    "ang",
    "ang_rate"
)]
pub struct Joint {
    /// Translational kinematic state variables
    pub pos: na::Vector3<f64>,
    /// Translational dynamic state variables
    pub pos_rate: na::Vector3<f64>,

    /// Joint Euler angles
    pub ang: na::Vector3<f64>,
    /// Euler angle rates about gim axes
    pub ang_rate: na::Vector3<f64>,
}
