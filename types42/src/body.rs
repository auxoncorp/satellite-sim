use derive_more::Display;

pub type BodyIndex = u64;

/// BodyType
#[derive(Copy, Clone, PartialEq, Debug, Display)]
#[display(fmt = "{{index: {}, wn: {}, qn: {}}}", "index", "wn", "qn")]
pub struct Body {
    pub index: BodyIndex,

    /// Angular Velocity of B wrt N expressed in B frame [rad/sec]
    pub wn: na::Vector3<f64>,

    pub qn: na::Quaternion<f64>,
}

impl Body {
    pub fn new(index: BodyIndex) -> Self {
        Self {
            index,
            wn: Default::default(),
            qn: Default::default(),
        }
    }
}
