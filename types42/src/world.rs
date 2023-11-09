use crate::orbit::OrbitDescriptor;
use derive_more::Display;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Display)]
pub enum WorldKind {
    Sol,
    Earth,
    Luna,
    Unknown(u64),
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug, Display)]
#[display(fmt = "{{kind: {}, pos_h: {}, ...}}", "kind", "pos_h")]
pub struct World {
    /// AKA WorldType.Type
    pub kind: WorldKind,

    /// Position in H frame [m]
    pub pos_h: na::Vector3<f64>,

    /// Ephemeris
    pub eph: OrbitDescriptor,
}

impl World {
    pub fn new(kind: WorldKind) -> Self {
        Self {
            kind,
            pos_h: Default::default(),
            eph: Default::default(),
        }
    }
}

impl From<u64> for WorldKind {
    fn from(value: u64) -> Self {
        use WorldKind::*;
        match value {
            0 => Sol,
            3 => Earth,
            10 => Luna,
            _ => Unknown(value),
        }
    }
}

impl From<WorldKind> for u64 {
    fn from(value: WorldKind) -> Self {
        use WorldKind::*;
        match value {
            Sol => 0,
            Earth => 3,
            Luna => 10,
            Unknown(v) => v,
        }
    }
}

impl WorldKind {
    pub const SOL_RADIUS: f64 = 6.98E8;
    pub const EARTH_RADIUS: f64 = 6.378145E6;
    pub const LUNA_RADIUS: f64 = 1.738E6;

    /// Radius [m]
    pub const fn radius(self) -> Option<f64> {
        use WorldKind::*;
        Some(match self {
            Sol => Self::SOL_RADIUS,
            Earth => Self::EARTH_RADIUS,
            Luna => Self::LUNA_RADIUS,
            _ => return None,
        })
    }

    /// Rotation rate [rad/s]
    pub const fn rotation_rate(self) -> Option<f64> {
        use WorldKind::*;
        Some(match self {
            Sol => 2.69E-6,
            Earth => 7.292115E-5,
            Luna => 2.66E-6,
            _ => return None,
        })
    }
}
