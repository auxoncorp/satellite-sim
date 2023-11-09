use crate::{
    body::{Body, BodyIndex},
    fsw_data::FswData,
    joint::Joint,
};
use derive_more::Display;
use std::collections::HashMap;

pub type SpacecraftIndex = u64;

#[derive(Clone, PartialEq, Debug, Display)]
#[display(
    fmt = "{{index: {}, pos_r: {}, vel_r: {}, ...}}",
    "index",
    "pos_r",
    "vel_r"
)]
pub struct Spacecraft {
    /// AKA SCType.ID
    pub index: SpacecraftIndex,

    /// Position of cm wrt Reference Orbit [m], expressed in N
    pub pos_r: na::Vector3<f64>,
    /// Velocity of cm wrt Reference Orbit [m/s], expressed in N
    pub vel_r: na::Vector3<f64>,
    /// Sun-pointing unit vector, expressed in SC.B[0]
    pub svb: na::Vector3<f64>,
    /// Magfield [Tesla], expressed in SC.B[0]
    pub bvb: na::Vector3<f64>,
    /// Total SC angular momentum [Nms], expressed in SC.B[0]
    pub hvb: na::Vector3<f64>,

    /// AKA SCType.B
    pub bodies: HashMap<BodyIndex, Body>,

    /// Joint between N and B[0]
    pub gn: Joint,

    /// AKA SCType.AC
    pub ac: FswData,
}

impl Spacecraft {
    pub fn new(index: SpacecraftIndex) -> Self {
        Self {
            index,
            pos_r: Default::default(),
            vel_r: Default::default(),
            svb: Default::default(),
            bvb: Default::default(),
            hvb: Default::default(),
            bodies: Default::default(),
            gn: Default::default(),
            ac: Default::default(),
        }
    }
}
