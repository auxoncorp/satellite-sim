use uom::si::{angle::radian, f64::*, luminous_intensity::candela, ratio::percent};

use crate::{
    ground_station::GroundStation,
    vision::{EventClass, IREvent},
};

// TODO - revisit this stuff
// need a "cmd/sim at time t"
// ScheduledIREVent
// mv this mod to scenario
pub fn gen_ground_stations_as_ir_events() -> Vec<IREvent> {
    GroundStation::all_known_ground_stations()
        .into_iter()
        .map(|(_id, gs)| IREvent {
            location: gs.position,
            intensity: LuminousIntensity::new::<candela>(1.0),
            classification: EventClass::Launch,
            classification_confidence: Ratio::new::<percent>(100.0),
        })
        .collect()
}
