pub extern crate nalgebra as na;

pub mod channel;
pub mod external_mission_control;
pub mod ground_station;
pub mod ground_truth_ir_events;
pub mod gui;
pub mod ir_event_generator;
pub mod modality;
pub mod mutator;
pub mod point_failure;
pub mod satellite;
pub mod scenario;
pub mod sim_info;
pub mod system;
pub mod units;

pub trait SimulationComponent<'a> {
    /// The type for state that is shared between multiple components; e.g. multiple subsystes in the satellite.
    type SharedState;

    /// The type for the environment structure that is scoped to this component.
    type Environment;

    fn init(&mut self, _env: &'a Self::Environment, _shared_state: &mut Self::SharedState) {}

    fn reset(&mut self, _env: &'a Self::Environment, _shared_state: &mut Self::SharedState) {}

    fn step(
        &mut self,
        dt: units::Time,
        env: &'a Self::Environment,
        common_state: &mut Self::SharedState,
    );
}

pub mod interruptor {
    use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
    use std::sync::Arc;

    #[derive(Clone, Debug)]
    #[repr(transparent)]
    pub struct Interruptor(Arc<AtomicBool>);

    impl Interruptor {
        pub fn new() -> Self {
            Interruptor(Arc::new(AtomicBool::new(false)))
        }

        pub fn set(&self) {
            self.0.store(true, SeqCst);
        }

        pub fn is_set(&self) -> bool {
            self.0.load(SeqCst)
        }
    }

    impl Default for Interruptor {
        fn default() -> Self {
            Self::new()
        }
    }
}
