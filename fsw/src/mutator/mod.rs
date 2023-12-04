use modality_mutation_plane::types::{MutationId, MutatorId};
use modality_mutator_protocol::{
    attrs::{AttrKey, AttrVal},
    descriptor::MutatorDescriptor,
};
use std::collections::BTreeMap;

pub use generic_mutator::GenericBooleanMutator;

mod generic_mutator;

pub type MutatorParams = BTreeMap<AttrKey, AttrVal>;

/// This is a specialized/infallible version of SyncMutatorActuator
pub trait MutatorActuator {
    fn mutator_id(&self) -> MutatorId;

    fn inject(&mut self, mutation_id: MutationId, params: BTreeMap<AttrKey, AttrVal>);

    fn reset(&mut self);
}

pub trait MutatorActuatorDescriptor: MutatorActuator + MutatorDescriptor {}
