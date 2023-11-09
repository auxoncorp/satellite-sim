use modality_mutation_plane::types::{MutationId, MutatorId};
use modality_mutator_protocol::{
    attrs::{AttrKey, AttrVal},
    descriptor::MutatorDescriptor,
};
use std::collections::BTreeMap;

/// This is an infallible version of SyncMutatorActuator
pub trait MutatorActuator {
    fn inject(&mut self, mutation_id: MutationId, params: BTreeMap<AttrKey, AttrVal>);

    fn reset(&mut self);
}

pub trait MutatorActuatorDescriptor: MutatorActuator + MutatorDescriptor {
    fn mutator_id(&self) -> MutatorId;
}
