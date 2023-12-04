use crate::mutator::{MutatorActuator, MutatorActuatorDescriptor, MutatorParams};
use modality_mutation_plane::types::{MutationId, MutatorId};
use modality_mutator_protocol::{
    attrs::{AttrKey, AttrVal},
    descriptor::{owned::*, MutatorDescriptor},
};

/// A generic mutator suited for boolean type operations
#[derive(Debug)]
pub struct GenericBooleanMutator {
    mutator_id: MutatorId,
    descriptor: OwnedMutatorDescriptor,
    is_active: bool,
}

impl GenericBooleanMutator {
    pub fn new(descriptor: OwnedMutatorDescriptor) -> Self {
        Self {
            mutator_id: MutatorId::allocate(),
            descriptor,
            is_active: false,
        }
    }

    pub fn is_active(&self) -> bool {
        self.is_active
    }
}

impl MutatorActuatorDescriptor for GenericBooleanMutator {}

impl MutatorDescriptor for GenericBooleanMutator {
    fn get_description_attributes(&self) -> Box<dyn Iterator<Item = (AttrKey, AttrVal)> + '_> {
        self.descriptor.clone().into_description_attributes()
    }
}

impl MutatorActuator for GenericBooleanMutator {
    fn mutator_id(&self) -> MutatorId {
        self.mutator_id
    }

    fn inject(&mut self, _mutation_id: MutationId, params: MutatorParams) {
        assert!(
            params.is_empty(),
            "GenericBooleanMutator doesn't expect any parameters"
        );
        self.is_active = true;
    }

    fn reset(&mut self) {
        self.is_active = false;
    }
}
