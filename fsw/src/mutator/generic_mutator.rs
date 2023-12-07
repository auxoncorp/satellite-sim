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

impl MutatorActuatorDescriptor for GenericBooleanMutator {
    fn as_dyn(&mut self) -> &mut dyn MutatorActuatorDescriptor {
        self
    }
}

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

/// A generic set-float-value mutator
#[derive(Debug)]
pub struct GenericSetFloatMutator {
    mutator_id: MutatorId,
    descriptor: OwnedMutatorDescriptor,
    active_mutation: Option<f64>,

    /// Some convenience state for the user.
    /// Gets cleared when a reset occurs.
    pub was_applied: bool,
}

impl GenericSetFloatMutator {
    pub fn new(descriptor: OwnedMutatorDescriptor) -> Self {
        assert!(
            descriptor.params.len() == 1,
            "GenericSetFloatMutator expects a single float parameter"
        );
        Self {
            mutator_id: MutatorId::allocate(),
            descriptor,
            active_mutation: None,
            was_applied: false,
        }
    }

    pub fn active_mutation(&self) -> Option<f64> {
        self.active_mutation
    }
}

impl MutatorActuatorDescriptor for GenericSetFloatMutator {
    fn as_dyn(&mut self) -> &mut dyn MutatorActuatorDescriptor {
        self
    }
}

impl MutatorDescriptor for GenericSetFloatMutator {
    fn get_description_attributes(&self) -> Box<dyn Iterator<Item = (AttrKey, AttrVal)> + '_> {
        self.descriptor.clone().into_description_attributes()
    }
}

impl MutatorActuator for GenericSetFloatMutator {
    fn mutator_id(&self) -> MutatorId {
        self.mutator_id
    }

    fn inject(&mut self, _mutation_id: MutationId, params: MutatorParams) {
        assert!(
            params.len() == 1,
            "GenericSetFloatMutator expects a single float parameter"
        );
        let (_key, val) = params.into_iter().take(1).next().unwrap();
        if let AttrVal::Float(f) = val {
            self.active_mutation = Some(f.0);
        } else {
            panic!("GenericSetFloatMutator expects a single float parameter, got {val}");
        }
    }

    fn reset(&mut self) {
        self.active_mutation = None;
        self.was_applied = false;
    }
}
