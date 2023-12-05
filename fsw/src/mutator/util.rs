use crate::satellite::SatelliteId;
use modality_mutator_protocol::descriptor::owned::*;
use std::collections::HashMap;

pub fn watchdog_out_of_sync_descriptor(
    component_name: &str,
    id: &SatelliteId,
) -> OwnedMutatorDescriptor {
    OwnedMutatorDescriptor {
        name: format!("{} watchdog execution out-of-sync", component_name).into(),
        description: format!(
            "Sets the {} watchdog execution out-of-sync error register bit",
            component_name
        )
        .into(),
        layer: MutatorLayer::Implementational.into(),
        group: component_name.to_owned().into(),
        operation: MutatorOperation::Enable.into(),
        statefulness: MutatorStatefulness::Transient.into(),
        organization_custom_metadata: OrganizationCustomMetadata::new(
            "satellite".to_string(),
            HashMap::from([
                ("id".to_string(), id.satcat_id.into()),
                ("name".to_string(), id.name.into()),
                ("component_name".to_string(), component_name.into()),
            ]),
        ),
        params: Default::default(),
    }
}
