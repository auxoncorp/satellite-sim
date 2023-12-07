mod consolidated;
mod mcui;
mod operator;
mod rack;
mod relay;
mod result_selection;

use crate::{channel::TracedMessage, modality::kv, units::Timestamp};

pub use consolidated::{ConsolidatedGroundStation, ConsolidatedGroundStationConfig};
pub use mcui::MissionControlUIConfig;
pub use operator::{IROperator, IROperatorConfig, SatOperator, SatOperatorConfig};
pub use rack::{
    CorrelationConfig, IntensityAnalysisConfig, RackConfig, RackId, SynthesisConfig,
    SynthesisFaultConfig, TimeSourceConfig, TimeSourceFaultConfig, VelocityAnalysisConfig,
};
pub use relay::{
    GroundStationId, RelayGroundStation, RelayGroundStationConfig, RelayGroundStationFaultConfig,
};
pub use result_selection::ResultSelectionConfig;

/// A message that has been forwared through a relay
#[derive(Debug, Clone)]
pub struct Relayed<T: TracedMessage> {
    pub relay_ground_station_id: GroundStationId,
    pub relay_timestamp: Timestamp,
    pub inner: T,
}

#[derive(Debug, Clone)]
pub enum OperationalStatus {
    /// Everything's working great
    Nominal,

    /// Everything is still functional, but the system is in a partial failure state.
    Degraded,

    /// The system is degraded to the point where the results cannot be trusted
    NotTrustworthy,
}

impl OperationalStatus {
    pub fn name(&self) -> &'static str {
        match self {
            OperationalStatus::Nominal => "Nominal",
            OperationalStatus::Degraded => "Degraded",
            OperationalStatus::NotTrustworthy => "NotTrustworthy",
        }
    }
}

impl<T: TracedMessage> TracedMessage for Relayed<T> {
    fn attrs(&self) -> Vec<(modality_api::AttrKey, modality_api::AttrVal)> {
        let mut kvs = self.inner.attrs();
        kvs.push(kv(
            "event.relay.ground_station_id",
            i64::from(self.relay_ground_station_id),
        ));
        kvs.push(kv("event.relay.timestamp", self.relay_timestamp));
        kvs
    }
}
