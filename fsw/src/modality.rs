use modality_api::{TimelineId, Uuid};
use modality_auth_token::AuthToken;
use modality_ingest_client::protocol::InternedAttrKey;
use modality_ingest_client::types::{AttrKey, AttrVal};
use modality_ingest_client::{BoundTimelineState, IngestClient, IngestError};
use modality_mutation_plane::{
    protocol::{LeafwardsMessage, RootwardsMessage, MUTATION_PROTOCOL_VERSION},
    types::{MutationId, MutatorId, ParticipantId},
};
use modality_mutation_plane_client::parent_connection::MutationParentConnection;
use std::borrow::{Borrow, Cow};
use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::env;
use std::fmt::Display;
use std::future::Future;
use std::rc::Rc;
use std::time::{Duration, SystemTime};
use url::Url;

use crate::ground_station::RackId;
use crate::mutator::MutatorActuatorDescriptor;
use crate::satellite::SatelliteId;
use crate::units::Timestamp;

const MUTATION_PROTOCOL_PARENT_URL_ENV_VAR: &str = "MUTATION_PROTOCOL_PARENT_URL";
const MUTATION_PROTOCOL_PARENT_URL_DEFAULT: &str = "modality-mutation://127.0.0.1:14192";
const MUTATION_PLANE_POLL_TIMEOUT: Duration = Duration::from_secs(0);

pub const MODALITY: ModalityClient = ModalityClient;

pub struct ModalityClient;

struct Inner {
    /// Ingest Clients, One per Timeline
    clients: HashMap<TimelineId, Option<TimelineClient>>,

    /// The current thread-local active timeline.
    ///
    /// When `timeline` is `TimelineId::zero()`, there is no current timeline and no traces will
    /// be emitted.
    timeline: TimelineId,

    /// An 'allow' filter for which, if any, satellites should emit traces.
    ///
    /// All -> Unfiltered, all emit traces
    /// None -> All filtered, none emit traces
    /// Some({A, B}) -> A and B emit traces
    satellite_filter: Filter<String>,

    /// This run's identifier, used to group all timelines from the same run.
    run_id: RunId,

    /// The mutation plane participant ID and connection.
    /// Single participant/connection for the entire simulation.
    mut_plane_pid: ParticipantId,
    mut_plane_conn: MutationParentConnection,

    /// Pending mutation plane messages for mutators.
    /// Rather than dispatch to mutators directly, we delegate message
    /// handling to the mutator-hosting-component. That way the component's
    /// ingest timeline is active and can have mutation plane events logged to it.
    mutator_messages: BTreeMap<MutatorId, Vec<MutatorMessage>>,

    /// Active mutation for mutators
    active_mutations: BTreeMap<MutatorId, MutationId>,

    /// The tokio runtime on which the clients and mutation plane participants are run.
    rt: tokio::runtime::Runtime,
}

struct TimelineClient {
    /// The client from the public SDK.
    ingest_client: IngestClient<BoundTimelineState>,
    interned_attr_keys: HashMap<AttrKey, InternedAttrKey>,
    event_index: u128,
    run_id: RunId,

    /// What the current timeline believes is the current time. Used to set 'event.timestamp'.
    local_time: Option<Timestamp>,

    /// The actual current time within the simulator. Used to set 'event.sim.timestamp'.
    sim_time: Timestamp,
}

#[must_use = "guard must be held to remain active"]
pub struct TimelineGuard {
    prev_timeline: TimelineId,
    prev_local_time: Option<Timestamp>,
}

impl Drop for TimelineGuard {
    fn drop(&mut self) {
        MODALITY.set_previous_timeline(self.prev_timeline);
        MODALITY.set_previous_local_time(self.prev_local_time);
    }
}

impl ModalityClient {
    thread_local! {
        static INNER: RefCell<Option<Inner>> = RefCell::new(None);
    }

    pub fn connect(&self) {
        Self::INNER.with(|inner| {
            if inner.borrow().is_none() {
                inner.replace(Inner::new().ok());
            }
        })
    }

    pub fn set_sim_time(&self, sim_time: Timestamp) {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner.with_current_timeline_client(|client| async {
                    client.set_sim_time(sim_time);
                });
            }
        })
    }

    pub fn set_current_timeline(
        &self,
        timeline: TimelineId,
        local_time: impl Into<Option<Timestamp>>,
    ) -> TimelineGuard {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                let prev_timeline = std::mem::replace(&mut inner.timeline, timeline);

                let prev_local_time = inner
                    .with_current_timeline_client(|client| async {
                        std::mem::replace(&mut client.local_time, local_time.into())
                    })
                    .flatten();

                TimelineGuard {
                    prev_timeline,
                    prev_local_time,
                }
            } else {
                TimelineGuard {
                    prev_timeline: TimelineId::zero(),
                    prev_local_time: None,
                }
            }
        })
    }

    pub fn current_timeline(&self) -> TimelineId {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner.timeline
            } else {
                TimelineId::zero()
            }
        })
    }

    fn set_previous_timeline(&self, timeline: TimelineId) -> TimelineId {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                std::mem::replace(&mut inner.timeline, timeline)
            } else {
                TimelineId::zero()
            }
        })
    }

    fn set_previous_local_time(&self, local_time: Option<Timestamp>) -> Option<Timestamp> {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner
                    .with_current_timeline_client(|client| async {
                        client.set_local_time(local_time)
                    })
                    .flatten()
            } else {
                None
            }
        })
    }

    pub fn emit_timeline_attrs<A, K, V>(&self, attributes: A)
    where
        A: IntoIterator<Item = (K, V)>,
        K: Into<AttrKey>,
        V: Into<AttrVal>,
    {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner
                    .with_current_timeline_client(|client| async {
                        client.emit_timeline_attrs(attributes).await
                    })
                    .unwrap_or(Ok(()))
                    .expect("emit timeline attrs");
            }
        })
    }

    pub fn emit_sat_timeline_attrs(&self, timeline_name: &str, id: &SatelliteId) {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                let should_emit = inner.satellite_filter.matches(id.name);
                if should_emit {
                    inner
                        .with_current_timeline_client(|client| async {
                            client
                                .emit_timeline_attrs([
                                    kv("timeline.name", timeline_name),
                                    kv("timeline.satellite.id", id.satcat_id),
                                    kv("timeline.satellite.name", id.name),
                                ])
                                .await
                        })
                        .unwrap_or(Ok(()))
                        .expect("emit timeline attrs");
                } else {
                    // Insert `None` client to prevent it's future creation/use.
                    inner.clients.insert(inner.timeline, None);
                }
            }
        });
    }

    pub fn emit_rack_timeline_attrs(&self, timeline_name: &str, id: RackId) {
        self.emit_timeline_attrs([
            kv("timeline.name", timeline_name),
            kv("timeline.ground_station.name", "consolidated"),
            kv("timeline.rack.id", id as i64),
        ])
    }

    pub fn emit_timeline_name(&self, name: &str) {
        let attributes = [(
            "timeline.name".to_string(),
            AttrVal::String(name.to_string().into()),
        )];
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner
                    .with_current_timeline_client(|client| client.emit_timeline_attrs(attributes))
                    .unwrap_or(Ok(()))
                    .expect("emit timeline name");
            }
        })
    }

    pub fn emit_event<A, K, V>(&self, attributes: A)
    where
        A: IntoIterator<Item = (K, V)>,
        K: Into<AttrKey>,
        V: Into<AttrVal>,
    {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner
                    .with_current_timeline_client(|client| async {
                        client.emit_event(attributes).await
                    })
                    .unwrap_or(Ok(()))
                    .expect("emit event");
            }
        })
    }

    pub fn quick_event(&self, name: &str) {
        let attributes: [(AttrKey, AttrVal); 1] = [(
            "event.name".to_string().into(),
            AttrVal::String(name.to_string().into()),
        )];

        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner
                    .with_current_timeline_client(|client| client.emit_event(attributes))
                    .unwrap_or(Ok(()))
                    .expect("emit quick event");
            }
        })
    }

    pub fn quick_event_attrs<A, K, V>(&self, name: &str, attributes: A)
    where
        A: IntoIterator<Item = (K, V)>,
        K: Into<AttrKey>,
        V: Into<AttrVal>,
    {
        let mut attrs = vec![("event.name".to_string().into(), name.into())];
        attrs.extend(attributes.into_iter().map(|(k, v)| (k.into(), v.into())));

        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                inner
                    .with_current_timeline_client(|client| client.emit_event(attrs))
                    .unwrap_or(Ok(()))
                    .expect("emit quick event with attrs");
            }
        })
    }

    pub fn poll_mutation_plane(&self) {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                let res = inner.rt.block_on(async {
                    tokio::time::timeout(
                        MUTATION_PLANE_POLL_TIMEOUT,
                        inner.mut_plane_conn.read_msg(),
                    )
                    .await
                });

                // Timeout errors are ignored when polling
                if let Ok(comms_res) = res {
                    match comms_res.expect("Mutation plane IO error") {
                        LeafwardsMessage::RequestForMutatorAnnouncements {} => {
                            // Broadcast to all mutators
                            for mailbox in inner.mutator_messages.values_mut() {
                                mailbox.push(MutatorMessage::RequestForMutatorAnnouncement);
                            }
                        }
                        LeafwardsMessage::NewMutation {
                            mutator_id,
                            mutation_id,
                            maybe_trigger_mask: _,
                            params,
                        } => {
                            if let Some(mailbox) = inner.mutator_messages.get_mut(&mutator_id) {
                                mailbox.push(
                                    MutatorMessage::NewMutation { mutation_id, params });
                            } else {
                                tracing::warn!(
                                    mutator_id = %mutator_id,
                                    "Failed to handle new mutation, mutator not hosted by this client");
                            }
                        }
                        LeafwardsMessage::ClearSingleMutation {
                            mutator_id,
                            mutation_id,
                            reset_if_active,
                        } => {
                            if let Some(mailbox) = inner.mutator_messages.get_mut(&mutator_id) {
                                if Some(&mutation_id) == inner.active_mutations.get(&mutator_id) {
                                mailbox.push(
                                    MutatorMessage::ClearMutation{reset_if_active});
                                } else {
                                    tracing::warn!(
                                        mutator_id = %mutator_id,
                                        mutation_id = %mutation_id,
                                        "Failed to clear mutation, mutation not active");
                                }
                            } else {
                                tracing::warn!(
                                    mutator_id = %mutator_id,
                                    "Failed to clear mutation, mutator not hosted by this client");
                            }
                        }
                        LeafwardsMessage::ClearMutationsForMutator {
                            mutator_id,
                            reset_if_active,
                        } => {
                            if let Some(mailbox) = inner.mutator_messages.get_mut(&mutator_id) {
                                mailbox.push(
                                    MutatorMessage::ClearMutation{reset_if_active});
                            } else {
                                tracing::warn!(
                                    mutator_id = %mutator_id,
                                    "Failed to clear mutation, mutator not hosted by this client");
                            }
                        }
                        LeafwardsMessage::ClearMutations {} => {
                            // Broadcast to all mutators
                            for mailbox in inner.mutator_messages.values_mut() {
                                mailbox.push(MutatorMessage::ClearMutation { reset_if_active: true });
                            }
                        }
                        msg => tracing::debug!(
                            message = msg.name(),
                            "Ignoring mutation plane leafwards message"
                        ),
                    }
                }
            }
        })
    }

    pub fn register_mutator<M: MutatorActuatorDescriptor>(&self, m: &M) {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                let announcement = mutator_announcement(inner.mut_plane_pid, m);
                inner
                    .rt
                    .block_on(inner.mut_plane_conn.write_msg(&announcement))
                    .expect("Mutator announcement");
                inner.mutator_messages.insert(m.mutator_id(), Vec::new());
            }
        })
    }

    /// Handle any pending mutation plane messages for the given mutators.
    /// NOTE: it is expected that the logical component's timeline will be active
    /// so that any mutation plane related events end up on the appropriate timeline.
    pub fn process_mutation_plane_messages<'a, M, I>(&self, mutators: I)
    where
        M: MutatorActuatorDescriptor + 'a,
        I: Iterator<Item = &'a mut M> + 'a,
    {
        Self::INNER.with(|inner| {
            if let Some(inner) = inner.borrow_mut().as_mut() {
                for mutator in mutators {
                    let mutator_id = mutator.mutator_id();
                    let mailbox = inner
                        .mutator_messages
                        .get_mut(&mutator_id)
                        .expect("Mutator not registered");
                    for msg in mailbox.drain(..) {
                        match msg {
                            MutatorMessage::RequestForMutatorAnnouncement => {
                                let announcement =
                                    mutator_announcement(inner.mut_plane_pid, mutator);
                                inner
                                    .rt
                                    .block_on(inner.mut_plane_conn.write_msg(&announcement))
                                    .expect("Mutator announcement");
                            }
                            MutatorMessage::NewMutation {
                                mutation_id,
                                params,
                            } => {
                                // Reset active mutation first, if any
                                if let Some(active_mutation_id) =
                                    inner.active_mutations.remove(&mutator_id)
                                {
                                    tracing::debug!(
                                            mutator_id = %mutator_id,
                                            mutation_id = %active_mutation_id,
                                            "Clearing currently active mutation");
                                    mutator.reset();
                                }

                                let params = params
                                    .0
                                    .into_iter()
                                    .map(|kv| (kv.key.into(), kv.value))
                                    .collect();

                                mutator.inject(mutation_id, params);
                            }
                            MutatorMessage::ClearMutation { reset_if_active } => {
                                if let Some(active_mutation_id) =
                                    inner.active_mutations.remove(&mutator_id)
                                {
                                    tracing::debug!(
                                            mutator_id = %mutator_id,
                                            mutation_id = %active_mutation_id,
                                            "Clearing mutation");

                                    if reset_if_active {
                                        mutator.reset();
                                    }
                                }
                            }
                        }
                    }
                }
            }
        })
    }
}

pub fn kv(k: &str, v: impl Into<AttrVal>) -> (AttrKey, AttrVal) {
    (AttrKey::from(k.to_string()), v.into())
}

pub struct AttrsBuilder {
    prefix: Vec<String>,
    kvs: Vec<(AttrKey, AttrVal)>,
}

impl AttrsBuilder {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        AttrsBuilder {
            prefix: vec![],
            kvs: vec![],
        }
    }
    pub fn kv(&mut self, k: &str, v: impl Into<AttrVal>) {
        let k = if self.prefix.is_empty() {
            AttrKey::from(k.to_string())
        } else {
            AttrKey::from(format!("{}.{}", self.prefix.join("."), k))
        };
        self.kvs.push((k, v.into()));
    }

    pub fn push_prefix(&mut self, p: &str) {
        self.prefix.push(p.to_string());
    }

    pub fn pop_prefix(&mut self) {
        self.prefix.pop();
    }

    pub fn with_prefix(&mut self, p: &str, f: impl Fn(&mut AttrsBuilder)) {
        self.push_prefix(p);
        (f)(self);
        self.pop_prefix();
    }

    pub fn build(self) -> Vec<(AttrKey, AttrVal)> {
        self.kvs
    }
}

impl Inner {
    fn new() -> Result<Inner, IngestError> {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();

        let run_id =
            std::env::var("MODALITY_RUN_ID").unwrap_or_else(|_1| Uuid::new_v4().to_string());
        let run_id = RunId::new(run_id);

        // Note: This connection isn't kept, it's just to check if modality is there to see if we
        // should try to actually connect each time a timeline is created.
        let _ = rt.block_on(async {
            TimelineClient::connect(TimelineId::zero(), run_id.clone()).await
        })?;

        let satellite_filter = match std::env::var("TRACE_SATELLITES").ok() {
            None => Filter::None,
            Some(e) if e.is_empty() => Filter::All,
            Some(f) => {
                let sat_list = f.split(',').map(|s| s.trim().to_string()).collect();
                Filter::Some(sat_list)
            }
        };

        // NOTE: By this point, we know the ingest connection is working, so rather than returning the
        // result, we panic so it doesn't go undetected since we always expect the mutation
        // plane to be connected along with ingest
        let mut_plane_pid = ParticipantId::from(Uuid::new_v4());
        let mut_url = mutation_proto_parent_url().expect("Mutation protocol parent URL");
        let auth_token = AuthToken::load().expect("Auth token for mutation client");
        let allow_insecure_tls = true;
        let mut mut_plane_conn = rt
            .block_on(MutationParentConnection::connect(
                &mut_url,
                allow_insecure_tls,
            ))
            .expect("Mutation parent connection");
        rt.block_on(
            mut_plane_conn.write_msg(&RootwardsMessage::ChildAuthAttempt {
                child_participant_id: mut_plane_pid,
                version: MUTATION_PROTOCOL_VERSION,
                token: auth_token.as_ref().to_vec(),
            }),
        )
        .expect("Mutation auth");

        match rt
            .block_on(mut_plane_conn.read_msg())
            .expect("Mutation auth response")
        {
            LeafwardsMessage::ChildAuthOutcome {
                child_participant_id,
                version: _,
                ok,
                message,
            } => {
                if child_participant_id == mut_plane_pid {
                    if !ok {
                        panic!("Mutation plane authorization failed. {message:?}");
                    }
                } else {
                    panic!("Mutation plane auth outcome received for a different participant");
                }
            }
            resp => panic!("Mutation plane unexpected auth response. Got {resp:?}"),
        }

        Ok(Inner {
            clients: HashMap::new(),
            timeline: TimelineId::zero(),
            satellite_filter,
            run_id,
            mut_plane_pid,
            mut_plane_conn,
            mutator_messages: Default::default(),
            active_mutations: Default::default(),
            rt,
        })
    }

    // NOTE.pb: I don't really like that this returns Option<O>, but I
    // couldn't think of a better way to do it.
    fn with_current_timeline_client<'s: 'c, 'c, F, O>(
        &'s mut self,
        f: impl FnOnce(&'c mut TimelineClient) -> F,
    ) -> Option<O>
    where
        F: Future<Output = O> + 'c,
    {
        let client = self.clients.entry(self.timeline).or_insert_with(|| {
            if self.timeline != TimelineId::zero() {
                Some(self.rt.block_on(async {
                    TimelineClient::connect(self.timeline, self.run_id.clone())
                        .await
                        .expect("connect for new timeline")
                }))
            } else {
                None
            }
        });

        client.as_mut().map(|client| self.rt.block_on(f(client)))
    }
}

impl TimelineClient {
    async fn connect(timeline: TimelineId, run_id: RunId) -> Result<TimelineClient, IngestError> {
        let ingest_client =
            IngestClient::connect_with_standard_config(Duration::from_secs(5), None, None).await?;

        let ingest_client = ingest_client.open_timeline(timeline).await?;

        let mut client = TimelineClient {
            ingest_client,
            interned_attr_keys: HashMap::new(),
            event_index: 0,
            run_id,
            local_time: None,
            sim_time: Timestamp::epoch(),
        };

        // pre-declare default attributes
        client
            .intern_attributes([
                ("event.name".to_string(), AttrVal::Bool(false)),
                ("event.timestamp".to_string(), AttrVal::Bool(false)),
                ("event.sim.timestamp".to_string(), AttrVal::Bool(false)),
                ("event.system.timestamp".to_string(), AttrVal::Bool(false)),
                ("timeline.run_id".to_string(), AttrVal::Bool(false)),
            ])
            .await?;

        Ok(client)
    }

    async fn intern_attributes<A, K, V>(
        &mut self,
        attributes: A,
    ) -> Result<Vec<(InternedAttrKey, AttrVal)>, IngestError>
    where
        A: IntoIterator<Item = (K, V)>,
        K: Into<AttrKey>,
        V: Into<AttrVal>,
    {
        let mut interned_attributes = Vec::new();
        for (ak, av) in attributes.into_iter() {
            let (ak, av) = (ak.into(), av.into());
            let iak = if let Some(iak) = self.interned_attr_keys.get(&ak) {
                *iak
            } else {
                let iak = self.ingest_client.declare_attr_key(ak.to_string()).await?;
                self.interned_attr_keys.insert(ak, iak);
                iak
            };
            interned_attributes.push((iak, av));
        }
        Ok(interned_attributes)
    }

    fn append_default_timeline_attrs(&mut self, attrs: &mut Vec<(InternedAttrKey, AttrVal)>) {
        let run_id_key = self
            .interned_attr_keys
            .get(&"timeline.run_id".to_string().into())
            .expect("get event.system.timestamp attr key, declared in connect");

        attrs.push((*run_id_key, self.run_id.to_string().into()));
    }

    fn append_default_event_attrs(&mut self, attrs: &mut Vec<(InternedAttrKey, AttrVal)>) {
        // event.system.timestamp
        let system_timestamp = (SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .expect("this software isn't used while timetraveling")
            .as_nanos() as u64)
            .into();

        let system_timestamp_key = self
            .interned_attr_keys
            .get(&"event.system.timestamp".to_string().into())
            .expect("get event.system.timestamp attr key, declared in connect");

        attrs.push((*system_timestamp_key, AttrVal::Timestamp(system_timestamp)));

        // event.sim.timestamp
        let sim_timestamp_key = self
            .interned_attr_keys
            .get(&"event.sim.timestamp".to_string().into())
            .expect("get event.sim.timestamp attr key, declared in connect");

        attrs.push((
            *sim_timestamp_key,
            AttrVal::Timestamp(self.sim_time.as_nanos().into()),
        ));

        // event.timestamp
        if let Some(ts) = self.local_time {
            let timestamp_key = self
                .interned_attr_keys
                .get(&"event.timestamp".to_string().into())
                .expect("get event.timestamp attr key, declared in connect");

            attrs.push((*timestamp_key, AttrVal::Timestamp(ts.as_nanos().into())));
        }
    }

    fn set_local_time(&mut self, local_time: Option<Timestamp>) -> Option<Timestamp> {
        std::mem::replace(&mut self.local_time, local_time)
    }

    fn set_sim_time(&mut self, sim_time: Timestamp) {
        self.sim_time = sim_time;
    }

    async fn emit_timeline_attrs<A, K, V>(&mut self, attributes: A) -> Result<(), IngestError>
    where
        A: IntoIterator<Item = (K, V)>,
        K: Into<AttrKey>,
        V: Into<AttrVal>,
    {
        let mut interned_attributes = self.intern_attributes(attributes).await?;
        self.append_default_timeline_attrs(&mut interned_attributes);
        self.ingest_client
            .timeline_metadata(interned_attributes)
            .await
    }

    async fn emit_event<A, K, V>(&mut self, attributes: A) -> Result<(), IngestError>
    where
        A: IntoIterator<Item = (K, V)>,
        K: Into<AttrKey>,
        V: Into<AttrVal>,
    {
        let mut interned_attributes = self.intern_attributes(attributes).await?;
        self.append_default_event_attrs(&mut interned_attributes);
        self.event_index += 1;
        self.ingest_client
            .event(self.event_index, interned_attributes)
            .await
    }
}

enum Filter<T> {
    All,
    None,
    Some(BTreeSet<T>),
}

impl<T> Filter<T> {
    fn matches<Q>(&self, value: &Q) -> bool
    where
        T: Borrow<Q> + Ord,
        Q: Ord + ?Sized,
    {
        match self {
            Filter::All => true,
            Filter::None => false,
            Filter::Some(filter) => filter.contains(value),
        }
    }
}

#[derive(Clone)]
struct RunId {
    id: Rc<String>,
}

impl RunId {
    pub fn new<'a>(id: impl Into<Cow<'a, str>>) -> Self {
        let sid = id.into().to_string();
        RunId { id: Rc::new(sid) }
    }
}

impl AsRef<str> for RunId {
    fn as_ref(&self) -> &str {
        &self.id
    }
}

impl Display for RunId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.id.fmt(f)
    }
}

impl From<RunId> for AttrVal {
    fn from(value: RunId) -> Self {
        AttrVal::String(value.id.to_string().into())
    }
}

#[derive(Debug, thiserror::Error)]
pub enum MutationProtocolUrlError {
    #[error(
        "The MUTATION_PROTOCOL_PARENT_URL environment variable contained a non-UTF-8-compatible string"
    )]
    EnvVarSpecifiedMutationProtoParentUrlNonUtf8,

    #[error("Mutation protocol parent URL error")]
    MutationProtoParentUrl(#[from] url::ParseError),
}

fn mutation_proto_parent_url() -> Result<Url, MutationProtocolUrlError> {
    match env::var(MUTATION_PROTOCOL_PARENT_URL_ENV_VAR) {
        Ok(val) => Ok(Url::parse(&val)?),
        Err(env::VarError::NotUnicode(_)) => {
            Err(MutationProtocolUrlError::EnvVarSpecifiedMutationProtoParentUrlNonUtf8)
        }
        Err(env::VarError::NotPresent) => Ok(Url::parse(MUTATION_PROTOCOL_PARENT_URL_DEFAULT)?),
    }
}

fn mutator_announcement<M: MutatorActuatorDescriptor>(
    participant_id: ParticipantId,
    m: &M,
) -> RootwardsMessage {
    let mutator_attrs = m
        .get_description_attributes()
        .map(|(k, value)| modality_mutation_plane::types::AttrKv {
            key: k.to_string(),
            value,
        })
        .collect();
    RootwardsMessage::MutatorAnnouncement {
        participant_id,
        mutator_id: m.mutator_id(),
        mutator_attrs: modality_mutation_plane::types::AttrKvs(mutator_attrs),
    }
}

/// A reduced variant of LeafwardsMessage, tailored for
/// a single mutator that only supports a single active mutation
/// at a time.
#[derive(Clone, Debug, PartialEq)]
enum MutatorMessage {
    RequestForMutatorAnnouncement,
    NewMutation {
        mutation_id: MutationId,
        params: modality_mutation_plane::types::AttrKvs,
    },
    ClearMutation {
        reset_if_active: bool,
    },
}
