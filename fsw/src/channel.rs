//! These are channel-like structures, but they are set up to work in
//! the deterministic, single-threaded simulation environment.

use crate::{
    modality::{kv, MODALITY},
    units::Length,
};
use modality_api::{AttrKey, AttrVal, TimelineId};
use nav_types::NVector;
use rand::Rng;
use std::{cell::RefCell, collections::VecDeque, rc::Rc};
use thiserror::Error;
use types42::prelude::WorldKind;

/// We need to be able to turn each message into a trace event
pub trait TracedMessage {
    fn attrs(&self) -> Vec<(AttrKey, AttrVal)>;
}

/// A single-threaded mpmc channel that requires manual stepping for
/// messages to propagate. Messages sit in a sender's outbox until the
/// step; after that, they sit in a receiver's inbox.  Senders and
/// receivers can have a location; if they both do, then messages are
/// delivered when there is line-of-sight.
#[derive(Debug)]
pub struct StepChannel<T: Clone + TracedMessage> {
    senders: Vec<Sender<T>>,
    receivers: Vec<Receiver<T>>,
}

impl<T: Clone + TracedMessage> Default for StepChannel<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Clone + TracedMessage> StepChannel<T> {
    pub fn new() -> Self {
        StepChannel {
            senders: vec![],
            receivers: vec![],
        }
    }

    pub fn sender(&mut self, outbox_capacity: impl Into<Option<usize>>) -> Sender<T> {
        let rc = Rc::new(RefCell::new(InnerSender {
            loc: None,
            outbox: VecDeque::new(),
            outbox_capacity: outbox_capacity.into(),
        }));
        self.senders.push(Sender(rc.clone()));
        Sender(rc)
    }

    pub fn receiver(&mut self, inbox_capacity: impl Into<Option<usize>>) -> Receiver<T> {
        let rc = Rc::new(RefCell::new(InnerReceiver {
            loc: None,
            inbox: VecDeque::new(),
            inbox_capacity: inbox_capacity.into(),
        }));
        self.receivers.push(Receiver(rc.clone()));
        Receiver(rc)
    }
}

/// This is a trait so we can erase the message type, and still step
pub trait Step {
    fn step(&mut self) -> Result<(), ChannelError>;
}

impl<T: Clone + TracedMessage> Step for StepChannel<T> {
    fn step(&mut self) -> Result<(), ChannelError> {
        for s in self.senders.iter() {
            let mut s_inner = s.0.borrow_mut();

            while let Some(msg) = s_inner.outbox.pop_front() {
                for r in self.receivers.iter() {
                    let mut r_inner = r.0.borrow_mut();

                    // use sender / receiver loc to determine line-of-sight
                    if let (Some(s_loc), Some(r_loc)) = (s_inner.loc, r_inner.loc) {
                        let sla = s_loc.altitude();
                        let rla = r_loc.altitude();
                        let maybe_sat_altitude = if sla > 0.0 && rla == 0.0 {
                            Some(sla)
                        } else if sla == 0.0 && rla > 0.0 {
                            Some(rla)
                        } else {
                            None
                        };

                        if let Some(sat_altitude_m) = maybe_sat_altitude {
                            let sat_altitude = Length::from_meters(sat_altitude_m);
                            let visibility_cutoff = max_line_of_sight(sat_altitude);

                            // TODO add configurable loss factor, based on distance
                            if euclidean_distance(s_loc, r_loc) > visibility_cutoff {
                                continue;
                            }
                        }
                    }

                    if let Some(inbox_capacity) = r_inner.inbox_capacity {
                        if r_inner.inbox.len() >= inbox_capacity {
                            return Err(ChannelError::QueueFull);
                        }
                    }

                    r_inner.inbox.push_back(msg.clone());
                }
            }
        }

        Ok(())
    }
}

/// The maximum line-of-sight distance that this satellite can communicate, based on its altitude.
///
/// Geometric justification: ../sat_line_of_sight.svg
///  - r: earth radius
///  - a: satellite altitude
///  - L: max line of sight
pub fn max_line_of_sight(sat_altitude: Length) -> Length {
    let r = earth_radius();
    let a = sat_altitude;
    ((a * a) + ((2.0 * a) * r)).sqrt()
}

const EARTH_RADIUS_METERS: f64 = WorldKind::EARTH_RADIUS;
fn earth_radius() -> Length {
    Length::from_meters(EARTH_RADIUS_METERS)
}

// See https://www.ffi.no/en/research/n-vector, example 5, bottom part
fn euclidean_distance(a: NVector<f64>, b: NVector<f64>) -> Length {
    let a_e: nav_types::ECEF<f64> = a.into();
    let b_e: nav_types::ECEF<f64> = b.into();

    // The docs say this is euclidean distance
    Length::from_meters(a_e.distance(&b_e))
}

#[derive(Debug, Clone)]
pub struct Msg<T> {
    item: T,
    sender: TimelineId,
    nonce: i64,
}

#[derive(Debug)]
pub struct Sender<T>(Rc<RefCell<InnerSender<T>>>);

#[derive(Debug)]
pub struct InnerSender<T> {
    loc: Option<NVector<f64>>,
    outbox: VecDeque<Msg<T>>,
    outbox_capacity: Option<usize>,
}

impl<T: TracedMessage> Sender<T> {
    #[allow(dead_code)]
    pub fn location(&self) -> Option<NVector<f64>> {
        let inner = self.0.borrow();
        inner.loc
    }

    pub fn try_send(&mut self, item: T) -> Result<(), ChannelError> {
        let mut inner = self.0.borrow_mut();

        if let Some(capacity) = inner.outbox_capacity {
            if inner.outbox.len() >= capacity {
                return Err(ChannelError::QueueFull);
            }
        }

        let sender = MODALITY.current_timeline();
        let mut rng = rand::thread_rng();
        let nonce = rng.gen();

        let mut kvs = item.attrs();
        kvs.push(kv("event.nonce", nonce));
        kvs.push(kv("event.channel.send", true));
        MODALITY.emit_event(kvs);

        let msg = Msg {
            item,
            sender,
            nonce,
        };

        inner.outbox.push_back(msg);
        Ok(())
    }

    pub fn set_location(&mut self, loc: NVector<f64>) {
        let mut inner = self.0.borrow_mut();
        inner.loc = Some(loc);
    }

    pub fn with_location(mut self, loc: NVector<f64>) -> Self {
        self.set_location(loc);
        self
    }

    pub fn clear(&mut self) {
        let mut inner = self.0.borrow_mut();
        inner.outbox.clear();
    }
}

#[derive(Debug)]
pub struct Receiver<T>(Rc<RefCell<InnerReceiver<T>>>);

#[derive(Debug)]
pub struct InnerReceiver<T> {
    loc: Option<NVector<f64>>,
    inbox: VecDeque<Msg<T>>,
    inbox_capacity: Option<usize>,
}

impl<T: TracedMessage> Receiver<T> {
    #[allow(dead_code)]
    pub fn location(&self) -> Option<NVector<f64>> {
        let inner = self.0.borrow();
        inner.loc
    }

    pub fn recv(&mut self) -> Option<T> {
        let mut inner = self.0.borrow_mut();
        let msg = inner.inbox.pop_front();

        if let Some(msg) = msg {
            let mut kvs = msg.item.attrs();
            kvs.extend([
                kv("event.interaction.remote_nonce", msg.nonce),
                kv(
                    "event.interaction.remote_timeline_id",
                    AttrVal::TimelineId(Box::new(msg.sender)),
                ),
                kv("event.channel.recv", true),
            ]);
            MODALITY.emit_event(kvs);

            Some(msg.item)
        } else {
            None
        }
    }

    pub fn set_location(&mut self, loc: NVector<f64>) {
        let mut inner = self.0.borrow_mut();
        inner.loc = Some(loc);
    }

    pub fn with_location(mut self, loc: NVector<f64>) -> Self {
        self.set_location(loc);
        self
    }

    pub fn clear(&mut self) {
        let mut inner = self.0.borrow_mut();
        inner.inbox.clear();
    }
}

#[derive(Debug, Error)]
pub enum ChannelError {
    #[error("Queue full")]
    QueueFull,
}
