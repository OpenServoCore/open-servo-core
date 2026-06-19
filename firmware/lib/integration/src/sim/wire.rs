use std::cmp::{Ordering, Reverse};
use std::collections::BinaryHeap;

use crate::sim::{DeviceId, SimTime};

pub struct Wire {
    subscribers: Vec<DeviceId>,
    delivery_queue: BinaryHeap<Reverse<EdgeArrival>>,
    active_source: Option<(DeviceId, SimTime)>,
    idle_since: SimTime,
    seq: SeqCounter,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct Seq(u64);

#[derive(Default)]
struct SeqCounter(u64);

impl SeqCounter {
    fn next(&mut self) -> Seq {
        let s = Seq(self.0);
        self.0 += 1;
        s
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct EdgeArrival {
    at: SimTime,
    seq: Seq,
    target: DeviceId,
    rising: bool,
}

impl Ord for EdgeArrival {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.at, self.seq).cmp(&(other.at, other.seq))
    }
}

impl PartialOrd for EdgeArrival {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Wire {
    pub fn new() -> Self {
        Self {
            subscribers: Vec::new(),
            delivery_queue: BinaryHeap::new(),
            active_source: None,
            idle_since: SimTime::ZERO,
            seq: SeqCounter::default(),
        }
    }

    pub fn subscribe(&mut self, id: DeviceId) {
        self.subscribers.push(id);
    }

    pub fn schedule_edge(&mut self, source: DeviceId, at: SimTime, rising: bool) {
        log::trace!(
            "wire: schedule_edge source={:?} at={:?} rising={} active_source={:?}",
            source,
            at,
            rising,
            self.active_source
        );
        if !rising {
            // Any falling edge while the line is already low is a physics
            // violation: the wire is single-level at any instant, regardless
            // of source. Catches both cross-source contention and same-source
            // overlap (two byte streams from one peripheral).
            if let Some((active, since)) = self.active_source {
                panic!(
                    "wire collision at {:?}: {:?} held line since {:?}, {:?} tried to drive",
                    at, active, since, source
                );
            }
            self.active_source = Some((source, at));
        } else {
            self.active_source = None;
            self.idle_since = at;
        }

        for &sub in &self.subscribers {
            if sub == source {
                continue;
            }
            let seq = self.seq.next();
            self.delivery_queue.push(Reverse(EdgeArrival {
                at,
                seq,
                target: sub,
                rising,
            }));
        }
    }

    pub fn idle_since(&self) -> SimTime {
        self.idle_since
    }

    pub fn next_delivery_time(&self) -> Option<SimTime> {
        self.delivery_queue.peek().map(|r| r.0.at)
    }

    pub fn has_no_pending(&self) -> bool {
        self.delivery_queue.is_empty()
    }

    /// Pop every delivery scheduled at exactly `t`. The conductor routes
    /// each tuple by calling `receive_edge` on the target. Returns an
    /// empty vec if no delivery lands at `t`.
    pub fn take_pending(&mut self, t: SimTime) -> Vec<(DeviceId, SimTime, bool)> {
        let mut out = Vec::new();
        while let Some(top) = self.delivery_queue.peek() {
            if top.0.at != t {
                break;
            }
            let arrival = self.delivery_queue.pop().unwrap().0;
            log::trace!(
                "wire: deliver target={:?} at={:?} rising={}",
                arrival.target,
                arrival.at,
                arrival.rising
            );
            out.push((arrival.target, arrival.at, arrival.rising));
        }
        out
    }
}

impl Default for Wire {
    fn default() -> Self {
        Self::new()
    }
}
