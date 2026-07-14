//! The protocol sec 7 hot loop as a mechanism: configure a [`Cycle`] once,
//! then [`Cycle::step`] forever -- GWRITE(HOLD) per group, one COMMIT so the
//! fleet applies in the same instant, then the telemetry GREAD. The chain is
//! the implicit ack: a rejected write surfaces as ALERT on that servo's
//! status. Pacing is the caller's; there is no timer here.

use osc_protocol::wire::{Id, ResultCode};

use crate::client::{Chain, Client};
use crate::error::Error;
use crate::pipe::Pipe;

/// One uniform GWRITE group: `count` bytes at `addr` for each listed target.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Group {
    pub addr: u16,
    pub count: u8,
    pub ids: Vec<Id>,
}

/// The read half of a step: a raw span, or a pre-configured profile slot
/// (sec 5.2) when the interesting registers don't sit contiguously.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Telemetry {
    Span { ids: Vec<Id>, addr: u16, count: u16 },
    Profile { ids: Vec<Id>, slot: u8 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Cycle {
    groups: Vec<Group>,
    telemetry: Telemetry,
}

impl Cycle {
    pub fn new(groups: Vec<Group>, telemetry: Telemetry) -> Self {
        Self { groups, telemetry }
    }

    /// Payload slots of one step, group-major: `payloads[k]` feeds the k-th
    /// (group, id) pair in declaration order and must be `count` bytes wide.
    pub fn payload_slots(&self) -> usize {
        self.groups.iter().map(|g| g.ids.len()).sum()
    }

    /// One hot-loop exchange. `payloads` is group-major (see
    /// [`Self::payload_slots`]); a wrong slot count or slice width is a
    /// caller bug and answers `range` without touching the wire.
    pub async fn step<P: Pipe>(
        &self,
        c: &mut Client<P>,
        payloads: &[&[u8]],
    ) -> Result<Chain, Error> {
        if payloads.len() != self.payload_slots() {
            return Err(Error::Servo(ResultCode::Range));
        }
        let mut next = payloads;
        for g in &self.groups {
            let (batch, rest) = next.split_at(g.ids.len());
            next = rest;
            if batch.iter().any(|d| d.len() != g.count as usize) {
                return Err(Error::Servo(ResultCode::Range));
            }
            let pairs: Vec<(Id, &[u8])> =
                g.ids.iter().copied().zip(batch.iter().copied()).collect();
            c.gwrite_hold(g.addr, g.count, &pairs).await?;
        }
        c.commit().await?;
        match &self.telemetry {
            Telemetry::Span { ids, addr, count } => c.gread(ids, *addr, *count).await,
            Telemetry::Profile { ids, slot } => c.gread_profile(ids, *slot).await,
        }
    }
}
