//! HIGH↔LOW dispatch handoff (osc-servo-transport §6, A3(b)): one work slot
//! each way. The transport at HIGH publishes an instruction frame as a
//! [`DispatchJob`] — write-class frames ahead of their CRC verdict (the
//! dispatch spine; effects stage, the verdict at adoption gates them),
//! verdict-first frames (COMMIT/MGMT and busy-pipeline fallbacks) after a
//! passing check. The [`DispatchConsumer`] at LOW decodes and dispatches it,
//! records the reply verbs into a [`ReplyRecord`], and wakes HIGH to adopt —
//! resolve the verdict, stage the reply into the TX engine, sequence the
//! chain, apply deferred config. No hardware handle crosses the boundary:
//! the CRC engine, TX engine, and deadline compare stay HIGH-exclusive, and
//! the consumer touches only ring bytes, the control table, and this cell.
//!
//! Backpressure is the slot itself: while it is occupied the framer holds
//! position and the RX ring absorbs the backlog (A2 — the ring is the
//! queue), so the handoff can never drop a frame.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU8, Ordering, compiler_fence};

use osc_core::traits::{Dispatch, Dispatched, Reply, SendError, Status};
use osc_core::{BaudRate, BootMode};
use osc_protocol::wire::ResultCode;

use super::decode::{Decoded, decode};
use super::frame_view;
use crate::traits::bus::{RxRing, SequenceWake};

/// An instruction frame awaiting decode + dispatch at LOW.
#[derive(Copy, Clone)]
pub struct DispatchJob {
    pub anchor: u16,
    pub footprint: u16,
    /// The framer's packet-end estimate — T_turn's wire anchor (§7).
    pub packet_end: u32,
    /// The servo's wire ID at publish time (ID changes land between frames).
    pub id: u8,
}

const NO_JOB: DispatchJob = DispatchJob {
    anchor: 0,
    footprint: 0,
    packet_end: 0,
    id: 0,
};

/// A completed dispatch awaiting adoption at HIGH. `staged` mirrors whether
/// the dispatcher sent a status; the payload is a raw span because every
/// non-empty core reply payload references control-table storage (§4.2
/// zero-copy contract — the same stability the TX engine's external arms
/// rely on), stable until adoption snapshots it.
#[derive(Copy, Clone)]
pub struct ReplyRecord {
    pub staged: bool,
    /// The dispatcher staged a table effect — the adopter owes the verdict
    /// ([`Dispatch::commit`] / [`Dispatch::revert`]).
    pub pending: bool,
    pub slot: u8,
    pub result: ResultCode,
    pub alert: bool,
    pub payload: *const u8,
    pub payload_len: u16,
    pub packet_end: u32,
    pub id_change: Option<u8>,
    pub baud_change: Option<BaudRate>,
    pub reboot: Option<BootMode>,
    pub response_deadline_us: Option<u16>,
}

impl ReplyRecord {
    const fn empty(packet_end: u32) -> Self {
        Self {
            staged: false,
            pending: false,
            slot: 0,
            result: ResultCode::Ok,
            alert: false,
            payload: core::ptr::null(),
            payload_len: 0,
            packet_end,
            id_change: None,
            baud_change: None,
            reboot: None,
            response_deadline_us: None,
        }
    }
}

/// Slot states. Each transition has exactly one writer: the producer (HIGH)
/// owns Empty→Job and Reply→Empty, the consumer (LOW) owns Job→Working and
/// Working→Reply — so plain load/store with compiler fences suffices on a
/// single hart (no RMW; rv32ec has no A extension).
const EMPTY: u8 = 0;
const JOB: u8 = 1;
const WORKING: u8 = 2;
const REPLY: u8 = 3;

pub struct Handoff {
    state: AtomicU8,
    job: UnsafeCell<DispatchJob>,
    reply: UnsafeCell<ReplyRecord>,
}

// SAFETY: single-hart SPSC — the state machine above gives every cell field
// exactly one accessor per state, and the compiler fences order the data
// against the state stores.
unsafe impl Sync for Handoff {}

impl Default for Handoff {
    fn default() -> Self {
        Self::new()
    }
}

impl Handoff {
    pub const fn new() -> Self {
        Self {
            state: AtomicU8::new(EMPTY),
            job: UnsafeCell::new(NO_JOB),
            reply: UnsafeCell::new(ReplyRecord::empty(0)),
        }
    }

    /// Producer (HIGH): the slot can take a job — nothing outstanding in
    /// either direction. The framer holds position while this is false.
    pub fn idle(&self) -> bool {
        self.state.load(Ordering::Relaxed) == EMPTY
    }

    /// Producer (HIGH): publish a job into an idle slot.
    pub fn publish(&self, job: DispatchJob) {
        debug_assert!(self.idle(), "publish into an occupied slot");
        unsafe { *self.job.get() = job };
        compiler_fence(Ordering::Release);
        self.state.store(JOB, Ordering::Relaxed);
    }

    /// Producer (HIGH): collect the consumer's completed record, freeing the
    /// slot.
    pub fn take_reply(&self) -> Option<ReplyRecord> {
        if self.state.load(Ordering::Relaxed) != REPLY {
            return None;
        }
        compiler_fence(Ordering::Acquire);
        let rec = unsafe { *self.reply.get() };
        compiler_fence(Ordering::Release);
        self.state.store(EMPTY, Ordering::Relaxed);
        Some(rec)
    }

    /// Consumer (LOW): claim the published job. A spurious wake (the lane
    /// vectors can pend redundantly) sees Working/Reply and no-ops.
    pub fn take_job(&self) -> Option<DispatchJob> {
        if self.state.load(Ordering::Relaxed) != JOB {
            return None;
        }
        compiler_fence(Ordering::Acquire);
        let job = unsafe { *self.job.get() };
        self.state.store(WORKING, Ordering::Relaxed);
        Some(job)
    }

    /// Consumer (LOW): publish the completed record.
    pub fn publish_reply(&self, rec: ReplyRecord) {
        debug_assert!(
            self.state.load(Ordering::Relaxed) == WORKING,
            "reply without a claimed job"
        );
        unsafe { *self.reply.get() = rec };
        compiler_fence(Ordering::Release);
        self.state.store(REPLY, Ordering::Relaxed);
    }
}

/// Records the dispatcher's reply verbs at LOW for adoption at HIGH.
struct RecordingReply<'a> {
    rec: &'a mut ReplyRecord,
}

impl Reply for RecordingReply<'_> {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError> {
        self.rec.staged = true;
        self.rec.result = status.result;
        self.rec.alert = status.alert;
        self.rec.payload = status.data.as_ptr();
        self.rec.payload_len = status.data.len() as u16;
        Ok(())
    }

    fn stage_id(&mut self, id: u8) {
        self.rec.id_change = Some(id);
    }

    fn stage_baud(&mut self, baud: BaudRate) {
        self.rec.baud_change = Some(baud);
    }

    fn set_response_deadline(&mut self, us: u16) {
        self.rec.response_deadline_us = Some(us);
    }

    fn stage_reboot(&mut self, mode: BootMode) {
        self.rec.reboot = Some(mode);
    }
}

/// The LOW half of the transport: one job at a time — decode from the ring,
/// dispatch into the session, record the reply, wake HIGH for adoption.
/// Runs in the chip's LOW vectors (and the sim's consumer context), so the
/// motor kernel and the dispatcher serialize by priority class — a table
/// write can no longer preempt a kernel tick mid-field.
pub struct DispatchConsumer<R: RxRing, W: SequenceWake> {
    handoff: &'static Handoff,
    ring: R,
    wake: W,
}

impl<R: RxRing, W: SequenceWake> DispatchConsumer<R, W> {
    pub fn new(handoff: &'static Handoff, ring: R, wake: W) -> Self {
        Self {
            handoff,
            ring,
            wake,
        }
    }

    /// LOW vector body. Returns whether a job was processed. A frame the
    /// decode disowns (a group op that doesn't list us — unicast to others
    /// is screened at HIGH) publishes an empty record: adoption just frees
    /// the slot.
    pub fn process<D: Dispatch>(&mut self, d: &mut D) -> bool {
        let Some(job) = self.handoff.take_job() else {
            return false;
        };
        let mut rec = ReplyRecord::empty(job.packet_end);
        let frame = frame_view(self.ring.bytes(), job.anchor, job.footprint);
        if let Decoded::Own(req, ctx, slot) = decode(frame, job.id) {
            rec.slot = slot;
            let mut reply = RecordingReply { rec: &mut rec };
            rec.pending = matches!(d.dispatch(req, ctx, &mut reply), Dispatched::Pending);
        }
        self.handoff.publish_reply(rec);
        self.wake.reply_ready();
        true
    }
}
