//! The dispatch spine: resolver -> frame routing -> CRC verdict -> reply sequencing.

use osc_protocol::wire::{ENUM_REPLY_SLOTS, ResultCode};
use osc_servo_core::traits::{Dispatch, Dispatched, Request, RequestCtx};

use super::super::chain::ChainOut;
use super::super::decode::{Decoded, decode};
use super::super::frame_view;
use super::super::framer::{FrameSpan, FramerOut};
use super::reply::ReplyHandle;
use super::{Pending, ServoBus};
use crate::traits::bus::{Deadline, Providers, RxRing, tick_reached};

impl<P: Providers> ServoBus<P> {
    /// Drive the resolver as far as ring DATA allows (data-first from the
    /// stream): complete backlog frames process in-line with zero clock
    /// involvement; the frontier arms a deadline and returns. Bounded per
    /// wake -- past the bound the framer
    /// slot re-arms at `now` (pend-on-past re-entry) so the other mux slots
    /// are never starved by a deep backlog.
    pub(super) fn drive_framer<D: Dispatch>(&mut self, d: &mut D) {
        for _ in 0..super::FRAMES_PER_WAKE {
            let now = self.deadline.now();
            let out = self
                .framer
                .resolve(self.ring.bytes(), self.ring.cursor(), now, self.tpb);
            match out {
                FramerOut::None => {
                    self.framer_at = None;
                    return;
                }
                FramerOut::Wait(t) => {
                    self.framer_at = Some(t);
                    return;
                }
                FramerOut::Covered { span, end_due } => {
                    self.framer_at = Some(end_due);
                    self.route_frame(span, false, d);
                    return;
                }
                FramerOut::Frame(span) => {
                    self.on_frame_end(span, d);
                }
            }
        }
        self.framer_at = Some(self.deadline.now());
    }

    pub(super) fn route_chain(&mut self, mut out: ChainOut) {
        loop {
            match out {
                ChainOut::None => return,
                ChainOut::Wait(t) => {
                    // A wait tick already reached needs no scheduling round
                    // trip: consume the slot in place (high-baud grids and
                    // backlog replies are past-due by sequencing time, and
                    // an expired reclaim window is a reclaim by definition).
                    let now = self.deadline.now();
                    if !tick_reached(now, t) {
                        self.chain_at = Some(t);
                        return;
                    }
                    out = self.chain.on_deadline(now);
                }
                ChainOut::Trigger { predecessor_silent } => {
                    let over = predecessor_silent.then_some(ResultCode::PredecessorSilent);
                    self.tx.trigger(&mut self.crc, over);
                    return;
                }
            }
        }
    }

    /// Frame end (deadline B): a pending frame gets its verdict; anything else
    /// is a complete frame entering the spine ([`Self::route_frame`]).
    fn on_frame_end<D: Dispatch>(&mut self, span: FrameSpan, d: &mut D) {
        match self.pending.take() {
            Some(p) if p.anchor == span.anchor && p.footprint == span.footprint => {
                self.verify(p, d);
            }
            Some(_) => {
                // Defensive: a pending frame that isn't this one -- drop it. Any
                // dangling staged write is reclaimed by the dispatcher
                // auto-revert on the next dispatch.
                self.drop_staged();
                self.route_frame(span, true, d);
            }
            None => self.route_frame(span, true, d),
        }
    }

    /// The spine: dispatch a frame ahead of its CRC verdict when its effects
    /// can stage (sec 4). `complete` distinguishes a whole frame (backlog / fast
    /// path -- every byte incl. the CRC is ringed, so the verdict fires now)
    /// from a frontier frame at its covered checkpoint (the two CRC bytes still
    /// inbound -- the verdict is deferred to the frame end). A stageable op
    /// (ping/read/gread/write/gwrite) dispatches inline with the CRC feed
    /// chewing underneath; the verdict gates its effects -- SEND/DON'T-SEND of a
    /// staged reply, COMMIT/REVERT of a staged write. Verdict-first ops
    /// (COMMIT/MGMT, effects unstageable) and any frame overlapping a live
    /// reply pipeline check the CRC first, then dispatch. A frontier frame that
    /// lands in the verdict-first path just defers -- its frame end arrives here
    /// `complete`.
    fn route_frame<D: Dispatch>(&mut self, span: FrameSpan, complete: bool, d: &mut D) {
        let anchor = span.anchor;
        let footprint = span.footprint;
        let packet_end = span.packet_end;
        // Status frames only advance the snoop chain (sec 6) -- framing-level
        // truth, NO validation: the chain consumes nothing from the body, and
        // skipping the CRC keeps the snapshot buffer free while our own reply
        // streams from it. A frontier status defers to its frame end.
        if self.ring_inst(anchor).is_status() {
            if complete {
                let out = self.chain.on_status_end(packet_end);
                self.route_chain(out);
            }
            return;
        }
        // The spine runs only from an idle reply pipeline: superseding a live
        // chain or staged reply belongs AFTER the CRC gate (a garbled frame
        // must touch nothing, sec 5.3 L1), so those overlaps fall to verdict-first
        // below. The guard also keeps the CRC engine free through a pending
        // window -- chain/tx activate only at a verdict, so no trigger reset
        // can clobber a fed span.
        if !self.chain.active() && !self.tx.busy() && !self.verdict_first(anchor) {
            // Feed first so the engine chews the covered span under the
            // dispatch body; the verdict then finds it settled.
            self.crc_feed(anchor, footprint);
            let (staged, slot, table) =
                match self
                    .dispatch_decoded(anchor, footprint, |req, ctx, h| d.dispatch(req, ctx, h))
                {
                    Some((staged, slot, out)) => (staged, slot, matches!(out, Dispatched::Pending)),
                    // Skip (a group op not listing us): a bare verdict moves the
                    // ladder (complete); the frame end does it for a frontier.
                    None if complete => (false, 0, false),
                    None => return,
                };
            let p = Pending {
                anchor,
                footprint,
                packet_end,
                slot,
                staged,
                table,
            };
            if complete {
                self.verify(p, d);
            } else {
                self.pending = Some(p);
            }
            return;
        }
        // Verdict-first (only meaningful complete -- a frontier defers): the CRC
        // is checked before any effect. COMMIT/MGMT (unstageable), and frames
        // overlapping a live reply pipeline. A fail drops silently (sec 5.3 L1),
        // the hunt resuming one byte in (sec 3.3).
        if !complete {
            return;
        }
        if !self.crc_ok(anchor, footprint) {
            if !self.framer.probing() {
                self.crc_fails = self.crc_fails.wrapping_add(1);
            }
            let len = self.ring.bytes().len();
            self.framer.on_frame_rejected(anchor, len);
            return;
        }
        self.framer.on_frame_verified();
        self.drift_note_verified(anchor, footprint);
        // A fresh instruction supersedes any stale, not-yet-streaming reply --
        // post-verdict, so a garbled frame touched nothing.
        self.chain.reset();
        self.chain_at = None;
        if self.tx.staged() {
            self.tx.abort();
        }
        // Dispatch inline -- the CRC already passed, so a staged table effect
        // commits directly and any reply sequences from the packet end. A frame
        // that decodes as another servo's touches nothing.
        let Some((staged, slot, out)) =
            self.dispatch_decoded(anchor, footprint, |req, ctx, h| d.dispatch(req, ctx, h))
        else {
            return;
        };
        if matches!(out, Dispatched::Pending) {
            let mut handle = self.reply_handle();
            d.commit(&mut handle);
        }
        if staged && self.tx.staged() {
            self.sequence_reply(slot, packet_end);
        }
    }

    /// Verify a pending frame's CRC and resolve the verdict. Pass -> commit a
    /// staged table effect (COMMIT) and sequence a staged reply (SEND); fail
    /// (or spin miss) -> revert the write (REVERT), drop the reply
    /// (DON'T-SEND), count, and rewind the ladder (sec 5.3 L1).
    #[cfg_attr(target_os = "none", unsafe(link_section = ".highcode"))]
    #[cfg_attr(target_os = "none", inline(never))]
    fn verify<D: Dispatch>(&mut self, p: Pending, d: &mut D) {
        if !self.crc_verify(p.anchor, p.footprint) {
            if p.table {
                d.revert();
            }
            self.drop_staged();
            if !self.framer.probing() {
                self.crc_fails = self.crc_fails.wrapping_add(1);
            }
            let len = self.ring.bytes().len();
            self.framer.on_frame_rejected(p.anchor, len);
            return;
        }
        self.framer.on_frame_verified();
        self.drift_note_verified(p.anchor, p.footprint);
        if p.table {
            let mut handle = self.reply_handle();
            d.commit(&mut handle);
        }
        // Sequence from the ENGINE's state, not the recorded flag: any path
        // that reclaimed the staged reply between dispatch and here would
        // otherwise arm the chain over an empty engine (ghost trigger).
        if p.staged && self.tx.staged() {
            self.sequence_reply(p.slot, p.packet_end);
        }
    }

    /// Sequence a staged reply onto the snoop chain. reply gap is a wire gap
    /// measured from the packet end (sec 7), not from this (later) verify wake --
    /// the estimate is the framer's, conservative by the drift adder.
    fn sequence_reply(&mut self, slot: u8, packet_end: u32) {
        // sec 9.2 slot delay: a collision-tolerant reply offsets its trigger by
        // (key ^ tick) & (SLOTS-1) byte-times. Twin matchers run
        // cycle-identical firmware and otherwise answer in unison -- and a
        // sub-bit-aligned superposition of near-equal frames reads back as
        // ONE clean frame instead of collision garble. The tick term re-draws
        // every exchange, so equal keys never stick.
        let gap = match self.tx.slot_key() {
            Some(key) => {
                let draw = (key ^ self.deadline.now() as u8) & (ENUM_REPLY_SLOTS - 1);
                self.reply_gap().wrapping_add(draw as u32 * self.tpb)
            }
            None => self.reply_gap(),
        };
        let out = self.chain.on_reply_staged(
            slot,
            packet_end,
            gap,
            self.reclaim(),
            self.frame_allowance(),
        );
        self.route_chain(out);
    }

    /// Drop a not-yet-streaming staged reply and any chain sequencing it began.
    fn drop_staged(&mut self) {
        if self.tx.staged() {
            self.tx.abort();
        }
        self.chain.reset();
        self.chain_at = None;
    }

    /// View the frame as up to two ring segments (one span unless it wraps the
    /// seam), decode, and run `f` over disjoint borrows (driver-pattern sec 4.3):
    /// the decoded request borrows the ring while `f` stages a reply through the
    /// [`ReplyHandle`]. Returns `(staged, slot, f-result)`, or `None` for a frame
    /// that isn't ours.
    fn dispatch_decoded<T>(
        &mut self,
        anchor: u16,
        footprint: u16,
        f: impl FnOnce(Request<'_>, RequestCtx, &mut ReplyHandle<'_, P::Tx, P::Crc>) -> T,
    ) -> Option<(bool, u8, T)> {
        let frame = frame_view(self.ring.bytes(), anchor, footprint);
        let (req, ctx, slot) = match decode(frame, self.id) {
            Decoded::Own(req, ctx, slot) => (req, ctx, slot),
            _ => return None,
        };
        // sec 9.2: an ENUM reply's staged wire effect is collision-tolerant --
        // its job is to collide with peer matchers (the on_break kill site
        // skips it, and staging keys its slot delay off the UID payload).
        let tolerant = matches!(req, Request::Enumerate { .. });
        // Disjoint field borrows: `frame`/`req` hold `&self.ring`; the handle
        // takes the reply-staging fields mutably.
        let mut handle = ReplyHandle {
            tx: &mut self.tx,
            crc: &mut self.crc,
            id: &mut self.id,
            pending_id: &mut self.pending_id,
            pending_baud: &mut self.pending_baud,
            pending_reboot: &mut self.pending_reboot,
            pending_cal: &mut self.clock.pending_cal,
            response_deadline_us: &mut self.response_deadline_us,
            staged: false,
            tolerant,
        };
        let out = f(req, ctx, &mut handle);
        Some((handle.staged, slot, out))
    }
}
