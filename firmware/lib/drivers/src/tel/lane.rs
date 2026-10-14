use core::sync::atomic::{Ordering, compiler_fence};

use osc_protocol::wire::STREAM_FLAG_LAST;
use osc_servo_core::{ControlTableCell, TelStream};

use super::*;
use crate::bench::{self, TelStep};
use crate::traits::bus::{CrcEngine, Providers, RxRing, TxWire};

/// Frame `done`'s place on the lane.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(super) enum Lane {
    Idle,
    Break,
    Stream,
}

/// Kernel-side half: the fast tick's [`TelStream`] sink, the row copies,
/// the frame heads and the TX lane. All fields but `ch` are kernel-private.
pub struct TelLane<P: Providers> {
    pub(super) ch: &'static TelChannel,
    pub(super) wire: P::Tx,
    pub(super) crc: P::Crc,
    pub(super) ring: P::Ring,
    /// The `bench-telcost` stamps' time base.
    #[cfg_attr(not(feature = "bench-telcost"), allow(dead_code))]
    pub(super) clock: P::Deadline,
    /// This burst stamps its steps.
    #[cfg(feature = "bench-telcost")]
    pub(super) stamp: bool,
    pub(super) table: &'static ControlTableCell,
    /// The arm this burst runs under (`arm_seq` at pickup).
    pub(super) epoch: u8,
    pub(super) id: u8,
    pub(super) plan: RowPlan,
    pub(super) remaining: u16,
    /// Frames begun: the filling one carries `seq`. Frames before `done`
    /// have left the wire, which may include the filling one: a frame the
    /// wire outran (late ticks) ships stale and fails its CRC, and its rows
    /// land on until its bank.
    pub(super) seq: u8,
    pub(super) done: u8,
    /// The filling frame: where its next row lands (exposed address, as
    /// `src`), rows landed and planned, and the rows a cut-through start
    /// waits past.
    pub(super) wr: usize,
    pub(super) n: u8,
    pub(super) rows: u8,
    pub(super) lead: u8,
    /// Frame `done` on the lane: its phase, the covered bytes fed to the
    /// CRC engine, and whether its CRC still owes a feed or the patch.
    pub(super) phase: Lane,
    pub(super) fed: usize,
    pub(super) crc_due: bool,
    /// This tick landed a head or a bank, or holds: the lane leaves it be.
    pub(super) landed: bool,
}

impl<P: Providers> TelLane<P> {
    /// The kernel owns the TX lane.
    pub fn active(&self) -> bool {
        self.ch.active()
    }

    /// Nothing of the posted burst is left for the lane: every frame has
    /// left the wire.
    pub fn idle(&self) -> bool {
        // SAFETY: bus-written mailbox seq, volatile-read (type doc).
        let seq = unsafe { self.ch.arm_seq.get().read_volatile() };
        seq == self.epoch && self.remaining == 0 && self.done == self.seq
    }

    #[cold]
    #[inline(never)]
    #[rustc_align(4)]
    fn pickup(&mut self, seq: u8) {
        let ch = self.ch;
        compiler_fence(Ordering::Acquire);
        // SAFETY: the bus wrote the mailbox before its seq and writes it
        // only while the lane is its own; the bus never preempts this side.
        self.plan = unsafe {
            self.remaining = ch.arm_count.get().read_volatile();
            self.id = ch.arm_id.get().read_volatile();
            *ch.arm_row.get()
        };
        // A plan of no row (`RowPlan::of`) streams nothing.
        if self.plan.row == 0 {
            self.remaining = 0;
        }
        self.epoch = seq;
        self.seq = 0;
        self.done = 0;
        self.n = 0;
        self.phase = Lane::Idle;
        if self.remaining == 0 {
            // The bus lends only for a nonzero admitted arm; an empty one
            // still hands the lane back.
            self.publish_last();
        }
    }

    /// Frame `done` may start: it is banked, or its rows passed the lead.
    #[inline(always)]
    fn startable(&self) -> bool {
        let banked = self.seq.wrapping_sub(self.done) as i8;
        banked > 0 || banked == 0 && self.n > self.lead
    }

    /// Frame `done` on the lane: the CRC engine's remaining feed or the
    /// patch, else its arm once the break has gone out. A result not in
    /// yet lets the arm go first, the patch retrying while the frame
    /// streams: a sick engine costs that frame its CRC, never the lane.
    #[inline(never)]
    #[rustc_align(4)]
    fn service(&mut self) {
        let f = self.done;
        // SAFETY: kernel side (fn doc of `image`).
        let img = unsafe { self.ch.image(f) };
        let cov = covered(img);
        if self.crc_due {
            if self.fed < cov {
                return self.feed(img, cov);
            }
            if let Some(v) = self.crc.result() {
                let slot = img
                    .get_mut(cov..)
                    .and_then(|s| s.first_chunk_mut::<CRC_LEN>());
                if let Some(slot) = slot {
                    *slot = v.to_le_bytes();
                }
                self.crc_due = false;
                return;
            }
        }
        if self.phase != Lane::Break || !self.wire.idle() {
            return;
        }
        if let Some(frame) = img.get(ID_AT..cov + CRC_LEN) {
            self.wire.stream(frame);
            self.phase = Lane::Stream;
            if img[FLAGS_AT] & STREAM_FLAG_LAST != 0 {
                self.publish_last();
            }
        }
    }

    /// Start frame `done`: reset the CRC engine, feed what is written, and
    /// break.
    #[inline(never)]
    #[rustc_align(4)]
    fn start(&mut self) {
        let f = self.done;
        // SAFETY: kernel side (fn doc of `image`).
        let img = unsafe { self.ch.image(f) };
        let banked = f != self.seq;
        let written = if banked {
            covered(img)
        } else {
            self.wr - img.as_ptr().addr()
        };
        self.crc.reset();
        self.fed = 0;
        self.feed(img, written);
        self.crc_due = banked;
        self.wire.break_start();
        self.phase = Lane::Break;
    }

    /// Feed the CRC engine the image's covered bytes from `fed` to the
    /// even `to`. One call site for the engine's arm: inlined twice it
    /// doubles the lane's flash.
    #[inline(never)]
    #[rustc_align(4)]
    fn feed(&mut self, img: &[u8; IMAGE_LEN], to: usize) {
        if let Some(span) = img.get(self.fed..to) {
            self.crc.feed(span);
        }
        self.fed = to;
    }

    /// The burst's LAST frame is streaming, and the receiver is deaf to the
    /// wire under its own TX: the ring cursor is still, and the bus
    /// resumes its ladder there.
    fn publish_last(&mut self) {
        let cursor = self.ring.cursor();
        // SAFETY: kernel-written fields (type doc); the fence orders the
        // cursor ahead of the epoch the bus keys on.
        unsafe {
            self.ch.resume.get().write_volatile(cursor);
            compiler_fence(Ordering::Release);
            self.ch.last_out.get().write_volatile(self.epoch);
        }
    }
}

impl<P: Providers> TelStream for TelLane<P> {
    /// The burst starts at the arm's pickup: a held start leaves the arm in
    /// the mailbox, and the lane idle, for the next tick.
    #[inline(always)]
    fn on_publish(&mut self, hold_start: impl FnOnce() -> bool) {
        let top = bench::tel_stamp(&self.clock);
        self.publish(hold_start);
        bench::tel_hook(&self.clock, top);
    }

    /// A stage tick's step waits a tick, as on a head's or a bank's. A hold
    /// while the bus owns the lane outlives it harmlessly: a burst's first
    /// tick lands a head.
    #[inline(always)]
    fn hold(&mut self) {
        self.landed = true;
    }

    /// The TX lane, inline: the tick that owes it nothing pays a flag test
    /// and the TX poll. The lane takes one step a tick at most - each is a
    /// few hundred HCLK, and two would not fit beside the dearest phases
    /// (budget `tel_lane`) - and none on a tick that landed a head or a
    /// bank, or holds. Frame `done`'s CRC step comes first (the patch lands
    /// ahead of an arm), then its arm, or the start of the next frame. A
    /// frame's end is a few stores, so the next can start in the same tick:
    /// a tick apart, the wire would idle one tick a frame.
    #[inline(always)]
    fn on_tick_end(&mut self) {
        let top = bench::tel_stamp(&self.clock);
        if self.ch.active() {
            if self.stamping() {
                let t = bench::tel_stamp(&self.clock);
                let s = self.lane();
                bench::tel_step(&self.clock, t, s);
            } else {
                self.lane();
            }
        }
        bench::tel_hook(&self.clock, top);
    }
}

impl<P: Providers> TelLane<P> {
    /// [`TelStream::on_publish`] inside its hook stamps.
    #[inline(always)]
    fn publish(&mut self, hold_start: impl FnOnce() -> bool) {
        if !self.ch.active() {
            return;
        }
        // SAFETY: bus-written mailbox seq, volatile-read (type doc).
        let seq = unsafe { self.ch.arm_seq.get().read_volatile() };
        if seq != self.epoch {
            if hold_start() {
                return;
            }
            let t = bench::tel_stamp(&self.clock);
            self.pickup(seq);
            bench::tel_step(&self.clock, t, TelStep::Pickup);
            #[cfg(feature = "bench-telcost")]
            {
                self.stamp = bench::tel_stamping();
            }
        }
        if self.remaining != 0 {
            if self.stamping() {
                let s = self.row_step();
                let t = bench::tel_stamp(&self.clock);
                self.row();
                bench::tel_step(&self.clock, t, s);
            } else {
                self.row();
            }
        }
    }

    /// This burst stamps its steps; never without `bench-telcost`.
    #[inline(always)]
    fn stamping(&self) -> bool {
        #[cfg(feature = "bench-telcost")]
        {
            self.stamp
        }
        #[cfg(not(feature = "bench-telcost"))]
        false
    }

    /// The step the tick's row takes, from the frame's rows landed.
    #[inline(always)]
    pub(super) fn row_step(&self) -> TelStep {
        if self.n == 0 {
            TelStep::Head
        } else if self.n + 1 == self.rows {
            TelStep::Bank
        } else {
            TelStep::Row
        }
    }

    /// The lane's step of an active tick ([`TelStream::on_tick_end`]).
    #[inline(always)]
    pub(super) fn lane(&mut self) -> TelStep {
        if core::mem::take(&mut self.landed) {
            return TelStep::Poll;
        }
        match self.phase {
            Lane::Idle => {}
            // A frame off the wire owes nothing more, a CRC step included.
            Lane::Stream if self.wire.idle() => {
                self.done = self.done.wrapping_add(1);
                self.phase = Lane::Idle;
                self.crc_due = false;
            }
            _ if self.crc_due => {
                self.service();
                return TelStep::Service;
            }
            Lane::Break if self.wire.idle() => {
                self.service();
                return TelStep::Service;
            }
            _ => return TelStep::Poll,
        }
        if self.startable() {
            self.start();
            return TelStep::Start;
        }
        TelStep::Poll
    }
}
