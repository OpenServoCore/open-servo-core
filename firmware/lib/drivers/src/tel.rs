//! TEL burst lane (protocol sec 5.6): while a burst streams, the kernel
//! fast tick (PFIC HIGH) owns the bus TX lane and is the transmitter. Each
//! row lands at its final wire offset in one of two frame images; a frame
//! breaks once its lead row has landed and its DMA reads the rows behind
//! it as the wire reaches them, the CRC engine fed by the producer and the
//! value patched in before the DMA gets there. No TX interrupt and no
//! bus-level body runs for the burst: the bus lends the lane at the arming
//! ack's release (at the arm itself when no ack follows) and takes it back
//! in its next body once the LAST frame has left the wire.
//!
//! Cross-side traffic is single-writer-per-field volatile load/store plus
//! compiler fences (single core, no atomic RMW on rv32ec). The bus writes
//! the arm mailbox only while the lane is its own, then `active`; the
//! kernel, which the bus never preempts, takes the arm whole.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::{Ordering, compiler_fence};

use osc_protocol::crc::osc_crc_byte;
use osc_protocol::frame::Header;
use osc_protocol::wire::{self, CHAR_BIT_TIMES, Inst, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::budget::TICK_HZ;
use osc_servo_core::tel::{
    FLAG_LAST, RowPlan, STREAM_HDR, STREAM_PAYLOAD_MAX, STREAM_SAMPLES_MAX, STREAM_TRAILER,
    TelSample, TelStream, encode_stream_head, encode_stream_trailer,
};

use crate::traits::bus::{CrcEngine, Providers, RxRing, TxWire};

const CRC_LEN: usize = core::mem::size_of::<u16>();

/// A frame image at its wire offsets: ALIGN, ID, LEN, INST, the payload,
/// the CRC (sec 3.1). The DMA streams ID..CRC.
pub const IMAGE_LEN: usize = Header::SIZE + STREAM_PAYLOAD_MAX + CRC_LEN;

const ID_AT: usize = 1;
const LEN_AT: usize = 2;
const INST_AT: usize = 3;
const FLAGS_AT: usize = Header::SIZE + 1;
const ROWS_AT: usize = Header::SIZE + STREAM_HDR;

/// Ticks the start rule leaves between the bank and the DMA reaching the
/// CRC. The lane skips the bank's tick and the next frame's head tick,
/// feeds on the one after and patches on the next: three ticks of slack
/// remain, and the last row's read has at least as many. Late or lost
/// ticks inside them still ship a whole frame (DES
/// `tel_kernel_lane_never_underruns_at_budget`); more fail its CRC (DES
/// `tel_lost_ticks_mid_frame_fail_the_crc_and_leave_a_seq_hole`).
const LEAD_MARGIN_TICKS: u32 = 5;

/// Control ticks per wire byte at `baud`, Q16, rounded down so the lead
/// rounds late. Each arm folds to a literal: no run-time division.
fn ticks_per_byte_q16(baud: BaudRate) -> u32 {
    const fn q(baud_hz: u32) -> u32 {
        ((((CHAR_BIT_TIMES * TICK_HZ) as u64) << 16) / baud_hz as u64) as u32
    }
    match baud {
        BaudRate::B500000 => const { q(500_000) },
        BaudRate::B1000000 => const { q(1_000_000) },
        BaudRate::B2000000 => const { q(2_000_000) },
        BaudRate::B3000000 => const { q(3_000_000) },
    }
}

/// Even base: the CRC feed reads halfwords from it (F12).
#[repr(C, align(2))]
struct Image([u8; IMAGE_LEN]);

/// The image's covered length (ALIGN..trailer), from its LEN, never past
/// the CRC slot.
#[inline(always)]
fn covered(img: &[u8; IMAGE_LEN]) -> usize {
    wire::covered_len(img[LEN_AT]).min(IMAGE_LEN - CRC_LEN)
}

/// The shared storage. Field writers: the images, `drops`, `last_out` and
/// `resume` belong to the kernel side ([`TelFeed`]); the mailbox and
/// `active` to the bus side ([`TelDrain`]).
pub struct TelChannel {
    images: [SyncUnsafeCell<Image>; 2],
    /// The kernel owns the TX lane.
    active: SyncUnsafeCell<bool>,
    plan: SyncUnsafeCell<RowPlan>,
    arm_mask: SyncUnsafeCell<u16>,
    arm_count: SyncUnsafeCell<u16>,
    arm_id: SyncUnsafeCell<u8>,
    arm_rate: SyncUnsafeCell<BaudRate>,
    /// Mailbox epoch, written last, never 0.
    arm_seq: SyncUnsafeCell<u8>,
    /// The epoch whose LAST frame is on the wire, and the RX ring cursor
    /// read while it streamed.
    last_out: SyncUnsafeCell<u8>,
    resume: SyncUnsafeCell<u16>,
    /// Rows dropped while both images held unfinished frames, monotonic
    /// wrapping.
    drops: SyncUnsafeCell<u16>,
}

impl TelChannel {
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            images: [const { SyncUnsafeCell::new(Image([0; IMAGE_LEN])) }; 2],
            active: SyncUnsafeCell::new(false),
            plan: SyncUnsafeCell::new(RowPlan::EMPTY),
            arm_mask: SyncUnsafeCell::new(0),
            arm_count: SyncUnsafeCell::new(0),
            arm_id: SyncUnsafeCell::new(0),
            arm_rate: SyncUnsafeCell::new(BaudRate::RESCUE),
            arm_seq: SyncUnsafeCell::new(0),
            last_out: SyncUnsafeCell::new(0),
            resume: SyncUnsafeCell::new(0),
            drops: SyncUnsafeCell::new(0),
        }
    }

    /// The kernel owns the TX lane. Any context: the bus is its only
    /// writer.
    pub fn active(&self) -> bool {
        // SAFETY: single-byte volatile read of a bus-written flag.
        unsafe { self.active.get().read_volatile() }
    }

    /// Rows dropped against full images, monotonic wrapping. Any context:
    /// the kernel side is the counter's only writer.
    pub fn drops(&self) -> u16 {
        // SAFETY: single-word volatile read of a kernel-written counter.
        unsafe { self.drops.get().read_volatile() }
    }

    /// Split into the two halves over the lane's wire, CRC engine and RX
    /// ring handles. Call once at bringup: a second feed or drain breaks
    /// the single-writer discipline.
    pub fn split<P: Providers>(
        &'static self,
        wire: P::Tx,
        crc: P::Crc,
        ring: P::Ring,
    ) -> (TelFeed<P>, TelDrain) {
        (
            TelFeed {
                ch: self,
                wire,
                crc,
                ring,
                epoch: 0,
                mask: 0,
                plan: RowPlan::EMPTY,
                id: 0,
                tpb_q16: 0,
                remaining: 0,
                seq: 0,
                done: 0,
                at: ROWS_AT,
                n: 0,
                rows: 0,
                lead: 0,
                valid: 0,
                alert: false,
                phase: Phase::Idle,
                fed: 0,
                crc_due: false,
                landed: false,
            },
            TelDrain { ch: self },
        )
    }

    /// Frame `f`'s image.
    ///
    /// # Safety
    /// Kernel side only: the images are its own, and the DMA only reads.
    #[allow(clippy::mut_from_ref)]
    unsafe fn image(&self, f: u8) -> &mut [u8; IMAGE_LEN] {
        // SAFETY: see fn doc.
        unsafe { &mut (*self.images[(f & 1) as usize].get()).0 }
    }
}

/// Frame `done`'s place on the lane.
#[derive(Copy, Clone, PartialEq, Eq)]
enum Phase {
    Idle,
    Break,
    Stream,
}

/// Kernel-side half: the fast tick's [`TelStream`] sink, the incremental
/// frame encoder and the TX lane. All fields but `ch` are kernel-private.
pub struct TelFeed<P: Providers> {
    ch: &'static TelChannel,
    wire: P::Tx,
    crc: P::Crc,
    ring: P::Ring,
    /// The arm this burst runs under (`arm_seq` at pickup).
    epoch: u8,
    mask: u16,
    plan: RowPlan,
    id: u8,
    tpb_q16: u32,
    remaining: u16,
    /// Frames begun: the filling one carries `seq`. Frames before `done`
    /// have left the wire, which may include the filling one: a frame the
    /// wire outran (late ticks) ships stale and fails its CRC, and its rows
    /// land on until its bank.
    seq: u8,
    done: u8,
    /// The filling frame: write offset, rows landed and planned, the rows
    /// a cut-through start waits past, the valid bitmap and the fault OR.
    at: usize,
    n: u8,
    rows: u8,
    lead: u8,
    valid: u32,
    alert: bool,
    /// Frame `done` on the lane: its phase, the covered bytes fed to the
    /// CRC engine, and whether its CRC still owes a feed or the patch.
    phase: Phase,
    fed: usize,
    crc_due: bool,
    /// This tick landed a head or a trailer: the lane leaves it be.
    landed: bool,
}

impl<P: Providers> TelFeed<P> {
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
    fn pickup(&mut self, seq: u8) {
        let ch = self.ch;
        compiler_fence(Ordering::Acquire);
        // SAFETY: the bus wrote the mailbox before its seq and writes it
        // only while the lane is its own; the bus never preempts this side.
        unsafe {
            self.plan = ch.plan.get().read_volatile();
            self.mask = ch.arm_mask.get().read_volatile();
            self.remaining = ch.arm_count.get().read_volatile();
            self.id = ch.arm_id.get().read_volatile();
            self.tpb_q16 = ticks_per_byte_q16(ch.arm_rate.get().read_volatile());
        }
        if self.plan.is_empty() {
            self.remaining = 0;
        }
        self.epoch = seq;
        self.seq = 0;
        self.done = 0;
        self.n = 0;
        self.phase = Phase::Idle;
        if self.remaining == 0 {
            // The bus lends only for a nonzero admitted arm; an empty one
            // still hands the lane back.
            self.publish_last();
        }
    }

    /// Row 0 of frame `seq`: header, head, and the start rule's lead.
    fn head(&mut self, img: &mut [u8; IMAGE_LEN], s: &TelSample) {
        let rows = (self.remaining as usize).min(STREAM_SAMPLES_MAX);
        let cov = ROWS_AT + rows * self.plan.len() + STREAM_TRAILER;
        img[ID_AT] = self.id;
        img[LEN_AT] = wire::len_for((cov - Header::SIZE) as u8);
        img[INST_AT] = Inst::status(ResultCode::Stream, s.fault).0;
        let last = rows == self.remaining as usize;
        encode_stream_head(self.seq, last, s.tick, &mut img[Header::SIZE..]);
        // The DMA reaches the CRC (cov - 2) byte-times after the arm, a
        // tick behind the break; the patch is a tick behind the bank.
        let ticks = ((cov - CRC_LEN) as u32 * self.tpb_q16) >> 16;
        self.lead = (rows as u32 - 1 + LEAD_MARGIN_TICKS).saturating_sub(ticks) as u8;
        self.rows = rows as u8;
        self.at = ROWS_AT;
        self.valid = 0;
        self.alert = false;
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
    fn service(&mut self) {
        let f = self.done;
        // SAFETY: kernel side (fn doc of `image`).
        let img = unsafe { self.ch.image(f) };
        let cov = covered(img);
        if self.crc_due {
            let even = cov & !1;
            if self.fed < even {
                return self.feed(img, even);
            }
            if let Some(v) = self.crc.result() {
                let v = match img.get(cov - 1) {
                    Some(&b) if cov & 1 == 1 => osc_crc_byte(v, b),
                    _ => v,
                };
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
        if self.phase != Phase::Break || !self.wire.idle() {
            return;
        }
        if let Some(frame) = img.get(ID_AT..cov + CRC_LEN) {
            self.wire.stream(frame);
            self.phase = Phase::Stream;
            if img[FLAGS_AT] & FLAG_LAST != 0 {
                self.publish_last();
            }
        }
    }

    /// Start frame `done`: reset the CRC engine, feed what is written, and
    /// break.
    #[inline(never)]
    fn start(&mut self) {
        let f = self.done;
        // SAFETY: kernel side (fn doc of `image`).
        let img = unsafe { self.ch.image(f) };
        let banked = f != self.seq;
        let written = if banked { covered(img) } else { self.at };
        self.crc.reset();
        self.fed = 0;
        self.feed(img, written & !1);
        self.crc_due = banked;
        self.wire.break_start();
        self.phase = Phase::Break;
    }

    /// Feed the CRC engine the image's covered bytes from `fed` to the
    /// even `to`. One call site for the engine's arm: inlined twice it
    /// doubles the lane's flash.
    #[inline(never)]
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

impl<P: Providers> TelStream for TelFeed<P> {
    fn row_mask(&mut self) -> u16 {
        if !self.ch.active() {
            return 0;
        }
        // SAFETY: bus-written mailbox seq, volatile-read (type doc).
        let seq = unsafe { self.ch.arm_seq.get().read_volatile() };
        if seq != self.epoch {
            self.pickup(seq);
        }
        if self.remaining == 0 { 0 } else { self.mask }
    }

    /// Out of line: the row's whole cost is its own, one call from the
    /// kernel, never the kernel's register pressure.
    #[inline(never)]
    fn on_tick(&mut self, sample: &TelSample) {
        if self.remaining == 0 {
            return;
        }
        // SAFETY: kernel side (fn doc of `image`).
        let img = unsafe { self.ch.image(self.seq) };
        if self.n == 0 {
            self.landed = true;
            if self.seq.wrapping_sub(self.done) as i8 >= 2 {
                // Both images hold unfinished frames: never block the tick.
                // SAFETY: kernel-written counter (type doc).
                unsafe {
                    let d = self.ch.drops.get();
                    d.write_volatile(d.read_volatile().wrapping_add(1));
                }
                self.landed = false;
                return;
            }
            self.head(img, sample);
        }
        let at = self.at;
        self.at = at + self.plan.len();
        self.valid |= (sample.window_valid as u32) << self.n;
        self.alert |= sample.fault;
        self.n += 1;
        self.remaining -= 1;
        if let Some(row) = img.get_mut(at..) {
            self.plan.encode(sample, row);
        }
        if self.n == self.rows {
            if let Some(out) = img.get_mut(self.at..) {
                encode_stream_trailer(self.valid, self.alert, out);
            }
            // A started frame owes the engine the rest of its bytes.
            self.crc_due |= self.phase != Phase::Idle && self.seq == self.done;
            self.landed = true;
            self.seq = self.seq.wrapping_add(1);
            self.n = 0;
            crate::bench::tel_banked();
        }
    }

    /// The TX lane, inline: the tick that owes it nothing pays a flag test
    /// and the TX poll. It takes one step a tick at most - each is a few
    /// hundred HCLK, and two would not fit beside the dearest phases
    /// (budget `tel_lane`) - and none on a tick that landed a head or a
    /// trailer. Frame `done`'s CRC step comes first (the patch lands ahead
    /// of an arm), then its arm, or the start of the next frame. A frame's
    /// end is a few stores, so the next can start in the same tick: a tick
    /// apart, the wire would idle one tick a frame.
    #[inline(always)]
    fn on_tick_end(&mut self) {
        if !self.ch.active() || core::mem::take(&mut self.landed) {
            return;
        }
        match self.phase {
            Phase::Idle => {}
            // A frame off the wire owes nothing more, a CRC step included.
            Phase::Stream if self.wire.idle() => {
                self.done = self.done.wrapping_add(1);
                self.phase = Phase::Idle;
                self.crc_due = false;
            }
            _ if self.crc_due => return self.service(),
            Phase::Break if self.wire.idle() => return self.service(),
            _ => return,
        }
        if self.startable() {
            self.start();
        }
    }
}

/// Bus-side half: posts arms, lends the lane and takes it back.
pub struct TelDrain {
    ch: &'static TelChannel,
}

impl TelDrain {
    /// Post a burst's row plan, while the lane is the bus's. The plan's
    /// layout walk is the arm's dear half (~500 HCLK): it runs in the body
    /// that dispatches the arm, not in the ack's release that lends.
    pub(crate) fn post(&mut self, mask: u16) {
        let plan = RowPlan::new(mask).unwrap_or(RowPlan::EMPTY);
        // SAFETY: sole mailbox writer, and the kernel reads the plan only
        // at the pickup a later `start` keys (type doc).
        unsafe {
            self.ch.plan.get().write_volatile(plan);
            self.ch.arm_mask.get().write_volatile(mask);
        }
    }

    /// Lend the kernel the lane for the posted plan: it takes the arm at
    /// its next tick. Returns the arm's epoch (never 0).
    pub(crate) fn start(&mut self, count: u16, id: u8, rate: BaudRate) -> u8 {
        let ch = self.ch;
        // SAFETY: sole mailbox writer, and the lane is the bus's until the
        // `active` store (type doc).
        unsafe {
            ch.arm_count.get().write_volatile(count);
            ch.arm_id.get().write_volatile(id);
            ch.arm_rate.get().write_volatile(rate);
            let mut seq = ch.arm_seq.get().read_volatile().wrapping_add(1);
            if seq == 0 {
                seq = 1;
            }
            compiler_fence(Ordering::Release);
            ch.arm_seq.get().write_volatile(seq);
            ch.active.get().write_volatile(true);
            seq
        }
    }

    /// The RX ring cursor to resume at once the burst posted as `epoch`
    /// has its LAST frame on the wire; read ahead of the caller's check
    /// that the wire went idle.
    pub(crate) fn last_out(&self, epoch: u8) -> Option<u16> {
        // SAFETY: kernel-written fields, volatile-read; the fence keeps the
        // cursor read behind the epoch it was published under.
        unsafe {
            if self.ch.last_out.get().read_volatile() != epoch {
                return None;
            }
            compiler_fence(Ordering::Acquire);
            Some(self.ch.resume.get().read_volatile())
        }
    }

    pub(crate) fn stop(&mut self) {
        // SAFETY: sole writer of `active` (type doc).
        unsafe { self.ch.active.get().write_volatile(false) };
    }
}

#[cfg(test)]
mod tests;
