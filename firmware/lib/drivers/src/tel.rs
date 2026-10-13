//! TEL burst lane (protocol sec 5.6): while a burst streams, the kernel
//! fast tick (PFIC HIGH) owns the bus TX lane and is the transmitter. Each
//! row is a copy of the table's chunks straight to its final wire offset in
//! one of two frame images, started by the kernel at fixed points of the
//! tick; a frame breaks once its lead row has landed and its DMA reads the
//! rows behind it as the wire reaches them, the CRC engine fed by the
//! producer and the value patched in before the DMA gets there. No TX
//! interrupt and no bus-level body runs for the burst: the bus lends the
//! lane at the arming ack's release (at the arm itself when no ack
//! follows) and takes it back in its next body once the LAST frame has
//! left the wire.
//!
//! Cross-side traffic is single-writer-per-field volatile load/store plus
//! compiler fences (single core, no atomic RMW on rv32ec). The bus writes
//! the arm mailbox only while the lane is its own, then `active`; the
//! kernel, which the bus never preempts, takes the arm whole. The row plan
//! is plain memory, ordered by the fences around `arm_seq`.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::{Ordering, compiler_fence};

use osc_protocol::frame::Header;
use osc_protocol::wire::{self, CHAR_BIT_TIMES, Inst, ResultCode};
use osc_servo_core::kernel::TICK_HZ;
use osc_servo_core::tel::{
    BAUD, Chunk, Chunks, FLAG_LAST, ROW_BYTES_MAX, STREAM_HDR, STREAM_PAYLOAD_MAX,
    STREAM_SAMPLES_MAX, TelStream, encode_stream_head, tick_flag,
};
use osc_servo_core::{ControlTableCell, RegionStorageRaw};

use crate::bench::{self, TelStep};
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

// Rows land as halfword copies, and every covered span (ROWS_AT and whole
// even rows) feeds the CRC engine whole: no odd byte to fold.
const _: () = assert!(ROWS_AT.is_multiple_of(2));

/// Ticks the start rule leaves between the bank and the DMA reaching the
/// CRC. The lane skips the bank's tick and the next frame's head tick,
/// feeds on the one after and patches on the next: three ticks of slack
/// remain, and the last row's read has at least as many. Late or lost
/// ticks, or stage ticks the lane holds on, inside them still ship a whole
/// frame (DES `tel_kernel_lane_never_underruns_at_budget`); more fail its
/// CRC (DES `tel_lost_ticks_mid_frame_fail_the_crc_and_leave_a_seq_hole`).
const LEAD_MARGIN_TICKS: u32 = 5;

/// Control ticks per wire byte at [`BAUD`], Q16, rounded down so the lead
/// rounds late.
const TICKS_PER_BYTE_Q16: u32 =
    ((((CHAR_BIT_TIMES * TICK_HZ) as u64) << 16) / BAUD.as_hz() as u64) as u32;

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
    arm_row: SyncUnsafeCell<RowPlan>,
    arm_count: SyncUnsafeCell<u16>,
    arm_id: SyncUnsafeCell<u8>,
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
            arm_row: SyncUnsafeCell::new(RowPlan::NONE),
            arm_count: SyncUnsafeCell::new(0),
            arm_id: SyncUnsafeCell::new(0),
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

    /// Split into the two halves over the lane's wire, CRC engine, RX ring
    /// and clock handles and the table the rows copy from. Call once at
    /// bringup, before interrupts run: a second feed or drain breaks the
    /// single-writer discipline.
    pub fn split<P: Providers>(
        &'static self,
        wire: P::Tx,
        crc: P::Crc,
        ring: P::Ring,
        clock: P::Deadline,
        table: &'static ControlTableCell,
    ) -> (TelFeed<P>, TelDrain) {
        bench::tel_calibrate(&clock);
        (
            TelFeed {
                ch: self,
                wire,
                crc,
                ring,
                clock,
                #[cfg(feature = "bench-telcost")]
                stamp: false,
                table,
                epoch: 0,
                id: 0,
                plan: RowPlan::NONE,
                remaining: 0,
                seq: 0,
                done: 0,
                wr: 0,
                n: 0,
                rows: 0,
                lead: 0,
                phase: Phase::Idle,
                fed: 0,
                crc_due: false,
                landed: false,
            },
            TelDrain { ch: self, table },
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

/// One of a row's copies: its first byte in the table (an exposed
/// address: the feed is touched from one context, but sits in a `Sync`
/// static), its halfwords and its offset in the row.
#[derive(Copy, Clone)]
struct RowCopy {
    src: usize,
    units: u16,
    at: u8,
}

impl RowCopy {
    const NONE: Self = Self {
        src: 0,
        units: 0,
        at: 0,
    };
}

/// A row's copies, the longer chunk by DMA and the shorter, at most half a
/// row, by the CPU (under the DMA arm's cost, and one CH6 arm a row leaves
/// none in flight at the next), and the row bytes. The bus builds it at
/// the arm, off the kernel's first tick.
#[derive(Copy, Clone)]
struct RowPlan {
    dma: RowCopy,
    cpu: RowCopy,
    row: u8,
}

impl RowPlan {
    const NONE: Self = Self {
        dma: RowCopy::NONE,
        cpu: RowCopy::NONE,
        row: 0,
    };

    /// The plan for `chunks` of the table at `base`, or none (row 0) for
    /// chunks that are no row: this holds the copies inside the table and
    /// the rows inside the images whatever the bus posts.
    fn of(chunks: Chunks, base: *const u8) -> Self {
        if !chunks.valid() {
            return Self::NONE;
        }
        let copy = |c: Chunk, at: u16| RowCopy {
            src: base.wrapping_add(c.addr as usize).expose_provenance(),
            units: c.len / 2,
            at: at as u8,
        };
        let [a, b] = chunks.0;
        let (dma, cpu) = if b.len > a.len {
            (copy(b, a.len), copy(a, 0))
        } else {
            (copy(a, 0), copy(b, a.len))
        };
        Self {
            dma,
            cpu,
            row: chunks.row_bytes() as u8,
        }
    }
}

/// Frame `done`'s place on the lane.
#[derive(Copy, Clone, PartialEq, Eq)]
enum Phase {
    Idle,
    Break,
    Stream,
}

/// Kernel-side half: the fast tick's [`TelStream`] sink, the row copies,
/// the frame heads and the TX lane. All fields but `ch` are kernel-private.
pub struct TelFeed<P: Providers> {
    ch: &'static TelChannel,
    wire: P::Tx,
    crc: P::Crc,
    ring: P::Ring,
    /// The `bench-telcost` stamps' time base.
    #[cfg_attr(not(feature = "bench-telcost"), allow(dead_code))]
    clock: P::Deadline,
    /// This burst stamps its steps.
    #[cfg(feature = "bench-telcost")]
    stamp: bool,
    table: &'static ControlTableCell,
    /// The arm this burst runs under (`arm_seq` at pickup).
    epoch: u8,
    id: u8,
    plan: RowPlan,
    remaining: u16,
    /// Frames begun: the filling one carries `seq`. Frames before `done`
    /// have left the wire, which may include the filling one: a frame the
    /// wire outran (late ticks) ships stale and fails its CRC, and its rows
    /// land on until its bank.
    seq: u8,
    done: u8,
    /// The filling frame: where its next row lands (exposed address, as
    /// `src`), rows landed and planned, and the rows a cut-through start
    /// waits past.
    wr: usize,
    n: u8,
    rows: u8,
    lead: u8,
    /// Frame `done` on the lane: its phase, the covered bytes fed to the
    /// CRC engine, and whether its CRC still owes a feed or the patch.
    phase: Phase,
    fed: usize,
    crc_due: bool,
    /// This tick landed a head or a bank, or holds: the lane leaves it be.
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
        self.phase = Phase::Idle;
        if self.remaining == 0 {
            // The bus lends only for a nonzero admitted arm; an empty one
            // still hands the lane back.
            self.publish_last();
        }
    }

    /// Row 0 of frame `seq`: header, head, and the start rule's lead, or
    /// false when both images hold unfinished frames and the row drops.
    /// The stamp and the fault are this tick's, from the tick block. Out of
    /// line: once a frame, and the rows between keep their registers.
    #[cold]
    #[inline(never)]
    fn head(&mut self) -> bool {
        if self.seq.wrapping_sub(self.done) as i8 >= 2 {
            // Never block the tick.
            // SAFETY: kernel-written counter (type doc).
            unsafe {
                let d = self.ch.drops.get();
                d.write_volatile(d.read_volatile().wrapping_add(1));
            }
            return false;
        }
        self.landed = true;
        // SAFETY: kernel side (fn doc of `image`).
        let img = unsafe { self.ch.image(self.seq) };
        let rows = (self.remaining as usize).min(STREAM_SAMPLES_MAX);
        let cov = ROWS_AT + rows * self.plan.row as usize;
        let p = self.table.region_ptr();
        // SAFETY: tick-block fields the tick ISR and the kernel, this
        // context, wrote earlier in the tick; volatile reads.
        let (tick, flags) = unsafe {
            let t = &raw const (*p).telemetry.tick;
            (
                (&raw const (*t).sample_tick).read_volatile() as u16,
                (&raw const (*t).tick_flags).read_volatile(),
            )
        };
        img[ID_AT] = self.id;
        img[LEN_AT] = wire::len_for((cov - Header::SIZE) as u8);
        img[INST_AT] = Inst::status(ResultCode::Stream, flags & tick_flag::FAULT != 0).0;
        let last = rows == self.remaining as usize;
        encode_stream_head(self.seq, last, tick, &mut img[Header::SIZE..]);
        // The DMA reaches the CRC (cov - 2) byte-times after the arm, a
        // tick behind the break; the patch is a tick behind the bank.
        let ticks = ((cov - CRC_LEN) as u32 * TICKS_PER_BYTE_Q16) >> 16;
        self.lead = (rows as u32 - 1 + LEAD_MARGIN_TICKS).saturating_sub(ticks) as u8;
        self.rows = rows as u8;
        self.wr = img[ROWS_AT..].as_mut_ptr().expose_provenance();
        true
    }

    /// The tick's row, copied now while the tick block holds this tick's
    /// scan (the next lands late in the tick): the CPU's chunk, then the
    /// DMA's. Out of line: the row's whole cost is its own, one call from
    /// the kernel.
    #[inline(never)]
    fn row(&mut self) {
        if self.n == 0 && !self.head() {
            return;
        }
        let wr = self.wr;
        // SAFETY: `head` planned at most STREAM_SAMPLES_MAX rows of at most
        // ROW_BYTES_MAX (`RowPlan::of`), which the image holds from ROWS_AT,
        // so the row lies inside it; the chunks lie inside the table
        // (`Chunks::valid`); ROWS_AT, the rows and the chunks are even.
        unsafe {
            // The shorter chunk, at most half a row: straight-line, three
            // halfwords at most whatever `units` holds (a loop's setup
            // alone outprices the copy).
            const { assert!(ROW_BYTES_MAX / 2 <= 3 * 2) };
            let s = core::ptr::with_exposed_provenance::<u16>(self.plan.cpu.src);
            let d = core::ptr::with_exposed_provenance_mut::<u16>(wr + self.plan.cpu.at as usize);
            let n = self.plan.cpu.units;
            if n != 0 {
                d.write_volatile(s.read_volatile());
                if n != 1 {
                    d.add(1).write_volatile(s.add(1).read_volatile());
                    if n != 2 {
                        d.add(2).write_volatile(s.add(2).read_volatile());
                    }
                }
            }
            let dst = core::ptr::with_exposed_provenance_mut(wr + self.plan.dma.at as usize);
            let src = core::ptr::with_exposed_provenance(self.plan.dma.src);
            self.crc.copy(dst, src, self.plan.dma.units);
        }
        self.wr = wr + self.plan.row as usize;
        self.n += 1;
        self.remaining -= 1;
        if self.n == self.rows {
            // A started frame owes the engine the rest of its bytes.
            self.crc_due |= self.phase != Phase::Idle && self.seq == self.done;
            self.landed = true;
            self.seq = self.seq.wrapping_add(1);
            self.n = 0;
            crate::bench::tel_banked();
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
    /// The burst starts at the arm's pickup: a held start leaves the arm in
    /// the mailbox, and the lane idle, for the next tick.
    #[inline(always)]
    fn on_tick(&mut self, hold_start: impl FnOnce() -> bool) {
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
        if !self.ch.active() {
            return;
        }
        if self.stamping() {
            let t = bench::tel_stamp(&self.clock);
            let s = self.lane();
            bench::tel_step(&self.clock, t, s);
        } else {
            self.lane();
        }
    }
}

impl<P: Providers> TelFeed<P> {
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
    fn row_step(&self) -> TelStep {
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
    fn lane(&mut self) -> TelStep {
        if core::mem::take(&mut self.landed) {
            return TelStep::Poll;
        }
        match self.phase {
            Phase::Idle => {}
            // A frame off the wire owes nothing more, a CRC step included.
            Phase::Stream if self.wire.idle() => {
                self.done = self.done.wrapping_add(1);
                self.phase = Phase::Idle;
                self.crc_due = false;
            }
            _ if self.crc_due => {
                self.service();
                return TelStep::Service;
            }
            Phase::Break if self.wire.idle() => {
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

/// Bus-side half: posts arms, lends the lane and takes it back, and zeroes
/// `tel_count` when an arm ends.
pub struct TelDrain {
    ch: &'static TelChannel,
    table: &'static ControlTableCell,
}

impl TelDrain {
    /// The arm is over (refused, its lend dropped, or its LAST frame off
    /// the wire): `tel_count` reads 0, so it reads nonzero only while a
    /// burst is armed or streams (protocol sec 5.6). Bus side, outside any
    /// table borrow: the bus is CONTROL's writer.
    pub(crate) fn end(&mut self) {
        let p = self.table.region_ptr();
        // SAFETY: in-bounds field of the static table; this context writes
        // CONTROL, and the kernel only reads it.
        unsafe { (&raw mut (*p).control.tel.tel_count).write_volatile(0) };
    }

    /// Post a burst's row, while the lane is the bus's.
    pub(crate) fn post(&mut self, chunks: Chunks) {
        let plan = RowPlan::of(chunks, self.table.region_ptr().cast());
        // SAFETY: sole mailbox writer, and the kernel reads the plan only
        // at the pickup a later `start` keys (type doc).
        unsafe { *self.ch.arm_row.get() = plan };
    }

    /// Lend the kernel the lane for the posted chunks: it takes the arm at
    /// its next tick. Returns the arm's epoch (never 0).
    pub(crate) fn start(&mut self, count: u16, id: u8) -> u8 {
        let ch = self.ch;
        // SAFETY: sole mailbox writer, and the lane is the bus's until the
        // `active` store (type doc).
        unsafe {
            ch.arm_count.get().write_volatile(count);
            ch.arm_id.get().write_volatile(id);
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
