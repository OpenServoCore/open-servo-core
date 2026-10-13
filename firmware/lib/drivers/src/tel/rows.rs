use osc_protocol::frame::Header;
use osc_protocol::tel::encode_stream_head;
use osc_protocol::wire::{self, CHAR_BIT_TIMES, Inst, ResultCode};
use osc_servo_core::RegionStorageRaw;
use osc_servo_core::kernel::TICK_HZ;
use osc_servo_core::regions::telemetry::tick_flag;
use osc_servo_core::tel::{BAUD, Chunk, Chunks, ROW_BYTES_MAX, STREAM_SAMPLES_MAX};

use super::*;
use crate::traits::bus::{CrcEngine, Providers};

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
pub(super) struct RowPlan {
    dma: RowCopy,
    cpu: RowCopy,
    pub(super) row: u8,
}

impl RowPlan {
    pub(super) const NONE: Self = Self {
        dma: RowCopy::NONE,
        cpu: RowCopy::NONE,
        row: 0,
    };

    /// The plan for `chunks` of the table at `base`, or none (row 0) for
    /// chunks that are no row: this holds the copies inside the table and
    /// the rows inside the images whatever the bus posts.
    pub(super) fn of(chunks: Chunks, base: *const u8) -> Self {
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

impl<P: Providers> TelLane<P> {
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
    pub(super) fn row(&mut self) {
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
            self.crc_due |= self.phase != Lane::Idle && self.seq == self.done;
            self.landed = true;
            self.seq = self.seq.wrapping_add(1);
            self.n = 0;
            crate::bench::tel_banked();
        }
    }
}
