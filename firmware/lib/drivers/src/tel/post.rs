use core::sync::atomic::{Ordering, compiler_fence};

use osc_servo_core::tel::Chunks;
use osc_servo_core::{ControlTableCell, RegionStorageRaw};

use super::*;

/// Bus-side half: posts arms, lends the lane and takes it back, and zeroes
/// `tel_count` when an arm ends.
pub struct TelPost {
    pub(super) ch: &'static TelChannel,
    pub(super) table: &'static ControlTableCell,
}

impl TelPost {
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
