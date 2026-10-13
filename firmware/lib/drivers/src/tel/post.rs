use core::sync::atomic::{Ordering, compiler_fence};

use osc_servo_core::tel::Chunks;
use osc_servo_core::{ControlTableCell, RegionStorageRaw};

use super::*;

/// Bus-side half (protocol sec 5.6): a committed nonzero `tel_count` write
/// arms a burst, and the bus lends the kernel the lane at the arming ack's
/// release, or at the arm when no ack follows. The servo owns the line from
/// there to the LAST frame: the bus serves nothing until it takes the lane
/// back, in its first body after LAST has left the wire.
pub struct TelPost {
    pub(super) ch: &'static TelChannel,
    pub(super) table: &'static ControlTableCell,
    /// The arm the next lend streams; its chunks are posted.
    pub(super) count: u16,
    /// The lent burst's epoch.
    pub(super) epoch: u8,
}

impl TelPost {
    /// `Reply::tel_arm`, never while the lane is lent: true when the arm
    /// wants the lane, its chunks then posted; count 0 disarms and ends it.
    pub(crate) fn arm(&mut self, chunks: Chunks, count: u16) -> bool {
        self.count = count;
        if count == 0 {
            self.end();
            return false;
        }
        self.post(chunks);
        true
    }

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
    fn post(&mut self, chunks: Chunks) {
        let plan = RowPlan::of(chunks, self.table.region_ptr().cast());
        // SAFETY: sole mailbox writer, and the kernel reads the plan only
        // at the pickup a later `start` keys (type doc).
        unsafe { *self.ch.arm_row.get() = plan };
    }

    /// Lend the kernel the lane for the posted chunks: it takes the arm at
    /// its next tick.
    pub(crate) fn start(&mut self, id: u8) {
        let ch = self.ch;
        // SAFETY: sole mailbox writer, and the lane is the bus's until the
        // `active` store (type doc).
        self.epoch = unsafe {
            ch.arm_count.get().write_volatile(self.count);
            ch.arm_id.get().write_volatile(id);
            let mut seq = ch.arm_seq.get().read_volatile().wrapping_add(1);
            if seq == 0 {
                seq = 1;
            }
            compiler_fence(Ordering::Release);
            ch.arm_seq.get().write_volatile(seq);
            ch.active.get().write_volatile(true);
            seq
        };
    }

    /// The RX ring cursor to resume at, once the LAST frame has left the
    /// wire. `idle` is read after the kernel's LAST notice: read the other
    /// way round, a gap before LAST would pass for its end.
    pub(crate) fn returned(&self, idle: impl FnOnce() -> bool) -> Option<u16> {
        // SAFETY: kernel-written fields, volatile-read; the fence keeps the
        // cursor read behind the epoch it was published under.
        let resume = unsafe {
            if self.ch.last_out.get().read_volatile() != self.epoch {
                return None;
            }
            compiler_fence(Ordering::Acquire);
            self.ch.resume.get().read_volatile()
        };
        idle().then_some(resume)
    }

    /// The lane is back: the burst is over.
    pub(crate) fn stop(&mut self) {
        // SAFETY: sole writer of `active` (type doc).
        unsafe { self.ch.active.get().write_volatile(false) };
        self.end();
    }
}
