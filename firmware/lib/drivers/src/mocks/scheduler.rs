use core::cell::Cell;
use std::vec::Vec;

use crate::traits::dxl::{FastLastScheduler, SendKind, TxScheduler};

/// One entry per `TxScheduler` call; tests assert the recorded sequence
/// against expected TX scheduling.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ScheduleOp {
    Schedule {
        deadline_tick: u16,
        byte_count: u16,
        kind: SendKind,
    },
    Cancel,
}

#[derive(Default)]
pub struct MockTxScheduler {
    pub log: Vec<ScheduleOp>,
}

impl TxScheduler for MockTxScheduler {
    // Same value as the production V006 binding (HCLK = 48 MHz) so driver
    // tests' deadline_tick math lands on the same numbers the chip sees.
    const TICKS_PER_US: u16 = 48;

    fn schedule(&mut self, deadline_tick: u16, byte_count: u16, kind: SendKind) {
        self.log.push(ScheduleOp::Schedule {
            deadline_tick,
            byte_count,
            kind,
        });
    }

    fn cancel(&mut self) {
        self.log.push(ScheduleOp::Cancel);
    }
}

/// One entry per `FastLastScheduler` call; tests assert the recorded
/// sequence against expected scheduler operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FastLastSchedulerOp {
    SetDeadline {
        packet_end_tick: u16,
        deadline_ticks: u32,
    },
    Schedule {
        offset_ticks: u32,
    },
    Cancel,
}

/// `deadline_passed` / `patch_window_expired` read via interior mutability
/// so the trait methods stay `&self`-compatible with production's
/// register-read impls. Tests write `deadline_passed_value.set(true)` /
/// `patch_window_expired_value.set(true)` to drive the busy-wait exits;
/// `patch_miss_count` accumulates `record_patch_deadline_miss` calls.
#[derive(Default)]
pub struct MockFastLastScheduler {
    pub log: Vec<FastLastSchedulerOp>,
    pub deadline_passed_value: Cell<bool>,
    pub patch_window_expired_value: Cell<bool>,
    pub patch_miss_count: Cell<u32>,
}

impl FastLastScheduler for MockFastLastScheduler {
    // Values match the V006 measurements.rs defaults so driver-side grid
    // math lines up with the chip-side reference.
    const FAST_LAST_ENTRY_TICKS: u16 = 240;
    const BYTES_PER_INTERVAL: u16 = 15;
    const GUARD_BYTES: u16 = 1;

    fn set_deadline(&mut self, packet_end_tick: u16, deadline_ticks: u32) {
        self.log.push(FastLastSchedulerOp::SetDeadline {
            packet_end_tick,
            deadline_ticks,
        });
    }

    fn schedule(&mut self, offset_ticks: u32) {
        self.log
            .push(FastLastSchedulerOp::Schedule { offset_ticks });
    }

    fn deadline_passed(&self) -> bool {
        self.deadline_passed_value.get()
    }

    fn patch_window_expired(&self) -> bool {
        self.patch_window_expired_value.get()
    }

    fn record_patch_deadline_miss(&mut self) {
        self.patch_miss_count
            .set(self.patch_miss_count.get().wrapping_add(1));
    }

    fn cancel(&mut self) {
        self.log.push(FastLastSchedulerOp::Cancel);
    }
}
