use mockall::mock;

use crate::traits::dxl::{FastLastScheduler, SendKind, TxScheduler};

/// One entry per `TxScheduler` call; downstream test harnesses record
/// these to assert TX scheduling sequences.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ScheduleOp {
    Schedule {
        deadline: u32,
        byte_count: u16,
        kind: SendKind,
    },
    CommitPending,
    Cancel,
}

mock! {
    pub TxScheduler {}
    impl TxScheduler for TxScheduler {
        // Same value as the production V006 binding (HCLK = 48 MHz) so driver
        // tests' deadline math lands on the same numbers the chip sees.
        const TICKS_PER_US: u16 = 48;

        fn schedule(&mut self, deadline: u32, byte_count: u16, kind: SendKind);
        fn commit_pending(&mut self);
        fn cancel(&mut self);
        fn on_schedule_due(&mut self) -> bool;
    }
}

/// One entry per `FastLastScheduler` call; downstream test harnesses
/// record these to assert scheduler operation sequences.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FastLastSchedulerOp {
    SetDeadline { deadline: u32 },
    Schedule { deadline: u32 },
    Cancel,
}

mock! {
    pub FastLastScheduler {}
    impl FastLastScheduler for FastLastScheduler {
        // Values match the V006 measurements.rs defaults so driver-side grid
        // math lines up with the chip-side reference.
        const FAST_LAST_ENTRY_TICKS: u16 = 240;
        const BYTES_PER_INTERVAL: u16 = 15;
        const GUARD_BYTES: u16 = 1;

        fn set_deadline(&mut self, deadline: u32);
        fn schedule(&mut self, deadline: u32);
        fn deadline_passed(&self) -> bool;
        fn patch_window_expired(&self) -> bool;
        fn record_patch_deadline_miss(&mut self);
        fn cancel(&mut self);
    }
}
