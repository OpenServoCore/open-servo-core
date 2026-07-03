use mockall::mock;

use crate::traits::dxl::{SendKind, TxScheduler};

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
