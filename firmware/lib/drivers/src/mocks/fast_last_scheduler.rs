use mockall::mock;

use crate::traits::dxl::FastLastScheduler;

/// One entry per `FastLastScheduler` call; downstream test harnesses
/// record these to assert scheduler operation sequences.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FastLastSchedulerOp {
    SetBusyWaitDeadline { deadline: u32 },
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

        fn set_busy_wait_deadline(&mut self, deadline: u32);
        fn schedule(&mut self, deadline: u32);
        fn deadline_passed(&self) -> bool;
        fn patch_window_expired(&self) -> bool;
        fn record_patch_deadline_miss(&mut self);
        fn cancel(&mut self);
    }
}
