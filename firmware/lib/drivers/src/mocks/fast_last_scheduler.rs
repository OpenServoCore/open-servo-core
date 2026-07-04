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
        // Values match the V006 measurements.rs defaults so driver-side wake
        // math lines up with the chip-side reference.
        const WAKE_LEAD_TICKS: u16 = 500;
        const INLINE_FOLD_HORIZON_TICKS: u16 = 2400;

        fn set_busy_wait_deadline(&mut self, deadline: u32);
        fn deadline_passed(&self) -> bool;
        fn schedule(&mut self, deadline: u32);
        fn patch_window_expired(&self) -> bool;
        fn cancel(&mut self);
    }
}
