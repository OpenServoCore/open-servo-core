use std::cell::{Cell, RefCell};
use std::rc::Rc;

use osc_drivers::mocks::{FastLastSchedulerOp, MockFastLastScheduler, MockTxScheduler, ScheduleOp};

#[derive(Clone, Default)]
pub struct TxSchedulerState {
    operations: Rc<RefCell<Vec<ScheduleOp>>>,
}

impl TxSchedulerState {
    pub fn operations(&self) -> Vec<ScheduleOp> {
        self.operations.borrow().clone()
    }
}

pub fn mock_tx_scheduler() -> (MockTxScheduler, TxSchedulerState) {
    let state = TxSchedulerState::default();
    let mut m = MockTxScheduler::new();
    {
        let ops = state.operations.clone();
        m.expect_schedule()
            .returning_st(move |deadline, byte_count, kind| {
                ops.borrow_mut().push(ScheduleOp::Schedule {
                    deadline,
                    byte_count,
                    kind,
                });
            });
    }
    {
        let ops = state.operations.clone();
        m.expect_commit_pending().returning_st(move || {
            ops.borrow_mut().push(ScheduleOp::CommitPending);
        });
    }
    {
        let ops = state.operations.clone();
        m.expect_cancel().returning_st(move || {
            ops.borrow_mut().push(ScheduleOp::Cancel);
        });
    }
    // Sim doesn't model the SysTick handoff path — the simulated wire clock
    // and scheduler tick share an axis, so `schedule` for Plain always lands
    // in the direct-arm branch and never arms a handoff CMP. Return false so
    // any stray invocation routes to the FastLast fold path. Override per-
    // test if a future timing test exercises the handoff.
    m.expect_on_schedule_due().returning_st(|| false);
    (m, state)
}

/// `deadline_passed` / `patch_window_expired` are staged via interior
/// mutability so the trait methods stay `&self`-compatible with
/// production's register-read impls. `patch_miss_count` accumulates
/// `record_patch_deadline_miss` calls.
#[derive(Clone, Default)]
pub struct FastLastSchedulerState {
    deadline_passed: Rc<Cell<bool>>,
    patch_window_expired: Rc<Cell<bool>>,
    patch_miss_count: Rc<Cell<u32>>,
    operations: Rc<RefCell<Vec<FastLastSchedulerOp>>>,
}

impl FastLastSchedulerState {
    pub fn stage_deadline_passed(&self, v: bool) {
        self.deadline_passed.set(v);
    }

    pub fn stage_patch_window_expired(&self, v: bool) {
        self.patch_window_expired.set(v);
    }

    pub fn patch_miss_count(&self) -> u32 {
        self.patch_miss_count.get()
    }

    pub fn operations(&self) -> Vec<FastLastSchedulerOp> {
        self.operations.borrow().clone()
    }
}

pub fn mock_fast_last_scheduler() -> (MockFastLastScheduler, FastLastSchedulerState) {
    let state = FastLastSchedulerState::default();
    let mut m = MockFastLastScheduler::new();
    {
        let ops = state.operations.clone();
        m.expect_set_busy_wait_deadline()
            .returning_st(move |deadline| {
                ops.borrow_mut()
                    .push(FastLastSchedulerOp::SetBusyWaitDeadline { deadline });
            });
    }
    {
        let ops = state.operations.clone();
        m.expect_schedule().returning_st(move |deadline| {
            ops.borrow_mut()
                .push(FastLastSchedulerOp::Schedule { deadline });
        });
    }
    {
        let dp = state.deadline_passed.clone();
        m.expect_deadline_passed().returning_st(move || dp.get());
    }
    {
        let pwe = state.patch_window_expired.clone();
        m.expect_patch_window_expired()
            .returning_st(move || pwe.get());
    }
    {
        let c = state.patch_miss_count.clone();
        m.expect_record_patch_deadline_miss().returning_st(move || {
            c.set(c.get().wrapping_add(1));
        });
    }
    {
        let ops = state.operations.clone();
        m.expect_cancel().returning_st(move || {
            ops.borrow_mut().push(FastLastSchedulerOp::Cancel);
        });
    }
    (m, state)
}
