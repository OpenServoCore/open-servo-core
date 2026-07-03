//! Shared consts, mock state companions, and wire builders for the DXL
//! UART unit-test modules — one place to keep per-baud tick math and mock
//! setups aligned. Test-only.
//!
//! State companions mirror `osc-integration::mocks::*` — clone-shared
//! Cell/RefCell handles the mock closures push into, so tests read
//! observed effects out of the state rather than the mock.

extern crate alloc;

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use dxl_protocol::{Id, InstructionEncoder, SoftwareCrcUmts, StatusEncoder, StatusError};
use osc_core::BaudRate;

use crate::mocks::{
    FastLastSchedulerOp, MockClockTrim, MockEdgeDma, MockFastLastScheduler, MockRxDma, MockTxBus,
    MockTxScheduler, MockUsartBaud, MockWireClock, ScheduleOp, TxBusOp,
};
use crate::traits::dxl::DmaFlags;

/// Default servo ID used by composite / stage_id tests.
pub(crate) const TEST_ID: u8 = 0x07;

/// Default RDT (µs) fed to `DxlUart::new` in composite tests.
pub(crate) const TEST_RDT_US: u32 = 250;

/// HCLK ticks per microsecond at the V006 reference clock (`CLOCK_HZ = 48 MHz`).
pub(crate) const TICKS_PER_US: u32 = 48;

/// Deterministic packet-end reference tick used by `bus_seeded_with` /
/// `force_anchor` in composite tests. Held below `TICKS_PER_BIT_9600` so
/// `lift`-under-wrap invariants stay well inside a single u16 window at all
/// tested bauds.
pub(crate) const SEED_TICK: u16 = 1000;

/// `ticks_per_bit` at `BaudRate::B3000000`, HCLK 48 MHz. Matches
/// `edge_parser::tests::TPB_3M`.
pub(crate) const TICKS_PER_BIT_3M: u16 = 16;

/// `ticks_per_bit` at `BaudRate::B9600`, HCLK 48 MHz. Byte-time
/// (`BITS_PER_FRAME · tpb = 50_000`) exceeds i16 range — regression fixture
/// for signed-distance walker aliasing.
pub(crate) const TICKS_PER_BIT_9600: u16 = 5000;

// ------------------------------------------------------------------
// Mock state companions
// ------------------------------------------------------------------

#[derive(Clone, Default)]
pub(crate) struct UsartBaudState {
    apply_log: Rc<RefCell<alloc::vec::Vec<BaudRate>>>,
}

impl UsartBaudState {
    pub(crate) fn apply_baud_log(&self) -> alloc::vec::Vec<BaudRate> {
        self.apply_log.borrow().clone()
    }
}

/// `MockUsartBaud` with a call log for `apply_baud` and `rx_edge_comp_ticks`
/// wired to a constant `0`. Tests that don't care about the log drop the
/// state via `let (m, _) = mk_usart_baud()`.
pub(crate) fn mk_usart_baud() -> (MockUsartBaud, UsartBaudState) {
    let state = UsartBaudState::default();
    let mut m = MockUsartBaud::new();
    {
        let log = state.apply_log.clone();
        m.expect_apply_baud().returning_st(move |b| {
            log.borrow_mut().push(b);
        });
    }
    m.expect_rx_edge_comp_ticks().returning_st(|_| 0);
    (m, state)
}

#[derive(Clone, Default)]
pub(crate) struct ClockTrimState {
    apply_log: Rc<RefCell<alloc::vec::Vec<i32>>>,
}

impl ClockTrimState {
    pub(crate) fn apply_ppm_log(&self) -> alloc::vec::Vec<i32> {
        self.apply_log.borrow().clone()
    }
}

/// `MockClockTrim` with a call log for `apply_ppm`. Tests that don't care
/// about the log drop the state via `let (m, _) = mk_clock_trim()`.
pub(crate) fn mk_clock_trim() -> (MockClockTrim, ClockTrimState) {
    let state = ClockTrimState::default();
    let mut m = MockClockTrim::new();
    {
        let log = state.apply_log.clone();
        m.expect_apply_ppm().returning_st(move |p| {
            log.borrow_mut().push(p);
        });
    }
    (m, state)
}

#[derive(Clone, Default)]
pub(crate) struct SchedState {
    ops: Rc<RefCell<alloc::vec::Vec<ScheduleOp>>>,
}

impl SchedState {
    pub(crate) fn operations(&self) -> alloc::vec::Vec<ScheduleOp> {
        self.ops.borrow().clone()
    }
}

/// `MockTxScheduler` logging `schedule` / `commit_pending` / `cancel` into
/// its state companion; `on_schedule_due` wired to `false`.
pub(crate) fn mk_scheduler() -> (MockTxScheduler, SchedState) {
    let state = SchedState::default();
    let mut m = MockTxScheduler::new();
    {
        let ops = state.ops.clone();
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
        let ops = state.ops.clone();
        m.expect_commit_pending().returning_st(move || {
            ops.borrow_mut().push(ScheduleOp::CommitPending);
        });
    }
    {
        let ops = state.ops.clone();
        m.expect_cancel().returning_st(move || {
            ops.borrow_mut().push(ScheduleOp::Cancel);
        });
    }
    m.expect_on_schedule_due().returning_st(|| false);
    (m, state)
}

#[derive(Clone, Default)]
pub(crate) struct TxBusState {
    ops: Rc<RefCell<alloc::vec::Vec<TxBusOp>>>,
}

impl TxBusState {
    pub(crate) fn operations(&self) -> alloc::vec::Vec<TxBusOp> {
        self.ops.borrow().clone()
    }
    pub(crate) fn clear(&self) {
        self.ops.borrow_mut().clear();
    }
}

/// `MockTxBus` logging every op into its state companion.
pub(crate) fn mk_tx_bus() -> (MockTxBus, TxBusState) {
    let state = TxBusState::default();
    let mut m = MockTxBus::new();
    {
        let ops = state.ops.clone();
        m.expect_start_now().returning_st(move |byte_count| {
            ops.borrow_mut().push(TxBusOp::StartNow { byte_count });
        });
    }
    {
        let ops = state.ops.clone();
        m.expect_take_bus().returning_st(move || {
            ops.borrow_mut().push(TxBusOp::TakeBus);
        });
    }
    {
        let ops = state.ops.clone();
        m.expect_release_bus().returning_st(move || {
            ops.borrow_mut().push(TxBusOp::ReleaseBus);
        });
    }
    (m, state)
}

#[derive(Clone, Default)]
pub(crate) struct RxDmaState {
    remaining: Rc<Cell<u16>>,
}

impl RxDmaState {
    pub(crate) fn stage_remaining(&self, n: u16) {
        self.remaining.set(n);
    }
}

/// `MockRxDma` whose `remaining()` reads the staged counter; `read_and_ack`
/// returns quiet flags and anchor-miss telemetry is accepted silently.
pub(crate) fn mk_rx_dma() -> (MockRxDma, RxDmaState) {
    let state = RxDmaState::default();
    let mut m = MockRxDma::new();
    {
        let r = state.remaining.clone();
        m.expect_remaining().returning_st(move || r.get());
    }
    m.expect_read_and_ack().returning_st(DmaFlags::default);
    m.expect_record_edge_anchor_miss().returning_st(|| ());
    (m, state)
}

#[derive(Clone, Default)]
pub(crate) struct EdgeDmaState {
    remaining: Rc<Cell<u16>>,
}

impl EdgeDmaState {
    pub(crate) fn stage_remaining(&self, n: u16) {
        self.remaining.set(n);
    }
}

/// `MockEdgeDma` whose `remaining()` reads the staged counter.
pub(crate) fn mk_edge_dma() -> (MockEdgeDma, EdgeDmaState) {
    let state = EdgeDmaState::default();
    let mut m = MockEdgeDma::default();
    {
        let r = state.remaining.clone();
        m.expect_remaining().returning_st(move || r.get());
    }
    (m, state)
}

/// Unified state companion for `MockFastLastScheduler` — carries every knob
/// any consumer stages (composite, `FsmScheduler`, and `ReplyHandle` tests
/// share this one shape).
#[derive(Clone, Default)]
pub(crate) struct FastLastState {
    ops: Rc<RefCell<alloc::vec::Vec<FastLastSchedulerOp>>>,
    deadline_passed: Rc<Cell<bool>>,
    patch_window_expired: Rc<Cell<bool>>,
    patch_miss_count: Rc<Cell<u32>>,
}

impl FastLastState {
    pub(crate) fn operations(&self) -> alloc::vec::Vec<FastLastSchedulerOp> {
        self.ops.borrow().clone()
    }
    pub(crate) fn stage_deadline_passed(&self, v: bool) {
        self.deadline_passed.set(v);
    }
    pub(crate) fn stage_patch_window_expired(&self, v: bool) {
        self.patch_window_expired.set(v);
    }
    pub(crate) fn patch_miss_count(&self) -> u32 {
        self.patch_miss_count.get()
    }
}

/// `MockFastLastScheduler` logging grid ops into its state companion, with
/// `deadline_passed` / `patch_window_expired` reading the staged flags and
/// `record_patch_deadline_miss` bumping the counter.
pub(crate) fn mk_fast_last() -> (MockFastLastScheduler, FastLastState) {
    let state = FastLastState::default();
    let mut m = MockFastLastScheduler::new();
    {
        let ops = state.ops.clone();
        m.expect_set_busy_wait_deadline()
            .returning_st(move |deadline| {
                ops.borrow_mut()
                    .push(FastLastSchedulerOp::SetBusyWaitDeadline { deadline });
            });
    }
    {
        let ops = state.ops.clone();
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
        let ops = state.ops.clone();
        m.expect_cancel().returning_st(move || {
            ops.borrow_mut().push(FastLastSchedulerOp::Cancel);
        });
    }
    (m, state)
}

/// `MockWireClock` pinned at [`SEED_TICK`].
pub(crate) fn mk_wire_clock() -> MockWireClock {
    let mut m = MockWireClock::new();
    m.expect_now().returning_st(|| SEED_TICK as u32);
    m
}

// ------------------------------------------------------------------
// Wire builders
// ------------------------------------------------------------------

pub(crate) fn wire_ping(id: u8) -> heapless::Vec<u8, 32> {
    let mut out: heapless::Vec<u8, 32> = heapless::Vec::new();
    InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
        .ping(Id::new(id))
        .unwrap();
    out
}

pub(crate) fn wire_status(id: u8) -> heapless::Vec<u8, 32> {
    let mut out: heapless::Vec<u8, 32> = heapless::Vec::new();
    StatusEncoder::<_, SoftwareCrcUmts>::new(&mut out)
        .empty(Id::new(id), StatusError::OK)
        .unwrap();
    out
}
