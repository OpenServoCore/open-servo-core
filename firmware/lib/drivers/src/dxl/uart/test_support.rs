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

use dxl_protocol::encode::{encode_instruction, encode_status};
use dxl_protocol::{Id, Instruction, SoftwareCrcUmts, StatusError};
use osc_core::BaudRate;

use crate::mocks::{
    FastLastSchedulerOp, MockClockTrim, MockFastLastScheduler, MockRxDma, MockTelemetry, MockTxBus,
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
/// `force_anchor` in composite tests. Small so `lift`-under-wrap invariants
/// stay well inside a single u16 window at all tested bauds.
pub(crate) const SEED_TICK: u16 = 1000;

/// `ticks_per_bit` at `BaudRate::B3000000`, HCLK 48 MHz.
pub(crate) const TICKS_PER_BIT_3M: u16 = 16;

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

/// `MockUsartBaud` with a call log for `apply_baud`. Tests that don't care
/// about the log drop the state via `let (m, _) = mk_usart_baud()`.
pub(crate) fn mk_usart_baud() -> (MockUsartBaud, UsartBaudState) {
    let state = UsartBaudState::default();
    let mut m = MockUsartBaud::new();
    {
        let log = state.apply_log.clone();
        m.expect_apply_baud().returning_st(move |b| {
            log.borrow_mut().push(b);
        });
    }
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
    schedule_due_owned: Rc<Cell<bool>>,
}

impl SchedState {
    pub(crate) fn operations(&self) -> alloc::vec::Vec<ScheduleOp> {
        self.ops.borrow().clone()
    }

    /// Stage the next `on_schedule_due` return — `true` models the
    /// SysTick handoff arm owning the match.
    pub(crate) fn stage_schedule_due_owned(&self, v: bool) {
        self.schedule_due_owned.set(v);
    }
}

/// `MockTxScheduler` logging `schedule` / `commit_pending` / `cancel` into
/// its state companion; `on_schedule_due` reads the staged ownership flag
/// (default `false`).
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
    {
        let owned = state.schedule_due_owned.clone();
        m.expect_on_schedule_due().returning_st(move || owned.get());
    }
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
        m.expect_release_bus().returning_st(move || {
            ops.borrow_mut().push(TxBusOp::ReleaseBus);
        });
    }
    (m, state)
}

#[derive(Clone, Default)]
pub(crate) struct RxDmaState {
    remaining: Rc<Cell<u16>>,
    status_start_watched: Rc<Cell<bool>>,
}

impl RxDmaState {
    pub(crate) fn stage_remaining(&self, n: u16) {
        self.remaining.set(n);
    }

    /// Whether the per-byte status-start wake window is currently open.
    pub(crate) fn status_start_watched(&self) -> bool {
        self.status_start_watched.get()
    }
}

/// `MockRxDma` whose `remaining()` reads the staged counter; `read_and_ack`
/// returns quiet flags; the status-start watch toggles a readable flag.
pub(crate) fn mk_rx_dma() -> (MockRxDma, RxDmaState) {
    let state = RxDmaState::default();
    let mut m = MockRxDma::new();
    {
        let r = state.remaining.clone();
        m.expect_remaining().returning_st(move || r.get());
    }
    m.expect_read_and_ack().returning_st(DmaFlags::default);
    {
        let w = state.status_start_watched.clone();
        m.expect_watch_status_start()
            .returning_st(move || w.set(true));
    }
    {
        let w = state.status_start_watched.clone();
        m.expect_unwatch_status_start()
            .returning_st(move || w.set(false));
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
}

/// `MockFastLastScheduler` logging grid ops into its state companion, with
/// `patch_window_expired` reading the staged flag.
///
/// `deadline_passed` defaults to TRUE: test time can't advance inside the
/// final body's busy-fold, so a live-streaming tail would spin forever —
/// the default degenerates the spin to a single walk (fall through to the
/// completion CMP). Stage `false` to exercise in-spin behavior.
pub(crate) fn mk_fast_last() -> (MockFastLastScheduler, FastLastState) {
    let state = FastLastState::default();
    state.stage_deadline_passed(true);
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
        let dp = state.deadline_passed.clone();
        m.expect_deadline_passed().returning_st(move || dp.get());
    }
    {
        let ops = state.ops.clone();
        m.expect_schedule().returning_st(move |deadline| {
            ops.borrow_mut()
                .push(FastLastSchedulerOp::Schedule { deadline });
        });
    }
    {
        let pwe = state.patch_window_expired.clone();
        m.expect_patch_window_expired()
            .returning_st(move || pwe.get());
    }
    {
        let ops = state.ops.clone();
        m.expect_cancel().returning_st(move || {
            ops.borrow_mut().push(FastLastSchedulerOp::Cancel);
        });
    }
    (m, state)
}

/// Counter companion for `MockTelemetry` — the miss counter accumulates so
/// composite tests assert on observed telemetry.
#[derive(Clone, Default)]
pub(crate) struct TelemetryState {
    patch_misses: Rc<Cell<u32>>,
}

impl TelemetryState {
    pub(crate) fn patch_miss_count(&self) -> u32 {
        self.patch_misses.get()
    }
}

/// `MockTelemetry` bumping the state companion's counter.
pub(crate) fn mk_telemetry() -> (MockTelemetry, TelemetryState) {
    let state = TelemetryState::default();
    let mut m = MockTelemetry::new();
    {
        let c = state.patch_misses.clone();
        m.expect_record_crc_patch_deadline_miss()
            .returning_st(move || {
                c.set(c.get().wrapping_add(1));
            });
    }
    (m, state)
}

#[derive(Clone)]
pub(crate) struct WireClockState {
    now: Rc<Cell<u32>>,
}

impl WireClockState {
    pub(crate) fn stage_now(&self, now: u32) {
        self.now.set(now);
    }
}

impl Default for WireClockState {
    fn default() -> Self {
        Self {
            now: Rc::new(Cell::new(SEED_TICK as u32)),
        }
    }
}

/// `MockWireClock` reading the staged `now` — defaults to [`SEED_TICK`].
pub(crate) fn mk_wire_clock() -> (MockWireClock, WireClockState) {
    let state = WireClockState::default();
    let mut m = MockWireClock::new();
    {
        let n = state.now.clone();
        m.expect_now().returning_st(move || n.get());
    }
    (m, state)
}

// ------------------------------------------------------------------
// Wire builders
// ------------------------------------------------------------------

pub(crate) fn wire_ping(id: u8) -> heapless::Vec<u8, 32> {
    let mut buf = [0u8; 32];
    let n = encode_instruction::<SoftwareCrcUmts>(
        &mut buf,
        Id::new(id),
        Instruction::Ping.as_u8(),
        &[],
    )
    .unwrap();
    heapless::Vec::from_slice(&buf[..n]).unwrap()
}

pub(crate) fn wire_status(id: u8) -> heapless::Vec<u8, 32> {
    let mut buf = [0u8; 32];
    let n = encode_status::<SoftwareCrcUmts>(&mut buf, Id::new(id), StatusError::OK, &[]).unwrap();
    heapless::Vec::from_slice(&buf[..n]).unwrap()
}
