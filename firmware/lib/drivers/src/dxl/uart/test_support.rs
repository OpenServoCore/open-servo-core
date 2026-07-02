//! Shared consts + state-companion factories for the DXL UART unit-test
//! modules — one place to keep per-baud tick math and mock setups aligned.
//! Test-only.
//!
//! Not every item is consumed by every test module (composite mod.rs cares
//! about `SEED_TICK` + `TICKS_PER_BIT_3M`; clock.rs pulls the per-baud
//! `TICKS_PER_BIT_*`, `TICKS_PER_US`, and both factories), so
//! `#[allow(dead_code)]` keeps the crate lint clean without per-item
//! attributes.

#![allow(dead_code)]

extern crate alloc;

use std::cell::RefCell;
use std::rc::Rc;

use osc_core::BaudRate;

use crate::mocks::{MockClockTrim, MockUsartBaud};

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

/// `ticks_per_bit` at `BaudRate::B1000000`, HCLK 48 MHz.
pub(crate) const TICKS_PER_BIT_1M: u16 = 48;

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
/// wired to a constant `0`. Composite tests that don't care about the log
/// drop the state via `let (m, _) = mk_usart_baud()`.
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

/// `MockClockTrim` with a call log for `apply_ppm`. Composite tests that
/// don't care about the log drop the state via
/// `let (m, _) = mk_clock_trim()`.
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
