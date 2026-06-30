use std::cell::RefCell;
use std::rc::Rc;

use osc_core::BaudRate;
use osc_drivers::mocks::{MockClockTrim, MockUsartBaud};

#[derive(Clone, Default)]
pub struct UsartBaudState {
    operations: Rc<RefCell<Vec<BaudRate>>>,
}

impl UsartBaudState {
    pub fn operations(&self) -> Vec<BaudRate> {
        self.operations.borrow().clone()
    }
}

/// Build a recording `MockUsartBaud` with the given per-baud RX edge-stamp
/// compensation, in driver `CLOCK_HZ` (= 48 MHz HCLK) ticks. Same value
/// returned regardless of baud — tests that exercise the compensation path
/// pick the value they want directly (e.g. `12` to mirror the V006 3 Mbaud
/// LUT entry), and tests that don't care pass `0` via [`mock_usart_baud`].
pub fn mock_usart_baud_with_comp(rx_edge_comp_ticks: u16) -> (MockUsartBaud, UsartBaudState) {
    let state = UsartBaudState::default();
    let mut m = MockUsartBaud::new();
    {
        let ops = state.operations.clone();
        m.expect_apply_baud().returning_st(move |baud| {
            ops.borrow_mut().push(baud);
        });
    }
    m.expect_rx_edge_comp_ticks()
        .returning_st(move |_baud| rx_edge_comp_ticks);
    (m, state)
}

/// Default `MockUsartBaud` with zero RX edge-stamp compensation — the
/// shape every existing timing test exercises, where the sim stamps at
/// wire-edge time and the driver subtracts nothing.
pub fn mock_usart_baud() -> (MockUsartBaud, UsartBaudState) {
    mock_usart_baud_with_comp(0)
}

#[derive(Clone, Default)]
pub struct ClockTrimState {
    operations: Rc<RefCell<Vec<i32>>>,
}

impl ClockTrimState {
    pub fn operations(&self) -> Vec<i32> {
        self.operations.borrow().clone()
    }
}

pub fn mock_clock_trim() -> (MockClockTrim, ClockTrimState) {
    let state = ClockTrimState::default();
    let mut m = MockClockTrim::new();
    {
        let ops = state.operations.clone();
        m.expect_apply_ppm().returning_st(move |ppm| {
            ops.borrow_mut().push(ppm);
        });
    }
    (m, state)
}
