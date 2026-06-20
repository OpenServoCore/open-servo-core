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

pub fn mock_usart_baud() -> (MockUsartBaud, UsartBaudState) {
    let state = UsartBaudState::default();
    let mut m = MockUsartBaud::new();
    {
        let ops = state.operations.clone();
        m.expect_apply_baud().returning_st(move |baud| {
            ops.borrow_mut().push(baud);
        });
    }
    (m, state)
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
