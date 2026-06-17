use std::cell::{Cell, RefCell};
use std::rc::Rc;

use osc_drivers::Level;
use osc_drivers::mocks::{MockDigitalOut, MockMonotonic};

#[derive(Clone, Default)]
pub struct DigitalOutState {
    operations: Rc<RefCell<Vec<Level>>>,
}

impl DigitalOutState {
    pub fn operations(&self) -> Vec<Level> {
        self.operations.borrow().clone()
    }
}

pub fn mock_digital_out() -> (MockDigitalOut, DigitalOutState) {
    let state = DigitalOutState::default();
    let mut m = MockDigitalOut::new();
    {
        let ops = state.operations.clone();
        m.expect_set().returning_st(move |level| {
            ops.borrow_mut().push(level);
        });
    }
    (m, state)
}

#[derive(Clone, Default)]
pub struct MonotonicState {
    now: Rc<Cell<u32>>,
}

impl MonotonicState {
    pub fn stage_now(&self, n: u32) {
        self.now.set(n);
    }
}

pub fn mock_monotonic() -> (MockMonotonic, MonotonicState) {
    let state = MonotonicState::default();
    let mut m = MockMonotonic::new();
    {
        let now = state.now.clone();
        m.expect_ticks().returning_st(move || now.get());
    }
    (m, state)
}
