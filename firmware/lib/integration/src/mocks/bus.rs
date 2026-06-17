use std::cell::RefCell;
use std::rc::Rc;

use osc_drivers::mocks::{MockTxBus, TxBusOp};

#[derive(Clone, Default)]
pub struct TxBusState {
    operations: Rc<RefCell<Vec<TxBusOp>>>,
}

impl TxBusState {
    pub fn operations(&self) -> Vec<TxBusOp> {
        self.operations.borrow().clone()
    }
}

pub fn mock_tx_bus() -> (MockTxBus, TxBusState) {
    let state = TxBusState::default();
    let mut m = MockTxBus::new();
    {
        let ops = state.operations.clone();
        m.expect_start_now().returning_st(move |byte_count| {
            ops.borrow_mut().push(TxBusOp::StartNow { byte_count });
        });
    }
    {
        let ops = state.operations.clone();
        m.expect_handle_start().returning_st(move || {
            ops.borrow_mut().push(TxBusOp::HandleStart);
        });
    }
    {
        let ops = state.operations.clone();
        m.expect_handle_tx_complete().returning_st(move || {
            ops.borrow_mut().push(TxBusOp::HandleTxComplete);
        });
    }
    (m, state)
}
