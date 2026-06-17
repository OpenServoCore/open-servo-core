use std::vec::Vec;

use crate::traits::dxl::TxBus;

/// One entry per [`TxBus`] call; tests assert the recorded sequence
/// against expected wire-driver activity.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TxBusOp {
    StartNow { byte_count: u16 },
    HandleStart,
    HandleTxComplete,
}

#[derive(Default)]
pub struct MockTxBus {
    pub log: Vec<TxBusOp>,
}

impl TxBus for MockTxBus {
    fn start_now(&mut self, byte_count: u16) {
        self.log.push(TxBusOp::StartNow { byte_count });
    }

    fn handle_start(&mut self) {
        self.log.push(TxBusOp::HandleStart);
    }

    fn handle_tx_complete(&mut self) {
        self.log.push(TxBusOp::HandleTxComplete);
    }
}
