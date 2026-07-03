use mockall::mock;

use crate::traits::dxl::TxBus;

/// One entry per [`TxBus`] call; downstream test harnesses (e.g. the
/// integration crate's spy state) use this as the recorded shape.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TxBusOp {
    StartNow { byte_count: u16 },
    TakeBus,
    ReleaseBus,
}

mock! {
    pub TxBus {}
    impl TxBus for TxBus {
        fn start_now(&mut self, byte_count: u16);
        fn take_bus(&mut self);
        fn release_bus(&mut self);
    }
}
