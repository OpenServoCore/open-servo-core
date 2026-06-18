use mockall::mock;

use crate::traits::dxl::RxDma;

mock! {
    pub RxDma {}
    impl RxDma for RxDma {
        fn remaining(&self) -> u16;
        fn record_edge_anchor_miss(&mut self);
    }
}
