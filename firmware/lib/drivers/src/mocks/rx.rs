use mockall::mock;

use crate::traits::dxl::{DmaFlags, RxDma};

mock! {
    pub RxDma {}
    impl RxDma for RxDma {
        fn remaining(&self) -> u16;
        fn read_and_ack(&mut self) -> DmaFlags;
        fn record_edge_anchor_miss(&mut self);
    }
}
