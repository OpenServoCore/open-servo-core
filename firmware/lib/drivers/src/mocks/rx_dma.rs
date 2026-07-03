use mockall::mock;

use crate::traits::dxl::{DmaFlags, RxDma};

mock! {
    pub RxDma {}
    impl RxDma for RxDma {
        fn remaining(&self) -> u16;
        fn read_and_ack(&mut self) -> DmaFlags;
        fn watch_status_start(&mut self);
        fn unwatch_status_start(&mut self);
    }
}
