use mockall::mock;

use crate::traits::dxl::RxDma;

mock! {
    pub RxDma {}
    impl RxDma for RxDma {
        fn remaining(&self) -> u16;
    }
}
