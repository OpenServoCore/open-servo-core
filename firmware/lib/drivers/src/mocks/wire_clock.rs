use mockall::mock;

use crate::traits::dxl::WireClock;

mock! {
    pub WireClock {}
    impl WireClock for WireClock {
        fn now(&self) -> u16;
    }
}
