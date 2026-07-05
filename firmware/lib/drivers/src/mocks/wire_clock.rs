use mockall::mock;

use crate::traits::dxl::WireClock;

mock! {
    pub WireClock {}
    impl WireClock for WireClock {
        const PACKET_END_ENTRY_COMP_TICKS: u32 = 0;

        fn now(&self) -> u32;
    }
}
