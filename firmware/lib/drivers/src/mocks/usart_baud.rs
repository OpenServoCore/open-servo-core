use mockall::mock;
use osc_core::BaudRate;

use crate::traits::dxl::UsartBaud;

mock! {
    pub UsartBaud {}
    impl UsartBaud for UsartBaud {
        // Same as the production V006 binding (PCLK = HCLK = 48 MHz) so driver
        // ticks_per_bit math matches the chip-side reference table.
        const CLOCK_HZ: u32 = 48_000_000;

        fn apply_baud(&mut self, baud: BaudRate);
        fn rx_edge_comp_ticks(&self, baud: BaudRate) -> u16;
    }
}
