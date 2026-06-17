use mockall::mock;
use osc_core::BaudRate;

use crate::traits::dxl::{ClockTrim, UsartBaud};

mock! {
    pub UsartBaud {}
    impl UsartBaud for UsartBaud {
        // Same as the production V006 binding (PCLK = HCLK = 48 MHz) so driver
        // ticks_per_bit math matches the chip-side reference table.
        const CLOCK_HZ: u32 = 48_000_000;

        fn apply_baud(&mut self, baud: BaudRate);
    }
}

mock! {
    pub ClockTrim {}
    impl ClockTrim for ClockTrim {
        // Same ratio as the real HSI so existing drift/threshold tests stay
        // numerically aligned without baking in chip-specific imports.
        const DELTA_MIN: i8 = -16;
        const DELTA_MAX: i8 = 15;
        const HZ: u32 = 24_000_000;
        const STEP_HZ: u32 = 60_000;

        fn apply_delta(&mut self, delta: i8);
    }
}
