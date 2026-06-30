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
        fn rx_edge_comp_ticks(&self, baud: BaudRate) -> u16;
    }
}

mock! {
    pub ClockTrim {}
    impl ClockTrim for ClockTrim {
        // Same envelope + step as the production V006 binding so existing
        // drift/threshold tests stay numerically aligned without baking in
        // chip-specific imports. 60 kHz / 24 MHz HSI = 2500 ppm/step;
        // envelope = -16/+15 trim register units × 2500 ppm/step.
        const STEP_PPM: u32 = 2500;
        const ENVELOPE_PPM: (i32, i32) = (-40_000, 37_500);

        fn apply_ppm(&mut self, ppm: i32);
    }
}
