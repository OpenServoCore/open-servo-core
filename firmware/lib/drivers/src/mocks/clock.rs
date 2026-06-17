use std::vec::Vec;

use osc_core::BaudRate;

use crate::traits::dxl::{ClockTrim, UsartBaud};

#[derive(Default)]
pub struct MockUsartBaud {
    pub log: Vec<BaudRate>,
}

impl UsartBaud for MockUsartBaud {
    // Same as the production V006 binding (PCLK = HCLK = 48 MHz) so driver
    // ticks_per_bit math matches the chip-side reference table.
    const CLOCK_HZ: u32 = 48_000_000;

    fn apply_baud(&mut self, baud: BaudRate) {
        self.log.push(baud);
    }
}

#[derive(Default)]
pub struct MockClockTrim {
    pub log: Vec<i8>,
}

impl ClockTrim for MockClockTrim {
    // Same ratio as the real HSI so existing drift/threshold tests stay
    // numerically aligned without baking in chip-specific imports.
    const DELTA_MIN: i8 = -16;
    const DELTA_MAX: i8 = 15;
    const HZ: u32 = 24_000_000;
    const STEP_HZ: u32 = 60_000;

    fn apply_delta(&mut self, delta: i8) {
        self.log.push(delta);
    }
}
