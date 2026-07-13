//! Monotonic provider -- supplies `Monotonic` from HCLK-driven SysTick.

use osc_drivers::traits;

use crate::hal::systick;

pub struct Monotonic;

impl traits::Monotonic for Monotonic {
    const TICKS_PER_US: u32 = systick::TICKS_PER_US;

    #[inline(always)]
    fn ticks(&self) -> u32 {
        systick::ticks()
    }
}
