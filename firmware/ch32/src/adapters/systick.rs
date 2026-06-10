//! SysTick adapter — provides `Monotonic` from the HCLK-driven SysTick counter.

use crate::drivers::traits::Monotonic;
use crate::hal::systick;

pub struct SysTick;

impl Monotonic for SysTick {
    const TICKS_PER_US: u32 = systick::TICKS_PER_US;

    #[inline(always)]
    fn ticks(&self) -> u32 {
        systick::ticks()
    }
}
