//! Deadline provider (osc-native §4.1) — binds `Deadline` to the SysTick
//! compare. SysTick's free-running CNT is also the `Monotonic` time source
//! for the LED; the transport owns only the CMP + STIE compare on top of it.

use osc_drivers::traits::bus;

use crate::hal::systick;

/// Production binding to the SysTick compare on HCLK (48 MHz).
pub struct Deadline;

impl bus::Deadline for Deadline {
    const TICKS_PER_US: u32 = systick::TICKS_PER_US;

    #[inline(always)]
    fn now(&self) -> u32 {
        systick::ticks()
    }

    #[inline(always)]
    fn set(&mut self, at: u32) {
        systick::set_cmp(at);
        systick::set_irq(true);
    }

    #[inline(always)]
    fn cancel(&mut self) {
        systick::set_irq(false);
    }
}
