//! Deadline provider (protocol sec 4.1) -- SysTick's free-running CNT for
//! `now()`, its 32-bit CMP + STIE compare for the wake.
//!
//! The compare is EQUALITY-ONLY: CNTIF latches when CNT counts up through
//! CMP exactly; a CMP the counter has already passed never matches until
//! the ~89 s wrap. Two disciplines make that safe (both long soak-proven):
//! CNTIF is cleared before every arm so a stale latch can't fire a ghost
//! wake, and a set that lands behind the running counter -- the CMP store
//! races CNT by a few HCLK cycles -- is caught by the post-arm recheck and
//! converted into a PFIC software pend (`pend_systick`), which dispatches
//! the same vector without needing the comparator. The mux due-checks every
//! wake against fresh `now` (protocol sec 4.1), so an extra wake is harmless.

use osc_drivers::traits::bus::{self, tick_reached};

use crate::hal::{pfic, systick};

/// Production binding: SysTick CNT + CMP/STIE on HCLK (48 MHz).
pub struct Deadline;

impl bus::Deadline for Deadline {
    const TICKS_PER_US: u32 = systick::TICKS_PER_US;
    const CLOCK_TRIM_STEP_PPM: u32 = crate::hal::rcc::CLOCK_TRIM_PPM_PER_STEP;

    #[inline(always)]
    fn now(&self) -> u32 {
        systick::ticks()
    }

    fn set(&mut self, at: u32) {
        crate::log::trace!("dl.set at={}", at);
        systick::set_irq(false);
        systick::clear_match();
        systick::set_cmp(at);
        systick::set_irq(true);
        if tick_reached(systick::ticks(), at) && !systick::matched() {
            pfic::pend_systick();
        }
    }

    #[inline(always)]
    fn cancel(&mut self) {
        crate::log::trace!("dl.cancel");
        systick::set_irq(false);
    }
}
