//! Deadline provider -- SysTick's free-running low word for `now()`, the
//! 64-bit compare + STIE for the wake. The host is crystal-clocked (the
//! bus's syntonization root), so there is no trim constant here.
//!
//! Arm discipline is the servo's soak-proven one: CNTIF cleared before
//! every arm (no ghost wake from a stale latch), and a set that lands
//! at-or-behind the running counter is caught by the post-arm recheck and
//! converted into a PFIC software pend -- the engine due-checks every wake
//! against fresh `now`, so an extra wake is harmless.

use osc_host::traits::{self, tick_reached};

use crate::hal::{pfic, systick};

/// Production binding: SysTick CNT + CMP/STIE at HCLK/8 (18 MHz).
pub struct Deadline;

impl traits::Deadline for Deadline {
    const TICKS_PER_US: u32 = systick::TICKS_PER_US;

    #[inline(always)]
    fn now(&self) -> u32 {
        systick::ticks()
    }

    fn set(&mut self, at: u32) {
        systick::set_irq(false);
        systick::clear_match();
        systick::set_cmp_low(at);
        systick::set_irq(true);
        if tick_reached(systick::ticks(), at) && !systick::matched() {
            pfic::pend_systick();
        }
    }

    #[inline(always)]
    fn cancel(&mut self) {
        systick::set_irq(false);
    }
}
