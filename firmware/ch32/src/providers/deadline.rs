//! Deadline provider (osc-native §4.1) — binds `Deadline` to the SysTick
//! compare. SysTick's free-running CNT is also the `Monotonic` time source
//! for the LED; the transport owns only the CMP + STIE compare on top of it.

use osc_drivers::traits::bus;

use crate::hal::systick;

/// Re-aim distance when a compare write lands behind the running counter:
/// far enough that the next write beats the counter, close enough to keep
/// the wake effectively immediate (~330 ns at 48 MHz).
const SET_RETRY_TICKS: u32 = 16;

/// Production binding to the SysTick compare on HCLK (48 MHz).
pub struct Deadline;

impl bus::Deadline for Deadline {
    const TICKS_PER_US: u32 = systick::TICKS_PER_US;

    #[inline(always)]
    fn now(&self) -> u32 {
        systick::ticks()
    }

    fn set(&mut self, at: u32) {
        // SysTick's compare is equality-only: a CMP the counter has already
        // passed (a stale slot re-armed by the mux, or CNT crossing `at`
        // during these writes) never matches until the 89 s wrap. Re-aim a
        // hair ahead until the target is provably in the future or the match
        // already latched — the ISR due-checks against the mux slots, so a
        // slightly-late wake is always correct (§4.1 wakes are verified).
        crate::log::trace!("dl.set at={}", at);
        let mut target = at;
        loop {
            systick::set_cmp(target);
            let now = systick::ticks();
            if !bus::tick_reached(now, target) || systick::matched() {
                break;
            }
            target = now.wrapping_add(SET_RETRY_TICKS);
        }
        systick::set_irq(true);
    }

    #[inline(always)]
    fn cancel(&mut self) {
        crate::log::trace!("dl.cancel");
        systick::set_irq(false);
    }
}
