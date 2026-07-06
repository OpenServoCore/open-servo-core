//! Deadline provider (osc-native §4.1) — `now()` from SysTick's free-running
//! CNT, the compare from TIM2 CH4 (pin-less frozen OC, CC4IE interrupt).
//!
//! SysTick's own compare is NOT used: bench characterization showed its
//! CNTIF does not reliably latch on an upward CNT crossing (armed CMP
//! provably crossed with SR staying 0). TIM2's CC compares carried the DXL
//! scheduler for months at these timescales — same 48 MHz domain, and
//! `init_tim2_ch4_oc_kickoff`'s co-zero locks TIM2.CNT to SysTick's low 16
//! bits, so a u32 deadline truncates straight into CCR4.
//!
//! TIM2 is 16-bit: deadlines beyond [`HORIZON`] are reached by hopping —
//! an early wake is harmless by design (the mux due-checks against fresh
//! `now` and re-arms, §4.1).

use osc_drivers::traits::bus::{self, tick_reached};

use crate::hal::{systick, timer};

/// Hop distance for far deadlines: well under the 16-bit wrap so a hop can
/// never alias, comfortably over any near-deadline (683 µs at 48 MHz).
const HORIZON: u32 = 0x8000;

/// Re-aim lead when an arm lands behind the running counter (~330 ns).
const SET_RETRY_TICKS: u16 = 16;

/// Production binding: SysTick CNT (time) + TIM2 CC4 (compare) on HCLK.
pub struct Deadline;

impl bus::Deadline for Deadline {
    const TICKS_PER_US: u32 = systick::TICKS_PER_US;

    #[inline(always)]
    fn now(&self) -> u32 {
        systick::ticks()
    }

    fn set(&mut self, at: u32) {
        let now = systick::ticks();
        let delta = at.wrapping_sub(now);
        let ccr = if delta >= HORIZON {
            timer::tim2_cnt().wrapping_add(HORIZON as u16)
        } else {
            at as u16
        };
        timer::set_tim2_ccr4(ccr);
        // A stale match from a previous arm would fire the ISR immediately;
        // the mux would just re-arm, but clearing keeps wakes purposeful.
        timer::clear_tim2_cc4_flag();
        timer::set_tim2_cc4_irq(true);
        // TIM2 compare is equality-only too: if CNT slipped past `ccr`
        // between the reads (a past `at`, or a long preemption), re-aim a
        // hair ahead until the match is pending or provably in the future —
        // the DXL scheduler's recheck pattern.
        if delta < HORIZON {
            loop {
                let n2 = systick::ticks();
                if !tick_reached(n2, at) || timer::tim2_cc4_matched() {
                    break;
                }
                timer::set_tim2_ccr4((n2 as u16).wrapping_add(SET_RETRY_TICKS));
            }
        }
    }

    #[inline(always)]
    fn cancel(&mut self) {
        timer::set_tim2_cc4_irq(false);
    }
}
