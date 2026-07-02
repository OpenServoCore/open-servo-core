//! TX scheduler — long-horizon wire-fire scheduling. Driver hands an
//! absolute u32 deadline in the WireClock domain (= SysTick on V006);
//! provider applies a time-remaining decision tree:
//!
//! - **Direct CC3 arm** — `remaining` fits a single TIM2 wrap. CCR3 back-dates
//!   by `TX_START_ENTRY_TICKS` to absorb the ISR+DMA+USART path so the wire
//!   bit lands at or after deadline; CCR2 = deadline verbatim drives PC2
//!   (TX_EN) via hardware OC (`docs/dxl-hw-timed-transport.md` §5). The §5.4
//!   set-and-recheck guard catches "armed in the past" and falls through to
//!   the software-fire path.
//! - **SysTick handoff** — `remaining` exceeds the u16 comfort window
//!   (multi-wrap distance — broadcast Ping at high IDs, Fast Middle with
//!   many predecessor servos). Provider stashes the deadline, arms SysTick
//!   CMP at `deadline - SAFE_HORIZON_HANDOFF`; the SysTick ISR re-enters
//!   `apply_schedule` via `on_schedule_due`, which by construction lands
//!   in the direct-CC3 branch.
//! - **Software fire** — `remaining` ≤ ~PFIC+body budget (already late, or
//!   intentional walk-end commit on Fast Last). Force CCR2 active + kick
//!   DMA inline; wire bit lands ~1 µs later.
//!
//! For `SendKind::FastLast` the schedule is *stashed* — the Fast Last walk
//! co-owns SysTick CMP during the predecessor window, so `commit_pending`
//! is the composite-driven trigger that runs the decision tree (composite
//! invokes it from inside the walk's final-anchor body, before busy-wait).
//!
//! Wire-driver state (activate at CC3, release at TC, sequence-driven
//! `start_now` for Plain chain k > 0) lives in the sibling
//! [`crate::providers::dxl_tx_bus::DxlTxBus`] per the `TxBus` trait split.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits::dxl::{SendKind, TxScheduler as TxSchedulerTrait};

use crate::hal::clocks::HCLK_HZ;
use crate::hal::{dma, systick, timer, usart};
use crate::measurements::{SCHEDULE_WRAP_GUARD_TICKS, TX_START_ENTRY_TICKS};

/// Upper threshold for the direct-CC3-arm branch: `remaining ≤ FITS_U16`
/// means we can arm CCR3 single-wrap from now. Below TIM2's 65 536-tick
/// wrap (1.365 ms at HCLK) with ~15k ticks of headroom against jitter.
const FITS_U16_COMFORTABLY: u32 = 50_000;

/// Headroom between SysTick CMP match and TIM2 CCR3 match for the handoff
/// path. Sized to cover worst-case (preceding HIGH ISR body ≈ 255 ticks
/// classifier walk + SysTick PFIC entry ≈ 240 ticks + handoff body ≈ 50
/// ticks + TX_START_ENTRY_TICKS = 46): ≈ 591 ticks. Rounded up to 1024
/// (~21 µs at HCLK) for a comfortable margin. Baud-independent — purely a
/// chip-side latency budget. Must stay below `FITS_U16_COMFORTABLY` so the
/// SysTick handoff body's re-entry lands in the direct-CC3 branch.
const SAFE_HORIZON_HANDOFF: u32 = 1024;

/// Slack between `remaining` and `TX_START_ENTRY_TICKS` below which the
/// decision tree drops to software-fire — leaves enough headroom for the
/// schedule body itself plus the §5.4 recheck. Same order as the recheck's
/// `SCHEDULE_WRAP_GUARD_TICKS` margin.
const IMMEDIATE_GUARD: u32 = 64;

#[derive(Default)]
pub struct DxlTxScheduler {
    /// Absolute u32 deadline + byte_count, set when `schedule` arms the
    /// SysTick handoff CMP. `on_schedule_due` consumes on match.
    handoff_stash: Option<(u32, u16)>,
    /// Absolute u32 deadline + byte_count, set when `schedule` is called
    /// with `SendKind::FastLast`. `commit_pending` consumes (called by
    /// composite from inside the FastLast walk's final-anchor body).
    fast_last_stash: Option<(u32, u16)>,
}

impl DxlTxScheduler {
    /// Run the time-remaining decision tree against an absolute SysTick
    /// deadline. Called from `schedule` (Plain), from `commit_pending`
    /// (FastLast walk-end), and from `on_schedule_due` (SysTick handoff CMP
    /// fired).
    fn apply_schedule(&mut self, deadline: u32, byte_count: u16) {
        let now = systick::ticks();
        let remaining = deadline.wrapping_sub(now) as i32;
        if remaining > FITS_U16_COMFORTABLY as i32 {
            // Multi-wrap distance — hand off via SysTick CMP. Body will
            // re-enter `apply_schedule` via `on_schedule_due`.
            self.handoff_stash = Some((deadline, byte_count));
            systick::clear_match();
            systick::set_cmp(deadline.wrapping_sub(SAFE_HORIZON_HANDOFF));
            systick::set_irq(true);
            return;
        }
        if remaining > (TX_START_ENTRY_TICKS as u32 + IMMEDIATE_GUARD) as i32 {
            // Single TIM2 wrap fits the wait — direct CC3 arm.
            self.arm_tim2(deadline as u16, byte_count);
            return;
        }
        // Past or too close for the hardware-OC path — software-fire.
        self.software_fire(byte_count);
    }

    fn arm_tim2(&mut self, deadline_u16: u16, byte_count: u16) {
        let ccr3 = deadline_u16.wrapping_sub(TX_START_ENTRY_TICKS);
        let ccr2 = deadline_u16;

        // DMA channel must be disabled before NDTR is written.
        dma::disable(dma::Channel::CH4);
        dma::set_count(dma::Channel::CH4, byte_count);
        usart::clear_tc(USART1_REGS);
        usart::set_dma_tx(USART1_REGS, true);
        usart::set_tc_irq(USART1_REGS, true);

        timer::set_tim2_ccr3(ccr3);
        timer::set_tim2_ccr2(ccr2);
        timer::tim2_ch2_active_on_match();
        timer::clear_tim2_cc3_flag();
        timer::enable_tim2_cc3_irq(true);

        // §5.4 set-and-recheck: if CNT just passed CCR3, the next CC3IF is a
        // full wrap (~1.365 ms) away. Detect modular underflow → start-now:
        // force TX_EN active, mask CC3 IRQ so the late wrap-around match
        // doesn't double-fire, kick DMA, clear the stale CC3 flag.
        let cnt = timer::tim2_cnt();
        if cc3_arm_wrapped(cnt, ccr3, SCHEDULE_WRAP_GUARD_TICKS) {
            self.software_fire(byte_count);
            // Software fire already configured DMA — undo the CC3IE arm so
            // the eventual wrap-around match doesn't double-fire.
            timer::enable_tim2_cc3_irq(false);
            timer::clear_tim2_cc3_flag();
        } else {
            crate::tune::record_schedule_remaining(ccr3.wrapping_sub(cnt));
        }
    }

    fn software_fire(&mut self, byte_count: u16) {
        // DMA channel must be disabled before NDTR is written.
        dma::disable(dma::Channel::CH4);
        dma::set_count(dma::Channel::CH4, byte_count);
        usart::clear_tc(USART1_REGS);
        usart::set_dma_tx(USART1_REGS, true);
        usart::set_tc_irq(USART1_REGS, true);

        timer::tim2_ch2_force_active();
        dma::enable(dma::Channel::CH4);
    }
}

impl TxSchedulerTrait for DxlTxScheduler {
    const TICKS_PER_US: u16 = (HCLK_HZ / 1_000_000) as u16;

    fn schedule(&mut self, deadline: u32, byte_count: u16, kind: SendKind) {
        match kind {
            SendKind::FastLast => {
                // FastLast walk co-owns SysTick CMP during the predecessor
                // window — defer arming until `commit_pending` runs from
                // inside the walk's final-anchor body.
                self.fast_last_stash = Some((deadline, byte_count));
            }
            SendKind::Plain => {
                self.apply_schedule(deadline, byte_count);
            }
        }
    }

    fn commit_pending(&mut self) {
        if let Some((deadline, byte_count)) = self.fast_last_stash.take() {
            self.apply_schedule(deadline, byte_count);
        }
    }

    fn cancel(&mut self) {
        timer::enable_tim2_cc3_irq(false);
        timer::tim2_ch2_force_inactive();
        usart::set_tc_irq(USART1_REGS, false);
        usart::set_dma_tx(USART1_REGS, false);
        dma::disable(dma::Channel::CH4);
        timer::clear_tim2_cc3_flag();
        // Drop both stashes — cancel covers both deferred FastLast and an
        // armed handoff. The SysTick CMP IRQ stays armed (FastLast walk
        // co-owns it); a spurious match while no stash is present will
        // return false from `on_schedule_due` and route to the walk.
        self.handoff_stash = None;
        self.fast_last_stash = None;
    }

    fn on_schedule_due(&mut self) -> bool {
        if let Some((deadline, byte_count)) = self.handoff_stash.take() {
            self.apply_schedule(deadline, byte_count);
            return true;
        }
        false
    }
}

/// True if TIM2 CNT has walked past CCR3 by more than `guard` — the next
/// CC3 match will be a full timer wrap (~1.365 ms at HCLK) away. The §5.4
/// set-and-recheck test after `arm_tim2` stages CCR3; when true the caller
/// falls through to the software-fire path so the wire bit doesn't stall a
/// full wrap.
///
/// Reads as "modular ticks from CNT to CCR3." A small remaining means CNT
/// is still before CCR3 (arm succeeded); a large remaining (> half the u16
/// range) means CNT has just wrapped past CCR3.
fn cc3_arm_wrapped(cnt: u16, ccr3: u16, guard: u16) -> bool {
    ccr3.wrapping_sub(cnt) > guard
}

#[cfg(test)]
mod tests {
    use super::*;

    // Sized like production so guard-boundary tests exercise the same
    // half-u16 threshold the chip sees.
    const GUARD: u16 = 0x8000;

    #[test]
    fn arm_before_ccr3_within_guard_not_wrapped() {
        // CNT well before CCR3 by a small delta → still time to arm.
        assert!(!cc3_arm_wrapped(1000, 1500, GUARD));
    }

    #[test]
    fn arm_exactly_at_ccr3_not_wrapped() {
        // remaining = 0, 0 > GUARD is false. Boundary case of arming at
        // the exact match tick.
        assert!(!cc3_arm_wrapped(1500, 1500, GUARD));
    }

    #[test]
    fn arm_just_past_ccr3_is_wrapped() {
        // CNT walked one tick past CCR3 — modular remaining = 0xFFFF,
        // firmly above GUARD.
        assert!(cc3_arm_wrapped(1501, 1500, GUARD));
    }

    #[test]
    fn arm_at_guard_boundary_not_wrapped() {
        // remaining == GUARD, `>` not `>=`, so still considered armed.
        assert!(!cc3_arm_wrapped(0, GUARD, GUARD));
    }

    #[test]
    fn arm_one_tick_past_guard_boundary_is_wrapped() {
        // remaining == GUARD + 1 crosses the threshold.
        assert!(cc3_arm_wrapped(0, GUARD + 1, GUARD));
    }
}
