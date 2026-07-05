//! TX scheduler — long-horizon wire-fire scheduling. Driver hands an
//! absolute u32 deadline in the WireClock domain (= SysTick on V006);
//! provider applies a time-remaining decision tree:
//!
//! - **Direct hardware arm** — `remaining` fits a single TIM2 wrap. CCR4
//!   back-dates by `TX_KICKOFF_FLOOR_TICKS` so the wire bit lands at or
//!   after the deadline; the compare match fires the DMA1_CH7 kickoff
//!   with zero CPU in the deadline path, and CCR2 = deadline verbatim
//!   drives PC2 (TX_EN) via hardware OC
//!   (`docs/dxl-hw-timed-transport.md` §5, [`tx_kickoff`]). The §5.4
//!   set-and-recheck guard catches "armed in the past" and re-aims CCR4
//!   just ahead of CNT (+ TX_EN force-active) — a real compare event is
//!   the only trigger; SWEVGR.CC4G is dead silicon on V006.
//! - **SysTick handoff** — `remaining` exceeds the u16 comfort window
//!   (multi-wrap distance — broadcast Ping at high IDs, Fast Middle with
//!   many predecessor servos). Provider stashes the deadline, arms SysTick
//!   CMP at `deadline - SAFE_HORIZON_HANDOFF`; the SysTick ISR re-enters
//!   `apply_schedule` via `on_schedule_due`, which by construction lands
//!   in the direct-arm branch.
//!
//! For `SendKind::FastLast` within the direct-arm horizon the hardware
//! arms IMMEDIATELY — the wire start is then locked in regardless of CPU
//! state while the fold grid runs in parallel. Only the far-horizon case
//! stashes (the fold grid co-owns SysTick CMP during the predecessor
//! window, so a handoff here would clobber its CMP); `commit_pending`
//! runs the decision tree from the walk's final-anchor body, by which
//! point the remaining wait fits the direct arm at any supported baud's
//! grid geometry — and if not, the composite re-runs a fold body after
//! the handoff consumes the match.
//!
//! Wire-driver state (release at TC, sequence-driven `start_now` for
//! Plain chain k > 0) lives in the sibling
//! [`crate::providers::dxl_tx_bus::DxlTxBus`] per the `TxBus` trait split.

use ch32_metapac::USART1 as USART1_REGS;
use osc_drivers::traits::dxl::{SendKind, TxScheduler as TxSchedulerTrait};

use super::tx_kickoff;
use crate::hal::clocks::HCLK_HZ;
use crate::hal::{dma, systick, timer, usart};
use crate::measurements::{SCHEDULE_WRAP_GUARD_TICKS, TX_KICKOFF_FLOOR_TICKS};

/// Upper threshold for the direct-arm branch: `remaining ≤ FITS_U16`
/// means we can arm CCR4 single-wrap from now. Must stay strictly below
/// `SCHEDULE_WRAP_GUARD_TICKS` (with margin for the arm sequence itself)
/// so the §5.4 modular recheck is unambiguous: every direct-arm distance
/// reads `ccr4 − cnt < guard`, and only a genuinely crossed CCR4 wraps
/// past it. At the old 50 000 value, a legal 33k–50k future arm read as
/// "wrapped" and fired ASAP — a spurious early wire start.
const FITS_U16_COMFORTABLY: u32 = SCHEDULE_WRAP_GUARD_TICKS as u32 - 2_048;

/// Headroom between SysTick CMP match and TIM2 CCR4 match for the handoff
/// path. Sized to cover worst-case (preceding HIGH ISR body ≈ 255 ticks
/// classifier walk + SysTick PFIC entry ≈ 240 ticks + handoff body ≈ 50
/// ticks + TX_KICKOFF_FLOOR_TICKS): ≈ 553 ticks. Rounded up to 1024
/// (~21 µs at HCLK) for a comfortable margin. Baud-independent — purely a
/// chip-side latency budget. Must stay below `FITS_U16_COMFORTABLY` so the
/// SysTick handoff body's re-entry lands in the direct-arm branch.
const SAFE_HORIZON_HANDOFF: u32 = 1024;

#[derive(Default)]
pub struct DxlTxScheduler {
    /// Absolute u32 deadline + byte_count, set when `schedule` arms the
    /// SysTick handoff CMP. `on_schedule_due` consumes on match.
    handoff_stash: Option<(u32, u16)>,
    /// Absolute u32 deadline + byte_count, set when `schedule` is called
    /// with `SendKind::FastLast` beyond the direct-arm horizon.
    /// `commit_pending` consumes (called by composite from inside the
    /// FastLast walk's final-anchor body).
    fast_last_stash: Option<(u32, u16)>,
}

impl DxlTxScheduler {
    /// Run the time-remaining decision tree against an absolute SysTick
    /// deadline. Called from `schedule`, from `commit_pending` (far-horizon
    /// FastLast walk-end), and from `on_schedule_due` (SysTick handoff CMP
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
        // Single TIM2 wrap fits the wait — direct hardware arm. Past or
        // too-close deadlines are caught inside.
        self.arm_hw_kickoff(deadline as u16, byte_count, remaining);
    }

    fn arm_hw_kickoff(&mut self, deadline_u16: u16, byte_count: u16, remaining: i32) {
        let ccr4 = deadline_u16.wrapping_sub(TX_KICKOFF_FLOOR_TICKS);

        // DMA channel must be disabled before NDTR is written.
        dma::disable(dma::Channel::CH4);
        dma::set_count(dma::Channel::CH4, byte_count);
        usart::clear_tc(USART1_REGS);
        usart::set_dma_tx(USART1_REGS, true);
        usart::set_tc_irq(USART1_REGS, true);

        timer::set_tim2_ccr2(deadline_u16);
        timer::tim2_ch2_active_on_match();
        tx_kickoff::arm(ccr4);

        // Past deadline at entry, or §5.4 set-and-recheck: CNT crossed
        // CCR4 during the arm sequence — either way the next compare
        // match is a full wrap (~1.365 ms) away. Force TX_EN active (its
        // CC2 match is equally in the past) and re-aim CCR4 just ahead of
        // CNT so a real compare fires the already-armed kickoff ASAP.
        // The `remaining` term is load-bearing at low baud: an
        // IDLE-parsed 9600 frame lands here 40k+ ticks past its RDT
        // deadline, where the u16 modular recheck aliases back into the
        // "still ahead" band and would silently eat a wrap.
        let cnt = timer::tim2_cnt();
        if remaining <= 0 || cc4_arm_wrapped(cnt, ccr4, SCHEDULE_WRAP_GUARD_TICKS) {
            timer::tim2_ch2_force_active();
            tx_kickoff::trigger_asap();
        }
    }
}

impl TxSchedulerTrait for DxlTxScheduler {
    const TICKS_PER_US: u16 = (HCLK_HZ / 1_000_000) as u16;

    fn schedule(&mut self, deadline: u32, byte_count: u16, kind: SendKind) {
        match kind {
            SendKind::FastLast => {
                let remaining = deadline.wrapping_sub(systick::ticks()) as i32;
                if remaining > FITS_U16_COMFORTABLY as i32 {
                    // Far horizon: the FastLast walk co-owns SysTick CMP
                    // during the predecessor window — a handoff arm here
                    // would clobber the grid. Defer to `commit_pending`
                    // from the walk's final-anchor body.
                    self.fast_last_stash = Some((deadline, byte_count));
                } else {
                    // Within the direct-arm horizon: lock the wire start
                    // into hardware NOW — late grid bodies can no longer
                    // move it.
                    self.arm_hw_kickoff(deadline as u16, byte_count, remaining);
                }
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
        timer::tim2_ch2_force_inactive();
        usart::set_tc_irq(USART1_REGS, false);
        usart::set_dma_tx(USART1_REGS, false);
        dma::disable(dma::Channel::CH4);
        // Undo an armed kickoff window — parks CH7 disabled and wipes its
        // TC latch. No-op when nothing was hardware-armed.
        tx_kickoff::cancel();
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

/// True if TIM2 CNT has walked past CCR4 by more than `guard` — the next
/// CC4 match will be a full timer wrap (~1.365 ms at HCLK) away. The §5.4
/// set-and-recheck test after `arm_hw_kickoff` stages CCR4; when true the
/// caller re-aims CCR4 just ahead of CNT so the wire bit doesn't stall a
/// full wrap.
///
/// Reads as "modular ticks from CNT to CCR4." A small remaining means CNT
/// is still before CCR4 (arm succeeded); a large remaining (> half the u16
/// range) means CNT has just wrapped past CCR4.
fn cc4_arm_wrapped(cnt: u16, ccr4: u16, guard: u16) -> bool {
    ccr4.wrapping_sub(cnt) > guard
}

#[cfg(test)]
mod tests {
    use super::*;

    // Sized like production so guard-boundary tests exercise the same
    // half-u16 threshold the chip sees.
    const GUARD: u16 = 0x8000;

    #[test]
    fn arm_before_ccr4_within_guard_not_wrapped() {
        // CNT well before CCR4 by a small delta → still time to arm.
        assert!(!cc4_arm_wrapped(1000, 1500, GUARD));
    }

    #[test]
    fn arm_exactly_at_ccr4_not_wrapped() {
        // remaining = 0, 0 > GUARD is false. Boundary case of arming at
        // the exact match tick.
        assert!(!cc4_arm_wrapped(1500, 1500, GUARD));
    }

    #[test]
    fn arm_just_past_ccr4_is_wrapped() {
        // CNT walked one tick past CCR4 — modular remaining = 0xFFFF,
        // firmly above GUARD.
        assert!(cc4_arm_wrapped(1501, 1500, GUARD));
    }

    #[test]
    fn arm_at_guard_boundary_not_wrapped() {
        // remaining == GUARD, `>` not `>=`, so still considered armed.
        assert!(!cc4_arm_wrapped(0, GUARD, GUARD));
    }

    #[test]
    fn arm_one_tick_past_guard_boundary_is_wrapped() {
        // remaining == GUARD + 1 crosses the threshold.
        assert!(cc4_arm_wrapped(0, GUARD + 1, GUARD));
    }
}
