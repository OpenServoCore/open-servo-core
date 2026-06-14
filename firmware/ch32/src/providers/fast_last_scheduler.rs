//! Fast Last catchup scheduler — SysTick CMP fires the catchup ISR at
//! fixed byte intervals during a Fast Sync / Bulk Read predecessor window.
//! Per `docs/dxl-hw-timed-transport.md` §10.6 + §12: TIM2 is reserved for
//! jitter-critical wire-edge events (CC4 IC, CC3 fire, CC2 TX_EN OC);
//! long-horizon catchup scheduling rides SysTick because the grid step at
//! low baud (`15 × byte_ticks`) can exceed TIM2's 16-bit / 1.365 ms wrap.
//!
//! The scheduler caches the parser-derived `packet_end_tick` lifted into
//! SysTick's u32 domain, plus the busy-wait exit deadline as a u32 SysTick
//! value. The lift uses the boot-captured TIM2 ↔ SysTick offset: both
//! clocks tick at HCLK so the offset is fixed; reading both back-to-back
//! at HIGH ISR priority (no preemption) recovers the upper bits via the
//! most-recent-TIM2-wrap window. See doc §12.

use osc_drivers::traits::dxl::FastLastScheduler as FastLastSchedulerTrait;

use crate::hal::{systick, timer};
use crate::measurements::{
    FAST_LAST_BYTES_PER_INTERVAL, FAST_LAST_ENTRY_TICKS, FAST_LAST_GUARD_BYTES,
};

#[derive(Default)]
pub struct FastLastScheduler {
    packet_end_lifted: u32,
    deadline_lifted: u32,
}

impl FastLastSchedulerTrait for FastLastScheduler {
    const FAST_LAST_ENTRY_TICKS: u16 = FAST_LAST_ENTRY_TICKS;
    const BYTES_PER_INTERVAL: u16 = FAST_LAST_BYTES_PER_INTERVAL;
    const GUARD_BYTES: u16 = FAST_LAST_GUARD_BYTES;

    fn set_deadline(&mut self, packet_end_tick: u16, deadline_ticks: u32) {
        // Read both clocks back-to-back. HIGH ISR priority + no preemption
        // means the inter-read skew is bounded by a couple of HCLK cycles,
        // negligible against the ~5 µs PFIC entry FastLast budgets for.
        let systick_now = systick::ticks();
        let tim2_now = timer::tim2_cnt();
        // packet_end_tick was captured in TIM2 at some past T. Modular
        // u16 sub gives the delta-from-now in HCLK ticks (≤1 wrap = 1.365
        // ms — caller arms immediately after parsing so this holds).
        let delta = tim2_now.wrapping_sub(packet_end_tick) as u32;
        self.packet_end_lifted = systick_now.wrapping_sub(delta);
        self.deadline_lifted = self.packet_end_lifted.wrapping_add(deadline_ticks);
    }

    fn schedule(&mut self, offset_ticks: u32) {
        let cmp = self.packet_end_lifted.wrapping_add(offset_ticks);
        // Past-CMP guard: low RDT + small predecessor counts can land the
        // back-dated CMP behind SysTick CNT. SysTick wouldn't match until
        // u32 wrap (~89 s) — clamp to "just past now" so the body fires
        // ASAP. Body's first run lands ENTRY ticks late; grid step
        // advances normally from there.
        let now = systick::ticks();
        let safe_cmp = if (cmp.wrapping_sub(now) as i32) < 0 {
            now.wrapping_add(1)
        } else {
            cmp
        };
        systick::clear_match();
        systick::set_cmp(safe_cmp);
        systick::set_irq(true);
    }

    fn deadline_passed(&self) -> bool {
        (systick::ticks().wrapping_sub(self.deadline_lifted) as i32) >= 0
    }

    fn cancel(&mut self) {
        systick::set_irq(false);
        systick::set_cmp(u32::MAX);
        systick::clear_match();
    }
}
