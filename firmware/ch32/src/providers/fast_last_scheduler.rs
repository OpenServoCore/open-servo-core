//! Fast Last catchup scheduler — SysTick CMP fires the catchup ISR at
//! fixed byte intervals during a Fast Sync / Bulk Read predecessor window.
//! Per `docs/dxl-hw-timed-transport.md` §10.6 + §12: TIM2 is reserved for
//! jitter-critical wire-edge events (CC4 IC, CC3 fire, CC2 TX_EN OC);
//! long-horizon catchup scheduling rides SysTick because the grid step at
//! low baud (`15 × byte_ticks`) can exceed TIM2's 16-bit / 1.365 ms wrap.
//!
//! Driver passes absolute u32 deadlines (WireClock domain = SysTick on
//! V006); provider stores them and arms CMP directly with no lift.

use dxl_protocol::wire::CRC_BYTES;
use osc_drivers::traits::dxl::FastLastScheduler as FastLastSchedulerTrait;

use crate::hal::{dma, systick};
use crate::measurements::{
    FAST_LAST_BYTES_PER_INTERVAL, FAST_LAST_ENTRY_TICKS, FAST_LAST_GUARD_BYTES,
};
use crate::runtime::statics::SHARED;

#[derive(Default)]
pub struct FastLastScheduler {
    deadline: u32,
}

impl FastLastSchedulerTrait for FastLastScheduler {
    const FAST_LAST_ENTRY_TICKS: u16 = FAST_LAST_ENTRY_TICKS;
    const BYTES_PER_INTERVAL: u16 = FAST_LAST_BYTES_PER_INTERVAL;
    const GUARD_BYTES: u16 = FAST_LAST_GUARD_BYTES;

    fn set_deadline(&mut self, deadline: u32) {
        self.deadline = deadline;
    }

    fn schedule(&mut self, deadline: u32) {
        // Past-CMP guard: low RDT + small predecessor counts can land the
        // back-dated CMP behind SysTick CNT. SysTick wouldn't match until
        // u32 wrap (~89 s) — clamp to "just past now" so the body fires
        // ASAP. Body's first run lands ENTRY ticks late; grid step
        // advances normally from there.
        let now = systick::ticks();
        let safe_cmp = if (deadline.wrapping_sub(now) as i32) < 0 {
            now.wrapping_add(1)
        } else {
            deadline
        };
        systick::clear_match();
        systick::set_cmp(safe_cmp);
        systick::set_irq(true);
    }

    fn deadline_passed(&self) -> bool {
        (systick::ticks().wrapping_sub(self.deadline) as i32) >= 0
    }

    fn patch_window_expired(&self) -> bool {
        // CH4 has prefetched into (or past) the trailing CRC slot — any
        // patch_crc write into tx_buf[len-CRC_BYTES..len] races CH4's read.
        dma::remaining(dma::Channel::CH4) <= CRC_BYTES as u16
    }

    fn record_patch_deadline_miss(&mut self) {
        // Volatile RMW into the telemetry region — same pattern as the
        // `sample_tick` increment in `runtime/isr.rs`. Concurrent host clear
        // (via DXL bus write to `crc_patch_deadline_miss`) can drop one
        // update per race window, which the region's `rw` declaration
        // explicitly accepts (`telemetry.rs:72-74`).
        // SAFETY: SHARED is the canonical chip-side storage for the
        // control-table region; writers other than this one only fire from
        // DXL-side ISRs at the same PFIC priority, so the RMW is atomic
        // w.r.t. itself.
        unsafe {
            let p = &raw mut (*SHARED.table.telemetry.get()).link.crc_patch_deadline_miss;
            p.write_volatile(p.read_volatile().wrapping_add(1));
        }
    }

    fn cancel(&mut self) {
        systick::set_irq(false);
        systick::set_cmp(u32::MAX);
        systick::clear_match();
    }
}
