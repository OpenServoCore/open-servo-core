//! Fast Last catchup scheduler — SysTick CMP triggers the catchup ISR at
//! fixed byte intervals during a Fast Sync / Bulk Read predecessor window.
//! Per `docs/dxl-hw-timed-transport.md` §10.6 + §12: TIM2 is reserved for
//! jitter-critical wire-edge events (CC4 IC, CC3 TX start, CC2 TX_EN OC);
//! long-horizon catchup scheduling rides SysTick instead. TIM2's shared
//! prescaler is pinned at PSC=0 for the IC side's 16-tick resolution at
//! 3M, so its 16-bit CNT wraps every 1.365 ms — the grid step at low baud
//! (`15 × byte_ticks`) can exceed that and the wire-start deadline can be
//! many wraps out. SysTick is 32-bit at HCLK with ~89.5 s horizon, a
//! separate IRQ vector from TIM2, and its ~5 µs PFIC-entry jitter is
//! dwarfed by the fold body cost.
//!
//! Driver passes absolute u32 deadlines (WireClock domain = SysTick on
//! V006); provider stores them and sets CMP directly with no lift.

use dxl_protocol::wire::CRC_BYTES;
use osc_drivers::traits::dxl::FastLastScheduler as FastLastSchedulerTrait;

use crate::hal::{dma, pfic, systick};
use crate::measurements::{
    FAST_LAST_BYTES_PER_INTERVAL, FAST_LAST_ENTRY_TICKS, FAST_LAST_GUARD_BYTES,
};
use crate::runtime::statics::SHARED;

#[derive(Default)]
pub struct FastLastScheduler {
    deadline: u32,
}

/// DEBUG-ONLY: ring of last 8 schedule() captures, written via volatile RMW.
/// Read via wlink after a wedge — `nm` the symbols for addresses.
/// Removed once the wedge investigation lands.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct FlSchedSample {
    pub now: u32,
    pub deadline: u32,
    pub diff: i32,
    pub safe_cmp: u32,
}

#[unsafe(no_mangle)]
pub static mut FL_SCHED_RING: [FlSchedSample; 8] = [FlSchedSample {
    now: 0,
    deadline: 0,
    diff: 0,
    safe_cmp: 0,
}; 8];

#[unsafe(no_mangle)]
pub static mut FL_SCHED_HEAD: u32 = 0;

impl FastLastSchedulerTrait for FastLastScheduler {
    const FAST_LAST_ENTRY_TICKS: u16 = FAST_LAST_ENTRY_TICKS;
    const BYTES_PER_INTERVAL: u16 = FAST_LAST_BYTES_PER_INTERVAL;
    const GUARD_BYTES: u16 = FAST_LAST_GUARD_BYTES;

    fn set_busy_wait_deadline(&mut self, deadline: u32) {
        self.deadline = deadline;
    }

    fn schedule(&mut self, deadline: u32) {
        // Past-CMP handling: the parser path (IDLE → Crc event → send_slot
        // → here) can easily exceed the slot-offset budget at high baud
        // with short chains, so `deadline` lands behind CNT on the common
        // case. Setting CMP near `now` races the ~30–80 HCLK cycles
        // between reading CNT and writing CMP — CNT typically overshoots
        // CMP and the next CNT == CMP match is a u32 wrap (~89 s) away,
        // which wedges the chip permanently. Instead force the SysTick IRQ
        // to dispatch via PFIC IPSR; the body fires on the next vector
        // entry independent of CMP, runs the catchup step, and re-arms
        // CMP for the next interval (which may itself be past — self-
        // healing across grid steps). CMP is left untouched on this path;
        // its prior value is either `u32::MAX` (cancel sentinel) or a
        // stale forward value the next body will overwrite.
        let now = systick::ticks();
        let diff = deadline.wrapping_sub(now) as i32;
        // SAFETY: debug-only scratch; same-prio ISR serialization (HIGH).
        unsafe {
            let head = (FL_SCHED_HEAD as usize) & 7;
            let p = &raw mut FL_SCHED_RING[head];
            p.write_volatile(FlSchedSample {
                now,
                deadline,
                diff,
                safe_cmp: if diff < 0 { 0 } else { deadline },
            });
            FL_SCHED_HEAD = FL_SCHED_HEAD.wrapping_add(1);
        }
        systick::clear_match();
        systick::set_irq(true);
        if diff < 0 {
            pfic::pend_systick();
        } else {
            systick::set_cmp(deadline);
        }
    }

    fn deadline_passed(&self) -> bool {
        (systick::ticks().wrapping_sub(self.deadline) as i32) >= 0
    }

    fn patch_window_expired(&self) -> bool {
        dma_prefetched_into_crc(dma::remaining(dma::Channel::CH4), CRC_BYTES as u16)
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

/// True if CH4's remaining count has reached the trailing CRC slot — any
/// `patch_crc` write into `tx_buf[len-crc_bytes..len]` from this point
/// races CH4's read of the same bytes.
fn dma_prefetched_into_crc(ndtr: u16, crc_bytes: u16) -> bool {
    ndtr <= crc_bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dma_before_crc_slot_window_open() {
        // Plenty of payload left → patch is safe.
        assert!(!dma_prefetched_into_crc(10, 2));
    }

    #[test]
    fn dma_one_byte_past_crc_boundary_window_open() {
        // ndtr = crc_bytes + 1: last non-CRC byte still ahead → safe.
        assert!(!dma_prefetched_into_crc(3, 2));
    }

    #[test]
    fn dma_at_crc_boundary_window_expired() {
        // ndtr == crc_bytes: DMA is about to output the first CRC byte —
        // the boundary is treated as inside the race window.
        assert!(dma_prefetched_into_crc(2, 2));
    }

    #[test]
    fn dma_inside_crc_slot_window_expired() {
        assert!(dma_prefetched_into_crc(1, 2));
    }

    #[test]
    fn dma_drained_window_expired() {
        assert!(dma_prefetched_into_crc(0, 2));
    }
}
