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

    fn set_deadline(&mut self, deadline: u32) {
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
