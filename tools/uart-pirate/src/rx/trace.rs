//! Walker ISR trace ring. One record per walker invocation, drained by
//! host `BTRACE`. Diagnoses stochastic walker behaviour that doesn't trip
//! the `DESYNCED` flag: trigger-source tag (`phase`), TIM2.CNT entry/exit
//! (ISR latency + walker duration), and pre-walk falling_pending +
//! edges/bytes deltas (per-invocation workload).

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::TIM2;
use portable_atomic::{AtomicU32, Ordering};

use crate::rx::rings;
use crate::rx::stamp::BYTE_HEAD;
use crate::rx::walker::{self, WALKED_FALLING};

pub const TRACE_PHASE_IDLE: u8 = 0;
pub const TRACE_PHASE_RX_HT: u8 = 1;
pub const TRACE_PHASE_RX_TC: u8 = 2;
pub const TRACE_PHASE_IC_HT: u8 = 3;
pub const TRACE_PHASE_IC_TC: u8 = 4;
pub const TRACE_PHASE_HOST: u8 = 5;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct WalkerTrace {
    pub phase: u8,
    pub tim2_cnt_entry: u16,
    pub tim2_cnt_exit: u16,
    /// `falling_total - walked` at ISR entry — how deep the IC ring was
    /// when the walker arrived. Computed from a live NDTR read so it
    /// reflects edges captured between the previous walker exit and this
    /// entry, not just the stale `FALLING_TOTAL` from the last refresh.
    pub falling_pending_entry: u16,
    pub edges_consumed: u8,
    pub bytes_emitted: u8,
    /// Cumulative IC edges captured at ISR entry. Compare to host-side
    /// expected edges-per-byte sum to see whether missing edges were lost
    /// upstream (CC filter / DMA) or downstream (walker classification).
    pub falling_total: u32,
    /// Cumulative RX bytes received at ISR entry.
    pub rx_total: u32,
}

const TRACE_LEN: usize = 64;
const TRACE_MASK: u32 = (TRACE_LEN - 1) as u32;
const _: () = assert!(TRACE_LEN.is_power_of_two());
const TRACE_ZERO: WalkerTrace = WalkerTrace {
    phase: 0,
    tim2_cnt_entry: 0,
    tim2_cnt_exit: 0,
    falling_pending_entry: 0,
    edges_consumed: 0,
    bytes_emitted: 0,
    falling_total: 0,
    rx_total: 0,
};

static TRACE_RING: SyncUnsafeCell<[WalkerTrace; TRACE_LEN]> =
    SyncUnsafeCell::new([TRACE_ZERO; TRACE_LEN]);
static TRACE_HEAD: AtomicU32 = AtomicU32::new(0);
static TRACE_TAIL: AtomicU32 = AtomicU32::new(0);

#[inline]
fn push(rec: WalkerTrace) {
    let head = TRACE_HEAD.load(Ordering::Relaxed);
    let idx = (head & TRACE_MASK) as usize;
    unsafe {
        (*TRACE_RING.get())[idx] = rec;
    }
    TRACE_HEAD.store(head.wrapping_add(1), Ordering::Release);
}

pub fn trace_drain() -> Option<WalkerTrace> {
    let tail = TRACE_TAIL.load(Ordering::Relaxed);
    let head = TRACE_HEAD.load(Ordering::Acquire);
    if tail == head {
        return None;
    }
    // Drop overrun: if more than TRACE_LEN unconsumed, snap tail forward
    // to the oldest still-in-ring record. Records get overwritten head-
    // side when the host falls behind.
    let avail = head.wrapping_sub(tail);
    let effective_tail = if (avail as usize) > TRACE_LEN {
        head.wrapping_sub(TRACE_LEN as u32)
    } else {
        tail
    };
    let idx = (effective_tail & TRACE_MASK) as usize;
    let rec = unsafe { (*TRACE_RING.get())[idx] };
    TRACE_TAIL.store(effective_tail.wrapping_add(1), Ordering::Release);
    Some(rec)
}

pub fn trace_clear() {
    critical_section::with(|_| {
        let head = TRACE_HEAD.load(Ordering::Relaxed);
        TRACE_TAIL.store(head, Ordering::Release);
    });
}

/// Shared bookkeeping for every walker-driving IRQ (IDLE + DMA HT/TC).
/// Snapshots pre-walk counters for the trace ring, runs `walk()`, then
/// pushes a record reflecting actual work performed.
#[inline]
pub(super) fn run_walker(phase: u8) {
    let cnt_entry = TIM2.cnt().read();
    let walked_pre = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };
    let byte_head_pre = BYTE_HEAD.load(Ordering::Relaxed);
    // Live-NDTR pending counts without mutating walker state; `walk()`
    // re-reads NDTR before processing.
    let (falling_total_now, _) = rings::peek_falling_total();
    let (rx_total_now, _) = rings::peek_rx_total();
    let falling_pending_entry = falling_total_now.wrapping_sub(walked_pre);

    walker::walk();

    let walked_post = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };
    let byte_head_post = BYTE_HEAD.load(Ordering::Relaxed);
    let edges = walked_post.wrapping_sub(walked_pre);
    let bytes = byte_head_post.wrapping_sub(byte_head_pre);
    // Skip zero-work traces so the 64-entry ring keeps a window of real
    // bursts rather than back-to-back no-op idle wake-ups (e.g. a spurious
    // HT firing on a quiet bus).
    if edges == 0 && bytes == 0 && falling_pending_entry == 0 {
        return;
    }
    push(WalkerTrace {
        phase,
        tim2_cnt_entry: cnt_entry,
        tim2_cnt_exit: TIM2.cnt().read(),
        falling_pending_entry: falling_pending_entry.min(u16::MAX as u32) as u16,
        edges_consumed: edges.min(u8::MAX as u32) as u8,
        bytes_emitted: bytes.min(u8::MAX as u32) as u8,
        falling_total: falling_total_now,
        rx_total: rx_total_now,
    });
}
