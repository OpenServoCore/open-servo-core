//! Wire-side capture: per-byte stamp pipeline.
//!
//! - `rings`  — IC + RX byte DMA rings.
//! - `stamp`  — walker-output per-byte stamp ring (host-drainable).
//! - `walker` — predict-and-snap PLL classifier.
//! - `filter` — per-baud TIM2 IC1F input-capture filter.
//! - `desync` — sticky-fatal desync flag + cause enum.
//! - `trace`  — walker ISR trace ring.
//! - `isr`    — DMA1_CH3/CH6 + USART3 IDLE vectors.
//!
//! Walker triggers: event-driven, no cadence. Three IRQ vectors fan into
//! `walker::walk()` at `PRIO_WALKER`:
//!
//! - `USART3` IDLE — end-of-burst drain. Fires 1 char time after the
//!   wire goes idle (~10 µs at 1M, ~3.3 µs at 3M). Catches the tail
//!   bytes of every reply.
//! - `DMA1_CHANNEL6` HT/TC — mid-burst drain when the IC ring fills.
//!   CH6 (TIM3 high half) is the trailing writer of the CH5/CH6 pair,
//!   so its NDTR bounds the count of fully-written 32-bit entries.
//! - `DMA1_CHANNEL3` HT/TC — mid-burst drain when `rx_ring` fills.
//!
//! All three sources share `PRIO_WALKER` so they cannot preempt one
//! another; `walk()` runs single-threaded across them.
//!
//! Classification is **predict-and-snap PLL**: the first byte's start
//! tick is anchored on the first unconsumed IC edge; every subsequent
//! byte's start is predicted at `prev_anchor + 10·bit` and snapped to
//! the closest IC entry within `±SNAP_BITS·bit` of that prediction
//! (later-edge tiebreak). On miss, the walker free-runs on the
//! prediction and flags `COUNT_UNDER` — anchor still updates, so the
//! prediction chain survives a short edge dropout.
//!
//! Two conditions flip the sticky `DESYNCED` flag — one designed-
//! impossible (`ic_overrun`) and one host-paced bench-script bug
//! (`stamp_overflow`). Host commands error out until a `RESET` (or
//! `BAUD`, which implicitly resets) clears the flag.

mod desync;
mod filter;
mod isr;
mod rings;
mod stamp;
mod trace;
mod walker;

use ch32_metapac::Interrupt;
use portable_atomic::Ordering;

use crate::parse::brr_for;
use crate::tx::{APB1_HZ, BaudError, DEFAULT_BAUD};

pub use desync::{DesyncCause, desync_cause};
pub use rings::FALL_LEN;
pub use stamp::{ByteRecord, drain_batch, drain_byte, stamps_available};
pub use trace::{WalkerTrace, trace_clear, trace_drain};

pub fn init() {
    rings::init();
    walker::BIT_TICKS.store(APB1_HZ / DEFAULT_BAUD, Ordering::Relaxed);
    filter::apply_for_brr(APB1_HZ / DEFAULT_BAUD);
    // Drop any stray IC entries deposited during USART3/PB10 init
    // transients. Without this, the first SEND's anchor falls on a
    // stale boot-glitch edge in the falling ring.
    reset_walker();

    unsafe {
        qingke::pfic::enable_interrupt(Interrupt::USART3 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL6 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL3 as u8);
    }
}

/// Reconfigure the walker's bit-time reference + IC filter. Caller
/// already retuned the USART itself via `tx::set_baud`; this just keeps
/// the walker's bit window aligned with the new baud.
pub fn set_baud(bps: u32) -> Result<(), BaudError> {
    let brr = brr_for(APB1_HZ, bps).ok_or(BaudError::OutOfRange)?;
    walker::BIT_TICKS.store(brr, Ordering::Relaxed);
    filter::apply_for_brr(brr);
    // A baud change with bytes mid-flight produces garbage anyway; drop
    // any pending IC entries so the next byte anchors on a fresh edge.
    reset_walker();
    Ok(())
}

/// Drop all in-flight walker state up to current DMA heads and clear
/// `DESYNCED` + cause. Called from init, `set_baud`, and the host `RESET`
/// command. The IC entries / RX bytes already in the rings are presumed
/// stale at these points (boot glitches, baud-bounce transients, leftover
/// packets, or the bytes that overflowed the stamp ring) — pulling the
/// walker counters forward to the DMA heads ensures the next byte
/// anchors on a fresh edge.
pub fn reset_walker() {
    critical_section::with(|_| {
        let falling_total = rings::refresh_falling_total();
        let rx_total = rings::refresh_rx_total();
        walker::reset_anchors(falling_total);
        stamp::reset_to(rx_total);
        desync::clear();
    });
}

/// Configured baud (= `APB1_HZ / BIT_TICKS`). Returns 0 before the first
/// `set_baud` completes.
pub fn current_baud() -> u32 {
    APB1_HZ
        .checked_div(walker::BIT_TICKS.load(Ordering::Relaxed))
        .unwrap_or(0)
}

/// Atomic snapshot of the IC ring + walker counters for the `BICSNAP`
/// diagnostic host command. Counters and ring contents are read under
/// `critical_section` so they're coherent against walker ISRs. Returns
/// the snapshot header plus the number of u32 ticks written to `out`, in
/// oldest-first order (`out[0]` is the oldest in-ring entry, `out[n-1]`
/// the newest). Each entry is the wrap-race-corrected combined
/// `(hi, lo)` pair, so the host receives sub-µs tick32 values — no
/// host-side lift required.
///
/// `falling_total` is sourced from the walker's last refresh, which
/// tracks CH6 (the trailing DMA writer). Entries past CH5's NDTR but
/// not yet CH6's are intentionally invisible here: they haven't formed
/// a fully-written 32-bit pair yet. `walk()` advances `falling_total`
/// on its next trigger; the host can re-`BICSNAP` after a `STATUS`.
#[derive(Copy, Clone)]
pub struct IcSnapshot {
    pub ref_tick: u32,
    pub falling_total: u32,
    pub walked: u32,
    pub rx_total: u32,
    pub byte_head: u32,
    pub bit_ticks: u32,
    pub cc_filter_delay: u32,
}

pub fn ic_snapshot(out: &mut [u32]) -> (IcSnapshot, usize) {
    critical_section::with(|_| {
        let falling_total = rings::falling_total_cached();
        let rx_total = rings::rx_total_cached();
        let walked = walker::walked();
        let byte_head = stamp::BYTE_HEAD.load(Ordering::Relaxed);
        let ref_tick = walker::LAST_LIFT_CEILING.load(Ordering::Acquire);
        let bit_ticks = walker::BIT_TICKS.load(Ordering::Relaxed);
        let cc_filter_delay = filter::delay_ticks();

        let in_ring = (falling_total as usize).min(FALL_LEN);
        let n = in_ring.min(out.len());
        let start = falling_total.wrapping_sub(n as u32);
        for (i, slot) in out.iter_mut().take(n).enumerate() {
            let probe = start.wrapping_add(i as u32);
            *slot = rings::falling_at(probe, falling_total, ref_tick);
        }
        (
            IcSnapshot {
                ref_tick,
                falling_total,
                walked,
                rx_total,
                byte_head,
                bit_ticks,
                cc_filter_delay,
            },
            n,
        )
    })
}
