//! IRQ vectors that drive the walker. All three share `PRIO_WALKER`
//! (set in `pfic`) so they cannot preempt one another mid-walk —
//! `walker::walk()` runs single-threaded across them.

use ch32_metapac::{DMA1, USART3};
use portable_atomic::Ordering;
use qingke_rt::interrupt;

use crate::rx::trace::{
    self, TRACE_PHASE_IC_HT, TRACE_PHASE_IC_TC, TRACE_PHASE_IDLE, TRACE_PHASE_RX_HT,
    TRACE_PHASE_RX_TC,
};
use crate::rx::{filter, rings, walker};
use crate::tick::read_tick32;

/// Provable-quiet threshold for the IDLE-clear DATAR read, in bit-times.
/// Must exceed the longest low run that still becomes a byte — a servo
/// SBK break runs 13–14 bits, and its ONLY falling edge is its start, so
/// edge age alone cannot distinguish "mid-break" from "quiet" any
/// earlier. Normal IDLE entries arrive at newest-edge age ≥ 12 bits, so
/// the confirming spin runs ~4 bit-times.
const IDLE_QUIET_BITS: u32 = 16;

/// `Some(now)` once no byte can be mid-reception: the newest IC falling
/// edge (filter-delay compensated) has aged past [`IDLE_QUIET_BITS`].
/// Spins up to that long — the PFIC gives ONE entry per IDLE event
/// (skipping and waiting for a re-fire strands the flag until the next
/// burst), so quiet must be confirmed or refuted before returning.
/// `None` when a fresh edge lands mid-spin: a new burst is starting, and
/// its bytes' RX-DMA DATAR accesses complete the SR→DR pair this entry's
/// STATR read armed — flag self-clears, the next true idle re-fires.
/// NDTR is peeked BEFORE `read_tick32` per `falling_at`'s ceiling
/// contract. Residual blind window: an edge that fell within the CC
/// filter delay (≤ ⅓ bit) is not yet visible — accepted.
fn idle_quiet_tick() -> Option<u32> {
    let bit_ticks = walker::BIT_TICKS.load(Ordering::Relaxed);
    let mut seen: Option<u32> = None;
    loop {
        // Release checkpoint, same contract as the walk loop: a TC that
        // fires mid-spin can't re-enter this very vector, and a spin
        // outlasting the reply gap holds PB10 into the servo's reply
        // (bench: hot-loop reply break+ID merged, 1/10k at 0.5M).
        crate::tx::poll_drive_release();
        let (total, _) = rings::peek_falling_total();
        let now = read_tick32();
        if bit_ticks == 0 || total == 0 {
            return Some(now);
        }
        let newest = rings::falling_at(total.wrapping_sub(1), total, now)
            .wrapping_sub(filter::delay_ticks());
        if seen.is_some_and(|s| s != newest) {
            return None;
        }
        seen = Some(newest);
        if now.wrapping_sub(newest) >= IDLE_QUIET_BITS.wrapping_mul(bit_ticks) {
            return Some(now);
        }
    }
}

/// DMA1_CH6 = FALLING_HI (trailing IC half) HT/TC. CH6 services after
/// CH5 (same TIM2 TRGO pulse), so HT/TC here guarantees both halves of
/// every entry up to the cursor are written. Drains the walker so edges
/// don't accumulate past the half-ring.
#[interrupt]
fn DMA1_CHANNEL6() {
    let isr = DMA1.isr().read();
    let tc = isr.tcif(5);
    DMA1.ifcr().write(|w| {
        w.set_htif(5, true);
        w.set_tcif(5, true);
    });
    let phase = if tc {
        TRACE_PHASE_IC_TC
    } else {
        TRACE_PHASE_IC_HT
    };
    trace::run_walker(phase);
}

/// DMA1_CH3 = RX_RING (USART3 RX) HT/TC. Drains the walker so bytes
/// don't accumulate past the half-ring.
#[interrupt]
fn DMA1_CHANNEL3() {
    let isr = DMA1.isr().read();
    let tc = isr.tcif(2);
    DMA1.ifcr().write(|w| {
        w.set_htif(2, true);
        w.set_tcif(2, true);
    });
    let phase = if tc {
        TRACE_PHASE_RX_TC
    } else {
        TRACE_PHASE_RX_HT
    };
    trace::run_walker(phase);
}

#[interrupt]
fn USART3() {
    let statr = USART3.statr().read();
    // TCIE is armed per-send by tx::arm_drive_release; TC here means the
    // last stop bit cleared the shifter — hand the wire back (PB10 → OD).
    if statr.tc() && USART3.ctlr1().read().tcie() {
        crate::tx::on_tx_complete();
    }
    if statr.idle() {
        // IDLE clears by the STATR read above + a DATAR read — but a CPU
        // DATAR read while a byte is mid-reception KILLS the byte in the
        // shifter (same WCH USART IP as the V006; bench 2026-07-09: a
        // half-ring walk delaying this entry into the servo's reply put
        // the read mid-flight and ate a reply byte ≈1/12k flood cycles).
        // So complete the pair only under IC-proven quiet — confirmed by
        // a bounded spin, since this entry is the ONE shot the IDLE event
        // gives us. When the spin instead sees a fresh edge (a burst is
        // starting), leave the flag latched: the incoming bytes' RX-DMA
        // DATAR accesses complete the pair armed by this entry's STATR
        // read within a byte-time (drain-self-clear), and the true idle
        // after the burst fires a fresh event.
        let quiet = idle_quiet_tick();
        if quiet.is_some() {
            let _ = USART3.datar().read();
        }
        // Walk on BOTH outcomes — the drain is DATAR-free and so always
        // safe, and skipping it while a burst is active starves the stamp
        // rings until a DMA boundary walk, whose then-long drain can land
        // in a send's feed window and hold the wire handback (the walk's
        // release checkpoints only bite when the walk actually runs). On
        // the quiet side this also drains the burst tail that would
        // otherwise wait for the next DMA HT/TC trigger.
        trace::run_walker(TRACE_PHASE_IDLE);
        if let Some(now) = quiet {
            crate::tx::on_idle(now);
        }
    }
    crate::led::signal();
}
