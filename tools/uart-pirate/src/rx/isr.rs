//! IRQ vectors that drive the walker. All three share `PRIO_WALKER`
//! (set in `pfic`) so they cannot preempt one another mid-walk —
//! `walker::walk()` runs single-threaded across them.

use ch32_metapac::{DMA1, USART3};
use portable_atomic::{AtomicU32, Ordering};
use qingke_rt::interrupt;

use crate::rx::rings;
use crate::rx::trace::{
    self, TRACE_PHASE_IC_HT, TRACE_PHASE_IC_TC, TRACE_PHASE_IDLE, TRACE_PHASE_RX_HT,
    TRACE_PHASE_RX_TC,
};
use crate::tick::read_tick32;

/// `rx_total` at the last progress-making idle service — the zero-
/// progress discriminator's memory (see the idle branch).
static LAST_IDLE_RX_TOTAL: AtomicU32 = AtomicU32::new(0);

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
    if statr.idle() && USART3.ctlr1().read().idleie() {
        // The FLAG stays latched — clearing it here is forbidden: the
        // SR→DR pair's CPU DATAR read kills a mid-reception byte in the
        // shifter (same WCH USART IP as the V006), and no quiet proof
        // can close the CC-filter visibility blind window (≈3.6 bits at
        // 2M — a reply's next DMA arm starts inside it; bench
        // 2026-07-10: the arm's first byte died 1/128 profile reads at
        // 2M). Mid-stream the flag drain-self-clears (this entry's
        // STATR read arms the SR half; the next byte's RX-DMA DATAR
        // access completes the pair), and every idle DETECTION follows
        // an RXNE, so servicing every event keeps the per-burst walk
        // cadence the walker's freshness horizons assume.
        let now = read_tick32();
        trace::run_walker(TRACE_PHASE_IDLE);
        // Zero-progress discriminator: an entry with no new RX bytes
        // since the previous idle entry cannot be a fresh detection
        // (those need an RXNE in between) — it is the latched flag
        // level-pending the vector. Mask the event so it cannot storm;
        // the send paths retire the flag under their own drive and
        // re-enable. On edge-pended silicon this branch never runs.
        let rx = rings::rx_total_cached();
        if rx == LAST_IDLE_RX_TOTAL.load(Ordering::Relaxed) {
            USART3.ctlr1().modify(|w| w.set_idleie(false));
        } else {
            LAST_IDLE_RX_TOTAL.store(rx, Ordering::Relaxed);
            // TC gates the after_idle trigger: a genuine trailing idle
            // has our transmitter drained (TC latches at the last stop
            // bit and stays up), while a service running mid-send (a
            // poll-fed stretch latched the flag early) has a byte still
            // shifting — without the gate it would consume a pending
            // injection mid-request.
            if statr.tc() {
                crate::tx::on_idle(now);
            }
        }
    }
    crate::led::signal();
}
