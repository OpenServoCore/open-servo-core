//! IRQ vectors that drive the walker. All three share `PRIO_WALKER`
//! (set in `pfic`) so they cannot preempt one another mid-walk —
//! `walker::walk()` runs single-threaded across them.

use ch32_metapac::{DMA1, USART3};
use qingke_rt::interrupt;

use crate::rx::trace::{
    self, TRACE_PHASE_IC_HT, TRACE_PHASE_IC_TC, TRACE_PHASE_IDLE, TRACE_PHASE_RX_HT,
    TRACE_PHASE_RX_TC,
};
use crate::tick::read_tick32;

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
    if statr.lbd() {
        // LIN break detect (LBDIE). Clear with an explicit write — rc_w0
        // semantics, so the all-ones background leaves TC/IDLE untouched
        // (a modify() RMW could write back a stale 0 into a flag that set
        // between the read and the write).
        USART3.statr().write(|w| {
            w.0 = u32::MAX;
            w.set_lbd(false);
        });
    }
    if statr.idle() {
        // STATR read above + DATAR read clears IDLE.
        let _ = USART3.datar().read();
        let idle_tick = read_tick32();
        // Drain in-flight bytes BEFORE handing off to the send scheduler.
        // IDLE means every received byte's CC3 capture + RX DMA write has
        // landed (RX DMA decrements NDTR before the USART transitions into
        // the idle window that triggers IDLE), so the walker sees a fully
        // populated tail. This catches the last-byte-of-reply case that
        // would otherwise wait for the next DMA HT/TC trigger.
        trace::run_walker(TRACE_PHASE_IDLE);
        crate::tx::on_idle(idle_tick);
    }
    crate::led::signal();
}
