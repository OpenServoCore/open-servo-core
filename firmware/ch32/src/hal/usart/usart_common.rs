pub use ch32_metapac::usart::Usart as Regs;

#[inline(always)]
pub fn init(r: Regs, brr: u32) {
    r.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });

    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));

    r.ctlr1().modify(|w| w.set_ue(true));
}

/// Bounces UE around a BRR change. Caller must ensure no TX/RX is in flight —
/// retuning mid-byte will garbage the wire.
#[inline]
pub fn set_baud(r: Regs, brr: u32) {
    r.ctlr1().modify(|w| w.set_ue(false));
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
    r.ctlr1().modify(|w| w.set_ue(true));
}

// SAFETY: see hal/SAFETY.md. CTLR3 is written from MAIN (arm_tx) + USART1 TC
// ISR; CS keeps RMW atomic against future different-bit additions.
// `inline(always)` folds this into the TIM2 CC3 hot path so the wire-driver
// activate sequence stays inside the `.highcode` body — no standalone flash
// fetch per call.
#[inline(always)]
pub fn set_dma_tx(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr3().modify(|w| w.set_dmat(enable));
    });
}

#[inline]
pub fn set_dma_rx(r: Regs, enable: bool) {
    r.ctlr3().modify(|w| w.set_dmar(enable));
}

#[inline]
pub fn data_addr(r: Regs) -> u32 {
    r.datar().as_ptr() as u32
}

#[inline]
pub fn set_idle_irq(r: Regs, enable: bool) {
    r.ctlr1().modify(|w| w.set_idleie(enable));
}

// SAFETY: see hal/SAFETY.md. CTLR1 is written from MAIN (arm_tx) + USART1 TC
// ISR (here, and set_baud's UE toggle).
// `inline(always)` folds this into the TIM2 CC3 hot path so the wire-driver
// activate sequence stays inside the `.highcode` body — no standalone flash
// fetch per call.
#[inline(always)]
pub fn set_tc_irq(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr1().modify(|w| w.set_tcie(enable));
    });
}

#[inline]
pub fn clear_idle(r: Regs) {
    // SR-then-DR is the only way to clear IDLE.
    let _ = r.statr().read();
    let _ = r.datar().read();
}

// SAFETY: see hal/SAFETY.md. STATR.modify is the only write-0-to-clear bit
// path on this register today; CS guards against future write-clear additions.
// `inline(always)` folds this into the TIM2 CC3 hot path so the wire-driver
// activate sequence stays inside the `.highcode` body — no standalone flash
// fetch per call.
#[inline(always)]
pub fn clear_tc(r: Regs) {
    critical_section::with(|_| {
        r.statr().modify(|w| w.set_tc(false));
    });
}

#[inline]
pub fn is_idle(r: Regs) -> bool {
    r.statr().read().idle()
}

#[derive(Copy, Clone, Default)]
pub struct RxErrors {
    pub ore: bool,
    pub pe: bool,
    pub fe: bool,
    pub ne: bool,
}

#[inline]
pub fn rx_errors(r: Regs) -> RxErrors {
    let s = r.statr().read();
    RxErrors {
        ore: s.ore(),
        pe: s.pe(),
        fe: s.fe(),
        ne: s.ne(),
    }
}

/// SR-then-DR is the only way to clear ORE/PE/FE/NE on V006.
#[inline]
pub fn clear_rx_errors(r: Regs) {
    let _ = r.statr().read();
    let _ = r.datar().read();
}

#[inline]
pub fn is_tc(r: Regs) -> bool {
    r.statr().read().tc()
}

#[inline]
pub fn is_rxne(r: Regs) -> bool {
    r.statr().read().rxne()
}

#[inline]
pub fn read_data(r: Regs) -> u8 {
    r.datar().read().dr() as u8
}
