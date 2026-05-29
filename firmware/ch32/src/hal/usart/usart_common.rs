pub use ch32_metapac::usart::Usart as Regs;

/// `const` so callers can fold at compile time and skip __udivsi3.
pub const fn brr(pclk: u32, baud_hz: u32) -> u32 {
    (pclk + baud_hz / 2) / baud_hz
}

#[inline(always)]
pub fn init(r: Regs, brr: u32, half_duplex: bool) {
    r.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });

    if half_duplex {
        r.ctlr3().modify(|w| w.set_hdsel(true));
    }

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

#[inline]
pub fn set_dma_tx(r: Regs, enable: bool) {
    r.ctlr3().modify(|w| w.set_dmat(enable));
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

#[inline]
pub fn set_tc_irq(r: Regs, enable: bool) {
    r.ctlr1().modify(|w| w.set_tcie(enable));
}

#[inline]
pub fn clear_idle(r: Regs) {
    // SR-then-DR is the only way to clear IDLE.
    let _ = r.statr().read();
    let _ = r.datar().read();
}

#[inline]
pub fn clear_tc(r: Regs) {
    r.statr().modify(|w| w.set_tc(false));
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
pub fn set_rxne_irq(r: Regs, enable: bool) {
    r.ctlr1().modify(|w| w.set_rxneie(enable));
}

#[inline]
pub fn is_rxne(r: Regs) -> bool {
    r.statr().read().rxne()
}

#[inline]
pub fn read_data(r: Regs) -> u8 {
    r.datar().read().dr() as u8
}
