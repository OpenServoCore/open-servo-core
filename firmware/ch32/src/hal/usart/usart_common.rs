pub use ch32_metapac::usart::Usart as Regs;

#[inline(always)]
pub fn init(r: Regs, pclk: u32, baud: u32, half_duplex: bool) {
    r.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });

    if half_duplex {
        r.ctlr3().modify(|w| w.set_hdsel(true));
    }

    let brr = (pclk + baud / 2) / baud;
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
