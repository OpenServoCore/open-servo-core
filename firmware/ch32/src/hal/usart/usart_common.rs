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

/// DATAR address for use as the DMA controller's peripheral address.
#[inline]
pub fn data_addr(r: Regs) -> u32 {
    r.datar().as_ptr() as u32
}
