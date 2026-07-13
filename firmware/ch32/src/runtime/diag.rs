//! Post-init register snapshot at `trace` level. Covers every peripheral
//! touched by `Ch32ControlIo::new` -- if a new init step writes a register that
//! isn't dumped here, add it. PFIC IENR is write-only and intentionally omitted.

use ch32_metapac::{ADC, AFIO, DMA1, GPIOA, GPIOC, GPIOD, OPA, RCC, TIM1};

pub(super) fn dump_init_regs() {
    crate::log::trace!(
        "rcc: ctlr={=u32:08x} cfgr0={=u32:08x} hbpcenr={=u32:08x} pb2pcenr={=u32:08x} pb1pcenr={=u32:08x}",
        RCC.ctlr().read().0,
        RCC.cfgr0().read().0,
        RCC.hbpcenr().read().0,
        RCC.pb2pcenr().read().0,
        RCC.pb1pcenr().read().0,
    );
    crate::log::trace!("afio: pcfr1={=u32:08x}", AFIO.pcfr1().read().0);
    crate::log::trace!(
        "gpio.cfglr: a={=u32:08x} c={=u32:08x} d={=u32:08x}",
        GPIOA.cfglr().read().0,
        GPIOC.cfglr().read().0,
        GPIOD.cfglr().read().0,
    );
    crate::log::trace!(
        "gpio.outdr: a={=u32:08x} c={=u32:08x} d={=u32:08x}",
        GPIOA.outdr().read().0,
        GPIOC.outdr().read().0,
        GPIOD.outdr().read().0,
    );
    crate::log::trace!("opa: ctlr1={=u32:08x}", OPA.ctlr1().read().0);
    crate::log::trace!(
        "adc: ctlr1={=u32:08x} ctlr2={=u32:08x} ctlr3={=u32:08x}",
        ADC.ctlr1().read().0,
        ADC.ctlr2().read().0,
        ADC.ctlr3().read().0,
    );
    crate::log::trace!(
        "adc: samptr1={=u32:08x} samptr2={=u32:08x} rsqr1={=u32:08x} rsqr2={=u32:08x} rsqr3={=u32:08x}",
        ADC.samptr1().read().0,
        ADC.samptr2().read().0,
        ADC.rsqr1().read().0,
        ADC.rsqr2().read().0,
        ADC.rsqr3().read().0,
    );
    let dma_ch = DMA1.ch(0);
    crate::log::trace!(
        "dma1.ch1: cr={=u32:08x} ndtr={=u32:08x} par={=u32:08x} mar={=u32:08x}",
        dma_ch.cr().read().0,
        dma_ch.ndtr().read().0,
        dma_ch.par().read(),
        dma_ch.mar().read(),
    );
    crate::log::trace!(
        "tim1: ctlr1={=u32:08x} ctlr2={=u32:08x} bdtr={=u32:08x} ccer={=u32:08x} psc={=u16} atrlr={=u16}",
        TIM1.ctlr1().read().0,
        TIM1.ctlr2().read().0,
        TIM1.bdtr().read().0,
        TIM1.ccer().read().0,
        TIM1.psc().read(),
        TIM1.atrlr().read(),
    );
    crate::log::trace!(
        "tim1: chctlr0={=u32:08x} chctlr1={=u32:08x} ccr1={=u16} ccr2={=u16} ccr3={=u16} ccr4={=u16}",
        TIM1.chctlr_output(0).read().0,
        TIM1.chctlr_output(1).read().0,
        TIM1.chcvr(0).read(),
        TIM1.chcvr(1).read(),
        TIM1.chcvr(2).read(),
        TIM1.chcvr(3).read(),
    );
}
