use ch32_metapac::{
    FLASH, RCC,
    rcc::vals::{Adcpre, Hpre, Pllsrc, Sw},
};

pub fn init_48mhz_hsi_pll() {
    FLASH.actlr().modify(|w| w.set_latency(2));

    RCC.cfgr0().modify(|w| {
        w.set_pllsrc(Pllsrc::HSI);
        w.set_hpre(Hpre::DIV1);
        w.set_adcpre(Adcpre::DIV4);
    });

    RCC.ctlr().modify(|w| w.set_pllon(true));
    while !RCC.ctlr().read().pllrdy() {}

    RCC.cfgr0().modify(|w| w.set_sw(Sw::PLL));
    while RCC.cfgr0().read().sws() != Sw::PLL {}
}

#[inline]
pub fn enable_gpio(port_index: usize) {
    RCC.pb2pcenr().modify(|w| w.0 |= 1 << (2 + port_index));
}

#[inline]
pub fn enable_afio() {
    RCC.pb2pcenr().modify(|w| w.set_afioen(true));
}

#[inline]
pub fn enable_tim1() {
    RCC.pb2pcenr().modify(|w| w.set_tim1en(true));
}

#[inline]
pub fn enable_adc1() {
    RCC.pb2pcenr().modify(|w| w.set_adcen(true));
}

#[inline]
pub fn enable_dma1() {
    RCC.hbpcenr().modify(|w| w.set_dma1en(true));
}
