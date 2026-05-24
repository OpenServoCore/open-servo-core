use ch32_metapac::{
    FLASH, RCC,
    rcc::vals::{Pllsrc, Sw},
};

use crate::hal::clocks::{adcpre_val, hpre_val};

pub fn init_pll() {
    const HPRE: ch32_metapac::rcc::vals::Hpre = hpre_val();
    const ADCPRE: ch32_metapac::rcc::vals::Adcpre = adcpre_val();

    FLASH.actlr().modify(|w| w.set_latency(2));

    RCC.cfgr0().modify(|w| {
        w.set_pllsrc(Pllsrc::HSI);
        w.set_hpre(HPRE);
        w.set_adcpre(ADCPRE);
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
pub fn enable_tim2() {
    RCC.pb1pcenr().modify(|w| w.set_tim2en(true));
}

#[inline]
pub fn enable_adc1() {
    RCC.pb2pcenr().modify(|w| w.set_adcen(true));
}

#[inline]
pub fn enable_dma1() {
    RCC.hbpcenr().modify(|w| w.set_dma1en(true));
}

#[inline]
pub fn enable_usart1() {
    RCC.pb2pcenr().modify(|w| w.set_usart1en(true));
}
