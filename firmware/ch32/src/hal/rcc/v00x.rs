use ch32_metapac::{
    FLASH, RCC,
    rcc::vals::{Pllsrc, Sw},
};

use crate::hal::clocks::{HSI_HZ, HSI_TRIM_STEP_HZ, adcpre_val, hpre_val};

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

#[inline]
pub fn enable_spi1() {
    RCC.pb2pcenr().modify(|w| w.set_spi1en(true));
}

/// HSITRIM[4:0] reset value — the V006 factory mid-trim default.
const HSITRIM_DEFAULT: i16 = 16;
/// HSITRIM[4:0] valid range upper bound.
const HSITRIM_MAX: i16 = 31;

/// Signed delta bounds for `apply_clock_trim_delta`. Anything outside this
/// gets clamped by the chip register, so callers can use these as the math
/// clamp before storing the result back into the control-table mirror.
pub const CLOCK_TRIM_DELTA_MIN: i8 = -16;
pub const CLOCK_TRIM_DELTA_MAX: i8 = 15;

/// u64 inside the const eval: STEP_HZ × 1_000_000 overflows u32.
pub const CLOCK_TRIM_PPM_PER_STEP: u32 =
    (HSI_TRIM_STEP_HZ as u64 * 1_000_000 / HSI_HZ as u64) as u32;

/// Apply a signed clock-trim delta around the chip's HSITRIM default.
/// ~0.25% HSI rate per step; out-of-range deltas are clamped.
#[inline]
pub fn apply_clock_trim_delta(delta: i8) {
    let v = (HSITRIM_DEFAULT + delta as i16).clamp(0, HSITRIM_MAX) as u8;
    RCC.ctlr().modify(|w| w.set_hsitrim(v));
}
