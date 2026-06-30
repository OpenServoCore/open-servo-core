//! RCC init: HSE → PLL ×18 → 144 MHz SYSCLK / HCLK / PCLK1 / PCLK2.
//! USB clock = 48 MHz via USBPRE = ÷3.
//!
//! Distilled from `ch32-hal/src/rcc/v3.rs::init` with `SYSCLK_FREQ_144MHZ_HSE`.
//! The generic init function in ch32-hal supports many V2/V3 family variants
//! and runtime-configurable PLL/HSE/HSI selection; we hard-code the single
//! V203C8T6 + 8 MHz HSE + PLL ×18 + USB ÷3 path the pirate needs.

use ch32_metapac::rcc::vals::{Hpre, PllMul, Ppre, Sw, Usbpre};
use ch32_metapac::{FLASH, RCC};

/// 144 MHz HCLK from 8 MHz HSE oscillator. Sets USB clock to 48 MHz so
/// USBD is usable immediately after this returns. Must be called before
/// any peripheral that consumes a bus clock.
pub fn init_144mhz_hse() {
    // 1. Ensure HSI is the boot clock and ready (V20x boots on HSI).
    while !RCC.ctlr().read().hsirdy() {}

    // 2. Enable HSE (8 MHz crystal, no bypass).
    RCC.ctlr().modify(|w| w.set_hsebyp(false));
    RCC.ctlr().modify(|w| w.set_hseon(true));
    while !RCC.ctlr().read().hserdy() {}

    // 3. Configure PLL: source = HSE / PREDIV=DIV1, multiplier = ×18 → 144 MHz.
    // Must disable PLL before changing settings.
    RCC.ctlr().modify(|w| w.set_pllon(false));
    RCC.cfgr0().modify(|w| {
        w.set_usbpre(Usbpre::DIV3); // 144 / 3 = 48 MHz USB
        w.set_pllmul(PllMul::MUL18);
        w.set_pllsrc(true); // HSE → PLL
        w.set_pllxtpre(false); // HSE / 1 (no prediv)
    });
    RCC.ctlr().modify(|w| w.set_pllon(true));
    while !RCC.ctlr().read().pllrdy() {}

    // 4. FLASH prefetch + enhanced mode. V20x has no separate ACTLR
    // latency register (unlike v1/V10x); wait states are handled by the
    // FPEC at the PFU level given enhancemode=true. `sckmode` selects the
    // half-cycle access mode used at ≤ 72 MHz HCLK; at 144 MHz it must
    // stay false.
    FLASH.ctlr().modify(|w| {
        w.set_sckmode(false);
        w.set_enhancemode(true);
    });

    // 5. Switch SYSCLK to PLL, AHB/APB1/APB2 all ÷1.
    RCC.cfgr0().modify(|w| {
        w.set_sw(Sw::PLL);
        w.set_hpre(Hpre::DIV1);
        w.set_ppre1(Ppre::DIV1);
        w.set_ppre2(Ppre::DIV1);
    });
    while RCC.cfgr0().read().sws() != Sw::PLL {}
}
