//! Clock tree: HSE 12 MHz -> PLL x12 -> 144 MHz sysclk, plus the USBHS
//! PHY's own PLL fed from the same crystal.

use ch32_metapac::rcc::vals::{Hpre, PllMul, Ppre, Prediv, Sw};
use ch32_metapac::{FLASH, RCC};

/// Sysclk = HSE 12 MHz x PLL 12; HPRE/PPRE1/PPRE2 all /1, so PCLK1 (USART3)
/// and HCLK run at the full rate.
pub const SYSCLK_HZ: u32 = 144_000_000;
pub const PCLK1_HZ: u32 = SYSCLK_HZ;

/// HSE-ready poll bound; a crystal that has not started by then is dead.
const HSE_READY_SPINS: u32 = 200_000;

/// Peripheral clock gates for everything this crate claims. USBHS is
/// enabled separately by the usb service (it also owns the reset pulse).
pub fn enable_peripherals() {
    RCC.apb2pcenr().modify(|w| {
        w.set_iopaen(true);
        w.set_iopben(true);
        w.set_iopcen(true);
    });
    RCC.apb1pcenr().modify(|w| w.set_usart3en(true));
    RCC.ahbpcenr().modify(|w| w.set_dma1en(true));
}

/// HSE on -> park SYSCLK on HSE -> reconfigure PLL -> x12 -> 144 MHz.
///
/// Two loader-entry hazards (measured): (1) the loader can hand over a
/// chip already RUNNING ON THE PLL, and a PLL driving SYSCLK ignores
/// PLLON=0 -- every PLL config write silently bounces until SYSCLK moves
/// off it; (2) the loader's prescalers (PPRE1=/2) and CFGR2 PREDIV1
/// survive entry and must be forced, never trusted. Returns false and
/// stays on the loader's clock if the crystal never reports ready.
pub fn init_sysclk() -> bool {
    RCC.ctlr().modify(|w| w.set_hsebyp(false));
    RCC.ctlr().modify(|w| w.set_hseon(true));
    let mut budget = HSE_READY_SPINS;
    while !RCC.ctlr().read().hserdy() {
        budget -= 1;
        if budget == 0 {
            return false;
        }
    }

    // Off the loader's PLL before touching it (see fn doc), prescalers /1.
    RCC.cfgr0().modify(|w| {
        w.set_sw(Sw::HSE);
        w.set_hpre(Hpre::DIV1);
        w.set_ppre1(Ppre::DIV1);
        w.set_ppre2(Ppre::DIV1);
    });
    while RCC.cfgr0().read().sws() != Sw::HSE {}

    RCC.ctlr().modify(|w| w.set_pllon(false));
    RCC.cfgr2().modify(|w| {
        w.set_prediv1src(false); // HSE feeds PREDIV1
        w.set_prediv1(Prediv::DIV1);
    });
    RCC.cfgr0().modify(|w| {
        w.set_pllmul(PllMul::MUL12);
        w.set_pllsrc(true); // PREDIV1 -> PLL
        w.set_pllxtpre(false);
    });
    RCC.ctlr().modify(|w| w.set_pllon(true));
    while !RCC.ctlr().read().pllrdy() {}

    // 144 MHz flash access: enhanced mode on, half-cycle (sckmode) must
    // stay off above 72 MHz.
    FLASH.ctlr().modify(|w| {
        w.set_sckmode(false);
        w.set_enhancemode(true);
    });

    RCC.cfgr0().modify(|w| {
        w.set_sw(Sw::PLL);
        w.set_hpre(Hpre::DIV1);
        w.set_ppre1(Ppre::DIV1);
        w.set_ppre2(Ppre::DIV1);
    });
    while RCC.cfgr0().read().sws() != Sw::PLL {}
    true
}

/// USBHS PHY PLL: HSE 12 MHz / 3 = the 4 MHz reference, PHY's 480 M PLL
/// alive. Sequenced dead -> config -> alive (RM order). Call only with the
/// crystal confirmed ready.
pub fn init_usbhs_pll() {
    const PREDIV3: u8 = 0b010;
    const REF_4M: u8 = 0b01;
    RCC.cfgr2().modify(|w| w.set_usbhs_pllalive(false));
    RCC.cfgr2().modify(|w| {
        w.set_usbhs_prediy(PREDIV3);
        w.set_usbhs_ckpef_sel(REF_4M);
        w.set_usbhs_pll_src(false); // HSE
        w.set_usbhs_clk_src(true); // PHY PLL drives the controller
    });
    RCC.cfgr2().modify(|w| w.set_usbhs_pllalive(true));
}
