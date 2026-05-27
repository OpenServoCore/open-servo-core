//! Onboard user LED on PA15 (MuseLab nanoCH32V203). Active-low: anode to 3V3
//! through a resistor, cathode to PA15. Pulling the pin low lights it.
//!
//! PA15 is JTDI at reset — using it as GPIO requires disabling the JTAG-DP
//! (we keep SW-DP on for probe-rs).

use ch32_hal::pac::{AFIO, GPIOA, RCC};

pub fn init() {
    RCC.apb2pcenr().modify(|w| {
        w.set_afioen(true);
        w.set_iopaen(true);
    });

    // SWJ_CFG = 010 → JTAG-DP disabled, SW-DP enabled. Frees PA13/14/15 + PB3/4
    // (we only need PA15 here; SWDIO/SWCLK on PA13/14 stay live for the probe).
    AFIO.pcfr1().modify(|w| {
        let mut v = w.0;
        v &= !(0b111 << 24);
        v |= 0b010 << 24;
        w.0 = v;
    });

    // PA15 = GP push-pull, 50 MHz. CFGHR controls PA8..PA15; PA15 sits in
    // bits [31:28]. Mode=11 (50 MHz), CNF=00 (GP push-pull) → 0b0011.
    GPIOA.cfghr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 28);
        v |= 0b0011 << 28;
        w.0 = v;
    });

    off();
}

#[inline]
pub fn on() {
    GPIOA.bshr().write(|w| w.set_br(15, true));
}

#[inline]
pub fn off() {
    GPIOA.bshr().write(|w| w.set_bs(15, true));
}

#[inline]
pub fn toggle() {
    if GPIOA.outdr().read().odr(15) {
        GPIOA.bshr().write(|w| w.set_br(15, true));
    } else {
        GPIOA.bshr().write(|w| w.set_bs(15, true));
    }
}
