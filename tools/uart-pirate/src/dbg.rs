//! Scope-facing dbg pin (PB0 -- free breakout on the nanoCH32V203).
//! Three width-coded markers, so a scope can width-trigger on the rare
//! event while the common ones stay visible on the same trace:
//!
//! - short (~0.5 us): LBD service entry -- every break on the wire;
//! - wide (~5 us): a standalone record consumed at drain -- the LIN
//!   engine swallowed this break's character;
//! - host (~20 us): `PULSE` command -- the bench probe fires it when it
//!   detects a corrupt capture, the single-shot trigger that holds the
//!   guilty exchange in pre-trigger memory.
//!
//! Forensics only -- nothing on the bench consumes the line.

use ch32_metapac::{GPIOB, RCC};

use crate::tick::read_tick32;

const SHORT_TICKS: u32 = 72;
const WIDE_TICKS: u32 = 720;
const HOST_TICKS: u32 = 2_880;

pub fn init() {
    RCC.apb2pcenr().modify(|w| w.set_iopben(true));
    // PB0 = GP push-pull, 50 MHz: Mode=11, CNF=00 -> 0b0011.
    GPIOB.cfglr().modify(|w| {
        let mut v = w.0;
        v &= !0xF;
        v |= 0b0011;
        w.0 = v;
    });
    GPIOB.bshr().write(|w| w.set_br(0, true));
}

fn pulse(width: u32) {
    GPIOB.bshr().write(|w| w.set_bs(0, true));
    let t0 = read_tick32();
    while read_tick32().wrapping_sub(t0) < width {}
    GPIOB.bshr().write(|w| w.set_br(0, true));
}

/// LBD service entry (wire ISR context -- kept short).
pub fn mark_break() {
    pulse(SHORT_TICKS);
}

/// Standalone boundary consumed at drain (USB command context).
pub fn mark_standalone() {
    pulse(WIDE_TICKS);
}

/// Host-detected event (`PULSE` command).
pub fn mark_host() {
    pulse(HOST_TICKS);
}
