//! V4 SysTick (64-bit CNT/CMP at HCLK/8 = 18 MHz). The engine's `Deadline`
//! runs on the low 32 bits; `set_cmp_low` composes the full 64-bit compare
//! for "the next time the low word equals `at`", and the provider's
//! post-arm recheck (servo discipline) converts a lost race into a PFIC
//! software pend.

use ch32_metapac::SYSTICK;
use ch32_metapac::systick::vals::{Mode, Stclk};

use crate::hal::rcc::SYSCLK_HZ;

/// HCLK/8.
pub const TICKS_PER_US: u32 = SYSCLK_HZ / 8 / 1_000_000;

pub fn init() {
    SYSTICK.ctlr().write(|w| w.set_ste(false));
    SYSTICK.cmp().write_value(u64::MAX);
    SYSTICK.cnt().write_value(0);
    SYSTICK.sr().write(|w| w.set_cntif(false));
    SYSTICK.ctlr().write(|w| {
        w.set_ste(true);
        w.set_stclk(Stclk::HCLK_DIV8);
        w.set_mode(Mode::UPCOUNT);
        w.set_stre(false);
    });
}

/// Engine tick domain: the free-running low word (wraps ~238 s; the engine
/// is wrap-aware).
#[inline(always)]
pub fn ticks() -> u32 {
    SYSTICK.cntl().read()
}

/// Arm the 64-bit compare at the next occurrence of low word == `at`.
/// The high-word read pair guards the 32-bit carry (one tick every ~238 s);
/// an `at` at-or-behind the current low word lands in the NEXT epoch, and
/// the caller's tick_reached recheck pends immediately for that case.
pub fn set_cmp_low(at: u32) {
    let mut high = SYSTICK.cnth().read();
    let mut low = SYSTICK.cntl().read();
    let high2 = SYSTICK.cnth().read();
    if high2 != high {
        high = high2;
        low = SYSTICK.cntl().read();
    }
    let epoch = if at > low { high } else { high.wrapping_add(1) };
    SYSTICK
        .cmp()
        .write_value(((epoch as u64) << 32) | at as u64);
}

// SAFETY: CTLR is written from MAIN (engine deadline set under CS) and the
// SysTick ISR; CS keeps the STIE RMW atomic.
#[inline(always)]
pub fn set_irq(enable: bool) {
    critical_section::with(|_| {
        SYSTICK.ctlr().modify(|w| w.set_stie(enable));
    });
}

/// CNTIF latched -- a compare match occurred since the last clear.
#[inline(always)]
pub fn matched() -> bool {
    SYSTICK.sr().read().cntif()
}

#[inline(always)]
pub fn clear_match() {
    SYSTICK.sr().write(|w| w.set_cntif(false));
}
