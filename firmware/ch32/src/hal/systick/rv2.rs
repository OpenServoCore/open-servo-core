use ch32_metapac::SYSTICK;
use ch32_metapac::systick::vals::Stclk;

pub use crate::hal::clocks::SYSTICK_TICKS_PER_US as TICKS_PER_US;

pub fn init() {
    SYSTICK.cmp().write_value(u32::MAX);
    SYSTICK.cnt().write_value(0);
    SYSTICK.ctlr().write(|w| {
        w.set_ste(true);
        w.set_stclk(Stclk::HCLK);
    });
}

#[inline(always)]
pub fn ticks() -> u32 {
    SYSTICK.cnt().read()
}

#[inline(always)]
pub fn set_cmp(value: u32) {
    SYSTICK.cmp().write_value(value);
}

#[inline(always)]
pub fn cmp() -> u32 {
    SYSTICK.cmp().read()
}

// SAFETY: see hal/SAFETY.md. CTLR is written from MAIN and HIGH ISRs (SysTick
// and USART1 TC); CS keeps STIE RMW atomic.
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
