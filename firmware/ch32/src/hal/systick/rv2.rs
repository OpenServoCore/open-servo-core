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

/// Zero SYSTICK.CNT in a single store. Used at boot from
/// `init_tim2_ch4_ic_capture` to align SysTick's low 16 bits with TIM2.CNT
/// immediately before TIM2 starts counting, so the IC4 u16 stamps and
/// `WireClock::now()` u32 share their low 16 bits (the contract `EdgeParser`
/// lifts u16 stamps against). Post-boot: no code may call this.
#[inline(always)]
pub fn reset_cnt() {
    SYSTICK.cnt().write_value(0);
}

#[inline(always)]
pub fn set_cmp(value: u32) {
    SYSTICK.cmp().write_value(value);
}

// SAFETY: see hal/SAFETY.md. CTLR is written from MAIN and HIGH ISRs (SysTick
// and USART1 TC); CS keeps STIE RMW atomic.
#[inline(always)]
pub fn set_irq(enable: bool) {
    critical_section::with(|_| {
        SYSTICK.ctlr().modify(|w| w.set_stie(enable));
    });
}

#[inline(always)]
pub fn clear_match() {
    SYSTICK.sr().write(|w| w.set_cntif(false));
}
