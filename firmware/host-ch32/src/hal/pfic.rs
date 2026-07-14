//! V4 PFIC primitives: vector enables for the two transport IRQs, the
//! loader-state scrub, the SysTick software pend the deadline provider
//! uses as its late-arm escape, and the system reset.

use ch32_metapac::PFIC;

pub use ch32_metapac::Interrupt;

/// QingKe core IRQ number for SysTick (not in metapac `Interrupt`).
const SYSTICK_IRQ: u32 = 12;

#[inline]
pub fn enable(irq: Interrupt) {
    let n = irq as u16;
    let bit = 1u32 << (n % 32);
    match n / 32 {
        0 => PFIC.ienr1().write(|w| w.0 = bit),
        1 => PFIC.ienr2().write(|w| w.0 = bit),
        2 => PFIC.ienr3().write(|w| w.0 = bit),
        3 => PFIC.ienr4().write(|w| w.0 = bit),
        _ => {}
    }
}

#[inline]
pub fn enable_systick() {
    PFIC.ienr1().write(|w| w.0 = 1 << SYSTICK_IRQ);
}

/// Mask and unpend every interrupt source. The WCH loader's jump to the
/// APP is not a reset: whatever it left enabled or pending in the PFIC
/// arrives here live, and qingke-rt's startup enables MIE -- scrub before
/// anything else can matter (first line of bringup).
pub fn scrub_loader_state() {
    PFIC.irer1().write(|w| w.0 = !0);
    PFIC.irer2().write(|w| w.0 = !0);
    PFIC.irer3().write(|w| w.0 = !0);
    PFIC.irer4().write(|w| w.0 = !0);
    PFIC.iprr1().write(|w| w.0 = !0);
    PFIC.iprr2().write(|w| w.0 = !0);
    PFIC.iprr3().write(|w| w.0 = !0);
    PFIC.iprr4().write(|w| w.0 = !0);
}

/// Mask + unpend one source by PFIC number ([`crate::runtime::trap`]'s
/// DefaultHandler retiring a stray loader leftover).
pub fn mask_and_unpend(irq: u32) {
    let bit = 1u32 << (irq % 32);
    match irq / 32 {
        0 => {
            PFIC.irer1().write(|w| w.0 = bit);
            PFIC.iprr1().write(|w| w.0 = bit);
        }
        1 => {
            PFIC.irer2().write(|w| w.0 = bit);
            PFIC.iprr2().write(|w| w.0 = bit);
        }
        2 => {
            PFIC.irer3().write(|w| w.0 = bit);
            PFIC.iprr3().write(|w| w.0 = bit);
        }
        _ => {
            PFIC.irer4().write(|w| w.0 = bit);
            PFIC.iprr4().write(|w| w.0 = bit);
        }
    }
}

/// Force the SysTick vector to dispatch independent of CNT/CMP: the
/// deadline provider's escape for an `at` already behind the counter
/// (writing CMP near `now` races the CNT read by a few HCLK cycles).
/// Write-only register, bit-set semantics.
#[inline]
pub fn pend_systick() {
    PFIC.ipsr1().write(|w| w.0 = 1 << SYSTICK_IRQ);
}

/// PFIC system reset -- the IAP disarm path's exit. The WCH loader cold
/// boots, finds b5 disarmed, and stays resident for wlink-iap.
pub fn software_reset() -> ! {
    PFIC.sctlr().modify(|w| w.set_sysreset(true));
    loop {
        core::hint::spin_loop();
    }
}
