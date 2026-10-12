//! V4 PFIC primitives: vector enables and priorities for the transport
//! IRQs, the priority threshold, the loader-state scrub, the SysTick and
//! software-interrupt pends, and the system reset.

use ch32_metapac::PFIC;

pub use ch32_metapac::Interrupt;

/// QingKe core IRQ numbers (not in metapac `Interrupt`).
pub const SYSTICK_IRQ: usize = 12;
pub const SOFTWARE_IRQ: usize = 14;

/// INTSYSCR (CSR 0x804) PMTCFG = 0b01: IPRIOR bit 7 is the one preemption
/// bit, two nesting levels. qingke-rt leaves HPE and nesting enabled with
/// no preemption bit.
#[cfg(target_arch = "riscv32")]
const INTSYSCR_ONE_PREEMPTION_BIT: usize = 0b01 << 2;

/// Give IPRIOR bit 7 its preemption meaning: a level-0x00 vector preempts
/// a level-0x80 body. Bringup-only, before any IRQ unmasks.
pub fn enable_preemption() {
    #[cfg(target_arch = "riscv32")]
    // SAFETY: CSR set of the PMTCFG field only; nothing is pending yet.
    unsafe {
        core::arch::asm!("csrs 0x804, {}", in(reg) INTSYSCR_ONE_PREEMPTION_BIT)
    };
}

#[inline(always)]
pub fn set_priority(irq: usize, level: u8) {
    PFIC.iprior(irq).write_value(level);
}

/// Hold off every vector at `level` or below in priority (0 = none). The
/// read-back orders the write before the caller's next access.
#[inline(always)]
pub fn set_threshold(level: u8) {
    PFIC.ithresdr().write(|w| w.set_threshold(level));
    let _ = PFIC.ithresdr().read();
}

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
pub fn enable_core(irq: usize) {
    PFIC.ienr1().write(|w| w.0 = 1 << irq);
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

#[inline(always)]
pub fn pend_software() {
    PFIC.ipsr1().write(|w| w.0 = 1 << SOFTWARE_IRQ);
}

/// PFIC system reset -- the IAP disarm path's exit. The WCH loader cold
/// boots, finds b5 disarmed, and stays resident for wlink-iap.
pub fn software_reset() -> ! {
    PFIC.sctlr().modify(|w| w.set_sysreset(true));
    loop {
        core::hint::spin_loop();
    }
}
