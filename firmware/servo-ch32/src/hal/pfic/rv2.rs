use core::sync::atomic::{Ordering, compiler_fence};

use ch32_metapac::PFIC;
use ch32_metapac::pfic::vals::Keycode;

pub use ch32_metapac::Interrupt;

/// QingKe V2 core IRQ for SysTick (not in metapac `Interrupt`).
const SYSTICK_IRQ: u32 = 12;
/// QingKe V2 core software interrupt (not in metapac `Interrupt`).
const SOFTWARE_IRQ: u32 = 14;

/// IPRIORn bits [7:6] with nesting on (RM sec 6.5.2.21): bit 7 is the
/// preemption level, bit 6 orders pending IRQs within a level; equal values
/// fall back to the lower vector number (RM Table 6-1).
#[derive(Copy, Clone)]
pub enum Priority {
    High,
    Low,
    /// LOW, taken after any pending [`Priority::Low`].
    LowLast,
}

impl Priority {
    #[inline]
    const fn as_u8(self) -> u8 {
        match self {
            Self::High => 0x00,
            Self::Low => 0x80,
            Self::LowLast => 0xC0,
        }
    }
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

/// qingke-rt v2 init sets STIE + mstatus.MIE but never writes PFIC IENR1
/// bit 12; without this, CNTIF latches but the SysTick vector never fires.
#[inline]
pub fn enable_systick() {
    PFIC.ienr1().write(|w| w.0 = 1 << SYSTICK_IRQ);
}

/// Force the SysTick IRQ to dispatch on the next vector entry, independent
/// of CNT == CMP. Used by `FastLastScheduler::schedule` when the requested
/// deadline is already in the past: writing CMP near `now` races the few
/// HCLK cycles between reading CNT and the CMP store, often leaving CMP
/// behind CNT -- at which point the next CNT == CMP match is a u32 wrap
/// (~89 s) away, wedging the chip. Pending the IRQ directly via IPSR1
/// sidesteps the match entirely. Write-only register; bit-set semantics
/// (other bits unaffected).
#[inline]
pub fn pend_systick() {
    PFIC.ipsr1().write(|w| w.0 = 1 << SYSTICK_IRQ);
}

#[inline]
pub fn enable_software() {
    PFIC.ienr1().write(|w| w.0 = 1 << SOFTWARE_IRQ);
}

#[inline]
pub fn pend_software() {
    PFIC.ipsr1().write(|w| w.0 = 1 << SOFTWARE_IRQ);
}

/// Runs `f` with every LOW vector (the bus, `runtime::isr`) held off and
/// HIGH (the kernel) live: ITHRESDR masks each priority value at or above
/// the threshold (QingKe V2 manual sec 3.2), and a vector pended meanwhile
/// enters at the restore. Main loop only: it restores no threshold but 0.
#[inline(always)]
pub fn mask_bus<R>(f: impl FnOnce() -> R) -> R {
    PFIC.ithresdr()
        .write(|w| w.set_threshold(Priority::Low.as_u8()));
    compiler_fence(Ordering::SeqCst);
    let r = f();
    compiler_fence(Ordering::SeqCst);
    PFIC.ithresdr().write(|w| w.set_threshold(0));
    r
}

#[inline]
pub fn set_priority(irq: Interrupt, prio: Priority) {
    PFIC.iprior(irq as usize).write_value(prio.as_u8());
}

#[inline]
pub fn set_systick_priority(prio: Priority) {
    PFIC.iprior(SYSTICK_IRQ as usize).write_value(prio.as_u8());
}

#[inline]
pub fn set_software_priority(prio: Priority) {
    PFIC.iprior(SOFTWARE_IRQ as usize).write_value(prio.as_u8());
}

pub fn software_reset() -> ! {
    ch32_metapac::RCC.rstsckr().write(|w| w.0 = 1 << 24);
    PFIC.cfgr().write(|w| {
        w.set_keycode(Keycode(0xBEEF));
        w.set_resetsys(true);
    });
    loop {
        core::hint::spin_loop();
    }
}
