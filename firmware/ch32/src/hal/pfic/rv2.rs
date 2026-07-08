use ch32_metapac::PFIC;
use ch32_metapac::pfic::vals::Keycode;

pub use ch32_metapac::Interrupt;

/// QingKe V2 core IRQ for SysTick (not in metapac `Interrupt`).
const SYSTICK_IRQ: u32 = 12;

/// QingKe V2 core IRQ for the software interrupt (vector 14, handler symbol
/// `Software` in qingke-rt's core table; not in metapac `Interrupt`).
const SOFTWARE_IRQ: u32 = 14;

/// QingKe V2A IPRIORn bit 7 selects preemption class; subpriority bits unused.
#[derive(Copy, Clone)]
pub enum Priority {
    High,
    Low,
}

impl Priority {
    #[inline]
    fn as_u8(self) -> u8 {
        match self {
            Self::High => 0x00,
            Self::Low => 0x80,
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
/// behind CNT — at which point the next CNT == CMP match is a u32 wrap
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
pub fn set_software_priority(prio: Priority) {
    PFIC.iprior(SOFTWARE_IRQ as usize).write_value(prio.as_u8());
}

/// Pend the software interrupt (vector 14) — the live-lane consumer wake:
/// lowest LOW-class vector number, so a live request's dispatch beats the
/// kernel's DMA1_CH1 (22) in same-class arbitration. Write-only bit-set.
#[inline]
pub fn pend_software() {
    PFIC.ipsr1().write(|w| w.0 = 1 << SOFTWARE_IRQ);
}

/// Pend a peripheral vector by number. Used for software-pended wake
/// targets on unused peripherals (I2C1_EV = 30, the queued consumer lane —
/// above the kernel's 22, so a pending kernel tick slots in first).
#[inline]
pub fn pend(irq: Interrupt) {
    let n = irq as u16;
    let bit = 1u32 << (n % 32);
    match n / 32 {
        0 => PFIC.ipsr1().write(|w| w.0 = bit),
        1 => PFIC.ipsr2().write(|w| w.0 = bit),
        2 => PFIC.ipsr3().write(|w| w.0 = bit),
        3 => PFIC.ipsr4().write(|w| w.0 = bit),
        _ => {}
    }
}

#[inline]
pub fn set_priority(irq: Interrupt, prio: Priority) {
    PFIC.iprior(irq as usize).write_value(prio.as_u8());
}

#[inline]
pub fn set_systick_priority(prio: Priority) {
    PFIC.iprior(SYSTICK_IRQ as usize).write_value(prio.as_u8());
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
