//! USART3 primitives for the full-duplex bus end: TX and RX pins both on
//! the bus wire. The V305 USART is 16x-oversampling: USARTDIV = BRR/16 >= 1
//! (sub-16 BRR clamps silently - the reason `rcc` runs the PLL; every
//! operational BRR at 144 MHz is crystal-exact and comfortably above the
//! floor).

pub use ch32_metapac::usart::Usart as Regs;
use ch32_metapac::usart::regs::Statr;

/// Host bus bring-up, all before the pin joins the USART: the break
/// detector's interrupt, then TE/RE, BRR, UE, and the DMA gates last.
/// LBDL stays reset-0 (10 bits): the law break is exactly 10 lows, which
/// the 11-bit detector never reports. LINEN stays reset-0: the detector
/// runs without LIN mode on this die, and in LIN mode the first break
/// after boot rings no `0x00`.
#[inline]
pub fn init_host(r: Regs, brr: u32) {
    r.ctlr2().modify(|w| w.set_lbdie(true));
    r.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
    r.ctlr1().modify(|w| w.set_ue(true));
    // Both DMA gates stay on for good: RX requests are muted by RE during
    // the claim window, TX requests by CH2's enable bit.
    r.ctlr3().modify(|w| {
        w.set_dmar(true);
        w.set_dmat(true);
    });
}

/// Send the protocol-law break (protocol sec 3): one 9-bit `0x00`
/// character under a bracketed M=1 -- start + 9 data lows = 10 low
/// bit-times, then a clean stop. Never the hardware SBK (~14 bits,
/// off-law). Blocks until the stop bit commits (TC), so DR is free for the
/// next byte and the M flips bracket a COMPLETED character. Runs only
/// inside the TX claim window, where RE is off -- the M flips are
/// invisible to our own receiver. The wait bound is the no-panic plateau
/// backstop, not a wait that ever runs in practice.
#[inline]
pub fn send_break(r: Regs) {
    r.ctlr1().modify(|w| w.set_m(true));
    clear_tc(r);
    r.datar().write(|w| w.set_dr(0));
    let mut bound = BREAK_COMMIT_SPINS;
    while !r.statr().read().tc() && bound > 0 {
        bound -= 1;
        core::hint::spin_loop();
    }
    r.ctlr1().modify(|w| w.set_m(false));
}

/// Break-commit poll bound: the law break is 11 bit-times (~22 us at the
/// 0.5M rescue floor ~= 3200 HCLK cycles); 16384 covers it with slack.
const BREAK_COMMIT_SPINS: u32 = 16384;

/// Live BRR write, no UE bounce. The engine only applies baud on an idle
/// bus.
#[inline]
pub fn set_baud(r: Regs, brr: u32) {
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
}

/// Receiver gate: RE off from wire claim to release, so the RX pin's echo
/// of our own TX never reaches RDR or the break detector - the RxRing
/// "never contains own TX" contract is enforced here, not filtered
/// downstream.
#[inline]
pub fn set_re(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr1().modify(|w| w.set_re(enable));
    });
}

#[inline]
pub fn data_addr(r: Regs) -> u32 {
    r.datar().as_ptr() as u32
}

// SAFETY: CTLR1 is written from MAIN, the engine vectors and the USART3
// vector above them; CS keeps every RMW atomic.
#[inline(always)]
pub fn set_tc_irq(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr1().modify(|w| w.set_tcie(enable));
    });
}

/// STATR flags clear by writing 0; ones leave the rest alone. A plain
/// write, never a read-modify-write: a flag that sets between the read and
/// the write-back would be cleared with it (RXNE then never reaches DMA).
#[inline(always)]
fn clear_flags(r: Regs, mask: Statr) {
    r.statr().write_value(Statr(!mask.0));
}

#[inline(always)]
pub fn clear_tc(r: Regs) {
    let mut m = Statr(0);
    m.set_tc(true);
    clear_flags(r, m);
}

#[inline(always)]
pub fn clear_lbd(r: Regs) {
    let mut m = Statr(0);
    m.set_lbd(true);
    clear_flags(r, m);
}

#[inline(always)]
pub fn statr(r: Regs) -> ch32_metapac::usart::regs::Statr {
    r.statr().read()
}

#[inline]
pub fn is_tcie(r: Regs) -> bool {
    r.ctlr1().read().tcie()
}
