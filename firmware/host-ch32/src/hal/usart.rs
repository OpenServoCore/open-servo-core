//! USART3 primitives for the HDSEL single-wire bus end. The V305 USART is
//! 16x-oversampling: USARTDIV = BRR/16 >= 1 (sub-16 BRR clamps silently --
//! the reason `rcc` runs the PLL; every operational BRR at 144 MHz is
//! crystal-exact and comfortably above the floor).

pub use ch32_metapac::usart::Usart as Regs;

/// Host bus bring-up: HDSEL first, then TE/RE, BRR, UE, and DMAR last in
/// its own CTLR3 write. The order is load-bearing for the released idle
/// level on the single wire (deviations latch the HDSEL TX output LOW
/// through the AF_OD listening pin and clamp the whole bus; measured).
/// LINEN stays reset-0: RM sec 18.5 forbids LIN mode with half-duplex,
/// and the host framer anchors on the break's ring byte, so no LBD is
/// configured at all.
#[inline]
pub fn init_host(r: Regs, brr: u32) {
    r.ctlr3().modify(|w| w.set_hdsel(true));
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

/// Live BRR write -- no UE bounce (dropping UE re-latches the HDSEL TX
/// output low through the listening pin and clamps the bus; measured on
/// the servo end, same IP family). The engine only applies baud on an
/// idle bus.
#[inline]
pub fn set_baud(r: Regs, brr: u32) {
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
}

/// Receiver gate: RE off from wire claim to release so the HDSEL echo of
/// our own TX never reaches RDR -- the RxRing "never contains own TX"
/// contract is enforced here, not filtered downstream.
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

// SAFETY: see servo hal precedent -- CTLR1 is written from MAIN and the
// USART3 TC ISR; CS keeps the RMW atomic.
#[inline(always)]
pub fn set_tc_irq(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr1().modify(|w| w.set_tcie(enable));
    });
}

// SAFETY: STATR.modify is the only write-0-to-clear path used on this
// register; CS guards the RMW against the TC ISR.
#[inline(always)]
pub fn clear_tc(r: Regs) {
    critical_section::with(|_| {
        r.statr().modify(|w| w.set_tc(false));
    });
}

#[inline(always)]
pub fn statr(r: Regs) -> ch32_metapac::usart::regs::Statr {
    r.statr().read()
}

#[inline]
pub fn is_tcie(r: Regs) -> bool {
    r.ctlr1().read().tcie()
}
