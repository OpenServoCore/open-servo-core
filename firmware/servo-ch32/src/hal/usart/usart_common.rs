pub use ch32_metapac::usart::Usart as Regs;

/// osc-native bus bring-up, in the break-framing spike's exact order:
/// HDSEL single wire, TE/RE, BRR, UE, and only then DMAR in a second CTLR3
/// write. The sequencing is load-bearing for the released idle level on the
/// wire: deviations (TE before HDSEL, or DMAR folded into the HDSEL write)
/// leave the HDSEL TX signal latched LOW, and through the AF_OD listening
/// pin that clamps the whole bus (wire stuck low at idle, rose the moment
/// PC0 left AF mode).
/// No IDLE interrupt -- the framer sources all timing from the ring cursor
/// and SysTick, never from IDLE.
///
/// No receive interrupt at all (protocol sec 3.4): the break wake is TIM2 on
/// the bus pin (`providers::break_wake`), and EIE is never set -- FE/NE/ORE
/// latch silently and nothing services them.
#[inline]
pub fn init_bus(r: Regs, brr: u32) {
    r.ctlr3().modify(|w| w.set_hdsel(true));
    r.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
    r.ctlr1().modify(|w| w.set_ue(true));
    r.ctlr3().modify(|w| w.set_dmar(true));
}

/// Send the protocol-law break (protocol sec 3): one 9-bit
/// `0x00` character under a bracketed M=1 -- start + 9 data lows = 10 low
/// bit-times, then a clean stop. Replaces SBK, whose ~14-bit shape is
/// off-law and a byte-sync hazard at LIN-mode receivers (bridge-class
/// hosts: their break detector re-arms start hunting at bit 10 while SBK
/// still holds the line low -- measured per-servo phase-dependent
/// head-of-reply corruption).
///
/// Blocks until the character's stop bit commits (TC), the same
/// shifter-state contract the SBK path kept: when this returns the wire
/// has carried the whole break, DR is free for arm0's first byte, and
/// the M flips have bracketed a COMPLETED character -- no format change
/// ever touches a shifting byte. Runs only inside the TX claim window,
/// where RX is muted (HDSEL, F9) or held at mark (buffer), so the M
/// flips are invisible to our own receiver. The wait bound is the
/// no-panic plateau backstop, not a wait that ever runs in practice.
#[inline(always)]
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
/// 0.5M rescue floor ~= 1050 HCLK cycles); 4096 covers it with slack.
const BREAK_COMMIT_SPINS: u32 = 4096;

/// TX-only bring-up (the TEL stream): TE + fixed BRR + UE, DMAT held on
/// for good - the DMA channel's EN is the per-frame gate, and a pending
/// TXE request with the channel idle transfers nothing. No RE, no
/// interrupts: completion is observed as channel-drained at the next send.
#[inline]
pub fn init_tx_only(r: Regs, brr: u32) {
    r.ctlr1().modify(|w| w.set_te(true));
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
    r.ctlr1().modify(|w| w.set_ue(true));
    r.ctlr3().modify(|w| w.set_dmat(true));
}

/// Live BRR write -- no UE bounce. Caller must ensure no TX/RX is in flight
/// (retuning mid-byte garbages the wire); the driver applies baud changes
/// only on an idle bus (protocol sec 4.2 deferred config). The UE bounce
/// is avoided for cause: dropping UE re-latches the HDSEL TX output to the
/// inactive-AF level (0), and through the AF_OD listening pin that clamps
/// the whole bus until the next own-TX (measured; the break-framing
/// path never bounces UE and its idle line sits released-high).
#[inline]
pub fn set_baud(r: Regs, brr: u32) {
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
}

// SAFETY: see hal/SAFETY.md. CTLR3 is written from MAIN and the USART1 TC
// ISR; CS keeps RMW atomic against future different-bit additions.
// `inline(always)` keeps this inside its caller's body, so a caller placed in
// RAM (`.data.ram_code`) makes no flash fetch for it.
#[inline(always)]
pub fn set_dma_tx(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr3().modify(|w| w.set_dmat(enable));
    });
}

#[inline]
pub fn data_addr(r: Regs) -> u32 {
    r.datar().as_ptr() as u32
}

// SAFETY: see hal/SAFETY.md. CTLR1 is written from MAIN and the USART1 TC
// ISR (here, and the UE toggle in set_baud).
// `inline(always)` keeps this inside its caller's body, so a caller placed in
// RAM (`.data.ram_code`) makes no flash fetch for it.
#[inline(always)]
pub fn set_tc_irq(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr1().modify(|w| w.set_tcie(enable));
    });
}

// SAFETY: see hal/SAFETY.md. STATR.modify is the only write-0-to-clear bit
// path on this register today; CS guards against future write-clear additions.
// `inline(always)` keeps this inside its caller's body, so a caller placed in
// RAM (`.data.ram_code`) makes no flash fetch for it.
#[inline(always)]
pub fn clear_tc(r: Regs) {
    critical_section::with(|_| {
        r.statr().modify(|w| w.set_tc(false));
    });
}

/// The STATR read is side-effect-free for the transport: it arms the SR
/// half of the hardware's SR-then-DR pair, but nothing on the receive side
/// ever performs the DR half (no DATAR reads, transport sec 7), so no flag
/// state changes hang off it.
#[inline]
pub fn is_tc(r: Regs) -> bool {
    r.statr().read().tc()
}

#[inline]
pub fn is_tcie(r: Regs) -> bool {
    r.ctlr1().read().tcie()
}
