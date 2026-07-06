pub use ch32_metapac::usart::Usart as Regs;

#[inline(always)]
pub fn init(r: Regs, brr: u32) {
    r.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });

    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));

    r.ctlr1().modify(|w| w.set_ue(true));
}

/// osc-native single-wire bring-up, in the break-framing spike's exact
/// order: HDSEL alone, TE/RE, BRR, UE — and only then DMAR + EIE in a
/// second CTLR3 write. The sequencing is load-bearing for the released
/// idle level: deviations (TE before HDSEL, or DMAR/EIE folded into the
/// HDSEL write) leave the HDSEL TX signal latched LOW — through the AF_OD
/// listening pin that clamps the whole bus (bench-measured: wire stuck
/// low at idle, rose the moment PC0 left AF mode). No IDLE interrupt —
/// the framer sources all timing from the ring cursor and SysTick, never
/// from IDLE.
#[inline]
pub fn init_bus(r: Regs, brr: u32) {
    r.ctlr3().modify(|w| w.set_hdsel(true));
    r.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
    r.ctlr1().modify(|w| w.set_ue(true));
    r.ctlr3().modify(|w| {
        w.set_dmar(true);
        w.set_eie(true);
    });
}

/// Send a UART break (SBK, the spike's `pulse_sbk`) and wait, bounded, for
/// the hardware to commit it — SBK self-clears during the break's stop
/// bit. Without the wait, a DR byte loaded by DMA right after SBK shifts
/// out FIRST and the break follows (bench-observed: DMA CNTR frozen at
/// n-1, wire showed data-then-break). Blocking keeps the shifter-state
/// contract `TxWire::send` relies on: when this returns, the wire has
/// carried the whole break and DR is free for arm0's first byte.
#[inline(always)]
pub fn send_break(r: Regs) {
    r.ctlr1().modify(|w| w.set_sbk(true));
    let mut bound = SBK_COMMIT_SPINS;
    while r.ctlr1().read().sbk() && bound > 0 {
        bound -= 1;
        core::hint::spin_loop();
    }
}

/// SBK self-clear poll bound: a break is ~14 bit-times (~28 µs at the 0.5M
/// rescue floor ≈ 1350 HCLK cycles); 4096 covers it with slack.
const SBK_COMMIT_SPINS: u32 = 4096;

/// Live BRR write — no UE bounce. Caller must ensure no TX/RX is in flight
/// (retuning mid-byte garbages the wire); the driver applies baud changes
/// only on an idle bus (§4.2 deferred config). The DXL-era UE bounce is
/// gone for cause: dropping UE re-latches the HDSEL TX output to the
/// inactive-AF level (0), and through the AF_OD listening pin that clamps
/// the whole bus until the next own-TX (bench-measured; the break-framing
/// spike never bounces UE and its idle line sits released-high).
#[inline]
pub fn set_baud(r: Regs, brr: u32) {
    r.brr().write_value(ch32_metapac::usart::regs::Brr(brr));
}

// SAFETY: see hal/SAFETY.md. CTLR3 is written from MAIN and the USART1 TC
// ISR; CS keeps RMW atomic against future different-bit additions.
// `inline(always)` folds this into the TIM2 CC3 hot path so the wire-driver
// activate sequence stays inside the `.highcode` body — no standalone flash
// fetch per call.
#[inline(always)]
pub fn set_dma_tx(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr3().modify(|w| w.set_dmat(enable));
    });
}

#[inline]
pub fn set_dma_rx(r: Regs, enable: bool) {
    r.ctlr3().modify(|w| w.set_dmar(enable));
}

#[inline]
pub fn data_addr(r: Regs) -> u32 {
    r.datar().as_ptr() as u32
}

#[inline]
pub fn set_idle_irq(r: Regs, enable: bool) {
    r.ctlr1().modify(|w| w.set_idleie(enable));
}

// SAFETY: see hal/SAFETY.md. CTLR1 is written from MAIN and the USART1
// ISR set (here from the RXNE wake window, plus the TCIE toggle and
// set_baud's UE bounce); CS keeps the RMW atomic across those writers.
#[inline]
pub fn set_rxne_irq(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr1().modify(|w| w.set_rxneie(enable));
    });
}

#[inline]
pub fn is_rxneie(r: Regs) -> bool {
    r.ctlr1().read().rxneie()
}

// SAFETY: see hal/SAFETY.md. CTLR1 is written from MAIN and the USART1 TC
// ISR (here, and the UE toggle in set_baud).
// `inline(always)` folds this into the TIM2 CC3 hot path so the wire-driver
// activate sequence stays inside the `.highcode` body — no standalone flash
// fetch per call.
#[inline(always)]
pub fn set_tc_irq(r: Regs, enable: bool) {
    critical_section::with(|_| {
        r.ctlr1().modify(|w| w.set_tcie(enable));
    });
}

#[inline]
pub fn clear_idle(r: Regs) {
    // SR-then-DR is the only way to clear IDLE.
    let _ = r.statr().read();
    let _ = r.datar().read();
}

// SAFETY: see hal/SAFETY.md. STATR.modify is the only write-0-to-clear bit
// path on this register today; CS guards against future write-clear additions.
// `inline(always)` folds this into the TIM2 CC3 hot path so the wire-driver
// activate sequence stays inside the `.highcode` body — no standalone flash
// fetch per call.
#[inline(always)]
pub fn clear_tc(r: Regs) {
    critical_section::with(|_| {
        r.statr().modify(|w| w.set_tc(false));
    });
}

#[inline]
pub fn is_idle(r: Regs) -> bool {
    r.statr().read().idle()
}

#[derive(Copy, Clone, Default)]
pub struct RxErrors {
    pub ore: bool,
    pub pe: bool,
    pub fe: bool,
    pub ne: bool,
}

/// Bench forensics: raw STATR image.
#[inline]
pub fn raw_statr(r: Regs) -> u32 {
    r.statr().read().0
}

#[inline]
pub fn rx_errors(r: Regs) -> RxErrors {
    let s = r.statr().read();
    RxErrors {
        ore: s.ore(),
        pe: s.pe(),
        fe: s.fe(),
        ne: s.ne(),
    }
}

/// SR-then-DR is the only way to clear ORE/PE/FE/NE on V006.
#[inline]
pub fn clear_rx_errors(r: Regs) {
    let _ = r.statr().read();
    let _ = r.datar().read();
}

#[inline]
pub fn is_tc(r: Regs) -> bool {
    r.statr().read().tc()
}

#[inline]
pub fn is_tcie(r: Regs) -> bool {
    r.ctlr1().read().tcie()
}

#[inline]
pub fn is_rxne(r: Regs) -> bool {
    r.statr().read().rxne()
}

#[inline]
pub fn read_data(r: Regs) -> u8 {
    r.datar().read().dr() as u8
}
