//! TIM2 as a low-time counter on its CH1 pin: gated mode on the inverted
//! pin counts only while the pin is low, and every rising edge's IC2 DMA
//! request (DMA1 CH7) zeroes the counter, so the update fires only once the
//! pin has held low for the whole reload.
//!
//! CH2CVR is never read: the read clears CC2IF, which the break wake keeps
//! as its rising-edge history.

use ch32_metapac::TIM2;
use ch32_metapac::timer::regs::{ChctlrInput, Intfr};

/// SMCFGR TS = 101: TI1FP1, the filtered and polarity-selected TI1.
const TS_TI1FP1: u8 = 0b101;
/// SMCFGR SMS = 101: gated mode, the counter runs while TRGI is high.
const SMS_GATED: u8 = 0b101;
/// CHCTLR1 CC1S = 01 (IC1 on TI1, the gate) and CC2S = 10 (IC2 on TI1,
/// the edge that zeroes the counter). IC1F = IC2F = 0: a filter only
/// delays both.
const IC1_IC2_ON_TI1: u32 = 0b01 | (0b10 << 8);
/// IC2's channel in the indexed CC accessors.
const IC2: usize = 1;

/// The counter's address, the destination of the rising-edge zero.
#[inline]
pub fn counter_addr() -> u32 {
    TIM2.cnt().as_ptr() as u32
}

/// Count the timer clock (PSC 0) up to `reload` while the pin is low: CC1P
/// inverts TI1FP1, so TRGI is high while the pin is low. IC2 captures on
/// the rising edge (CC2P = 0) and raises its DMA request; the update
/// interrupt is armed. No channel drives a pin: CH1 and CH2 are inputs,
/// CH3 and CH4 stay at reset with CCxE = 0. A closing CC2G seeds the edge
/// history and zeroes the counter as an edge would.
pub fn init_low_timer(reload: u16) {
    TIM2.psc().write_value(0);
    TIM2.atrlr().write_value(reload);
    TIM2.chctlr_input(0)
        .write_value(ChctlrInput(IC1_IC2_ON_TI1));
    TIM2.ccer().write(|w| {
        w.set_ccp(0, true);
        w.set_cce(IC2, true);
    });
    // TS is writable only while SMS = 0.
    TIM2.smcfgr().write(|w| w.set_ts(TS_TI1FP1));
    TIM2.smcfgr().write(|w| {
        w.set_ts(TS_TI1FP1);
        w.set_sms(SMS_GATED);
    });
    TIM2.intfr().write_value(Intfr(0));
    listen();
    rise();
    TIM2.ctlr1().write(|w| w.set_cen(true));
}

#[inline(always)]
pub fn set_reload(reload: u16) {
    TIM2.atrlr().write_value(reload);
}

/// One INTFR image per ISR entry.
#[inline(always)]
pub fn flags() -> Intfr {
    TIM2.intfr().read()
}

/// `true` when a rising edge (or [`rise`]) came since the last [`park`].
#[inline(always)]
pub fn rose(f: Intfr) -> bool {
    f.ccif(IC2)
}

/// INTFR flags are rc_w0: a constant write, every other bit 1, clears
/// exactly UIF.
#[inline(always)]
pub fn clear_update() {
    TIM2.intfr().write(|w| {
        w.0 = 0xFFFF;
        w.set_uif(false);
    });
}

/// Hold off the next update while the pin stays low: the counter restarts
/// past the reload, so it must wrap through 0xFFFF (65536 ticks) before it
/// can overflow again, unless the rising edge zeroes it first. The edge
/// history is cleared with it.
#[inline(always)]
pub fn park() {
    TIM2.intfr().write(|w| {
        w.0 = 0xFFFF;
        w.set_ccif(IC2, false);
    });
    TIM2.cnt().write_value(TIM2.atrlr().read().wrapping_add(1));
}

/// What a rising edge does, by software (CC2G): CC2IF and the DMA zero.
#[inline(always)]
pub fn rise() {
    TIM2.swevgr().write(|w| w.set_ccg(IC2, true));
}

/// Deaf: no update interrupt. The rising-edge zero keeps running.
#[inline(always)]
pub fn mute() {
    TIM2.dmaintenr().write(|w| w.set_ccde(IC2, true));
}

/// Drop any update taken while deaf, then arm the update interrupt.
#[inline(always)]
pub fn listen() {
    clear_update();
    TIM2.dmaintenr().write(|w| {
        w.set_uie(true);
        w.set_ccde(IC2, true);
    });
}
