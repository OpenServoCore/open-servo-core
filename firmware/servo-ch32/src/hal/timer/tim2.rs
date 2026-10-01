//! TIM2 as an edge-reset quiet timer on its CH1 pin: slave reset mode on
//! TI1F_ED zeroes the counter at every edge, so the update fires only once
//! the pin has held one level for the whole reload.

use ch32_metapac::TIM2;
use ch32_metapac::timer::regs::{ChctlrInput, Dmaintenr, Intfr};
use ch32_metapac::timer::vals::Urs;

/// SMCFGR TS = 100: TI1F_ED, the both-edges detector on TI1.
const TS_TI1F_ED: u8 = 0b100;
/// SMCFGR SMS = 100: reset mode, the trigger zeroes the counter.
const SMS_RESET: u8 = 0b100;
/// CHCTLR1 CC1S = 01: CH1 is an input on TI1, IC1F = 0 (a filter only
/// delays the reset).
const CC1S_TI1: u32 = 0b01;

/// Count the timer clock (PSC 0) up to `reload`, with the update interrupt
/// and its DMA request armed. URS = 1: the update a reset-mode trigger
/// causes raises neither UIF nor a DMA request, so only an overflow does.
/// CH2..CH4 stay at reset (inputs, CCxE = 0): no TIM2 output reaches a pin.
pub fn init_quiet_timer(reload: u16) {
    TIM2.psc().write_value(0);
    TIM2.atrlr().write_value(reload);
    TIM2.chctlr_input(0).write_value(ChctlrInput(CC1S_TI1));
    TIM2.smcfgr().write(|w| {
        w.set_ts(TS_TI1F_ED);
        w.set_sms(SMS_RESET);
    });
    TIM2.intfr().write_value(Intfr(0));
    arm_update();
    TIM2.ctlr1().write(|w| {
        w.set_urs(Urs::COUNTERONLY);
        w.set_cen(true);
    });
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

#[inline(always)]
pub fn update_armed() -> bool {
    TIM2.dmaintenr().read().uie()
}

/// After an overflow: mask the update and its DMA request and wake on the
/// next edge (TIE) instead.
#[inline(always)]
pub fn park() {
    TIM2.dmaintenr().write_value(Dmaintenr(0));
    clear_update_and_edge();
    TIM2.dmaintenr().write(|w| w.set_tie(true));
}

/// Deaf until the next `rearm`: no update, no edge wake, no DMA request,
/// and no stale flag for a pended entry to act on.
#[inline(always)]
pub fn mute() {
    TIM2.dmaintenr().write_value(Dmaintenr(0));
    clear_update_and_edge();
}

/// On the first edge after a park: the counter is already counting from
/// it, so arm the update again.
#[inline(always)]
pub fn rearm() {
    TIM2.dmaintenr().write_value(Dmaintenr(0));
    clear_update_and_edge();
    arm_update();
}

/// The DMA request source rides the same, last write as the interrupt.
#[inline(always)]
fn arm_update() {
    TIM2.dmaintenr().write(|w| {
        w.set_uie(true);
        w.set_ude(true);
    });
}

/// INTFR flags are rc_w0: a constant write, every other bit 1, clears
/// exactly UIF and TIF.
#[inline(always)]
fn clear_update_and_edge() {
    TIM2.intfr().write(|w| {
        w.0 = 0xFFFF;
        w.set_uif(false);
        w.set_tif(false);
    });
}
