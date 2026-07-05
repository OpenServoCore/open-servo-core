//! TX-start hardware kickoff (`docs/dxl-hw-timed-transport.md` §5).
//!
//! TIM2_CH4 is a permanent, pin-less output compare (OC4M=Frozen, CC4E=0 —
//! the compare EVENT fires CC4DE without driving the live RX pin,
//! spike-verified). DMA1_CH7 is the permanent kickoff channel: a
//! one-transfer MEM→PER write of an EN=1 config word into `DMA1_CH4.CR`.
//! At the scheduled deadline the CH4 compare match fires DMA1_CH7, which
//! enables the TX DMA and starts the USART1 stream with zero CPU in the
//! deadline path.
//!
//! Free functions + a module static rather than provider-struct state: the
//! arm/cancel sites live on [`DxlTxScheduler`] (driver-called) and the
//! TC-park lives in the DMA1_CH7 ISR (no driver involvement) — one
//! PFIC-HIGH-serialized state.
//!
//! ## #134 silicon discipline
//!
//! A CC4 DMA request LATCHES in the V006 DMA controller and survives a
//! channel disable — but a request only latches into an *enabled* channel.
//! With CH4 permanently OC there are no input captures, so the only CC4
//! requests are real compare matches. Between windows CH7 is parked
//! disabled ([`on_kickoff_complete`] / [`cancel`]) so a stray match (CNT
//! wrapping past a stale CCR4) can't latch; [`arm`] then points the compare
//! strictly ahead of CNT and re-enables CH7 with a clean TC flag, leaving
//! no latched request to fire the kickoff early.
//!
//! [`DxlTxScheduler`]: super::dxl_tx_scheduler::DxlTxScheduler

use core::cell::SyncUnsafeCell;

use ch32_metapac::DMA1;
use ch32_metapac::dma::vals::{Dir, Pl, Size};

use crate::hal::{dma, timer};
use crate::measurements::KICKOFF_RETRY_LEAD_TICKS;

/// The word CH7 writes into `DMA1_CH4.CR` at the compare match: CH4's
/// bring-up TX config (`runtime/init.rs`) plus EN. Populated at every
/// [`arm`] from the same metapac field constructors so the two can't
/// drift. `SyncUnsafeCell` because DMA reads it while the CPU owns it —
/// writes happen only while CH7 is disabled.
static KICKOFF_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

fn ch4_cr_with_en() -> u32 {
    let mut cr = ch32_metapac::dma::regs::Cr(0);
    cr.set_dir(Dir::FROMMEMORY);
    cr.set_circ(false);
    cr.set_pinc(false);
    cr.set_minc(true);
    cr.set_psize(Size::BITS8);
    cr.set_msize(Size::BITS8);
    cr.set_pl(Pl::LOW);
    cr.set_en(true);
    cr.0
}

/// Arm the hardware kickoff for a compare at TIM2 CCR4 = `compare`.
///
/// CH4 is permanently OC (`timer::init_tim2_ch4_oc_kickoff`): point its
/// compare at the deadline and wipe CC4IF so the only CC4 request that can
/// pulse comes from a real match on THIS value. Then (re)configure CH7 as
/// the one-transfer kickoff and enable it. See the module's #134 note —
/// CH7 is parked disabled between windows, so re-enabling here can't
/// inherit a latched request.
pub fn arm(compare: u16) {
    timer::set_tim2_ccr4(compare);
    timer::clear_tim2_cc4_flag();

    dma::disable(dma::Channel::CH7);
    // SAFETY: CH7 is disabled — no DMA read of the word can race this.
    unsafe { *KICKOFF_WORD.get() = ch4_cr_with_en() };
    let cfg = dma::Config {
        dir: Dir::FROMMEMORY,
        circ: false,
        pinc: false,
        minc: false,
        size: Size::BITS32,
        htie: false,
        tcie: true,
        pl: Pl::VERYHIGH,
    };
    dma::configure(
        dma::Channel::CH7,
        &cfg,
        DMA1.ch(3).cr().as_ptr() as u32,
        KICKOFF_WORD.get() as u32,
        1,
    );
    // A stale TC latch from the previous kickoff must not re-run the ISR
    // the moment the vector unmasks.
    dma::clear_tc_flag(dma::Channel::CH7);
    dma::enable(dma::Channel::CH7);
}

/// Past-deadline retry: re-aim CCR4 just ahead of CNT so a REAL compare
/// event fires the already-armed kickoff ASAP. (SWEVGR.CC4G would be the
/// obvious trigger, but it's dead silicon on V006 — spike-verified.)
pub fn trigger_asap() {
    timer::clear_tim2_cc4_flag();
    timer::set_tim2_ccr4(timer::tim2_cnt().wrapping_add(KICKOFF_RETRY_LEAD_TICKS));
}

/// DMA1_CH7 TC ISR body — the kickoff word landed, TX DMA is streaming.
/// Park CH7 disabled until the next [`arm`]: with CH4 permanently OC the
/// only CC4 requests are compare matches, and a match can only LATCH into
/// an ENABLED channel (#134), so disabling here closes the window between
/// the kickoff and the next arm against a stray match arming the kickoff
/// early. TC-flag-gated so a stale PFIC pend can't disable a freshly-armed
/// channel mid-window.
pub fn on_kickoff_complete() {
    if !dma::is_tc_flag(dma::Channel::CH7) {
        return;
    }
    dma::clear_tc_flag(dma::Channel::CH7);
    dma::disable(dma::Channel::CH7);
}

/// Cancel-path park: a schedule was dropped while the kickoff was armed
/// (host retry canceling an armed reply). Disable CH7 and wipe its TC latch
/// so a later CC4 match can't fire the stale enable word. No-op if nothing
/// was armed.
pub fn cancel() {
    dma::disable(dma::Channel::CH7);
    dma::clear_tc_flag(dma::Channel::CH7);
}
