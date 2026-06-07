use core::cell::SyncUnsafeCell;

use crate::hal::{exti, systick};
use crate::statics::SHARED;

use super::statics::DXL_RX_PIN;

use super::isr::{body_chain_catchup, body_chain_fast_fire, body_noop, body_plain_fire};

/// Errors the chain CRC path can flag; each variant maps 1:1 to a persistent
/// counter in `TelemetryDxlLink`.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(super) enum FastChainFault {
    /// `set_phase` rejected a transition not in the legal table — FSM bug.
    IllegalTransition,
    /// At TxArmed (post-straggle-walk), observed predecessor bytes fell short
    /// of predicted. The Q16 reciprocal carries ±1 LSB rounding noise, so
    /// boundary cases may trip without a real predecessor miss.
    PreviousSlotTimeout,
    /// Fire CMP ran more than one byte-time past `fire_tick`.
    SlotTimingMiss,
    /// `patch_crc` finished with ≤ 2 bytes of DMA prefetch slack left.
    CrcPatchDeadlineMiss,
    /// USART STATR.ORE — RX outpaced DMA drain.
    DmaOverrun,
    /// USART STATR.PE — parity error on the wire.
    ParityError,
    /// USART STATR.FE — framing error (stop bit missing).
    FramingError,
    /// USART STATR.NE — noise on the wire mid-bit.
    NoiseError,
}

/// Where in the chain reply timeline we currently sit.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(super) enum FastChainPhase {
    /// Periodic SysTick CMP every `BYTES_PER_INTERVAL × byte_time` ticks.
    /// Each body walks whatever's in the ring; the last tick (aligned to
    /// `last_catchup_tick`) runs a busy-wait fold loop until
    /// `walk_deadline`, then hands off to `TxArmed` at `fire_tick`.
    PeriodicCatchup,
    /// Pre-fire walk done; SysTick CMP armed at `fire_tick`. The TxArmed
    /// body fires, runs the post-fire walk, and patches CRC.
    TxArmed,
}

/// Scheduler state for the next reply this slave will send.
#[derive(Copy, Clone)]
pub(super) enum ReplyState {
    /// Nothing armed.
    Idle,
    /// SysTick CMP armed for a non-snoop reply (Ping, Read, Sync/Bulk slot,
    /// Fast First/Middle/Only).
    Plain,
    /// SysTick CMP armed for a Fast last-slave reply with snoop.
    Chain {
        phase: FastChainPhase,
        /// SysTick value at which TX must start shifting.
        fire_tick: u32,
        /// SysTick value at which the last periodic catchup body fires —
        /// the *start* of a busy-wait loop that absorbs the predecessor
        /// window. Earlier periodic ticks land at `last_catchup_tick − k ×
        /// interval`.
        last_catchup_tick: u32,
        /// SysTick value at which the pre-fire walk-loop times out, leaving
        /// the trailing GUARD bytes for the post-fire walk. Equals
        /// `t_prior_end − t_guard`.
        walk_deadline: u32,
        /// Ring index up to which `bulk_crc` has folded predecessor bytes.
        snoop_head: u16,
        /// Rolling CRC over predecessor bytes observed so far.
        bulk_crc: u16,
        /// Total predecessor byte count — the walk-loop's exit target. In
        /// practice the loop exits via `walk_deadline` with `n_pred − GUARD`
        /// folded; post-fire absorbs the remaining GUARD bytes.
        expected_predecessor_bytes: u16,
        bytes_walked: u32,
    },
}

pub(super) static STATE: SyncUnsafeCell<ReplyState> = SyncUnsafeCell::new(ReplyState::Idle);

/// Per-state dispatch handler called from `on_systick`. Recomputed on every
/// state/phase transition so the SysTick ISR pays one indirect call instead
/// of a data-dependent match chain.
pub(super) type DispatchFn = fn();

pub(super) static DISPATCH: SyncUnsafeCell<DispatchFn> = SyncUnsafeCell::new(body_noop);

#[inline(always)]
fn dispatch_for(s: &ReplyState) -> DispatchFn {
    match s {
        ReplyState::Idle => body_noop,
        ReplyState::Plain => body_plain_fire,
        ReplyState::Chain {
            phase: FastChainPhase::PeriodicCatchup,
            ..
        } => body_chain_catchup,
        ReplyState::Chain {
            phase: FastChainPhase::TxArmed,
            ..
        } => body_chain_fast_fire,
    }
}

#[inline(always)]
pub(super) fn set_state(new: ReplyState) {
    let dispatch = dispatch_for(&new);
    // SAFETY: see on_systick. Both stores are uncontested under PFIC HIGH.
    unsafe {
        *STATE.get() = new;
        *DISPATCH.get() = dispatch;
    }
}

// Inlined into `body_chain_catchup` (its only caller) — body lands in
// `.highcode` via that caller. Runs on the catchup→TxArmed handoff
// path, so the CALL/RET removal trims a few cycles before the
// `set_cmp(fire_tick)` that arms the actual fire.
#[inline(always)]
pub(super) fn set_phase(new: FastChainPhase) {
    // SAFETY: see on_systick. Calls outside Chain are silent no-ops so
    // set_phase chains after cancel() don't double-fault on re-entry.
    let illegal = unsafe {
        match &mut *STATE.get() {
            ReplyState::Chain { phase, .. } => {
                if is_legal_transition(*phase, new) {
                    *phase = new;
                    false
                } else {
                    true
                }
            }
            _ => false,
        }
    };
    if illegal {
        report_fault(FastChainFault::IllegalTransition);
        cancel();
        return;
    }
    // Refresh dispatch from the now-updated state. No-op when state isn't Chain.
    unsafe { *DISPATCH.get() = dispatch_for(&*STATE.get()) };
}

#[inline]
fn is_legal_transition(from: FastChainPhase, to: FastChainPhase) -> bool {
    use FastChainPhase::*;
    matches!((from, to), (PeriodicCatchup, TxArmed))
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn cancel() {
    systick::set_irq(false);
    systick::clear_match();
    // Tail EXTI may still be armed if cancel races a fire that never
    // reached TxArmed. Mask unconditionally — no-op when already off.
    // SAFETY: bring-up writes DXL_RX_PIN before any caller of cancel().
    if let Some(p) = unsafe { *DXL_RX_PIN.get() } {
        exti::set_irq(p, false);
    }
    set_state(ReplyState::Idle);
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub(super) fn report_fault(f: FastChainFault) {
    unsafe {
        let link = &raw mut (*SHARED.table.telemetry.get()).link;
        let counter: *mut u32 = match f {
            FastChainFault::IllegalTransition => &raw mut (*link).illegal_transition,
            FastChainFault::PreviousSlotTimeout => &raw mut (*link).previous_slot_timeout,
            FastChainFault::SlotTimingMiss => &raw mut (*link).slot_timing_miss,
            FastChainFault::CrcPatchDeadlineMiss => &raw mut (*link).crc_patch_deadline_miss,
            FastChainFault::DmaOverrun => &raw mut (*link).dma_overrun,
            FastChainFault::ParityError => &raw mut (*link).parity_error,
            FastChainFault::FramingError => &raw mut (*link).framing_error,
            FastChainFault::NoiseError => &raw mut (*link).noise_error,
        };
        counter.write_volatile(counter.read_volatile().wrapping_add(1));
    }
}

#[inline(always)]
pub fn report_dma_overrun() {
    report_fault(FastChainFault::DmaOverrun);
}

#[inline(always)]
pub fn report_parity_error() {
    report_fault(FastChainFault::ParityError);
}

#[inline(always)]
pub fn report_framing_error() {
    report_fault(FastChainFault::FramingError);
}

#[inline(always)]
pub fn report_noise_error() {
    report_fault(FastChainFault::NoiseError);
}

#[cfg(test)]
mod tests {
    use super::{FastChainPhase, is_legal_transition};

    #[test]
    fn legal_transitions_form_chain() {
        use FastChainPhase::*;
        assert!(is_legal_transition(PeriodicCatchup, TxArmed));
    }

    #[test]
    fn illegal_transitions_rejected() {
        use FastChainPhase::*;
        assert!(!is_legal_transition(TxArmed, PeriodicCatchup));
        assert!(!is_legal_transition(PeriodicCatchup, PeriodicCatchup));
        assert!(!is_legal_transition(TxArmed, TxArmed));
    }
}
