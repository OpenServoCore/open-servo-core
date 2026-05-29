use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;

use ch32_metapac::USART1;
use dxl_protocol::crc16_continue;

use crate::hal::{dma, gpio, systick, usart};
use crate::statics::{
    DXL_BYTE_TIME_TICKS, DXL_BYTES_PER_US_Q16, DXL_RX_BUF, DXL_RX_BUF_LEN, DXL_TX_BUF, DXL_TX_EN,
    SHARED,
};

/// Which request end tick detection + fire path a given reply takes.
#[allow(dead_code)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum DxlTimingStrategy {
    /// Per-byte RXNE publishes the request end tick; used when IDLE lags RDT
    /// (low baud).
    LengthCounted,
    /// Single IDLE IRQ, backdated by one char-time; used at high baud.
    IdleBackdated,
    /// Fast Sync/Bulk last-slave — snoop predecessor bytes for chain CRC.
    ChainedFastStatus,
}

/// Errors the chain CRC path can flag; each variant maps 1:1 to a persistent
/// counter in `TelemetryDxlLink`.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum FastChainFault {
    /// `set_phase` rejected a transition not in the legal table — FSM bug.
    IllegalTransition,
    /// Snoop walk saw a delta inconsistent with the TC wrap count.
    #[allow(dead_code)]
    UnexpectedByteCount,
    /// At WaitingSwitch CMP, observed predecessor bytes fell short of
    /// predicted. The Q16 reciprocal carries ±1 LSB rounding noise, so
    /// boundary cases may trip without a real predecessor miss.
    PreviousSlotTimeout,
    /// Fire CMP ran more than one byte-time past `fire_tick`.
    SlotTimingMiss,
    /// `patch_crc` finished with ≤ 2 bytes of DMA prefetch slack left.
    CrcPatchDeadlineMiss,
    /// USART STATR.ORE — RX outpaced DMA drain.
    DmaOverrun,
    /// USART STATR.{PE,FE,NE} — parity / framing / noise on the wire.
    UartError,
}

/// Where in the chain reply timeline we currently sit.
#[allow(dead_code)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum FastChainPhase {
    /// Armed, WaitingSwitch CMP pending, no DMA TC wrap seen yet.
    CatchupArmed,
    /// At least one TC wrap has folded a full ring into `bulk_crc`.
    Snoop,
    /// WaitingSwitch CMP body running its catch-up walk to current NDTR.
    Catchup,
    /// Catch-up done; fire CMP pending.
    TxArmed,
    /// Fire CMP fired; TX shifting and straggle walk in progress.
    TxStreaming,
    /// Trailing CRC bytes patched into the TX buffer.
    CrcPatched,
    /// USART TC drained the reply; bus released.
    Done,
    /// Reply aborted; the matching counter has been incremented.
    Fault(FastChainFault),
}

/// Scheduler state for the next reply this slave will send.
#[derive(Copy, Clone)]
pub enum ReplyState {
    /// Nothing armed.
    Idle,
    /// SysTick CMP armed for a non-snoop reply (Ping, Read, Sync/Bulk slot,
    /// Fast First/Middle/Only).
    Plain,
    /// SysTick (+ optionally DMA CH5 TC) armed for a Fast last-slave reply.
    Chain {
        phase: FastChainPhase,
        /// SysTick value at which TX must start shifting.
        fire_tick: u32,
        /// Ring index up to which `bulk_crc` has folded predecessor bytes.
        snoop_head: u16,
        /// Rolling CRC over predecessor bytes observed so far.
        bulk_crc: u16,
        /// Predicted predecessor byte count; floor for `PreviousSlotTimeout`.
        expected_predecessor_bytes: u16,
        bytes_walked: u32,
    },
}

static STATE: SyncUnsafeCell<ReplyState> = SyncUnsafeCell::new(ReplyState::Idle);

const _: () = assert!(DXL_RX_BUF_LEN.is_power_of_two() && DXL_RX_BUF_LEN <= u16::MAX as usize + 1);
const RX_MASK_U16: u16 = (DXL_RX_BUF_LEN as u16).wrapping_sub(1);
const RX_MASK_U32: u32 = (DXL_RX_BUF_LEN - 1) as u32;

/// Gap between WaitingSwitch CMP and the fire CMP. The switch walk drains
/// whatever bytes have accumulated since the last DMA TC; the fire ISR walks
/// the M µs straggle and patches CRC inside the TX DMA prefetch window.
///
/// Lower bound: switch walk must fit M, and at 3 Mbaud + N=512 a full ring
/// walks in ~51 µs. Upper bound: M µs of straggle (~45 bytes at 3 Mbaud,
/// ~4.5 µs of CRC) plus the ~1 µs patch must fit the smallest reply's TX
/// prefetch slack of ~6.66 µs at 3 Mbaud, giving M ≤ ~188 µs. 150 sits in
/// the middle with margin on both sides.
const SWITCH_MARGIN_US: u32 = 150;

/// TX_EN and DMA CH4 stay off so the bus remains in RX through any preceding
/// snoop window; `fire_now` flips both at the slot deadline.
pub fn arm_tx() -> bool {
    // SAFETY: caller has &mut Ch32Bus (sole writer to DXL_TX_BUF).
    let len = unsafe { (*DXL_TX_BUF.get()).as_slice().len() };
    if len == 0 {
        return false;
    }
    // `dma::set_count` requires CH4 disabled; arming over an in-flight TX
    // silently keeps the old NDTR and corrupts the wire mid-byte.
    if dma::is_enabled(dma::Channel::CH4) {
        return false;
    }
    dma::set_count(dma::Channel::CH4, len as u16);
    usart::clear_tc(USART1);
    usart::set_dma_tx(USART1, true);
    usart::set_tc_irq(USART1, true);
    true
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn fire_now() {
    // SAFETY: written once during bring-up before USART1 IRQ unmask.
    if let Some(t) = unsafe { *DXL_TX_EN.get() } {
        gpio::set_level(t.pin, t.tx_level);
    }
    dma::enable(dma::Channel::CH4);
}

pub fn start_plain_after(request_end_tick: u32, delay_us: u32) {
    systick::set_irq(false);
    systick::clear_match();

    let needed = delay_us.saturating_mul(systick::TICKS_PER_US);

    if systick::ticks().wrapping_sub(request_end_tick) >= needed {
        if arm_tx() {
            fire_now();
        }
        return;
    }
    if !arm_tx() {
        cancel();
        return;
    }

    let fire_tick = request_end_tick.wrapping_add(needed);
    set_state(ReplyState::Plain);
    systick::set_cmp(fire_tick);
    systick::set_irq(true);

    // CNTIF latches only on CNT==CMP up-count; if CNT crossed the deadline
    // before CMP was written, the next match is ~89 s away on wrap.
    if systick::ticks().wrapping_sub(request_end_tick) >= needed {
        on_systick();
    }
}

pub fn start_fast_after(request_end_tick: u32, fire_us: u32, snoop_from: Option<u32>) {
    systick::set_irq(false);
    systick::clear_match();

    if !arm_tx() {
        cancel();
        return;
    }

    let fire_needed = fire_us.saturating_mul(systick::TICKS_PER_US);
    let fire_tick = request_end_tick.wrapping_add(fire_needed);

    let snoop_head = match snoop_from {
        Some(rx_start) => (rx_start & RX_MASK_U32) as u16,
        None => 0,
    };

    let expected_predecessor_bytes = match snoop_from {
        Some(_) => predict_predecessor_bytes(fire_us),
        None => 0,
    };

    let (initial_phase, cmp_tick) = match snoop_from {
        None => (FastChainPhase::TxArmed, fire_tick),
        Some(_) if fire_us <= SWITCH_MARGIN_US => (FastChainPhase::TxArmed, fire_tick),
        Some(_) => {
            let switch_us = fire_us - SWITCH_MARGIN_US;
            let switch_ticks = switch_us.saturating_mul(systick::TICKS_PER_US);
            (
                FastChainPhase::CatchupArmed,
                request_end_tick.wrapping_add(switch_ticks),
            )
        }
    };

    set_state(ReplyState::Chain {
        phase: initial_phase,
        fire_tick,
        snoop_head,
        bulk_crc: 0,
        expected_predecessor_bytes,
        bytes_walked: 0,
    });

    if matches!(initial_phase, FastChainPhase::CatchupArmed) {
        dma::clear_tc_flag(dma::Channel::CH5);
        dma::set_tcie(dma::Channel::CH5, true);
    }

    systick::set_cmp(cmp_tick);
    systick::set_irq(true);
    if systick::ticks().wrapping_sub(request_end_tick) >= cmp_tick.wrapping_sub(request_end_tick) {
        on_systick();
    }
}

fn predict_predecessor_bytes(fire_us: u32) -> u16 {
    // SAFETY: u8 access — no tearing. Boot-seeded or main-loop-written; ISR
    // here only reads.
    let rdt_us = unsafe {
        (&raw const (*SHARED.table.config.get()).comms.return_delay_2us).read_volatile() as u32 * 2
    };
    let bytes_per_us_q16 = DXL_BYTES_PER_US_Q16.load(Ordering::Relaxed);
    if bytes_per_us_q16 == 0 {
        return 0;
    }
    // Predecessor reply spans (request_end + rdt) .. (request_end + fire_us).
    // PreviousSlotTimeout fires at TxArmed (post-straggle-walk), so we compare
    // bytes_walked against the full predecessor reply length here.
    let snoop_us = fire_us.saturating_sub(rdt_us);
    let bytes_q16 = snoop_us.saturating_mul(bytes_per_us_q16);
    ((bytes_q16 >> 16) as u32).min(u16::MAX as u32) as u16
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn on_systick() {
    systick::clear_match();
    systick::set_irq(false);

    // SAFETY: USART1 / SysTick / DMA1_CH5 share PFIC HIGH and never preempt
    // each other; STATE access is uncontested across these handlers.
    let snapshot = unsafe { *STATE.get() };
    match snapshot {
        ReplyState::Idle => {}
        ReplyState::Plain => {
            set_state(ReplyState::Idle);
            fire_now();
        }
        ReplyState::Chain {
            phase,
            fire_tick,
            expected_predecessor_bytes,
            ..
        } => match phase {
            FastChainPhase::CatchupArmed | FastChainPhase::Snoop => {
                set_phase(FastChainPhase::Catchup);
                accumulate_snoop();
                set_phase(FastChainPhase::TxArmed);
                systick::set_cmp(fire_tick);
                systick::set_irq(true);
                let now = systick::ticks();
                if (now.wrapping_sub(fire_tick) as i32) >= 0 {
                    on_systick();
                }
            }
            FastChainPhase::Catchup => {
                // Defensive: only reached if a prior CMP body returned without
                // rescheduling. Re-arm against the fire deadline.
                set_phase(FastChainPhase::TxArmed);
                systick::set_cmp(fire_tick);
                systick::set_irq(true);
            }
            FastChainPhase::TxArmed => {
                let now = systick::ticks();
                let byte_time_ticks = byte_time_ticks_now();
                if (now.wrapping_sub(fire_tick) as i32) > byte_time_ticks as i32 {
                    report_fault(FastChainFault::SlotTimingMiss);
                }
                dma::set_tcie(dma::Channel::CH5, false);
                set_phase(FastChainPhase::TxStreaming);
                // Fire before any CRC work: DMA CH4 has only one byte of TX
                // prefetch, so the wire-side jitter window is just the time
                // from here to `dma::enable`. The CRC patch below races a
                // (payload_end - 1) byte-time slack against DMA's read cursor.
                fire_now();
                accumulate_snoop();
                patch_crc();
                if dma::remaining(dma::Channel::CH4) <= 2 {
                    report_fault(FastChainFault::CrcPatchDeadlineMiss);
                }
                if current_bytes_walked() < expected_predecessor_bytes as u32 {
                    report_fault(FastChainFault::PreviousSlotTimeout);
                }
                set_phase(FastChainPhase::CrcPatched);
            }
            FastChainPhase::TxStreaming
            | FastChainPhase::CrcPatched
            | FastChainPhase::Done
            | FastChainPhase::Fault(_) => {}
        },
    }
}

fn byte_time_ticks_now() -> u32 {
    DXL_BYTE_TIME_TICKS.load(Ordering::Relaxed)
}

fn current_bytes_walked() -> u32 {
    // SAFETY: see on_systick.
    match unsafe { *STATE.get() } {
        ReplyState::Chain { bytes_walked, .. } => bytes_walked,
        _ => 0,
    }
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
pub fn on_dma1_ch5_tc() {
    // Unconditional clear: a stuck TCIF re-asserts the IRQ on next unmask even
    // if STATE has moved past the snoop window.
    dma::clear_tc_flag(dma::Channel::CH5);

    let phase_now = unsafe {
        match &*STATE.get() {
            ReplyState::Chain { phase, .. } => Some(*phase),
            _ => None,
        }
    };
    let Some(phase_now) = phase_now else { return };
    if !matches!(
        phase_now,
        FastChainPhase::CatchupArmed | FastChainPhase::Snoop | FastChainPhase::TxArmed
    ) {
        return;
    }

    unsafe {
        if let ReplyState::Chain {
            snoop_head,
            bulk_crc,
            bytes_walked,
            ..
        } = &mut *STATE.get()
        {
            let ring = &*DXL_RX_BUF.get();
            let start = *snoop_head as usize;
            let walked = DXL_RX_BUF_LEN - start;
            if walked > 0 {
                *bulk_crc = crc16_continue(*bulk_crc, &ring[start..]);
                *bytes_walked = bytes_walked.wrapping_add(walked as u32);
            }
            *snoop_head = 0;
        }
    }

    if matches!(phase_now, FastChainPhase::CatchupArmed) {
        set_phase(FastChainPhase::Snoop);
    }
}

pub fn cancel() {
    systick::set_irq(false);
    systick::clear_match();
    dma::set_tcie(dma::Channel::CH5, false);
    set_state(ReplyState::Idle);
}

pub fn report_dma_overrun() {
    report_fault(FastChainFault::DmaOverrun);
}

pub fn report_uart_error() {
    report_fault(FastChainFault::UartError);
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn patch_crc() {
    // SAFETY: see on_systick. Sole writer to DXL_TX_BUF in ISR context.
    unsafe {
        let buf = &mut *DXL_TX_BUF.get();
        let n = buf.len();
        let payload_end = n.saturating_sub(2);
        let seed = match *STATE.get() {
            ReplyState::Chain { bulk_crc, .. } => bulk_crc,
            _ => 0,
        };
        let crc = crc16_continue(seed, &buf[..payload_end]);
        let bytes = crc.to_le_bytes();
        let slice = buf.as_mut_slice();
        slice[n - 2] = bytes[0];
        slice[n - 1] = bytes[1];
    }
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn accumulate_snoop() {
    let write_pos = current_rx_write_pos();
    // SAFETY: see on_systick.
    unsafe {
        if let ReplyState::Chain {
            snoop_head,
            bulk_crc,
            bytes_walked,
            ..
        } = &mut *STATE.get()
        {
            if *snoop_head == write_pos {
                return;
            }
            let ring = &*DXL_RX_BUF.get();
            let prior = *snoop_head;
            *bulk_crc = ring_crc(*bulk_crc, ring, prior, write_pos);
            let delta = write_pos.wrapping_sub(prior) & RX_MASK_U16;
            *bytes_walked = bytes_walked.wrapping_add(delta as u32);
            *snoop_head = write_pos;
        }
    }
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
fn ring_crc(seed: u16, ring: &[u8], head: u16, write_pos: u16) -> u16 {
    if head == write_pos {
        return seed;
    }
    let start = head as usize;
    let end = write_pos as usize;
    if start < end {
        crc16_continue(seed, &ring[start..end])
    } else {
        let mid = crc16_continue(seed, &ring[start..]);
        crc16_continue(mid, &ring[..end])
    }
}

fn current_rx_write_pos() -> u16 {
    let remaining = dma::remaining(dma::Channel::CH5);
    (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining) & RX_MASK_U16
}

fn set_state(new: ReplyState) {
    // SAFETY: see on_systick.
    unsafe { *STATE.get() = new };
}

fn set_phase(new: FastChainPhase) {
    // SAFETY: see on_systick. The borrow on STATE is released before the
    // error path runs, since cancel() re-enters STATE. Calls outside Chain
    // are silent no-ops, not faults — set_phase chains after `cancel()` need
    // to be safe to re-enter without double-faulting.
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
    }
}

fn is_legal_transition(from: FastChainPhase, to: FastChainPhase) -> bool {
    use FastChainPhase::*;
    if matches!(to, Fault(_)) {
        return true;
    }
    matches!(
        (from, to),
        (CatchupArmed, Snoop)
            | (CatchupArmed, Catchup)
            | (Snoop, Catchup)
            | (Catchup, TxArmed)
            | (TxArmed, TxStreaming)
            | (TxStreaming, CrcPatched)
            | (CrcPatched, Done)
    )
}

fn report_fault(f: FastChainFault) {
    unsafe {
        let link = &raw mut (*SHARED.table.telemetry.get()).link;
        let counter: *mut u32 = match f {
            FastChainFault::IllegalTransition => &raw mut (*link).illegal_transition,
            FastChainFault::UnexpectedByteCount => &raw mut (*link).unexpected_byte_count,
            FastChainFault::PreviousSlotTimeout => &raw mut (*link).previous_slot_timeout,
            FastChainFault::SlotTimingMiss => &raw mut (*link).slot_timing_miss,
            FastChainFault::CrcPatchDeadlineMiss => &raw mut (*link).crc_patch_deadline_miss,
            FastChainFault::DmaOverrun => &raw mut (*link).dma_overrun,
            FastChainFault::UartError => &raw mut (*link).uart_error,
        };
        counter.write_volatile(counter.read_volatile().wrapping_add(1));
    }
}

#[cfg(test)]
mod tests {
    use super::{FastChainPhase, is_legal_transition, ring_crc};
    use dxl_protocol::crc16;

    const RING: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

    #[test]
    fn empty_walk_returns_seed_unchanged() {
        assert_eq!(ring_crc(0x1234, &RING, 3, 3), 0x1234);
        assert_eq!(ring_crc(0, &RING, 0, 0), 0);
    }

    #[test]
    fn non_wrap_walk_matches_contiguous_crc() {
        assert_eq!(ring_crc(0, &RING, 2, 5), crc16(&[0x30, 0x40, 0x50]));
    }

    #[test]
    fn wrap_walk_stitches_tail_and_head() {
        assert_eq!(ring_crc(0, &RING, 6, 2), crc16(&[0x70, 0x80, 0x10, 0x20]));
    }

    #[test]
    fn seed_chaining_matches_single_pass() {
        let mid = ring_crc(0, &RING, 1, 4);
        let chained = ring_crc(mid, &RING, 4, 7);
        assert_eq!(chained, ring_crc(0, &RING, 1, 7));
    }

    #[test]
    fn seed_chaining_survives_wrap_boundary() {
        let pre = ring_crc(0, &RING, 5, 0);
        let post = ring_crc(pre, &RING, 0, 3);
        assert_eq!(post, ring_crc(0, &RING, 5, 3));
    }

    #[test]
    fn legal_transitions_form_chain() {
        use FastChainPhase::*;
        assert!(is_legal_transition(CatchupArmed, Snoop));
        assert!(is_legal_transition(CatchupArmed, Catchup));
        assert!(is_legal_transition(Snoop, Catchup));
        assert!(is_legal_transition(Catchup, TxArmed));
        assert!(is_legal_transition(TxArmed, TxStreaming));
        assert!(is_legal_transition(TxStreaming, CrcPatched));
        assert!(is_legal_transition(CrcPatched, Done));
    }

    #[test]
    fn illegal_transitions_rejected() {
        use FastChainPhase::*;
        assert!(!is_legal_transition(CatchupArmed, TxArmed));
        assert!(!is_legal_transition(Snoop, TxStreaming));
        assert!(!is_legal_transition(TxArmed, Catchup));
        assert!(!is_legal_transition(CrcPatched, TxArmed));
    }
}
