use core::sync::atomic::Ordering;

use ch32_metapac::USART1;

use dxl_protocol::CrcUmts;

use super::Ch32DxlCrc;
#[cfg(feature = "scope")]
use crate::hal::gpio::Level;
use crate::hal::{dma, gpio, systick, usart};

use super::scheduler::{GUARD_BYTES, catchup_interval_ticks};
use super::state::{
    DISPATCH, FastChainFault, FastChainPhase, ReplyState, STATE, report_fault, set_phase,
};
#[cfg(feature = "scope")]
use super::statics::DXL_DBG_PIN;
use super::statics::{
    DXL_BYTE_TIME_TICKS, DXL_RX_BUF, DXL_RX_BUF_LEN, DXL_TX_BUF, DXL_TX_EN, RX_MASK_U16,
};

/// TX_EN and DMA CH4 stay off so the bus remains in RX through any preceding
/// snoop window; `fire_now` flips both at the slot deadline.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
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

/// Scope trigger pulse on `DXL_DBG_PIN`. Moved between sites as the bench
/// experiment requires. Compile-time no-op without `--features scope`.
#[inline(always)]
fn dbg_pulse() {
    #[cfg(feature = "scope")]
    // SAFETY: written once at bring-up before any ISR can read.
    if let Some(p) = unsafe { *DXL_DBG_PIN.get() } {
        gpio::set_level(p, Level::High);
        gpio::set_level(p, Level::Low);
    }
}

/// SysTick ISR entry. Path-uniform: load the cached dispatch fn and call it.
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
pub fn on_systick() {
    systick::clear_match();
    systick::set_irq(false);
    // SAFETY: USART1 / SysTick share PFIC HIGH and never preempt each other;
    // DISPATCH access is uncontested across these handlers.
    let dispatch = unsafe { *DISPATCH.get() };
    dispatch();
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
pub(super) fn body_noop() {}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
pub(super) fn body_plain_fire() {
    fire_now();
    super::state::set_state(ReplyState::Idle);
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
pub(super) fn body_chain_catchup() {
    accumulate_snoop();
    // SAFETY: see on_systick. Dispatch == body_chain_catchup implies Chain
    // with PeriodicCatchup phase; fall through to no-op otherwise.
    let (fire_tick, last_catchup_tick, walk_deadline, n_pred) = unsafe {
        match *STATE.get() {
            ReplyState::Chain {
                fire_tick,
                last_catchup_tick,
                walk_deadline,
                expected_predecessor_bytes,
                ..
            } => (
                fire_tick,
                last_catchup_tick,
                walk_deadline,
                expected_predecessor_bytes,
            ),
            _ => return,
        }
    };

    let interval = catchup_interval_ticks();
    let now = systick::ticks();
    let byte_time = byte_time_ticks_now();
    let half_byte = (byte_time / 2) as i32;
    let is_last_catchup = (now.wrapping_sub(last_catchup_tick) as i32) >= -half_byte;

    if !is_last_catchup {
        // Snap to the anchor if `now + interval` would overshoot. ISR-entry
        // drift accumulates across intermediate ticks; anchoring the final
        // tick keeps fire on the wall-clock.
        let candidate = now.wrapping_add(interval);
        let next_tick = if (candidate.wrapping_sub(last_catchup_tick) as i32) >= -half_byte {
            last_catchup_tick
        } else {
            candidate
        };
        systick::set_cmp(next_tick);
        systick::set_irq(true);
        if (systick::ticks().wrapping_sub(next_tick) as i32) >= 0 {
            on_systick();
        }
        return;
    }

    // Last catchup. Busy-wait + fold until `n_pred` reached or
    // `walk_deadline` hits. The deadline exit is normal
    // (`bytes_walked = n_pred − GUARD`); post-fire absorbs the trailing
    // GUARD bytes.
    wait_and_fold_until(n_pred as u32, walk_deadline);
    dbg_pulse();

    // Hand off to TxArmed: arm SysTick at fire_tick, return. The
    // `GUARD_BYTES ≥ 2` invariant guarantees `walk_deadline < fire_tick`,
    // so the CPU yields cleanly and no past-tick re-entry is needed here.
    set_phase(FastChainPhase::TxArmed);
    systick::set_cmp(fire_tick);
    systick::set_irq(true);
}

#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
#[inline(never)]
#[unsafe(no_mangle)]
pub(super) fn body_chain_fast_fire() {
    // ── Critical path. patch_crc must land bytes [n-2, n-1] of TX_BUF before
    // CH4's read cursor reaches them.
    fire_now();
    // Sample entry tick immediately post-fire so the slot_timing_miss check
    // reflects CMP→fire latency rather than the predecessor walk's natural
    // n_pred·byte_time duration.
    let entry_now = systick::ticks();
    // SAFETY: see on_systick. Dispatch == body_chain_fast_fire implies Chain
    // with TxArmed phase; fall through to no-op otherwise.
    let (fire_tick, walk_deadline, n_pred) = unsafe {
        match *STATE.get() {
            ReplyState::Chain {
                fire_tick,
                walk_deadline,
                expected_predecessor_bytes,
                ..
            } => (fire_tick, walk_deadline, expected_predecessor_bytes),
            _ => return,
        }
    };
    if n_pred > 0 {
        // Post-fire walks bytes that arrive between walk_deadline and
        // t_prior_end. Add one byte_time of slack past t_prior_end for DMA
        // latch latency on the final byte.
        let byte_time = byte_time_ticks_now();
        let post_deadline =
            walk_deadline.wrapping_add(byte_time.saturating_mul((GUARD_BYTES + 1) as u32));
        wait_and_fold_until(n_pred as u32, post_deadline);
    }
    patch_crc();

    // ── Off the critical path: diagnostics + FSM transitions.
    if dma::remaining(dma::Channel::CH4) <= 2 {
        report_fault(FastChainFault::CrcPatchDeadlineMiss);
    }
    if current_bytes_walked() < n_pred as u32 {
        report_fault(FastChainFault::PreviousSlotTimeout);
    }
    let byte_time = byte_time_ticks_now();
    if (entry_now.wrapping_sub(fire_tick) as i32) > byte_time as i32 {
        report_fault(FastChainFault::SlotTimingMiss);
    }
}

#[inline(always)]
fn byte_time_ticks_now() -> u32 {
    DXL_BYTE_TIME_TICKS.load(Ordering::Relaxed)
}

#[inline(always)]
fn current_bytes_walked() -> u32 {
    // SAFETY: see on_systick.
    match unsafe { *STATE.get() } {
        ReplyState::Chain { bytes_walked, .. } => bytes_walked,
        _ => 0,
    }
}

// Inlined into `body_chain_fast_fire` (its only caller); the body lands
// in `.highcode` via that caller, so no explicit section is needed. The
// CALL/RET savings shift the first STORE to `TX_BUF[n-2]` earlier by
// ~10 cycles, widening the DMA-fetch race window the patch must win.
#[inline(always)]
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
        let crc = Ch32DxlCrc::accumulate(seed, &buf[..payload_end]);
        let bytes = crc.to_le_bytes();
        let slice = buf.as_mut_slice();
        slice[n - 2] = bytes[0];
        slice[n - 1] = bytes[1];
    }
}

/// Busy-wait + fold until `bytes_walked ≥ target` OR `now ≥ deadline`.
/// Both pre-fire and post-fire walks share this primitive — they differ
/// only in the deadline (absolute SysTick tick).
// Inlined into both caller bodies (`body_chain_catchup`,
// `body_chain_fast_fire`) — both are in `.highcode`, so the duplicated
// loop bodies stay in RAM. Saves one CALL/RET per shot and lets the
// compiler hoist the inner `accumulate_snoop`/deadline math.
#[inline(always)]
fn wait_and_fold_until(target: u32, deadline: u32) {
    loop {
        accumulate_snoop();
        if current_bytes_walked() >= target {
            return;
        }
        if (systick::ticks().wrapping_sub(deadline) as i32) >= 0 {
            return;
        }
    }
}

// Called from the `wait_and_fold_until` busy-loop; CALL/RET on every
// iteration would dominate the poll cadence. Inlining lands the body
// in each caller's `.highcode` frame and lets the compiler keep
// `*STATE` field references in registers across iterations.
#[inline(always)]
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

// Folded into `accumulate_snoop` (its only caller) so the slice
// arithmetic + CRC table walk land directly in the snoop hot path,
// inheriting `.highcode` from the surrounding inline chain.
#[inline(always)]
fn ring_crc(seed: u16, ring: &[u8], head: u16, write_pos: u16) -> u16 {
    if head == write_pos {
        return seed;
    }
    let start = head as usize;
    let end = write_pos as usize;
    if start < end {
        Ch32DxlCrc::accumulate(seed, &ring[start..end])
    } else {
        let mid = Ch32DxlCrc::accumulate(seed, &ring[start..]);
        Ch32DxlCrc::accumulate(mid, &ring[..end])
    }
}

#[inline(always)]
fn current_rx_write_pos() -> u16 {
    let remaining = dma::remaining(dma::Channel::CH5);
    (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining) & RX_MASK_U16
}

#[cfg(test)]
mod tests {
    use super::Ch32DxlCrc;
    use super::ring_crc;
    use dxl_protocol::CrcUmts;

    const RING: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

    #[test]
    fn empty_walk_returns_seed_unchanged() {
        assert_eq!(ring_crc(0x1234, &RING, 3, 3), 0x1234);
        assert_eq!(ring_crc(0, &RING, 0, 0), 0);
    }

    #[test]
    fn non_wrap_walk_matches_contiguous_crc() {
        assert_eq!(
            ring_crc(0, &RING, 2, 5),
            Ch32DxlCrc::accumulate(0, &[0x30, 0x40, 0x50])
        );
    }

    #[test]
    fn wrap_walk_stitches_tail_and_head() {
        assert_eq!(
            ring_crc(0, &RING, 6, 2),
            Ch32DxlCrc::accumulate(0, &[0x70, 0x80, 0x10, 0x20])
        );
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
}
