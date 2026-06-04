use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;
use heapless::Vec;
use portable_atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU16, AtomicU32};

use crate::board::TxEn;
use crate::hal::{Pin, systick};

pub(crate) const DXL_RX_BUF_LEN: usize = 512;

const _: () = assert!(DXL_RX_BUF_LEN.is_power_of_two() && DXL_RX_BUF_LEN <= u16::MAX as usize + 1);

pub(super) const RX_MASK_U16: u16 = (DXL_RX_BUF_LEN as u16).wrapping_sub(1);
pub(crate) const RX_MASK_U32: u32 = (DXL_RX_BUF_LEN - 1) as u32;

pub(crate) static DXL_RX_BUF: SyncUnsafeCell<[u8; DXL_RX_BUF_LEN]> =
    SyncUnsafeCell::new([0; DXL_RX_BUF_LEN]);

/// USART1 IDLE handler's previous DMA write index; used to compute the byte
/// delta between consecutive IDLE events. ISR-local state.
pub(crate) static DXL_RX_WRITE_POS: AtomicU16 = AtomicU16::new(0);

pub(crate) const DXL_TX_BUF_LEN: usize = 256;

pub(crate) static DXL_TX_BUF: SyncUnsafeCell<Vec<u8, DXL_TX_BUF_LEN>> =
    SyncUnsafeCell::new(Vec::new());

/// Written once during `bring_up_dxl` before USART1 IRQ is unmasked; read-only thereafter.
pub(crate) static DXL_TX_EN: SyncUnsafeCell<Option<TxEn>> = SyncUnsafeCell::new(None);

/// Scope debug pin — bench instrumentation for the chain-CRC ISRs. Seeded
/// once at bring-up; ISR readers only.
pub(crate) static DXL_DBG_PIN: SyncUnsafeCell<Option<Pin>> = SyncUnsafeCell::new(None);

/// Set by `Ch32Device::reboot`; USART1 TC ISR fires the soft reset after TX drains.
pub(crate) static DXL_REBOOT_PENDING: AtomicBool = AtomicBool::new(false);

/// Pending USART1 BRR change requested by a control-table BAUD write. 0 = none.
/// TC ISR consumes after clearing DXL_TX_BUF so the Status reply ships at the
/// old wire rate (host can still decode), then the next byte is at the new rate.
pub(crate) static DXL_BAUD_PENDING_BRR: AtomicU32 = AtomicU32::new(0);

/// HCLK ticks for 9 bit-times on the wire (brr × 9); on_usart1_idle
/// backdates the request end tick by this to recover the moment the
/// last byte's stop bit completed (IDLE counts the stop bit as bit 1
/// of the 10 it needs, so it asserts 9 bit-times later).
pub(crate) static DXL_CHAR_TIME_TICKS: AtomicU32 = AtomicU32::new(0);

pub(crate) static DXL_BYTE_TIME_TICKS: AtomicU32 = AtomicU32::new(0);

/// `(1 << 16) / byte_time_us` — precomputed reciprocal so the snoop ISR
/// converts µs → bytes with a multiply + shift instead of a runtime divide
/// (RV32EC + zmmul has no hardware divide).
pub(crate) static DXL_BYTES_PER_US_Q16: AtomicU32 = AtomicU32::new(0);

/// Per-chip clock_fine_trim residual converted to HCLK ticks, summed at the
/// fire site with the per-path entry-tick constant from
/// [`crate::dxl::calibration`]. Updated from USART1 TC after a Write touches
/// `comms.clock_fine_trim_us`. Signed: a negative Q8.8 nudges fire later by
/// partially cancelling the per-path constant.
pub(crate) static FIRE_ADVANCE_FINE_TICKS: AtomicI16 = AtomicI16::new(0);

/// Pending HSI trim delta queued by a `comms.clock_trim` Write. `i16::MIN`
/// (outside i8's native range) is the no-pending sentinel; USART1 TC swaps
/// it back to apply.
pub(crate) const CLOCK_TRIM_NO_PENDING: i16 = i16::MIN;
pub(crate) static DXL_CLOCK_TRIM_PENDING: AtomicI16 = AtomicI16::new(CLOCK_TRIM_NO_PENDING);

/// Pending Q8.8 µs sub-trim residual queued by a `comms.clock_fine_trim_us`
/// Write. `i32::MIN` (outside i16's native range) is the no-pending sentinel.
pub(crate) const CLOCK_FINE_TRIM_NO_PENDING: i32 = i32::MIN;
pub(crate) static DXL_CLOCK_FINE_TRIM_PENDING: AtomicI32 = AtomicI32::new(CLOCK_FINE_TRIM_NO_PENDING);

/// Q8.8 µs → ticks → atomic publish. Fire site sums with the per-path
/// `TX_{PLAIN,FAST}_LATENCY_TICKS` and clamps the total to ≥ 0 there.
pub(crate) fn recompute_fire_advance_fine_ticks(q88_us: i16) {
    let residual_ticks = (q88_us as i32 * systick::TICKS_PER_US as i32) >> 8;
    let clamped = residual_ticks.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    FIRE_ADVANCE_FINE_TICKS.store(clamped, Ordering::Release);
}

/// Called at bring-up and after a BAUD_RATE write — never on the snoop hot
/// path, so the runtime division is fine here.
pub(crate) fn store_baud_derived(brr: u32) {
    DXL_CHAR_TIME_TICKS.store(brr.wrapping_mul(9), Ordering::Relaxed);
    let byte_time_ticks = brr.wrapping_mul(10);
    DXL_BYTE_TIME_TICKS.store(byte_time_ticks, Ordering::Relaxed);
    // Round-to-nearest on the reciprocal so the ISR's product floors at
    // the true completed-byte count; plain truncation drifts ~1 byte
    // low on long snoops.
    let bytes_per_us_q16 = ((systick::TICKS_PER_US << 16) + byte_time_ticks / 2)
        .checked_div(byte_time_ticks)
        .unwrap_or(0);
    DXL_BYTES_PER_US_Q16.store(bytes_per_us_q16, Ordering::Relaxed);
}
