//! Wire-time translation for `Ch32Bus::send`. The chip owns this layer so the
//! services dispatcher can stay protocol-pure (structural byte offsets only)
//! and not import a baud-rate enum.

use core::sync::atomic::Ordering;

use super::statics::DXL_US_PER_BYTE_Q16;

/// Bytes of inter-slot bus turnaround / worst-case stuffing margin added on
/// top of each Status frame in a Sync/Bulk reply train. Vendor convention;
/// not specified by the DXL 2.0 protocol manual. Stays chip-side because
/// it's a wire-layer hedge against TX_EN switching delay and clock skew.
///
/// Only meaningful for Sync/Bulk Read at slot_index > 0; single-target
/// replies (Ping/Read/Write/Action/Reboot/Calibrate/RegWrite/Error) all
/// schedule with `slot_index = 0`, and Fast replies take a different path
/// (`fire_fast`) that never touches this constant.
///
/// Stuffing bound: each preceding slot can add up to ⌊data_len/3⌋ wire
/// bytes from FF FF FD → FF FF FD FD insertion. 4 bytes/slot fully covers
/// the worst case for `data_len ≤ 14`, which fits every read in our
/// current control table. For larger reads with adversarial data (e.g.
/// multiple signed fields holding small negatives → 0xFF MSBs aligning),
/// the prediction can land short and our TX collides with the prior
/// slot's tail — corrupting both replies on that turn. Acceptable at
/// current payload sizes; revisit (see task #99) when the bound is
/// approached.
pub(crate) const SLOT_MARGIN: u32 = 4;

/// Translate a wire-byte offset to whole µs at the current effective baud.
/// Uses [`DXL_US_PER_BYTE_Q16`] so the hot path is one multiply + shift, no
/// divide (RV32EC has no hardware divide).
#[inline]
pub(crate) fn bytes_to_us(bytes: u32) -> u32 {
    let q16 = DXL_US_PER_BYTE_Q16.load(Ordering::Relaxed);
    ((bytes as u64 * q16 as u64) >> 16) as u32
}

/// Q8.8 µs variant (1 unit = 1/256 µs ≈ 3.9 ns) for chain Last fire
/// scheduling, where flooring to whole µs costs up to ~0.67 µs at 3M and
/// would fire ahead of the predecessor's last stop bit.
#[inline]
pub(crate) fn bytes_to_us_q88(bytes: u32) -> u32 {
    let q16 = DXL_US_PER_BYTE_Q16.load(Ordering::Relaxed);
    ((bytes as u64 * q16 as u64) >> 8) as u32
}
