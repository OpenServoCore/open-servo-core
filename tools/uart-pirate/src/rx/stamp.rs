//! Drain-time stamp synthesis. `BBATCH`/`BDRAIN` walk the byte ring and
//! emit one record per byte -- the same host-facing shape as the old
//! walker pipeline -- with ticks synthesized here instead of captured
//! per byte:
//!
//! - a byte with an attached boundary gets its real capture tick,
//!   lifted back to the modeled break fall ([`BOUNDARY_LIFT_BITS`]),
//!   and the [`flags::BOUNDARY`] flag;
//! - a standalone boundary (a law-shaped break whose character the LIN
//!   receiver consumed -- our own echoes) is RE-EMITTED as a synthetic
//!   `0x00` stamp with its real tick, so the host-visible stream stays
//!   byte-identical to a ringed break;
//! - the byte after a boundary strides the break frame's own span
//!   ([`BREAK_TO_NEXT_BITS`]), every later byte a full byte-time -- for
//!   our own DMA-fed TX echo those strides are crystal-exact;
//! - bytes with no boundary seen since reset carry [`flags::COUNT_UNDER`]
//!   (tick is a placeholder cadence, not a measurement).
//!
//! Ticks are therefore boundary-anchored: absolute per-byte accuracy is
//! gone, but every load-bearing bench quantity (turnaround, chain gaps,
//! seam spans) differences boundary-flavor ticks, where the anchor's
//! service latency cancels.
//!
//! Single consumer by construction: the USB command context is the only
//! caller of the drain functions and of `reset_to` (via `rx::reset`).

use core::cell::SyncUnsafeCell;
use core::ptr;

use portable_atomic::{AtomicU32, Ordering};

use crate::rx::boundary;
use crate::rx::desync::{self, DesyncCause};
use crate::rx::rings::{self, RING_LEN};
use crate::tick::read_tick32;

/// 8N1 wire framing: 1 start + 8 data + 1 stop = 10 bit-times per byte.
const BITS_PER_BYTE_8N1: u32 = 10;
/// Captured boundary ticks are lifted back to the modeled break FALL:
/// the receiver frames a break's `0x00` exactly 10 bit-times after the
/// fall -- for ANY break length (FE is the stop-bit verdict) -- so the
/// subtraction is shape-independent and break stamps keep the old
/// IC-era "tick ~ fall" convention every consumer was built on.
const BOUNDARY_LIFT_BITS: u32 = 10;
/// Break fall -> next byte's start: a 10-bit-exact break plus its stop
/// bit (our own frames; a longer SBK break makes foreign interiors read
/// a few bits early, which nothing consumes quantitatively).
const BREAK_TO_NEXT_BITS: u32 = 11;

#[derive(Copy, Clone)]
pub struct ByteRecord {
    pub tick: u32,
    pub byte: u8,
    pub flags: u8,
}

pub mod flags {
    /// No boundary anchor since reset -- the tick is a placeholder
    /// cadence, not a measurement.
    pub const COUNT_UNDER: u8 = 1 << 0;
    /// The tick is a real capture from the RX-error service: this byte
    /// is a wire boundary (a break, or garble).
    pub const BOUNDARY: u8 = 1 << 1;
}

/// Bit time in tick32 ticks (= BRR at HCLK-rate APB1). Set on baud
/// changes, read by the stride synthesis.
static BIT_TICKS: AtomicU32 = AtomicU32::new(0);

/// Cumulative bytes the host has consumed.
static TAIL: AtomicU32 = AtomicU32::new(0);

static LAST_TICK: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static PREV_WAS_BOUNDARY: SyncUnsafeCell<bool> = SyncUnsafeCell::new(false);
static ANCHORED: SyncUnsafeCell<bool> = SyncUnsafeCell::new(false);

pub(super) fn set_bit_ticks(brr: u32) {
    BIT_TICKS.store(brr, Ordering::Relaxed);
}

pub(super) fn bit_ticks() -> u32 {
    BIT_TICKS.load(Ordering::Relaxed)
}

/// Records queued for drain. Hosts pace `BBATCH` calls against this.
pub fn stamps_available() -> u32 {
    let total = critical_section::with(|_| rings::refresh_rx_total());
    total.wrapping_sub(TAIL.load(Ordering::Relaxed))
}

pub fn drain_byte() -> Result<Option<ByteRecord>, DesyncCause> {
    let mut one = [ByteRecord {
        tick: 0,
        byte: 0,
        flags: 0,
    }];
    Ok(match drain_batch(&mut one)? {
        0 => None,
        _ => Some(one[0]),
    })
}

/// Drain up to `out.len()` byte records into `out`. Returns the count
/// written, or the sticky desync cause if the ring lapped undrained
/// data (detected both before and after the copy -- DMA keeps writing
/// while we read).
pub fn drain_batch(out: &mut [ByteRecord]) -> Result<usize, DesyncCause> {
    if let Some(cause) = desync::desync_cause() {
        return Err(cause);
    }
    let total = critical_section::with(|_| rings::refresh_rx_total());
    let tail0 = TAIL.load(Ordering::Relaxed);
    if total.wrapping_sub(tail0) > RING_LEN as u32 {
        desync::set(DesyncCause::StampOverflow);
        return Err(DesyncCause::StampOverflow);
    }

    let bit = bit_ticks();
    let avail = total.wrapping_sub(tail0);
    let mut last_tick = unsafe { ptr::read_volatile(LAST_TICK.get()) };
    let mut prev_boundary = unsafe { ptr::read_volatile(PREV_WAS_BOUNDARY.get()) };
    let mut anchored = unsafe { ptr::read_volatile(ANCHORED.get()) };

    let mut consumed = 0u32;
    let mut written = 0usize;
    while written < out.len() {
        let pos = tail0.wrapping_add(consumed);
        let (byte, tick, f) = match boundary::next_at(pos) {
            // A break whose character LIN consumed: re-emit it as a
            // synthetic 0x00 stamp carrying the real capture tick --
            // the stream the host sees stays byte-identical to a
            // ringed break, and no ring byte is spent... unless the
            // swallow didn't happen after all: it is NONDETERMINISTIC
            // on this die (bench: a law-shaped break's
            // character sometimes rings late, past the recorder's
            // spin), and the duplicate zero corrupted reply headers.
            // A frame can never START with 0x00 (ID is never zero), so
            // a ring zero at the standalone's own position IS the
            // break's late character -- fold it into the synthetic.
            Some(boundary::Bound::Standalone(t)) => {
                crate::dbg::mark_standalone();
                if consumed < avail && rings::rx_at(pos) == 0x00 {
                    consumed += 1;
                }
                (
                    0x00,
                    t.wrapping_sub(BOUNDARY_LIFT_BITS * bit),
                    flags::BOUNDARY,
                )
            }
            Some(boundary::Bound::Attached(t)) => {
                consumed += 1;
                (
                    rings::rx_at(pos),
                    t.wrapping_sub(BOUNDARY_LIFT_BITS * bit),
                    flags::BOUNDARY,
                )
            }
            None => {
                if consumed >= avail {
                    break;
                }
                consumed += 1;
                let stride = if prev_boundary {
                    BREAK_TO_NEXT_BITS
                } else {
                    BITS_PER_BYTE_8N1
                };
                (
                    rings::rx_at(pos),
                    last_tick.wrapping_add(stride * bit),
                    if anchored { 0 } else { flags::COUNT_UNDER },
                )
            }
        };
        anchored |= f & flags::BOUNDARY != 0;
        prev_boundary = f & flags::BOUNDARY != 0;
        last_tick = tick;
        out[written] = ByteRecord {
            tick,
            byte,
            flags: f,
        };
        written += 1;
    }

    // The copy read live DMA memory: if the writer lapped past the batch
    // base while we walked, some of what we copied was overwritten.
    let total_after = critical_section::with(|_| rings::refresh_rx_total());
    if total_after.wrapping_sub(tail0) > RING_LEN as u32 {
        desync::set(DesyncCause::StampOverflow);
        return Err(DesyncCause::StampOverflow);
    }

    unsafe {
        ptr::write_volatile(LAST_TICK.get(), last_tick);
        ptr::write_volatile(PREV_WAS_BOUNDARY.get(), prev_boundary);
        ptr::write_volatile(ANCHORED.get(), anchored);
    }
    TAIL.store(tail0.wrapping_add(consumed), Ordering::Release);
    Ok(written)
}

pub(super) fn reset_to(rx_total: u32) {
    TAIL.store(rx_total, Ordering::Release);
    unsafe {
        // Seed the pre-anchor placeholder cadence near now.
        ptr::write_volatile(LAST_TICK.get(), read_tick32());
        ptr::write_volatile(PREV_WAS_BOUNDARY.get(), false);
        ptr::write_volatile(ANCHORED.get(), false);
    }
}
