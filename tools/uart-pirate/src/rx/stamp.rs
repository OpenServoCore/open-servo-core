//! Drain-time stamp synthesis. `BBATCH`/`BDRAIN` walk the byte ring and
//! emit one record per byte — the same host-facing shape as the old
//! walker pipeline — with ticks synthesized here instead of captured
//! per byte:
//!
//! - a byte with a boundary entry gets its real capture tick and the
//!   [`flags::BOUNDARY`] flag;
//! - the byte after a boundary strides [`BREAK_TO_NEXT_BITS`] (the
//!   boundary tick sits at the break's framed end, one stop bit before
//!   the next start), every later byte strides a full byte-time — for
//!   our own DMA-fed TX echo that stride is crystal-exact;
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
/// Boundary tick → next byte's start: the FE service tick sits at the
/// break's framed end (fall + ~10 bit-times), and a gapless sender's
/// next start bit falls one stop bit later.
const BREAK_TO_NEXT_BITS: u32 = 1;

#[derive(Copy, Clone)]
pub struct ByteRecord {
    pub tick: u32,
    pub byte: u8,
    pub flags: u8,
}

pub mod flags {
    /// No boundary anchor since reset — the tick is a placeholder
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
/// data (detected both before and after the copy — DMA keeps writing
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
    let avail = total.wrapping_sub(tail0) as usize;
    let n = avail.min(out.len());
    let mut last_tick = unsafe { ptr::read_volatile(LAST_TICK.get()) };
    let mut prev_boundary = unsafe { ptr::read_volatile(PREV_WAS_BOUNDARY.get()) };
    let mut anchored = unsafe { ptr::read_volatile(ANCHORED.get()) };

    for (i, slot) in out.iter_mut().take(n).enumerate() {
        let idx = tail0.wrapping_add(i as u32);
        let byte = rings::rx_at(idx);
        let (tick, f) = match boundary::take_for(idx) {
            Some(t) => {
                anchored = true;
                prev_boundary = true;
                (t, flags::BOUNDARY)
            }
            None => {
                let stride = if prev_boundary {
                    BREAK_TO_NEXT_BITS
                } else {
                    BITS_PER_BYTE_8N1
                };
                prev_boundary = false;
                (
                    last_tick.wrapping_add(stride * bit),
                    if anchored { 0 } else { flags::COUNT_UNDER },
                )
            }
        };
        last_tick = tick;
        *slot = ByteRecord {
            tick,
            byte,
            flags: f,
        };
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
    TAIL.store(tail0.wrapping_add(n as u32), Ordering::Release);
    Ok(n)
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
