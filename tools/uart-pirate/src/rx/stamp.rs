//! Per-byte stamp ring. Walker-populated, host-drained via `drain_byte`
//! (one record) or `drain_batch` (bulk for BBATCH transport). Sized to
//! absorb realistic measurement bursts (host pulls between sends)
//! without overflowing while leaving 5+ KB stack headroom on the V203's
//! 20 KB SRAM. At 1024 records the three rings total 6 KB (`bytes_ring`
//! 1 KB + `ts_ring` 4 KB + `flags_ring` 1 KB) — projected bss ~14.9 KB,
//! ~5.5 KB stack room. Past 1024 records of undrained backlog the host
//! has either gone away or has a bench-script bug; either way
//! `stamp_overflow` trips and host must `RESET`.
//!
//! `bytes_ring` mirrors the byte value at stamp time, decoupling drain
//! from `rx_ring`'s DMA wrap. Without this mirror, a host that drains
//! long after stamping reads through to `rx_ring[i & RX_MASK]` which DMA
//! has since overwritten with a later byte → silent value corruption.

use core::cell::SyncUnsafeCell;

use portable_atomic::{AtomicU32, Ordering};

pub(super) const STAMP_LEN: usize = 1024;
const STAMP_MASK: u32 = (STAMP_LEN - 1) as u32;
const _: () = assert!(STAMP_LEN.is_power_of_two());

#[derive(Copy, Clone)]
pub struct ByteRecord {
    pub tick: u32,
    pub byte: u8,
    pub flags: u8,
}

pub mod flags {
    /// Walker predicted this byte's start at `prev_anchor + 10·bit_ticks`
    /// but found no IC entry in `±SNAP_BITS·bit_ticks` of the prediction;
    /// emitted `tick = predicted − cc_filter_delay` instead. Cause: real
    /// start edge eaten by CC filter, upstream chip drove an inter-byte
    /// gap exceeding the snap window, or pirate snapped on the wrong edge
    /// the byte before. Host treats this byte's `tick` as ±0.5·bit
    /// accurate, not sub-tick.
    pub const COUNT_UNDER: u8 = 1 << 0;
}

static BYTES_RING: SyncUnsafeCell<[u8; STAMP_LEN]> = SyncUnsafeCell::new([0; STAMP_LEN]);
static TS_RING: SyncUnsafeCell<[u32; STAMP_LEN]> = SyncUnsafeCell::new([0; STAMP_LEN]);
static FLAGS_RING: SyncUnsafeCell<[u8; STAMP_LEN]> = SyncUnsafeCell::new([0; STAMP_LEN]);

/// Cumulative count of bytes the walker has stamped (= ts_head). Read by
/// host-side drainers with Acquire to see the matching ring writes.
pub(super) static BYTE_HEAD: AtomicU32 = AtomicU32::new(0);
/// Cumulative count of bytes the host has consumed.
pub(super) static BYTE_TAIL: AtomicU32 = AtomicU32::new(0);

#[inline]
pub(super) fn emit(byte_idx: u32, byte: u8, tick: u32, flags: u8) {
    let i = (byte_idx & STAMP_MASK) as usize;
    unsafe {
        (*BYTES_RING.get())[i] = byte;
        (*TS_RING.get())[i] = tick;
        (*FLAGS_RING.get())[i] = flags;
    }
}

/// Stamps queued for drain (= `byte_head - byte_tail`). Host paces BBATCH
/// calls against this.
pub fn stamps_available() -> u32 {
    let head = BYTE_HEAD.load(Ordering::Acquire);
    let tail = BYTE_TAIL.load(Ordering::Relaxed);
    head.wrapping_sub(tail)
}

pub fn drain_byte() -> Option<ByteRecord> {
    let tail = BYTE_TAIL.load(Ordering::Relaxed);
    let head = BYTE_HEAD.load(Ordering::Acquire);
    if tail == head {
        return None;
    }
    let idx = (tail & STAMP_MASK) as usize;
    let rec = unsafe {
        ByteRecord {
            tick: (*TS_RING.get())[idx],
            byte: (*BYTES_RING.get())[idx],
            flags: (*FLAGS_RING.get())[idx],
        }
    };
    BYTE_TAIL.store(tail.wrapping_add(1), Ordering::Release);
    Some(rec)
}

/// Drain up to `out.len()` byte records into `out`. Returns the count
/// actually written. Cheaper per-byte than `drain_byte` for the BBATCH
/// host transport.
pub fn drain_batch(out: &mut [ByteRecord]) -> usize {
    let tail0 = BYTE_TAIL.load(Ordering::Relaxed);
    let head = BYTE_HEAD.load(Ordering::Acquire);
    let avail = head.wrapping_sub(tail0) as usize;
    let n = avail.min(out.len());
    for (i, slot) in out.iter_mut().take(n).enumerate() {
        let idx = (tail0.wrapping_add(i as u32) & STAMP_MASK) as usize;
        *slot = unsafe {
            ByteRecord {
                tick: (*TS_RING.get())[idx],
                byte: (*BYTES_RING.get())[idx],
                flags: (*FLAGS_RING.get())[idx],
            }
        };
    }
    if n > 0 {
        BYTE_TAIL.store(tail0.wrapping_add(n as u32), Ordering::Release);
    }
    n
}

pub(super) fn reset_to(rx_total: u32) {
    BYTE_HEAD.store(rx_total, Ordering::Release);
    BYTE_TAIL.store(rx_total, Ordering::Release);
}
