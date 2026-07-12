//! Break-boundary recorder: one `(tick32, byte index)` entry per LIN
//! break detect (LBD) service. LBD is line-level based — 10 consecutive
//! dominant bits (LBDL=0), the protocol-law break length — so it fires
//! for the pirate's OWN echo breaks exactly as it does for foreign
//! ones, always at bit 10 after the fall. FE/EIE was tried first and
//! never latches for breaks on this V203 (2026-07-12: zero services
//! across ping traffic with EIE set and verified bit mappings, while
//! the same vector's TC branch ran); LBD also clears by a safe
//! write-0, so none of the latched-flag storm machinery exists here.
//!
//! Two record flavors, matching what LIN reception does with the break
//! CHARACTER:
//!
//! - **Attached** — the break rang a `0x00` (any break longer than the
//!   exact-10 law shape, e.g. today's servo SBK): the record anchors
//!   that ring byte.
//! - **Standalone** — LIN consumed the character (an exact-10 break is
//!   its textbook delimiter; our own echoes, and law-shaped foreign
//!   breaks once the fleet migrates): the record marks a break BETWEEN
//!   ring bytes, positioned before the next byte to ring; the drain
//!   re-emits it as a synthetic `0x00` stamp carrying this real tick.
//!
//! Attribution is entry-anchored: the break character, when it exists,
//! frames within ±1 bit of the LBD assert, so only the newest byte at
//! service entry and bytes arriving during the fresh-byte spin are
//! candidates — older ring bytes are pre-break traffic by construction.
//! (Residual: a swallowed break whose predecessor frame ends in a
//! `0x00` CRC byte can mis-attach to it, ~1/256 of law-shaped breaks;
//! costs one exchange parse, bounded, tracked with the band.)

use core::cell::SyncUnsafeCell;
use core::ptr;

use portable_atomic::{AtomicU32, Ordering};

use crate::rx::{rings, stamp};

/// Boundary ring depth. Bounded by the byte ring's own drain contract:
/// even bare-break trains cost ≥ 2 boundary-ring entries per 3 wire
/// events, and real traffic carries ≥ 5 bytes per boundary — 256 covers
/// every shape the bench produces between drains.
const BLEN: usize = 256;
const BMASK: u32 = (BLEN - 1) as u32;
const _: () = assert!(BLEN.is_power_of_two());

/// Attribution scan cap in bytes (belt over the entry-anchored window).
const ATTRIB_WINDOW: u32 = 4;

/// LBD asserts at the 10th dominant bit; a break character (when one
/// rings) frames within a bit of it, plus a DMA beat. Two bit-times
/// bounds the wait at any baud; an exact-10 break whose character LIN
/// consumed exits here and records standalone.
fn fresh_byte_spin() -> u32 {
    2 * stamp::bit_ticks() + 72
}

static TICKS: SyncUnsafeCell<[u32; BLEN]> = SyncUnsafeCell::new([0; BLEN]);
static IDXS: SyncUnsafeCell<[u32; BLEN]> = SyncUnsafeCell::new([0; BLEN]);
static STANDALONE: SyncUnsafeCell<[bool; BLEN]> = SyncUnsafeCell::new([false; BLEN]);

/// Cumulative boundaries recorded (SPSC head; the drain owns the tail).
pub(super) static HEAD: AtomicU32 = AtomicU32::new(0);
pub(super) static TAIL: AtomicU32 = AtomicU32::new(0);

/// Byte index of the newest attached boundary — the attribution floor.
/// `u32::MAX` = none since reset.
static LAST_IDX: SyncUnsafeCell<u32> = SyncUnsafeCell::new(u32::MAX);
/// `rx_total` at the previous service — the fresh-byte race gate.
static LAST_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Service-entry / boundary-recorded counters for the `BDIAG` probe.
pub(super) static SERVICES: AtomicU32 = AtomicU32::new(0);
pub(super) static RECORDS: AtomicU32 = AtomicU32::new(0);

/// Break-detect service body. `now` is the vector's entry tick.
pub(super) fn on_break(now: u32) {
    SERVICES.fetch_add(1, Ordering::Relaxed);
    unsafe {
        let entry_total = rings::refresh_rx_total();
        let prev_total = ptr::read_volatile(LAST_TOTAL.get());
        let mut total = entry_total;
        let progressed = loop {
            if total != prev_total {
                break true;
            }
            if crate::tick::read_tick32().wrapping_sub(now) > fresh_byte_spin() {
                break false;
            }
            total = rings::refresh_rx_total();
        };
        ptr::write_volatile(LAST_TOTAL.get(), total);

        let mut attached: Option<u32> = None;
        if progressed {
            // Candidates: the newest pre-entry byte and spin arrivals —
            // anything older rang before the break's own low span.
            // (Wrapped max: the cumulative counters lap u32 in a ~4 h
            // 3M soak, so plain `max` is not sound here.)
            let entry_newest = entry_total.wrapping_sub(1);
            let floor = if prev_total.wrapping_sub(entry_newest) <= u32::MAX / 2 {
                prev_total
            } else {
                entry_newest
            };
            let last_idx = ptr::read_volatile(LAST_IDX.get());
            let mut k = 0;
            let mut idx = total.wrapping_sub(1);
            while k < ATTRIB_WINDOW && idx.wrapping_sub(floor) <= u32::MAX / 2 && idx != last_idx {
                if rings::rx_at(idx) == 0x00 {
                    attached = Some(idx);
                    break;
                }
                idx = idx.wrapping_sub(1);
                k += 1;
            }
        }

        let (idx, standalone) = match attached {
            Some(i) => {
                ptr::write_volatile(LAST_IDX.get(), i);
                (i, false)
            }
            // The character was consumed (LIN's delimiter handling of a
            // law-shaped break) or never zero: the break sits before the
            // next byte to ring.
            None => (total, true),
        };

        let head = HEAD.load(Ordering::Relaxed);
        let i = (head & BMASK) as usize;
        (*TICKS.get())[i] = now;
        (*IDXS.get())[i] = idx;
        (*STANDALONE.get())[i] = standalone;
        HEAD.store(head.wrapping_add(1), Ordering::Release);
        RECORDS.fetch_add(1, Ordering::Relaxed);
    }
}

/// What the boundary queue says about the drain's ring position `pos`.
pub(super) enum Bound {
    /// A break with no ring byte of its own, positioned at or before
    /// `pos`: the drain emits a synthetic `0x00` stamp with this tick.
    Standalone(u32),
    /// The byte AT `pos` is a ringed break character: this tick times it.
    Attached(u32),
}

/// Consume the next boundary relevant at ring position `pos`, if any.
/// Stale attached entries (behind a reset) are discarded in passing.
pub(super) fn next_at(pos: u32) -> Option<Bound> {
    loop {
        let tail = TAIL.load(Ordering::Relaxed);
        let head = HEAD.load(Ordering::Acquire);
        if tail == head {
            return None;
        }
        // Overrun (only reachable after the byte ring itself lapped —
        // stamp_overflow fires first): snap to the oldest intact entry.
        let tail = if head.wrapping_sub(tail) > BLEN as u32 {
            head.wrapping_sub(BLEN as u32)
        } else {
            tail
        };
        let i = (tail & BMASK) as usize;
        let (b_idx, b_tick, standalone) =
            unsafe { ((*IDXS.get())[i], (*TICKS.get())[i], (*STANDALONE.get())[i]) };
        let behind = pos.wrapping_sub(b_idx) <= u32::MAX / 2;
        if standalone {
            if behind {
                TAIL.store(tail.wrapping_add(1), Ordering::Release);
                return Some(Bound::Standalone(b_tick));
            }
            return None;
        }
        if b_idx == pos {
            TAIL.store(tail.wrapping_add(1), Ordering::Release);
            return Some(Bound::Attached(b_tick));
        }
        if behind {
            // Attached entry behind the drain cursor — stale, discard.
            TAIL.store(tail.wrapping_add(1), Ordering::Release);
            continue;
        }
        return None;
    }
}

/// `BDIAG` snapshot: (services, records, head, tail).
pub(super) fn diag() -> (u32, u32, u32, u32) {
    (
        SERVICES.load(Ordering::Relaxed),
        RECORDS.load(Ordering::Relaxed),
        HEAD.load(Ordering::Relaxed),
        TAIL.load(Ordering::Relaxed),
    )
}

/// Drop pending boundaries and re-anchor the progress gate at `rx_total`
/// (without this, the first post-reset service reads every byte since
/// the previous service as fresh progress and can mis-attach — the
/// 2026-07-12 stale-window bug).
pub(super) fn clear(rx_total: u32) {
    let head = HEAD.load(Ordering::Relaxed);
    TAIL.store(head, Ordering::Release);
    unsafe {
        ptr::write_volatile(LAST_IDX.get(), u32::MAX);
        ptr::write_volatile(LAST_TOTAL.get(), rx_total);
    }
}
