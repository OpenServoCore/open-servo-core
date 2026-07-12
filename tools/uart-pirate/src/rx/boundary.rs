//! Break-boundary recorder: one `(tick32, byte index)` entry per USART3
//! RX-error service. On this wire FE means "a break just rang" (breaks
//! ring as exactly one `0x00`, 1:1 with FE — protocol facts F1/F2), so
//! each entry pins a real capture tick to a frame boundary; NE/ORE
//! garble takes the same path and anchors the garbled byte instead.
//!
//! Attribution: the service reads the ring cursor and claims the newest
//! `0x00` within the last [`ATTRIB_WINDOW`] bytes. Our own frames are
//! gapless (break stop bit → first data byte is one bit-time), so at 3M
//! the header byte often rings before the service enters — but ID, LEN,
//! and INST are never `0x00` on a valid frame, so the scan is
//! unambiguous within the window. A garbled non-zero byte (no `0x00` in
//! the window) anchors the newest byte as a best effort.
//!
//! Storm discipline (the latched-flag lesson, `osc-servo-transport.md`
//! §6 A4): the flags are latched and level-pended; a service with no new
//! ring bytes is a re-fire, not an event. [`STORM_LIMIT`] consecutive
//! zero-progress entries mask the event (CTLR3.EIE); ring progress seen
//! by the IDLE service, or any frame-send arm point, re-enables it.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::USART3;
use portable_atomic::{AtomicU32, Ordering};

use crate::rx::rings;

/// Boundary ring depth. Bounded by the byte ring's own drain contract:
/// a break costs at least 2 ring bytes (bare break + the next frame's
/// break), so a full byte ring holds at most `RING_LEN / 2` boundaries
/// — 512 would be airtight, 256 covers every real traffic shape (frames
/// carry ≥ 5 bytes per boundary) with 2 KB instead of 4.
const BLEN: usize = 256;
const BMASK: u32 = (BLEN - 1) as u32;
const _: () = assert!(BLEN.is_power_of_two());

/// How far back the `0x00` attribution scan reaches, in bytes. Bounds
/// the tolerated FE-service lag; the three bytes after a break are
/// ID/LEN/INST (never `0x00`), so within 4 the scan cannot claim frame
/// data. No ISR on this firmware runs long enough to lag further.
const ATTRIB_WINDOW: u32 = 4;

/// Consecutive zero-progress services before the event is masked.
/// More than 1 because a fresh break's re-fire can genuinely beat its
/// successor byte at low baud (break stop bit = one bit-time = 2 µs at
/// 0.5M, longer than the service itself).
const STORM_LIMIT: u8 = 4;

static TICKS: SyncUnsafeCell<[u32; BLEN]> = SyncUnsafeCell::new([0; BLEN]);
static IDXS: SyncUnsafeCell<[u32; BLEN]> = SyncUnsafeCell::new([0; BLEN]);

/// Cumulative boundaries recorded (SPSC head; the drain owns the tail).
pub(super) static HEAD: AtomicU32 = AtomicU32::new(0);
pub(super) static TAIL: AtomicU32 = AtomicU32::new(0);

/// Byte index of the newest recorded boundary — the attribution scan's
/// floor. `u32::MAX` = none since reset (indexes restart well below it).
static LAST_IDX: SyncUnsafeCell<u32> = SyncUnsafeCell::new(u32::MAX);
/// `rx_total` at the previous service — the zero-progress discriminator.
static LAST_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static ZERO_PROGRESS: SyncUnsafeCell<u8> = SyncUnsafeCell::new(0);

/// RX-error service body. `now` is the vector's entry tick.
pub(super) fn on_rx_error(now: u32) {
    let total = rings::refresh_rx_total();
    unsafe {
        if total == ptr::read_volatile(LAST_TOTAL.get()) {
            // Latched re-fire: no byte, no boundary. Bound the storm.
            let n = ptr::read_volatile(ZERO_PROGRESS.get()).saturating_add(1);
            ptr::write_volatile(ZERO_PROGRESS.get(), n);
            if n >= STORM_LIMIT {
                USART3.ctlr3().modify(|w| w.set_eie(false));
            }
            return;
        }
        ptr::write_volatile(LAST_TOTAL.get(), total);
        ptr::write_volatile(ZERO_PROGRESS.get(), 0);

        let last_idx = ptr::read_volatile(LAST_IDX.get());
        let mut chosen = total.wrapping_sub(1);
        for k in 0..ATTRIB_WINDOW {
            let idx = total.wrapping_sub(1 + k);
            if idx == last_idx {
                break;
            }
            if rings::rx_at(idx) == 0x00 {
                chosen = idx;
                break;
            }
        }
        if chosen == last_idx {
            return;
        }
        ptr::write_volatile(LAST_IDX.get(), chosen);

        let head = HEAD.load(Ordering::Relaxed);
        let i = (head & BMASK) as usize;
        (*TICKS.get())[i] = now;
        (*IDXS.get())[i] = chosen;
        HEAD.store(head.wrapping_add(1), Ordering::Release);
    }
}

/// Ring progress observed by the IDLE service: bytes are flowing again,
/// so the armed SR half retires latched flags and the event is safe to
/// re-arm after a storm mask.
pub(super) fn on_rx_progress() {
    unsafe { ptr::write_volatile(ZERO_PROGRESS.get(), 0) };
    if !USART3.ctlr3().read().eie() {
        USART3.ctlr3().modify(|w| w.set_eie(true));
    }
}

/// Consume the boundary at the drain's cursor if it anchors byte `idx`.
/// Entries behind `idx` (skipped by a reset) are discarded in passing.
pub(super) fn take_for(idx: u32) -> Option<u32> {
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
        let (b_idx, b_tick) = unsafe { ((*IDXS.get())[i], (*TICKS.get())[i]) };
        let delta = idx.wrapping_sub(b_idx);
        if delta == 0 {
            TAIL.store(tail.wrapping_add(1), Ordering::Release);
            return Some(b_tick);
        }
        if delta <= u32::MAX / 2 {
            // Boundary is behind the drain cursor — stale, discard.
            TAIL.store(tail.wrapping_add(1), Ordering::Release);
            continue;
        }
        return None;
    }
}

pub(super) fn clear() {
    let head = HEAD.load(Ordering::Relaxed);
    TAIL.store(head, Ordering::Release);
    unsafe {
        ptr::write_volatile(LAST_IDX.get(), u32::MAX);
        ptr::write_volatile(ZERO_PROGRESS.get(), 0);
    }
}
