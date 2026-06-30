//! Predict-and-snap PLL walker. Drains IC + RX rings into the stamp ring
//! one byte at a time, with bytes' start ticks anchored to IC entries
//! (cold-start path) or predicted as `prev_anchor + 10·bit_ticks` and
//! snapped to the closest IC entry within `±SNAP_BITS·bit_ticks` (steady
//! state). On miss, the walker free-runs on the prediction and flags
//! `COUNT_UNDER` — anchor still updates, so the prediction chain survives
//! a short edge dropout.

use core::cell::SyncUnsafeCell;
use core::ptr;

use portable_atomic::{AtomicU32, Ordering};

use crate::rx::desync::{self, DesyncCause};
use crate::rx::filter;
use crate::rx::rings::{self, FALL_LEN};
use crate::rx::stamp::{self, BYTE_HEAD, BYTE_TAIL, STAMP_LEN};
use crate::rx::stamp::flags::COUNT_UNDER;
use crate::tick::read_tick32;

/// 8N1 wire framing: 1 start + 8 data + 1 stop = 10 bit-times per byte.
const BITS_PER_BYTE_8N1: u32 = 10;

/// Half-width of the PLL snap window in bit-times. Each byte's start is
/// predicted at `prev_anchor + BITS_PER_BYTE_8N1·bit_ticks`; the walker
/// scans `[predicted − SNAP_BITS·bit_ticks, predicted + SNAP_BITS·bit_ticks]`
/// for the closest IC entry. 1 bit-time is ~50× the observed loopback
/// jitter floor (tool-pirate-tune stage 1: dev ≤ 24 ticks at brr=2500,
/// = 0.01 bit-times). Widen to 2 or 3 if real upstream chips drive more
/// inter-byte hardware idle than 1 bit-time between bytes in a chain.
const SNAP_BITS: u32 = 1;

pub(super) static WALKED_FALLING: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Last emitted byte's start tick (lifted u32). Source of the prediction
/// chain — next byte's start is `LAST_ANCHOR + 10·bit_ticks` ± SNAP_BITS·
/// bit_ticks. Meaningless when `HAS_ANCHOR == false`.
static LAST_ANCHOR: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
/// `false` at cold boot, after `reset_walker`, and after every `set_baud`.
/// First post-reset byte anchors on the next unconsumed IC entry (no
/// prediction yet); subsequent bytes use the PLL with `LAST_ANCHOR`.
static HAS_ANCHOR: SyncUnsafeCell<bool> = SyncUnsafeCell::new(false);

/// Bit-time in tick32 ticks. Refreshed on `set_baud`. brr at HCLK = ticks
/// per bit (USART3 sits on APB1 which runs at HCLK).
pub(super) static BIT_TICKS: AtomicU32 = AtomicU32::new(0);

/// `ceiling` snapshot at the walker's most recent exit. Reported by
/// `ic_snapshot` as the chip-side "now" reference so the host can place
/// the IC ring window relative to wall time, and reused as the
/// wrap-race correction `ceiling` for entries the walker has already
/// confirmed are in the ring.
pub(super) static LAST_LIFT_CEILING: AtomicU32 = AtomicU32::new(0);

#[inline]
pub(super) fn walked() -> u32 {
    unsafe { ptr::read_volatile(WALKED_FALLING.get()) }
}

pub(super) fn reset_anchors(falling_total: u32) {
    unsafe {
        ptr::write_volatile(WALKED_FALLING.get(), falling_total);
        // Drop the PLL chain. The next byte the walker emits will anchor
        // on its own IC entry (no prediction) — same cold-start path as
        // boot, so post-reset behaviour matches first-packet behaviour.
        ptr::write_volatile(HAS_ANCHOR.get(), false);
    }
}

/// Predict-and-snap PLL walker. Constructs `ceiling` internally: refresh
/// the IC NDTR snapshot first, THEN read `tick32`. Ordering guarantees
/// `combined ≤ ceiling` for every non-raced entry in `[walked,
/// falling_total)` — newer entries that land between the NDTR refresh
/// and the tick32 read aren't in this walker's snapshot, so they fall
/// to the next trigger.
///
/// `idle == true` marks this invocation as USART-IDLE-triggered: the
/// wire has been quiet one character time, so the next byte (when it
/// arrives) is the start of a new burst — not contiguous with the last.
/// After draining, the walker flushes trailing interior edges and drops
/// `has_anchor` so the next byte cold-starts on its own IC edge instead
/// of free-running off a stale `last_anchor + 10·bit_ticks` prediction
/// that would sit one inter-packet gap (RDT) before the real edge. IDLE
/// is signal-only here — the next byte's tick still comes from its own
/// IC entry via the cold-start path.
pub fn walk(idle: bool) {
    if desync::is_desynced() {
        return;
    }
    let bit_ticks = BIT_TICKS.load(Ordering::Relaxed);
    if bit_ticks == 0 {
        return;
    }
    let byte_period = BITS_PER_BYTE_8N1.wrapping_mul(bit_ticks);
    let snap = SNAP_BITS.wrapping_mul(bit_ticks);
    let cc_filter_delay = filter::delay_ticks();

    let falling_total = rings::refresh_falling_total();
    let rx_total = rings::refresh_rx_total();
    let ceiling = read_tick32();
    LAST_LIFT_CEILING.store(ceiling, Ordering::Release);
    let mut walked = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };

    // ic_overrun: if more new entries arrived than the ring can hold,
    // we lost edges → permanent lockstep desync. Sticky-fatal.
    if falling_total.wrapping_sub(walked) > FALL_LEN as u32 {
        desync::set(DesyncCause::IcOverrun);
        return;
    }

    let mut byte_head = BYTE_HEAD.load(Ordering::Relaxed);
    let mut anchor = unsafe { ptr::read_volatile(LAST_ANCHOR.get()) };
    let mut has_anchor = unsafe { ptr::read_volatile(HAS_ANCHOR.get()) };

    while byte_head != rx_total {
        // stamp_overflow: if the host hasn't drained, the next emit
        // would overwrite an unread stamp. Sticky-fatal; host must RESET.
        // Check is at the loop head so byte_head is NOT advanced for the
        // byte we refused to emit.
        let byte_tail = BYTE_TAIL.load(Ordering::Acquire);
        if byte_head.wrapping_sub(byte_tail) >= STAMP_LEN as u32 {
            desync::set(DesyncCause::StampOverflow);
            break;
        }
        let byte = rings::rx_at(byte_head);

        let chosen_anchor: u32;
        let mut flags = 0u8;

        if !has_anchor {
            // Cold-start path: post-boot, post-RESET, post-set_baud. No
            // prediction available, so anchor on the first unconsumed IC
            // entry. Need `ceiling` to bound `byte_period` past
            // first_edge so we don't anchor on what is actually some
            // interior edge of the same byte whose start bit is still
            // unwalked.
            if walked == falling_total {
                break;
            }
            let first_edge = rings::falling_at(walked, falling_total, ceiling);
            if ceiling.wrapping_sub(first_edge) < byte_period {
                break;
            }
            chosen_anchor = first_edge;
            walked = walked.wrapping_add(1);
        } else {
            // Steady-state PLL. Predict the next start bit, then snap to
            // the IC entry closest to prediction within `±snap`.
            let predicted = anchor.wrapping_add(byte_period);
            let snap_high = predicted.wrapping_add(snap);
            // Yield mid-byte: `ceiling` must reach past `snap_high`
            // before we can rule out a real edge that's still in flight
            // toward the IC ring. Without this guard, an edge landing at
            // (predicted, snap_high] after the walker exits would be lost
            // — we'd have already stamped a free-run miss.
            if ceiling.wrapping_sub(snap_high) > u32::MAX / 2 {
                break;
            }
            let snap_low = predicted.wrapping_sub(snap);

            // Skip any leftover prev-byte interior edges (everything
            // sitting before `snap_low` is past byte N−1's start bit but
            // before byte N's snap window).
            while walked != falling_total {
                let tick = rings::falling_at(walked, falling_total, ceiling);
                if tick.wrapping_sub(snap_low) > u32::MAX / 2 {
                    walked = walked.wrapping_add(1);
                } else {
                    break;
                }
            }

            // Scan `[snap_low, snap_high]` for the closest-to-predicted
            // edge. Tiebreak prefers the later edge: real start bits come
            // AT or AFTER prediction (upstream chip inter-byte hardware
            // idle shifts the start LATER, never earlier), so on a tie
            // the later candidate is far more likely the real start than
            // the earlier one (which is then a glitch or stuck-edge
            // ringing tail).
            let mut chosen: Option<u32> = None;
            let mut chosen_walked = walked;
            let mut best_dist = u32::MAX;
            let mut probe = walked;
            while probe != falling_total {
                let tick = rings::falling_at(probe, falling_total, ceiling);
                // Past snap_high → stop scanning.
                if tick.wrapping_sub(snap_high) <= u32::MAX / 2 && tick != snap_high {
                    break;
                }
                let dist = if tick >= predicted {
                    tick.wrapping_sub(predicted)
                } else {
                    predicted.wrapping_sub(tick)
                };
                if dist < best_dist || (dist == best_dist && tick >= predicted) {
                    best_dist = dist;
                    chosen = Some(tick);
                    chosen_walked = probe.wrapping_add(1);
                }
                probe = probe.wrapping_add(1);
            }

            match chosen {
                Some(c) => {
                    chosen_anchor = c;
                    walked = chosen_walked;
                }
                None => {
                    // Miss. Free-run on the prediction so the chain
                    // survives one or two dropouts. `walked` was already
                    // advanced past everything below `snap_low`, so any
                    // remaining entries (between snap_high and the next
                    // byte's snap_low) stay in the ring for the next
                    // iteration's skip pass.
                    chosen_anchor = predicted;
                    flags |= COUNT_UNDER;
                }
            }
        }

        let start_tick = chosen_anchor.wrapping_sub(cc_filter_delay);
        stamp::emit(byte_head, byte, start_tick, flags);
        byte_head = byte_head.wrapping_add(1);
        anchor = chosen_anchor;
        has_anchor = true;
    }

    // Chain-break on IDLE-after-drain: USART IDLE means the wire just
    // went quiet. If the byte queue fully drained, any trailing IC
    // entries are interior edges of the last byte that the snap window
    // chose not to consume; they're stale once the next burst starts.
    // Drop them, drop the prediction chain, and the next byte cold-starts
    // on its own real IC edge. Without this, request/reply traffic (TX
    // echo → RDT silence → reply) tries to anchor reply byte 0 at
    // `last_echo_anchor + 10·bit_ticks` and free-runs every reply byte
    // because the real edges sit hundreds of bit-times past the snap.
    if idle && byte_head == rx_total {
        walked = falling_total;
        has_anchor = false;
    }

    BYTE_HEAD.store(byte_head, Ordering::Release);
    unsafe {
        ptr::write_volatile(WALKED_FALLING.get(), walked);
        ptr::write_volatile(LAST_ANCHOR.get(), anchor);
        ptr::write_volatile(HAS_ANCHOR.get(), has_anchor);
    }
}
