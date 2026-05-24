use core::sync::atomic::Ordering;
use portable_atomic::AtomicU32;

use crate::drop_oldest_ring::{DropOldestRing, RingAction};

#[derive(Copy, Clone)]
struct IdleStamp {
    bytes: u32,
    tick: u32,
}

static IDLE_RING: DropOldestRing<IdleStamp, 4> =
    DropOldestRing::new(IdleStamp { bytes: 0, tick: 0 });

/// Lives outside `IDLE_RING` so the running total survives drop-oldest evictions.
static BYTES_AT_IDLE: AtomicU32 = AtomicU32::new(0);

pub fn record(delta_bytes: u16, tick: u32) {
    let bytes = BYTES_AT_IDLE
        .load(Ordering::Relaxed)
        .wrapping_add(delta_bytes as u32);
    BYTES_AT_IDLE.store(bytes, Ordering::Relaxed);
    IDLE_RING.push(IdleStamp { bytes, tick });
}

/// Returns the tick of the stamp at `parsed_end`; older stamps are dropped,
/// newer stamps stay for later calls.
pub fn pop_matching(parsed_end: u32) -> Option<u32> {
    IDLE_RING.pop_matching(|stamp| {
        let delta = stamp.bytes.wrapping_sub(parsed_end) as i32;
        if delta > 0 {
            RingAction::Stop
        } else if delta == 0 {
            RingAction::Take(stamp.tick)
        } else {
            RingAction::Skip
        }
    })
}
