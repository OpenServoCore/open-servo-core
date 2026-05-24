use core::sync::atomic::Ordering;
use portable_atomic::AtomicU32;

use crate::drop_oldest_ring::{DropOldestRing, RingAction};

#[derive(Copy, Clone)]
struct IdleStamp {
    bytes: u32,
    tick: u32,
}

type IdleRing = DropOldestRing<IdleStamp, 4>;

static IDLE_RING: IdleRing = IdleRing::new(IdleStamp { bytes: 0, tick: 0 });

/// Lives outside `IDLE_RING` so the running total survives drop-oldest evictions.
static BYTES_AT_IDLE: AtomicU32 = AtomicU32::new(0);

fn record_into(ring: &IdleRing, counter: &AtomicU32, delta_bytes: u16, tick: u32) {
    let bytes = counter
        .load(Ordering::Relaxed)
        .wrapping_add(delta_bytes as u32);
    counter.store(bytes, Ordering::Relaxed);
    ring.push(IdleStamp { bytes, tick });
}

fn pop_matching_from(ring: &IdleRing, parsed_end: u32) -> Option<u32> {
    ring.pop_matching(|stamp| {
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

pub fn record(delta_bytes: u16, tick: u32) {
    record_into(&IDLE_RING, &BYTES_AT_IDLE, delta_bytes, tick);
}

/// Returns the tick of the stamp at `parsed_end`; older stamps are dropped,
/// newer stamps stay for later calls.
pub fn pop_matching(parsed_end: u32) -> Option<u32> {
    pop_matching_from(&IDLE_RING, parsed_end)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fresh() -> (IdleRing, AtomicU32) {
        (
            IdleRing::new(IdleStamp { bytes: 0, tick: 0 }),
            AtomicU32::new(0),
        )
    }

    #[test]
    fn record_accumulates_bytes_across_calls() {
        let (ring, counter) = fresh();
        record_into(&ring, &counter, 4, 100);
        record_into(&ring, &counter, 6, 200);
        assert_eq!(pop_matching_from(&ring, 4), Some(100));
        assert_eq!(pop_matching_from(&ring, 10), Some(200));
    }

    #[test]
    fn pop_matching_takes_exact_skips_past_stops_at_future() {
        let (ring, counter) = fresh();
        record_into(&ring, &counter, 4, 100); // bytes = 4
        record_into(&ring, &counter, 6, 200); // bytes = 10
        record_into(&ring, &counter, 5, 300); // bytes = 15
        // parsed_end = 10 → skips 4, takes 10, leaves 15 for next call.
        assert_eq!(pop_matching_from(&ring, 10), Some(200));
        assert_eq!(pop_matching_from(&ring, 15), Some(300));
    }

    #[test]
    fn pop_matching_returns_none_when_parsed_end_between_stamps() {
        let (ring, counter) = fresh();
        record_into(&ring, &counter, 5, 100); // bytes = 5
        record_into(&ring, &counter, 5, 200); // bytes = 10
        // parsed_end = 7 → skip 5, stop at 10. Stamp at 5 is gone; 10 remains.
        assert_eq!(pop_matching_from(&ring, 7), None);
        assert_eq!(pop_matching_from(&ring, 10), Some(200));
    }

    #[test]
    fn cumulative_counter_survives_drop_oldest_eviction() {
        let (ring, counter) = fresh();
        // N=4 holds 3 entries; 5 pushes force two evictions.
        for n in 1u32..=5 {
            record_into(&ring, &counter, 10, n * 100);
        }
        // Oldest two stamps (bytes=10,20) evicted; counter still at 50.
        assert_eq!(pop_matching_from(&ring, 10), None);
        assert_eq!(pop_matching_from(&ring, 20), None);
        assert_eq!(pop_matching_from(&ring, 50), Some(500));
    }

    #[test]
    fn match_logic_survives_u32_wrap() {
        let (ring, counter) = fresh();
        counter.store(u32::MAX - 5, Ordering::Relaxed);
        record_into(&ring, &counter, 10, 100); // bytes wraps to 4
        // parsed_end = u32::MAX - 3 is "behind" the wrapped stamp: wrap-aware
        // delta is positive (+8) → Stop, leave stamp.
        assert_eq!(pop_matching_from(&ring, u32::MAX - 3), None);
        // parsed_end = 4 (post-wrap) → exact match.
        assert_eq!(pop_matching_from(&ring, 4), Some(100));
    }
}
