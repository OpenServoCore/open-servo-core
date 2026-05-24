use core::cell::SyncUnsafeCell;
use core::sync::atomic::Ordering;
use portable_atomic::AtomicU8;

pub enum RingAction<R> {
    /// Advance past this entry and keep walking.
    Skip,
    /// Advance past this entry and return `Some(value)`.
    Take(R),
    /// Leave this entry in place and return `None`.
    Stop,
}

/// SPSC ring with drop-oldest producer. `N` must be a power of two and ≤ 256.
pub struct DropOldestRing<T: Copy, const N: usize> {
    cells: SyncUnsafeCell<[T; N]>,
    head: AtomicU8,
    tail: AtomicU8,
}

impl<T: Copy, const N: usize> DropOldestRing<T, N> {
    pub const fn new(init: T) -> Self {
        const {
            assert!(N.is_power_of_two() && N <= 256);
        }
        Self {
            cells: SyncUnsafeCell::new([init; N]),
            head: AtomicU8::new(0),
            tail: AtomicU8::new(0),
        }
    }

    pub fn push(&self, item: T) {
        let mask = (N as u8).wrapping_sub(1);
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        // SAFETY: sole producer; the consumer wraps its walk in a critical
        // section, so the slot at `head` is uncontested.
        unsafe {
            (*self.cells.get())[head as usize] = item;
        }
        let next = (head + 1) & mask;
        if next == tail {
            self.tail.store((tail + 1) & mask, Ordering::Relaxed);
        }
        self.head.store(next, Ordering::Release);
    }

    /// Walks oldest-to-newest under a critical section that masks the producer.
    pub fn pop_matching<R, F>(&self, mut f: F) -> Option<R>
    where
        F: FnMut(T) -> RingAction<R>,
    {
        critical_section::with(|_| {
            let mask = (N as u8).wrapping_sub(1);
            let head = self.head.load(Ordering::Relaxed);
            let mut tail = self.tail.load(Ordering::Relaxed);
            let mut hit = None;
            while tail != head {
                // SAFETY: the producer ISR is masked by the surrounding CS.
                let item = unsafe { (*self.cells.get())[tail as usize] };
                match f(item) {
                    RingAction::Stop => break,
                    RingAction::Skip => tail = (tail + 1) & mask,
                    RingAction::Take(r) => {
                        tail = (tail + 1) & mask;
                        hit = Some(r);
                        break;
                    }
                }
            }
            self.tail.store(tail, Ordering::Relaxed);
            hit
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn take_eq(target: u32) -> impl FnMut(u32) -> RingAction<u32> {
        move |x| {
            if x < target {
                RingAction::Skip
            } else if x == target {
                RingAction::Take(x)
            } else {
                RingAction::Stop
            }
        }
    }

    #[test]
    fn empty_ring_returns_none() {
        let r: DropOldestRing<u32, 4> = DropOldestRing::new(0);
        assert_eq!(r.pop_matching(take_eq(1)), None);
    }

    #[test]
    fn take_matching_advances_past_it() {
        let r: DropOldestRing<u32, 4> = DropOldestRing::new(0);
        r.push(1);
        r.push(2);
        r.push(3);
        assert_eq!(r.pop_matching(take_eq(2)), Some(2));
        // 1 was Skip'd, 2 was Take'n; 3 remains.
        assert_eq!(r.pop_matching(take_eq(3)), Some(3));
        assert_eq!(r.pop_matching(take_eq(3)), None);
    }

    #[test]
    fn stop_leaves_future_entries_in_place() {
        let r: DropOldestRing<u32, 4> = DropOldestRing::new(0);
        r.push(5);
        // Target 3 is below 5 → Stop on first entry.
        assert_eq!(r.pop_matching(take_eq(3)), None);
        // Entry still there for a later call.
        assert_eq!(r.pop_matching(take_eq(5)), Some(5));
    }

    #[test]
    fn skip_drops_stale_entries_permanently() {
        let r: DropOldestRing<u32, 4> = DropOldestRing::new(0);
        r.push(1);
        r.push(2);
        r.push(3);
        // Target 3 skips 1 and 2.
        assert_eq!(r.pop_matching(take_eq(3)), Some(3));
        // 1 and 2 are gone.
        r.push(4);
        assert_eq!(r.pop_matching(take_eq(2)), None);
        assert_eq!(r.pop_matching(take_eq(4)), Some(4));
    }

    #[test]
    fn drop_oldest_evicts_when_full() {
        // N=4 means at most 3 entries before drop-oldest kicks in
        // (one slot is left empty to disambiguate full vs empty).
        let r: DropOldestRing<u32, 4> = DropOldestRing::new(0);
        r.push(1);
        r.push(2);
        r.push(3);
        // Fourth push evicts the oldest (1).
        r.push(4);
        assert_eq!(r.pop_matching(take_eq(1)), None);
        assert_eq!(r.pop_matching(take_eq(4)), Some(4));
    }

    #[test]
    fn head_and_tail_wrap_after_many_cycles() {
        let r: DropOldestRing<u32, 4> = DropOldestRing::new(0);
        for n in 1u32..=20 {
            r.push(n);
            assert_eq!(r.pop_matching(take_eq(n)), Some(n));
        }
    }
}
