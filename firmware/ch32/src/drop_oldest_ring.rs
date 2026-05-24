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
