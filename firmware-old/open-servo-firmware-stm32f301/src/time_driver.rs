//! Embassy time driver using TIM2.
//!
//! TIM2 is configured as a 32-bit free-running counter at 1MHz (1µs per tick).
//! This provides the time source for embassy-time's Duration and Instant types.
//!
//! The driver uses a simple wake queue - embassy-time handles the scheduling.

use core::cell::RefCell;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use stm32f3::stm32f301::TIM2;

/// Maximum pending wakers.
const WAKE_QUEUE_SIZE: usize = 4;

struct WakeEntry {
    at: u64,
    waker: Option<Waker>,
}

impl WakeEntry {
    const fn new() -> Self {
        Self {
            at: u64::MAX,
            waker: None,
        }
    }
}

/// TIM2-based embassy time driver.
pub struct Tim2Driver {
    wakers: Mutex<RefCell<[WakeEntry; WAKE_QUEUE_SIZE]>>,
}

impl Tim2Driver {
    const fn new() -> Self {
        Self {
            wakers: Mutex::new(RefCell::new([
                WakeEntry::new(),
                WakeEntry::new(),
                WakeEntry::new(),
                WakeEntry::new(),
            ])),
        }
    }

    /// Check and fire expired wakers. Call this periodically from SysTick.
    pub fn check_wakers(&self) {
        let now = self.now();
        critical_section::with(|cs| {
            let mut wakers = self.wakers.borrow(cs).borrow_mut();
            for entry in wakers.iter_mut() {
                if entry.at <= now {
                    if let Some(waker) = entry.waker.take() {
                        waker.wake();
                    }
                    entry.at = u64::MAX;
                }
            }
        });
    }
}

impl Driver for Tim2Driver {
    fn now(&self) -> u64 {
        // SAFETY: We only read the counter register, which is safe from any context.
        let cnt = unsafe { (*TIM2::ptr()).cnt.read().bits() };
        cnt as u64
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut wakers = self.wakers.borrow(cs).borrow_mut();

            // Find a free slot or the slot with matching waker
            let mut slot = None;
            for (i, entry) in wakers.iter().enumerate() {
                if entry
                    .waker
                    .as_ref()
                    .map(|w| w.will_wake(waker))
                    .unwrap_or(false)
                {
                    slot = Some(i);
                    break;
                }
                if entry.waker.is_none() && slot.is_none() {
                    slot = Some(i);
                }
            }

            if let Some(i) = slot {
                wakers[i] = WakeEntry {
                    at,
                    waker: Some(waker.clone()),
                };
            }
        });
    }
}

// Global driver instance
embassy_time_driver::time_driver_impl!(static DRIVER: Tim2Driver = Tim2Driver::new());

/// Check alarms - call this from SysTick or similar periodic interrupt.
#[inline]
pub fn check_alarms() {
    DRIVER.check_wakers();
}
