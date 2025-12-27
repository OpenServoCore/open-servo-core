//! Cross-domain "wire" primitive.
//!
//! `Mailbox<T>` is intended to live in **kernel-owned shared state** to carry values
//! between nodes running in different tick domains (fast/medium/slow/system).
//!
//! Typical use:
//! - slow loop writes a velocity setpoint for a fast loop
//! - medium loop writes a torque budget for a fast loop
//!
//! # Safety: Non-Atomic, Same-Context Assumption
//!
//! **`Mailbox<T>` is NOT atomic.** It assumes one of the following:
//!
//! 1. All tick domains run in the **same interrupt priority / thread context**, OR
//! 2. Callers use a **critical section** around read/write operations
//!
//! If `slow_tick` writes from one ISR priority and `fast_tick` reads from another
//! **without synchronization**, you have a **data race** (undefined behavior in Rust).
//!
//! This is intentional for performance: most embedded servo kernels run all ticks
//! in the same DMA/ADC ISR context or use a cooperative scheduler. If you need
//! ISR-safe cross-domain communication across priority levels, use:
//! - Atomics (e.g., `AtomicI32` for small values)
//! - A proper SPSC queue (e.g., `heapless::spsc::Queue`)
//! - Critical sections (e.g., `cortex_m::interrupt::free`)
//!
//! # Example
//!
//! ```rust,ignore
//! use open_servo_kernel_api::Mailbox;
//! use open_servo_kernel_api::units::DegPerSec10;
//!
//! pub struct State {
//!     pub vel_sp: Mailbox<DegPerSec10>,
//! }
//!
//! // slow tick:
//! state.vel_sp.write(DegPerSec10::from_dps10(120), slow_seq);
//! // fast tick:
//! let vel_sp = state.vel_sp.read();
//! ```

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mailbox<T: Copy> {
    val: T,
    seq: u32,
}

impl<T: Copy> Mailbox<T> {
    #[inline]
    pub const fn new(init: T) -> Self {
        Self { val: init, seq: 0 }
    }

    /// Write a new value with an associated sequence counter (domain-defined).
    #[inline]
    pub fn write(&mut self, v: T, seq: u32) {
        self.val = v;
        self.seq = seq;
    }

    /// Read the most recently written value.
    #[inline]
    pub fn read(&self) -> T {
        self.val
    }

    /// Sequence of the most recent write (optional diagnostic).
    #[inline]
    pub fn last_seq(&self) -> u32 {
        self.seq
    }
}
