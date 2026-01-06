//! Debug-only reentrancy guard (no atomics).
//!
//! This module provides [`DebugReentrancyGuard`], a helper for detecting
//! concurrent access to single-writer resources during development.
//!
//! # Contract
//!
//! The guarded resource is **single-writer**. Boards must ensure:
//! - Only one context holds the mutable reference at a time
//! - ISRs do not preempt while a tick is in progress (use priority masking)
//!
//! # CH32V006 Compatibility
//!
//! This module intentionally avoids `core::sync::atomic`. The guard uses
//! `Cell<Option<TickDomain>>` which is safe because the single-writer
//! contract forbids concurrent access.
//!
//! # Usage
//!
//! This guard is **optional**. Kernel and device implementations may embed
//! it to catch reentrancy bugs during development, but the API does not
//! enforce its use.
//!
//! ```rust,ignore
//! use open_servo_kernel_api::debug_guard::DebugReentrancyGuard;
//! use open_servo_kernel_api::TickDomain;
//!
//! struct MyKernel {
//!     guard: DebugReentrancyGuard,
//!     // ...
//! }
//!
//! impl MyKernel {
//!     fn tick(&mut self, domain: TickDomain) {
//!         self.guard.enter(domain);
//!         // ... do tick work ...
//!         self.guard.exit();
//!     }
//! }
//! ```

#[cfg(debug_assertions)]
use core::cell::Cell;

use crate::tick::TickDomain;

/// Debug-only reentrancy guard.
///
/// In debug builds, this tracks which tick domain is currently executing
/// and panics if a second domain attempts to enter concurrently.
///
/// In release builds, this compiles to a zero-size type with no-op methods.
///
/// # Single-Writer Contract
///
/// This guard does **not** provide thread safety. It assumes the single-writer
/// contract is enforced externally (via critical sections, priority masking,
/// or cooperative scheduling). The guard only *detects* violations during
/// development.
#[derive(Debug)]
pub struct DebugReentrancyGuard {
    #[cfg(debug_assertions)]
    in_use: Cell<Option<TickDomain>>,
}

impl DebugReentrancyGuard {
    /// Create a new guard in the "not in use" state.
    #[inline]
    pub const fn new() -> Self {
        Self {
            #[cfg(debug_assertions)]
            in_use: Cell::new(None),
        }
    }

    /// Mark entry for a domain.
    ///
    /// # Panics (debug only)
    ///
    /// Panics if another domain is already in progress, indicating a
    /// reentrancy bug.
    #[inline]
    pub fn enter(&self, domain: TickDomain) {
        #[cfg(debug_assertions)]
        {
            let current = self.in_use.get();
            debug_assert!(
                current.is_none(),
                "Reentrancy detected: already in {:?}, tried to enter {:?}",
                current.unwrap(),
                domain
            );
            self.in_use.set(Some(domain));
        }
        // Silence unused warning in release builds.
        let _ = domain;
    }

    /// Mark exit from the current domain.
    #[inline]
    pub fn exit(&self) {
        #[cfg(debug_assertions)]
        self.in_use.set(None);
    }

    /// Check if currently inside a tick (debug only).
    ///
    /// Returns `None` in release builds.
    #[inline]
    pub fn current_domain(&self) -> Option<TickDomain> {
        #[cfg(debug_assertions)]
        {
            self.in_use.get()
        }
        #[cfg(not(debug_assertions))]
        {
            None
        }
    }
}

impl Default for DebugReentrancyGuard {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}
