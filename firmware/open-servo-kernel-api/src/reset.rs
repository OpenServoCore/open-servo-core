//! Lifecycle vocabulary.
//!
//! This module defines:
//! - [`ResetReason`]: why the kernel is requesting a reset
//! - [`Resettable`]: lifecycle hook for nodes/features with internal state
//! - [`impl_reset_nop!`]: convenience macro for stateless nodes
//!
//! Reset propagation is part of kernel behavior, but the *meaning* of “reset” is
//! component-specific (controllers, observers, supervisors, filters, etc.).
//!
//! ## Design choice: `Resettable` is required for scheduled nodes
//!
//! This project leans toward **explicitness over implicit defaults**.
//!
//! While many nodes *could* be stateless, in practice most control-adjacent nodes
//! carry some hidden state (integrators, trackers, debouncers, windows, FSMs,
//! latch flags). Missing a reset at a mode transition or fault clear is a classic
//! source of “works most of the time” bugs.
//!
//! Therefore, we intentionally keep `Resettable` as a small, explicit contract
//! with **no default implementation**. If a type truly has nothing to reset,
//! it should still implement `Resettable` with an explicit empty body.
//!
//! This mirrors the philosophy behind [`crate::role::HasRole`]: force the author
//! to make an intentional choice.
//!
//! ## Telemetry
//!
//! Kernels expose reset events via telemetry for debugging and system-id:
//! - `RESET_LAST_REASON`: most recent [`ResetReason`] discriminant
//! - `RESET_SEQ`: monotonic counter (increments on each reset propagation)
//! - `RESET_COUNT_*`: per-reason counters (histograms)
//!
//! Unlike faults, resets are point-in-time events (not latched state), so we use
//! counters rather than a bitmask.
//!
//! See [`crate::telemetry::ids`] for suggested IDs.

/// Reason the kernel is requesting a reset.
///
/// This is useful for:
/// - telemetry (“why did my integrator clear?”)
/// - debugging (“why did we drop out of hold?”)
/// - implementing different reset policies depending on the cause
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ResetReason {
    /// User or host requested disengage/torque-off.
    Disengage,

    /// User or host requested engage/torque-on.
    ///
    /// Useful for initializing “first tick after engage” behaviors.
    Engage,

    /// A fault was raised (or safety gate asserted).
    FaultRaised,

    /// A fault was cleared (often implies reinitializing supervisors/estimators).
    FaultCleared,

    /// Operating mode changed (e.g., position -> velocity mode).
    ModeChanged,

    /// Configuration changed in a way that requires reinitialization.
    ConfigChanged,
}

impl ResetReason {
    /// Discriminant as `u32` (useful for telemetry).
    #[inline]
    pub const fn as_u32(self) -> u32 {
        self as u32
    }
}

/// Reset/lifecycle hook for nodes/features/controllers that have internal state.
///
/// Intended usage:
/// - Kernel calls `reset()` on mode transitions, engage/disengage, and fault edges.
/// - Components clear integrators, window accumulators, trackers, FSM state, etc.
///
/// This trait intentionally has **no default no-op** implementation to force
/// authors to consider reset behavior. If a type truly has nothing to reset,
/// implement an explicit empty reset body.
///
/// ```rust
/// use open_servo_kernel_api::reset::{ResetReason, Resettable};
///
/// struct Constant;
///
/// impl Resettable for Constant {
///     #[inline]
///     fn reset(&mut self, _reason: ResetReason) {
///         // Intentionally no state to reset.
///     }
/// }
/// ```
pub trait Resettable {
    fn reset(&mut self, reason: ResetReason);
}

/// Convenience macro for an explicit no-op reset impl.
///
/// This still forces the author to make an intentional choice (add one line).
#[macro_export]
macro_rules! impl_reset_nop {
    ($ty:ty) => {
        impl $crate::reset::Resettable for $ty {
            #[inline]
            fn reset(&mut self, _reason: $crate::reset::ResetReason) {}
        }
    };
}
