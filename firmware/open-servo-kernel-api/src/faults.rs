//! Fault vocabulary and fault plumbing boundary.
//!
//! This module defines:
//! - [`FaultKind`]: stable fault identifiers (part of the public API surface)
//! - [`GateReason`]: high-level gating reasons (useful for telemetry/debugging)
//! - [`FaultSink`]: **raise-only** interface for nodes/features (write-only)
//! - [`FaultAdmin`]: kernel/host-only fault management (clear/ack policy)
//! - [`FaultLatch`]: a small reference implementation (bitmask + last + sequence)
//!
//! ## Design intent
//!
//! **Nodes/features may raise faults, but must not clear them.**
//!
//! Fault clearing is a policy decision owned by the kernel/host command path:
//! - some faults should require explicit operator acknowledgement
//! - some faults should only clear after conditions are safe again
//! - clearing often needs coordinated resets (controllers, estimators, etc.)
//!
//! Therefore the API is split:
//! - [`FaultSink`] is the only thing passed into `TickCtx` for nodes (raise-only)
//! - [`FaultAdmin`] is used by the kernel internally (clear/clear_all)
//!
//! ## Telemetry
//!
//! Kernels typically expose fault state via telemetry rather than giving nodes a
//! read-only fault view. Common pattern:
//! - `FAULT_MASK`: latched fault bitmask (`u32`)
//! - `FAULT_LAST_KIND`: most recent fault kind (discriminant) or 0
//! - `FAULT_SEQ`: monotonic sequence counter (increments on fault edge)
//!
//! See `telemetry::ids::*` for suggested IDs.
//!
//! # Invariant: Nodes Must Not Read Fault State
//!
//! Nodes receive only `&mut dyn FaultSink` via `TickCtx`. They can **raise** faults
//! but cannot:
//! - query whether a fault is latched (`is_faulted`, `has`, `mask`)
//! - clear faults ([`FaultAdmin`] methods)
//!
//! Fault-based gating (output disable, mode restrictions) is **kernel policy**.
//! Nodes should NOT branch on fault state; they should produce outputs normally
//! and let the kernel gate them.
//!
//! This separation ensures:
//! - Safety policy is centralized in the kernel
//! - Nodes remain testable in isolation
//! - No "self-clearing" foot-guns

/// Fault kinds are part of the API surface because they affect kernel gating
/// and externally-visible telemetry/tooling.
///
/// Keep this enum stable; add new faults carefully.
///
/// Encoding convention:
/// - `FaultKind` discriminant is used for telemetry (`kind as u32`)
/// - `FaultKind::mask()` defines the canonical bit position for `FAULT_MASK`
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FaultKind {
    EncoderFault = 0,
    OverCurrent = 1,
    Stall = 2,
    PositionError = 3,
    McuOverTemp = 4,
    MotorOverTemp = 5,
}

impl FaultKind {
    /// Canonical bitmask encoding for this fault kind.
    ///
    /// This is intended for `telemetry::ids::FAULT_MASK`.
    #[inline]
    pub const fn mask(self) -> u32 {
        1u32 << (self as u8)
    }

    /// Discriminant as `u32` (useful for telemetry).
    #[inline]
    pub const fn as_u32(self) -> u32 {
        self as u32
    }
}

/// Gate reasons are useful for telemetry and debugging.
///
/// This is intentionally higher-level than [`FaultKind`] and may include
/// conditions that are *not* latched faults (e.g. "Disengaged").
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GateReason {
    Ok,
    Disengaged,
    Faulted,
    DriverNotOk,
}

/// Raise-only fault sink passed into nodes/features.
///
/// Contract:
/// - `raise(kind)` may be called from any node/feature when a fault condition is detected.
/// - Returns `true` if the fault transitioned from **not latched** → **latched** (edge).
/// - Nodes **must not** clear faults. Clearing is kernel/host policy.
///
/// This keeps the safety policy centralized and avoids "self-clearing" foot-guns.
pub trait FaultSink {
    /// Raise (latch) a fault.
    ///
    /// Returns `true` if this call newly latched the fault (0→1 transition).
    fn raise(&mut self, kind: FaultKind) -> bool;
}

/// Kernel/host-only fault administration.
///
/// This interface is intentionally separate from [`FaultSink`] so it is not
/// accidentally provided to nodes via `TickCtx`.
///
/// Kernel policies vary:
/// - clear only after explicit operator acknowledgement
/// - clear only if conditions are safe again
/// - clear-all on reboot / mode switch / etc.
///
/// Return values mirror [`FaultSink::raise`]:
/// - `true` indicates a state change (edge)
pub trait FaultAdmin {
    /// Clear a specific fault.
    ///
    /// Returns `true` if the fault transitioned from latched → cleared (1→0).
    fn clear(&mut self, kind: FaultKind) -> bool;

    /// Clear all faults.
    ///
    /// Returns `true` if any fault was cleared.
    fn clear_all(&mut self) -> bool;
}

/// Small reference fault latch implementation.
///
/// Storage:
/// - `mask`: latched faults as a bitset (see [`FaultKind::mask`])
/// - `last`: most recently raised fault (for telemetry/UI)
/// - `seq`: monotonic counter incremented on *state changes* (raise edge / clear edge)
///
/// This is suitable for `no_std` kernels and is easy to serialize for telemetry.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug)]
pub struct FaultLatch {
    mask: u32,
    last: Option<FaultKind>,
    seq: u32,
}

impl Default for FaultLatch {
    fn default() -> Self {
        Self {
            mask: 0,
            last: None,
            seq: 0,
        }
    }
}

impl FaultLatch {
    /// True if any fault is currently latched.
    #[inline]
    pub const fn is_faulted(&self) -> bool {
        self.mask != 0
    }

    /// Current latched fault mask (`u32` bitset).
    #[inline]
    pub const fn mask(&self) -> u32 {
        self.mask
    }

    /// Most recently raised fault kind (if any).
    #[inline]
    pub const fn last(&self) -> Option<FaultKind> {
        self.last
    }

    /// Monotonic sequence counter (increments on raise/clear edges).
    ///
    /// Useful for telemetry consumers to detect “fault state changed” without
    /// diffing masks.
    #[inline]
    pub const fn seq(&self) -> u32 {
        self.seq
    }

    /// Convenience: check if a particular fault is latched.
    #[inline]
    pub const fn has(&self, kind: FaultKind) -> bool {
        (self.mask & kind.mask()) != 0
    }
}

impl FaultSink for FaultLatch {
    #[inline]
    fn raise(&mut self, kind: FaultKind) -> bool {
        let bit = kind.mask();
        let was_new = (self.mask & bit) == 0;

        // Latch it
        self.mask |= bit;

        // Track recency for UI/telemetry
        self.last = Some(kind);

        // Only bump seq on edge by default (avoid spamming)
        if was_new {
            self.seq = self.seq.wrapping_add(1);
        }

        was_new
    }
}

impl FaultAdmin for FaultLatch {
    #[inline]
    fn clear(&mut self, kind: FaultKind) -> bool {
        let bit = kind.mask();
        let was_set = (self.mask & bit) != 0;

        if was_set {
            self.mask &= !bit;
            self.seq = self.seq.wrapping_add(1);
        }

        was_set
    }

    #[inline]
    fn clear_all(&mut self) -> bool {
        let was_faulted = self.mask != 0;

        if was_faulted {
            self.mask = 0;
            self.seq = self.seq.wrapping_add(1);
        }

        was_faulted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raise_returns_edge() {
        let mut f = FaultLatch::default();
        assert!(!f.is_faulted());
        assert_eq!(f.seq(), 0);

        assert_eq!(f.raise(FaultKind::OverCurrent), true);
        assert!(f.is_faulted());
        assert_eq!(f.has(FaultKind::OverCurrent), true);
        assert_eq!(f.seq(), 1);

        // Raising again should not be an edge.
        assert_eq!(f.raise(FaultKind::OverCurrent), false);
        assert_eq!(f.seq(), 1);
    }

    #[test]
    fn clear_returns_edge() {
        let mut f = FaultLatch::default();
        f.raise(FaultKind::Stall);
        assert_eq!(f.seq(), 1);

        assert_eq!(FaultAdmin::clear(&mut f, FaultKind::Stall), true);
        assert_eq!(f.is_faulted(), false);
        assert_eq!(f.seq(), 2);

        // Clearing again should not be an edge.
        assert_eq!(FaultAdmin::clear(&mut f, FaultKind::Stall), false);
        assert_eq!(f.seq(), 2);
    }

    #[test]
    fn clear_all() {
        let mut f = FaultLatch::default();
        f.raise(FaultKind::Stall);
        f.raise(FaultKind::EncoderFault);
        assert!(f.is_faulted());

        assert_eq!(FaultAdmin::clear_all(&mut f), true);
        assert!(!f.is_faulted());

        // Already clear
        assert_eq!(FaultAdmin::clear_all(&mut f), false);
    }
}
