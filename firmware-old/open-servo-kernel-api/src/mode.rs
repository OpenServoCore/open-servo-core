//! Operating mode vocabulary.
//!
//! This is intentionally minimal so you can start shipping while the architecture
//! is still settling.
//!
//! Today you can support:
//! - `Position`: PID position control (setpoint is a position)
//! - `OpenLoop`: direct output control (setpoint is duty/effort; kernel-defined)
//!
//! Later you can add `Velocity`, `Torque`, etc., or map to Dynamixel modes.
//!
//! Mode switching *policy* (what is allowed when faulted/engaged/etc.) belongs
//! in the kernel crate, not here.
//!
//! Recommended: implement mode switching by selecting different graphs per mode.”

/// High-level operating mode.
///
/// Keep this stable and expand carefully.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OperatingMode {
    /// PID position control.
    Position = 0,
    /// Open-loop control (direct duty/effort mapping).
    OpenLoop = 1,
}

/// A request to change operating mode.
///
/// This is the “wire/kernel boundary” shape. The kernel decides whether
/// the request is accepted immediately, deferred, or rejected.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ModeRequest {
    pub mode: OperatingMode,
}

impl ModeRequest {
    #[inline]
    pub const fn new(mode: OperatingMode) -> Self {
        Self { mode }
    }
}

/// Mode-related errors.
///
/// Keep it minimal; refine once you implement real mode switching.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ModeError {
    /// The requested mode is not supported by this build/board.
    Unsupported,
    /// Mode change cannot be performed right now (e.g., EEPROM write in progress).
    Busy,
    /// Mode change refused due to a faulted state (policy decided by kernel).
    Faulted,
}
