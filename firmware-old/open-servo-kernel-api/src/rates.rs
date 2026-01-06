//! Domain rate declarations.
//!
//! Tick domains are semantic concepts; the board decides how to actualize them.
//! Still, it's useful to **declare** chosen rates for:
//! - diagnostics (sanity checks, tuning expectations)
//! - telemetry metadata (system identification)
//! - tooling (plotters can infer expected sample rates)
//!
//! ```rust,ignore
//! use open_servo_kernel_api::DomainRatesHz;
//!
//! // Board declares its chosen scheduling rates (informational, not a mandate).
//! let rates = DomainRatesHz::from_u32(10_000, 1_000, 200, 100);
//! //           fast=10kHz  medium=1kHz  slow=200Hz  system=100Hz
//! ```

use crate::units::Hertz;

/// Declared scheduling rates for each semantic domain.
///
/// This is informational (diagnostics/telemetry), not a scheduling mandate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct DomainRatesHz {
    pub fast: Hertz,
    pub medium: Hertz,
    pub slow: Hertz,
    pub system: Hertz,
}

impl DomainRatesHz {
    /// Construct from typed Hertz values.
    #[inline]
    pub const fn new(fast: Hertz, medium: Hertz, slow: Hertz, system: Hertz) -> Self {
        Self {
            fast,
            medium,
            slow,
            system,
        }
    }

    /// Construct from raw integer Hz values (ergonomic at call sites).
    #[inline]
    pub const fn from_u32(fast_hz: u32, medium_hz: u32, slow_hz: u32, system_hz: u32) -> Self {
        Self {
            fast: Hertz(fast_hz),
            medium: Hertz(medium_hz),
            slow: Hertz(slow_hz),
            system: Hertz(system_hz),
        }
    }
}
