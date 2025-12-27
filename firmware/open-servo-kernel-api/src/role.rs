//! Node roles (declarative metadata).
//!
//! Roles are intentionally named like traditional embedded C/C++ modules/classes:
//! control / filter / model / monitor / limiter / state / system.
//!
//! The kernel does **not** schedule by role; scheduling is still by tick domain.
//! Roles exist to force clarity in review and keep the architecture legible.
//!
//! ```rust,ignore
//! use open_servo_kernel_api::role::{HasRole, Role};
//!
//! pub struct ThermalModel;
//! impl HasRole for ThermalModel {
//!     const ROLE: Role = Role::Model;
//! }
//! ```

use crate::TickDomain;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Role {
    /// Control laws (PID, cascaded loops, feedforward).
    Control = 0,

    /// Signal conditioning (LPF, moving avg, debouncer, differentiator).
    Filter = 1,

    /// State derivation / observers / tracking that integrate over time.
    /// (Pure plant models belong in `open-servo-math`.)
    Estimator = 2,

    /// Health checks + fault detection + watchdogs (raises faults).
    Guard = 3,

    /// Constraints/budgets/gating/clamps (may shape commands, not necessarily faults).
    Limiter = 4,

    /// Diagnostics/telemetry/background tasks (non-control, non-safety).
    Service = 5,

    /// (Optional) Persistence policy / commit state machines (EEPROM/flash).
    /// Consider keeping this out of the node graph entirely.
    Persistence = 6,
}

/// Declarative role metadata for a node type.
///
/// Implement this for node types to force explicit thinking about intent.
/// This does not affect scheduling and has zero runtime cost.
pub trait HasRole {
    const ROLE: Role;
}

impl Role {
    #[inline]
    pub const fn debug_allowed_domains(self) -> &'static [TickDomain] {
        match self {
            Role::Control => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
            ],

            // Filters are everywhere (including debouncers in System).
            Role::Filter => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            // Estimators can be fast (vel estimation), but commonly medium/slow/system.
            Role::Estimator => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            // Guards are often fast/medium, but some are slow/system (thermal, sanity checks).
            Role::Guard => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            // Limiters show up everywhere: setpoint clamps (slow/medium), effort clamps (fast),
            // gating decisions (fast/medium).
            Role::Limiter => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
            ],

            // Housekeeping & telemetry: primarily System; allow Slow if you treat slow as supervisory.
            Role::Service => &[TickDomain::ControlSlow, TickDomain::System],

            // Optional: persistence policy should be system-only.
            Role::Persistence => &[TickDomain::System],
        }
    }
}

#[inline]
pub fn debug_assert_role_domain<N: HasRole>(domain: TickDomain) {
    #[cfg(debug_assertions)]
    {
        let ok = N::ROLE.debug_allowed_domains().iter().any(|d| *d == domain);

        debug_assert!(
            ok,
            "Role/domain mismatch: role={:?} domain={:?}",
            N::ROLE,
            domain
        );
    }
}
