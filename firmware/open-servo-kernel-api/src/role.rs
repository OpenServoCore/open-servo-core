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
    /// Control law (PID, cascaded loops, feedforward).
    Control = 0,
    /// Signal conditioning (LPF, moving avg, debouncer, differentiator).
    Filter = 1,
    /// Physical/plant models and estimators (thermal, motor, observers).
    Model = 2,
    /// Health monitoring / protection checks (fault detection, watchdog).
    Monitor = 3,
    /// Constraints and gating (clamps, budgets, soft limits, torque limiting).
    Limiter = 4,
    /// Configuration/state container / register-facing state / persistence policy.
    State = 5,
    /// Housekeeping / logging / telemetry / background tasks.
    System = 6,
    /// Protocol-facing logic (optional; often lives outside nodes).
    Protocol = 7,
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
    pub fn debug_expected_domains(self) -> &'static [TickDomain] {
        match self {
            Role::Control => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
            ],
            Role::Filter => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],
            Role::Model => &[
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],
            Role::Monitor => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::System,
            ],
            Role::Limiter => &[TickDomain::ControlFast, TickDomain::ControlMedium],
            Role::State => &[TickDomain::ControlMedium, TickDomain::System],
            Role::System => &[TickDomain::System],
            Role::Protocol => &[TickDomain::System],
        }
    }
}

#[inline]
pub fn debug_assert_role_domain<N: HasRole>(domain: TickDomain) {
    #[cfg(debug_assertions)]
    {
        let ok = N::ROLE
            .debug_expected_domains()
            .iter()
            .any(|d| *d == domain);

        debug_assert!(
            ok,
            "Role/domain mismatch: role={:?} domain={:?}",
            N::ROLE,
            domain
        );
    }
}
