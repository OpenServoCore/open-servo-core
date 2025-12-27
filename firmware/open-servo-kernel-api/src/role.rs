//! Node roles (declarative metadata).
//!
//! Roles are *review tools*, not scheduling tools.
//! Scheduling remains by TickDomain.
//!
//! The goals:
//! - Make intent obvious from the type (and file name).
//! - Enable cheap debug assertions that a node runs in reasonable domains.
//! - Keep architecture legible as the graph grows.
//!
//! Naming guidance (recommended):
//! - state_*        => Role::State
//! - estimator_*    => Role::Estimator
//! - filter_*       => Role::Filter
//! - control_*      => Role::Control
//! - limiter_*      => Role::Limiter
//! - monitor_*      => Role::Monitor
//! - model_*        => Role::Model
//! - actuation_*    => Role::Actuation
//! - service_*      => Role::Service
//! - persist_*      => Role::Persistence

use crate::TickDomain;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Role {
    /// Moves raw inputs into kernel state, tracks setpoints, mode requests, etc.
    /// Also includes cross-domain "bridge" accumulators (fast->medium).
    State = 0,

    /// Signal conditioning (LPF, moving avg, debouncer, differentiator).
    Filter = 1,

    /// State derivation / observers / tracking that integrate over time.
    /// (Pure plant models belong in `open-servo-math`.)
    Estimator = 2,

    /// Control laws (PID, cascaded loops, feedforward).
    Control = 3,

    /// Constraints/budgets/gating/clamps (may shape commands, not necessarily faults).
    Limiter = 4,

    /// Health checks + fault detection + watchdogs (raise faults).
    /// Monitor nodes should be *raise-only* and never clear faults directly.
    Monitor = 5,

    /// Physics/plant/thermal models (time integration, energy estimates, etc).
    Model = 6,

    /// The final conversion into actuation outputs (e.g., effort -> MotorCommand).
    /// Prefer a single Actuation node per domain (especially fast).
    Actuation = 7,

    /// Diagnostics/telemetry/background tasks (non-control, non-safety).
    Service = 8,

    /// Persistence policy / commit state machines (EEPROM/flash orchestration).
    /// Often better kept out of the realtime graph; but role exists if needed.
    Persistence = 9,
}

/// Declarative role metadata for a node type.
///
/// Implement this for node types to force explicit thinking about intent.
/// This does not affect scheduling and has zero runtime cost.
pub trait HasRole {
    const ROLE: Role;
}

impl Role {
    /// Conservative defaults for where roles "make sense".
    /// This is intentionally not perfect; it's a debug/review guardrail.
    #[inline]
    pub const fn debug_allowed_domains(self) -> &'static [TickDomain] {
        match self {
            // State updates can occur anywhere, but cross-domain bridges usually happen fast/medium.
            Role::State => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            Role::Filter => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            Role::Estimator => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            Role::Control => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
            ],

            // Limiters are used a lot in control domains; allow System if you want supervisory limiters,
            // but default to control domains to keep actuators honest.
            Role::Limiter => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
            ],

            // Monitors are usually fast/medium/slow; allow System for thermal + sanity checks.
            Role::Monitor => &[
                TickDomain::ControlFast,
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            // Models often run medium/slow/system. Allow fast if needed, but prefer not.
            Role::Model => &[
                TickDomain::ControlMedium,
                TickDomain::ControlSlow,
                TickDomain::System,
            ],

            // Actuation is normally fast (and sometimes medium in certain architectures).
            Role::Actuation => &[TickDomain::ControlFast, TickDomain::ControlMedium],

            // Housekeeping & telemetry: primarily System; allow Slow if you treat slow as supervisory.
            Role::Service => &[TickDomain::ControlSlow, TickDomain::System],

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
