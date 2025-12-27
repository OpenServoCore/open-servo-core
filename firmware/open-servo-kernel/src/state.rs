//! Kernel-owned state.
//!
//! This is the "shared state struct" the kernel uses internally.
//! It is *not* a public regmap yet; it's just a clean container so you can
//! evolve storage without spreading fields everywhere.

use open_servo_hw::v2::io::{MotorCommand, SensorFrame};
use open_servo_kernel_api::faults::GateReason;
use open_servo_kernel_api::mode::OperatingMode;
use open_servo_kernel_api::reset::ResetScope;
use open_servo_units::{CentiDeg32, Effort, MicroSecond};

/// PID gains (Stage-0).
///
/// You’ll likely replace this with your controller crate types later.
/// Keep it tiny and explicit for now.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct PidGains {
    /// Proportional gain in Q?? (project-defined).
    pub kp: i16,
    /// Integral gain in Q?? (project-defined).
    pub ki: i16,
    /// Derivative gain in Q?? (project-defined).
    pub kd: i16,
}

/// Kernel configuration (tunable).
///
/// This is "knobs"; you can later back it with regmap/EEPROM.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct KernelConfig {
    pub pos_pid: PidGains,

    /// Position deadband for hold/coast decisions (centideg32).
    pub hold_deadband_cdeg: i32,

    /// Output clamp in normalized effort (abs).
    /// (Stage-0: simple clamp; later: compliance/limits/current/thermal budgets.)
    pub effort_limit_raw: i16,
}

/// Pending host operations (deferred to safe boundary).
///
/// When a host operation needs to affect state that shouldn't be modified
/// mid-tick (e.g., controller reset), we record the request here and apply
/// it on a safe boundary (typically System or Slow tick).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct PendingOps {
    /// Pending soft reset request.
    pub reset_req: Option<ResetScope>,
    // Future: persist_req: bool, etc.
}

/// Runtime state (mutable).
///
/// Keep this “boring and obvious” — it becomes your foundation.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct KernelState {
    // ===== Host-visible / conceptual state =====
    pub engaged: bool,
    pub mode: OperatingMode,

    // ===== Commands/setpoints =====
    pub pos_sp: CentiDeg32,
    pub open_loop_effort: Effort,

    // ===== Latest sensor frame (board cadence) =====
    pub frame: SensorFrame,

    // ===== Derived signals (kernel-defined) =====
    pub pos: CentiDeg32,

    /// Latest gated output (useful for telemetry / debug).
    pub last_cmd: MotorCommand,
    pub last_gate: GateReason,

    // ===== Simple internal integrators (Stage-0 PID) =====
    pub pos_i: i32,
    pub last_pos_err: i32,

    /// Optional: track last dt for sanity checks.
    pub last_dt: MicroSecond,
}

impl Default for KernelState {
    fn default() -> Self {
        Self {
            engaged: false,
            mode: OperatingMode::Position,
            pos_sp: CentiDeg32::from_cdeg(0),
            open_loop_effort: Effort::ZERO,
            frame: SensorFrame::default(),
            pos: CentiDeg32(0),
            last_cmd: MotorCommand::safe(),
            last_gate: GateReason::Disengaged,
            pos_i: 0,
            last_pos_err: 0,
            last_dt: MicroSecond::from_us(0),
        }
    }
}

impl KernelState {
    #[inline]
    pub fn update_frame(&mut self, frame: SensorFrame) {
        self.frame = frame;
        // Stage-0: just lift pos directly.
        self.pos = frame.pos;
    }
}
