//! Kernel-owned state.
//!
//! This is the "shared state struct" the kernel uses internally.
//! It is *not* a public regmap yet; it's just a clean container so you can
//! evolve storage without spreading fields everywhere.

use open_servo_hw::v2::capability::{MotorType, SensorCapabilities, ServoPosKind};
use open_servo_hw::v2::io::{MotorCommand, SensorFrame};
use open_servo_kernel_api::faults::GateReason;
use open_servo_kernel_api::mode::OperatingMode;
use open_servo_kernel_api::reset::ResetScope;
use open_servo_math::pid::{DerivativeMode, PidControllerI16};
use open_servo_units::{CentiDeg32, Effort, MicroSecond};

/// PID gains in Q8.8 fixed-point format.
///
/// Gain = raw_value / 256. E.g., kp=1280 means 5.0 proportional gain.
/// Use `open_servo_math::gain::Gain::from_f32(5.0).to_q8_8()` to convert.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct PidGains {
    /// Proportional gain in Q8.8 (raw / 256).
    pub kp: i16,
    /// Integral gain in Q8.8 (raw / 256).
    pub ki: i16,
    /// Derivative gain in Q8.8 (raw / 256).
    pub kd: i16,
}

/// Kernel configuration (tunable).
///
/// This is "knobs"; you can later back it with regmap/EEPROM.
/// Board crates must construct this explicitly - no Default impl.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct KernelConfig {
    pub pos_pid: PidGains,

    /// Position deadband for hold/coast decisions (centideg32).
    pub hold_deadband_cdeg: i32,

    /// Output clamp in normalized effort (abs).
    /// (Stage-0: simple clamp; later: compliance/limits/current/thermal budgets.)
    pub effort_limit_raw: i16,

    // === Board capabilities (set once at init) ===
    /// Servo position sensor semantics for this board.
    pub servo_pos_kind: ServoPosKind,

    /// Motor topology for this board (BDC or BLDC).
    pub motor_type: MotorType,

    /// Runtime capability flags for optional sensors.
    pub sensor_caps: SensorCapabilities,
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
/// Keep this "boring and obvious" — it becomes your foundation.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct KernelState {
    // ===== Host-visible / conceptual state =====
    pub engaged: bool,
    pub mode: OperatingMode,
    pub fault_mask: u32,

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

    // ===== Position PID controller =====
    pub pos_pid: PidControllerI16,

    /// Optional: track last dt for sanity checks.
    pub last_dt: MicroSecond,
}

impl KernelState {
    /// Create state with specified PID gains and effort limits.
    pub fn new(gains: PidGains, effort_limit: i16) -> Self {
        let lim = effort_limit.abs() as i32;
        Self {
            engaged: false,
            mode: OperatingMode::Position,
            fault_mask: 0,
            pos_sp: CentiDeg32::from_cdeg(0),
            open_loop_effort: Effort::ZERO,
            frame: SensorFrame::default(),
            pos: CentiDeg32(0),
            last_cmd: MotorCommand::safe(),
            last_gate: GateReason::Disengaged,
            pos_pid: PidControllerI16::new_auto_anti_windup(
                gains.kp,
                gains.ki,
                gains.kd,
                DerivativeMode::OnMeasurement, // Avoids derivative kick on setpoint steps
                -lim,
                lim,
            ),
            last_dt: MicroSecond::from_us(0),
        }
    }
}

impl KernelState {
    #[inline]
    pub fn update_frame(&mut self, frame: SensorFrame) {
        self.frame = frame;
        // Extract position from Reading (keep last known good if invalid).
        if let Some(pos_val) = frame.pos.value() {
            self.pos = pos_val;
        }
    }
}
