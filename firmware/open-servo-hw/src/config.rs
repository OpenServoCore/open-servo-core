//! Board configuration trait for providing board-specific parameters.
//!
//! This trait eliminates the need for generic Default implementations,
//! ensuring all configurations are explicitly provided by the board.

use open_servo_math::{CentiDeg, ComplianceConfig, DegPerSec10, Duty};

/// Board-specific safety threshold configuration.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardSafetyConfig {
    /// Maximum allowed motor current (absolute value compared)
    pub current_limit_ma: i16,
    /// Maximum allowed MCU temperature
    pub mcu_temp_limit_cc: i16,
    /// Maximum allowed position change per control tick
    pub position_max_delta_cdeg: i16,
    /// Consecutive bad sensor reads before hard fault
    pub sensor_fault_count: u8,
    /// Minimum allowed position (setpoint clamped to this)
    pub position_min_cdeg: i16,
    /// Maximum allowed position (setpoint clamped to this)
    pub position_max_cdeg: i16,
    /// Stall detection timeout in ticks
    pub stall_timeout_ticks: u16,
    /// Position change tolerance for stall detection
    pub stall_position_tolerance_cdeg: i16,
    /// Maximum allowed position error before fault
    pub position_error_limit_cdeg: i16,
    /// Position error timeout in microseconds
    pub position_error_timeout_us: u32,
}

/// Board-specific thermal model configuration.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardThermalConfig {
    // --- Thermal model parameters (motor winding estimation) ---
    /// Electrical resistance in milliohms (5000 = 5.0Ω)
    pub resistance_mohm: i16,
    /// Thermal resistance in centi-°C/W (1000 = 10°C/W)
    pub thermal_resistance_cw: i16,
    /// Thermal capacity in centi-J/°C (1500 = 15 J/°C)
    pub thermal_capacity_cj: i16,

    // --- Thermal fault detection parameters ---
    /// Maximum motor temperature before fault (centidegrees, 10000 = 100°C)
    pub max_temp_cdeg: i16,
    /// Hysteresis for fault reset (centidegrees, 1000 = 10°C)
    pub hysteresis_cdeg: i16,
    /// Default ambient temperature when no sensor (centidegrees, 2500 = 25°C)
    pub default_ambient_cdeg: i16,
}

/// Board-specific kinematics configuration.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardKinematicsConfig {
    /// Minimum raw sensor value
    pub sensor_raw_min: u16,
    /// Maximum raw sensor value
    pub sensor_raw_max: u16,
    /// Sensor position range minimum in centidegrees (typically 0)
    pub sensor_min_cdeg: i32,
    /// Sensor position range maximum in centidegrees (typically 36000 = 360°)
    pub sensor_max_cdeg: i32,
    /// Minimum physical/mechanical position in centidegrees
    pub mechanical_min_cdeg: i32,
    /// Maximum physical/mechanical position in centidegrees
    pub mechanical_max_cdeg: i32,
    /// Calibration zero offset in centidegrees
    pub zero_offset_cdeg: i32,
    /// Motor rotation reversed
    pub reversed: bool,
}

/// Board-specific policy configuration for Move/Hold/Yield FSM.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardPolicyConfig {
    // --- Hold mode entry/exit thresholds ---
    /// Position error threshold to enter Hold mode
    pub hold_enter_error: CentiDeg,
    /// Position error threshold to exit Hold mode
    pub hold_exit_error: CentiDeg,
    /// Velocity threshold to enter Hold mode
    pub hold_enter_vel: DegPerSec10,
    /// Velocity threshold to exit Hold mode
    pub hold_exit_vel: DegPerSec10,

    // --- Backdrive/Yield detection ---
    /// Velocity threshold for backdrive detection
    pub backdrive_vel_threshold: DegPerSec10,
    /// PWM deadband for backdrive detection
    pub backdrive_deadband: Duty,
    /// Backdrive persistence time before yield (microseconds)
    pub backdrive_persist_us: u32,
    /// Maximum duty during yield alive phase
    pub yield_alive_duty_max: Duty,
    /// Coast phase duration within yield window (microseconds)
    pub yield_coast_us: u32,
    /// Total yield window duration (microseconds)
    pub yield_duration_us: u32,

    // --- Hold mode duty curve ---
    /// Error at which hold duty curve starts ramping
    pub hold_duty_error_start: CentiDeg,
    /// Error at which hold duty curve reaches max
    pub hold_duty_error_end: CentiDeg,
    /// Minimum hold duty at small errors
    pub hold_duty_min: Duty,
    /// Maximum hold duty at large errors
    pub hold_duty_max: Duty,
}

/// Board configuration provider trait.
///
/// Each board implementation must provide all configuration values
/// explicitly, eliminating reliance on generic defaults.
pub trait BoardConfig {
    /// Get safety configuration.
    fn safety_config(&self) -> BoardSafetyConfig;

    /// Get compliance configuration for move mode.
    fn move_compliance_config(&self) -> ComplianceConfig;

    /// Get compliance configuration for hold mode.
    fn hold_compliance_config(&self) -> ComplianceConfig;

    /// Get thermal model configuration.
    fn thermal_config(&self) -> BoardThermalConfig;

    /// Get kinematics configuration.
    fn kinematics_config(&self) -> BoardKinematicsConfig;

    /// Get policy configuration for FSM behavior.
    fn policy_config(&self) -> BoardPolicyConfig;

    /// Get PID gains as (kp, ki, kd) in raw i32 format.
    /// These will be converted to Gain types by the controller.
    fn pid_gains(&self) -> (i32, i32, i32);
}
