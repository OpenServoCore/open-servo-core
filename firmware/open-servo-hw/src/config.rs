//! Board configuration trait for providing board-specific parameters.
//!
//! This trait eliminates the need for generic Default implementations,
//! ensuring all configurations are explicitly provided by the board.

use open_servo_math::ComplianceConfig;

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
    /// Electrical resistance in milliohms (5000 = 5.0Ω)
    pub resistance_mohm: i16,
    /// Thermal resistance in centi-°C/W (1000 = 10°C/W)
    pub thermal_resistance_cw: i16,
    /// Thermal capacity in centi-J/°C (1500 = 15 J/°C)
    pub thermal_capacity_cj: i16,
}

/// Board-specific kinematics configuration.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoardKinematicsConfig {
    /// Minimum raw sensor value
    pub sensor_raw_min: u16,
    /// Maximum raw sensor value  
    pub sensor_raw_max: u16,
    /// Minimum physical position in centidegrees
    pub mechanical_min_cdeg: i32,
    /// Maximum physical position in centidegrees
    pub mechanical_max_cdeg: i32,
    /// Calibration zero offset in centidegrees
    pub zero_offset_cdeg: i32,
    /// Motor rotation reversed
    pub reversed: bool,
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

    /// Get PID gains as (kp, ki, kd) in raw i32 format.
    /// These will be converted to Gain types by the controller.
    fn pid_gains(&self) -> (i32, i32, i32);
}
