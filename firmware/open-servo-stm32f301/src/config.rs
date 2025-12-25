//! Board configuration constants and structures for STM32F301 with SG90 servo.

use open_servo_hw::config::{BoardKinematicsConfig, BoardSafetyConfig, BoardThermalConfig};
use open_servo_math::ComplianceConfig;

/// Slow tick frequency in Hz
pub const SLOW_HZ: u32 = 100;

// Tick timing constants (authoritative for this board)
pub const CONTROL_FAST_HZ: u32 = 10_000;
pub const CONTROL_FAST_DT_US: u32 = 1_000_000 / CONTROL_FAST_HZ; // 100µs

// TODO: CONTROL_MEDIUM_DECIMATE should be derived from timing config later
pub const CONTROL_MEDIUM_DECIMATE: u8 = 10;
pub const CONTROL_MEDIUM_DT_US: u32 = CONTROL_FAST_DT_US * (CONTROL_MEDIUM_DECIMATE as u32); // 1000µs

pub const SYSTEM_HZ: u32 = SLOW_HZ; // TIM2 "system tick" today
pub const SYSTEM_DT_US: u32 = 1_000_000 / SYSTEM_HZ; // 10,000µs

/// Board-specific configuration provider
pub struct BoardConfigProvider;

// Safety configuration constants
impl BoardConfigProvider {
    /// Over-current threshold: 1200 mA
    /// Provides headroom for normal operation while still protecting
    /// against genuine overcurrent conditions before DRV8231A hardware limiting (~1.5A)
    pub const CURRENT_LIMIT_MA: i16 = 1200;

    /// MCU over-temperature threshold: 8000 centiC (80.0°C)
    /// STM32F301 max junction temp is 105°C, leave margin
    pub const MCU_TEMP_LIMIT_CC: i16 = 8000;

    /// Max position change per tick: 500 centidegrees (5.0°)
    /// At 10kHz, this allows ~50,000 deg/sec motion
    pub const POS_MAX_DELTA_CDEG: i16 = 500;

    /// Consecutive bad sensor reads before hard fault: 10
    /// At 10kHz, this is ~1ms of bad readings
    pub const SENSOR_FAULT_COUNT: u8 = 10;

    /// Minimum position: 0 centidegrees (0°)
    /// Mechanical hard limit is -5°
    pub const POSITION_MIN_CDEG: i16 = 0;

    /// Maximum position: 18000 centidegrees (180°)
    /// Mechanical hard limit is 185°
    pub const POSITION_MAX_CDEG: i16 = 18000;

    /// Stall detection timeout: 1000 ticks (100ms at 10kHz)
    pub const STALL_TIMEOUT_TICKS: u16 = 1000;

    /// Stall position tolerance: 10 centidegrees (0.1°)
    /// Position must change by more than this to not be considered stalled
    pub const STALL_POSITION_TOLERANCE_CDEG: i16 = 10;

    /// Position error limit: 3000 centidegrees (30°)
    /// Fault if |setpoint - position| exceeds this for too long
    pub const POSITION_ERROR_LIMIT_CDEG: i16 = 3000;

    /// Position error timeout: 50 ticks (500ms at 100Hz slow loop)
    pub const POSITION_ERROR_TIMEOUT_TICKS: u16 = 50;
}

// Compliance configuration constants
impl BoardConfigProvider {
    /// Move mode current limit: 800mA
    pub const MOVE_CURRENT_LIMIT_MA: i16 = 800;

    /// Hold mode current limit: 150mA
    pub const HOLD_CURRENT_LIMIT_MA: i16 = 150;

    /// Current hysteresis band
    pub const CURRENT_HYSTERESIS_MA: i16 = 50;
    pub const CURRENT_HYSTERESIS_HOLD_MA: i16 = 25;

    /// Deglitch samples before triggering compliance
    pub const COMPLIANCE_DEGLITCH_SAMPLES: u8 = 3;

    /// Backoff factor in Q8 format (230 = 0.9)
    pub const COMPLIANCE_BACKOFF_FACTOR_Q8: u16 = 230;

    /// Recovery rate: 10% per second (3277 duty units)
    pub const COMPLIANCE_RECOVERY_RATE: i16 = 3277;
}

// Thermal model constants for SG90 servo
impl BoardConfigProvider {
    /// Motor winding resistance: 5.0Ω
    pub const MOTOR_RESISTANCE_MOHM: i16 = 5000;

    /// Thermal resistance: 10°C/W
    pub const THERMAL_RESISTANCE_CW: i16 = 1000;

    /// Heat capacity: 15 J/°C
    pub const THERMAL_CAPACITY_CJ: i16 = 1500;
}

// Kinematics constants
impl BoardConfigProvider {
    /// 12-bit ADC range
    pub const SENSOR_RAW_MIN: u16 = 0;
    pub const SENSOR_RAW_MAX: u16 = 4095;

    /// Mechanical limits in centidegrees
    pub const MECHANICAL_MIN_CDEG: i32 = -500; // -5°
    pub const MECHANICAL_MAX_CDEG: i32 = 18500; // 185°

    /// Zero calibration offset
    pub const ZERO_OFFSET_CDEG: i32 = 0;

    /// Motor direction
    pub const MOTOR_REVERSED: bool = false;
}

// PID gain constants (Q8.8 format)
impl BoardConfigProvider {
    /// Proportional gain: 5.0 -> 1280 in Q8.8
    pub const PID_KP_Q8: i16 = 1280;

    /// Integral gain: 0.0 -> 0 in Q8.8
    pub const PID_KI_Q8: i16 = 0;

    /// Derivative gain: 5.0 -> 1280 in Q8.8
    pub const PID_KD_Q8: i16 = 1280;
}

// Configuration factory methods
impl BoardConfigProvider {
    pub fn safety_config() -> BoardSafetyConfig {
        BoardSafetyConfig {
            current_limit_ma: Self::CURRENT_LIMIT_MA,
            mcu_temp_limit_cc: Self::MCU_TEMP_LIMIT_CC,
            position_max_delta_cdeg: Self::POS_MAX_DELTA_CDEG,
            sensor_fault_count: Self::SENSOR_FAULT_COUNT,
            position_min_cdeg: Self::POSITION_MIN_CDEG,
            position_max_cdeg: Self::POSITION_MAX_CDEG,
            stall_timeout_ticks: Self::STALL_TIMEOUT_TICKS,
            stall_position_tolerance_cdeg: Self::STALL_POSITION_TOLERANCE_CDEG,
            position_error_limit_cdeg: Self::POSITION_ERROR_LIMIT_CDEG,
            position_error_timeout_ticks: Self::POSITION_ERROR_TIMEOUT_TICKS,
        }
    }

    pub fn move_compliance_config() -> ComplianceConfig {
        ComplianceConfig::new(
            Self::MOVE_CURRENT_LIMIT_MA,
            Self::CURRENT_HYSTERESIS_MA,
            Self::COMPLIANCE_DEGLITCH_SAMPLES,
            Self::COMPLIANCE_BACKOFF_FACTOR_Q8,
            Self::COMPLIANCE_RECOVERY_RATE,
        )
    }

    pub fn hold_compliance_config() -> ComplianceConfig {
        ComplianceConfig::new(
            Self::HOLD_CURRENT_LIMIT_MA,
            Self::CURRENT_HYSTERESIS_HOLD_MA,
            Self::COMPLIANCE_DEGLITCH_SAMPLES,
            Self::COMPLIANCE_BACKOFF_FACTOR_Q8,
            Self::COMPLIANCE_RECOVERY_RATE,
        )
    }

    pub fn thermal_config() -> BoardThermalConfig {
        BoardThermalConfig {
            resistance_mohm: Self::MOTOR_RESISTANCE_MOHM,
            thermal_resistance_cw: Self::THERMAL_RESISTANCE_CW,
            thermal_capacity_cj: Self::THERMAL_CAPACITY_CJ,
        }
    }

    pub fn kinematics_config() -> BoardKinematicsConfig {
        BoardKinematicsConfig {
            sensor_raw_min: Self::SENSOR_RAW_MIN,
            sensor_raw_max: Self::SENSOR_RAW_MAX,
            mechanical_min_cdeg: Self::MECHANICAL_MIN_CDEG,
            mechanical_max_cdeg: Self::MECHANICAL_MAX_CDEG,
            zero_offset_cdeg: Self::ZERO_OFFSET_CDEG,
            reversed: Self::MOTOR_REVERSED,
        }
    }

    pub fn pid_gains() -> (i32, i32, i32) {
        (
            Self::PID_KP_Q8 as i32,
            Self::PID_KI_Q8 as i32,
            Self::PID_KD_Q8 as i32,
        )
    }
}
