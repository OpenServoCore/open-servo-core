//! Board configuration constants and structures for STM32F301 with SG90 servo.

use open_servo_hw::config::{
    BoardKinematicsConfig, BoardPolicyConfig, BoardSafetyConfig, BoardThermalConfig,
};
use open_servo_math::{CentiDeg, ComplianceConfig, DegPerSec10, Duty};

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

    /// Position error timeout: 500ms
    /// Fault if large position error persists for this duration.
    /// (Fixes previous bug: was 50 ticks checked at 10kHz = 5ms, not 500ms)
    pub const POSITION_ERROR_TIMEOUT_US: u32 = 500_000;
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

    /// Maximum motor temperature before fault: 100°C
    pub const MAX_MOTOR_TEMP_CDEG: i16 = 10000;

    /// Temperature hysteresis for fault reset: 10°C
    pub const THERMAL_HYSTERESIS_CDEG: i16 = 1000;

    /// Default ambient temperature when no sensor: 25°C
    pub const DEFAULT_AMBIENT_CDEG: i16 = 2500;
}

// Kinematics constants
impl BoardConfigProvider {
    /// 12-bit ADC range
    pub const SENSOR_RAW_MIN: u16 = 0;
    pub const SENSOR_RAW_MAX: u16 = 4095;

    /// Sensor position range (full 360° potentiometer)
    pub const SENSOR_MIN_CDEG: i32 = 0;
    pub const SENSOR_MAX_CDEG: i32 = 36000;

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

// Policy/FSM constants for Move/Hold/Yield behavior
impl BoardConfigProvider {
    // --- Hold mode entry/exit thresholds ---
    /// Position error threshold to enter Hold mode: 5°
    pub const HOLD_ENTER_ERROR_CDEG: i16 = 500;
    /// Position error threshold to exit Hold mode: 7°
    pub const HOLD_EXIT_ERROR_CDEG: i16 = 700;
    /// Velocity threshold to enter Hold mode: 10°/s
    pub const HOLD_ENTER_VEL_DPS10: i16 = 100;
    /// Velocity threshold to exit Hold mode: 15°/s
    pub const HOLD_EXIT_VEL_DPS10: i16 = 150;

    // --- Backdrive/Yield detection ---
    /// Velocity threshold for backdrive detection: 30°/s
    pub const BACKDRIVE_VEL_THRESHOLD_DPS10: i16 = 300;
    /// PWM deadband for backdrive detection: 5%
    pub const BACKDRIVE_DEADBAND_DUTY: i16 = 1638;
    /// Backdrive persistence time before yield: 0.5ms
    pub const BACKDRIVE_PERSIST_US: u32 = 500;
    /// Maximum duty during yield alive phase: 5%
    pub const YIELD_ALIVE_DUTY_MAX: i16 = 1638;
    /// Coast phase duration within yield window: 100ms
    pub const YIELD_COAST_US: u32 = 100_000;
    /// Total yield window duration: 200ms
    pub const YIELD_DURATION_US: u32 = 200_000;

    // --- Hold mode duty curve ---
    /// Error at which hold duty curve starts ramping: 5°
    pub const HOLD_DUTY_ERROR_START_CDEG: i16 = 500;
    /// Error at which hold duty curve reaches max: 15°
    pub const HOLD_DUTY_ERROR_END_CDEG: i16 = 1500;
    /// Minimum hold duty at small errors: 20%
    pub const HOLD_DUTY_MIN: i16 = 6553;
    /// Maximum hold duty at large errors: 45%
    pub const HOLD_DUTY_MAX: i16 = 14746;
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
            position_error_timeout_us: Self::POSITION_ERROR_TIMEOUT_US,
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
            max_temp_cdeg: Self::MAX_MOTOR_TEMP_CDEG,
            hysteresis_cdeg: Self::THERMAL_HYSTERESIS_CDEG,
            default_ambient_cdeg: Self::DEFAULT_AMBIENT_CDEG,
        }
    }

    pub fn kinematics_config() -> BoardKinematicsConfig {
        BoardKinematicsConfig {
            sensor_raw_min: Self::SENSOR_RAW_MIN,
            sensor_raw_max: Self::SENSOR_RAW_MAX,
            sensor_min_cdeg: Self::SENSOR_MIN_CDEG,
            sensor_max_cdeg: Self::SENSOR_MAX_CDEG,
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

    pub fn policy_config() -> BoardPolicyConfig {
        BoardPolicyConfig {
            hold_enter_error: CentiDeg::from_cdeg(Self::HOLD_ENTER_ERROR_CDEG),
            hold_exit_error: CentiDeg::from_cdeg(Self::HOLD_EXIT_ERROR_CDEG),
            hold_enter_vel: DegPerSec10::from_dps10(Self::HOLD_ENTER_VEL_DPS10),
            hold_exit_vel: DegPerSec10::from_dps10(Self::HOLD_EXIT_VEL_DPS10),
            backdrive_vel_threshold: DegPerSec10::from_dps10(Self::BACKDRIVE_VEL_THRESHOLD_DPS10),
            backdrive_deadband: Duty::from_raw(Self::BACKDRIVE_DEADBAND_DUTY),
            backdrive_persist_us: Self::BACKDRIVE_PERSIST_US,
            yield_alive_duty_max: Duty::from_raw(Self::YIELD_ALIVE_DUTY_MAX),
            yield_coast_us: Self::YIELD_COAST_US,
            yield_duration_us: Self::YIELD_DURATION_US,
            hold_duty_error_start: CentiDeg::from_cdeg(Self::HOLD_DUTY_ERROR_START_CDEG),
            hold_duty_error_end: CentiDeg::from_cdeg(Self::HOLD_DUTY_ERROR_END_CDEG),
            hold_duty_min: Duty::from_raw(Self::HOLD_DUTY_MIN),
            hold_duty_max: Duty::from_raw(Self::HOLD_DUTY_MAX),
        }
    }
}
