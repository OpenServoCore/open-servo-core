//! Safety thresholds for fault detection.

use open_servo_math::{CentiC, CentiDeg, MilliAmp};

// ============= Default threshold constants =============

/// Default over-current threshold: 800 mA
/// Observed stall current ~180mA; 800mA provides margin for transients
/// while detecting faults before DRV8231A hardware limiting (~1.5A)
pub const DEFAULT_CURRENT_LIMIT_MA: i16 = 800;

/// Default MCU over-temperature threshold: 8000 centiC (80.0°C)
/// STM32F301 max junction temp is 105°C, leave margin
pub const DEFAULT_MCU_TEMP_LIMIT_CC: i16 = 8000;

/// Default max position change per tick: 500 centidegrees (5.0°)
/// At 10kHz, this allows ~50,000 deg/sec motion
pub const DEFAULT_POS_MAX_DELTA_CDEG: i16 = 500;

/// Default consecutive bad sensor reads before hard fault: 10
/// At 10kHz, this is ~1ms of bad readings
pub const DEFAULT_SENSOR_FAULT_COUNT: u8 = 10;

/// Default minimum position: 0 centidegrees (0°)
/// Mechanical hard limit is -5°
pub const DEFAULT_POSITION_MIN_CDEG: i16 = 0;

/// Default maximum position: 18000 centidegrees (180°)
/// Mechanical hard limit is 185°
pub const DEFAULT_POSITION_MAX_CDEG: i16 = 18000;

/// Default stall detection timeout: 1000 ticks (100ms at 10kHz)
pub const DEFAULT_STALL_TIMEOUT_TICKS: u16 = 1000;

/// Default stall position tolerance: 10 centidegrees (0.1°)
/// Position must change by more than this to not be considered stalled
pub const DEFAULT_STALL_POSITION_TOLERANCE_CDEG: i16 = 10;

/// Default position error limit: 3000 centidegrees (30°)
/// Fault if |setpoint - position| exceeds this for too long
pub const DEFAULT_POSITION_ERROR_LIMIT_CDEG: i16 = 3000;

/// Default position error timeout: 50 ticks (500ms at 100Hz slow loop)
pub const DEFAULT_POSITION_ERROR_TIMEOUT_TICKS: u16 = 50;

/// Safety thresholds for automatic fault detection.
///
/// All thresholds use the same unit types as sensor readings
/// to ensure type-safe comparisons.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SafetyThresholds {
    /// Maximum allowed motor current (absolute value compared)
    pub current_limit: MilliAmp,

    /// Maximum allowed MCU temperature
    pub mcu_temp_limit: CentiC,

    /// Maximum allowed position change per control tick
    pub position_max_delta: CentiDeg,

    /// Consecutive bad sensor reads before hard fault
    pub sensor_fault_count: u8,

    /// Minimum allowed position (setpoint clamped to this)
    pub position_min: CentiDeg,

    /// Maximum allowed position (setpoint clamped to this)
    pub position_max: CentiDeg,

    /// Stall detection timeout in ticks
    pub stall_timeout_ticks: u16,

    /// Position change tolerance for stall detection
    pub stall_position_tolerance: CentiDeg,

    /// Maximum allowed position error before fault
    pub position_error_limit: CentiDeg,

    /// Position error timeout in ticks
    pub position_error_timeout_ticks: u16,
}

impl Default for SafetyThresholds {
    fn default() -> Self {
        Self {
            current_limit: MilliAmp::from_ma(DEFAULT_CURRENT_LIMIT_MA),
            mcu_temp_limit: CentiC::from_centi_c(DEFAULT_MCU_TEMP_LIMIT_CC),
            position_max_delta: CentiDeg::from_cdeg(DEFAULT_POS_MAX_DELTA_CDEG),
            sensor_fault_count: DEFAULT_SENSOR_FAULT_COUNT,
            position_min: CentiDeg::from_cdeg(DEFAULT_POSITION_MIN_CDEG),
            position_max: CentiDeg::from_cdeg(DEFAULT_POSITION_MAX_CDEG),
            stall_timeout_ticks: DEFAULT_STALL_TIMEOUT_TICKS,
            stall_position_tolerance: CentiDeg::from_cdeg(DEFAULT_STALL_POSITION_TOLERANCE_CDEG),
            position_error_limit: CentiDeg::from_cdeg(DEFAULT_POSITION_ERROR_LIMIT_CDEG),
            position_error_timeout_ticks: DEFAULT_POSITION_ERROR_TIMEOUT_TICKS,
        }
    }
}

impl SafetyThresholds {
    /// Clamp a setpoint to the configured position bounds.
    ///
    /// This prevents the servo from driving into mechanical stops.
    #[inline]
    pub fn clamp_setpoint(&self, setpoint: CentiDeg) -> CentiDeg {
        let val = setpoint
            .as_cdeg()
            .max(self.position_min.as_cdeg())
            .min(self.position_max.as_cdeg());
        CentiDeg::from_cdeg(val)
    }
}
