//! Safety thresholds for fault detection.

use open_servo_math::{CentiC, CentiDeg, MilliAmp};

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

    /// Position error timeout in microseconds
    pub position_error_timeout_us: u32,
}

impl SafetyThresholds {
    /// Create new safety thresholds with explicit values.
    /// Board implementations must provide all values - no defaults.
    pub fn new(
        current_limit_ma: i16,
        mcu_temp_limit_cc: i16,
        position_max_delta_cdeg: i16,
        sensor_fault_count: u8,
        position_min_cdeg: i16,
        position_max_cdeg: i16,
        stall_timeout_ticks: u16,
        stall_position_tolerance_cdeg: i16,
        position_error_limit_cdeg: i16,
        position_error_timeout_us: u32,
    ) -> Self {
        Self {
            current_limit: MilliAmp::from_ma(current_limit_ma),
            mcu_temp_limit: CentiC::from_centi_c(mcu_temp_limit_cc),
            position_max_delta: CentiDeg::from_cdeg(position_max_delta_cdeg),
            sensor_fault_count,
            position_min: CentiDeg::from_cdeg(position_min_cdeg),
            position_max: CentiDeg::from_cdeg(position_max_cdeg),
            stall_timeout_ticks,
            stall_position_tolerance: CentiDeg::from_cdeg(stall_position_tolerance_cdeg),
            position_error_limit: CentiDeg::from_cdeg(position_error_limit_cdeg),
            position_error_timeout_us,
        }
    }

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
