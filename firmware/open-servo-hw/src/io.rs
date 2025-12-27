//! Board↔kernel IO boundary types.
//!
//! This module defines the **boundary shapes** between board/HAL and kernel:
//! - [`SensorFrame`]: all sensor readings produced by the board per sample
//! - [`MotorCommand`]: kernel → board motor control command
//! - [`DriveMode`]: electrical behavior of the motor bridge

use crate::samples::*;
use open_servo_units::*;

/// Per-frame sensor readings produced by the board/HAL layer.
///
/// "Frame" refers to the board sampling cadence (PWM/ADC/DMA ISR), not a specific control-loop rate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SensorFrame {
    pub pos: ServoPosSample,
    pub current: MotorCurrentSample,
    pub mcu_vdd: McuVddSample,
    pub vsys: VsysSample,
    pub ambient_temp: AmbientTempSample,
    pub motor_temp: MotorTempSample,
    pub motor_pos: MotorPosSample,
    pub driver_ok: bool,
    pub motor_v: MotorVoltageSample,
}

/// Electrical behavior of the bridge when the driver is enabled.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DriveMode {
    /// High impedance / freewheel.
    Coast,
    /// Dynamic braking (short terminals / brake mode).
    Brake,
    /// Actively drive using PWM according to `effort`.
    Drive,
}

/// Kernel -> board command at the motor-driver boundary.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct MotorCommand {
    /// Driver chip enable (power stage allowed to do anything).
    ///
    /// - `false`: driver outputs must be disabled regardless of `mode/effort`.
    /// - `true`: board applies `mode` + `effort`.
    pub driver_en: bool,

    /// Desired bridge behavior when `driver_en == true`.
    pub mode: DriveMode,

    /// Normalized actuator command (dimensionless).
    /// Only meaningful when `driver_en == true && mode == DriveMode::Drive`.
    pub effort: Effort,
}

impl MotorCommand {
    /// Unambiguously safe: driver disabled + zero effort.
    ///
    /// This is the default output for fault gating.
    #[inline]
    pub const fn safe() -> Self {
        Self {
            driver_en: false,
            mode: DriveMode::Coast,
            effort: Effort::ZERO,
        }
    }

    /// Enable driver and actively drive with effort.
    #[inline]
    pub const fn drive(effort: Effort) -> Self {
        Self {
            driver_en: true,
            mode: DriveMode::Drive,
            effort,
        }
    }

    /// Enable driver and coast (high-Z outputs).
    #[inline]
    pub const fn coast_enabled() -> Self {
        Self {
            driver_en: true,
            mode: DriveMode::Coast,
            effort: Effort::ZERO,
        }
    }

    /// Enable driver and brake (dynamic braking).
    #[inline]
    pub const fn brake_enabled() -> Self {
        Self {
            driver_en: true,
            mode: DriveMode::Brake,
            effort: Effort::ZERO,
        }
    }

    /// Disable driver regardless of bridge mode.
    #[inline]
    pub const fn driver_off() -> Self {
        Self::safe()
    }
}
