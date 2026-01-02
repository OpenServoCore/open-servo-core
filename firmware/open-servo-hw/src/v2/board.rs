//! Unified hardware abstraction trait.
//!
//! The `Board` trait is the single interface between kernel and hardware.
//! It replaces the previous 24+ fragmented traits with a minimal, cohesive API.

use crate::v2::capability::{MotorType, SensorCapabilities};
use crate::v2::io::{MotorCommand, SensorFrame};
use crate::v2::ServoPosKind;

/// Unified hardware abstraction for servo boards.
///
/// This is intentionally minimal. Timing (`now`, `dt_us`) is passed as parameters
/// to kernel tick functions by the board/HAL caller, not via this trait.
///
/// # Design notes
///
/// - Board owns ADC DMA buffer internally (sidesteps const generic in trait)
/// - All sensor readings are returned via [`SensorFrame`]
/// - Optional sensors use `None` when not present, `Some(Reading::Valid/Invalid)` otherwise
/// - Motor commands go through [`MotorCommand`]
pub trait Board {
    /// Servo position semantics for this board.
    fn servo_pos_kind(&self) -> ServoPosKind;

    /// Motor topology for this board (BDC or BLDC).
    fn motor_type(&self) -> MotorType;

    /// Runtime capability flags for optional sensors.
    fn sensor_capabilities(&self) -> SensorCapabilities;

    /// Read all sensor samples from internal ADC buffer.
    ///
    /// Board owns `AdcDmaBuf<N>` internally (N determined by board features).
    /// This method converts raw ADC values to typed samples using calibration.
    fn read_sensors(&mut self) -> SensorFrame;

    /// Apply motor command to hardware.
    fn write_motor(&mut self, cmd: MotorCommand);
}
