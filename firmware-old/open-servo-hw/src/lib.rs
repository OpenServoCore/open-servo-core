#![no_std]
//! Hardware abstraction for open-servo.
//!
//! This crate defines:
//!
//! - **Board trait** (`board`): Unified hardware abstraction interface
//! - **Boundary types** (`io`): `SensorFrame`, `MotorCommand` for board↔kernel
//! - **Sample vocabulary** (`samples`): `Sampled<T>` and typed sample aliases
//! - **Capability flags** (`capability`): Runtime hardware capability detection
//! - **ADC vocabulary** (`adc`): Resolution enum for calibration functions
//! - **Config structs** (`config`): Board configuration shapes
//!
//! ## Legacy modules (to be removed)
//!
//! The following modules are deprecated and will be deleted:
//! - `sensor`, `motor`, `peripheral`, `types`

#![forbid(unsafe_code)]

pub mod config;
pub mod motor;
pub mod peripheral;
pub mod sensor;
pub mod types;
pub mod v2;

// Re-export commonly used items at crate root for convenience
pub use config::{
    BoardConfig, BoardKinematicsConfig, BoardPolicyConfig, BoardSafetyConfig, BoardThermalConfig,
};
pub use motor::BdcMotorDriver;
#[cfg(feature = "motor-bldc")]
pub use motor::BldcMotorDriver;
pub use peripheral::DebugIo;
pub use peripheral::SystemTime;
pub use peripheral::UartDriver;
pub use sensor::BusCurrentSensor;
pub use sensor::BusVoltageSensor;
pub use sensor::McuTemperatureSensor;
pub use sensor::MotorTemperatureSensor;
pub use sensor::MotorVoltageSensor;
pub use sensor::PositionSensor;
pub use sensor::SafetyCurrentSource;
pub use sensor::SafetyMcuTempSource;
pub use sensor::SafetyMotorTempSource;
pub use sensor::SafetyMotorVoltageSource;
pub use sensor::SafetyVoltageSource;
pub use sensor::VelocitySensor;
pub use types::UartPort;
pub use v2::Timebase;
