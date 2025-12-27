//! Hardware abstraction for open-servo.
//!
//! This crate defines:
//!
//! - **Boundary types** (`io`): `SensorFrame`, `MotorCommand` for board↔kernel
//! - **Sample vocabulary** (`samples`): `Sampled<T>` and typed sample aliases
//! - **Config structs** (`config`): Board configuration shapes
//!
//! ## Legacy modules (to be removed)
//!
//! The following modules are deprecated and will be deleted:
//! - `sensor`, `motor`, `peripheral`, `types`

#![no_std]
#![forbid(unsafe_code)]

pub mod config;
pub mod io;
pub mod motor;
pub mod peripheral;
pub mod samples;
pub mod sensor;
pub mod types;

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

// New boundary types (replacing legacy traits)
pub use io::{DriveMode, MotorCommand, SensorFrame};
pub use samples::{
    AmbientTempSample, McuVddSample, MotorCurrent, MotorCurrentSample, MotorPosSample,
    MotorTempSample, MotorVoltage, MotorVoltageSample, Sampled, ServoPosSample, VsysSample,
};
