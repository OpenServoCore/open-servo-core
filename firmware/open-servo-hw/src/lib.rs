//! Hardware abstraction traits for open-servo.
//!
//! This crate defines the hardware interface that board crates implement.
//! It provides:
//!
//! - **Sensor traits** (`sensor`): Position, current, voltage, temperature sensing
//! - **Motor traits** (`motor`): BDC and BLDC motor driver control
//! - **Peripheral traits** (`peripheral`): UART, timing, debug I/O
//!
//! ## Safety Capability Traits
//!
//! Sensor traits include "Safety*Source" variants that return `Option<T>`.
//! Boards with a sensor get automatic implementation via blanket impl.
//! Boards without implement the safety trait directly, returning `None`.
//!
//! This allows SafetyManager to automatically skip checks for unavailable sensors.

#![no_std]
#![forbid(unsafe_code)]

pub mod config;
pub mod motor;
pub mod peripheral;
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
