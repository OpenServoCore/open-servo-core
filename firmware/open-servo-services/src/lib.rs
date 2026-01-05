#![no_std]
//! # open-servo-services
//!
//! Executor-agnostic async services for non-realtime servo operations.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────┐
//! │  ADC ISR (10kHz) - ControlExecutor              │
//! │    Deterministic, time-bounded, single-writer   │
//! └─────────────────────────────────────────────────┘
//!                ↑ KernelOp         KernelResult ↓
//! ┌─────────────────────────────────────────────────┐
//! │  Async Executor (main loop)                     │
//! │    persist, rpc, dxl services                   │
//! └─────────────────────────────────────────────────┘
//! ```
//!
//! ## Executor-Agnostic Design
//!
//! This crate has **zero dependency on any async runtime** (embassy, tokio, etc).
//! Services use trait bounds from [`open_servo_hw::v2`]:
//!
//! | Trait | Purpose |
//! |-------|---------|
//! | [`AsyncTimer`] | Monotonic time and delays |
//! | [`SignalReader`] | Async event waiting |
//! | [`SignalWriter`] | Event signaling (sync side) |
//!
//! Firmware provides concrete implementations via newtype wrappers:
//!
//! ```rust,ignore
//! // In firmware crate
//! struct EmbassyTimer;
//! impl AsyncTimer for EmbassyTimer { ... }
//!
//! struct EmbassySignal(&'static Signal<...>);
//! impl SignalReader for EmbassySignal { ... }
//! ```
//!
//! ## Service Modules
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`persist`] | EEPROM/flash persistence with wear-leveling |
//! | [`rpc`] | Debug/telemetry RPC over RTT |
//! | [`dxl_rx`] | Dynamixel frame parsing |
//! | [`dxl_req`] | Dynamixel request handling |
//!
//! [`AsyncTimer`]: open_servo_hw::v2::AsyncTimer
//! [`SignalReader`]: open_servo_hw::v2::SignalReader
//! [`SignalWriter`]: open_servo_hw::v2::SignalWriter

// Service modules
pub mod bundle;
pub mod dxl_req;
pub mod dxl_rx;
pub mod persist;
pub mod rpc;
pub mod rpc_transport;

// Primary re-exports
pub use bundle::{ServiceError, Services};
pub use persist::{PersistResult, PersistTask};
pub use rpc::RpcService;
pub use rpc_transport::RttTransport;

// Secondary re-exports
pub use dxl_req::{DxlReqTask, Response};
pub use dxl_rx::DxlRxTask;
