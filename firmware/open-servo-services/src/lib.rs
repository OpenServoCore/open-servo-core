#![no_std]
//! # open-servo-services
//!
//! Async service logic for non-RT servo operations.
//!
//! ## Architecture
//!
//! This crate provides pure async service logic. Orchestration (signal waiting,
//! task dispatch) happens in the `runtime` crate via `Services::run_once()`.
//!
//! ```text
//! ┌─────────────────────────────────────────────────┐
//! │  ADC ISR (10kHz) - ControlExecutor              │
//! │    Deterministic, time-bounded, single-writer   │
//! └─────────────────────────────────────────────────┘
//!                ↑ KernelOp         KernelResult ↓
//! ┌─────────────────────────────────────────────────┐
//! │  Async Executor (main) - runtime::Services      │
//! │    Orchestrates: persist, rpc, dxl, etc.        │
//! └─────────────────────────────────────────────────┘
//! ```
//!
//! ## Design Principles
//!
//! - **Pure logic**: Service structs provide async methods, no signal storage
//! - **Runtime orchestration**: `runtime::Services` handles signal dispatch
//! - **Executor agnostic**: No direct embassy_sync dependency
//! - **Standard I/O**: Uses `embedded-io-async` traits for UART
//!
//! ## Service Modules
//!
//! - [`persist`]: EEPROM/flash persistence logic
//! - [`rpc`]: RPC service for debug/telemetry
//! - [`dxl_rx`]: DXL frame parsing (stub)
//! - [`dxl_req`]: DXL request handling (stub)

// Task modules
pub mod dxl_req;
pub mod dxl_rx;
pub mod persist;
pub mod rpc;
pub mod rpc_transport;

// Task type re-exports
pub use dxl_req::{DxlReqTask, Response};
pub use dxl_rx::DxlRxTask;
pub use persist::{PersistResult, PersistTask};
pub use rpc::RpcService;
pub use rpc_transport::RttTransport;
