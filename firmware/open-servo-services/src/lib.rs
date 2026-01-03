#![no_std]
//! # open-servo-services
//!
//! Embassy async tasks for non-RT servo operations.
//!
//! ## Architecture
//!
//! This crate provides async service tasks that run in the embassy executor
//! (main loop context), separate from the RT kernel (ADC ISR context).
//!
//! ```text
//! ┌─────────────────────────────────────────────────┐
//! │  ADC ISR (10kHz) - ControlExecutor              │
//! │    Deterministic, time-bounded, single-writer   │
//! └─────────────────────────────────────────────────┘
//!                ↑ KernelOp         KernelResult ↓
//! ┌─────────────────────────────────────────────────┐
//! │  Embassy Executor (main) - Services             │
//! │    dxl_rx, dxl_req, persist, rpc, etc.          │
//! └─────────────────────────────────────────────────┘
//! ```
//!
//! ## Design Principles
//!
//! - **Separation**: RT kernel is ISR-owned; services are async-owned
//! - **Single-outstanding**: Only one `KernelOp` in flight at a time
//! - **Copy-in TX**: `TxFrame` uses owned buffers (DMA can't outlive borrow)
//! - **Soft-time**: `embassy-time` for periodic tasks; DXL timing uses RT timer
//!
//! ## Tasks
//!
//! - [`dxl_rx`]: Parse DXL frames from UART RX buffer
//! - [`dxl_req`]: Handle DXL requests, dispatch to kernel or shadow
//! - [`persist`]: EEPROM/flash persistence on signal
//!
//! ## Usage
//!
//! Board crate spawns tasks in `#[embassy_executor::main]`:
//!
//! ```rust,ignore
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     spawner.spawn(dxl_rx_task(...)).unwrap();
//!     spawner.spawn(dxl_req_task(...)).unwrap();
//!     spawner.spawn(persist_task(...)).unwrap();
//! }
//! ```

// Task modules
pub mod dxl_req;
pub mod dxl_rx;
pub mod persist;
pub mod rpc;
pub mod rpc_transport;

// Re-exports for convenience
pub use embassy_executor::Spawner;
pub use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
pub use embassy_sync::channel::Channel;
pub use embassy_sync::signal::Signal;

// Task type re-exports
pub use dxl_req::{DxlReqTask, KernelResultSignal, Response};
pub use dxl_rx::{DxlRxTask, OpChannel, OP_CHANNEL_CAPACITY};
pub use persist::{PersistResult, PersistSignal, PersistTask};
pub use rpc::RpcService;
pub use rpc_transport::RttTransport;
