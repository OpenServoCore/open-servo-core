#![no_std]
//! # open-servo-kernel
//!
//! Concrete kernel implementation.
//!
//! Stage-0 goals:
//! - PID position control + open-loop effort
//! - Clear gating (fault/engage/driver_ok)
//! - Minimal mode switching policy
//! - Shadow table integration for host-plane register I/O

pub mod kernel;
pub mod state;

pub use kernel::ServoKernel;
pub use state::{KernelConfig, KernelState, PidGains};
