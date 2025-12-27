#![no_std]
//! # open-servo-kernel
//!
//! Concrete kernel implementation.
//!
//! Stage-0 goals:
//! - PID position control + open-loop effort
//! - Clear gating (fault/engage/driver_ok)
//! - Minimal mode switching policy
//! - Minimal host-plane register ops (not full regmap yet)
//!
//! This crate intentionally does **not** commit to a registry architecture yet.
//! You can evolve to a real regmap later without breaking the realtime contract.

pub mod kernel;
pub mod regs;
pub mod state;

pub use kernel::ServoKernel;
pub use state::{KernelConfig, KernelState, PidGains};
