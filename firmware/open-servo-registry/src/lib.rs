//! Register map definitions for servo kernels.
//!
//! This crate provides field specifications and lookup functions for
//! services (REPL, Dynamixel bridge, debug tools) to access shadow table fields.
//!
//! # Usage
//!
//! ```ignore
//! use open_servo_registry::{find, RegSpec, Encoding};
//!
//! // Lookup by name prefix
//! if let Some(spec) = find("ctrl.mode") {
//!     println!("{}: offset={:#x}, len={}", spec.name, spec.offset, spec.len);
//! }
//! ```

#![no_std]

pub mod servo_kernel;
pub mod spec;

// Re-export main types
pub use servo_kernel::{find, ALL_FIELDS, CTRL_FIELDS, TELEM_FIELDS};
pub use spec::{Encoding, RegSpec};

// Re-export compat module for kernel use
pub use servo_kernel::compat;
