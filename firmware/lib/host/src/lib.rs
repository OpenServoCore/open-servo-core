//! osc-host: the host-side (bus-scheduler) core library -- engine + link.
//!
//! Grid position (driver-pattern sec 2.6): core-lib row, host column.
//! Depends on the foundation row only; sans-io, alloc-free, no statics --
//! multi-instance by construction. The engine schedules the wire (one
//! outstanding command, every protocol deadline runs here, never
//! client-side); link wraps the engine's vocabulary into pipe records.
//!
//! The provider traits deliberately mirror the servo shapes without sharing
//! code: consumer-owned interfaces (driver-pattern sec 5.1) plus the
//! never-across-columns law outrank DRY -- the servo-proven RX design ports
//! as design, not as a shared crate.
#![no_std]

#[cfg(test)]
extern crate std;

pub mod engine;
pub mod link;
pub mod traits;

#[cfg(test)]
pub(crate) mod testutil;
