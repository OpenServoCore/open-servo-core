//! osc-host-ch32 -- the host column's orchestration crate (driver-pattern
//! sec 2.4/2.6): CH32V305 providers for the `osc_host` engine traits, the
//! USBHS pipe behind the link server, and the IAP flash service. All
//! protocol logic lives in `osc-host`; nothing here decodes a frame.
//!
//! One chip, one board (WCH-LinkE R0-1v3 as `osc-adapter-wchlinke`):
//! layers stay flat files, and the pin map lives in `providers::pins`
//! with its rules instead of behind a wiring schema.
//!
//! `hal::*` access is confined to `providers/` and `runtime/` (grid law 3).

#![no_std]
#![feature(sync_unsafe_cell)]
#![cfg_attr(target_arch = "riscv32", feature(abi_riscv_interrupt))]

pub mod hal;
pub(crate) mod providers;
pub mod runtime;
