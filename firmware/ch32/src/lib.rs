#![no_std]
#![feature(sync_unsafe_cell)]
#![allow(unexpected_cfgs)]

pub use osc_core::bp;
pub use osc_core::{BaudRate, ConfigDefaults};

pub(crate) mod bench;
pub mod cfg;
pub mod chip_flash;
pub mod control;
pub(crate) mod dxl;
pub mod hal;
pub(crate) mod idle_anchor;
pub mod log;
pub mod prelude;
pub(crate) mod providers;
pub mod runtime;
pub mod services;
pub(crate) mod statics;
#[cfg(feature = "defmt")]
pub mod telemetry;
