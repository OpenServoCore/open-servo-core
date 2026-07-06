#![no_std]
#![feature(sync_unsafe_cell)]
#![allow(unexpected_cfgs)]

pub use osc_core::bp;
pub use osc_core::{BaudRate, ConfigDefaults};

pub mod cfg;
pub mod control;
pub mod hal;
pub mod log;
pub mod prelude;
pub(crate) mod providers;
pub mod runtime;
