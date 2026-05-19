#![no_std]
#![feature(sync_unsafe_cell)]
#![allow(unexpected_cfgs)]

pub use ch32_metapac as pac;
pub use osc_core::ConfigDefaults;

pub mod board;
pub mod flash;
pub mod hal;
pub mod irq;
pub mod statics;
