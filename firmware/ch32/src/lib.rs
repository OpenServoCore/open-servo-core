#![no_std]
#![allow(unexpected_cfgs)]

pub use ch32_metapac as pac;

pub mod board;
pub mod flash;
pub mod hal;
