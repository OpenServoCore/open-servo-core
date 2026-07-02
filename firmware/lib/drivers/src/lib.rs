#![no_std]
#![feature(sync_unsafe_cell)]

pub mod dxl;
pub mod led;
pub mod log;
pub mod ring;
pub mod traits;

#[cfg(any(test, feature = "mocks"))]
extern crate std;

#[cfg(any(test, feature = "mocks"))]
pub mod mocks;

pub use traits::Level;
