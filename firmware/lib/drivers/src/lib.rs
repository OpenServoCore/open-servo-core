#![no_std]
#![feature(sync_unsafe_cell)]
#![feature(fn_align)]

pub mod bench;
pub mod bus;
pub mod led;
pub mod log;
pub mod tel;
pub mod traits;

#[cfg(any(test, feature = "mocks"))]
extern crate std;

#[cfg(any(test, feature = "mocks"))]
pub mod mocks;

pub use traits::Level;
