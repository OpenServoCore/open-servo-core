//! Peripheral traits for UART, timing, and debug I/O.

mod debug_io;
mod time;
mod uart;

pub use debug_io::*;
pub use time::*;
pub use uart::*;
