//! Host-side bench library. Two layers: [`pirate`] wraps the uart-pirate's
//! USB-CDC grammar, and [`osc`] turns osc-native frames into wire bytes and
//! parses captured exchanges back into timing + status.

pub mod osc;
pub mod pirate;
pub mod run;

/// Wire bauds the firmware supports, in ascending order.
pub const SUPPORTED_BAUDS: [u32; 4] = [500_000, 1_000_000, 2_000_000, 3_000_000];

/// Default baud the chip boots at.
pub const BOOT_BAUD: u32 = 1_000_000;
