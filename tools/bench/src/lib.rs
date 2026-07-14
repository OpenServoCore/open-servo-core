//! Host-side bench library. [`wire`] drives the osc-adapter's instrument
//! surface (raw TX + edge capture) through osc-client, [`edges`] decodes
//! captures into stamps, and [`osc`] turns osc-native frames into wire
//! bytes and parses captured exchanges back into timing + status.

pub mod cli;
pub mod discover;
pub mod edges;
pub mod osc;
pub mod run;
pub mod wire;

/// Wire bauds the firmware supports, in ascending order.
pub const SUPPORTED_BAUDS: [u32; 4] = [500_000, 1_000_000, 2_000_000, 3_000_000];

/// Default baud the chip boots at.
pub const BOOT_BAUD: u32 = 1_000_000;

/// Rate every servo listens at after a rescue break (protocol sec 9.1) -- the floor of
/// the option set, entered only via the rescue pulse.
pub const RESCUE_BAUD: u32 = 500_000;

/// Index of `baud` in [`SUPPORTED_BAUDS`] -- the `baud_rate_idx` register
/// value that selects it.
pub fn baud_index(baud: u32) -> Option<u8> {
    SUPPORTED_BAUDS
        .iter()
        .position(|&b| b == baud)
        .map(|i| i as u8)
}
