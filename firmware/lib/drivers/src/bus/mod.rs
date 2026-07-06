//! osc-native transport driver (`docs/osc-native-protocol.md`): break-framed
//! RX over a counted DMA ring, enable-when-ready TX with hardware CRC,
//! snoop-driven status chains. The composite lands with the composition
//! chunk; sub-drivers are pure state machines.

pub mod chain;
pub mod framer;
pub mod tx;

/// Minimum reply lead after the frame being answered, in byte-times (§7).
pub const T_TURN_BYTES: u32 = 2;

/// Largest legal frame footprint in ring bytes (§3.1).
pub const FRAME_MAX: usize = osc_protocol::wire::footprint(u8::MAX);
