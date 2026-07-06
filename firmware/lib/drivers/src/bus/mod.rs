//! osc-native transport driver (`docs/osc-native-protocol.md`): break-framed
//! RX over a counted DMA ring, enable-when-ready TX with hardware CRC,
//! snoop-driven status chains. The composite lands with the composition
//! chunk; sub-drivers are pure state machines.

pub mod chain;
mod decode;
pub mod framer;
mod servo_bus;
pub mod tx;

pub use servo_bus::{LinkDiag, ServoBus};

/// Minimum reply lead after the frame being answered, in byte-times (§7).
pub const T_TURN_BYTES: u32 = 2;

/// Largest legal frame footprint in ring bytes (§3.1).
pub const FRAME_MAX: usize = osc_protocol::wire::footprint(u8::MAX);

/// CRC spin backstop, iterations per covered byte: the hardware engine runs
/// ~8x wire speed (F6), so this is orders of magnitude past any healthy
/// completion. Shared by TX generation ([`tx`]) and RX validation
/// ([`servo_bus`]) — one CRC engine serves both.
pub(crate) const SPIN_PER_BYTE: u32 = 64;
