//! osc-native transport driver (`docs/osc-native-protocol.md`): break-framed
//! RX over a counted DMA ring, enable-when-ready TX with hardware CRC,
//! snoop-driven status chains. The composite lands with the composition
//! chunk; sub-drivers are pure state machines.

pub mod chain;
mod decode;
pub mod framer;
pub mod handoff;
mod servo_bus;
pub mod tx;

pub use handoff::{DispatchConsumer, Handoff};
pub use servo_bus::{LinkDiag, ServoBus};

/// Minimum reply lead after the frame being answered, in byte-times (§7).
pub const T_TURN_BYTES: u32 = 2;

/// Ring-index wrap. The RX ring length is a power of two by contract
/// (512 on V006, §11), so this is a mask — rv32ec has no hardware divide
/// and a `% len` on a runtime length is a ~100-cycle `__udivsi3` call the
/// resolver would pay several times per frame.
#[inline(always)]
pub(crate) fn ring_wrap(i: usize, len: usize) -> usize {
    debug_assert!(
        len.is_power_of_two(),
        "RX ring length must be a power of two"
    );
    i & (len - 1)
}

/// Largest legal frame footprint in ring bytes (§3.1).
pub const FRAME_MAX: usize = osc_protocol::wire::footprint(u8::MAX);

/// View a resolved frame as up to two ring segments (one span unless it
/// wraps the seam). Shared by the HIGH decode path (speculation) and the
/// LOW dispatch consumer.
pub(crate) fn frame_view(ring: &[u8], anchor: u16, footprint: u16) -> osc_protocol::FrameBytes<'_> {
    let anchor = anchor as usize;
    let footprint = footprint as usize;
    let len = ring.len();
    if anchor + footprint <= len {
        osc_protocol::FrameBytes::from(&ring[anchor..anchor + footprint])
    } else {
        osc_protocol::FrameBytes::new(&ring[anchor..], &ring[..footprint - (len - anchor)])
    }
}

/// CRC spin backstop, iterations per covered byte: the hardware engine runs
/// ~8x wire speed (F6), so this is orders of magnitude past any healthy
/// completion. Shared by TX generation ([`tx`]) and RX validation
/// ([`servo_bus`]) — one CRC engine serves both.
pub(crate) const SPIN_PER_BYTE: u32 = 64;
