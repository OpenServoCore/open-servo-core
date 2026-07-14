//! Host transport interfaces, engine-owned (driver-pattern sec 5.1); chip
//! providers implement them over real peripherals, tests over recording
//! fakes. Deliberately absent (see the crate doc for the grid rationale):
//! a break-event input (no LBD on the adapter target; all host RX is
//! solicited, the framer walks the ring), a CRC engine (software CRC from
//! osc-protocol -- the host has cycles to spare), and a separate monotonic
//! (`Deadline::now` covers it).

use osc_protocol::wire::BaudRate;

/// Counted circular RX DMA ring, armed once. Contract: the ring never
/// contains the host's own TX bytes (HDSEL no-echo or buffer mute -- the
/// provider guarantees it, the framer assumes it). Ring length must be a
/// power of two exceeding the largest legal frame (258) with polling margin.
pub trait RxRing {
    fn bytes(&self) -> &[u8];
    /// Index where the next received byte lands.
    fn cursor(&self) -> u16;
}

/// One-shot compare on the host tick domain. The host is crystal-clocked --
/// it is the bus's syntonization root (protocol sec 9.3) -- so there is no
/// trim-step constant here. `set` arms the compare; the chip ISR it fires
/// calls back into the engine's `on_deadline`.
pub trait Deadline {
    const TICKS_PER_US: u32;
    fn now(&self) -> u32;
    fn set(&mut self, at: u32);
    fn cancel(&mut self);
}

/// TX side of the wire: drive discipline + law break + byte streaming.
/// Send completion surfaces as the chip calling the engine's
/// `on_tx_complete`. The provider owns break/M-bit sequencing internally --
/// a `send` queued behind a break is legal, and blocking a character time
/// inside the provider is acceptable.
pub trait TxWire {
    /// Claim the wire: push-pull drive (protocol sec 2 discipline).
    fn claim(&mut self);
    /// One law break: a 10-bit `0x00` character (protocol sec 3). Wire
    /// claimed. Never the off-law hardware SBK (~14 bits).
    fn send_break(&mut self);
    /// Stream one span (DMA arm or PIO -- provider's choice).
    fn send(&mut self, span: &[u8]);
    /// Drive the line dominant and hold it (rescue pulse, protocol sec 9.1);
    /// ends at [`release`](Self::release). The engine times the pulse.
    fn hold_low(&mut self);
    /// Release the wire: open-drain idle, TX off.
    fn release(&mut self);
}

/// Host-side UART rate: the four operational rates; `BaudRate::RESCUE` is
/// the sec 9.1 floor the rescue verb drops to.
pub trait UsartBaud {
    fn apply(&mut self, baud: BaudRate);
}

/// Timestamped wire-edge capture, the adapter-as-instrument organ: both
/// polarities of every wire transition, hardware-stamped on the `Deadline`
/// tick domain's low 16 bits, captured continuously from boot (never
/// re-armed -- re-arming mangles captures, linke-edgecap spike). Drains
/// are consumer-paced like the rest of the instrument surface; a ring lap
/// between drains sets the sticky overflow flag rather than lying.
///
/// Capture-fidelity contract the PC decoder must honor (spike-measured):
/// data-region edges are exact; break edges are not captured faithfully
/// (phantoms inside the break span, break fall never present), so frames
/// anchor on their byte-0 start fall and break geometry is law-derived.
pub trait EdgeCapture {
    /// Pop up to `buf.len()` falling-edge ticks in capture order.
    fn drain_falls(&mut self, buf: &mut [u16]) -> usize;
    /// Pop up to `buf.len()` rising-edge ticks in capture order.
    fn drain_rises(&mut self, buf: &mut [u16]) -> usize;
    /// A ring lapped undrained captures since the last reset -- sticky.
    fn overflow(&self) -> bool;
    /// Drop unread captures and the overflow flag.
    fn reset(&mut self);
}

/// Role bundle for the `HostBus` composite (driver-pattern sec 5.4).
pub trait Providers {
    type Ring: RxRing;
    type Deadline: Deadline;
    type Tx: TxWire;
    type Baud: UsartBaud;
    type Edges: EdgeCapture;
}

/// Wrap-aware "`a` is at or after `b`" on the u32 tick domain.
#[inline]
pub const fn tick_reached(a: u32, b: u32) -> bool {
    a.wrapping_sub(b) < u32::MAX / 2
}
