//! Poll-surface vocabulary for [`CodecRx::poll`] — the event the codec
//! fans to its sink callback and the action the sink returns. Split out
//! per the one-type-per-file convention; the types are re-exported at the
//! `codec` root so consumers keep the `codec::{PollEvent, PollAction}`
//! paths.
//!
//! [`CodecRx::poll`]: super::codec_rx::CodecRx::poll

use dxl_protocol::streaming::Event;

use crate::dxl::uart::poll_src::PollSrc;

/// Packet-end timing the codec resolves and attaches to every `Crc`
/// [`PollEvent::Parser`] — primitives per driver-pattern §3.3, so the
/// sink never reaches into the codec's edge-capture half.
///
/// `tick` is the anchored wire-end tick (CRC byte start + one byte-time,
/// lifted into the WireClock u32 domain); `None` means interference /
/// edge loss starved the tail-anchor back-search — the sink owns the
/// anchor-miss telemetry and the decision whether `fallback_tick` (the
/// per-`src` ISR-entry estimate) is an acceptable substitute.
#[derive(Clone, Copy)]
pub struct PacketEnd {
    pub tick: Option<u32>,
    pub fallback_tick: u32,
    pub src: PollSrc,
}

/// Event surfaced from [`CodecRx::poll`] to its sink callback.
///
/// `Parser { .. }` is a 1:1 forward of [`Event`]; the codec translates
/// `WriteDataChunk` `(offset, length)` into a contiguous ring slice via
/// `ring` (empty for other events). `next_status_pos` is the codec's
/// wire-byte position at this event's emit point — the value
/// `wire_bytes_consumed` will hold after the parser has consumed the
/// bytes that produced this event. Named for its load-bearing use: the
/// Fast Last fold path captures it at the chain instruction's Crc event
/// as `fold_start_cursor` — at that point it equals the wire position
/// where the First predecessor's status packet will start (the next byte
/// on the wire). At Header / Payload events the value is still the
/// codec's running cursor, but no consumer reads it there today.
/// `packet_end` is `Some` exactly at `Crc` events — see [`PacketEnd`].
/// `SkipComplete` fires when the universal byte-skip's remaining-byte
/// counter hits zero — `id` round-trips the value the sink passed in
/// [`PollAction::Skip`], so the chain predecessor-match check
/// (doc §5.2) can compare against `predecessor_id`.
///
/// [`CodecRx::poll`]: super::codec_rx::CodecRx::poll
pub enum PollEvent<'a> {
    Parser {
        ev: Event,
        ring: &'a [u8],
        next_status_pos: u32,
        packet_end: Option<PacketEnd>,
    },
    SkipComplete {
        id: u8,
    },
}

/// Return value from the sink callback.
///
/// `Skip` is meaningful only after [`PollEvent::Parser`] carrying a
/// [`Event::Header`]; on other events the codec ignores it and continues
/// normally. On `Skip`, the codec reads `Parser::packet_remaining`,
/// resets the parser, and consumes the indicated count from the RX ring
/// tail before surfacing [`PollEvent::SkipComplete`].
///
/// `Stop` exits the poll immediately after committing the bytes the
/// parser already consumed for the in-flight event. Returned when the
/// sink engages a per-byte consumer (today: the Fast Last CRC fold
/// engine) that must own all subsequent ring bytes; leaving them in the
/// ring is the only way to hand them off without losing the cursor. Used
/// to honor the RX-tail ownership contract across the fold-start boundary
/// itself (`dxl-streaming-rx.md` §6) — the edge-IRQ mask at fold start
/// only stops *future* polls.
pub enum PollAction {
    Continue,
    Skip { id: u8 },
    Stop,
}
