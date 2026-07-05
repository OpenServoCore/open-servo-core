//! Poll-surface vocabulary for [`CodecRx::poll`] — the verdict the framer
//! fans to its sink callback and the action the sink returns. Split out per
//! the one-type-per-file convention; the types are re-exported at the
//! `codec` root so consumers keep the `codec::{FrameVerdict, FrameAction}`
//! paths.
//!
//! [`CodecRx::poll`]: super::codec_rx::CodecRx::poll

use dxl_protocol::types::packet::Instruction;

use crate::dxl::uart::poll_src::PollSrc;

/// Packet-end timing the codec resolves and attaches to every
/// [`FrameVerdict::Instruction`] — a primitive per driver-pattern §3.3, so
/// the sink never reaches into the codec's drain-ISR stash.
///
/// `tick` is the drain-source-corrected ISR-entry estimate of the wire end
/// (`LineIdle` → `now − one frame`; `ByteBatch` → `now`), less the chip's
/// entry-latency compensation (`WireClock::PACKET_END_ENTRY_COMP_TICKS`).
/// `src` is the drain source that produced it.
#[derive(Clone, Copy)]
pub struct PacketEnd {
    pub tick: u32,
    pub src: PollSrc,
}

/// Verdict surfaced from [`CodecRx::poll`] to its sink callback.
///
/// The `Instruction` variant is one whole own/broadcast instruction frame,
/// CRC-verified and decoded from a contiguous scratch copy — the sink derives
/// the request plus reply context and dispatches once. Its `broadcast` field
/// is whether the frame arrived via the broadcast id (an SRL gating input);
/// `packet_end` is the wire-end estimate for reply timing; `fold_start_cursor`
/// is the codec's wire-byte cursor just past the frame, where a chain First
/// predecessor's leading `0xFF` will land (the Fast Last fold's start cursor).
///
/// The `SkipComplete` variant emits when a byte-skipped foreign or Status
/// frame's counter hits zero; its `id` round-trips the frame id so the chain
/// predecessor-match (`docs/dxl-streaming-rx.md` §5.2) can compare against
/// `predecessor_id`.
///
/// [`CodecRx::poll`]: super::codec_rx::CodecRx::poll
pub enum FrameVerdict<'a> {
    Instruction {
        instr: Instruction<'a>,
        broadcast: bool,
        packet_end: PacketEnd,
        fold_start_cursor: u32,
    },
    SkipComplete {
        id: u8,
    },
}

/// Return value from the sink callback.
///
/// `Stop` exits the poll immediately, leaving the remaining ring bytes in
/// place. Returned when the sink engaged a per-byte consumer (the Fast Last
/// CRC fold engine, or a deferred successor slot's status-start wait) that
/// must own all subsequent ring bytes — the poll's `fold_active()` /
/// `awaited_status_start()` self-gate only stops *future* polls, so the poll
/// already in flight bails here (`docs/dxl-streaming-rx.md` §6).
pub enum FrameAction {
    Continue,
    Stop,
}
