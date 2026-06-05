use dxl_protocol::{Packet, Slot, SlotPosition, Status};

use crate::services::dxl::{OscExt, OscReplyExt};
use crate::{BaudRate, BootMode};

/// Wire scheduling info for a single outbound reply. The chip translates
/// these protocol-structural fields into wall-clock delays via its own
/// cached `byte_time` (which is the rate currently on the wire — after a
/// BAUD write, this still reflects the old rate until USART TC drains the
/// reply and the deferred retune lands).
#[derive(Copy, Clone, Debug)]
pub struct Schedule {
    /// Return Delay Time in µs (from `comms.return_delay_2us × 2`).
    pub rdt_us: u32,
    /// Structural wire-byte offset of this reply on the wire relative to the
    /// master's request end. Zero for direct replies; nonzero for Sync/Bulk
    /// slot N and Fast First/Middle/Last.
    pub bytes_before: u32,
    /// Position in the slot train (0-indexed). 0 for direct replies. Chip
    /// uses this to compose its inter-slot margin policy for plain Status
    /// trains (Sync Read / Bulk Read). Ignored for Fast variants.
    pub slot_index: u16,
}

/// Bus surface the DXL services layer reads, writes, and schedules through.
/// `poll` returns the single dispatchable packet ending at the latest
/// IDLE-anchored wire-end. `send` emits one of the non-fast typed Status
/// shapes (full standalone status frame). `send_slot` emits one piece of a
/// Fast Sync/Bulk Read coalesced response and patches the chain CRC when the
/// slot is the last in the response.
pub trait DxlBus {
    /// Latest IDLE-anchored request, or `None` if no fresh anchor since the
    /// previous `poll`. The returned `Packet`'s borrowed bytes live in chip
    /// storage that stays valid until the next `poll` overwrites it. OSC
    /// vendor verbs surface as `Packet::Ext(OscVariant::..)`.
    fn poll(&mut self) -> Option<Packet<'static, OscExt>>;

    /// Compose a standalone Status reply on the chip's TX buffer and schedule
    /// it. The chip owns all wire-timing math: it consumes `Schedule` to
    /// compute the fire delay (RDT + per-byte translation).
    fn send(&mut self, status: Status<'_, OscReplyExt>, schedule: Schedule);

    /// Compose one Fast Sync/Bulk Read slot on the chip's TX buffer and
    /// schedule it. `position` selects header / body / CRC framing:
    /// `Only` and `Last` emit (or reserve) the response CRC; `First` and
    /// `Middle` don't (successors continue the response).
    fn send_slot(&mut self, slot: Slot<'_>, position: SlotPosition, schedule: Schedule);
}

/// Fire-and-forget notifications the dispatcher delivers when control-table
/// writes or lifecycle instructions land. Chip impls match exhaustively —
/// unsupported variants get an explicit no-op arm, not a silent default.
pub enum Event {
    /// Wire-rate change. Queued after the Status reply for the triggering
    /// WRITE is buffered but before TX completes — impls MUST defer the UART
    /// retune until TC, otherwise the host can't decode the reply.
    SetDxlBaud(BaudRate),
    /// New HSI trim delta. Impls MUST defer the register write until USART TC
    /// drains the in-flight Status reply, since retuning HSI shifts the BRR
    /// divider mid-byte.
    SetClockTrim(i8),
    /// New Q8.8 µs sub-trim drift residual. Impls recompute fire-advance
    /// compensation and apply at USART TC.
    SetClockFineTrimUs(i16),
    /// Reboot, honored after any in-flight TX drains.
    Reboot(BootMode),
}

/// Sink for `Event` notifications. Single dispatch keeps the trait surface
/// thin — extending the protocol means adding a variant, not a method.
pub trait ServiceEvents {
    fn send(&mut self, event: Event);
}

/// The chip-side bundle of capabilities the services layer needs. Splits into
/// disjoint sub-trait borrows so the dispatcher can hold bus + events at once.
pub trait ServicesIo {
    type Bus: DxlBus;
    type Events: ServiceEvents;

    fn parts(&mut self) -> (&mut Self::Bus, &mut Self::Events);
}
