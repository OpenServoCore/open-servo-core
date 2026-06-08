use dxl_protocol::{CrcUmts, Slot, SlotPosition, Status};

use crate::services::dxl::OscReplyExt;
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

/// Chip-side drift-filter state surfaced for the CALIB Status reply.
/// `observed_ticks` / `nominal_ticks` describe the most recently snooped
/// packet (the CALIB request itself, when the request triggered the
/// snapshot via the normal `poll` path); `applied_*` carry the most recent
/// batched apply queued by the chip's filter. Returned `None` when the
/// chip-side filter hasn't observed any packet yet — dispatcher treats this
/// as a measurement failure.
#[derive(Copy, Clone, Debug)]
pub struct CalSnapshot {
    pub observed_ticks: u32,
    pub nominal_ticks: u32,
    pub applied_trim_delta: i8,
    pub applied_fine_trim_us: i16,
}

/// Bus surface the DXL services layer reads, writes, and schedules through.
/// `rx_window` exposes the latest IDLE-anchored byte window as 1-2
/// contiguous slices the kernel-side `Dxl` feeds through a streaming
/// [`Decoder`](dxl_protocol::decoder::Decoder); `send` emits one of the
/// non-fast typed Status shapes (full standalone status frame).
/// `send_slot` emits one piece of a Fast Sync/Bulk Read coalesced response
/// and patches the chain CRC when the slot is the last in the response.
pub trait DxlBus {
    /// CRC engine implementation used to validate inbound frames.
    type Crc: CrcUmts;

    /// Latest IDLE-anchored RX window as a `(head, tail)` slice pair (the
    /// tail is non-empty only when the burst wraps the ring). Returns
    /// `None` when no fresh window has arrived since the previous call.
    /// The returned slices stay valid until the next `rx_window` call.
    fn rx_window(&mut self) -> Option<(&[u8], &[u8])>;

    /// Snoop hook called by `Dxl::poll` exactly once per successfully
    /// decoded non-Status packet. Chip impls use it to drive any
    /// per-packet calibration state machine (e.g. HSI drift filter).
    fn snoop(&mut self);

    /// Compose a standalone Status reply on the chip's TX buffer and schedule
    /// it. The chip owns all wire-timing math: it consumes `Schedule` to
    /// compute the fire delay (RDT + per-byte translation).
    fn send(&mut self, status: Status<'_, OscReplyExt>, schedule: Schedule);

    /// Compose one Fast Sync/Bulk Read slot on the chip's TX buffer and
    /// schedule it. `position` selects header / body / CRC framing:
    /// `Only` and `Last` emit (or reserve) the response CRC; `First` and
    /// `Middle` don't (successors continue the response).
    fn send_slot(&mut self, slot: Slot<'_>, position: SlotPosition, schedule: Schedule);

    /// Snapshot the wire-timing state captured during RX of the just-polled
    /// CALIB request. Called from the dispatcher's CALIB path; impls without
    /// a first-byte capture mechanism return `None`.
    fn cal_snapshot(&mut self) -> Option<CalSnapshot>;
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
