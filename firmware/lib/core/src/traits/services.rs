use dxl_protocol::prelude::WriteBuf;

use crate::{BaudRate, BootMode, RxSnapshot};

/// What the DXL service layer reads from / writes to the bus, plus the
/// minimal scheduling primitives slot-timed replies need.
pub trait DxlBus {
    type ReplyBuffer: WriteBuf;

    fn received(&self) -> RxSnapshot<'_>;
    fn reply_buffer(&mut self) -> &mut Self::ReplyBuffer;

    /// True once the bus has been idle past `request_end` (the request finished
    /// arriving). Stashes the resolved idle moment internally so any following
    /// `send_after*` in this dispatch fires at the right deadline. Slot-timed
    /// callers (Sync/Bulk/Fast Read) MUST skip on `false`; direct unicasts
    /// MAY proceed and accept the degraded immediate fire.
    fn request_complete(&mut self, request_end: u32) -> bool;

    /// `delay_us` measured from the request's trailing idle (stashed by the
    /// preceding `request_complete`). If no idle moment is stashed, fires
    /// immediately as graceful degradation.
    fn send_after(&mut self, delay_us: u32);
    /// Same timing semantics as `send_after`, but the delay is Q8.8 µs
    /// (1 unit = 1/256 µs) — chain Last fire scheduling rounds wire-time off
    /// at sub-µs precision (3M: 3.333 µs/byte) and integer-µs flooring fires
    /// the slave ahead of the predecessor's last stop bit. Before TX, patch
    /// the reply's trailing two bytes with a CRC of inter-slave bytes
    /// received from `snoop_from` onward. `None` ⇒ Only-slot path (no
    /// predecessors to snoop).
    fn send_with_snoop_crc(&mut self, delay_q88_us: u32, snoop_from: Option<u32>);
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
