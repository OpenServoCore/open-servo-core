use dxl_protocol::prelude::WriteBuf;

use crate::{BaudRate, BootMode};

/// Bus surface the DXL services layer reads, writes, and schedules through.
/// Every dispatched packet runs under an IDLE-anchored window; the chip
/// tracks the wire-end timestamp internally so `send_*` delays land on it.
pub trait DxlBus {
    type TxBuffer: WriteBuf;

    /// New RX bytes since the previous `rx_poll`, sliced to the latest
    /// IDLE-anchored wire-end. `(head, tail)` — `tail` is empty unless the
    /// burst wrapped the ring. Returns `None` until the next IDLE fires.
    /// The chip also stashes the matching wire-end tick + byte cursor for
    /// the next `send_*` to consume.
    fn rx_poll(&mut self) -> Option<(&[u8], &[u8])>;

    fn tx_buffer(&mut self) -> &mut Self::TxBuffer;

    /// Schedule a plain Status reply at `wire_end + delay_us`.
    fn send_after(&mut self, delay_us: u32);
    /// Q8.8-µs delay variant — chain Last fire scheduling rounds wire-time at
    /// sub-µs precision (3M: 3.333 µs/byte) and integer-µs flooring would
    /// fire ahead of the predecessor's last stop bit. When `snoop` is true,
    /// the chip patches the reply's trailing CRC over inter-slave bytes
    /// received past its stored wire-end cursor.
    fn send_with_snoop_crc(&mut self, delay_q88_us: u32, snoop: bool);
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
