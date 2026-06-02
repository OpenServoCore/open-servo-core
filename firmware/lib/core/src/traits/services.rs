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

    /// Request a wire-rate change. Called after the Status reply for the
    /// triggering WRITE has been queued but before it finishes transmitting —
    /// implementations MUST defer the actual UART retune until TX completes,
    /// otherwise the host can't decode the reply. Default no-op.
    fn set_baud(&mut self, _rate: BaudRate) {}

    /// Notify the bus that RETURN_DELAY_TIME has changed. Framing-mode pickers
    /// (`Plain` vs `Snoop` on Phase B) recompute their deadlines off RDT, so
    /// the recompute has to happen the moment the host writes the field —
    /// waiting until the next request leaves one transaction running on stale
    /// timing. Default no-op.
    fn set_return_delay(&mut self, _rdt_us: u32) {}

    /// Queue a new HSI trim delta. Implementations MUST defer the actual
    /// register write until USART1 TC drains the in-flight Status reply,
    /// since retuning HSI shifts the BRR divider mid-byte. Default no-op so
    /// chips without an HSI trim lever ignore it silently.
    fn set_clock_trim(&mut self, _delta: i8) {}

    /// Queue a new Q8.8 µs sub-trim drift residual. Implementations recompute
    /// their fire-advance compensation and apply at USART1 TC. Default no-op.
    fn set_clock_fine_trim_us(&mut self, _q88_us: i16) {}

    /// Publish a new Q8.8 µs `TX_PLAIN_LATENCY` for the plain reply path.
    /// Applied immediately — the next reply uses the new value. Default no-op.
    fn set_tx_plain_latency_us(&mut self, _q88_us: u16) {}

    /// Publish a new Q8.8 µs `TX_FAST_LATENCY` for the Fast chain (last-slave)
    /// path. Applied immediately. Default no-op.
    fn set_tx_fast_latency_us(&mut self, _q88_us: u16) {}
}

/// Lifecycle commands the dispatcher delivers to the device (reboot today;
/// factory_reset / control_table_backup / clear when those handlers land).
pub trait DeviceControl {
    /// Reboot, honored after any in-flight TX drains.
    fn reboot(&mut self, mode: BootMode);
}

/// The chip-side bundle of capabilities the services layer needs. Splits into
/// disjoint sub-trait borrows so the dispatcher can hold bus + device at once.
pub trait ServicesIo {
    type Bus: DxlBus;
    type Device: DeviceControl;

    fn parts(&mut self) -> (&mut Self::Bus, &mut Self::Device);
}
