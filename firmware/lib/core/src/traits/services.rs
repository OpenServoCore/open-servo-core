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
    /// `send_after*` in this dispatch fires at the right deadline. Callers
    /// MUST skip slot-timed replies when this returns `false`.
    fn request_complete(&mut self, request_end: u32) -> bool;

    fn send(&mut self);
    /// `delay_us` measured from the request's trailing idle (stashed by the
    /// preceding `request_complete`).
    fn send_after(&mut self, delay_us: u32);
    /// Same timing as `send_after`, plus: before TX, patch the reply's trailing
    /// two bytes with a CRC of inter-slave bytes received from `snoop_from`
    /// onward. `None` ⇒ Only-slot path (no predecessors to snoop).
    fn send_with_snoop_crc(&mut self, delay_us: u32, snoop_from: Option<u32>);

    /// Request a wire-rate change. Called after the Status reply for the
    /// triggering WRITE has been queued but before it finishes transmitting —
    /// implementations MUST defer the actual UART retune until TX completes,
    /// otherwise the host can't decode the reply. Default no-op.
    fn set_baud(&mut self, _rate: BaudRate) {}
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
