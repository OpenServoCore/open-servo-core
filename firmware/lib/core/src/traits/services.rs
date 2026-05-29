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
    /// Same timing as `send_after`, plus: before TX, patch the reply's trailing
    /// two bytes with a CRC of inter-slave bytes received from `snoop_from`
    /// onward. `None` ⇒ Only-slot path (no predecessors to snoop).
    fn send_with_snoop_crc(&mut self, delay_us: u32, snoop_from: Option<u32>);

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

    /// Boot-time clock-trim calibration trigger. Dispatcher calls this after a
    /// CAL packet has been accepted (torque off). The implementation observes
    /// byte-time on the just-received broadcast itself, computes drift, applies
    /// the chip's trim register, and updates the `comms.clock_trim` mirror.
    /// Default no-op so non-trimming chips can ignore CAL silently.
    fn trigger_clock_cal(&mut self) {}
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
