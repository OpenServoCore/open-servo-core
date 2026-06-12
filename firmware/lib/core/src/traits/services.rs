use dxl_protocol::packet::{Slot, Status};
use dxl_protocol::{InstructionPacket, WriteError};

use crate::{BaudRate, BootMode};

/// Dispatcher-facing reply surface: encode a Status / encode a Fast slot /
/// cancel an in-flight fire / stage a deferred config change. Per
/// driver-pattern §7.4 these methods are **data-centric** — they carry only
/// "what to do," never "where on the wire to position it." Slot positioning,
/// RDT, chain-CRC anchoring, wire-end timing all live inside the bus impl,
/// derived from the request the bus just handed the dispatcher.
///
/// Reply trait is separate from [`DxlBus`] so the dispatcher (and
/// [`ControlTableHooks`]) can be generic over `DxlReply` alone — it never
/// re-polls mid-dispatch. The chip-side `DxlBus` impl spawns a `&mut dyn
/// DxlReply` through the closure handed to [`DxlBus::poll`]; the chip's
/// `DxlBus` type itself does *not* impl `DxlReply` because reply state lives
/// in driver internals reachable only through the disjoint split-borrow that
/// `poll` orchestrates.
pub trait DxlReply {
    /// Encode a standalone Status reply and arm its fire. Bus folds RDT +
    /// slot offset (broadcast Ping / Sync/Bulk Read slot N) from its cached
    /// request state; the dispatcher passes only the reply data.
    fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError>;

    /// Encode one Fast Sync/Bulk Read slot reply and arm its fire. Slot
    /// position (Only/First/Middle/Last) and chain-CRC anchor come from the
    /// bus's cached request state; Last engages the chain-CRC fold
    /// scheduler arm.
    fn send_slot(&mut self, slot: &Slot<'_>) -> Result<(), WriteError>;

    /// Stage a deferred ID change — applies after the next TX completes.
    fn stage_id(&mut self, id: u8);

    /// Stage a deferred baud-rate change. Applied at the next USART TC so
    /// the in-flight reply finishes at the old baud.
    fn stage_baud(&mut self, baud: BaudRate);

    /// Stage a deferred Return Delay Time change in µs.
    fn stage_rdt(&mut self, us: u32);

    /// Stage a deferred reboot, honored after any in-flight TX drains.
    fn stage_reboot(&mut self, mode: BootMode);
}

/// Bus surface the DXL services layer drives. Closure-based `poll` to break
/// the otherwise-fatal borrow conflict between the parsed
/// [`InstructionPacket`] (borrowed from bus-internal RX storage) and the
/// `&mut` access the dispatcher needs for send / stage calls on the bus.
/// The implementor splits its internal state into disjoint RX and TX halves
/// and hands both into the closure simultaneously — see
/// [`docs/driver-pattern.md §7.4`](../../../../../docs/driver-pattern.md)
/// for the data-centric principle.
///
/// The parser lives entirely in the driver per [`docs/dxl-hw-timed-transport.md §10.5`](../../../../../docs/dxl-hw-timed-transport.md);
/// the trait carries no CRC type parameter because services never touch the
/// decoder. Staging methods (`stage_*`) and `cancel` only exist on
/// [`DxlReply`] — the bus type itself is poll-only.
pub trait DxlBus {
    /// Drain bus state until a request addressed to us (or BROADCAST)
    /// surfaces; if so, invoke `f` with the parsed packet and a reply
    /// handle. Foreign-ID requests and Status frames are consumed silently
    /// (they may feed bus-internal calibration state but never reach the
    /// closure). `f` is called at most once per `poll` call — implementors
    /// must NOT loop back into the closure after a return.
    fn poll<F>(&mut self, f: F)
    where
        F: for<'a> FnOnce(InstructionPacket<'a>, &mut dyn DxlReply);
}
