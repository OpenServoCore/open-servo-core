use dxl_protocol::streaming::Event;
use dxl_protocol::types::{Id, Slot, Status, StatusError};
use dxl_protocol::{Chunk, WriteError};

use crate::{BaudRate, BootMode};

/// Dispatcher-facing reply surface: encode a Status / encode a Fast slot /
/// stage a deferred config change. Per driver-pattern §7.4 these methods are
/// **data-centric** — they carry only "what to do," never "where on the wire
/// to position it." Slot positioning, RDT, chain-CRC anchoring, wire-end
/// timing all live inside the bus impl, derived from the request the bus just
/// handed the dispatcher.
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

    /// Streamed counterpart of [`Self::send_status`] for `Status::Read`
    /// replies: the dispatcher hands a [`Chunk`] iterator (sourced from
    /// a control-table read), which the bus stuffs straight into the TX
    /// buffer without a scratch copy.
    fn send_status_read_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>;

    /// Streamed counterpart of [`Self::send_slot`]: slot body bytes come
    /// from a [`Chunk`] iterator instead of a `Slot::data` slice. Slot
    /// position and chain-CRC anchor still come from the bus's cached
    /// request state.
    fn send_slot_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>;

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

/// Streaming dispatcher: bus hands one parser [`Event`] at a time, plus an
/// optional contiguous ring slice for chunked-payload events (driver resolves
/// wrap-around). `R` is generic per method so we avoid `dyn DxlReply` in the
/// hot path; each bus impl monomorphizes over its concrete reply type.
///
/// `ring` is empty for non-chunk events. For
/// `Event::Payload(Instruction(WriteDataChunk { .. }))` the slice is the
/// resolved contiguous wire bytes for that chunk (length matches the event).
pub trait DxlDispatcher {
    fn on_event<R: DxlReply>(&mut self, ev: Event, ring: &[u8], reply: &mut R);
}

/// Bus surface the DXL services layer drives. The bus owns the streaming
/// parser; on `poll` it feeds buffered wire bytes to the parser and forwards
/// each [`Event`] to the dispatcher together with the resolved `ring` slice
/// for chunked-payload events.
///
/// The dispatcher's `&mut R: DxlReply` is borrowed from a disjoint field of
/// bus-internal state, paired with the parser borrowing the wire ring — see
/// `docs/driver-pattern.md` §7.4 for the data-centric principle.
pub trait DxlBus {
    fn poll<D: DxlDispatcher>(&mut self, dispatcher: &mut D);
}
