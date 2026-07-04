/// Chip-side control of the half-duplex DXL bus during transmission. Owns
/// the wire-driver state machine — TX_EN gating, TX DMA enable/disable, the
/// "send these bytes now, no deadline involved" path.
///
/// Sibling of [`super::TxScheduler`]. Scheduled sends need no driver
/// involvement at the deadline — the scheduler's hardware kickoff starts
/// the TX DMA at the compare match with zero CPU in the deadline path —
/// so this trait carries only the two CPU-driven transitions:
///
/// - The Plain Sync / Bulk Read chain reply at slot k > 0
///   (`docs/dxl-streaming-rx.md` §5.2) — calls [`Self::start_now`] from
///   the codec's `PollEvent::SkipComplete` arm matching the chip's
///   immediate predecessor's ID, starting the wire immediately with no
///   deadline math.
/// - Every send, scheduled or immediate, releases the bus via
///   [`Self::release_bus`] when the chip-side TC IRQ surfaces.
pub trait TxBus {
    /// Stage DMA + USART for `byte_count` outgoing bytes, drive TX_EN
    /// active, and let DMA fetch byte 0 — the first wire bit lands shortly
    /// after this returns. Used when the chip is responding to an observed
    /// wire event with no future deadline, currently the Plain Sync / Bulk
    /// Read chain reply at slot k > 0 per `docs/dxl-streaming-rx.md` §5.2.
    fn start_now(&mut self, byte_count: u16);

    /// Driver's `on_tx_complete` calls this to release the bus — drop
    /// TX_EN, disable USART TX direction + TC IRQ, disable TX DMA. Driver
    /// body then drains pending config + surfaces any pending reboot.
    fn release_bus(&mut self);
}
