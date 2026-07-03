/// Chip-side control of the half-duplex DXL bus during transmission. Owns
/// the wire-driver state machine — TX_EN gating, TX DMA enable/disable, the
/// "send these bytes now, no deadline involved" path.
///
/// Sibling of [`super::TxScheduler`]. Two activation sources:
///
/// - The scheduler's deadline ISR — calls [`Self::take_bus`] to take
///   over the bus once the hardware match channel fires.
/// - The Plain Sync / Bulk Read chain reply at slot k > 0
///   (`docs/dxl-streaming-rx.md` §5.2) — calls [`Self::start_now`] from
///   the codec's `PollEvent::SkipComplete` arm matching the chip's
///   immediate predecessor's ID, fires the wire bit immediately with no
///   deadline math.
///
/// In both cases the bus is released via [`Self::release_bus`] when
/// the chip-side TC IRQ surfaces.
pub trait TxBus {
    /// Stage DMA + USART for `byte_count` outgoing bytes, drive TX_EN
    /// active, and let DMA fetch byte 0 — the first wire bit lands shortly
    /// after this returns. Used when the chip is responding to an observed
    /// wire event with no future deadline, currently the Plain Sync / Bulk
    /// Read chain reply at slot k > 0 per `docs/dxl-streaming-rx.md` §5.2.
    fn start_now(&mut self, byte_count: u16);

    /// Driver's `on_tx_start` calls this from the scheduler's deadline ISR
    /// to take over the bus (enable TX DMA so byte 0 ships). TX_EN is
    /// already up via hardware OC. Not called on the [`Self::start_now`]
    /// path — `start_now` does its own activation inline.
    fn take_bus(&mut self);

    /// Driver's `on_tx_complete` calls this to release the bus — drop
    /// TX_EN, disable USART TX direction + TC IRQ, disable TX DMA. Driver
    /// body then drains pending config + surfaces any pending reboot.
    fn release_bus(&mut self);
}
