/// HT/TC interrupt flags for a single DMA channel.
#[derive(Copy, Clone, Default, PartialEq, Eq, Debug)]
pub struct DmaFlags {
    pub ht: bool,
    pub tc: bool,
}

/// RX byte-ring DMA channel — NDTR accessor + HT/TC flag ack. The RX DMA
/// channel itself runs unconditionally (USART RX → byte ring); HT/TC
/// triggers a publish-only ISR (no parser drain, no codec poll) so the
/// codec's view of `write_seq` stays within `RX_BUF_LEN/2` of the wire
/// regardless of edge-ring cadence. The driver borrows one through its
/// [`Providers`] bundle. Always live — no pause/resume; redundant
/// publishes during Fast Last cost ~10 cycles and don't perturb the
/// catchup body's `drain_raw`-driven NDTR refresh.
///
/// [`Providers`]: super::Providers
pub trait RxDma {
    fn remaining(&self) -> u16;

    /// Read and clear HT/TC flags on this channel. Called from the
    /// publish-only ISR before [`Self::remaining`] feeds
    /// `CodecRx::on_rx_progress`. Returned flags are informational;
    /// the publish proceeds regardless of which crossing triggered.
    fn read_and_ack(&mut self) -> DmaFlags;

    /// Open the per-byte wake window a deferred FAST slot k > 0 uses to
    /// observe the Status packet's first byte: each received byte routes
    /// one `DxlUart::on_status_start` wake until [`Self::unwatch_status_start`]
    /// closes the window. The wake is timing-agnostic — the observed tick
    /// comes from the edge-capture ring, never from the wake itself.
    /// Spurious wakes are the driver's problem (cursor-qualified there),
    /// so the provider needs no per-byte state.
    fn watch_status_start(&mut self);

    /// Close the per-byte wake window — the status start was observed
    /// (or the wait was dropped). Idempotent; closing an unopened window
    /// is a no-op.
    fn unwatch_status_start(&mut self);
}
