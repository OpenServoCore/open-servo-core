/// HT/TC interrupt flags for a single DMA channel.
#[derive(Copy, Clone, Default, PartialEq, Eq, Debug)]
pub struct DmaFlags {
    pub ht: bool,
    pub tc: bool,
}

/// RX byte-ring DMA channel — NDTR accessor + HT/TC flag ack. The RX DMA
/// channel itself runs unconditionally (USART1 → byte ring); HT/TC fires
/// a publish-only ISR (no parser drain, no codec poll) so the codec's view
/// of `write_seq` stays within `RX_BUF_LEN/2` of the wire regardless of
/// edge-ring cadence. The driver borrows one through its [`Providers`]
/// bundle; the production adapter binds to DMA1_CH5. Always live — no
/// pause/resume; redundant publishes during Fast Last cost ~10 cycles and
/// don't perturb the catchup body's `drain_raw`-driven NDTR refresh.
///
/// [`Providers`]: super::Providers
pub trait RxDma {
    fn remaining(&self) -> u16;

    /// Read and clear HT/TC flags on this channel. Called from the
    /// publish-only ISR before [`Self::remaining`] feeds
    /// `CodecRx::on_rx_progress`. Returned flags are informational;
    /// the publish proceeds regardless of which crossing fired.
    fn read_and_ack(&mut self) -> DmaFlags;

    /// Bump the `edge_anchor_miss` telemetry counter. Called once per
    /// parser Crc event where the classifier had no anchor (interference
    /// / edge loss) and the composite invoked
    /// [`crate::dxl::uart::PollSrc`]-driven fallback for
    /// `packet_end_tick`. Wire-condition floor signal — peer of the
    /// `crc_patch_deadline_miss` counter on [`super::FastLastScheduler`].
    ///
    /// Domain-wise this is an edge-ring event, but it lives here because
    /// the composite detects the miss at its poll-router Crc arm, where
    /// only the RxDma handle is borrowable — the [`super::EdgeDma`]
    /// instance is owned by the codec's edge-capture half and out of
    /// reach at that call site.
    fn record_edge_anchor_miss(&mut self);
}
