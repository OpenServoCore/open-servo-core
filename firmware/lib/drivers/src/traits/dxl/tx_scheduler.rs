/// What kind of TX this is — passes through `schedule` so the provider can
/// apply variant-specific bias if needed. Carrying the tag means future
/// bench-tuning doesn't reshape the trait if Fast-Last turns out to need a
/// different setup margin from Plain (e.g. catchup-completion slack).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SendKind {
    /// Plain Status reply or Fast Sync/Bulk Read non-Last slot — no chain
    /// participation.
    Plain,
    /// Fast Sync/Bulk Read Last slot — chain-CRC tail folder runs alongside.
    FastLast,
}

/// Schedule a TX at a protocol-prescribed wire deadline. The driver hands
/// an absolute `deadline` in the WireClock u32 domain; the chip-side
/// provider applies its own bias compensation (PFIC + ISR-entry latency,
/// TX_EN OC setup, wrap guard, time-remaining decision tree) without
/// further lifting — the driver already computed the absolute deadline
/// from a `WireClock::now()`-domain anchor.
///
/// Sibling of [`super::TxBus`] — `TxScheduler` decides *when* the chip
/// transmits; `TxBus` does the wire driving when that moment lands (or
/// when a chain k > 0 reply fires sequence-driven off a SkipComplete
/// event).
pub trait TxScheduler {
    /// Tick rate of the chip-side TX-start timer, ticks per µs. Driver uses
    /// this to convert protocol delay (µs / Q8.8 µs) to ticks before adding
    /// to the wire-clock anchor.
    const TICKS_PER_US: u16;

    /// Schedule a wire TX at the absolute `deadline` (WireClock u32 domain).
    /// Provider applies chip-specific bias + decision tree directly — this
    /// method's contract is "the first wire bit of `byte_count` bytes lands
    /// at approximately `deadline`." Idempotent on re-schedule (overwrites
    /// any prior schedule).
    ///
    /// - `kind == SendKind::Plain` — arm per the chip-side decision tree.
    /// - `kind == SendKind::FastLast` — stash; the composite triggers the
    ///   commit via [`Self::commit_pending`] when the FastLast walk reaches
    ///   its final anchor (chain-CRC catchup co-owns the long-horizon timer
    ///   during the predecessor window, so the scheduler defers).
    ///
    /// `byte_count` is the size of the encoded packet sitting in the
    /// driver-owned TX buffer (codec's `tx_len`); the provider hands it to
    /// the chip-side DMA channel as the transfer count.
    fn schedule(&mut self, deadline: u32, byte_count: u16, kind: SendKind);

    /// Composite signals "the FastLast walk reached its final anchor — commit
    /// the stashed schedule now." Provider runs its time-remaining decision
    /// against the stashed deadline; by construction the caller invokes this
    /// within ~1 byte_time of the wire deadline, so the decision lands in
    /// the direct-arm or software-fire branch. No-op when nothing stashed;
    /// idempotent.
    fn commit_pending(&mut self);

    /// Drop any pending TX schedule. Idempotent. The wire driver itself is
    /// owned by [`super::TxBus`]; canceling the schedule alone leaves the
    /// bus in whatever state it was in (typically idle — between schedule
    /// and the deadline ISR no wire activity is in flight).
    fn cancel(&mut self);

    /// Long-horizon timer match fired — provider returns `true` if it owned
    /// the deadline (re-runs its decision tree internally) and `false` if
    /// the match belongs to another scheduling consumer (the FastLast walk
    /// grid co-owns the long-horizon timer during a chain-CRC catchup
    /// window). The composite uses the return value to demux which sub-
    /// driver consumes the match.
    fn on_schedule_due(&mut self) -> bool;
}
