/// Long-horizon CMP scheduler for the Fast successor CRC pipeline.
///
/// Drives the single checkpoint wake (plus starve retries) that reads the
/// predecessor window's trailing chain-CRC bytes and patches our block's
/// trailing slot during a Fast Sync / Bulk Read exchange. The wire start
/// itself is hardware-armed through [`super::TxScheduler`] — this
/// scheduler's only job is landing the chain-CRC patch before the TX
/// DMA's read cursor reaches the trailing CRC slot.
///
/// The driver works entirely in absolute u32 deadlines (WireClock domain).
/// The chip-side provider applies these directly — no lifting, no
/// per-anchor caching of a separate scheduling-domain tick — because the
/// WireClock contract guarantees a u32 horizon wide enough to span any
/// supported Fast Sync / Bulk Read predecessor window.
pub trait FastLastScheduler {
    /// Early bias applied to the checkpoint wake CMP, in scheduler ticks:
    /// the CMP targets `window_end − WAKE_LEAD_TICKS` so the body arrives
    /// BEFORE the predecessor's trailing CRC bytes and spins them in.
    /// Sized to worst-case CMP-vector entry latency plus margin — waking
    /// early costs a short spin; waking late eats directly into the
    /// patch-vs-fetch margin.
    const WAKE_LEAD_TICKS: u16;

    /// Run the wake body inline at the status-start observation when the
    /// slot's wire-start deadline is within this many ticks — an ISR-
    /// occupancy budget: the inline body spins at most until the
    /// predecessor window closes, and skips the CMP dispatch latency a
    /// short window can't afford.
    const INLINE_FOLD_HORIZON_TICKS: u16;

    /// Cache the wake body's starve bound (WireClock u32 domain).
    /// Subsequent `deadline_passed()` calls compare against it.
    fn set_busy_wait_deadline(&mut self, deadline: u32);

    /// True once the wall clock has passed the deadline staged via
    /// `set_busy_wait_deadline`. Polled by the wake body's spin — the
    /// checkpoint-is-late/silent-predecessor exit that hands the pipeline
    /// to a byte-stride retry instead of spinning past the window.
    fn deadline_passed(&self) -> bool;

    /// Schedule the next CMP at the absolute `deadline` (caller has already
    /// applied the early bias). Idempotent on re-schedule.
    ///
    /// A CMP target that lands in the past — a late observation of a
    /// short window — triggers the IRQ ASAP; the body runs one dispatch
    /// late and the pickup proceeds from whatever has published.
    fn schedule(&mut self, deadline: u32);

    /// True when the TX DMA channel's read cursor has reached the trailing
    /// CRC slot. Once true, any further `patch_crc` write into
    /// `tx_buf[len-CRC_BYTES..len]` ships too late — placeholder bytes are
    /// already on the wire. The terminal guard on the grid's completion
    /// body: the hardware kickoff fires in parallel with the fold, so a
    /// starved fold (silent predecessor) converges here once the TX
    /// drains.
    fn patch_window_expired(&self) -> bool;

    /// Drop any pending CMP and return to idle. Idempotent.
    fn cancel(&mut self);
}
