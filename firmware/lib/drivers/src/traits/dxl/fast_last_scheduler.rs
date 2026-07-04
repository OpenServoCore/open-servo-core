/// Long-horizon CMP scheduler for the Fast Last CRC fold pipeline.
///
/// Drives the periodic walks that classify edges, drain the parser, and
/// fold predecessor wire bytes into the running CRC during a Fast Sync /
/// Bulk Read predecessor window. The wire start itself is hardware-armed
/// through [`super::TxScheduler`] — the grid's only job is landing the
/// chain-CRC patch before the TX DMA's read cursor reaches the trailing
/// CRC slot.
///
/// The driver works entirely in absolute u32 deadlines (WireClock domain).
/// The chip-side provider applies these directly — no lifting, no
/// per-anchor caching of a separate scheduling-domain tick — because the
/// WireClock contract guarantees a u32 horizon wide enough to span any
/// supported Fast Sync / Bulk Read predecessor window.
pub trait FastLastScheduler {
    /// CMP-match → body fold-start latency, in scheduler ticks. Driver
    /// subtracts this from every grid anchor before handing it to
    /// `schedule` so the body's actual fold-start lands on the formula's
    /// intended wall-clock anchor.
    const FAST_LAST_ENTRY_TICKS: u16;

    /// Periodic-walk grid step, in predecessor wire bytes. Each fold body
    /// folds up to `BYTES_PER_INTERVAL` bytes of newly-classified residue
    /// before re-scheduling the next CMP one grid step ahead.
    const BYTES_PER_INTERVAL: u16;

    /// Schedule the next CMP at the absolute `deadline` (caller has already
    /// back-dated by `FAST_LAST_ENTRY_TICKS`). Idempotent on re-schedule.
    ///
    /// A CMP target that lands in the past — possible at low RDT + small
    /// predecessor counts where back-dating by ENTRY underflows — triggers
    /// the IRQ ASAP; the body's first run lands ENTRY ticks late but the
    /// grid step advances cleanly from there.
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
