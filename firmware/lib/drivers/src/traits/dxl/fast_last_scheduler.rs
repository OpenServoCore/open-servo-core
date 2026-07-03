/// Long-horizon CMP scheduler for the Fast Last CRC fold pipeline.
///
/// Drives the periodic walks that classify edges, drain the parser, and
/// fold predecessor wire bytes into the running CRC during a Fast Sync /
/// Bulk Read predecessor window.
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

    /// Pre-start fold residue cap, in predecessor wire bytes. The final
    /// busy-wait exits at `deadline = t_prior_end − GUARD_BYTES × byte_ticks`,
    /// leaving up to `GUARD_BYTES` predecessor bytes for the TX-start
    /// body's tail to fold inline. At 3M GUARD=1 keeps `patch_crc` ahead
    /// of CH4's DMA-prefetch on byte[n − 2].
    const GUARD_BYTES: u16;

    /// Cache the busy-wait exit `deadline` (WireClock u32 domain).
    /// Subsequent `deadline_passed()` calls compare against it.
    fn set_busy_wait_deadline(&mut self, deadline: u32);

    /// Schedule the next CMP at the absolute `deadline` (caller has already
    /// back-dated by `FAST_LAST_ENTRY_TICKS`). Idempotent on re-schedule.
    ///
    /// A CMP target that lands in the past — possible at low RDT + small
    /// predecessor counts where back-dating by ENTRY underflows — triggers
    /// the IRQ ASAP; the body's first run lands ENTRY ticks late but the
    /// grid step advances cleanly from there.
    fn schedule(&mut self, deadline: u32);

    /// True once the wall clock has passed the deadline staged via
    /// `set_busy_wait_deadline`. Polled by the final-step busy-wait.
    fn deadline_passed(&self) -> bool;

    /// True when the TX DMA channel's read cursor has reached the trailing
    /// CRC slot. Once true, any further `patch_crc` write into
    /// `tx_buf[len-CRC_BYTES..len]` ships too late — placeholder bytes are
    /// already on the wire. Polled by `on_tx_start`'s post-start fold loop
    /// alongside the predecessor-byte-plateau backstop; whichever trips
    /// first ends the loop.
    fn patch_window_expired(&self) -> bool;

    /// Bump the `crc_patch_deadline_miss` telemetry counter. Called once
    /// per `on_tx_start` exit-via-miss — both the patch-window-expired
    /// route and the plateau (predecessor-bytes-stalled) route feed the
    /// same counter; both ship a placeholder CRC observable to the host as
    /// a bad-CRC packet.
    fn record_patch_deadline_miss(&mut self);

    /// Drop any pending CMP and return to idle. Idempotent.
    fn cancel(&mut self);
}
