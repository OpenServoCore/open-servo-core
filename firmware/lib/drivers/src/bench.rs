//! Bench-only probes: counter bodies exist under the `bench` feature; call
//! sites stay unconditional in hot paths and compile to nothing without it
//! (the closure is never called and the empty inline erases).
//!
//! Counters live in `no_mangle` statics so the debug link can dump them by
//! symbol address (`nm` the ELF) on a running chip — no wire traffic, no
//! table space, and they survive test tails that re-center transport state.

/// Trim chain-pair pipeline counters (task #28): where do BURST-food pairs
/// die between the break stamp and a window verdict? One field per exit of
/// [`ServoBus::on_drift_break`]'s decision ladder, in ladder order.
#[repr(C)]
pub struct TrimProbe {
    /// Drift stamps taken (fresh break wakes with a break byte newest).
    pub stamps: u32,
    /// Stamp with no predecessor (first after boot/restart) — no pair.
    pub no_prev: u32,
    /// Pair rejected: NO verified frame between the stamps (a coalesced or
    /// spurious service — the fault-contract starvation class).
    pub span_none: u32,
    /// Pair rejected: MORE than one verified frame between the stamps
    /// (coalesced break service under zero-gap bursts).
    pub span_many: u32,
    /// Pair rejected: exactly one verified frame, but a solicited shape (a
    /// reply's turnaround rides the responder's clock).
    pub unsilent: u32,
    /// Pair rejected: ring span != the verified footprint (something else
    /// ringed — status, garble, echo).
    pub inexact: u32,
    /// Pair rejected by the 1/16 span gate (a real inter-burst pause).
    pub gated: u32,
    /// Pair accepted into the seam baseline.
    pub base_pairs: u32,
    /// Pair accepted into a drift window (baseline established).
    pub win_pairs: u32,
    /// Window verdicts handed to the trim loop.
    pub verdicts: u32,
    // --- verdict-content layer ---
    /// Latest drift verdict's raw window sums at handoff.
    pub verdict_err: i32,
    pub verdict_span: u32,
    /// `poll_clock_trim` consumptions by source.
    pub poll_cal: u32,
    pub poll_drift: u32,
    /// Drift polls discarded by the ±8k ppm sanity band.
    pub sanity_drop: u32,
    /// Latest drift poll's computed ppm (pre-sanity).
    pub poll_ppm: i32,
    /// `TrimLoop::on_window` invocations (both sources) and its latest
    /// measurement, effect estimate, applied steps, and running total.
    pub windows: u32,
    pub tw_ppm: i32,
    pub tw_effect: i32,
    pub tw_applied: i32,
    pub tw_total: i32,
}

impl TrimProbe {
    pub const ZERO: Self = Self {
        stamps: 0,
        no_prev: 0,
        span_none: 0,
        span_many: 0,
        unsilent: 0,
        inexact: 0,
        gated: 0,
        base_pairs: 0,
        win_pairs: 0,
        verdicts: 0,
        verdict_err: 0,
        verdict_span: 0,
        poll_cal: 0,
        poll_drift: 0,
        sanity_drop: 0,
        poll_ppm: 0,
        windows: 0,
        tw_ppm: 0,
        tw_effect: 0,
        tw_applied: 0,
        tw_total: 0,
    };
}

#[cfg(feature = "bench")]
#[unsafe(no_mangle)]
pub static mut TRIM_PROBE: TrimProbe = TrimProbe::ZERO;

/// Run `f` over the trim probe — a no-op without the `bench` feature.
#[inline(always)]
#[allow(unused_variables)]
pub fn trim_probe(f: impl FnOnce(&mut TrimProbe)) {
    // SAFETY: single-hart; every writer runs at the one transport priority
    // (HIGH), so accesses never interleave. The debug link only reads.
    #[cfg(feature = "bench")]
    unsafe {
        f(&mut *core::ptr::addr_of_mut!(TRIM_PROBE));
    }
}
