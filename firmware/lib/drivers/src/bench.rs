//! Bench-only probes: counter bodies exist under the `bench` feature; call
//! sites stay unconditional in hot paths and compile to nothing without it
//! (the closure is never called and the empty inline erases).
//!
//! Counters live in `no_mangle` statics so the debug link can dump them by
//! symbol address (`nm` the ELF) on a running chip -- no wire traffic, no
//! table space, and they survive test tails that re-center transport state.

/// Clock-trim counters: CAL decisions, and the resolver's per-wake bound.
///
/// Wire format for the debug-link dump: 7 little-endian 32-bit words in
/// field order, 28 bytes; the four `tw_*` fields are signed (`struct`
/// format `<3I4i`).
#[repr(C)]
pub struct TrimProbe {
    /// Break services whose resolver drive hit its per-wake frame bound:
    /// the ladder lagged the wire by more than `FRAMES_PER_WAKE` frames.
    pub bound_hits: u32,
    /// Completed CAL trains drained by `poll_clock_trim`.
    pub poll_cal: u32,
    /// `TrimLoop` decisions and the latest measurement, effect estimate,
    /// applied steps, and running total.
    pub windows: u32,
    pub tw_ppm: i32,
    pub tw_effect: i32,
    pub tw_applied: i32,
    pub tw_total: i32,
}

impl TrimProbe {
    pub const ZERO: Self = Self {
        bound_hits: 0,
        poll_cal: 0,
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

/// Run `f` over the trim probe -- a no-op without the `bench` feature.
#[inline(always)]
#[allow(unused_variables)]
pub fn trim_probe(f: impl FnOnce(&mut TrimProbe)) {
    // SAFETY: single-hart; every writer runs at the one transport priority
    // (the bus level), so accesses never interleave. The debug link only reads.
    #[cfg(feature = "bench")]
    unsafe {
        f(&mut *core::ptr::addr_of_mut!(TRIM_PROBE));
    }
}

#[cfg(feature = "bench")]
static mut TEL_BANKS: u32 = 0;

/// The TEL encoder banked a batch (kernel tick, the only writer).
#[inline(always)]
pub fn tel_banked() {
    // SAFETY: single writer; readers take one word.
    #[cfg(feature = "bench")]
    unsafe {
        let b = &raw mut TEL_BANKS;
        b.write_volatile(b.read_volatile().wrapping_add(1));
    }
}

/// Batches banked so far; 0 without the `bench` feature.
#[inline(always)]
pub fn tel_banks() -> u32 {
    #[cfg(feature = "bench")]
    // SAFETY: one-word volatile read.
    unsafe {
        (&raw const TEL_BANKS).read_volatile()
    }
    #[cfg(not(feature = "bench"))]
    0
}
