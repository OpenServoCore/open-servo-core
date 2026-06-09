//! Bench-only instrumentation helpers.
//!
//! Bodies are gated behind `#[cfg(feature = "bench")]` so call sites stay
//! unconditional in hot paths and compile to nothing in production builds.

#[cfg(feature = "bench")]
use crate::drivers::output_pin::OutputPin;

/// Scope-marker pulse on the DBG instance. Compile-time no-op without
/// `--features bench`.
#[inline(always)]
pub fn dbg_pulse() {
    #[cfg(feature = "bench")]
    // SAFETY: see `OutputPin::dbg` — ISR-at-PFIC-HIGH access.
    unsafe {
        OutputPin::dbg().pulse();
    }
}
