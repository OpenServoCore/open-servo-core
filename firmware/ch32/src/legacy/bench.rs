//! Bench-only instrumentation helpers.
//!
//! Bodies are gated behind `#[cfg(feature = "bench")]` so call sites stay
//! unconditional in hot paths and compile to nothing in production builds.

#[cfg(feature = "bench")]
use osc_drivers::Level;
#[cfg(feature = "bench")]
use osc_drivers::traits::DigitalOut;

#[cfg(feature = "bench")]
use crate::runtime::Drivers;

/// Scope-marker pulse on the DBG instance. Compile-time no-op without
/// `--features bench`. Currently call-site-less — M2 (#33) drained the
/// legacy ISR pulses; per [[bench_feature]] the helper body stays for
/// future re-wiring instead of being deleted.
#[inline(always)]
#[allow(dead_code)]
pub fn dbg_pulse() {
    #[cfg(feature = "bench")]
    // SAFETY: see `Drivers::dbg` — ISR-at-PFIC-HIGH access.
    unsafe {
        let dbg = Drivers::dbg();
        dbg.set(Level::High);
        dbg.set(Level::Low);
    }
}
