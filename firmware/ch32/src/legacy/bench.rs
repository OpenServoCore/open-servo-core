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
/// `--features bench`.
#[inline(always)]
pub fn dbg_pulse() {
    #[cfg(feature = "bench")]
    // SAFETY: see `Drivers::dbg` — ISR-at-PFIC-HIGH access.
    unsafe {
        let dbg = Drivers::dbg();
        dbg.set(Level::High);
        dbg.set(Level::Low);
    }
}
