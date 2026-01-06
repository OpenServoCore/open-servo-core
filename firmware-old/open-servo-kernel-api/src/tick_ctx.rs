use crate::tick::Tick;
use crate::{FaultSink, TelemetrySink, TickDomain};
use open_servo_units::TimeStampUs;

/// Per-tick execution context.
///
/// This is what nodes receive during `tick()` so they can:
/// - access timing (`tick.dt`, `tick.domain`, `tick.seq`)
/// - access monotonic time (`now`)
/// - raise faults
/// - emit telemetry (system id)
///
/// # Timing Semantics
///
/// - `tick.dt`: delta time since *this domain's* last tick
/// - `tick.seq`: per-domain sequence counter (wrapping, monotonic within domain)
/// - `now`: global monotonic timestamp (cross-domain correlation)
///
/// # Multi-Timer Scheduling
///
/// Each domain may be driven by a different hardware timer. The `seq` counter
/// is per-domain, allowing detection of missed ticks or out-of-order processing.
/// Boards provide `dt` based on actual elapsed time, not assumed cadence.
///
/// # Non-Reentrancy Contract
///
/// `TickCtx` is NOT safe for concurrent access. Boards must ensure:
/// - Only one tick executes at a time per domain
/// - ISRs do not preempt an in-progress tick of the same domain
///
/// This contract allows implementations to avoid atomics (CH32V006 compatible).
pub struct TickCtx<'a, F, T>
where
    F: FaultSink + ?Sized,
    T: TelemetrySink + ?Sized,
{
    pub tick: Tick,
    pub now: TimeStampUs,
    pub faults: &'a mut F,
    pub telem: &'a mut T,
}

impl<'a, F, T> TickCtx<'a, F, T>
where
    F: FaultSink + ?Sized,
    T: TelemetrySink + ?Sized,
{
    /// Construct a new tick context.
    #[inline]
    pub fn new(tick: Tick, now: TimeStampUs, faults: &'a mut F, telem: &'a mut T) -> Self {
        Self {
            tick,
            now,
            faults,
            telem,
        }
    }

    /// Validate the tick and assert expected domain.
    ///
    /// This is debug-only; it compiles out in release builds.
    #[inline]
    pub fn debug_assert_domain(&self, expected: TickDomain) {
        self.tick.debug_assert_valid();

        debug_assert!(
            self.tick.domain == expected,
            "TickCtx.domain mismatch: expected {:?}, got {:?}",
            expected,
            self.tick.domain
        );
    }

    /// Delta time in microseconds since last tick of this domain.
    ///
    /// This is a convenience accessor for `self.tick.dt.0`.
    #[inline]
    pub fn dt_us(&self) -> u32 {
        self.tick.dt.0
    }

    /// Per-domain monotonic sequence counter.
    ///
    /// This is a convenience accessor for `self.tick.seq`.
    /// The sequence is monotonically increasing within each domain
    /// and wraps at `u32::MAX`.
    #[inline]
    pub fn domain_seq(&self) -> u32 {
        self.tick.seq
    }
}
