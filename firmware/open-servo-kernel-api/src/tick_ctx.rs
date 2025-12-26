use crate::tick::Tick;
use crate::{FaultSink, TelemetrySink, TickDomain, TimeStampUs};

/// Per-tick execution context.
///
/// This is what nodes receive during `tick()` so they can:
/// - access timing (`tick.dt`, `tick.domain`)
/// - access monotonic time (`now`)
/// - raise faults
/// - emit telemetry (system id)
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
}
