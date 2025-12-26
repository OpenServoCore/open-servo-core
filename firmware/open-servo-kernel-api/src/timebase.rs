//! Timebase / monotonic clock boundary.
//!
//! Many features can run purely on `TickCtx.dt_us`, but some require a notion
//! of "now" that is independent of tick domains:
//! - timeouts and windows (debounce durations, yield windows, etc.)
//! - telemetry timestamps for system identification
//! - correlating logs across domains
//!
//! The board provides a monotonic microsecond timebase. Wrapping is OK.
//!
//! ```rust,ignore
//! use open_servo_kernel_api::{Timebase, TimeStampUs};
//! use open_servo_kernel_api::units::MicroSecond;
//!
//! fn did_timeout(tb: &impl Timebase, start: TimeStampUs, timeout: MicroSecond) -> bool {
//!     tb.now_us().wrapping_since(start).0 >= timeout.0
//! }
//! ```

use crate::units::MicroSecond;

/// Monotonic timestamp in microseconds (wrapping is OK).
///
/// This is distinct from [`MicroSecond`], which represents a **duration**.
/// Keeping timestamps and durations separate avoids many bugs.
#[repr(transparent)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
pub struct TimeStampUs(pub u32);

impl TimeStampUs {
    #[inline]
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Wrapping difference (newer - older), returned as a duration.
    ///
    /// This is safe across wrap if elapsed intervals are "reasonably small"
    /// compared to the wrap period (~71 minutes for u32 microseconds).
    #[inline]
    pub fn wrapping_since(self, older: TimeStampUs) -> MicroSecond {
        MicroSecond(self.0.wrapping_sub(older.0))
    }
}

/// Board-provided monotonic time source.
///
/// The board can implement this using:
/// - a free-running hardware timer
/// - SysTick-derived microseconds
/// - a combined (ms tick + sub-tick) scheme
///
/// Wrapping is acceptable.
pub trait Timebase {
    fn now_us(&self) -> TimeStampUs;
}
