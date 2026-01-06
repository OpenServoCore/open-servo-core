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
//! use open_servo_hw::{Timebase, TimeStampUs};
//! use open_servo_units::MicroSecond;
//!
//! fn did_timeout(tb: &impl Timebase, start: TimeStampUs, timeout: MicroSecond) -> bool {
//!     tb.now_us().wrapping_since(start).0 >= timeout.0
//! }
//! ```

use open_servo_units::TimeStampUs;

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
