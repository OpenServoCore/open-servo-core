//! Driver-owned interfaces. Drivers declare what they need from their
//! environment; adapters in `crate::adapters` implement these over real HAL
//! peripherals (production) or recording mocks (tests).

use crate::types::Level;

/// Drive a single digital output. Owned by the adapter; the driver holds
/// one `P: DigitalOut` and calls `set` to change the wire state.
pub trait DigitalOut {
    fn set(&mut self, level: Level);
}

/// Free-running monotonic tick counter, used by drivers that schedule by
/// elapsed time. `TICKS_PER_US` describes the rate so the driver can
/// convert µs ↔ ticks without importing chip constants.
pub trait Monotonic {
    const TICKS_PER_US: u32;
    fn ticks(&self) -> u32;
}

/// Single-channel USART baud-rate control. Drivers compute BRR off-line
/// (chip-specific math) and hand the value here; the adapter writes it to
/// the correct USART instance.
pub trait UsartBaud {
    fn set_baud(&mut self, brr: u32);
}

/// Trim a clock by integer steps. Constants describe the physics of the
/// trim mechanism so drivers can compute deadbands, thresholds, etc.
/// without importing chip constants.
pub trait ClockTrim {
    /// Inclusive lower / upper bounds for the trim delta.
    const DELTA_MIN: i8;
    const DELTA_MAX: i8;
    /// Base frequency of the trimmed clock, in Hz.
    const HZ: u32;
    /// Frequency shift per trim step, in Hz.
    const STEP_HZ: u32;
    fn apply_delta(&mut self, delta: i8);
}
