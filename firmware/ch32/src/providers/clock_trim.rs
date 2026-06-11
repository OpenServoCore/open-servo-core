//! Clock-trim provider — binds `ClockTrim` to the HSI trim register.

use osc_drivers::traits;

use crate::hal::clocks::{HSI_HZ, HSI_TRIM_STEP_HZ};
use crate::hal::rcc;

/// Production binding to HSITRIM. Delta is added on top of the factory
/// default trim.
pub struct ClockTrim;

impl traits::ClockTrim for ClockTrim {
    const DELTA_MIN: i8 = rcc::CLOCK_TRIM_DELTA_MIN;
    const DELTA_MAX: i8 = rcc::CLOCK_TRIM_DELTA_MAX;
    const HZ: u32 = HSI_HZ;
    const STEP_HZ: u32 = HSI_TRIM_STEP_HZ;

    #[inline(always)]
    fn apply_delta(&mut self, delta: i8) {
        rcc::apply_clock_trim_delta(delta);
    }
}
