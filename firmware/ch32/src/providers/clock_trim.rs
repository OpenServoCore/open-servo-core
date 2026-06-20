//! Clock-trim provider — binds `ClockTrim` to the HSI trim register.

use osc_drivers::traits::dxl;

use crate::hal::rcc;
use crate::hal::rcc::CLOCK_TRIM_PPM_PER_STEP;

/// Production binding to HSITRIM. The driver hands an absolute correction
/// in ppm relative to factory cal; the provider quantizes to the nearest
/// HSI trim step (~2500 ppm/step on V006) and writes the register.
pub struct ClockTrim;

impl dxl::ClockTrim for ClockTrim {
    const STEP_PPM: u32 = CLOCK_TRIM_PPM_PER_STEP;
    const ENVELOPE_PPM: (i32, i32) = (
        rcc::CLOCK_TRIM_DELTA_MIN as i32 * CLOCK_TRIM_PPM_PER_STEP as i32,
        rcc::CLOCK_TRIM_DELTA_MAX as i32 * CLOCK_TRIM_PPM_PER_STEP as i32,
    );

    #[inline(always)]
    fn apply_ppm(&mut self, ppm: i32) {
        // Round to nearest step away from zero so requests below half a
        // step still snap to zero; requests above half snap to ±1 step.
        let step_ppm = CLOCK_TRIM_PPM_PER_STEP as i32;
        let half = step_ppm / 2;
        let rounded = if ppm >= 0 { ppm + half } else { ppm - half };
        let step = (rounded / step_ppm).clamp(
            rcc::CLOCK_TRIM_DELTA_MIN as i32,
            rcc::CLOCK_TRIM_DELTA_MAX as i32,
        ) as i8;
        rcc::apply_clock_trim_delta(step);
    }
}
