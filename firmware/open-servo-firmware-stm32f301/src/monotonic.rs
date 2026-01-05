//! Free-running monotonic microsecond counter via TIM2.
//!
//! TIM2 is configured as a 32-bit free-running counter at 1MHz (1µs resolution).
//! This provides ~4295 seconds before wrap, sufficient for most timing needs.

use open_servo_kernel_api::TimeStampUs;
use stm32f3::stm32f301::TIM2;

/// Read the current monotonic time in microseconds.
///
/// This reads TIM2.CNT directly. The counter wraps after ~4295 seconds.
/// For dt calculations, use wrapping subtraction.
#[inline]
pub fn now_us() -> TimeStampUs {
    // SAFETY: We only read the counter register, which is safe from any context.
    let cnt = unsafe { (*TIM2::ptr()).cnt.read().bits() };
    TimeStampUs(cnt)
}
