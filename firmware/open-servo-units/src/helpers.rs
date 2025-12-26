//! Helper functions for unit conversions.

/// Integer multiplication and division with rounding.
#[inline]
pub fn mul_div_i32(val: i32, num: i32, den: i32) -> i32 {
    (val * num + den / 2) / den
}
