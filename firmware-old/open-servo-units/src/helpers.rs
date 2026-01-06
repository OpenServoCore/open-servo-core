//! Helper functions for unit conversions.

/// Integer division with rounding (unsigned).
#[inline]
pub const fn div_round_u32(num: u32, den: u32) -> u32 {
    (num + den / 2) / den
}

/// Integer division with rounding (signed).
#[inline]
pub const fn div_round_i32(num: i32, den: i32) -> i32 {
    if (num >= 0) == (den >= 0) {
        (num + den / 2) / den
    } else {
        (num - den / 2) / den
    }
}

/// Integer multiplication and division with rounding (unsigned).
#[inline]
pub const fn mul_div_round_u32(val: u32, num: u32, den: u32) -> u32 {
    div_round_u32(val * num, den)
}

/// Integer multiplication and division with rounding (signed).
#[inline]
pub const fn mul_div_round_i32(val: i32, num: i32, den: i32) -> i32 {
    div_round_i32(val * num, den)
}

/// Integer multiplication and division with rounding (i64 intermediate).
#[inline]
pub const fn mul_div_round_i64(val: i64, num: i64, den: i64) -> i64 {
    let product = val * num;
    if (product >= 0) == (den >= 0) {
        (product + den / 2) / den
    } else {
        (product - den / 2) / den
    }
}

/// Integer multiplication and division with rounding (u64 intermediate).
#[inline]
pub const fn mul_div_round_u64(val: u64, num: u64, den: u64) -> u64 {
    (val * num + den / 2) / den
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_div_round_u32() {
        assert_eq!(div_round_u32(10, 3), 3); // 3.33 -> 3
        assert_eq!(div_round_u32(11, 3), 4); // 3.67 -> 4
        assert_eq!(div_round_u32(9, 3), 3); // exact
        assert_eq!(div_round_u32(1_000_000, 60), 16667); // 16666.67 -> 16667
    }

    #[test]
    fn test_div_round_i32() {
        // Positive / positive
        assert_eq!(div_round_i32(10, 3), 3);
        assert_eq!(div_round_i32(11, 3), 4);

        // Negative / positive (rounds toward zero)
        assert_eq!(div_round_i32(-10, 3), -3);
        assert_eq!(div_round_i32(-11, 3), -4);

        // Positive / negative
        assert_eq!(div_round_i32(10, -3), -3);
        assert_eq!(div_round_i32(11, -3), -4);
    }

    #[test]
    fn test_mul_div_round_u32() {
        assert_eq!(mul_div_round_u32(100, 3, 10), 30); // exact
        assert_eq!(mul_div_round_u32(100, 1, 3), 33); // 33.33 -> 33
        assert_eq!(mul_div_round_u32(100, 2, 3), 67); // 66.67 -> 67
    }

    #[test]
    fn test_mul_div_round_i32() {
        assert_eq!(mul_div_round_i32(100, 3, 10), 30); // exact
        assert_eq!(mul_div_round_i32(100, 1, 3), 33); // 33.33 -> 33
        assert_eq!(mul_div_round_i32(100, 2, 3), 67); // 66.67 -> 67
        assert_eq!(mul_div_round_i32(-100, 2, 3), -67); // -66.67 -> -67
    }
}
