//! Board configuration constants.
//!
//! Clock frequencies and timing constants for STM32F301.
//! Board-specific configuration (capabilities, tuning) is in board.rs via BoardConfig trait.

/// System clock frequency (Hz).
#[cfg(feature = "osctl")]
pub const SYSCLK_HZ: u32 = 72_000_000;

/// APB2 clock frequency (Hz).
pub const PCLK2_HZ: u32 = 72_000_000;

/// PWM timer ARR value (center-aligned mode).
/// ARR = SYSCLK / PWM_FREQ - 1 = 72MHz / 20kHz - 1 = 3599
pub const PWM_ARR: u16 = 3599;

/// Fast tick period (µs).
pub const FAST_DT_US: u32 = 100;
