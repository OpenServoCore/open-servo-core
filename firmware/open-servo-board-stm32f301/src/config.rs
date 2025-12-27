//! Board configuration and kernel config provider.
//!
//! Per CLAUDE.md: core defines schemas; board provides values.

use open_servo_kernel::{KernelConfig, PidGains};

/// System clock frequency (Hz).
pub const SYSCLK_HZ: u32 = 72_000_000;

/// APB1 clock frequency (Hz).
pub const PCLK1_HZ: u32 = 36_000_000;

/// APB2 clock frequency (Hz).
pub const PCLK2_HZ: u32 = 72_000_000;

/// PWM frequency (Hz).
pub const PWM_FREQ_HZ: u32 = 20_000;

/// PWM timer ARR value (center-aligned mode).
/// ARR = SYSCLK / (2 * PWM_FREQ) - 1 = 72MHz / 40kHz - 1 = 1799
/// But for center-aligned: ARR = SYSCLK / PWM_FREQ - 1 = 72MHz / 20kHz - 1 = 3599
pub const PWM_ARR: u16 = 3599;

/// Fast tick rate (Hz) - ADC trigger rate.
pub const FAST_TICK_HZ: u32 = 10_000;

/// Fast tick period (µs).
pub const FAST_DT_US: u32 = 100;

/// Medium tick decimation (fast ticks per medium tick).
pub const MEDIUM_DECIMATE: u32 = 10;

/// Slow tick decimation (medium ticks per slow tick).
pub const SLOW_DECIMATE: u32 = 5;

/// Kernel configuration for this board.
///
/// This is where tuning values live. Adjust PID gains, deadband, limits here.
pub fn kernel_config() -> KernelConfig {
    KernelConfig {
        pos_pid: PidGains {
            kp: 100,
            ki: 1,
            kd: 10,
        },
        hold_deadband_cdeg: 50,
        effort_limit_raw: 16000,
    }
}
