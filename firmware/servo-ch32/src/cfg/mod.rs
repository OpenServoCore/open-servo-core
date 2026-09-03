pub mod board_wiring;
pub mod chip;

#[cfg(not(feature = "half-duplex"))]
pub use board_wiring::BusWiring;
pub use board_wiring::{AdcPins, BoardWiring, Calibration, CurrentSenseConfig, Divider, DrvEn};
pub use chip::{AnalogChannel, DigitalPin};

use osc_servo_core::estimator::bemf::RECIP_ARR_SHIFT;
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::{ConfigDefaults, KernelTiming};

use crate::providers::usart_baud;

#[derive(Copy, Clone)]
pub struct BoardConfig {
    pub wiring: BoardWiring,
    pub calibration: Calibration,
    pub defaults: ConfigDefaults,
    /// Identity the board stamps into the RO block (protocol sec 5.4); the
    /// firmware version comes from `osc_servo_core::FIRMWARE_VERSION`, not here.
    pub model: u16,
    pub hw_rev: u8,
}

/// Boot-time-derived values that the `run!` macro folds at compile time so the
/// linker can drop __udivdi3 / __udivsi3 / __umodsi3 entirely.
#[derive(Copy, Clone)]
pub struct Precomputed {
    pub pwm_psc: u16,
    pub pwm_arr: u16,
    pub usart_brr: u32,
    pub kernel_timing: KernelTiming,
}

impl Precomputed {
    pub const fn compute(cfg: &BoardConfig) -> Self {
        let (pwm_psc, pwm_arr) = crate::hal::timer::pwm_dividers_from_hz(chip::MOTOR_PWM_FREQ_HZ);
        // Const-eval quotients per the KernelTiming field docs; the divides
        // fold at compile time so no soft-div symbol ever links.
        let med_hz = chip::MOTOR_PWM_FREQ_HZ / DECIM_MED as u32;
        Self {
            pwm_psc,
            pwm_arr,
            usart_brr: usart_baud::brr_for(cfg.defaults.baud),
            kernel_timing: KernelTiming {
                pwm_arr,
                recip_arr_q24: (1u32 << RECIP_ARR_SHIFT) / pwm_arr as u32,
                tick_hz: chip::MOTOR_PWM_FREQ_HZ as u16,
                dt_med_q32: ((1u64 << 32) / med_hz as u64) as u32,
                med_ticks_per_ms_q16: ((med_hz as u64 * 65536) / 1000) as u32,
            },
        }
    }
}
