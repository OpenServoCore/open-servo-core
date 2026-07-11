pub mod board_wiring;
pub mod chip;

#[cfg(feature = "wire-buffered")]
pub use board_wiring::BusWiring;
pub use board_wiring::{
    AdcPins, BoardWiring, Calibration, CurrentSenseConfig, Divider, DrvEn, NtcCal,
};
pub use chip::{AnalogChannel, DigitalPin};

use osc_core::ConfigDefaults;

use crate::control::Scales;
use crate::providers::usart_baud;

#[derive(Copy, Clone)]
pub struct BoardConfig {
    pub wiring: BoardWiring,
    pub calibration: Calibration,
    pub defaults: ConfigDefaults,
}

/// Boot-time-derived values that the `run!` macro folds at compile time so the
/// linker can drop __udivdi3 / __udivsi3 / __umodsi3 entirely.
#[derive(Copy, Clone)]
pub struct Precomputed {
    pub scales: Scales,
    pub pwm_psc: u16,
    pub pwm_arr: u16,
    pub usart_brr: u32,
}

impl Precomputed {
    pub const fn compute(cfg: &BoardConfig) -> Self {
        let (pwm_psc, pwm_arr) = crate::hal::timer::pwm_dividers_from_hz(chip::MOTOR_PWM_FREQ_HZ);
        let gain_factor = cfg.wiring.current_sense.gain.factor();
        Self {
            scales: Scales::new(&cfg.calibration, gain_factor),
            pwm_psc,
            pwm_arr,
            usart_brr: usart_baud::brr_for(cfg.defaults.baud),
        }
    }
}
