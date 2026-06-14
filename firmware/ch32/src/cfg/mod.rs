pub mod board_wiring;

pub use board_wiring::{
    AdcPins, BoardWiring, Calibration, CurrentSenseConfig, Divider, DxlUart, MotorConfig, NtcCal,
    TxEn,
};

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
        let (pwm_psc, pwm_arr) =
            crate::hal::timer::pwm_dividers_from_hz(cfg.wiring.motor.pwm_freq_hz);
        let gain_factor = cfg.wiring.current_sense.opa.gain.factor();
        Self {
            scales: Scales::new(&cfg.calibration, gain_factor),
            pwm_psc,
            pwm_arr,
            usart_brr: usart_baud::brr_for(cfg.defaults.dxl_baud),
        }
    }
}
