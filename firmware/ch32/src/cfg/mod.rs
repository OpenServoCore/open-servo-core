pub mod board_wiring;

pub use board_wiring::{
    AdcPins, BoardWiring, Calibration, CurrentSenseConfig, Divider, Duplex, DxlUart, MotorConfig,
    NtcCal, TxEn,
};

use osc_core::{BaudRate, ConfigDefaults};

use crate::control::Scales;
use crate::hal::clocks::PCLK_HZ;

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
            usart_brr: usart_brr(cfg.defaults.dxl_baud),
        }
    }
}

/// Round-to-nearest USART_BRR divisor at our fixed PCLK for the chip's six
/// supported DXL bauds. Each arm folds to a literal — RV32EC has no
/// hardware divide. Driver-side `Clock::brr` mirrors this for runtime baud
/// changes; the two stay parallel because the driver and the chip see the
/// same PCLK through `UsartBaud::CLOCK_HZ`.
const fn usart_brr(baud: BaudRate) -> u32 {
    const fn compute(baud_hz: u32) -> u32 {
        (PCLK_HZ + baud_hz / 2) / baud_hz
    }
    match baud {
        BaudRate::B9600 => const { compute(BaudRate::B9600.as_hz()) },
        BaudRate::B57600 => const { compute(BaudRate::B57600.as_hz()) },
        BaudRate::B115200 => const { compute(BaudRate::B115200.as_hz()) },
        BaudRate::B1000000 => const { compute(BaudRate::B1000000.as_hz()) },
        BaudRate::B2000000 => const { compute(BaudRate::B2000000.as_hz()) },
        BaudRate::B3000000 => const { compute(BaudRate::B3000000.as_hz()) },
    }
}
