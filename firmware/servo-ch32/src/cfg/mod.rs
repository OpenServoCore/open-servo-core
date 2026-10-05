pub mod board_wiring;
pub mod chip;

pub use board_wiring::{
    AdcPins, BoardWiring, Calibration, CurrentSenseConfig, Divider, DrvEn, Ntc,
};
pub use chip::{AnalogChannel, DigitalPin};

use osc_servo_core::estimator::bemf::RECIP_ARR_SHIFT;
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::regions::config::{BURST_MAX_MV, DEFAULT_V_UNDERVOLT_MV, vmotor_counts};
use osc_servo_core::{ConfigDefaults, CurrentDefaults, KernelTiming};

use crate::providers::{break_wake, usart_baud};

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
    pub break_reload: u16,
    pub kernel_timing: KernelTiming,
    pub current_defaults: CurrentDefaults,
    pub burst_v_max_counts: u16,
    pub v_undervolt_counts: u16,
}

impl Precomputed {
    pub const fn compute(cfg: &BoardConfig) -> Self {
        let (pwm_psc, pwm_arr) = crate::hal::timer::pwm_dividers_from_hz(chip::MOTOR_PWM_FREQ_HZ);
        // Const-eval quotients per the KernelTiming field docs; the divides
        // fold at compile time so no soft-div symbol ever links.
        let med_hz = chip::MOTOR_PWM_FREQ_HZ / DECIM_MED as u32;
        let (rail, term) = (
            &cfg.calibration.vbus_divider,
            &cfg.calibration.vmotor_divider,
        );
        let vbus_scale_q15 = ((rail.top_ohm + rail.bot_ohm) as u64 * term.bot_ohm as u64 * 32768
            / (rail.bot_ohm as u64 * (term.top_ohm + term.bot_ohm) as u64))
            as u32;
        Self {
            pwm_psc,
            pwm_arr,
            usart_brr: usart_baud::brr_for(cfg.defaults.baud),
            break_reload: break_wake::reload_for(cfg.defaults.baud),
            kernel_timing: KernelTiming {
                pwm_arr,
                recip_arr_q24: (1u32 << RECIP_ARR_SHIFT) / pwm_arr as u32,
                tick_hz: chip::MOTOR_PWM_FREQ_HZ as u16,
                dt_med_q32: ((1u64 << 32) / med_hz as u64) as u32,
                med_ticks_per_ms_q16: ((med_hz as u64 * 65536) / 1000) as u32,
                vbus_scale_q15,
                bias_brake_min_ticks: cfg.calibration.bias_brake_min_ticks,
                v_trough_min_ticks: cfg.calibration.v_trough_window_min_ticks,
                bemf_min_ticks: cfg.calibration.bemf_window_min_ticks,
                i_settle_gain: cfg.calibration.i_settle_gain,
            },
            current_defaults: CurrentDefaults::from_sense(
                cfg.calibration.shunt_r_mohm,
                cfg.wiring.current_sense.gain_milli,
                cfg.calibration.vdd_mv,
            ),
            burst_v_max_counts: vmotor_counts(
                BURST_MAX_MV,
                term.top_ohm,
                term.bot_ohm,
                cfg.calibration.vdd_mv,
            ),
            v_undervolt_counts: vmotor_counts(
                DEFAULT_V_UNDERVOLT_MV,
                term.top_ohm,
                term.bot_ohm,
                cfg.calibration.vdd_mv,
            ),
        }
    }
}
