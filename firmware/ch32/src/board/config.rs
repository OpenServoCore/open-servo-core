use osc_core::ConfigDefaults;

use crate::hal::{
    Pin, Tim1Mapping, Tim2Mapping, UsartMapping, adc, gpio::Level, gpio::Pull, opa, timer,
};

pub enum Duplex {
    Full,
    Half,
}

pub struct TxEn {
    pub pin: Pin,
    /// Level driven to enable TX; inverse drives RX.
    pub tx_level: Level,
}

pub struct DxlBus {
    pub usart: UsartMapping,
    pub duplex: Duplex,
    pub rx_pull: Pull,
    pub tx_en: Option<TxEn>,
}

pub struct MotorConfig {
    pub tim1: Tim1Mapping,
    pub in1: timer::Channel,
    pub in2: timer::Channel,
    pub drv_en: Pin,
    pub pwm_freq_hz: u32,
    pub polarity: timer::Polarity,
}

pub struct CurrentSenseConfig {
    pub opa: opa::Config,
    pub adc_sample_time: adc::SampleTime,
}

pub struct Sensors {
    pub pos: adc::Input,
    pub ntc: adc::Input,
    pub vbus: adc::Input,
    pub vmotor: (adc::Input, adc::Input),
}

/// `V_adc = V_in · bot_ohm / (top_ohm + bot_ohm)`.
pub struct Divider {
    pub top_ohm: u32,
    pub bot_ohm: u32,
}

/// β-model NTC params: `R_ntc(T) = r0_ohm · exp(beta · (1/T − 1/T₀))`.
pub struct NtcCal {
    pub beta: u16,
    pub r0_ohm: u32,
    /// T₀ in centi-°C (matches `osc_units::CentiCelsius`).
    pub t0_cc: i16,
    pub bias_r_ohm: u32,
}

/// Schematic-derived constants identical across every unit of a PCB design.
pub struct Calibration {
    pub shunt_r_mohm: u16,
    pub vbus_divider: Divider,
    pub vmotor_divider: Divider,
    pub ntc: NtcCal,
}

/// Schematic-fixed wiring; consumed during `Ch32Board::new` and not retained.
pub struct BoardWiring {
    pub stat_led: Pin,
    /// Scope/probe pad; toggled once per DMA-TC ISR.
    pub dbg: Pin,
    pub tim2_remap: Tim2Mapping,
    pub motor: MotorConfig,
    pub current_sense: CurrentSenseConfig,
    pub sensors: Sensors,
    pub dxl: DxlBus,
}

pub struct BoardConfig {
    pub wiring: BoardWiring,
    pub calibration: Calibration,
    pub defaults: ConfigDefaults,
}
