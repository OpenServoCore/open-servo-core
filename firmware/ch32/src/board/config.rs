use osc_core::ConfigDefaults;

use crate::hal::{Pin, Tim1Mapping, Tim2Mapping, adc, opa, timer};

pub struct MotorConfig {
    pub tim1: Tim1Mapping,
    pub in1: timer::Channel,
    pub in2: timer::Channel,
    pub drv_en: Pin,
    pub pwm_freq_hz: u32,
    pub polarity: timer::Polarity,
}

pub struct CurrentSenseConfig {
    pub opa_input: opa::InputMode,
    pub opa_gain: opa::Gain,
    pub opa_bias: opa::Bias,
    /// OPA output is hard-driven; shortest sample time usually suffices.
    pub adc_sample_time: adc::SampleTime,
}

/// Non-shunt sensor inputs. Scan layout in DMA buffer is
/// `[shunt, pos, ntc, vbus, vmotor.0, vmotor.1, enc.0, enc.1, vref]`;
/// shunt and Vref slots are chip-lib-owned.
pub struct Sensors {
    pub pos: adc::Input,
    pub ntc: adc::Input,
    pub vbus: adc::Input,
    pub vmotor: (adc::Input, adc::Input),
    pub enc: (adc::Input, adc::Input),
}

/// `V_adc = V_in · bot_ohm / (top_ohm + bot_ohm)`.
pub struct Divider {
    pub top_ohm: u32,
    pub bot_ohm: u32,
}

/// β-model NTC params: `R_ntc(T) = r0_ohm · exp(beta · (1/T − 1/T₀))`.
pub struct NtcCal {
    /// β in Kelvin.
    pub beta: u16,
    pub r0_ohm: u32,
    /// T₀ in deci-°C (25.0 °C → 250) — matches `osc_units::CentiCelsius` (0.1 °C/unit).
    pub t0_dc: i16,
    /// Series bias resistor to VDD.
    pub bias_r_ohm: u32,
}

/// Schematic-derived constants identical across every unit of a PCB design.
/// Per-unit calibration lives in flash CALIB, not here.
pub struct Calibration {
    pub shunt_r_mohm: u16,
    pub vbus_divider: Divider,
    /// Shared by V_motor_A and V_motor_B (matched dividers).
    pub vmotor_divider: Divider,
    pub ntc: NtcCal,
}

/// Schematic-fixed wiring; consumed during `Ch32Board::new` and not retained.
pub struct BoardWiring {
    pub stat_led: Pin,
    pub tim2_remap: Tim2Mapping,
    pub motor: MotorConfig,
    pub current_sense: CurrentSenseConfig,
    pub sensors: Sensors,
}

/// `defaults` is seeded into `Shared.table.config` once at boot; thereafter owned by host.
pub struct BoardConfig {
    pub wiring: BoardWiring,
    pub calibration: Calibration,
    pub defaults: ConfigDefaults,
}
