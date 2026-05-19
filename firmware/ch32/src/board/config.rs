//! `BoardConfig` and its sub-structs — the wire-up + scaling contract
//! between board firmware and chip-lib. All types here are inert data;
//! behaviour lives in `bringup`, `convert`, and `mod.rs`.

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
    /// Carries the positive pin and (for differential) the negative pin.
    /// The chip-lib auto-configures both as analog inputs.
    pub opa_input: opa::InputMode,
    pub opa_gain: opa::Gain,
    pub opa_bias: opa::Bias,
    /// ADC sample time for the OPA output (shunt). The OPA drives the pin
    /// directly so this can be the shortest setting; bumped longer only if
    /// switching noise pulls the converter off.
    pub adc_sample_time: adc::SampleTime,
}

/// ADC inputs for the board's non-shunt sensors. Each `adc::Input` carries
/// a channel + sample time (board picks the sample time to match its
/// divider Thevenin impedance). Scan layout in the DMA buffer is
/// `[shunt, pos, ntc, vbus, vmotor.0, vmotor.1, enc.0, enc.1, vref]`.
/// `vmotor` is the H-bridge half-rail pair `(A, B)`; `enc` is the
/// quadrature pair `(A, B)`. Each channel's GPIO pin is auto-configured
/// as analog input by `Ch32Board::new`. The shunt and Vref slots are
/// chip-lib-owned (not in this struct).
pub struct Sensors {
    pub pos: adc::Input,
    pub ntc: adc::Input,
    pub vbus: adc::Input,
    pub vmotor: (adc::Input, adc::Input),
    pub enc: (adc::Input, adc::Input),
}

/// Resistive voltage divider feeding an ADC pin. `top_ohm` sits between
/// the measured rail and the tap; `bot_ohm` sits between the tap and
/// ground. `V_adc = V_in · bot_ohm / (top_ohm + bot_ohm)`. For the
/// rev-B 20 k / 10 k divider: `top_ohm = 20_000, bot_ohm = 10_000` →
/// V_adc = V_in · 1/3.
pub struct Divider {
    pub top_ohm: u32,
    pub bot_ohm: u32,
}

/// β-model NTC parameters for `R_ntc(T) = r0_ohm · exp(beta · (1/T − 1/T₀))`,
/// plus the series bias resistor that forms the V_adc divider with the NTC.
pub struct NtcCal {
    /// β coefficient (Kelvin). Typical 3950 for 10 kΩ NTCs.
    pub beta: u16,
    /// Reference resistance R₀ at T₀.
    pub r0_ohm: u32,
    /// Reference temperature in deci-°C (25.0 °C → 250), matching
    /// `osc_units::CentiCelsius` (0.1 °C / unit despite the type name).
    pub t0_dc: i16,
    /// Series bias resistor to VDD in the NTC divider.
    pub bias_r_ohm: u32,
}

/// Per-board physical constants that scale raw ADC counts → SI units.
/// Schematic-derived; identical across every assembled unit of a given
/// PCB design. Per-unit calibration (pot endpoints, mechanical angle
/// limits, BEMF Ke, etc.) lives in flash CALIB, not here.
pub struct Calibration {
    /// Low-side current-sense shunt resistance.
    pub shunt_r_mohm: u16,
    pub vbus_divider: Divider,
    /// Shared by V_motor_A and V_motor_B (matched dividers).
    pub vmotor_divider: Divider,
    pub ntc: NtcCal,
}

/// How this PCB is wired: pins, channels, TIM remaps. Schematic-fixed.
/// All fields are consumed during `Ch32Board::new` and not retained.
pub struct BoardWiring {
    pub stat_led: Pin,
    pub tim2_remap: Tim2Mapping,
    pub motor: MotorConfig,
    pub current_sense: CurrentSenseConfig,
    pub sensors: Sensors,
}

/// Everything `Ch32Board` needs to bring the chip up and convert raw scans
/// into SI units. Three buckets by concern:
///
/// - `wiring`: PCB-fixed pins, channels, TIM remap selections.
/// - `calibration`: schematic-derived constants (shunt R, divider ratios,
///   NTC params) used by `build_sample_frame`.
/// - `defaults`: runtime-mutable values that need a sensible boot value;
///   seeded into `Shared.table.config` once and then owned by the host.
pub struct BoardConfig {
    pub wiring: BoardWiring,
    pub calibration: Calibration,
    pub defaults: ConfigDefaults,
}
