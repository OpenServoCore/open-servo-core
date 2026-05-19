#![no_std]
#![no_main]

use osc_ch32::{
    ConfigDefaults,
    board::{
        BoardConfig, BoardWiring, Calibration, Ch32Board, CurrentSenseConfig, Divider, MotorConfig,
        NtcCal, Sensors,
    },
    hal::{
        Pin, Tim1Mapping, Tim2Mapping, adc, opa,
        timer::{Channel, Polarity},
    },
    statics::install_kernel,
};
use panic_halt as _;

tinyboot_ch32::app::app_version!();

#[qingke_rt::entry]
fn main() -> ! {
    let board = Ch32Board::new(BoardConfig {
        wiring: BoardWiring {
            // Rev B routes STAT to PC7 = TIM1_CH4 (Remap7) for HW-driven
            // blink. Currently still driven as plain GPIO; TIM1 CH4 wiring
            // is a follow-up task.
            stat_led: Pin::PC7,
            tim2_remap: Tim2Mapping::Remap0,
            motor: MotorConfig {
                tim1: Tim1Mapping::Remap7,
                in1: Channel::CH3,
                in2: Channel::CH2,
                drv_en: Pin::PD0,
                pwm_freq_hz: 20_000,
                polarity: Polarity::ActiveHigh,
            },
            current_sense: CurrentSenseConfig {
                // ISNS+ on PA2 (OPP0), ISNS- on PA4 (OPN2).
                opa_input: opa::InputMode::Differential {
                    pos: opa::PositiveInput::PA2,
                    neg: opa::NegativeInput::PA4,
                },
                opa_gain: opa::Gain::X32,
                opa_bias: opa::Bias::MidRail,
                // OPA output is hard-driven; CYCLES3 is plenty.
                adc_sample_time: adc::SampleTime::CYCLES3,
            },
            // Rev B vs rev A: VPOS and ENCB swap (PD4 ↔ PD2). V_bus now
            // has a real divider on PA1 (IN1). Sample times sized to
            // divider Thevenin impedance at fADC = 12 MHz — CYCLES9 clears
            // ~13 kΩ R_AIN, comfortable for all 5-7 kΩ taps.
            sensors: Sensors {
                pos: adc::Input::new(adc::Channel::IN3, adc::SampleTime::CYCLES9),
                ntc: adc::Input::new(adc::Channel::IN2, adc::SampleTime::CYCLES9),
                vbus: adc::Input::new(adc::Channel::IN1, adc::SampleTime::CYCLES9),
                vmotor: (
                    adc::Input::new(adc::Channel::IN5, adc::SampleTime::CYCLES9),
                    adc::Input::new(adc::Channel::IN6, adc::SampleTime::CYCLES9),
                ),
                // Quadrature outputs are digital — CYCLES3 once wired up.
                enc: (
                    adc::Input::new(adc::Channel::IN4, adc::SampleTime::CYCLES3),
                    adc::Input::new(adc::Channel::IN7, adc::SampleTime::CYCLES3),
                ),
            },
        },
        calibration: Calibration {
            // RS1 = 10 mΩ low-side shunt.
            shunt_r_mohm: 10,
            // 20k/10k divider → V_adc = V_in · 1/3 (V_motor sensors
            // VSNA/VSNB, V_bus sense VSNS).
            vbus_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            vmotor_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            // TH1 = SDNT2012X103F3950FTF: 10 kΩ, ±1 %, β = 3950 K @ 25 °C.
            // R13 = 10 kΩ pull-up from +3V3; NTC to GND.
            ntc: NtcCal {
                beta: 3950,
                r0_ohm: 10_000,
                t0_dc: 250,
                bias_r_ohm: 10_000,
            },
        },
        // SG90 mechanical sweep: ±π/2 rad ≈ ±1_570_796 µrad. Linear-ramp
        // endpoints until per-unit pot_lut lands in CALIB.
        defaults: ConfigDefaults {
            pos_min_phys_urad: -1_570_796,
            pos_max_phys_urad: 1_570_796,
        },
    });

    install_kernel(board);

    loop {
        core::hint::spin_loop();
    }
}
