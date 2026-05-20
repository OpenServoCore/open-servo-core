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

#[cfg(feature = "defmt")]
use defmt_rtt as _;

tinyboot_ch32::app::app_version!();

#[qingke_rt::entry]
fn main() -> ! {
    osc_ch32::log::info!("osc-dev-v006 rev B: boot");
    let board = Ch32Board::new(BoardConfig {
        wiring: BoardWiring {
            // STAT on PC7 = TIM1_CH4 (Remap7); TODO drive via TIM1 instead of GPIO.
            stat_led: Pin::PC7,
            // Placeholder — pick a real scope pad on rev_b when board is in hand.
            dbg: Pin::PC3,
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
                // IN9 OPA tap empirically reads at any SMP C3..C241 on this
                // silicon (verified via bringup isns_peak_dma 2026-05-20).
                // CYCLES9 leaves the 7-channel scan at ~12 µs, well inside
                // the 25 µs half-period budget → peak + trough both honored.
                adc_sample_time: adc::SampleTime::CYCLES9,
            },
            // Rev B vs A: VPOS and ENCB swap (PD4 ↔ PD2); V_bus has real divider on PA1.
            // CYCLES9 clears ~13 kΩ R_AIN at fADC=12 MHz; covers all 5-7 kΩ taps.
            sensors: Sensors {
                pos: adc::Input::new(adc::Channel::IN3, adc::SampleTime::CYCLES9),
                ntc: adc::Input::new(adc::Channel::IN2, adc::SampleTime::CYCLES9),
                vbus: adc::Input::new(adc::Channel::IN1, adc::SampleTime::CYCLES9),
                vmotor: (
                    adc::Input::new(adc::Channel::IN5, adc::SampleTime::CYCLES9),
                    adc::Input::new(adc::Channel::IN6, adc::SampleTime::CYCLES9),
                ),
            },
        },
        calibration: Calibration {
            // RS1 = 10 mΩ low-side shunt.
            shunt_r_mohm: 10,
            // 20k/10k → V_adc = V_in · 1/3 (VSNS, VSNA, VSNB).
            vbus_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            vmotor_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            // TH1 SDNT2012X103F3950FTF; R13 = 10 kΩ pull-up to +3V3, NTC to GND.
            ntc: NtcCal {
                beta: 3950,
                r0_ohm: 10_000,
                t0_dc: 250,
                bias_r_ohm: 10_000,
            },
        },
        // SG90 ±π/2 rad ≈ ±1_570_796 µrad; linear ramp until pot_lut lands in CALIB.
        defaults: ConfigDefaults {
            pos_min_phys_urad: -1_570_796,
            pos_max_phys_urad: 1_570_796,
            // Spec-typical fallback. Replace with DMM-measured value at the
            // VDD pin once the board is in service — host can also overwrite
            // the live knob via CONFIG.calibration.vdd_mv at runtime.
            vdd_mv: 3300,
        },
    });

    install_kernel(board);

    loop {
        #[cfg(feature = "defmt")]
        osc_ch32::telemetry::pump();
        core::hint::spin_loop();
    }
}
