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
    osc_ch32::log::info!("osc-dev-v006 rev A: boot");
    let board = Ch32Board::new(BoardConfig {
        wiring: BoardWiring {
            stat_led: Pin::PD0,
            // Scope pad exposed on this rev_a board.
            dbg: Pin::PC3,
            tim2_remap: Tim2Mapping::Remap0,
            motor: MotorConfig {
                tim1: Tim1Mapping::Remap7,
                in1: Channel::CH3,
                in2: Channel::CH2,
                drv_en: Pin::PC7,
                pwm_freq_hz: 20_000,
                polarity: Polarity::ActiveHigh,
            },
            current_sense: CurrentSenseConfig {
                // Differential PGA targeting rev_b. On rev_a the differential
                // input is hardware-broken (ISNS- never reaches PA4 cleanly),
                // so current readings here are garbage until rev_b lands.
                opa_input: opa::InputMode::Differential {
                    pos: opa::PositiveInput::PA2,
                    neg: opa::NegativeInput::PA4,
                },
                opa_gain: opa::Gain::X32,
                opa_bias: opa::Bias::MidRail,
                adc_sample_time: adc::SampleTime::CYCLES9,
            },
            sensors: Sensors {
                pos: adc::Input::new(adc::Channel::IN7, adc::SampleTime::CYCLES9),
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
            // 20k/10k → V_adc = V_in · 1/3.
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
        core::hint::spin_loop();
    }
}
