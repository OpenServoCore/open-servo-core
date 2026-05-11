#![no_std]
#![no_main]

use osc_ch32::{
    board::{BoardConfig, Ch32Board, CurrentSenseConfig, MotorConfig},
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
        stat_led: Pin::PD0,
        motor: MotorConfig {
            tim1: Tim1Mapping::Remap7,
            in1: Channel::CH3,
            in2: Channel::CH2,
            drv_en: Pin::PC7,
            pwm_freq_hz: 20_000,
            polarity: Polarity::ActiveHigh,
        },
        current_sense: CurrentSenseConfig {
            opa_pos: opa::PositiveInput::PA2,
            opa_gain: opa::Gain::X32,
            opa_bias: opa::Bias::MidRail,
            adc_sample_time: adc::SampleTime::CYCLES241,
        },
        tim2: Tim2Mapping::Remap0,
    });

    install_kernel(board);

    loop {
        core::hint::spin_loop();
    }
}
