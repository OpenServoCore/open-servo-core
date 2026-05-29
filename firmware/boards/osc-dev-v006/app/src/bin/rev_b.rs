#![no_std]
#![no_main]

use osc_ch32::prelude::*;

use panic_halt as _;

#[cfg(feature = "defmt")]
use defmt_rtt as _;

tinyboot_ch32::app::app_version!();
osc_ch32::install_isrs!();

#[qingke_rt::entry]
fn main() -> ! {
    osc_ch32::log::info!("osc-dev-v006 rev B: boot");
    osc_ch32::run!(BoardConfig {
        wiring: BoardWiring {
            // STAT on PC7 = TIM1_CH4 (Remap7); TODO drive via TIM1 instead of GPIO.
            stat_led: Pin::PC7,
            dbg: Pin::PC3,
            tim2_remap: Tim2Mapping::Remap0,
            motor: MotorConfig {
                tim1: Tim1Mapping::Remap7,
                in1: timer::Channel::CH3,
                in2: timer::Channel::CH2,
                drv_en: Pin::PD0,
                pwm_freq_hz: 20_000,
                polarity: timer::Polarity::ActiveHigh,
            },
            current_sense: CurrentSenseConfig {
                opa: opa::Config {
                    input: opa::InputMode::Differential {
                        pos: opa::PositiveInput::PA2,
                        neg: opa::NegativeInput::PA4,
                    },
                    gain: opa::Gain::X32,
                    bias: opa::Bias::MidRail,
                    output: opa::Output::Internal,
                },
                adc_sample_time: adc::SampleTime::CYCLES9,
            },
            sensors: AdcPins {
                pos: adc::Input::new(adc::Channel::IN3, adc::SampleTime::CYCLES9),
                ntc: adc::Input::new(adc::Channel::IN2, adc::SampleTime::CYCLES9),
                vbus: adc::Input::new(adc::Channel::IN1, adc::SampleTime::CYCLES9),
                vmotor: (
                    adc::Input::new(adc::Channel::IN5, adc::SampleTime::CYCLES9),
                    adc::Input::new(adc::Channel::IN6, adc::SampleTime::CYCLES9),
                ),
            },
            dxl: DxlBus {
                usart: UsartMapping::Usart1Remap3,
                duplex: Duplex::Full,
                rx_pull: gpio::Pull::None,
                tx_en: Some(TxEn {
                    pin: Pin::PC2,
                    tx_level: gpio::Level::High,
                }),
            },
        },
        calibration: Calibration {
            shunt_r_mohm: 10,
            vbus_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            vmotor_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            // TH1 SDNT2012X103F3950FTF.
            ntc: NtcCal {
                beta: 3950,
                r0_ohm: 10_000,
                t0_cc: 2500,
                bias_r_ohm: 10_000,
            },
        },
        defaults: ConfigDefaults {
            pos_min_phys_urad: -1_570_796,
            pos_max_phys_urad: 1_570_796,
            vdd_mv: 3300,
            dxl_id: 1,
            dxl_baud: BaudRate::B1000000,
            dxl_return_delay_2us: 125,
            clock_trim: 0,
        },
    })
}
