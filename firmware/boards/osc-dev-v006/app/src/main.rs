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
    osc_ch32::log::info!("osc-dev-v006: boot");
    osc_ch32::run!(BoardConfig {
        wiring: BoardWiring {
            dbg: DigitalPin::PC3,
            drv_en: DrvEn {
                pin: DigitalPin::PD0,
                active: Level::High,
            },
            current_sense: CurrentSenseConfig {
                gain: opa::Gain::X32,
                bias: opa::Bias::MidRail,
            },
            sensors: AdcPins {
                pos: AnalogChannel::A3,
                ntc: AnalogChannel::A2,
                vbus: AnalogChannel::A1,
                vmotor: (AnalogChannel::A5, AnalogChannel::A6),
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
            id: 1,
            baud: BaudRate::B1000000,
            response_deadline_us: DEFAULT_RESPONSE_DEADLINE_US,
        },
    })
}
