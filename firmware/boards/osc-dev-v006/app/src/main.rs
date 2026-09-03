#![no_std]
#![no_main]

use osc_servo_ch32::prelude::*;

use panic_halt as _;

#[cfg(feature = "defmt")]
use defmt_rtt as _;

tinyboot_ch32::app::app_version!();
osc_servo_ch32::install_isrs!();

#[qingke_rt::entry]
fn main() -> ! {
    osc_servo_ch32::log::info!("osc-dev-v006: boot");
    osc_servo_ch32::run!(BoardConfig {
        wiring: BoardWiring {
            dbg: DigitalPin::PC3,
            drv_en: DrvEn {
                pin: DigitalPin::PD0,
                active: Level::High,
            },
            // Rev B TTL bus subsystem (the default): the 74LVC2G241 is in
            // play, TX_EN = PC2 gating direction. `--features half-duplex`
            // drops the bus wiring -- the direct HDSEL wire carries none,
            // and on a buffer-populated board the TX_EN pull-down (R16)
            // keeps the buffer released.
            #[cfg(not(feature = "half-duplex"))]
            bus: BusWiring { tx_en: Pin::PC2 },
            // Rev B arm-B bodge: bare OPA closed by an external 1k/15k
            // network (G = 15.0) off a 33 mohm shunt.
            current_sense: CurrentSenseConfig {
                opa: opa::Config {
                    pos: opa::PositiveInput::PD3,
                    neg: opa::NegativeInput::PA1,
                    out: opa::Output::PD4,
                },
                gain_milli: 15_000,
            },
            sensors: AdcPins {
                pos: AnalogChannel::A3,
                vmotor: (AnalogChannel::A5, AnalogChannel::A6),
            },
        },
        calibration: Calibration {
            shunt_r_mohm: 33,
            vmotor_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            vdd_mv: 3300,
        },
        defaults: ConfigDefaults {
            pos_min_phys_urad: -1_570_796,
            pos_max_phys_urad: 1_570_796,
            id: 1,
            baud: BaudRate::B1000000,
            response_deadline_us: DEFAULT_RESPONSE_DEADLINE_US,
        },
        model: MODEL_OSC_SERVO,
        hw_rev: 1,
    })
}
