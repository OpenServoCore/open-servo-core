#![no_std]
#![no_main]

use osc_servo_ch32::prelude::*;

use panic_halt as _;

#[cfg(feature = "defmt")]
use defmt_rtt as _;

#[unsafe(link_section = ".tb_version")]
#[used]
static APP_VERSION: u16 = osc_servo_ch32::FIRMWARE_VERSION;
osc_servo_ch32::install_isrs!();

#[qingke_rt::entry]
fn main() -> ! {
    osc_servo_ch32::log::info!("osc-dev-v006: boot");
    osc_servo_ch32::run!(BoardConfig {
        wiring: BoardWiring {
            // DRV8212P nSLEEP; the Rh1 pull-down holds it asleep through reset.
            drv_en: DrvEn {
                pin: DigitalPin::PC3,
                active: Level::High,
            },
            stat_led_active: Level::Low,
            // Bare OPA (PD7 needs RST_MODE=11 and JP1 closed) behind the
            // external Rf 6K4 / Rg 430 difference network: G = 14.884.
            current_sense: CurrentSenseConfig {
                opa: opa::Config {
                    pos: opa::PositiveInput::PD7,
                    neg: opa::NegativeInput::PD0,
                    out: opa::Output::PD4,
                },
                gain_milli: 14_884,
            },
            // A0 is the NTC with JP2 on NTC-IN.
            sensors: AdcPins {
                pos: AnalogChannel::A1,
                vmotor: (AnalogChannel::A5, AnalogChannel::A6),
                vbus: AnalogChannel::A2,
                ntc: AnalogChannel::A0,
            },
        },
        calibration: Calibration {
            shunt_r_mohm: 60,
            // Bottom legs return to VB: a tap reads 0.2 x terminal + 0.8 x VB.
            vmotor_divider: Divider {
                top_ohm: 6_400,
                bot_ohm: 1_600,
            },
            // Returned to GND.
            vbus_divider: Divider {
                top_ohm: 6_400,
                bot_ohm: 1_600,
            },
            ntc: Ntc {
                pullup_ohm: 10_000,
                r25_ohm: 10_000,
                beta: 3950,
            },
            // VB from 3V3 through 430 / 100: 4095 x 100 / 530.
            vmotor_bias_nom_counts: 773,
            // SS54 OR-ing Schottkys (Dp1, Dp2) between the supply inputs and
            // VSYS.
            rail_drop_mv: 250,
            vdd_mv: 3300,
            // Scan order is [shunt, vmA, vmB, pos, vcal, vbus, ntc]. The i
            // floor trades the amplifier's settle after the bridge edge for
            // reach: the shunt approaches its plateau from below, inside 3%
            // from 57 ticks and 1% from 125, and a reading a known few
            // percent low beats none. On a motor the edge also charges the
            // winding's terminal capacitance (~10 nF on the MG90) through the
            // shunt, which reads HIGH by a fixed few counts under ~72 ticks,
            // 10-15% of a 30-count current at the floor. The v floor covers
            // both terminal taps settling to the rail and vmotor_b's S/H
            // close, 131 ticks after the crest trigger at ADCCLK 24 MHz with
            // 13.5-cycle apertures.
            i_window_min_ticks: 64,
            v_window_min_ticks: 160,
            // The amplifier tail after a drive pulse, measured on board D
            // over a duty grid: the trough sits a flat 5 counts over the
            // disabled-driver rest bias down to 960 brake ticks (20% duty),
            // then climbs with drive current - 7 counts at 30%, 16 at 60%,
            // 38 at 85%.
            bias_brake_min_ticks: 960,
            i_settle_gain: SettleGain::UNITY,
        },
        defaults: ConfigDefaults {
            pos_min_phys_counts: 0,
            pos_max_phys_counts: 4095,
            id: 1,
            baud: BaudRate::B1000000,
            response_deadline_us: DEFAULT_RESPONSE_DEADLINE_US,
        },
        model: MODEL_OSC_SERVO,
        hw_rev: 2,
    })
}
