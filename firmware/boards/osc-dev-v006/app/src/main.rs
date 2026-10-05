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
            // floor sits inside the settle gain's reach (`i_settle_gain`),
            // where the amplifier's own deficit is a calibrated gain, not an
            // error. On a motor the edge also charges the winding's terminal
            // capacitance (~10 nF on the MG90) through the shunt, which
            // reads HIGH by a fixed few counts under ~72 ticks, 10-15% of a
            // 30-count current at the floor; no gain removes an additive
            // term. The v floor is the crest taps' window: tap B closes 37
            // ticks before the crest (injected), so it needs the ON edge's
            // 42-tick lag plus its settle, and tap A at +83 must still be
            // inside the window when the OFF edge lands 14 ticks late. A 4.2
            // ohm grid ladder reads R within 1.1% from 84 ticks forward;
            // reverse, tap B is the high side and reads its post-edge
            // overshoot, 3-5% high from 96 to 180 ticks.
            i_window_min_ticks: 64,
            v_window_min_ticks: 93,
            // The trough scan's tap B closes 135 ticks into the window.
            v_trough_window_min_ticks: 160,
            // The drive pulse comes out ~27 ticks short of the commanded
            // width and the observer scales by the commanded one.
            bemf_window_min_ticks: 160,
            // The amplifier tail after a drive pulse, measured on board D
            // over a duty grid: the trough sits a flat 5 counts over the
            // disabled-driver rest bias down to 960 brake ticks (20% duty),
            // then climbs with drive current - 7 counts at 30%, 16 at 60%,
            // 38 at 85%.
            bias_brake_min_ticks: 960,
            // The amplifier's step response at the crest sample: the shunt
            // approaches its plateau from below, 2.9% low at 60 ticks, 1.1%
            // at 120, settled from ~208, the same in both drive signs (a
            // 3.7 ohm grid ladder, 60..300 ticks). Each band holds the
            // inverse at its centre; 40 and 48 come from the burst fold
            // alone, under the ladder's lowest rung. Every grid rung from 60
            // ticks lands within 0.16% with it.
            i_settle_gain: SettleGain {
                start_ticks: 40,
                q15: &[
                    35143, 34194, 33734, 33575, 33456, 33376, 33314, 33259, 33213, 33156, 33103,
                    33052, 33020, 32982, 32938, 32900, 32868, 32841, 32824, 32806, 32785,
                ],
            },
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
