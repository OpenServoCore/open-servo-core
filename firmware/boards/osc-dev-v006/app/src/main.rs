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
            // network (G = 15.0) off a 60 mohm 1206 shunt: 0.9 V/A, 0.9 mA
            // per count.
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
                vbus: AnalogChannel::A0,
                ntc: AnalogChannel::A2,
            },
        },
        calibration: Calibration {
            shunt_r_mohm: 60,
            // Hacked board D: 6k8 0805 top / 3k3 THT bottom returned to VB,
            // ~150 pF reservoir at each tap (2.2 kohm source, fast aperture).
            vmotor_divider: Divider {
                top_ohm: 6_800,
                bot_ohm: 3_300,
            },
            // Board D bodge as fitted (15k/10k); rev-2A lands 20_000/10_000.
            vbus_divider: Divider {
                top_ohm: 15_000,
                bot_ohm: 10_000,
            },
            ntc: Ntc {
                pullup_ohm: 10_000,
                r25_ohm: 10_000,
                beta: 3950,
            },
            // Terminal-divider bias VB from 3V3 through 200/47: 4095 x 47 / 247.
            vmotor_bias_nom_counts: 779,
            vdd_mv: 3300,
            // Scan order is [shunt, vmA, vmB, pos, vcal, vbus, ntc]. The i floor is
            // amp-settling-bound: arm-B network measured true from duty 13%
            // (bringup docs/armb-comp-sizing.md). The v floor covers
            // vmotor_b's S/H close, 44 ticks (0.91 us) after the crest
            // trigger at ADCCLK 48 MHz (vmotor_a's at 24 ticks, 0.49 us),
            // plus settling: on the 6k8/3k3 + 150 pF dividers both terminals
            // read the rail within 1% from 96 ticks (duty 8%, edge grid both
            // directions), 24 ticks margin. A v floor below the true edge lets
            // off-phase samples seed the vbus EWMA and latch a false undervolt.
            i_window_min_ticks: 160,
            v_window_min_ticks: 120,
        },
        defaults: ConfigDefaults {
            pos_min_phys_counts: 0,
            pos_max_phys_counts: 4095,
            id: 1,
            baud: BaudRate::B1000000,
            response_deadline_us: DEFAULT_RESPONSE_DEADLINE_US,
        },
        model: MODEL_OSC_SERVO,
        hw_rev: 1,
    })
}
