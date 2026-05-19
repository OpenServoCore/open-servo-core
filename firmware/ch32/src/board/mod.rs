//! Chip-lib board surface: the `Ch32Board` type that satisfies
//! `osc_core::Board`. Boot-time bring-up lives in `bringup`; per-tick
//! raw → SI conversion lives in `convert`; the user-facing config types
//! live in `config` and are re-exported here.

mod bringup;
mod config;
mod convert;

pub use config::{
    BoardConfig, BoardWiring, Calibration, CurrentSenseConfig, Divider, MotorConfig, NtcCal,
    Sensors,
};

use osc_core::{Board, Capabilities, FrameInputs, MotorCmd, RawSamples, SampleFrame};

use crate::hal::{
    Pin,
    gpio::{self, Level},
    pfic,
};
use crate::statics::{ADC_SCAN_LEN, SHARED, read_sample_tick};

use bringup::{
    bring_up_analog_chain, configure_adc_dma_scan, configure_pins, enable_clocks_and_remaps,
    start_center_aligned_pwm,
};
use convert::{
    SCAN_IDX_NTC, SCAN_IDX_POS, SCAN_IDX_SHUNT, SCAN_IDX_VBUS, SCAN_IDX_VMOTOR_A,
    SCAN_IDX_VMOTOR_B, SCAN_IDX_VREF, SCAN_PEAK_OFFSET, divider_to_mv, ntc_to_centi_celsius,
    pos_to_microrads, shunt_to_milliamps, vdd_mv_from_vref, vmotor_diff_mv, volatile_snapshot_scan,
};

/// Pre-computed raw-domain scalars for `shunt_to_milliamps`. Derived once
/// from `CurrentSenseConfig` in `Ch32Board::new` so the per-tick hot path
/// doesn't match on `opa::Gain` / `opa::Bias` enums.
struct ShuntScale {
    /// OPA quiescent ADC count (raw at zero shunt current).
    bias_raw: u16,
    /// PGA gain factor (`V_out = V_diff · factor + V_bias`).
    gain_factor: u16,
}

pub struct Ch32Board {
    stat_led: Pin,
    calibration: Calibration,
    shunt_scale: ShuntScale,
}

impl Ch32Board {
    pub fn new(cfg: BoardConfig) -> Self {
        let BoardConfig {
            wiring,
            calibration,
            defaults,
        } = cfg;

        let shunt_scale = ShuntScale {
            bias_raw: wiring.current_sense.opa_bias.quiescent_raw(),
            gain_factor: wiring.current_sense.opa_gain.factor(),
        };
        let stat_led = wiring.stat_led;

        enable_clocks_and_remaps(&wiring);
        configure_pins(&wiring);
        bring_up_analog_chain(&wiring.current_sense);
        configure_adc_dma_scan(&wiring.sensors, wiring.current_sense.adc_sample_time);

        // Seed CONFIG before IRQs can read it. Sole writer at this point;
        // the DMA TC ISR snapshots from `Shared` on every tick once KERNEL
        // is installed (`install_kernel` runs after `new` returns). Once
        // flash persistence lands, load order becomes: flash → these
        // defaults → core's zero `const_new`.
        SHARED.table.seed_config_defaults(&defaults);

        pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
        start_center_aligned_pwm(&wiring.motor);

        Self {
            stat_led,
            calibration,
            shunt_scale,
        }
    }

    #[inline]
    pub fn set_stat_led(&self, on: bool) {
        gpio::set_level(self.stat_led, if on { Level::High } else { Level::Low });
    }

    /// Snapshot the latest ADC scan and convert it to engineering units.
    /// Called from the DMA1 TC ISR after one PWM period's worth of scans
    /// has landed in `ADC_DMA_BUF`. Uses the peak half-scan (ON-window
    /// centre) for every field; `current` is the motor current during the
    /// ON window, `vmotor` is the driven |V_A − V_B|. The trough half-scan
    /// is reserved for freewheel diagnostics and not surfaced yet.
    ///
    /// Reads live values via `inputs` (snapped from `Shared` by the ISR
    /// shim) — host writes via DXL take effect on the next tick.
    /// Schematic-fixed constants come from `self.calibration` and the
    /// pre-computed `self.shunt_scale`.
    //
    // TODO: replace `inputs: &FrameInputs` with a flash-backed read-only
    // view once persistence lands. Torque-off serializes host writes
    // against board reads, so the view can be plain raw pointers into
    // the active CONFIG page and into CALIB — no per-tick snapshot.
    pub fn build_sample_frame(&self, inputs: &FrameInputs) -> SampleFrame {
        let scan = volatile_snapshot_scan();
        let peak = &scan[SCAN_PEAK_OFFSET..SCAN_PEAK_OFFSET + ADC_SCAN_LEN];

        let raw_shunt = peak[SCAN_IDX_SHUNT];
        let raw_pos = peak[SCAN_IDX_POS];
        let raw_ntc = peak[SCAN_IDX_NTC];
        let raw_vbus = peak[SCAN_IDX_VBUS];
        let raw_vmotor_a = peak[SCAN_IDX_VMOTOR_A];
        let raw_vmotor_b = peak[SCAN_IDX_VMOTOR_B];
        let raw_vref = peak[SCAN_IDX_VREF];

        let vdd_mv = vdd_mv_from_vref(raw_vref);

        SampleFrame {
            tick: read_sample_tick(),
            pos: pos_to_microrads(raw_pos, inputs.pos_min_phys_urad, inputs.pos_max_phys_urad),
            current: shunt_to_milliamps(
                raw_shunt,
                self.shunt_scale.bias_raw,
                self.shunt_scale.gain_factor,
                vdd_mv,
                self.calibration.shunt_r_mohm,
            ),
            temp: ntc_to_centi_celsius(raw_ntc, &self.calibration.ntc),
            vbus: divider_to_mv(raw_vbus, vdd_mv, &self.calibration.vbus_divider),
            vmotor: vmotor_diff_mv(
                raw_vmotor_a,
                raw_vmotor_b,
                vdd_mv,
                &self.calibration.vmotor_divider,
            ),
            raw: RawSamples {
                pos: raw_pos,
                current: raw_shunt,
                temp: raw_ntc,
                vbus: raw_vbus,
                vmotor_a: raw_vmotor_a,
                vmotor_b: raw_vmotor_b,
            },
        }
    }
}

impl Board for Ch32Board {
    fn caps(&self) -> Capabilities {
        Capabilities::default()
    }

    fn write_motor(&mut self, _cmd: MotorCmd) {}

    fn pulse_tick_indicator(&mut self) {
        gpio::toggle(self.stat_led);
    }
}
