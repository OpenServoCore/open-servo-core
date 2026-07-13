//! Chip-side sensor composer. The 20 kHz control-loop ISR drains
//! `ADC_DMA_BUF` at the slot indices in `scan`, feeds the raw codes through
//! the pipeline in `convert` (+ the Vcal LPF in `lpf`), and hands the result
//! to the kernel via `osc_servo_core::Sensors::sample`. Math is V006-and-board-
//! shaped: 12-bit ADC, shunt-resistor current, NTC-thermistor temp,
//! voltage-divider rails, peak/trough sampling under center-aligned PWM.

mod convert;
mod lpf;
pub(crate) mod scan;

pub use convert::Scales;

use osc_servo_core::{ConversionVariables, RawSamples, Sample, Sensors as SensorsTrait};

use crate::cfg::Calibration;
use crate::runtime::statics::read_sample_tick;

use convert::{
    divider_to_mv, ntc_to_centi_celsius, pos_to_microrads, shunt_to_milliamps, vmotor_diff_mv,
};
use lpf::VcalLpf;
use scan::{
    SCAN_IDX_NTC, SCAN_IDX_POS, SCAN_IDX_SHUNT_POST, SCAN_IDX_VCAL, SCAN_IDX_VMOTOR_A,
    SCAN_IDX_VMOTOR_B, SCAN_PEAK_OFFSET, SCAN_TROUGH_OFFSET, scan_slot,
};

pub struct Ch32Sensors {
    calibration: Calibration,
    shunt_bias_raw: u16,
    scales: Scales,
    vcal_lpf: VcalLpf,
}

impl Ch32Sensors {
    pub const fn new(calibration: Calibration, shunt_bias_raw: u16, scales: Scales) -> Self {
        Self {
            calibration,
            shunt_bias_raw,
            scales,
            vcal_lpf: VcalLpf::new(),
        }
    }
}

impl SensorsTrait for Ch32Sensors {
    /// Called from DMA1 TC ISR. Peak drives current; trough is diagnostic.
    fn sample(&mut self, vars: &ConversionVariables) -> Sample {
        let raw_shunt_post_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_SHUNT_POST);
        let raw_vmotor_a_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_A);
        let raw_vmotor_b_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_B);

        let raw_shunt_post_peak = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_SHUNT_POST);
        let raw_pos = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_POS);
        let raw_ntc = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_NTC);
        let raw_vmotor_a = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_A);
        let raw_vmotor_b = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_B);
        let raw_vcal = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VCAL);
        let raw_vbus = 0u16;

        let filtered_vcal = self.vcal_lpf.update(raw_vcal);
        let vdd_mv = vars.vdd_mv as u32;

        let post_peak = shunt_to_milliamps(
            raw_shunt_post_peak,
            self.shunt_bias_raw,
            vdd_mv,
            self.scales.shunt_q32,
        );
        let post_trough = shunt_to_milliamps(
            raw_shunt_post_trough,
            self.shunt_bias_raw,
            vdd_mv,
            self.scales.shunt_q32,
        );

        Sample {
            tick: read_sample_tick(),
            pos: pos_to_microrads(raw_pos, vars.pos_min_phys_urad, vars.pos_max_phys_urad),
            current: post_peak,
            current_post_trough: post_trough,
            temp: ntc_to_centi_celsius(raw_ntc, &self.calibration.ntc),
            vbus: divider_to_mv(raw_vbus, vdd_mv, self.scales.vbus_q32),
            vmotor: vmotor_diff_mv(raw_vmotor_a, raw_vmotor_b, vdd_mv, self.scales.vmotor_q32),
            raw: RawSamples {
                pos: raw_pos,
                current: raw_shunt_post_peak,
                shunt_post_trough: raw_shunt_post_trough,
                temp: raw_ntc,
                vbus: raw_vbus,
                vmotor_a: raw_vmotor_a,
                vmotor_a_trough: raw_vmotor_a_trough,
                vmotor_b: raw_vmotor_b,
                vmotor_b_trough: raw_vmotor_b_trough,
                vcal: raw_vcal,
                vcal_lpf: filtered_vcal,
            },
        }
    }
}
