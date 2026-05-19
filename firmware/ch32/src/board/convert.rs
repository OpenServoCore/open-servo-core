use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::statics::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN};

use super::config::{Divider, NtcCal};

// CH32V006 V_REFINT typ 1.20 V (DS Table 3-5: min 1.18, max 1.22).
const VREF_NOMINAL_MV: u32 = 1200;

const ADC_MAX_RAW: u32 = 4095;

// Two scans per PWM period: peak = ON-window centre, trough = OFF-window centre.
// Which half lands first depends on UEV alignment at timer start.
pub(super) const SCAN_PEAK_OFFSET: usize = 0;
#[allow(dead_code)]
pub(super) const SCAN_TROUGH_OFFSET: usize = ADC_SCAN_LEN;

pub(super) const SCAN_IDX_SHUNT: usize = 0;
pub(super) const SCAN_IDX_POS: usize = 1;
pub(super) const SCAN_IDX_NTC: usize = 2;
pub(super) const SCAN_IDX_VBUS: usize = 3;
pub(super) const SCAN_IDX_VMOTOR_A: usize = 4;
pub(super) const SCAN_IDX_VMOTOR_B: usize = 5;
#[allow(dead_code)]
pub(super) const SCAN_IDX_ENC_A: usize = 6;
#[allow(dead_code)]
pub(super) const SCAN_IDX_ENC_B: usize = 7;
pub(super) const SCAN_IDX_VREF: usize = 8;

// DMA writes ADC_DMA_BUF in the background; each cell needs a volatile read.
pub(super) fn volatile_snapshot_scan() -> [u16; ADC_DMA_BUF_LEN] {
    let src = ADC_DMA_BUF.get() as *const u16;
    let mut out = [0u16; ADC_DMA_BUF_LEN];
    for (i, slot) in out.iter_mut().enumerate() {
        // SAFETY: ADC_DMA_BUF is a `SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]>`;
        // `src.add(i)` stays in-bounds for i < ADC_DMA_BUF_LEN.
        *slot = unsafe { src.add(i).read_volatile() };
    }
    out
}

pub(super) fn vdd_mv_from_vref(raw_vref: u16) -> u32 {
    // V_dd = V_REFINT · ADC_MAX / raw_vref. Clamp denom to 1 so a dead Vref
    // returns a huge V_dd rather than panicking in the ISR.
    let denom = raw_vref.max(1) as u32;
    VREF_NOMINAL_MV * ADC_MAX_RAW / denom
}

fn raw_to_pin_mv(raw: u16, vdd_mv: u32) -> u32 {
    raw as u32 * vdd_mv / ADC_MAX_RAW
}

pub(super) fn divider_to_mv(raw: u16, vdd_mv: u32, div: &Divider) -> Millivolts {
    let v_in = pin_mv_to_input_mv(raw_to_pin_mv(raw, vdd_mv), div);
    Millivolts(v_in.min(i16::MAX as u32) as i16)
}

pub(super) fn vmotor_diff_mv(raw_a: u16, raw_b: u16, vdd_mv: u32, div: &Divider) -> Millivolts {
    let a = pin_mv_to_input_mv(raw_to_pin_mv(raw_a, vdd_mv), div);
    let b = pin_mv_to_input_mv(raw_to_pin_mv(raw_b, vdd_mv), div);
    let diff = a.max(b) - a.min(b);
    Millivolts(diff.min(i16::MAX as u32) as i16)
}

fn pin_mv_to_input_mv(v_pin_mv: u32, div: &Divider) -> u32 {
    // V_in = V_pin · (top + bot) / bot. Clamp bot to 1 against misconfig.
    let bot = div.bot_ohm.max(1);
    let sum = div.top_ohm.saturating_add(div.bot_ohm);
    (v_pin_mv as u64 * sum as u64 / bot as u64) as u32
}

pub(super) fn shunt_to_milliamps(
    raw: u16,
    bias_raw: u16,
    gain_factor: u16,
    vdd_mv: u32,
    shunt_r_mohm: u16,
) -> Milliamps {
    // I_shunt_ma = (raw − bias) · Vdd_mv · 1000 / (4095 · gain · R_mohm).
    // i64 fold: peak ~ ±2048 × 3300 × 1000 ≈ 6.8e9 overflows i32.
    let signed_count = raw as i32 - bias_raw as i32;
    let r = shunt_r_mohm.max(1) as i64;
    let num = signed_count as i64 * vdd_mv as i64 * 1_000;
    let den = ADC_MAX_RAW as i64 * gain_factor.max(1) as i64 * r;
    let i_ma = num / den;
    Milliamps(i_ma.clamp(i16::MIN as i64, i16::MAX as i64) as i16)
}

pub(super) fn ntc_to_centi_celsius(_raw: u16, cal: &NtcCal) -> CentiCelsius {
    // TODO: full β-model needs fixed-point ln; pass-through returns T₀.
    let _ = (cal.beta, cal.r0_ohm, cal.bias_r_ohm);
    CentiCelsius(cal.t0_dc)
}

pub(super) fn pos_to_microrads(raw: u16, min_urad: i32, max_urad: i32) -> Microrads {
    // Linear ramp until pot_lut interpolation lands in CALIB.
    // i64 intermediates: span ~6.3e6 × 4095 overflows i32.
    let span = max_urad as i64 - min_urad as i64;
    let interp = span * raw as i64 / ADC_MAX_RAW as i64;
    Microrads(min_urad.saturating_add(interp as i32))
}
