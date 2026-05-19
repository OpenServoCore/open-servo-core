//! Raw ADC counts → SI units. Pure math; no hardware access except the
//! `read_volatile` over `ADC_DMA_BUF` in `volatile_snapshot_scan`. Scan
//! layout constants and the snapshot helper live here so `build_sample_frame`
//! is a thin assembly of conversions.

use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::statics::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN};

use super::config::{Divider, NtcCal};

/// CH32V006 internal reference voltage, typical (DS Table 3-5: min 1.18,
/// typ 1.20, max 1.22 V). Used to back out V_dd from the V_ref ADC reading.
const VREF_NOMINAL_MV: u32 = 1200;

/// 12-bit ADC full scale.
const ADC_MAX_RAW: u32 = 4095;

/// Scan layout in `ADC_DMA_BUF`. Two scans per PWM period (peak + trough);
/// peak = ON-window centre (shunt sees motor current, V_motor sees driven
/// rail); trough = OFF-window centre (shunt sees freewheel/recirc current).
/// `SCAN_*_OFFSET` are the indices of the first u16 of each half-scan.
/// Which half lands first in the buffer depends on the UEV alignment at
/// timer start — verifying that mapping is task #6.
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

/// Volatile copy of `ADC_DMA_BUF` into a stack array — DMA writes the same
/// memory in the background, so each cell needs a `read_volatile` to keep
/// the compiler from folding repeated reads.
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
    // V_dd = V_REFINT · ADC_MAX / raw_vref. raw_vref shouldn't ever be 0
    // in practice (would mean the internal reference is dead), but guard
    // against /0 by clamping to 1 so a bad read returns a huge V_dd rather
    // than panicking inside the ISR.
    let denom = raw_vref.max(1) as u32;
    VREF_NOMINAL_MV * ADC_MAX_RAW / denom
}

fn raw_to_pin_mv(raw: u16, vdd_mv: u32) -> u32 {
    raw as u32 * vdd_mv / ADC_MAX_RAW
}

pub(super) fn divider_to_mv(raw: u16, vdd_mv: u32, div: &Divider) -> Millivolts {
    // V_in = V_pin · (top + bot) / bot.
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
    // V_in = V_pin · (top + bot) / bot. Guard against a 0 bot_ohm
    // (misconfigured divider) by clamping the denominator to 1.
    let bot = div.bot_ohm.max(1);
    let sum = div.top_ohm.saturating_add(div.bot_ohm);
    // Worst case: v_pin_mv ~3300, sum ~30_000 → 99M, comfortably in u32.
    (v_pin_mv as u64 * sum as u64 / bot as u64) as u32
}

pub(super) fn shunt_to_milliamps(
    raw: u16,
    bias_raw: u16,
    gain_factor: u16,
    vdd_mv: u32,
    shunt_r_mohm: u16,
) -> Milliamps {
    // V_shunt = (V_opa_out − V_bias) / gain
    //        = (raw − bias) · Vdd / 4095 / gain   [mV]
    // I_shunt = V_shunt / R_shunt
    //        = V_shunt_mv · 1000 / R_mohm         [mA]
    let signed_count = raw as i32 - bias_raw as i32;
    // Fold through i64 so the ×1000 doesn't overflow on i32 (peak case
    // ~ ±2048 × 3300 × 1000 ≈ 6.8e9).
    let r = shunt_r_mohm.max(1) as i64;
    let num = signed_count as i64 * vdd_mv as i64 * 1_000;
    let den = ADC_MAX_RAW as i64 * gain_factor.max(1) as i64 * r;
    let i_ma = num / den;
    Milliamps(i_ma.clamp(i16::MIN as i64, i16::MAX as i64) as i16)
}

pub(super) fn ntc_to_centi_celsius(_raw: u16, cal: &NtcCal) -> CentiCelsius {
    // Phase 3 pass-through: returns T₀ regardless of `raw`. Full β-model
    // conversion (R_ntc = R_bias · raw / (ADC_MAX − raw); T = 1/(1/T₀ +
    // ln(R/R₀)/β)) needs a fixed-point ln and is tracked separately. All
    // calibration scalars are still consumed at the type level so the
    // field set stays stable across the swap.
    let _ = (cal.beta, cal.r0_ohm, cal.bias_r_ohm);
    CentiCelsius(cal.t0_dc)
}

pub(super) fn pos_to_microrads(raw: u16, min_urad: i32, max_urad: i32) -> Microrads {
    // Linear ramp across the full ADC range: raw=0 → min_urad, raw=4095 →
    // max_urad. CALIB-Erased fallback per the control-table doc; replaced
    // by `pot_lut` interpolation once CALIB persistence lands.
    // i64 intermediates: (max-min) ~ 6.3e6, ×4095 ~ 2.6e10, overflows i32.
    let span = max_urad as i64 - min_urad as i64;
    let interp = span * raw as i64 / ADC_MAX_RAW as i64;
    Microrads(min_urad.saturating_add(interp as i32))
}
