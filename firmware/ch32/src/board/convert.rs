use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::statics::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN};

use super::config::{Calibration, Divider, NtcCal};

const ADC_MAX_RAW: u32 = 4095;

/// Q32 reciprocals so the 20 kHz ISR runs a single `mulhu` per conversion
/// instead of pulling in `__udivdi3` / `__divdi3`.
#[derive(Copy, Clone)]
pub(super) struct Scales {
    pub(super) vbus_q32: u32,
    pub(super) vmotor_q32: u32,
    pub(super) shunt_q32: u32,
}

impl Scales {
    pub(super) fn new(cal: &Calibration, gain_factor: u16) -> Self {
        Self {
            vbus_q32: divider_q32(&cal.vbus_divider),
            vmotor_q32: divider_q32(&cal.vmotor_divider),
            shunt_q32: shunt_q32(gain_factor, cal.shunt_r_mohm),
        }
    }
}

fn divider_q32(d: &Divider) -> u32 {
    let bot = d.bot_ohm.max(1) as u64;
    let sum = d.top_ohm.saturating_add(d.bot_ohm) as u64;
    let num = sum << 32;
    let den = ADC_MAX_RAW as u64 * bot;
    (num / den).min(u32::MAX as u64) as u32
}

fn shunt_q32(gain_factor: u16, r_mohm: u16) -> u32 {
    let g = gain_factor.max(1) as u64;
    let r = r_mohm.max(1) as u64;
    let num = 1000u64 << 32;
    let den = ADC_MAX_RAW as u64 * g * r;
    (num / den).min(u32::MAX as u64) as u32
}

// UG fires TRGO at CNT=0 before CEN=1, so the trough scan lands first.
pub(super) const SCAN_PEAK_OFFSET: usize = ADC_SCAN_LEN;
pub(super) const SCAN_TROUGH_OFFSET: usize = 0;

pub(super) const SCAN_IDX_SHUNT_POST: usize = 0;
pub(super) const SCAN_IDX_POS: usize = 1;
pub(super) const SCAN_IDX_NTC: usize = 2;
pub(super) const SCAN_IDX_VMOTOR_A: usize = 3;
pub(super) const SCAN_IDX_VMOTOR_B: usize = 4;
pub(super) const SCAN_IDX_VCAL: usize = 5;

pub(super) fn volatile_snapshot_scan() -> [u16; ADC_DMA_BUF_LEN] {
    let src = ADC_DMA_BUF.get() as *const u16;
    let mut out = [0u16; ADC_DMA_BUF_LEN];
    for (i, slot) in out.iter_mut().enumerate() {
        // SAFETY: `src.add(i)` stays in-bounds for i < ADC_DMA_BUF_LEN.
        *slot = unsafe { src.add(i).read_volatile() };
    }
    out
}

/// EWMA low-pass, α = 1/128. `state_q6` keeps 6 sub-LSB bits.
pub(super) struct VcalLpf {
    state_q6: i32,
    initialized: bool,
}

impl VcalLpf {
    pub(super) const fn new() -> Self {
        Self {
            state_q6: 0,
            initialized: false,
        }
    }

    pub(super) fn update(&mut self, raw: u16) -> u16 {
        let x = (raw as i32) << 6;
        if !self.initialized {
            self.state_q6 = x;
            self.initialized = true;
        } else {
            self.state_q6 += (x - self.state_q6) >> 7;
        }
        (self.state_q6 >> 6) as u16
    }
}

pub(super) fn divider_to_mv(raw: u16, vdd_mv: u32, scale_q32: u32) -> Millivolts {
    let prod = raw as u32 * vdd_mv;
    let v_in = ((prod as u64) * scale_q32 as u64) >> 32;
    Millivolts(v_in.min(i16::MAX as u64) as i16)
}

pub(super) fn vmotor_diff_mv(raw_a: u16, raw_b: u16, vdd_mv: u32, scale_q32: u32) -> Millivolts {
    let a = divider_to_mv(raw_a, vdd_mv, scale_q32).0 as i32;
    let b = divider_to_mv(raw_b, vdd_mv, scale_q32).0 as i32;
    Millivolts((a - b).unsigned_abs().min(i16::MAX as u32) as i16)
}

pub(super) fn shunt_to_milliamps(
    raw: u16,
    bias_raw: u16,
    vdd_mv: u32,
    scale_q32: u32,
) -> Milliamps {
    let signed = raw as i32 - bias_raw as i32;
    let mag = signed.unsigned_abs() as u64 * vdd_mv as u64;
    let i_ma_mag = ((mag * scale_q32 as u64) >> 32) as i32;
    let i_ma = if signed >= 0 { i_ma_mag } else { -i_ma_mag };
    Milliamps(i_ma.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
}

pub(super) fn ntc_to_centi_celsius(_raw: u16, cal: &NtcCal) -> CentiCelsius {
    // TODO: full β-model needs fixed-point ln; pass-through returns T₀.
    let _ = (cal.beta, cal.r0_ohm, cal.bias_r_ohm);
    CentiCelsius(cal.t0_cc)
}

pub(super) fn pos_to_microrads(raw: u16, min_urad: i32, max_urad: i32) -> Microrads {
    // `>>12` approximates `/4095` (1 part in 4096) to dodge __divdi3 in the ISR.
    let span = (max_urad as i64).wrapping_sub(min_urad as i64);
    let interp = (span * raw as i64) >> 12;
    Microrads(min_urad.saturating_add(interp as i32))
}
