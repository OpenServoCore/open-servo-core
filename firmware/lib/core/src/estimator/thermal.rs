//! Winding thermometer, SLOW rate (control-theory "The Winding as a
//! Thermometer"). The absolute base is the board NTC: the winding excess
//! over it, `x`, is carried by a first-order model `x += alpha (g P - x)`
//! with `P = i (v_mean - Ke omega)` whenever nothing better is known
//! (CARRY). At a seat, once the drive has held one operating point long
//! enough to average a reference `R_ref`, copper's tempco turns the
//! same-seat resistance RATIO into the temperature change (TRACK):
//! `t = t_ref + k_ref (r_hat - R_ref)`. No resistance is ever an absolute:
//! the brush contact moves R by percents per seat (1% is 2.6 C), so the
//! base is dropped when the seat changes (position or current moved) or
//! the samples stop, and the carry bridges to the next seat's own
//! reference. A contact step inside one hold reads as temperature until
//! that re-base (a known blind spot: +1.5% is +3.9 C).
//!
//! Two SLOW ticks of the same `(v_mean, i)` within an eighth gate a
//! sample: the pot observer cannot vouch for a motor turning inside the
//! gear lash (the bench shake heater passed 0 of 45202 polls, a seated
//! hold every one). A sample never carries a back-EMF term: a shaft that
//! moves leaves the seat band and drops the base.
//!
//! Zero model constants (`alpha`, `g`) or no NTC curve is UNSET: `t` reads
//! [`UNSET_CC`], which sits under every derate threshold, and the flag says
//! so; nothing is ever frozen.

use crate::math::{q_mul, q_mul_u, recip_div};

/// What the estimator reads from CALIB and CONFIG (kernel config load).
#[derive(Copy, Clone, Default)]
pub struct ThermCfg {
    /// Carry step per SLOW tick, dt/tau, Q0.24.
    pub alpha_q24: u16,
    /// Steady winding excess over the NTC per unit of `i (v - Ke omega)`,
    /// centi-C per vcount-ccount, Q0.16.
    pub g_q016: u16,
    /// LMS step, Q0.16.
    pub mu_q016: u16,
    /// Cold winding R at [`R_COLD_REF_CC`], Q4.12; 0 = none (no hot-boot
    /// floor, no cold check).
    pub r_cold_q12: u16,
    pub i_min_counts: u16,
    /// Position band a seat holds, counts.
    pub seat_band_counts: u16,
    /// Cold-R divergence that raises the recalibrate flag, Q0.16 of
    /// `r_cold_q12`.
    pub cold_band_q016: u16,
    pub ntc: NtcCfg,
}

/// The board NTC's counts-to-centi-C curve, host-derived from the beta
/// model (`CalibSenseExt` pull-up / R25 / beta) as a quadratic about the
/// 25 C point: `t = t_ref + k1 d + k2 d^2`, `d = raw_ref - raw`. Within
/// 0.7 C of the beta curve from 15 to 50 C on 10K/10K/3950 (the tangent
/// alone reads 2.3 C low at 45 C); `k1 == 0` = no curve.
#[derive(Copy, Clone, Default)]
pub struct NtcCfg {
    pub raw_ref: u16,
    pub t_ref_cc: i16,
    /// centi-C per count at the reference, Q8.8.
    pub k1_q88: i16,
    /// centi-C per count squared, Q0.24.
    pub k2_q24: u16,
}

/// One SLOW tick's inputs.
#[derive(Copy, Clone, Default)]
pub struct Sample {
    /// Drive-window terminal differential, duty-weighted (bemf RECIP_ARR
    /// convention); None while the taps' window is invalid.
    pub v_mean: Option<i32>,
    pub i_meas: Option<i32>,
    /// The pot observer's signed speed, c/s.
    pub omega_hat_cps: i32,
    /// Forward Ke, Q4.12 vcounts per c/s.
    pub ke_vpc_q: u16,
    /// The observer's position, counts.
    pub pos_counts: i32,
    pub ntc_raw: u16,
}

pub mod flag {
    /// No model constants or no NTC curve: `t_winding_cc` is the sentinel.
    pub const UNSET: u8 = 1 << 0;
    /// A same-seat reference is live: the ratio reads the temperature.
    pub const TRACK: u8 = 1 << 1;
    /// The cold-R check found the stored cold R stale: recalibrate.
    pub const COLD_RECAL: u8 = 1 << 2;
    /// The first seat after boot read hotter than the NTC through the
    /// stored cold R: the carry started there.
    pub const HOT_BOOT: u8 = 1 << 3;
}

/// `t_winding_cc` while UNSET: under every threshold, so derate and
/// cutoff never act on it.
pub const UNSET_CC: i16 = i16::MIN;

/// Copper's resistance is proportional to `234.5 C + T`.
pub const COPPER_ZERO_CC: i32 = 23450;
/// The temperature `r_cold_q12` is stored at.
pub const R_COLD_REF_CC: i32 = 2500;

/// Guard bits under Q4.12 for the LMS (bemf `state_qg` convention).
const GUARD: u32 = 8;
const R_MAX_QG: i32 = 16 << (12 + GUARD);
/// `e_v` beyond this is garbage input; bounds the LMS product.
const E_MAX: i32 = 1 << 20;
/// Realistic `|v_mean|` (12-bit taps): REF_TICKS of it shifted by 12 stays
/// inside u32.
const V_MAX: i32 = 4095;

/// Guard bits under centi-C for the carried excess.
const XG: u32 = 8;
const X_MAX_QG: i32 = 32767 << XG;

/// A sample is steady when neither `v_mean` nor `i` moved by more than
/// this fraction of itself (a shift) since the previous SLOW tick, for
/// STEADY_TICKS consecutive ticks (32 ms).
const STEADY_SHIFT: u32 = 3;
const STEADY_TICKS: u8 = 2;

/// Gated ticks skipped at a new seat before the reference window opens:
/// 1 s at 62.5 Hz. Every bench anchor phase fell 0.4-0.9% through its
/// first 1.5 s (contact settling under current); heating cannot do that.
const REF_SKIP_TICKS: u8 = 62;
/// Reference window, 2 s: averages the one-vcount quantization of
/// `v_mean` (0.3% of R at the 280-count hold) down to 0.03%.
const REF_TICKS: u8 = 128;
const REF_SHIFT: u32 = 7;
/// Ticks without a gated sample that end a hold (0.5 s): a ratio is
/// same-hold only; a torque-off gap at the same seat moved the contact
/// 0.4-1.2% on the bench.
const GAP_TICKS: u8 = 32;
/// Beyond an eighth of the reference current the kernel's V/I offset
/// (~0.02% per count) would read as temperature.
const I_BAND_SHIFT: u32 = 3;
/// NTC conversions per SLOW tick: one a second is plenty for a board.
const NTC_DECIM: u8 = 64;
/// Cold-R check idle: 11 min without current (41250 ticks), and the NTC
/// flat within NTC_SLOPE_MAX counts over NTC_SLOPE_TICKS (1 min).
const IDLE_TICKS: u16 = 41250;
const NTC_SLOPE_TICKS: u16 = 3750;
const NTC_SLOPE_MAX: u16 = 2;
/// Cold seats the low percentile runs over: contact only adds, and 73% of
/// the bench motor's stops sit in the low cluster, so the minimum of five
/// lands in it with 99.9%.
const COLD_SEATS: usize = 5;

/// 12-bit ADC full scale; a reading at either rail is no reading.
const ADC_FULL: u32 = 4096;

/// Board NTC counts to centi-C through the host's quadratic; None at the
/// rails or without a curve.
pub fn ntc_cc(raw: u16, ntc: &NtcCfg) -> Option<i16> {
    if ntc.k1_q88 == 0 || raw == 0 || raw as u32 >= ADC_FULL - 1 {
        return None;
    }
    let d = ntc.raw_ref as i32 - raw as i32;
    let t =
        ntc.t_ref_cc as i32 + q_mul(d, ntc.k1_q88 as i32, 8) + q_mul(d * d, ntc.k2_q24 as i32, 24);
    (i16::MIN as i32 + 1..=i16::MAX as i32)
        .contains(&t)
        .then_some(t as i16)
}

#[derive(Default)]
pub struct WindingTherm {
    r_qg: i32,
    /// Carried excess over the NTC, centi-C << XG.
    x_qg: i32,
    t_ntc_cc: i16,
    ntc_ok: bool,
    ntc_decim: u8,
    t_cc: i16,
    flags: u8,
    based: bool,
    r_ref_q12: u16,
    k_ref_q88: u16,
    t_ref_cc: i16,
    i_ref: i32,
    pos_ref: i32,
    /// Reference window: gated ticks seen at this seat, the sums.
    seat_ticks: u8,
    sum_v: i32,
    sum_i: i32,
    v_prev: i32,
    i_prev: i32,
    steady: u8,
    gap: u8,
    /// Cold-R check: ticks without current, the NTC a minute ago, the
    /// slope timer, the last COLD_SEATS cold readings.
    idle: u16,
    ntc_min_ago: u16,
    ntc_slope_n: u16,
    ntc_flat: bool,
    /// The idle criterion held when the current last returned.
    cold_ok: bool,
    cold: [u16; COLD_SEATS],
    cold_n: u8,
    first_seat: bool,
}

impl WindingTherm {
    pub const fn new() -> Self {
        Self {
            r_qg: 0,
            x_qg: 0,
            t_ntc_cc: 0,
            ntc_ok: false,
            ntc_decim: 0,
            t_cc: UNSET_CC,
            flags: flag::UNSET,
            based: false,
            r_ref_q12: 0,
            k_ref_q88: 0,
            t_ref_cc: 0,
            i_ref: 0,
            pos_ref: 0,
            seat_ticks: 0,
            sum_v: 0,
            sum_i: 0,
            v_prev: 0,
            i_prev: 0,
            steady: 0,
            gap: 0,
            idle: 0,
            ntc_min_ago: 0,
            ntc_slope_n: 0,
            ntc_flat: false,
            cold_ok: false,
            cold: [0; COLD_SEATS],
            cold_n: 0,
            first_seat: true,
        }
    }

    /// One SLOW-tick update; returns the winding temperature, centi-C, or
    /// [`UNSET_CC`].
    pub fn step(&mut self, s: &Sample, cfg: &ThermCfg) -> i16 {
        self.ntc(s.ntc_raw, cfg);
        if cfg.alpha_q24 == 0 || cfg.g_q016 == 0 || !self.ntc_ok {
            self.x_qg = 0;
            self.drop_base();
            self.flags = flag::UNSET;
            self.t_cc = UNSET_CC;
            return self.t_cc;
        }
        self.flags &= !flag::UNSET;
        let gated = self.gate(s, cfg);
        // the carry runs every tick; a tracked sample overwrites it below
        let v_emf =
            s.v_mean
                .unwrap_or(0)
                .saturating_sub(q_mul(s.ke_vpc_q as i32, s.omega_hat_cps, 12));
        let p = q_mul(s.i_meas.unwrap_or(0), v_emf, 0).max(0);
        let target_qg = q_mul(p, cfg.g_q016 as i32, 16).min(X_MAX_QG >> XG) << XG;
        self.x_qg += q_mul(target_qg - self.x_qg, cfg.alpha_q24 as i32, 24);
        let mut t = self.t_ntc_cc as i32 + (self.x_qg >> XG);
        if self.based {
            if let Some(i) = gated {
                self.track(s.v_mean.unwrap_or(0), i, cfg);
                t = self.t_ref_cc as i32
                    + q_mul(
                        (self.r_qg >> GUARD) - self.r_ref_q12 as i32,
                        self.k_ref_q88 as i32,
                        8,
                    );
                t = t.clamp(i16::MIN as i32, i16::MAX as i32);
                self.x_qg = ((t - self.t_ntc_cc as i32) << XG).clamp(-X_MAX_QG, X_MAX_QG);
            }
            if self.exits(s.i_meas, s.pos_counts, cfg) {
                self.drop_base();
            }
        } else if let Some(i) = gated {
            self.reference(s.v_mean.unwrap_or(0), i, s.pos_counts, t, cfg);
            if self.based {
                t = self.t_ref_cc as i32;
            }
        }
        self.t_cc = t.clamp(UNSET_CC as i32 + 1, i16::MAX as i32) as i16;
        self.t_cc
    }

    /// Once a second: the NTC in centi-C; once a minute: whether it moved.
    fn ntc(&mut self, raw: u16, cfg: &ThermCfg) {
        if self.ntc_decim == 0 {
            match ntc_cc(raw, &cfg.ntc) {
                Some(t) => {
                    self.t_ntc_cc = t;
                    self.ntc_ok = true;
                }
                None => self.ntc_ok = false,
            }
        }
        self.ntc_decim = if self.ntc_decim + 1 < NTC_DECIM {
            self.ntc_decim + 1
        } else {
            0
        };
        if self.ntc_slope_n == 0 {
            self.ntc_flat = raw.abs_diff(self.ntc_min_ago) < NTC_SLOPE_MAX;
            self.ntc_min_ago = raw;
        }
        self.ntc_slope_n = if self.ntc_slope_n + 1 < NTC_SLOPE_TICKS {
            self.ntc_slope_n + 1
        } else {
            0
        };
    }

    /// The sample gate: both windows valid, current over the floor, the
    /// operating point held for STEADY_TICKS. Keeps the idle and gap
    /// counters.
    fn gate(&mut self, s: &Sample, cfg: &ThermCfg) -> Option<i32> {
        let over_floor = s
            .i_meas
            .is_some_and(|i| i.unsigned_abs() > cfg.i_min_counts as u32);
        if over_floor {
            if self.idle != 0 {
                self.cold_ok = self.idle >= IDLE_TICKS && self.ntc_flat;
            }
            self.idle = 0;
        } else {
            self.idle = self.idle.saturating_add(1);
        }
        let gated = match (s.v_mean, s.i_meas) {
            (Some(v), Some(i)) => {
                let held = v.wrapping_sub(self.v_prev).unsigned_abs()
                    <= v.unsigned_abs() >> STEADY_SHIFT
                    && i.wrapping_sub(self.i_prev).unsigned_abs()
                        <= i.unsigned_abs() >> STEADY_SHIFT;
                self.steady = if held {
                    self.steady.saturating_add(1)
                } else {
                    0
                };
                self.v_prev = v;
                self.i_prev = i;
                (over_floor && self.steady >= STEADY_TICKS).then_some(i)
            }
            _ => {
                self.steady = 0;
                None
            }
        };
        self.gap = if gated.is_some() {
            0
        } else {
            self.gap.saturating_add(1)
        };
        gated
    }

    /// Signed LMS on `e = v - R i` (sign-data variant: the step stays
    /// mu-controlled independent of current magnitude).
    fn track(&mut self, v: i32, i: i32, cfg: &ThermCfg) {
        let e_v = v
            .saturating_sub(q_mul(self.r_qg >> GUARD, i, 12))
            .clamp(-E_MAX, E_MAX);
        let e_signed = if i < 0 { -e_v } else { e_v };
        let step = q_mul((cfg.mu_q016 as i32) << GUARD, e_signed, 16);
        self.r_qg = (self.r_qg + step).clamp(0, R_MAX_QG);
    }

    /// Why a tracked seat ends: the shaft left the band, the current moved
    /// an eighth, or the samples stopped.
    fn exits(&self, i_meas: Option<i32>, pos: i32, cfg: &ThermCfg) -> bool {
        let moved = (pos - self.pos_ref).unsigned_abs() > cfg.seat_band_counts as u32;
        let shifted = i_meas.is_some_and(|i| {
            (i - self.i_ref).unsigned_abs() > self.i_ref.unsigned_abs() >> I_BAND_SHIFT
        });
        moved || shifted || self.gap >= GAP_TICKS
    }

    fn drop_base(&mut self) {
        self.based = false;
        self.seat_ticks = 0;
        self.sum_v = 0;
        self.sum_i = 0;
        self.flags &= !flag::TRACK;
    }

    /// A gated tick at an unbased seat: skip the contact transient, then
    /// average REF_TICKS into the reference. `t` is the carried
    /// temperature the reference inherits. Out of line: the tick body
    /// takes it once per seat.
    #[inline(never)]
    fn reference(&mut self, v: i32, i: i32, pos: i32, t: i32, cfg: &ThermCfg) {
        self.seat_ticks = self.seat_ticks.saturating_add(1);
        if self.seat_ticks <= REF_SKIP_TICKS {
            self.pos_ref = pos;
            return;
        }
        if (pos - self.pos_ref).unsigned_abs() > cfg.seat_band_counts as u32 {
            self.drop_base();
            return;
        }
        self.sum_v += v.clamp(-V_MAX, V_MAX);
        self.sum_i += i;
        if self.seat_ticks - REF_SKIP_TICKS < REF_TICKS {
            return;
        }
        let r_ref = recip_div(
            self.sum_v.unsigned_abs() << 12,
            self.sum_i.unsigned_abs().max(1),
        )
        .min(u16::MAX as u32) as u16;
        let mut t_ref = t;
        if self.first_seat && cfg.r_cold_q12 != 0 {
            // the first seat after boot: the stored cold R says how hot
            // the winding is; hotter than the NTC by more than the band is
            // a hot reboot, and the carry starts there
            let k_cold = recip_div(
                ((COPPER_ZERO_CC + R_COLD_REF_CC) as u32) << 8,
                cfg.r_cold_q12 as u32,
            );
            let t_r0 =
                R_COLD_REF_CC + q_mul(r_ref as i32 - cfg.r_cold_q12 as i32, k_cold as i32, 8);
            let band = q_mul(
                COPPER_ZERO_CC + R_COLD_REF_CC,
                cfg.cold_band_q016 as i32,
                16,
            );
            if t_r0 - self.t_ntc_cc as i32 > band {
                t_ref = t_r0;
                self.flags |= flag::HOT_BOOT;
            }
        }
        self.first_seat = false;
        if self.cold_ok && cfg.r_cold_q12 != 0 {
            self.cold_check(r_ref, cfg);
        }
        let t_ref = t_ref.clamp(1 - COPPER_ZERO_CC, i16::MAX as i32);
        self.k_ref_q88 = recip_div(((COPPER_ZERO_CC + t_ref) as u32) << 8, r_ref.max(1) as u32)
            .min(u16::MAX as u32) as u16;
        self.r_ref_q12 = r_ref;
        self.r_qg = (r_ref as i32) << GUARD;
        self.t_ref_cc = t_ref as i16;
        self.i_ref = self.sum_i >> REF_SHIFT;
        self.x_qg = ((t_ref - self.t_ntc_cc as i32) << XG).clamp(-X_MAX_QG, X_MAX_QG);
        self.based = true;
        self.flags |= flag::TRACK;
    }

    /// A reference taken after the idle criterion is a cold reading: reduce
    /// it to R_COLD_REF_CC through copper and the NTC, keep the minimum of
    /// the last COLD_SEATS (contact only adds), flag a divergence from the
    /// stored cold R beyond the band, either way.
    #[inline(never)]
    fn cold_check(&mut self, r_ref: u16, cfg: &ThermCfg) {
        let f_q16 = recip_div(
            ((COPPER_ZERO_CC + R_COLD_REF_CC) as u32) << 16,
            (COPPER_ZERO_CC + self.t_ntc_cc as i32).max(1) as u32,
        );
        let r25 = q_mul_u(r_ref as u32, f_q16, 16).min(u16::MAX as u32) as u16;
        let slot = self.cold_n as usize % COLD_SEATS;
        self.cold[slot] = r25;
        self.cold_n = self.cold_n.saturating_add(1);
        if (self.cold_n as usize) < COLD_SEATS {
            return;
        }
        let min = self.cold.iter().copied().min().unwrap_or(0) as i32;
        let band = q_mul(cfg.r_cold_q12 as i32, cfg.cold_band_q016 as i32, 16);
        if (min - cfg.r_cold_q12 as i32).abs() > band {
            self.flags |= flag::COLD_RECAL;
        }
    }

    /// Saturating cast matching telemetry `est.r_hat_q12`.
    pub fn r_q12(&self) -> u16 {
        (self.r_qg >> GUARD).clamp(0, u16::MAX as i32) as u16
    }

    pub fn t_cc(&self) -> i16 {
        self.t_cc
    }

    /// The board NTC as last converted, centi-C (0 before the first).
    pub fn t_ntc_cc(&self) -> i16 {
        self.t_ntc_cc
    }

    pub fn flags(&self) -> u8 {
        self.flags
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The beta model of the dev-v006 board's NTC (10K pull-up, 10K at
    /// 25 C, beta 3950), as the host evaluates it.
    fn beta_c(raw: f64) -> f64 {
        let r_over_r25 = raw / (4096.0 - raw);
        1.0 / (1.0 / 298.15 + r_over_r25.ln() / 3950.0) - 273.15
    }

    /// The host's quadratic about 25 C: the central-difference slope at
    /// the reference and the curvature that lands the +18 C point.
    fn ntc_from_beta() -> NtcCfg {
        let raw_ref = 2048.0;
        let k1 = (beta_c(raw_ref - 100.0) - beta_c(raw_ref + 100.0)) / 200.0 * 100.0;
        let d = 800.0;
        let k2 = ((beta_c(raw_ref - d) - 25.0) * 100.0 - k1 * d) / (d * d);
        NtcCfg {
            raw_ref: raw_ref as u16,
            t_ref_cc: 2500,
            k1_q88: (k1 * 256.0).round() as i16,
            k2_q24: (k2 * 16_777_216.0).round() as u16,
        }
    }
    /// The dev-v006 board's NTC and the MG90 family model: tau 128 s
    /// (alpha 2097 Q0.24 at 62.5 Hz), 63 C/W over the board NTC (g 1500
    /// Q0.16 through 4.0283 mV and 0.90216 mA per count), mu for a 2 s
    /// settle at 280 counts, cold R 4128 Q4.12 at 25 C.
    const NTC: NtcCfg = NtcCfg {
        raw_ref: 2048,
        t_ref_cc: 2500,
        k1_q88: 563,
        k2_q24: 5800,
    };
    pub(crate) const CFG: ThermCfg = ThermCfg {
        alpha_q24: 2097,
        g_q016: 1500,
        mu_q016: 7670,
        r_cold_q12: 4128,
        i_min_counts: 167,
        seat_band_counts: 2,
        cold_band_q016: 1966,
        ntc: NTC,
    };
    /// Counts at 25.00 C for a 10K/10K divider.
    const RAW_25: u16 = 2048;
    /// SLOW ticks per tau at alpha 2097.
    const TAU_TICKS: usize = 8000;

    fn seated(v: i32, i: i32, pos: i32) -> Sample {
        Sample {
            v_mean: Some(v),
            i_meas: Some(i),
            omega_hat_cps: 0,
            ke_vpc_q: 350,
            pos_counts: pos,
            ntc_raw: RAW_25,
        }
    }

    fn off(pos: i32) -> Sample {
        Sample {
            v_mean: None,
            i_meas: None,
            omega_hat_cps: 0,
            ke_vpc_q: 350,
            pos_counts: pos,
            ntc_raw: RAW_25,
        }
    }

    /// The R the reference window reads back from `v = q_mul(r, i, 12)`
    /// (v_mean truncates to a vcount: 0.07% of R at 280 counts).
    fn r_read(r: i32, i: i32) -> u16 {
        (q_mul(r, i, 12) * 4096 / i) as u16
    }

    /// The winding R of a copper winding `r25` at `t_cc`, Q4.12.
    fn r_at(r25: i32, t_cc: i32) -> i32 {
        (r25 as i64 * (COPPER_ZERO_CC + t_cc) as i64 / (COPPER_ZERO_CC + R_COLD_REF_CC) as i64)
            as i32
    }

    fn hold(th: &mut WindingTherm, n: usize, s: &Sample) -> i16 {
        let mut t = 0;
        for _ in 0..n {
            t = th.step(s, &CFG);
        }
        t
    }

    /// Ticks until a reference is taken at a fresh seat.
    const BASE_TICKS: usize = STEADY_TICKS as usize + REF_SKIP_TICKS as usize + REF_TICKS as usize;

    #[test]
    fn ntc_quadratic_tracks_the_beta_curve_within_a_degree_from_15_to_50_c() {
        let ntc = ntc_from_beta();
        assert!(
            (ntc.k1_q88 as i32 - NTC.k1_q88 as i32).abs() <= 8,
            "{}",
            ntc.k1_q88
        );
        assert!(
            (ntc.k2_q24 as i32 - NTC.k2_q24 as i32).abs() <= 400,
            "{}",
            ntc.k2_q24
        );
        for t_c in [15.0f64, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0] {
            let r = 10_000.0 * (3950.0 * (1.0 / (t_c + 273.15) - 1.0 / 298.15)).exp();
            let raw = (4096.0 * r / (10_000.0 + r)).round();
            let t_fw = ntc_cc(raw as u16, &ntc).expect("in range") as f64 / 100.0;
            assert!((t_fw - beta_c(raw)).abs() < 0.7, "{t_c} C: {t_fw}");
        }
        assert_eq!(ntc_cc(0, &NTC), None);
        assert_eq!(ntc_cc(4095, &NTC), None);
        assert_eq!(ntc_cc(2048, &NtcCfg::default()), None);
    }

    #[test]
    fn unset_reads_the_sentinel_and_never_a_temperature() {
        let mut th = WindingTherm::new();
        let s = seated(q_mul(4128, 280, 12), 280, 300);
        let no_model = ThermCfg {
            alpha_q24: 0,
            ..CFG
        };
        for _ in 0..BASE_TICKS + 10 {
            assert_eq!(th.step(&s, &no_model), UNSET_CC);
        }
        assert_eq!(th.flags() & flag::UNSET, flag::UNSET);
        let no_ntc = ThermCfg {
            ntc: NtcCfg::default(),
            ..CFG
        };
        // the curve is re-read once a second
        for _ in 0..NTC_DECIM {
            th.step(&s, &no_ntc);
        }
        assert_eq!(th.step(&s, &no_ntc), UNSET_CC);
        // the model lands: the sentinel leaves within a second, the NTC
        // is the base
        hold(&mut th, NTC_DECIM as usize, &s);
        assert!((2500..2510).contains(&th.t_cc()), "{}", th.t_cc());
        assert_eq!(th.flags() & flag::UNSET, 0);
    }

    #[test]
    fn a_seat_bases_on_the_ntc_and_reads_the_ratio_rise() {
        let mut th = WindingTherm::new();
        let r0 = r_at(4128, 2500);
        let s = seated(q_mul(r0, 280, 12), 280, 300);
        hold(&mut th, BASE_TICKS, &s);
        assert_eq!(th.flags() & flag::TRACK, flag::TRACK);
        assert_eq!(th.r_q12(), r_read(r0, 280));
        // the base inherits the carry: 3 s at 0.25 W reads under 0.6 C
        let t0 = th.t_cc();
        assert!((2500..2560).contains(&t0), "{t0}");
        // the winding at +20 C: the ratio reads it within a degree once
        // the LMS settles
        let hot = seated(q_mul(r_at(4128, 4500), 280, 12), 280, 300);
        let t = hold(&mut th, 500, &hot);
        assert!((t - (t0 + 2000)).abs() <= 100, "{t} from {t0}");
    }

    #[test]
    fn a_seat_change_rebases_without_a_temperature_step() {
        let mut th = WindingTherm::new();
        let r0 = r_at(4128, 2500);
        hold(
            &mut th,
            BASE_TICKS + 200,
            &seated(q_mul(r0, 280, 12), 280, 300),
        );
        let before = th.t_cc();
        // the shaft moves 40 counts and seats with +1% of contact: the old
        // thermometer read +2.6 C
        let r1 = r0 + r0 / 100;
        let moved = seated(q_mul(r1, 280, 12), 280, 340);
        th.step(&moved, &CFG);
        assert_eq!(th.flags() & flag::TRACK, 0);
        let t = hold(&mut th, BASE_TICKS + 200, &moved);
        assert!((t - before).abs() < 50, "{t} from {before}");
        assert!(th.r_q12().abs_diff(r_read(r1, 280)) <= 1, "{}", th.r_q12());
    }

    #[test]
    fn a_current_shift_at_the_seat_rebases() {
        let mut th = WindingTherm::new();
        let r0 = r_at(4128, 2500);
        hold(
            &mut th,
            BASE_TICKS + 100,
            &seated(q_mul(r0, 280, 12), 280, 300),
        );
        // the derate drops the hold to 200 counts: the kernel's V/I offset
        // would read as cooling; the base is dropped instead
        let derated = seated(q_mul(r0, 200, 12) - 20, 200, 300);
        th.step(&derated, &CFG);
        assert_eq!(th.flags() & flag::TRACK, 0);
    }

    /// The documented blind spot: a +1.5% contact step inside one hold
    /// reads as +3.9 C until the seat is left.
    #[test]
    fn a_contact_step_inside_a_hold_reads_as_heat_until_the_rebase() {
        let mut th = WindingTherm::new();
        let r0 = r_at(4128, 2500);
        hold(
            &mut th,
            BASE_TICKS + 200,
            &seated(q_mul(r0, 280, 12), 280, 300),
        );
        let before = th.t_cc();
        let stepped = seated(q_mul(r0 + r0 * 15 / 1000, 280, 12), 280, 300);
        let t = hold(&mut th, 400, &stepped);
        assert!((340..450).contains(&(t - before)), "{t} from {before}");
        assert_eq!(th.flags() & flag::TRACK, flag::TRACK);
        // the next seat re-bases from the carry, which took the step too:
        // it persists until a long idle washes it out
        let next = seated(q_mul(r0 + r0 * 15 / 1000, 280, 12), 280, 340);
        let t2 = hold(&mut th, BASE_TICKS + 100, &next);
        assert!((t2 - t).abs() < 60, "{t2} from {t}");
    }

    #[test]
    fn torque_off_carries_the_excess_to_the_ntc_on_tau() {
        let mut th = WindingTherm::new();
        let r0 = r_at(4128, 2500);
        hold(&mut th, BASE_TICKS, &seated(q_mul(r0, 280, 12), 280, 300));
        hold(
            &mut th,
            800,
            &seated(q_mul(r_at(4128, 6500), 280, 12), 280, 300),
        );
        let hot = th.t_cc();
        assert!(hot > 6400, "{hot}");
        // one tau: 37% of the excess remains; five: under 1%
        let t1 = hold(&mut th, TAU_TICKS, &off(300));
        let x1 = (t1 - 2500) as f64 / (hot - 2500) as f64;
        assert!((x1 - 0.368).abs() < 0.02, "{x1}");
        let t5 = hold(&mut th, 4 * TAU_TICKS, &off(300));
        assert!(t5 - 2500 < 40, "{t5}");
        assert_eq!(th.flags() & flag::TRACK, 0);
    }

    #[test]
    fn hot_reboot_starts_the_carry_from_the_cold_r() {
        let mut th = WindingTherm::new();
        // the winding at 65 C through the stored cold R, at a fresh boot
        let r_hot = r_at(4128, 6500);
        let t = hold(
            &mut th,
            BASE_TICKS,
            &seated(q_mul(r_hot, 280, 12), 280, 300),
        );
        assert!((6400..6600).contains(&t), "{t}");
        assert_eq!(th.flags() & flag::HOT_BOOT, flag::HOT_BOOT);
        // a second seat is not a boot: +2% of contact there reads from
        // the carry, not through the cold R
        th.step(&seated(q_mul(r_hot, 280, 12), 280, 400), &CFG);
        hold(
            &mut th,
            BASE_TICKS,
            &seated(q_mul(r_hot + r_hot / 50, 280, 12), 280, 400),
        );
        assert!((th.t_cc() - t).abs() < 100, "{} from {t}", th.t_cc());
    }

    #[test]
    fn cold_check_flags_a_shifted_cloud_after_five_idle_seats() {
        let mut th = WindingTherm::new();
        // five long-idle seats, each +7% over the stored cold R (the bench
        // motor's shift after its first running)
        let r_shift = r_at(4128, 2500) * 107 / 100;
        for seat in 0..COLD_SEATS as i32 {
            hold(&mut th, IDLE_TICKS as usize + 1, &off(300 + 50 * seat));
            hold(
                &mut th,
                BASE_TICKS,
                &seated(q_mul(r_shift, 280, 12), 280, 300 + 50 * seat),
            );
            if seat + 1 < COLD_SEATS as i32 {
                assert_eq!(th.flags() & flag::COLD_RECAL, 0, "seat {seat}");
            }
        }
        assert_eq!(th.flags() & flag::COLD_RECAL, flag::COLD_RECAL);
        // a warm winding after a short idle is not a cold reading
        let mut th = WindingTherm::new();
        for seat in 0..COLD_SEATS as i32 {
            hold(&mut th, 2000, &off(300 + 50 * seat));
            hold(
                &mut th,
                BASE_TICKS,
                &seated(q_mul(r_shift, 280, 12), 280, 300 + 50 * seat),
            );
        }
        assert_eq!(th.flags() & flag::COLD_RECAL, 0);
    }

    /// The brief's gap pin against notebook 18's two-node plant (winding
    /// 1.3 J/C over 35 C/W to a 4.0 J/C can over 32 C/W to air): a 150 s
    /// hold at 0.27 W, then 60 s of loaded motion at 0.18 W with no seat,
    /// carried by the one-node family model (tau 128 s, 63 C/W). The
    /// plant's winding node (tau 45 s) drops toward its lower equilibrium
    /// faster than the family tau lets the carry follow, so the carry
    /// OVER-reads the plant through the gap, by well under the 3 C budget
    /// (0.7 C here); the next seat re-bases on the carried value. A family
    /// R_th fitted low would flip the sign: this pin holds it.
    #[test]
    fn a_sixty_second_gap_is_carried_within_three_degrees_over_reading() {
        const DT: f64 = 1.0 / 62.5;
        struct Plant {
            tw: f64,
            tc: f64,
        }
        impl Plant {
            fn step(&mut self, p_w: f64) -> f64 {
                let (c_w, r_wc, c_c, r_ca) = (1.3f64, 35.0f64, 4.0f64, 32.0f64);
                let q_wc = (self.tw - self.tc) / r_wc;
                self.tw += DT * (p_w - q_wc) / c_w;
                self.tc += DT * (q_wc - self.tc / r_ca) / c_c;
                self.tw
            }
        }
        let mut plant = Plant { tw: 0.0, tc: 0.0 };
        // 0.27 W: 280 counts into 4.4652 ohm per Q12 unit through the
        // board's 3.634e-6 W per vcount-ccount
        let w_per_unit = 4.0283e-3 * 0.90216e-3;
        let mut th = WindingTherm::new();
        let mut t_fw = 0;
        for _ in 0..150 * 62 {
            let x = plant.step(0.27);
            let r = r_at(4128, 2500 + (x * 100.0) as i32);
            t_fw = th.step(&seated(q_mul(r, 280, 12), 280, 300), &CFG);
        }
        let x_hold = plant.tw;
        assert!(
            (t_fw - 2500) as f64 / 100.0 - x_hold > -1.5,
            "{t_fw} vs {x_hold}"
        );
        // the move: 0.18 W, the shaft never at one seat for a second
        let i = 230;
        let v = (0.18 / w_per_unit / i as f64) as i32;
        let mut pos = 300;
        for _ in 0..60 * 62 {
            plant.step(0.18);
            pos += 5;
            t_fw = th.step(&seated(v, i, pos), &CFG);
        }
        assert_eq!(th.flags() & flag::TRACK, 0);
        let err = (t_fw - 2500) as f64 / 100.0 - plant.tw;
        assert!(err > 0.0, "the carry under-read the plant: {err}");
        assert!(err < 3.0, "the carry ran {err} C over the plant");
    }
}
