//! The winding's thermal constants from one seated hold, for the kernel's
//! carry (core `estimator::thermal`). Per SLOW tick the carry runs
//! `x += alpha (g P - x)` on the excess `x` over the board NTC, centi-C,
//! with `P = i (v_mean - Ke omega)` in vcount-ccount, `alpha` Q0.24 and
//! `g` Q0.16. Inside a hold the same-seat ratio reads the real excess to
//! about a degree, so the hold's own rise identifies both. In continuous
//! time the recursion is `dx/dt = a P + b x` with `b = -alpha f_slow` and
//! `a = alpha f_slow g` (alpha is about 1e-4: per tick the discrete and
//! continuous decays differ by alpha^2 / 2), and a least-squares fit of
//! the rise rate on `(P, x)` returns both.
//!
//! Only ticks the ratio read are fitted: untracked, the kernel reports its
//! carry, which is the table's model, not the winding. The first
//! [`BASE_SKIP_S`] of every base and the [`EXIT_GUARD_S`] before every
//! exit are cut too. The rise is differenced over [`CHUNK_S`] chunks
//! against each chunk's time means of `P` and `x`: the recursion
//! integrated over the chunk, so the 1 centi-C steps of `t_winding_cc` do
//! not swamp a per-tick rise of a fraction of one.

use core::fmt::Write as _;

use osc_servo_core::estimator::thermal::{UNSET_CC, flag};

use crate::exp::anchor::contact_state;
use crate::frame::TelemetrySnapshot;
use crate::gains::{Encoded, enc};
use crate::thermometer::{self, SETTLE_S};

const Q15: f64 = 32767.0;
const Q24: f64 = 16_777_216.0;
const Q016: f64 = 65536.0;

/// Cut after every base: its R is the 2 s reference window's mean, about
/// a second behind the winding, and the LMS closes that lag on SETTLE_S.
pub const BASE_SKIP_S: f64 = 3.0;
/// Cut before every exit: the kernel drops a base up to its 1 s rise-rate
/// window after a contact step, and the LMS read the step as heat for
/// that long.
pub const EXIT_GUARD_S: f64 = 2.0;
/// One LMS settle per chunk: the tracker's output is correlated over it,
/// so a shorter difference adds noise and no information.
pub const CHUNK_S: f64 = SETTLE_S;
/// Chunks whose rise sits this many residual sd off the first fit are
/// dropped once and the fit redone.
pub const OUTLIER_SD: f64 = 3.0;
/// The excess over the NTC a fit starts from, at most, centi-C: the first
/// base inherits the carried excess and every tracked reading after it
/// counts from there, so a carry still off the winding offsets the hold.
pub const REST_CC: f64 = 100.0;
/// Around the MG90 family's 128 s and 63 C/W: a fit outside is not a
/// winding's warm-up and is never written.
pub const TAU_BAND_S: (f64, f64) = (30.0, 600.0);
pub const R_TH_BAND_C_PER_W: (f64, f64) = (10.0, 200.0);

/// A tracked R that moves this much across a guard exit is a contact
/// state, not heat: copper moves 0.39% per C, and the bench's brush bridge
/// read 23% low.
pub const R_STEP: f64 = 0.10;

/// The two slopes the fit solves for.
const PARAMS: usize = 2;

/// One read of the hold, in the kernel's units.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Row {
    /// Host time of the read, s.
    pub t_s: f64,
    /// `t_winding_cc - t_ntc_cc`.
    pub x_cc: f64,
    /// The ident window's current, counts, and duty-weighted `duty x
    /// vdiff`, vcounts, both folded by the drive's sign; the duty, a
    /// fraction of full scale.
    pub i: f64,
    pub v: f64,
    pub duty: f64,
    /// `i v`, vcount-ccount: the kernel's `P` at a seat, where omega is
    /// zero.
    pub p: f64,
    pub flags: u8,
}

impl Row {
    pub fn from_snapshot(o: &TelemetrySnapshot) -> Self {
        let sign = o.duty_mean_q15.signum() as f64;
        let duty = o.duty_mean_q15.unsigned_abs() as f64 / Q15;
        let (i, v) = (
            o.i_mean_counts as f64 * sign,
            duty * o.vdiff_mean as f64 * sign,
        );
        Self {
            t_s: o.host_ms / 1000.0,
            x_cc: o.t_winding_cc as f64 - o.t_ntc_cc as f64,
            i,
            v,
            duty,
            p: (i * v).max(0.0),
            flags: o.therm_flags,
        }
    }

    fn tracked(&self) -> bool {
        self.flags & (flag::TRACK | flag::UNSET) == flag::TRACK
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Fit {
    /// Rise rate per unit of `P`, centi-C/s per vcount-ccount.
    pub a: f64,
    /// Rise rate per centi-C of excess, 1/s.
    pub b: f64,
    /// Residual sd of the chunks' rise rates, centi-C/s.
    pub sd: f64,
    pub chunks: usize,
    pub rejected: usize,
    /// Tracked hold the chunks cover, s.
    pub used_s: f64,
    /// Tracked stretches, and the stretches between them the fit cut.
    pub segments: usize,
    pub gaps: usize,
    /// Guard exits the tracked R stepped more than [`R_STEP`] across.
    pub r_steps: usize,
}

impl Fit {
    pub fn tau_s(&self) -> f64 {
        -1.0 / self.b
    }

    /// The steady excess per unit of `P`, centi-C per vcount-ccount: the
    /// carry's `g`.
    pub fn g_cc(&self) -> f64 {
        -self.a / self.b
    }

    pub fn r_th(&self, u: &Units) -> f64 {
        thermometer::r_th_c_per_w(self.g_cc(), u.v_per_vcount, u.a_per_ccount)
    }

    /// `th_alpha_q24` and `th_g_q016` at the kernel's SLOW rate.
    pub fn encode(&self, slow_hz: f64) -> (Encoded, Encoded) {
        (
            enc(thermometer::alpha(self.tau_s(), slow_hz), Q24),
            enc(self.g_cc(), Q016),
        )
    }
}

#[derive(Copy, Clone)]
struct Chunk {
    rate: f64,
    p: f64,
    x: f64,
}

/// Least squares of the rise rate on `(P, x)`, no intercept: the carry
/// has none.
fn solve(pts: &[Chunk]) -> Option<(f64, f64)> {
    let (mut spp, mut spx, mut sxx, mut spy, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for c in pts {
        spp += c.p * c.p;
        spx += c.p * c.x;
        sxx += c.x * c.x;
        spy += c.p * c.rate;
        sxy += c.x * c.rate;
    }
    let det = spp * sxx - spx * spx;
    (det > 0.0).then(|| ((sxx * spy - spx * sxy) / det, (spp * sxy - spx * spy) / det))
}

fn residual(c: &Chunk, (a, b): (f64, f64)) -> f64 {
    c.rate - a * c.p - b * c.x
}

fn sd(pts: &[Chunk], ab: (f64, f64)) -> f64 {
    let ss: f64 = pts.iter().map(|c| residual(c, ab).powi(2)).sum();
    (ss / (pts.len() - PARAMS) as f64).sqrt()
}

/// The chunks of one tracked stretch, inside its cuts, and the time they
/// cover.
fn stretch_chunks(seg: &[Row], out: &mut Vec<Chunk>) -> f64 {
    let (Some(first), Some(last)) = (seg.first(), seg.last()) else {
        return 0.0;
    };
    let (t0, t1) = (first.t_s + BASE_SKIP_S, last.t_s - EXIT_GUARD_S);
    let rows: Vec<&Row> = seg.iter().filter(|r| (t0..=t1).contains(&r.t_s)).collect();
    let mut used = 0.0;
    let mut k = 0;
    while k + 1 < rows.len() {
        let start = k;
        while k + 1 < rows.len() && rows[k].t_s - rows[start].t_s < CHUNK_S {
            k += 1;
        }
        let c = &rows[start..=k];
        let dt = rows[k].t_s - rows[start].t_s;
        if dt < CHUNK_S {
            break;
        }
        let (mut ip, mut ix) = (0.0, 0.0);
        for w in c.windows(2) {
            let h = w[1].t_s - w[0].t_s;
            ip += h * (w[0].p + w[1].p) / 2.0;
            ix += h * (w[0].x_cc + w[1].x_cc) / 2.0;
        }
        out.push(Chunk {
            rate: (rows[k].x_cc - rows[start].x_cc) / dt,
            p: ip / dt,
            x: ix / dt,
        });
        used += dt;
    }
    used
}

/// V over I across rows with current: the hold's R.
fn r_of<'a>(rows: impl Iterator<Item = &'a Row>) -> Option<f64> {
    let (v, i) = rows
        .filter(|r| r.i > 0.0)
        .fold((0.0, 0.0), |(v, i), r| (v + r.v, i + r.i));
    (i > 0.0).then(|| v / i)
}

/// The hold's mean V/I and duty against the identified winding `r_vpc`
/// on the rail `vbus`, vcounts ([`contact_state`]); Err is the refusal.
pub fn check_contact(rows: &[Row], r_vpc: f64, vbus: f64) -> Result<(), String> {
    let held: Vec<&Row> = rows.iter().filter(|r| r.i > 0.0).collect();
    let r = r_of(held.iter().copied()).ok_or("the hold read no current")?;
    let n = held.len() as f64;
    let duty = held.iter().map(|r| r.duty).sum::<f64>() / n;
    let i = held.iter().map(|r| r.i).sum::<f64>() / n;
    contact_state(r, duty, i, r_vpc, vbus)
}

/// Fit the hold's rows, in time order; Err says why nothing fits.
pub fn fit(rows: &[Row]) -> Result<Fit, String> {
    let mut segments = 0;
    let mut r_steps = 0;
    let mut last_r: Option<f64> = None;
    let mut pts = Vec::new();
    let mut used_s = 0.0;
    let mut rest = rows;
    while let Some(s) = rest.iter().position(Row::tracked) {
        let n = rest[s..]
            .iter()
            .position(|r| !r.tracked())
            .unwrap_or(rest.len() - s);
        let seg = &rest[s..s + n];
        used_s += stretch_chunks(seg, &mut pts);
        let (Some(t0), Some(t1)) = (seg.first().map(|r| r.t_s), seg.last().map(|r| r.t_s)) else {
            break;
        };
        let head = r_of(seg.iter().filter(|r| r.t_s - t0 <= CHUNK_S));
        if let (Some(before), Some(after)) = (last_r, head)
            && (after / before - 1.0).abs() > R_STEP
        {
            r_steps += 1;
        }
        last_r = r_of(seg.iter().filter(|r| t1 - r.t_s <= CHUNK_S));
        segments += 1;
        rest = &rest[s + n..];
    }
    if pts.len() <= PARAMS {
        return Err(format!(
            "the hold tracked {segments} stretch(es) giving {} chunk(s) of {CHUNK_S:.0} s after \
             the cuts, too few for two constants",
            pts.len()
        ));
    }
    let degenerate = || "the power and the excess moved together: tau and R_th do not separate";
    let first = solve(&pts).ok_or_else(degenerate)?;
    let band = OUTLIER_SD * sd(&pts, first);
    let kept: Vec<Chunk> = pts
        .iter()
        .copied()
        .filter(|c| residual(c, first).abs() <= band)
        .collect();
    if kept.len() <= PARAMS {
        return Err(format!(
            "{} of {} chunks sit over {OUTLIER_SD} sd off the fit",
            pts.len() - kept.len(),
            pts.len()
        ));
    }
    let (a, b) = solve(&kept).ok_or_else(degenerate)?;
    if !(a > 0.0 && b < 0.0) {
        return Err(format!(
            "the excess did not rise toward a level: {a:.3e} centi-C/s per vcount-ccount, \
             {b:.3e} per s"
        ));
    }
    Ok(Fit {
        a,
        b,
        sd: sd(&kept, (a, b)),
        chunks: pts.len(),
        rejected: pts.len() - kept.len(),
        used_s,
        segments,
        gaps: segments.saturating_sub(1),
        r_steps,
    })
}

/// Tau and R_th inside the band, or why not.
pub fn plausible(tau_s: f64, r_th_c_per_w: f64) -> Result<(), String> {
    let inside = |x: f64, (lo, hi): (f64, f64)| (lo..=hi).contains(&x);
    if inside(tau_s, TAU_BAND_S) && inside(r_th_c_per_w, R_TH_BAND_C_PER_W) {
        return Ok(());
    }
    Err(format!(
        "tau {tau_s:.1} s, R_th {r_th_c_per_w:.1} C/W: outside tau {:.0}-{:.0} s, R_th \
         {:.0}-{:.0} C/W",
        TAU_BAND_S.0, TAU_BAND_S.1, R_TH_BAND_C_PER_W.0, R_TH_BAND_C_PER_W.1
    ))
}

/// What the servo reads before the hold.
#[derive(Copy, Clone, Debug, Default)]
pub struct Rest {
    pub ntc_k1_q88: i16,
    pub r0_q12: u16,
    pub therm_flags: u8,
    pub t_winding_cc: i16,
    pub t_ntc_cc: i16,
    pub alpha_q24: u16,
}

/// The servo can start a fit: a thermometer that reads, a winding at the
/// NTC. Err is the one-line reason, with how long to rest when warm.
pub fn rested(r: &Rest, slow_hz: f64) -> Result<(), String> {
    if r.ntc_k1_q88 == 0 {
        return Err(
            "the servo carries no board NTC curve, the base the excess is read over: `osc ident \
             anchor` writes it"
                .into(),
        );
    }
    if r.r0_q12 == 0 {
        return Err(
            "the servo stores no cold R, so its thermometer has no bound: `osc ident anchor \
             --rested` after 11 min without current stores it"
                .into(),
        );
    }
    if r.therm_flags & flag::UNSET != 0 || r.t_winding_cc == UNSET_CC || r.alpha_q24 == 0 {
        return Err(
            "the winding thermometer reads its sentinel, it carries no model: `osc ident \
             anchor` writes the family's"
                .into(),
        );
    }
    let x = r.t_winding_cc as f64 - r.t_ntc_cc as f64;
    if x.abs() > REST_CC {
        let tau = thermometer::tau_s(r.alpha_q24 as f64 / Q24, slow_hz);
        return Err(format!(
            "the winding reads {:.1} C over the board NTC, over the {:.0} C a fit starts from: \
             rest it torque off about {:.0} s more",
            x / 100.0,
            REST_CC / 100.0,
            (tau * (x.abs() / REST_CC).ln()).ceil()
        ));
    }
    Ok(())
}

/// The board's sense scale and rate the report converts through.
#[derive(Copy, Clone, Debug)]
pub struct Units {
    pub slow_hz: f64,
    pub v_per_vcount: f64,
    pub a_per_ccount: f64,
}

impl Units {
    pub fn tau_s(&self, alpha_q24: f64) -> f64 {
        thermometer::tau_s(alpha_q24 / Q24, self.slow_hz)
    }

    pub fn r_th(&self, g_q016: f64) -> f64 {
        thermometer::r_th_c_per_w(g_q016 / Q016, self.v_per_vcount, self.a_per_ccount)
    }
}

/// The report section: the fit beside the table's `(alpha_q24, g_q016)`.
pub fn render(fit: &Result<Fit, String>, table: (u16, u16), u: &Units) -> String {
    let mut s = String::new();
    let _ = writeln!(
        s,
        "\n[thermal] the winding's excess over the board NTC through one hold"
    );
    let f = match fit {
        Ok(f) => f,
        Err(why) => {
            let _ = writeln!(s, "  declined      {why}");
            return s;
        }
    };
    let _ = writeln!(
        s,
        "  fit           {:.0} s of tracked hold ({:.0} ticks) in {} stretch(es), {} cut between \
         them; {} of {} chunks rejected; residual sd {:.3} centi-C/tick",
        f.used_s,
        f.used_s * u.slow_hz,
        f.segments,
        f.gaps,
        f.rejected,
        f.chunks,
        f.sd / u.slow_hz
    );
    let _ = writeln!(
        s,
        "  contact       {} step(s) of the tracked R over {:.0}% across the guard exits",
        f.r_steps,
        R_STEP * 100.0
    );
    let (alpha, g) = f.encode(u.slow_hz);
    let _ = writeln!(
        s,
        "  th_alpha_q24  fitted {:>6} (tau {:.1} s)      table {:>6} (tau {:.1} s)",
        alpha.raw,
        f.tau_s(),
        table.0,
        u.tau_s(table.0 as f64)
    );
    let r_th = f.r_th(u);
    let _ = writeln!(
        s,
        "  th_g_q016     fitted {:>6} (R_th {:.1} C/W)  table {:>6} (R_th {:.1} C/W)",
        g.raw,
        r_th,
        table.1,
        u.r_th(table.1 as f64)
    );
    let _ = match plausible(f.tau_s(), r_th) {
        Ok(()) => writeln!(s, "  verdict       inside the band"),
        Err(why) => writeln!(s, "  verdict       {why}; nothing is written"),
    };
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The dev-v006 rev 2A board: 4.0283 mV per terminal count, 0.90216 mA
    /// per current count, the kernel's SLOW rate at 20 kHz.
    const UNITS: Units = Units {
        slow_hz: 62.5,
        v_per_vcount: 4.0283e-3,
        a_per_ccount: 0.90216e-3,
    };

    /// The family's table values convert to its constants and back:
    /// 2^24 / (2097 x 62.5) = 128.0 s; 1500 / 2^16 / 100 / 3.634e-6 W per
    /// vcount-ccount = 63.0 C/W.
    #[test]
    fn the_family_table_values_are_128_s_and_63_c_per_w() {
        assert!(
            (UNITS.tau_s(2097.0) - 128.0).abs() < 0.05,
            "{}",
            UNITS.tau_s(2097.0)
        );
        assert!(
            (UNITS.r_th(1500.0) - 63.0).abs() < 0.05,
            "{}",
            UNITS.r_th(1500.0)
        );
        let g = thermometer::g_cc(63.0, UNITS.v_per_vcount, UNITS.a_per_ccount);
        let f = Fit {
            a: g / 128.0,
            b: -1.0 / 128.0,
            sd: 0.0,
            chunks: 0,
            rejected: 0,
            used_s: 0.0,
            segments: 0,
            gaps: 0,
            r_steps: 0,
        };
        let (alpha, g) = f.encode(UNITS.slow_hz);
        assert_eq!(alpha.raw, 2097);
        assert!((g.raw as i32 - 1500).abs() <= 1, "{}", g.raw);
    }

    /// A first-order excess sampled at the SLOW rate, the fit's rows.
    fn rise(tau_s: f64, g_cc: f64, p: f64, secs: f64, flags: u8) -> Vec<Row> {
        let dt = 1.0 / UNITS.slow_hz;
        let mut x = 0.0f64;
        (0..(secs / dt) as usize)
            .map(|k| {
                // a winding R of 1 vcount/ccount, so v = i and p = i^2
                let i = p.sqrt();
                let r = Row {
                    t_s: k as f64 * dt,
                    x_cc: x,
                    i,
                    v: i,
                    duty: i / VBUS,
                    p,
                    flags,
                };
                x += dt / tau_s * (g_cc * p - x);
                r
            })
            .collect()
    }

    /// A synthetic hold at known slopes, quantized to the kernel's whole
    /// centi-C: tau and g come back within 1%.
    #[test]
    fn the_fit_recovers_known_slopes_from_a_quantized_rise() {
        let mut rows = rise(128.0, 0.0229, 1.38e5, 480.0, flag::TRACK);
        for r in &mut rows {
            r.x_cc = r.x_cc.round();
        }
        let f = fit(&rows).unwrap();
        assert!((f.tau_s() / 128.0 - 1.0).abs() < 0.01, "tau {}", f.tau_s());
        assert!((f.g_cc() / 0.0229 - 1.0).abs() < 0.01, "g {}", f.g_cc());
        assert_eq!((f.segments, f.gaps), (1, 0));
        assert!(f.sd / UNITS.slow_hz < 0.05, "{}", f.sd);
    }

    /// Spikes the tracker could read inside a stretch (a contact step
    /// under every guard) are rejected at 3 sd, and the fit holds.
    #[test]
    fn outlier_chunks_are_rejected_once() {
        let mut rows = rise(100.0, 0.02, 1.2e5, 480.0, flag::TRACK);
        for (k, r) in rows.iter_mut().enumerate() {
            // +/-0.3 centi-C of tracker noise, and two 5 C spikes
            r.x_cc += 0.3 * ((k * 7919 % 13) as f64 / 6.0 - 1.0);
            if (10_000..10_010).contains(&k) || (20_000..20_010).contains(&k) {
                r.x_cc += 500.0;
            }
        }
        let f = fit(&rows).unwrap();
        assert!(f.rejected >= 2, "{f:?}");
        assert!((f.tau_s() / 100.0 - 1.0).abs() < 0.02, "tau {}", f.tau_s());
        assert!((f.g_cc() / 0.02 - 1.0).abs() < 0.02, "g {}", f.g_cc());
    }

    /// Untracked rows carry the table's model, not the winding: a carry
    /// running on wrong constants between two stretches is cut out, and
    /// each stretch loses its first 3 s and last 2 s.
    #[test]
    fn untracked_rows_and_the_cuts_around_them_are_never_fitted() {
        let mut rows = rise(128.0, 0.0229, 1.38e5, 480.0, flag::TRACK);
        for r in rows.iter_mut().filter(|r| (200.0..210.0).contains(&r.t_s)) {
            r.flags = 0;
            r.x_cc *= 3.0;
        }
        let f = fit(&rows).unwrap();
        assert_eq!((f.segments, f.gaps), (2, 1));
        let tracked = 480.0 - 10.0 - 2.0 * (BASE_SKIP_S + EXIT_GUARD_S);
        assert!(
            f.used_s <= tracked && f.used_s > tracked - 2.0 * CHUNK_S,
            "{}",
            f.used_s
        );
        assert!((f.tau_s() / 128.0 - 1.0).abs() < 0.01, "tau {}", f.tau_s());
        // the same rows fitted through the carry read another motor
        let mut through = rows.clone();
        for r in &mut through {
            r.flags = flag::TRACK;
        }
        let f = fit(&through).unwrap();
        assert!((f.tau_s() / 128.0 - 1.0).abs() > 0.1, "tau {}", f.tau_s());
    }

    /// The rail the synthetic rows' duty is read against, vcounts.
    const VBUS: f64 = 1780.0;

    /// A hold read through a bridged brush, its V/I 23% under the
    /// identified winding, is refused before any fit; the same hold at the
    /// winding passes.
    #[test]
    fn a_hold_in_a_low_resistance_contact_state_is_refused() {
        let rows = rise(128.0, 0.0229, 1.38e5, 60.0, flag::TRACK);
        assert_eq!(check_contact(&rows, 1.0, VBUS), Ok(()));
        let bridged: Vec<Row> = rows
            .iter()
            .map(|r| Row {
                v: r.v * 0.77,
                duty: r.duty * 0.77,
                ..*r
            })
            .collect();
        let why = check_contact(&bridged, 1.0, VBUS).unwrap_err();
        assert!(why.contains("a low-resistance contact state"), "{why}");
    }

    /// A bridge in and out inside the hold: two guard exits whose tracked
    /// R steps 23%, counted and reported.
    #[test]
    fn r_steps_across_the_guard_exits_are_counted() {
        let mut rows = rise(128.0, 0.0229, 1.38e5, 480.0, flag::TRACK);
        for r in rows.iter_mut() {
            if (200.0..203.0).contains(&r.t_s) || (213.0..216.0).contains(&r.t_s) {
                r.flags = 0;
            }
            if (201.0..214.0).contains(&r.t_s) {
                r.v *= 0.77;
            }
        }
        let f = fit(&rows).unwrap();
        assert_eq!((f.segments, f.r_steps), (3, 2), "{f:?}");
        let s = render(&Ok(f), (2097, 1500), &UNITS);
        assert!(s.contains("contact       2 step(s)"), "{s}");
        assert_eq!(
            fit(&rise(128.0, 0.0229, 1.38e5, 480.0, flag::TRACK))
                .unwrap()
                .r_steps,
            0
        );
    }

    #[test]
    fn a_flat_or_short_hold_fits_nothing() {
        assert!(fit(&rise(128.0, 0.0229, 1.38e5, 6.0, flag::TRACK)).is_err());
        assert!(fit(&rise(128.0, 0.0229, 1.38e5, 480.0, 0)).is_err());
        let flat = rise(128.0, 0.0, 0.0, 480.0, flag::TRACK);
        assert!(fit(&flat).is_err());
    }

    #[test]
    fn outside_the_band_is_refused() {
        assert!(plausible(128.0, 63.0).is_ok());
        assert!(plausible(20.0, 63.0).unwrap_err().contains("outside"));
        assert!(plausible(128.0, 250.0).is_err());
    }

    const SET: Rest = Rest {
        ntc_k1_q88: 563,
        r0_q12: 4128,
        therm_flags: 0,
        t_winding_cc: 2550,
        t_ntc_cc: 2500,
        alpha_q24: 2097,
    };

    #[test]
    fn a_rested_set_servo_starts() {
        assert_eq!(rested(&SET, UNITS.slow_hz), Ok(()));
    }

    #[test]
    fn a_missing_ntc_curve_refuses_naming_the_ntc() {
        let r = Rest {
            ntc_k1_q88: 0,
            therm_flags: flag::UNSET,
            t_winding_cc: UNSET_CC,
            ..SET
        };
        assert!(
            rested(&r, UNITS.slow_hz)
                .unwrap_err()
                .contains("no board NTC curve")
        );
        let r = Rest { r0_q12: 0, ..r };
        assert!(rested(&r, UNITS.slow_hz).unwrap_err().contains("NTC"));
    }

    #[test]
    fn a_missing_cold_r_or_a_sentinel_thermometer_refuses() {
        let r = Rest {
            r0_q12: 0,
            therm_flags: flag::UNSET,
            ..SET
        };
        assert!(rested(&r, UNITS.slow_hz).unwrap_err().contains("no cold R"));
        let r = Rest {
            therm_flags: flag::UNSET,
            t_winding_cc: UNSET_CC,
            ..SET
        };
        assert!(rested(&r, UNITS.slow_hz).unwrap_err().contains("sentinel"));
    }

    /// 5 C over the NTC decays to 1 C in tau ln 5 = 206 s on the family's
    /// carry.
    #[test]
    fn a_warm_servo_refuses_with_the_rest_it_needs() {
        let r = Rest {
            t_winding_cc: 3000,
            ..SET
        };
        assert_eq!(
            rested(&r, UNITS.slow_hz).unwrap_err(),
            "the winding reads 5.0 C over the board NTC, over the 1 C a fit starts from: rest it \
             torque off about 207 s more"
        );
    }

    #[test]
    fn the_report_sets_the_fit_beside_the_table() {
        let rows = rise(128.0, 0.0229, 1.38e5, 480.0, flag::TRACK);
        let s = render(&fit(&rows), (2097, 1500), &UNITS);
        assert!(s.contains("[thermal]"), "{s}");
        assert!(s.contains("table   2097 (tau 128.0 s)"), "{s}");
        assert!(s.contains("table   1500 (R_th 63.0 C/W)"), "{s}");
        assert!(s.contains("inside the band"), "{s}");
        let s = render(&Err("too short".into()), (2097, 1500), &UNITS);
        assert!(s.contains("declined      too short"), "{s}");
    }
}
