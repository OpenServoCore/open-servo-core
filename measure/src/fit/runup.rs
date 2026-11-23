//! The inertia and the coast's friction line from run-ups from rest and
//! their own coasts: the stage 3 run-up notebook's J through the origin
//! and the friction notebook's coast route, ported.
//!
//! Under drive `J a_d = Kt i - T(w)` and on the coast `J a_c = -T(w)`;
//! taking the coast's drag at the same speed as the drive's,
//! `J (a_d - a_c(w)) = Kt i`, so the acceleration the current buys over
//! the coast's at the same speed gives J with no drag model, and the
//! coast through J gives the drag. Kt is Ke in SI units.
//!
//! The drive's speed is the order count of its current's ripple
//! ([`crate::ripple::order`]), first seeded by the EMF the volts leave
//! through R and L; no pot enters. Its crossings give turn windows and
//! acceleration fits. Three flags keep a miscounted stretch out: a turn
//! window off its neighbors' quadratic, a fit whose angle residual is
//! over twice its run's median, and a window more than
//! [`CROSS_RUN_TOL`] off the median of the run-ups of its direction at the
//! same time. A fit is kept when it accelerates, passes its residual flag
//! and no flagged window is centered inside it.

use core::f64::consts::PI;

use super::coast::{self, CoastTable, DecelCurve, Pooled};
use super::{lstsq, mean, median, sd};
use crate::capture::rows::TelFrame;
use crate::ripple::order::{self, arange};
use crate::ripple::table::{self, AccelFit, FIT_HALF_S, TurnWindow};

/// 2 ms, no dropout read inside the current's rise.
pub const EDGE_ROWS: usize = 40;
/// 1 ms, the running mean's edge at the drive's end.
pub const TAIL_ROWS: usize = 20;
/// A driven row under this share of the current around it is a dropout.
pub const DROPOUT_FRAC: f64 = 0.25;
/// A fit's angle residual over this many times its run's median is
/// flagged.
pub const RES_FLAG: f64 = 2.0;
/// s, the step of the cross-run check's times.
pub const CROSS_RUN_STEP_S: f64 = 0.0005;
/// A turn window this far off its direction's median is flagged.
pub const CROSS_RUN_TOL: f64 = 0.06;
/// Fits a run's J needs.
pub const MIN_FITS: usize = 8;
/// rev/s, the friction line's speed bins.
pub const FRICTION_BIN_REV_S: f64 = 25.0;
/// The bins' top edge, rev/s: a faster fit sits in the last bin.
pub const FRICTION_TOP_REV_S: f64 = 500.0;
/// Fits a bin needs.
pub const FRICTION_BIN_MIN: usize = 5;
/// rev/s, bins at or over this stay out of the line.
pub const FRICTION_LINE_MAX_REV_S: f64 = 480.0;

/// The conversions and the motor's constants a run-up is read through.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Scales {
    pub counts_per_a: f64,
    /// The current's in-drive offset over the rest zero, counts.
    pub drive_offset_counts: f64,
    /// Counts of tap difference a volt, and the difference at zero volts.
    pub counts_per_v: f64,
    pub terminal_offset_counts: f64,
    /// The PWM half period, ticks: the duty fraction is `|duty| / pwm_arr`.
    pub pwm_arr: f64,
    /// S, the window term: the crest sample reads `g (1 - D) V_on` over the
    /// winding current. Nothing at full duty.
    pub window_s: f64,
    pub r_ohm: f64,
    pub l_h: f64,
    /// V s/rad; Kt is taken equal.
    pub ke: f64,
    /// A, the drag at rest the count's first turns integrate over.
    pub i_drag0_a: f64,
    /// Ripple events a motor revolution.
    pub per_rev: usize,
    /// Rows a second.
    pub fs: f64,
}

impl Scales {
    fn coast(&self) -> coast::Scales {
        coast::Scales {
            counts_per_v: self.counts_per_v,
            per_rev: self.per_rev,
            fs: self.fs,
        }
    }
}

/// One run-up from rest: the spin's rows, the coast's (opening with the
/// drive's tail), and the sweep's rest zeros.
#[derive(Copy, Clone, Debug)]
pub struct Run<'a> {
    pub dir: i8,
    pub spin: &'a [TelFrame],
    pub coast: &'a [TelFrame],
    pub zero_i: f64,
    pub zero_e: f64,
}

/// One run-up read on its own.
#[derive(Clone, Debug, PartialEq)]
pub struct RunUp {
    pub dir: i8,
    /// The driven rows.
    pub rows: usize,
    /// The order count's crossings, s from the first driven row.
    pub tc: Vec<f64>,
    pub added: usize,
    pub removed: usize,
    pub dropouts: usize,
    pub windows: Vec<TurnWindow>,
    /// The turn windows its neighbors keep.
    pub windows_ok: Vec<bool>,
    pub fits: Vec<AccelFit>,
    /// The fits under twice the run's median residual.
    pub fits_ok: Vec<bool>,
    pub coast: CoastTable,
    pub curve: DecelCurve,
}

fn contiguous(rows: &[TelFrame]) -> bool {
    rows.windows(2).all(|w| w[1].tick == w[0].tick + 1)
}

/// The driven rows' winding current, A, and applied volts, V: the crest
/// over the rest zero less the in-drive offset and the window term, and
/// the tap difference times the duty.
fn drive(run: &Run, s: &Scales) -> Result<(Vec<f64>, Vec<f64>), String> {
    let on = run
        .spin
        .iter()
        .position(|f| f.duty_counts.is_some_and(|d| d != 0))
        .ok_or("the spin never drives")?;
    let (a_per_count, v_per_count) = (1.0 / s.counts_per_a, 1.0 / s.counts_per_v);
    let mut i = Vec::with_capacity(run.spin.len() - on);
    let mut u = Vec::with_capacity(run.spin.len() - on);
    for (k, f) in run.spin[on..].iter().enumerate() {
        let (Some(raw), Some(a), Some(b), Some(d)) =
            (f.current_raw, f.vmotor_a, f.vmotor_b, f.duty_counts)
        else {
            return Err(format!("driven row {k} lacks a channel"));
        };
        let taps = (i32::from(a) - i32::from(b)) * i32::from(run.dir);
        let v_on = (f64::from(taps) - s.terminal_offset_counts) * v_per_count;
        let duty = f64::from(d.unsigned_abs()) / s.pwm_arr;
        let crest = (f64::from(raw) - run.zero_i - s.drive_offset_counts) * a_per_count;
        i.push(crest - s.window_s * (1.0 - duty) * v_on);
        u.push(duty * v_on);
    }
    Ok((i, u))
}

/// The dropout rows: the current under [`DROPOUT_FRAC`] of its running
/// mean, none in the rise or at the drive's end.
fn dropouts(i: &[f64]) -> Vec<bool> {
    let r = coast::local_mean(i);
    let n = i.len();
    (0..n)
        .map(|k| k >= EDGE_ROWS && k + TAIL_ROWS < n && i[k] < DROPOUT_FRAC * r[k])
        .collect()
}

/// One run-up's count, turn windows, fits and coast.
pub fn read_run(run: &Run, s: &Scales) -> Result<RunUp, String> {
    if !(contiguous(run.spin) && contiguous(run.coast)) {
        return Err("a row is missing: the capture is no evidence".into());
    }
    let (i, u) = drive(run, s)?;
    if !i.iter().chain(&u).all(|v| v.is_finite()) {
        return Err("a driven row is not finite".into());
    }
    let open = dropouts(&i);
    let rough = order::rough_speed(&i, &u, s.r_ohm, s.l_h, s.ke, s.fs);
    let count = order::count_speed(&i, &rough, s.i_drag0_a, s.per_rev, s.fs)
        .ok_or("the ripple count found too few crossings")?;
    let windows = table::turn_windows(&count.tc, s.per_rev);
    let windows_ok = table::flag_windows(&windows);
    let fits = table::fit_table(&count.tc, s.per_rev, &i, &open, s.fs);
    let res: Vec<f64> = fits.iter().map(|f| f.res_deg).collect();
    let med = median(&res).ok_or("no acceleration fit")?;
    let fits_ok = res.iter().map(|r| *r <= RES_FLAG * med).collect();
    let cut = coast::cut_row(run.coast).ok_or("the coast never opens the bridge")?;
    let emf = coast::emf_from_cut(run.coast, run.dir, cut, run.zero_e, s.counts_per_v)
        .ok_or("a coast row lacks a tap")?;
    let table =
        coast::coast_table(&emf, s.ke, &s.coast()).ok_or("the coast is too short to read")?;
    let curve = coast::decel_curve(&table.fits).ok_or("the coast holds no deceleration fit")?;
    Ok(RunUp {
        dir: run.dir,
        rows: i.len(),
        tc: count.tc,
        added: count.added,
        removed: count.removed,
        dropouts: open.iter().filter(|o| **o).count(),
        windows,
        windows_ok,
        fits,
        fits_ok,
        coast: table,
        curve,
    })
}

/// One run-up's J.
#[derive(Clone, Debug, PartialEq)]
pub struct RunJ {
    pub dir: i8,
    /// kg m^2 through the origin; None under [`MIN_FITS`] kept fits.
    pub j: Option<f64>,
    pub fits_kept: usize,
    /// Turn windows flagged by either check.
    pub windows_flagged: usize,
}

/// The coast's drag against speed through J, one direction: the
/// deceleration `a0 + a1 w` (rad/s^2, positive slowing) and the torque
/// `Tc + b w` it takes at the motor, w in rad/s.
#[derive(Clone, Debug, PartialEq)]
pub struct FrictionLine {
    pub dir: i8,
    pub a0: f64,
    pub a1: f64,
    /// N m and N m s/rad.
    pub tc: f64,
    pub b: f64,
    /// N m, the line's rms about the bins.
    pub rms: f64,
    pub bins: usize,
}

/// J pooled over run-ups, and the coasts' friction line per direction.
#[derive(Clone, Debug, PartialEq)]
pub struct Inertia {
    pub runs: Vec<RunJ>,
    /// kg m^2 over the runs that have one.
    pub j: Pooled,
    pub by_dir: Vec<(i8, Pooled)>,
    pub friction: Vec<FrictionLine>,
}

fn nanmedian(v: &[f64]) -> f64 {
    let f: Vec<f64> = v.iter().copied().filter(|x| !x.is_nan()).collect();
    median(&f).unwrap_or(f64::NAN)
}

/// The windows each run keeps after the cross-run check: within
/// [`CROSS_RUN_TOL`] of the median of its direction's runs at its time.
fn cross_run(reads: &[RunUp], s: &Scales) -> Vec<Vec<bool>> {
    let end =
        reads.iter().map(|r| r.rows).max().unwrap_or(0) as f64 / s.fs + 2.0 * CROSS_RUN_STEP_S;
    let grid = arange(0.0, end, CROSS_RUN_STEP_S);
    let mut ok: Vec<Vec<bool>> = reads.iter().map(|r| r.windows_ok.clone()).collect();
    let mut dirs: Vec<i8> = reads.iter().map(|r| r.dir).collect();
    dirs.sort_unstable();
    dirs.dedup();
    for d in dirs {
        let curves: Vec<Vec<f64>> = reads
            .iter()
            .filter(|r| r.dir == d)
            .filter_map(|r| {
                let (t, w): (Vec<f64>, Vec<f64>) = r
                    .windows
                    .iter()
                    .zip(&r.windows_ok)
                    .filter(|(_, ok)| **ok)
                    .map(|(x, _)| (x.t, x.w))
                    .unzip();
                (!t.is_empty()).then(|| {
                    grid.iter()
                        .map(|&g| super::interp(g, &t, &w, f64::NAN, f64::NAN))
                        .collect()
                })
            })
            .collect();
        let rf: Vec<f64> = (0..grid.len())
            .map(|k| nanmedian(&curves.iter().map(|c| c[k]).collect::<Vec<_>>()))
            .collect();
        for (r, ok) in reads.iter().zip(ok.iter_mut()).filter(|(r, _)| r.dir == d) {
            for (x, o) in r.windows.iter().zip(ok.iter_mut()) {
                let at = super::interp(x.t, &grid, &rf, rf[0], rf[rf.len() - 1]);
                if (x.w / at - 1.0).abs() > CROSS_RUN_TOL {
                    *o = false;
                }
            }
        }
    }
    ok
}

/// J through the origin from one run's kept fits against its own coast:
/// `sum x^2 / sum x y` over `x = Kt i`, `y = a_d - a_c(w)`.
fn run_j(r: &RunUp, windows_ok: &[bool], s: &Scales) -> RunJ {
    let bad: Vec<f64> = r
        .windows
        .iter()
        .zip(windows_ok)
        .filter(|(_, ok)| !**ok)
        .map(|(x, _)| x.t)
        .collect();
    let (mut sxx, mut sxy, mut n) = (0.0, 0.0, 0);
    for (f, ok) in r.fits.iter().zip(&r.fits_ok) {
        let win_ok = !bad.iter().any(|t| (f.t - t).abs() <= FIT_HALF_S);
        if !(*ok && win_ok && f.alpha > 0.0 && f.i > 0.0) {
            continue;
        }
        let (x, y) = (s.ke * f.i, f.alpha - r.curve.at(f.w));
        if x.is_finite() && y.is_finite() {
            sxx += x * x;
            sxy += x * y;
            n += 1;
        }
    }
    let j = sxx / sxy;
    RunJ {
        dir: r.dir,
        j: (n >= MIN_FITS && j.is_finite() && j > 0.0).then_some(j),
        fits_kept: n,
        windows_flagged: windows_ok.iter().filter(|o| !**o).count(),
    }
}

/// The friction notebook's coast route: the fits of every coast of `dir`
/// binned by [`FRICTION_BIN_REV_S`], the median deceleration per bin with
/// [`FRICTION_BIN_MIN`] fits, and a line through the bins under
/// [`FRICTION_LINE_MAX_REV_S`], weighted by each bin's standard error.
fn friction(reads: &[RunUp], dir: i8, j: f64) -> Option<FrictionLine> {
    let top = (FRICTION_TOP_REV_S / FRICTION_BIN_REV_S) as i64;
    let mut fits: Vec<(i64, f64, f64)> = reads
        .iter()
        .filter(|r| r.dir == dir)
        .flat_map(|r| r.coast.fits.iter())
        .map(|f| {
            let rev_s = f.w / (2.0 * PI);
            let bin = if rev_s < 0.0 {
                -1
            } else {
                ((rev_s / FRICTION_BIN_REV_S).floor() as i64).min(top)
            };
            (bin, rev_s, -f.alpha)
        })
        .collect();
    fits.sort_by_key(|f| f.0);
    let mut rows: Vec<(f64, f64, f64)> = Vec::new();
    for g in fits.chunk_by(|a, b| a.0 == b.0) {
        if g.len() < FRICTION_BIN_MIN {
            continue;
        }
        let w: Vec<f64> = g.iter().map(|f| f.1).collect();
        let a: Vec<f64> = g.iter().map(|f| f.2).collect();
        let rev_s = mean(&w)?;
        let se = sd(&a, 1)? / (a.len() as f64).sqrt();
        if rev_s >= FRICTION_LINE_MAX_REV_S || se.is_nan() || se <= 0.0 {
            continue;
        }
        rows.push((2.0 * PI * rev_s, median(&a)?, se));
    }
    // the speed column in krad/s keeps the normal equations' digits
    let x: Vec<[f64; 2]> = rows
        .iter()
        .map(|(w, _, se)| [1.0 / se, w / 1e3 / se])
        .collect();
    let y: Vec<f64> = rows.iter().map(|(_, a, se)| a / se).collect();
    let [a0, a1] = lstsq(&x, &y)?;
    let a1 = a1 / 1e3;
    let res: Vec<f64> = rows
        .iter()
        .map(|(w, a, _)| j * (a - (a0 + a1 * w)))
        .collect();
    Some(FrictionLine {
        dir,
        a0,
        a1,
        tc: j * a0,
        b: j * a1,
        rms: mean(&res.iter().map(|r| r * r).collect::<Vec<_>>())?.sqrt(),
        bins: rows.len(),
    })
}

/// J over run-ups read by [`read_run`], each against its own coast after
/// the cross-run check over all of them, and the friction line of each
/// direction's coasts through the pooled J.
pub fn inertia(reads: &[RunUp], s: &Scales) -> Result<Inertia, String> {
    if reads.is_empty() {
        return Err("no run-up".into());
    }
    let ok = cross_run(reads, s);
    let runs: Vec<RunJ> = reads
        .iter()
        .zip(&ok)
        .map(|(r, ok)| run_j(r, ok, s))
        .collect();
    let js: Vec<f64> = runs.iter().filter_map(|r| r.j).collect();
    let j = Pooled::of(&js).ok_or("no run-up kept enough fits for J")?;
    let mut dirs: Vec<i8> = runs.iter().map(|r| r.dir).collect();
    dirs.sort_unstable();
    dirs.dedup();
    let by_dir = dirs
        .iter()
        .filter_map(|&d| {
            let v: Vec<f64> = runs
                .iter()
                .filter(|r| r.dir == d)
                .filter_map(|r| r.j)
                .collect();
            Some((d, Pooled::of(&v)?))
        })
        .collect();
    let friction = dirs
        .iter()
        .filter_map(|&d| friction(reads, d, j.mean))
        .collect();
    Ok(Inertia {
        runs,
        j,
        by_dir,
        friction,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: Scales = Scales {
        counts_per_a: 1000.0,
        drive_offset_counts: 2.0,
        counts_per_v: 250.0,
        terminal_offset_counts: 1.0,
        pwm_arr: 1200.0,
        window_s: 0.0,
        r_ohm: 4.2,
        l_h: 0.0,
        ke: 1.16e-3,
        i_drag0_a: 0.043,
        per_rev: 6,
        fs: 20_000.0,
    };
    const J: f64 = 6.4e-9;
    /// N m and N m s/rad, the planted drag.
    const C0: f64 = 50e-6;
    const C1: f64 = 4.8e-8;
    const ZERO_I: f64 = 1200.0;
    const ZERO_E: f64 = 2.0;

    /// The two taps of a signed difference.
    fn taps(diff: i32) -> (Option<u16>, Option<u16>) {
        let (a, b) = if diff >= 0 {
            (1000 + diff, 1000)
        } else {
            (1000, 1000 - diff)
        };
        (Some(a as u16), Some(b as u16))
    }

    /// A 5 V run-up from rest at full duty and its coast, the motor's own
    /// equations a row at a time, an 8 % commutation ripple on the
    /// current and 5 % on the EMF.
    fn run_up(dir: i8) -> (Vec<TelFrame>, Vec<TelFrame>) {
        let dt = 1.0 / S.fs;
        let (mut w, mut th) = (0.0f64, 0.0f64);
        let mut spin = Vec::new();
        for k in 0..1040u64 {
            let i = (5.0 - S.ke * w) / S.r_ohm;
            let raw = ZERO_I
                + S.drive_offset_counts
                + i * (1.0 + 0.08 * (6.0 * th).cos()) * S.counts_per_a;
            let (vmotor_a, vmotor_b) =
                taps((5.0 * S.counts_per_v + S.terminal_offset_counts) as i32 * i32::from(dir));
            spin.push(TelFrame {
                tick: k,
                duty_counts: Some(1200 * i16::from(dir)),
                current_raw: Some(raw.round() as u16),
                vmotor_a,
                vmotor_b,
                ..TelFrame::default()
            });
            th += w * dt;
            w += (S.ke * i - (C0 + C1 * w)) / J * dt;
        }
        let mut coast = Vec::new();
        for k in 0..6000u64 {
            let emf = S.ke * w * (1.0 + 0.05 * (6.0 * th).cos());
            let (vmotor_a, vmotor_b) =
                taps(((emf * S.counts_per_v + ZERO_E).round() as i32) * i32::from(dir));
            coast.push(TelFrame {
                tick: 1040 + k,
                duty_counts: Some(0),
                current_raw: Some(ZERO_I as u16),
                vmotor_a,
                vmotor_b,
                ..TelFrame::default()
            });
            th += w * dt;
            w = (w - (C0 + C1 * w) / J * dt).max(0.0);
        }
        (spin, coast)
    }

    fn run<'a>(dir: i8, rows: &'a (Vec<TelFrame>, Vec<TelFrame>)) -> Run<'a> {
        Run {
            dir,
            spin: &rows.0,
            coast: &rows.1,
            zero_i: ZERO_I,
            zero_e: ZERO_E,
        }
    }

    #[test]
    fn a_planted_motor_gives_its_inertia_and_drag() {
        let up = [run_up(1), run_up(-1)];
        let reads: Vec<RunUp> = [1, -1]
            .iter()
            .zip(&up)
            .map(|(d, r)| read_run(&run(*d, r), &S).expect("reads"))
            .collect();
        assert!(
            reads
                .iter()
                .all(|r| r.tc.len() > 80 && r.added + r.removed == 0)
        );
        let got = inertia(&reads, &S).expect("J");
        assert_eq!(got.j.n, 2);
        assert!(
            (got.j.mean / J - 1.0).abs() < 0.01,
            "J {} against {J}",
            got.j.mean
        );
        assert_eq!(got.friction.len(), 2);
        for f in &got.friction {
            assert!(
                (f.a0 / (C0 / J) - 1.0).abs() < 0.02,
                "a0 {} against {}",
                f.a0,
                C0 / J
            );
            assert!(
                (f.a1 / (C1 / J) - 1.0).abs() < 0.03,
                "a1 {} against {}",
                f.a1,
                C1 / J
            );
        }
    }

    #[test]
    fn empty_flat_and_missing_rows_are_refused() {
        let up = run_up(1);
        let read = |spin: &[TelFrame], coast: &[TelFrame]| {
            read_run(
                &Run {
                    dir: 1,
                    spin,
                    coast,
                    zero_i: ZERO_I,
                    zero_e: ZERO_E,
                },
                &S,
            )
        };
        assert!(read(&[], &up.1).is_err());
        assert!(read(&up.0, &[]).is_err());
        let idle: Vec<TelFrame> =
            up.0.iter()
                .map(|f| TelFrame {
                    duty_counts: Some(0),
                    ..*f
                })
                .collect();
        assert!(read(&idle, &up.1).is_err());
        let flat: Vec<TelFrame> =
            up.0.iter()
                .map(|f| TelFrame {
                    current_raw: Some(1500),
                    ..*f
                })
                .collect();
        assert!(read(&flat, &up.1).is_err());
        let mut hole = up.0.clone();
        hole[500].current_raw = None;
        assert!(read(&hole, &up.1).is_err());
        let mut gap = up.0.clone();
        gap.remove(500);
        assert!(read(&gap, &up.1).is_err());
        assert!(inertia(&[], &S).is_err());
    }
}
