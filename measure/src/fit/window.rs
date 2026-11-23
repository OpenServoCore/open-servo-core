//! The window term from open-loop onsets from rest: the stage 3
//! window-term notebook's fit (oscnb `windowterm.onsets`, `gate`,
//! `pulse_train`; the notebook's `onset_set`, `fit`, `bootstrap`, `pinned`,
//! `by` and `lateness`), ported.
//!
//! The crest shunt sample at partial duty carries a part that is not
//! winding current, `g (1 - D) V_on`. At an onset from rest the winding
//! starts empty, so over the first read periods its current follows from L
//! and R alone, pulse by pulse from each row's own applied width. Each read
//! period `k` is modeled as
//!
//! `y_k = i_w(k; L, R) + g (1 - D_k) V_on`,
//!
//! linear in g at every (L, R) of a grid: g in closed form there, the cell
//! of least squares kept. L and R are nuisance values of the fit. The
//! field holds g plus the sample's lateness past the window's middle, the
//! same `(1 - D) V_on` form, so the reading is the period's mean current.

use crate::capture::rows::TelFrame;

/// Rows of the pot's median at the start and a millisecond in.
const MOVED_FROM: std::ops::Range<usize> = 0..5;
const MOVED_TO: std::ops::Range<usize> = 16..21;
/// Rows the drive must hold one sign over.
const SIGN_ROWS: usize = 21;
/// Widths of the ramp compared across onsets by the gate.
const SEQ_ROWS: usize = 14;
/// The firmware's bound on `g_window_q016 x r_q12`, past which it takes
/// no term (core `estimator::window::WindowTerm::conductance`).
pub const FENCE: f64 = (1u32 << 25) as f64;

/// One drive segment of a capture, its rows from the segment's first.
#[derive(Copy, Clone, Debug)]
pub struct Segment<'a> {
    pub seg: u32,
    pub cmd_duty_q15: i16,
    pub rows: &'a [TelFrame],
}

/// A grid as numpy's `arange(start, stop, step)` lays it.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Grid {
    pub start: f64,
    pub stop: f64,
    pub step: f64,
}

impl Grid {
    pub fn values(&self) -> Vec<f64> {
        let n = ((self.stop - self.start) / self.step).ceil().max(0.0) as usize;
        let delta = (self.start + self.step) - self.start;
        (0..n).map(|i| self.start + i as f64 * delta).collect()
    }
}

/// The method's constants, none of them measured.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Method {
    /// Rows kept after an onset; their trough mean is its zero.
    pub after: usize,
    /// The largest tap difference before the onset, counts: no entry
    /// back-EMF.
    pub tap_gate: f64,
    /// Counts the pot may move over the first millisecond.
    pub pot_gate: f64,
    pub read_periods: usize,
    /// The last row V_on is averaged to.
    pub v_row_max: usize,
    /// Rows the kept onsets must share one ramp over.
    pub ramp_rows: usize,
    pub l_grid: Grid,
    pub r_grid: Grid,
    /// Bootstrap draws over the onsets, within direction.
    pub draws: usize,
    pub seed: u64,
}

impl Method {
    pub const NOTEBOOK: Method = Method {
        after: 60,
        tap_gate: 6.0,
        pot_gate: 6.0,
        read_periods: 12,
        v_row_max: 30,
        ramp_rows: 40,
        l_grid: Grid {
            start: 0.40e-3,
            stop: 2.0001e-3,
            step: 0.01e-3,
        },
        r_grid: Grid {
            start: 3.0,
            stop: 7.5001,
            step: 0.02,
        },
        draws: 2000,
        seed: 20261117,
    };
}

/// The image's settle gain on a crest sample: Q15 words per 8-tick band of
/// drive width from `start_ticks`, unity past the table (core
/// `estimator::window::SettleGain`).
#[derive(Clone, Debug, PartialEq)]
pub struct Settle {
    pub start_ticks: u16,
    pub q15: Vec<u16>,
}

impl Settle {
    pub fn gain(&self, ticks: u16) -> f64 {
        let band = (ticks.saturating_sub(self.start_ticks) >> 3) as usize;
        self.q15.get(band).map_or(1 << 15, |&g| g as u32) as f64 / 32768.0
    }
}

/// What the rows are read through, every one measured or the board's.
#[derive(Clone, Debug, PartialEq)]
pub struct Scales {
    pub counts_per_a: f64,
    /// Counts of tap difference a volt, and the difference at zero volts.
    pub terminal_counts_per_v: f64,
    pub terminal_offset_counts: f64,
    /// Rows a second, one a PWM period.
    pub tick_hz: f64,
    /// The counter's crest, timer ticks: a period is twice it.
    pub pwm_arr: u16,
    /// Where the crest shunt sample closes after the crest, ticks.
    pub sample_ticks: f64,
    /// The tap ON and OFF edge lags at the captures' rail, ticks
    /// ([`on_rail_line`]).
    pub lag_on_ticks: f64,
    pub lag_off_ticks: f64,
    /// The low-side loop the winding decays through between pulses.
    pub r_brake_ohm: f64,
    /// The terminal floor: rows narrower carry no V_on.
    pub v_window_min_ticks: u16,
    pub settle: Settle,
}

impl Scales {
    fn tick_s(&self) -> f64 {
        1.0 / (self.tick_hz * 2.0 * self.pwm_arr as f64)
    }
}

/// An edge lag at `rail_v` on the line through two rails' `(volts, lag)`,
/// extrapolated past them as the notebook does.
pub fn on_rail_line(rail_v: f64, lo: (f64, f64), hi: (f64, f64)) -> f64 {
    lo.1 + (rail_v - lo.0) / (hi.0 - lo.0) * (hi.1 - lo.1)
}

#[derive(Clone, Debug, PartialEq)]
pub enum WindowError {
    /// A tick is missing inside a segment: the capture is refused.
    TickGap {
        cap: usize,
        seg: u32,
    },
    /// No onset passed the gates.
    NoOnsets,
    /// The kept onsets' widths differ at this read period.
    Ramps {
        k: usize,
    },
    /// No row of the ramp is at the terminal floor inside `v_row_max`.
    NoVoltRows,
    Method(&'static str),
    /// The fit came out NaN or infinite: a scale is not a number.
    NotFinite,
}

impl std::fmt::Display for WindowError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WindowError::TickGap { cap, seg } => {
                write!(f, "capture {cap} segment {seg}: a tick is missing")
            }
            WindowError::NoOnsets => f.write_str("no onset from rest passed the gates"),
            WindowError::Ramps { k } => {
                write!(f, "the kept onsets do not share one ramp at row {k}")
            }
            WindowError::NoVoltRows => f.write_str("no row of the ramp carries V_on"),
            WindowError::Method(why) => write!(f, "method: {why}"),
            WindowError::NotFinite => f.write_str("the fit is not finite"),
        }
    }
}

/// One driven row of an onset.
#[derive(Copy, Clone, Debug, PartialEq)]
struct Drive {
    h: u16,
    crest: f64,
    trough: f64,
    /// The tap difference along the drive, counts.
    vd: f64,
}

/// One onset from rest, the first row of non-zero duty in a drive segment,
/// with what the gates read.
#[derive(Clone, Debug, PartialEq)]
pub struct Onset {
    /// The caller's index of the capture.
    pub cap: usize,
    pub seg: u32,
    pub cmd_q15: i16,
    pub sgn: i8,
    /// Still rows before the onset.
    pub pre_rows: usize,
    /// Their largest tap difference, counts.
    pub pre_vd_max: Option<f64>,
    /// The pot's move over the first millisecond, counts.
    pub moved: Option<f64>,
    pub same_sign: bool,
    /// The ramp's first widths.
    pub seq: Vec<u16>,
    pub valid0: bool,
    /// Every row carries the raw row's fields, and `after` driven rows
    /// follow the onset.
    pub whole: bool,
    /// Set by [`gate`].
    pub ok: bool,
    drive: Vec<Drive>,
}

/// Every onset of one capture's segments; a segment commanded to zero has
/// none. Its window runs from the segment's first row to `after` rows past
/// the onset, a row's width paired with the crest sample of its own row.
pub fn onsets(cap: usize, segs: &[Segment], m: &Method) -> Result<Vec<Onset>, WindowError> {
    let mut out = Vec::new();
    for s in segs {
        if s.rows.windows(2).any(|w| w[1].tick != w[0].tick + 1) {
            return Err(WindowError::TickGap { cap, seg: s.seg });
        }
        if s.seg == 0 || s.cmd_duty_q15 == 0 {
            continue;
        }
        let Some(on) = s
            .rows
            .iter()
            .position(|r| r.duty_counts.is_some_and(|d| d != 0))
        else {
            continue;
        };
        let w = &s.rows[..(on + m.after).min(s.rows.len())];
        let sgn = s.rows[on].duty_counts.map_or(0, |d| d.signum() as i8);
        let (pre, post) = w.split_at(on);
        let vd = |r: &TelFrame| Some((r.vmotor_a? as f64 - r.vmotor_b? as f64) * sgn as f64);
        let pre_vd_max = if pre.is_empty() {
            None
        } else {
            pre.iter()
                .map(|r| vd(r).map(f64::abs))
                .collect::<Option<Vec<_>>>()
                .map(|v| v.into_iter().fold(f64::NEG_INFINITY, f64::max))
        };
        let pos_median = |range: std::ops::Range<usize>| {
            let v: Option<Vec<f64>> = post
                .get(range.start.min(post.len())..range.end.min(post.len()))?
                .iter()
                .map(|r| r.pos.map(f64::from))
                .collect();
            super::median(&v?)
        };
        let moved = match (pos_median(MOVED_TO), pos_median(MOVED_FROM)) {
            (Some(a), Some(b)) => Some((a - b).abs()),
            _ => None,
        };
        let complete = |r: &TelFrame| {
            r.pos.is_some()
                && r.duty_counts.is_some()
                && r.current_raw.is_some()
                && r.current_trough.is_some()
                && r.vmotor_a.is_some()
                && r.vmotor_b.is_some()
        };
        let whole = post.len() >= m.after && w.iter().all(complete);
        let drive = if whole {
            post.iter()
                .filter_map(|r| {
                    Some(Drive {
                        h: r.duty_counts?.unsigned_abs(),
                        crest: r.current_raw? as f64,
                        trough: r.current_trough? as f64,
                        vd: vd(r)?,
                    })
                })
                .collect()
        } else {
            Vec::new()
        };
        out.push(Onset {
            cap,
            seg: s.seg,
            cmd_q15: s.cmd_duty_q15,
            sgn,
            pre_rows: pre.len(),
            pre_vd_max,
            moved,
            same_sign: post
                .iter()
                .take(SIGN_ROWS)
                .all(|r| r.duty_counts.map(|d| d.signum() as i8) == Some(sgn)),
            seq: post
                .iter()
                .take(SEQ_ROWS)
                .filter_map(|r| r.duty_counts.map(i16::unsigned_abs))
                .collect(),
            valid0: post.first().is_some_and(|r| r.window_valid),
            whole,
            ok: false,
            drive,
        });
    }
    Ok(out)
}

/// The gates of an onset from rest over a session's onsets: a still row
/// before it within the tap gate, the pot within its gate, one sign, and
/// the session's common ramp (the most frequent, the least on a tie).
/// Returns that ramp; None without onsets.
pub fn gate(onsets: &mut [Onset], m: &Method) -> Option<Vec<u16>> {
    let mut counts: Vec<(&Vec<u16>, usize)> = Vec::new();
    for o in onsets.iter() {
        match counts.iter_mut().find(|(s, _)| *s == &o.seq) {
            Some((_, n)) => *n += 1,
            None => counts.push((&o.seq, 1)),
        }
    }
    let seq = counts
        .iter()
        .max_by(|a, b| a.1.cmp(&b.1).then_with(|| b.0.cmp(a.0)))
        .map(|(s, _)| (*s).clone())?;
    for o in onsets.iter_mut() {
        o.ok = o.whole
            && o.pre_rows >= 1
            && o.pre_vd_max.is_some_and(|v| v <= m.tap_gate)
            && o.moved.is_some_and(|v| v <= m.pot_gate)
            && o.same_sign
            && o.seq == seq;
    }
    Some(seq)
}

/// The winding current per volt of on-window at each row's shunt sample,
/// through a train of same-sign whole pulses from an empty winding: the
/// pulse around crest k conducts from `crest - h[k] + lag_on` to
/// `crest + h[k] + lag_off`, the winding decays through the brake loop
/// between pulses, and the sample closes `sample_ticks` after the crest.
pub fn pulse_train(h: &[f64], l_h: f64, r_ohm: f64, s: &Scales) -> Vec<f64> {
    let period = 2.0 * s.pwm_arr as f64;
    let dt = s.tick_s();
    let (k_drive, k_brake) = (dt * r_ohm / l_h, dt * (r_ohm + s.r_brake_ohm) / l_h);
    let brake = |i: f64, t0: f64, t1: f64| i * (-(t1 - t0).max(0.0) * k_brake).exp();
    let drive = |i: f64, t0: f64, t1: f64| {
        1.0 / r_ohm + (i - 1.0 / r_ohm) * (-(t1 - t0).max(0.0) * k_drive).exp()
    };
    let mut i = 0.0;
    let mut t = -period / 2.0;
    h.iter()
        .enumerate()
        .map(|(k, &hk)| {
            let crest = k as f64 * period;
            let s_at = crest + s.sample_ticks;
            if hk == 0.0 {
                return brake(i, t, s_at);
            }
            let (a, b) = (crest - hk + s.lag_on_ticks, crest + hk + s.lag_off_ticks);
            i = brake(i, t, a);
            let inside = if s_at <= a {
                Some(i)
            } else if s_at <= b {
                Some(drive(i, a, s_at))
            } else {
                None
            };
            i = drive(i, a, b);
            t = b;
            inside.unwrap_or_else(|| brake(i, b, s_at))
        })
        .collect()
}

/// The pulse train on every (L, R) of the grid, read period major, and the
/// sums the closed-form g needs.
struct Table {
    l: Vec<f64>,
    r: Vec<f64>,
    /// `f[k * cells + cell]`, L major within a period.
    f: Vec<f64>,
    sf2: Vec<f64>,
    sfc: Vec<f64>,
    /// The term's lever per volt, `1 - D`.
    c: Vec<f64>,
    scc: f64,
}

/// The least squares cell of one profile.
struct Cell {
    cell: usize,
    g: f64,
    rms: f64,
    winding: Vec<f64>,
    resid: Vec<f64>,
}

impl Table {
    fn new(h: &[f64], s: &Scales, m: &Method) -> Self {
        let (l, r) = (m.l_grid.values(), m.r_grid.values());
        let cells = l.len() * r.len();
        let n = h.len();
        let mut f = vec![0.0; n * cells];
        for (i, &lv) in l.iter().enumerate() {
            for (j, &rv) in r.iter().enumerate() {
                let cell = i * r.len() + j;
                for (k, v) in pulse_train(h, lv, rv, s).into_iter().enumerate() {
                    f[k * cells + cell] = v;
                }
            }
        }
        let c: Vec<f64> = h.iter().map(|&x| 1.0 - x / s.pwm_arr as f64).collect();
        let (mut sf2, mut sfc) = (vec![0.0; cells], vec![0.0; cells]);
        for k in 0..n {
            let row = &f[k * cells..(k + 1) * cells];
            for cell in 0..cells {
                sf2[cell] += row[cell] * row[cell];
                sfc[cell] += row[cell] * c[k];
            }
        }
        let scc = c.iter().map(|x| x * x).sum();
        Self {
            l,
            r,
            f,
            sf2,
            sfc,
            c,
            scc,
        }
    }

    fn cells(&self) -> usize {
        self.l.len() * self.r.len()
    }

    /// g and its residual at one cell, the reading `ya` in amps.
    fn at(&self, cell: usize, ya: &[f64], v: f64) -> Cell {
        let cells = self.cells();
        let winding: Vec<f64> = (0..ya.len())
            .map(|k| v * self.f[k * cells + cell])
            .collect();
        let x: Vec<f64> = self.c.iter().map(|c| c * v).collect();
        let r: Vec<f64> = ya.iter().zip(&winding).map(|(y, w)| y - w).collect();
        let g = r.iter().zip(&x).map(|(r, x)| r * x).sum::<f64>()
            / x.iter().map(|x| x * x).sum::<f64>();
        let resid: Vec<f64> = r.iter().zip(&x).map(|(r, x)| r - g * x).collect();
        let sse: f64 = resid.iter().map(|e| e * e).sum();
        Cell {
            cell,
            g,
            rms: (sse / (ya.len() - 3) as f64).sqrt(),
            winding,
            resid,
        }
    }

    /// The cell of least squares: per cell the sse of the closed-form g,
    /// `sum r^2 - (sum r c)^2 / sum c^2` with `r = y - v F`.
    fn solve(&self, ya: &[f64], v: f64) -> Option<Cell> {
        let cells = self.cells();
        let mut syf = vec![0.0; cells];
        for (k, &y) in ya.iter().enumerate() {
            let row = &self.f[k * cells..(k + 1) * cells];
            for (acc, fv) in syf.iter_mut().zip(row) {
                *acc += y * fv;
            }
        }
        let syy: f64 = ya.iter().map(|y| y * y).sum();
        let syc: f64 = ya.iter().zip(&self.c).map(|(y, c)| y * c).sum();
        let mut best: Option<(usize, f64)> = None;
        for cell in 0..cells {
            let rc = syc - v * self.sfc[cell];
            let sse = syy - 2.0 * v * syf[cell] + v * v * self.sf2[cell] - rc * rc / self.scc;
            if best.is_none_or(|(_, b)| sse < b) {
                best = Some((cell, sse));
            }
        }
        let (cell, sse) = best?;
        sse.is_finite().then(|| self.at(cell, ya, v))
    }
}

/// One kept onset's profile.
#[derive(Clone, Debug, PartialEq)]
pub struct Profile {
    pub cap: usize,
    pub seg: u32,
    pub sgn: i8,
    /// The trough mean over the onset's `after` rows, counts.
    pub zero_counts: f64,
    /// The crest sample over the zero, each read period, counts.
    pub y_counts: Vec<f64>,
    pub v_on_v: f64,
    /// g of this onset alone, L and R held at the fit's.
    pub g_pinned_s: f64,
}

/// One direction's onsets: g of their mean profile, free and with L and R
/// held at the pooled fit's.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Direction {
    pub n: usize,
    pub g_free_s: f64,
    pub g_pinned_s: f64,
}

/// The fit of a set of onsets.
#[derive(Clone, Debug, PartialEq)]
pub struct Window {
    /// The loss term alone at the sample's instant, S.
    pub g_s: f64,
    /// The bootstrap sd of g over the onsets; None under two draws.
    pub g_sd_s: Option<f64>,
    /// The sample's place past the window's middle at the fit's L, S.
    pub lateness_s: f64,
    /// What the field holds: g plus the lateness, S.
    pub g_over_mean_s: f64,
    pub l_h: f64,
    pub r_ohm: f64,
    /// L / R in PWM periods.
    pub tau_periods: f64,
    /// The residual rms over the read periods, 3 degrees of freedom taken.
    pub rms_a: f64,
    pub on_grid_edge: bool,
    /// The mean of the onsets' own V_on.
    pub v_on_v: f64,
    /// The read periods' half-widths, ticks.
    pub h: Vec<u16>,
    pub winding_a: Vec<f64>,
    pub resid_a: Vec<f64>,
    pub forward: Option<Direction>,
    pub reverse: Option<Direction>,
    pub profiles: Vec<Profile>,
    pub draws: usize,
}

struct Set {
    profiles: Vec<Profile>,
    /// The settle gain at each read period's width.
    gain: Vec<f64>,
}

impl Set {
    /// The mean reading of `idx` in amps and its mean V_on.
    fn mean(&self, idx: &[usize], s: &Scales) -> (Vec<f64>, f64) {
        let n = idx.len() as f64;
        let ya = (0..self.gain.len())
            .map(|k| {
                let y = idx
                    .iter()
                    .map(|&i| self.profiles[i].y_counts[k])
                    .sum::<f64>()
                    / n;
                y * self.gain[k] / s.counts_per_a
            })
            .collect();
        let v = idx.iter().map(|&i| self.profiles[i].v_on_v).sum::<f64>() / n;
        (ya, v)
    }
}

/// g alone with the winding held: `(r . c) / (c . c) / v`.
fn pinned(f: &[f64], c: &[f64], ya: &[f64], v: f64) -> f64 {
    let rc: f64 = ya
        .iter()
        .zip(f)
        .zip(c)
        .map(|((y, f), c)| (y - v * f) * c)
        .sum();
    rc / c.iter().map(|c| c * c).sum::<f64>() / v
}

struct SplitMix(u64);

impl SplitMix {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn below(&mut self, n: usize) -> usize {
        ((self.next() as u128 * n as u128) >> 64) as usize
    }
}

/// The window term of the gated onsets (`ok`), pooled.
pub fn fit(onsets: &[Onset], s: &Scales, m: &Method) -> Result<Window, WindowError> {
    let n_read = m.read_periods;
    if n_read <= 3 {
        return Err(WindowError::Method(
            "read_periods leaves no degree of freedom",
        ));
    }
    if m.after < m.ramp_rows || m.ramp_rows < n_read.max(m.v_row_max + 1) {
        return Err(WindowError::Method(
            "ramp_rows must cover read_periods and v_row_max, after ramp_rows",
        ));
    }
    let kept: Vec<&Onset> = onsets
        .iter()
        .filter(|o| o.ok && o.drive.len() >= m.after)
        .collect();
    let first = kept.first().ok_or(WindowError::NoOnsets)?;
    for k in 0..m.ramp_rows {
        if kept.iter().any(|o| o.drive[k].h != first.drive[k].h) {
            return Err(WindowError::Ramps { k });
        }
    }
    let ramp: Vec<u16> = first.drive[..m.ramp_rows].iter().map(|d| d.h).collect();
    let vrows: Vec<usize> = (0..=m.v_row_max)
        .filter(|&k| ramp[k] >= s.v_window_min_ticks)
        .collect();
    if vrows.is_empty() {
        return Err(WindowError::NoVoltRows);
    }
    let h: Vec<f64> = ramp[..n_read].iter().map(|&x| x as f64).collect();
    let mut set = Set {
        profiles: kept
            .iter()
            .map(|o| {
                let zero =
                    o.drive[..m.after].iter().map(|d| d.trough).sum::<f64>() / m.after as f64;
                let vd = vrows.iter().map(|&k| o.drive[k].vd).sum::<f64>() / vrows.len() as f64;
                Profile {
                    cap: o.cap,
                    seg: o.seg,
                    sgn: o.sgn,
                    zero_counts: zero,
                    y_counts: o.drive[..n_read].iter().map(|d| d.crest - zero).collect(),
                    v_on_v: (vd - s.terminal_offset_counts) / s.terminal_counts_per_v,
                    g_pinned_s: f64::NAN,
                }
            })
            .collect(),
        gain: ramp[..n_read].iter().map(|&x| s.settle.gain(x)).collect(),
    };
    let table = Table::new(&h, s, m);
    let all: Vec<usize> = (0..set.profiles.len()).collect();
    let (ya, v) = set.mean(&all, s);
    let head = table.solve(&ya, v).ok_or(WindowError::NotFinite)?;
    let (li, ri) = (head.cell / table.r.len(), head.cell % table.r.len());
    let (l_h, r_ohm) = (table.l[li], table.r[ri]);
    let f_head = pulse_train(&h, l_h, r_ohm, s);
    for p in &mut set.profiles {
        let ya: Vec<f64> = p
            .y_counts
            .iter()
            .zip(&set.gain)
            .map(|(y, g)| y * g / s.counts_per_a)
            .collect();
        p.g_pinned_s = pinned(&f_head, &table.c, &ya, p.v_on_v);
    }
    let side = |d: i8| -> Vec<usize> {
        all.iter()
            .copied()
            .filter(|&i| set.profiles[i].sgn == d)
            .collect()
    };
    let (fw, rv) = (side(1), side(-1));
    let by = |idx: &[usize]| -> Option<Direction> {
        if idx.is_empty() {
            return None;
        }
        let (ya, v) = set.mean(idx, s);
        Some(Direction {
            n: idx.len(),
            g_free_s: table.solve(&ya, v)?.g,
            g_pinned_s: pinned(&f_head, &table.c, &ya, v),
        })
    };
    let (forward, reverse) = (by(&fw), by(&rv));
    let mut rng = SplitMix(m.seed);
    let mut boot = Vec::with_capacity(m.draws);
    let mut idx = Vec::with_capacity(all.len());
    for _ in 0..m.draws {
        idx.clear();
        for side in [&fw, &rv] {
            for _ in 0..side.len() {
                idx.push(side[rng.below(side.len())]);
            }
        }
        let (ya, v) = set.mean(&idx, s);
        boot.push(table.solve(&ya, v).ok_or(WindowError::NotFinite)?.g);
    }
    let g_sd_s = (boot.len() >= 2).then(|| {
        let mu = boot.iter().sum::<f64>() / boot.len() as f64;
        (boot.iter().map(|g| (g - mu) * (g - mu)).sum::<f64>() / (boot.len() - 1) as f64).sqrt()
    });
    let middle = (s.lag_on_ticks + s.lag_off_ticks) / 2.0;
    let lateness_s = (s.sample_ticks - middle) * s.tick_s() / l_h;
    let w = Window {
        g_s: head.g,
        g_sd_s,
        lateness_s,
        g_over_mean_s: head.g + lateness_s,
        l_h,
        r_ohm,
        tau_periods: l_h / r_ohm * s.tick_hz,
        rms_a: head.rms,
        on_grid_edge: li == 0 || li + 1 == table.l.len() || ri == 0 || ri + 1 == table.r.len(),
        v_on_v: v,
        h: ramp[..n_read].to_vec(),
        winding_a: head.winding,
        resid_a: head.resid,
        forward,
        reverse,
        profiles: set.profiles,
        draws: m.draws,
    };
    let finite = [w.g_s, w.lateness_s, w.rms_a, w.v_on_v]
        .iter()
        .chain(w.g_sd_s.iter())
        .chain(w.profiles.iter().map(|p| &p.g_pinned_s))
        .all(|x| x.is_finite());
    finite.then_some(w).ok_or(WindowError::NotFinite)
}

/// `g_window_q016` for a term over the period's mean, rounded half to even.
pub fn field_q016(g_over_mean_s: f64, s: &Scales) -> f64 {
    (g_over_mean_s * s.counts_per_a / s.terminal_counts_per_v * 65536.0).round_ties_even()
}

/// One acceptance check: its value, None when there is nothing to judge,
/// and the verdict.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Check {
    pub name: &'static str,
    pub value: Option<f64>,
    pub unit: &'static str,
    pub pass: bool,
}

/// The notebook's acceptance of a fit, `r_q12` the servo's winding field.
pub fn acceptance(w: &Window, s: &Scales, r_q12: u16) -> Vec<Check> {
    let check = |name, value: Option<f64>, unit, ok: &dyn Fn(f64) -> bool| Check {
        name,
        value,
        unit,
        pass: value.is_some_and(ok),
    };
    let dirs = match (w.forward, w.reverse) {
        (Some(f), Some(r)) => Some(
            (f.g_free_s - r.g_free_s)
                .abs()
                .max((f.g_pinned_s - r.g_pinned_s).abs())
                / w.g_s.abs()
                * 100.0,
        )
        .filter(|v| v.is_finite()),
        _ => None,
    };
    let field = field_q016(w.g_over_mean_s, s);
    let fence = field * r_q12 as f64;
    vec![
        check("per-period rms <= 1 mA", Some(w.rms_a * 1e3), "mA", &|v| {
            v <= 1.0
        }),
        check(
            "L / R inside 2 to 8 periods",
            Some(w.tau_periods),
            "periods",
            &|v| (2.0..=8.0).contains(&v),
        ),
        check("the two directions within 10 %", dirs, "%", &|v| v <= 10.0),
        check("g >= 0", Some(w.g_s * 1e3), "mS", &|v| v >= 0.0),
        check(
            "g_window_q016 x r_q12 <= 2^25",
            Some(fence),
            "field units",
            &|v| r_q12 != 0 && (0.0..=u16::MAX as f64).contains(&field) && v <= FENCE,
        ),
        check(
            "L and R inside the grid",
            Some(w.on_grid_edge as u8 as f64),
            "on an edge",
            &|v| v == 0.0,
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scales(counts_per_a: f64) -> Scales {
        Scales {
            counts_per_a,
            terminal_counts_per_v: 248.0,
            terminal_offset_counts: 2.0,
            tick_hz: 20_000.0,
            pwm_arr: 1200,
            sample_ticks: 31.0,
            lag_on_ticks: 38.7,
            lag_off_ticks: 13.7,
            r_brake_ohm: 0.3,
            v_window_min_ticks: 93,
            settle: Settle {
                start_ticks: 40,
                q15: vec![37138, 35303, 34342, 33918, 33643, 33470, 33374, 33259],
            },
        }
    }

    const ZERO: u16 = 1000;
    const PRE: usize = 3;

    fn ramp(k: usize) -> u16 {
        (68.0 + 4.6 * k as f64).round() as u16
    }

    /// One onset's segment from the model: still rows, then `after` driven
    /// rows whose crest reads the winding of (L, R) plus g (1 - D) V_on.
    fn segment(sgn: i16, l: f64, r: f64, g: f64, s: &Scales) -> Vec<TelFrame> {
        let n = Method::NOTEBOOK.after;
        let vd = 1211u16;
        let v_on = (vd as f64 - s.terminal_offset_counts) / s.terminal_counts_per_v;
        let h: Vec<f64> = (0..n).map(|k| ramp(k) as f64).collect();
        let iw = pulse_train(&h, l, r, s);
        let still = TelFrame {
            pos: Some(2000),
            duty_counts: Some(0),
            current_raw: Some(ZERO),
            current_trough: Some(ZERO),
            vmotor_a: Some(770),
            vmotor_b: Some(770),
            ..TelFrame::default()
        };
        let mut rows = vec![still; PRE];
        for k in 0..n {
            let y = v_on * iw[k] + g * (1.0 - h[k] / s.pwm_arr as f64) * v_on;
            let crest = ZERO as f64 + y * s.counts_per_a / s.settle.gain(ramp(k));
            let (hi, lo) = (770 + vd, 770);
            let (a, b) = if sgn > 0 { (hi, lo) } else { (lo, hi) };
            rows.push(TelFrame {
                duty_counts: Some(ramp(k) as i16 * sgn),
                current_raw: Some(crest.round() as u16),
                vmotor_a: Some(a),
                vmotor_b: Some(b),
                window_valid: true,
                ..still
            });
        }
        for (t, row) in rows.iter_mut().enumerate() {
            row.tick = t as u64;
        }
        rows
    }

    fn method() -> Method {
        Method {
            draws: 20,
            ..Method::NOTEBOOK
        }
    }

    #[test]
    fn the_grid_is_numpys_arange() {
        let m = Method::NOTEBOOK;
        assert_eq!(
            (m.l_grid.values().len(), m.r_grid.values().len()),
            (161, 226)
        );
        assert_eq!(m.r_grid.values()[0], 3.0);
    }

    #[test]
    fn the_settle_gain_is_the_firmwares() {
        use osc_servo_core::estimator::window::SettleGain;
        const Q: &[u16] = &[37138, 35303, 34342, 33918, 33643, 33470, 33374, 33259];
        let fw = SettleGain {
            start_ticks: 40,
            q15: Q,
        };
        let s = scales(1.0).settle;
        for t in 0..200u16 {
            assert_eq!(s.gain(t), fw.q15_at(t as u32) as f64 / 32768.0, "{t}");
        }
    }

    #[test]
    fn an_onset_built_from_the_model_returns_its_g_l_and_r() {
        let m = method();
        let s = scales(1.0e5);
        let (l, r, g) = (m.l_grid.values()[39], m.r_grid.values()[69], 3.8e-3);
        let segs: Vec<Vec<TelFrame>> = [1i16, 1, -1, -1]
            .iter()
            .map(|&sgn| segment(sgn, l, r, g, &s))
            .collect();
        let view: Vec<Segment> = segs
            .iter()
            .enumerate()
            .map(|(i, rows)| Segment {
                seg: i as u32 + 1,
                cmd_duty_q15: 8191,
                rows,
            })
            .collect();
        let mut ons = onsets(0, &view, &m).expect("onsets");
        assert_eq!(ons.len(), 4);
        assert_eq!(ons[2].sgn, -1);
        assert_eq!(gate(&mut ons, &m).expect("ramp")[..2], [68, 73]);
        assert!(ons.iter().all(|o| o.ok && o.pre_rows == PRE));
        let w = fit(&ons, &s, &m).expect("fits");
        assert_eq!((w.l_h, w.r_ohm), (l, r));
        assert!((w.g_s / g - 1.0).abs() < 2e-3, "{}", w.g_s);
        assert!(w.rms_a < 1e-5, "{}", w.rms_a);
        assert!(w.g_sd_s.expect("draws") < 1e-5);
        // the field on the board's own scale: the model's is only fine
        let board = scales(1135.0);
        let checks = acceptance(&w, &board, 3725);
        assert!(checks.iter().all(|c| c.pass), "{checks:?}");
        assert!(!acceptance(&w, &board, 0)[4].pass, "no winding, no term");
    }

    #[test]
    fn nothing_to_fit_is_an_error_never_a_panic() {
        let m = method();
        let s = scales(1135.0);
        assert_eq!(onsets(0, &[], &m), Ok(vec![]));
        assert_eq!(gate(&mut [], &m), None);
        assert_eq!(fit(&[], &s, &m), Err(WindowError::NoOnsets));

        let rows = segment(1, 0.79e-3, 4.38, 3.8e-3, &s);
        let one = |rows: &[TelFrame]| {
            let seg = Segment {
                seg: 1,
                cmd_duty_q15: 8191,
                rows,
            };
            let mut ons = onsets(0, &[seg], &m)?;
            gate(&mut ons, &m);
            fit(&ons, &s, &m)
        };
        assert!(one(&rows).is_ok());
        // an onset with too few periods after it
        assert_eq!(one(&rows[..PRE + 20]), Err(WindowError::NoOnsets));
        // a row without its crest sample, and one without a still row before
        let mut holed = rows.clone();
        holed[PRE + 5].current_raw = None;
        assert_eq!(one(&holed), Err(WindowError::NoOnsets));
        assert_eq!(one(&rows[PRE..]), Err(WindowError::NoOnsets));
        let mut gap = rows.clone();
        gap.remove(PRE + 2);
        assert_eq!(one(&gap), Err(WindowError::TickGap { cap: 0, seg: 1 }));
        // a scale that is not a number
        let seg = Segment {
            seg: 1,
            cmd_duty_q15: 8191,
            rows: &rows,
        };
        let mut ons = onsets(0, &[seg], &m).expect("onsets");
        gate(&mut ons, &m);
        let nan = scales(f64::NAN);
        assert_eq!(fit(&ons, &nan, &m), Err(WindowError::NotFinite));
        let short = Method {
            read_periods: 3,
            ..m
        };
        assert!(matches!(fit(&ons, &s, &short), Err(WindowError::Method(_))));
        // a capture commanded to zero has no onset
        let brake = Segment {
            seg: 1,
            cmd_duty_q15: 0,
            rows: &rows,
        };
        assert_eq!(onsets(0, &[brake], &m), Ok(vec![]));
    }
}
