//! The spin step's decisions, pure: the polled go-to that brings the shaft
//! to rest at a target, the order of the spins, the window each spin's
//! drive may last, and the checks after each spin. Positions are raw pot
//! counts, speeds counts/s, times s.
//!
//! The guards lie [`GUARD_COUNTS`] inside each stop. A spin starts from
//! rest at the guard on the side it drives away from, and its whole
//! predicted travel, the dead-man's share and the coast included, ends at
//! least [`SHORT_COUNTS`] before the far guard:
//!
//! ```text
//! bound_speed x (T + DRIVE_DEADMAN_S) + bound_coast <= runway
//! runway      = from the start to the far guard, less SHORT_COUNTS
//! ```
//!
//! The pilot and the lowest level are bounded by the nameplate and what
//! the run has measured, the larger, so no measurement lowers the bound:
//!
//! ```text
//! bound_speed = SPEED_GAIN x max(the nameplate's speed at the level's
//!               volts, each speed measured this run scaled by volts)
//! bound_coast = COAST_GAIN x each coast measured this run scaled by
//!               (bound_speed / its spin's speed)^2, the largest
//! ```
//!
//! A higher level in direction d is bounded by the next lower level's
//! spins in d alone, the pilot among them, and is left out in d until one
//! has run:
//!
//! ```text
//! bound_speed = SPEED_GAIN x the fastest of them x the volts' ratio
//! bound_coast = COAST_GAIN x each of their coasts scaled by
//!               (bound_speed / its spin's speed)^2, the largest
//! ```
//!
//! The servo's own speed replaces the nameplate's there because the
//! nameplate's is far from it (1.3 to 1.6 times this servo's at 25 %),
//! and squared in the coast bound it leaves no runway. Speed against volts
//! is a line with a positive friction offset, so the volts' ratio under-
//! reads the next level's speed by at most the offset's share (about 8 %
//! from 25 to 50 % on the bench servo), inside [`SPEED_GAIN`]; and a coast
//! under a constant or rising drag grows no faster than the square of its
//! speed. A servo faster than that is caught by the checks after its first
//! spin over the bound.
//!
//! A spin's drive lasts the lesser of that T and [`RISE_TIMES`] of the
//! pilot's 90 % rise time plus [`RISE_PAD_S`]; a level whose T comes out
//! under [`T_MIN_S`] in a direction is left out in it. The pilot runs before any coast is
//! measured: its coast is bounded by the notebooks' spin-down rule, its
//! bound speed for [`COAST_TAU_S`] (oscnb `sitting.COAST_TAU_S`), and it
//! drives half the runway's T. Nothing stops a spin mid-drive but the
//! servo's dead-man, so a spin that ran past its bound or faster than it
//! ends the step.

use super::polarity::{HANDS, MOVE_MIN_COUNTS};
use super::seek::{LEG_MAX_S, WINDOW_S, WRONG_WAY_COUNTS};
use crate::capture::rows::TelFrame;
use crate::envelope::DRIVE_DEADMAN_S;
use crate::fit::{median, polyfit};

pub const GUARD_COUNTS: f64 = 250.0;
pub const SHORT_COUNTS: f64 = 300.0;
/// The levels, shares of the stated voltage: none above the last.
pub const LEVELS: [f64; 3] = [0.25, 0.5, 0.75];
pub const SPINS_EACH: usize = 2;
pub const PILOT_LEVEL: f64 = 0.25;
/// The pilot drives this share of the runway's T.
pub const PILOT_SHARE: f64 = 0.5;
/// The shortest pilot: its last third still holds 50 ms of steady rows
/// (oscnb `sitting.PILOT_MIN_MS`).
pub const PILOT_MIN_S: f64 = 0.150;
pub const RISE_SHARE: f64 = 0.9;
pub const RISE_TIMES: f64 = 3.0;
pub const RISE_PAD_S: f64 = 0.100;
pub const SPEED_GAIN: f64 = 1.25;
pub const COAST_GAIN: f64 = 1.5;
/// The pilot's spin-down as its bound speed for this long: a bare MG90
/// rotor coasting from full speed covers what 93 ms at that speed would.
pub const COAST_TAU_S: f64 = 0.1;
pub const T_MIN_S: f64 = 0.060;
/// The coast's rows after each spin, s.
pub const COAST_S: f64 = 0.400;
/// A go-to zeroes its duty this close to its target, or past it.
pub const ARRIVE_COUNTS: f64 = 40.0;
/// The longest a shaft may take to come to rest at zero duty, s.
pub const SETTLE_MAX_S: f64 = 2.0;
/// The slope windows a rise is read through: 5 ms every 1 ms at 20 kHz.
pub const SLOPE_ROWS: usize = 100;
pub const SLOPE_STEP_ROWS: usize = 20;

/// The two stops, raw counts.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Travel {
    pub lo: f64,
    pub hi: f64,
}

impl Travel {
    pub fn guards(&self) -> (f64, f64) {
        (self.lo + GUARD_COUNTS, self.hi - GUARD_COUNTS)
    }

    /// The guard a spin of sign `dir` starts at: the one it drives away
    /// from.
    pub fn start(&self, dir: i8) -> f64 {
        let (lo, hi) = self.guards();
        if dir > 0 { lo } else { hi }
    }

    /// From `start` to the far guard of a spin of sign `dir`, less
    /// [`SHORT_COUNTS`].
    pub fn runway(&self, dir: i8, start: f64) -> f64 {
        let (lo, hi) = self.guards();
        if dir > 0 {
            hi - SHORT_COUNTS - start
        } else {
            start - (lo + SHORT_COUNTS)
        }
    }

    pub fn middle(&self) -> f64 {
        (self.lo + self.hi) / 2.0
    }

    pub fn inside_guards(&self, pos: f64) -> bool {
        let (lo, hi) = self.guards();
        (lo..=hi).contains(&pos)
    }
}

/// One spin of the twelve.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Slot {
    /// From 1.
    pub n: usize,
    pub level: f64,
    /// The logical duty's sign: + raises the counts.
    pub dir: i8,
}

/// The spins in order: the lowest level first, [`SPINS_EACH`] each way a
/// level, the direction alternating, up first.
pub fn order() -> Vec<Slot> {
    let mut out = Vec::new();
    for level in LEVELS {
        for k in 0..2 * SPINS_EACH {
            out.push(Slot {
                n: out.len() + 1,
                level,
                dir: if k % 2 == 0 { 1 } else { -1 },
            });
        }
    }
    out
}

/// What a spin did that the next windows are sized by.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Seen {
    pub level: f64,
    /// The logical sign it drove.
    pub dir: i8,
    /// The steady speed by the pot, counts/s.
    pub speed_cps: f64,
    /// The coast's travel, counts.
    pub coast_counts: f64,
}

/// Where a window's bounds came from.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum BoundFrom {
    /// The nameplate and every spin of the run, the larger.
    Nameplate,
    /// The spins of this level that ran in this sign.
    Spins { level: f64, dir: i8, n: usize },
    /// No spin of this level ran in this sign: left out.
    Missing { level: f64, dir: i8 },
}

impl BoundFrom {
    pub fn said(&self) -> String {
        let way = |d: i8| if d > 0 { "up" } else { "down" };
        match *self {
            BoundFrom::Nameplate => "the nameplate and the run's spins".into(),
            BoundFrom::Spins { level, dir, n } => {
                format!("the {:.0}% level's {n} spins {}", 100.0 * level, way(dir))
            }
            BoundFrom::Missing { level, dir } => {
                format!(
                    "no spin of the {:.0}% level ran {}",
                    100.0 * level,
                    way(dir)
                )
            }
        }
    }
}

/// A spin's drive as sized.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Window {
    pub level: f64,
    pub dir: i8,
    pub start: f64,
    pub runway: f64,
    pub bound_speed: f64,
    pub bound_coast: f64,
    /// [`RISE_TIMES`] of the pilot's rise and [`RISE_PAD_S`]; None for the
    /// pilot.
    pub t_rise_s: Option<f64>,
    /// The longest T the runway allows.
    pub t_runway_s: f64,
    pub t_s: f64,
    pub from: BoundFrom,
}

impl Window {
    /// The predicted travel at the bounds: the drive, its dead-man and the
    /// coast.
    pub fn travel_bound(&self) -> f64 {
        self.bound_speed * (self.t_s + DRIVE_DEADMAN_S) + self.bound_coast
    }

    /// The drive is long enough to read.
    pub fn fits(&self) -> bool {
        self.t_s >= T_MIN_S
    }

    pub fn said(&self) -> String {
        format!(
            "T {:.0} ms (rise {}, runway {:.0} ms over {:.0} counts), bound {:.0} counts/s and coast {:.0} counts from {}: at most {:.0} counts",
            1e3 * self.t_s,
            self.t_rise_s
                .map_or("not yet read".into(), |t| format!("{:.0} ms", 1e3 * t)),
            1e3 * self.t_runway_s,
            self.runway,
            self.bound_speed,
            self.bound_coast,
            self.from.said(),
            self.travel_bound()
        )
    }
}

/// The windows of a run, sized from the nameplate and what the run has
/// measured.
#[derive(Clone, Debug)]
pub struct Planner {
    travel: Travel,
    /// The nameplate's no-load speed at the stated voltage, counts/s.
    rated_cps: f64,
    t90_s: Option<f64>,
    seen: Vec<Seen>,
}

impl Planner {
    pub fn new(travel: Travel, rated_cps: f64) -> Self {
        Planner {
            travel,
            rated_cps,
            t90_s: None,
            seen: Vec::new(),
        }
    }

    pub fn travel(&self) -> &Travel {
        &self.travel
    }

    pub fn seen(&self) -> &[Seen] {
        &self.seen
    }

    pub fn t90_s(&self) -> Option<f64> {
        self.t90_s
    }

    /// The nameplate's speed at `level` of the stated voltage.
    pub fn nameplate_cps(&self, level: f64) -> f64 {
        self.rated_cps * level
    }

    /// The nameplate's rule, for the pilot and the lowest level.
    pub fn bound_speed(&self, level: f64) -> f64 {
        let measured = self
            .seen
            .iter()
            .map(|s| s.speed_cps * level / s.level)
            .fold(0.0, f64::max);
        SPEED_GAIN * self.nameplate_cps(level).max(measured)
    }

    pub fn bound_coast(&self, bound_speed: f64) -> f64 {
        if self.seen.is_empty() {
            return bound_speed * COAST_TAU_S;
        }
        self.seen
            .iter()
            .map(|s| {
                let k = bound_speed / s.speed_cps;
                COAST_GAIN * s.coast_counts.max(0.0) * k * k
            })
            .fold(0.0, f64::max)
    }

    /// The bounds of `level` in `dir`, and where they came from: the
    /// nameplate's rule at the lowest level, else the next lower level's
    /// spins in `dir`.
    pub fn bounds(&self, level: f64, dir: i8) -> (f64, f64, BoundFrom) {
        let lower = LEVELS
            .iter()
            .copied()
            .filter(|l| *l < level)
            .fold(None, |m: Option<f64>, l| Some(m.map_or(l, |m| m.max(l))));
        let Some(lower) = lower else {
            let bs = self.bound_speed(level);
            return (bs, self.bound_coast(bs), BoundFrom::Nameplate);
        };
        let them: Vec<&Seen> = self
            .seen
            .iter()
            .filter(|s| s.level == lower && s.dir == dir && s.speed_cps > 0.0)
            .collect();
        if them.is_empty() {
            return (f64::NAN, f64::NAN, BoundFrom::Missing { level: lower, dir });
        }
        let fastest = them.iter().map(|s| s.speed_cps).fold(0.0, f64::max);
        let bs = SPEED_GAIN * fastest * level / lower;
        let bc = them
            .iter()
            .map(|s| {
                let k = bs / s.speed_cps;
                COAST_GAIN * s.coast_counts.max(0.0) * k * k
            })
            .fold(0.0, f64::max);
        let n = them.len();
        (
            bs,
            bc,
            BoundFrom::Spins {
                level: lower,
                dir,
                n,
            },
        )
    }

    fn sized(&self, level: f64, dir: i8, start: f64, rise: Option<f64>, share: f64) -> Window {
        let runway = self.travel.runway(dir, start);
        let (bound_speed, bound_coast, from) = self.bounds(level, dir);
        let t_runway_s = (runway - bound_coast) / bound_speed - DRIVE_DEADMAN_S;
        let t = share * t_runway_s;
        let t_s = match rise {
            _ if matches!(from, BoundFrom::Missing { .. }) => f64::NEG_INFINITY,
            Some(r) => r.min(t),
            None => t,
        };
        Window {
            level,
            dir,
            start,
            runway,
            bound_speed,
            bound_coast,
            t_rise_s: rise,
            t_runway_s,
            t_s: if t_s.is_finite() {
                t_s
            } else {
                f64::NEG_INFINITY
            },
            from,
        }
    }

    /// The pilot from `start`: upward at [`PILOT_LEVEL`], [`PILOT_SHARE`] of
    /// the runway's T.
    pub fn pilot(&self, start: f64) -> Window {
        self.sized(PILOT_LEVEL, 1, start, None, PILOT_SHARE)
    }

    /// A spin at `level` of sign `dir` from `start`; it is left out unless
    /// [`Window::fits`].
    pub fn window(&self, level: f64, dir: i8, start: f64) -> Window {
        let rise = self.t90_s.map(|t| RISE_TIMES * t + RISE_PAD_S);
        self.sized(level, dir, start, rise, 1.0)
    }

    /// The pilot read: its rise sizes every later drive.
    pub fn feed_pilot(&mut self, seen: Seen, t90_s: f64) {
        self.t90_s = Some(t90_s);
        self.seen.push(seen);
    }

    pub fn feed(&mut self, seen: Seen) {
        self.seen.push(seen);
    }
}

/// A spin read by the pot, for the runway's bookkeeping only.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpinRead {
    pub start: f64,
    /// At rest after the coast.
    pub end: f64,
    /// Along the spin's sign, counts: the drive's rows, and the whole.
    pub drive_travel: f64,
    pub travel: f64,
    /// The pot's slope over the last third of the drive's rows.
    pub speed_cps: f64,
    /// From the first driven row to the first 5 ms slope at
    /// [`RISE_SHARE`] of the steady speed.
    pub t90_s: Option<f64>,
    pub cut_pos: f64,
    pub coast_counts: f64,
    /// From the cut to the first row from which the pot stays within the
    /// still band of its rest.
    pub coast_s: Option<f64>,
}

impl SpinRead {
    pub fn seen(&self, level: f64, dir: i8) -> Seen {
        Seen {
            level,
            dir,
            speed_cps: self.speed_cps,
            coast_counts: self.coast_counts,
        }
    }
}

fn slope(rows: &[(f64, f64)]) -> Option<f64> {
    let (t, p): (Vec<f64>, Vec<f64>) = rows.iter().copied().unzip();
    let [_, b] = polyfit::<2>(&t, &p)?;
    b.is_finite().then_some(b)
}

/// The spin of sign `dir` from `start` read by its drive and coast rows
/// and its rest `end` after them; None when the rows carry too little pot.
pub fn read_spin(
    dir: i8,
    start: f64,
    drive: &[TelFrame],
    coast: &[TelFrame],
    end: f64,
    still_counts: f64,
    fs: f64,
) -> Option<SpinRead> {
    let d = dir as f64;
    let on = drive
        .iter()
        .position(|f| f.duty_counts.is_some_and(|x| x != 0))
        .unwrap_or(0);
    let pts: Vec<(f64, f64)> = drive[on..]
        .iter()
        .enumerate()
        .filter_map(|(k, f)| f.pos.map(|p| (k as f64 / fs, p as f64)))
        .collect();
    if pts.len() < 3 * SLOPE_ROWS {
        return None;
    }
    let speed_cps = d * slope(&pts[pts.len() - pts.len() / 3..])?;
    let t90_s = (0..)
        .map(|k| k * SLOPE_STEP_ROWS)
        .take_while(|a| a + SLOPE_ROWS <= pts.len())
        .find(|&a| slope(&pts[a..a + SLOPE_ROWS]).is_some_and(|s| d * s >= RISE_SHARE * speed_cps))
        .map(|a| (a + SLOPE_ROWS / 2) as f64 / fs);
    let first = pts.first()?.1;
    let last = pts.last()?.1;
    let cut = coast
        .iter()
        .position(|f| f.duty_counts == Some(0))
        .unwrap_or(0);
    let cut_pos = coast.get(cut).and_then(|f| f.pos).map_or(last, f64::from);
    let after: Vec<f64> = coast
        .get(cut..)
        .unwrap_or(&[])
        .iter()
        .filter_map(|f| f.pos.map(f64::from))
        .collect();
    let coast_s = (0..after.len())
        .find(|&k| after[k..].iter().all(|p| (p - end).abs() <= still_counts))
        .map(|k| k as f64 / fs);
    Some(SpinRead {
        start,
        end,
        drive_travel: d * (last - first),
        travel: d * (end - start),
        speed_cps,
        t90_s,
        cut_pos,
        coast_counts: d * (end - cut_pos),
        coast_s,
    })
}

/// The checks after a spin, at rest: it turned, it stayed inside both
/// guards, and it ran no faster than its bound. A failure ends the step.
pub fn after(w: &Window, r: &SpinRead, travel: &Travel, what: &str) -> Result<(), String> {
    if r.drive_travel < MOVE_MIN_COUNTS {
        return Err(format!(
            "{what} did not turn the shaft: {:.0} counts in its drive, under {MOVE_MIN_COUNTS}. Nothing more drives; {HANDS}, and check that nothing holds the horn",
            r.drive_travel
        ));
    }
    let what_now = match w.from {
        BoundFrom::Nameplate => {
            "The nameplate's speed is too low for this servo; nothing more drives: enter the servo's own no-load speed and start again".to_string()
        }
        _ => format!(
            "Its speed rose with the volts faster than {} predict, past the {SPEED_GAIN} margin; nothing more drives: check that nothing changed on the horn, then start again",
            w.from.said()
        ),
    };
    let too_fast = |why: String| {
        format!(
            "{what} {why}: it ran {:.0} counts from {:.0} to {:.0} against a bound of {:.0}, at {:.0} counts/s against a bound of {:.0}. {what_now}",
            r.travel,
            r.start,
            r.end,
            w.travel_bound(),
            r.speed_cps,
            w.bound_speed
        )
    };
    if !travel.inside_guards(r.end) {
        let (lo, hi) = travel.guards();
        return Err(too_fast(format!(
            "ended at {:.0}, outside the guards {lo:.0}..{hi:.0}",
            r.end
        )));
    }
    if r.speed_cps > w.bound_speed {
        return Err(too_fast("ran faster than its bound".into()));
    }
    Ok(())
}

/// A window of polls whose every reading lies within a band of its median
/// is still.
#[derive(Clone, Debug)]
struct Still {
    t0: f64,
    rows: Vec<f64>,
}

impl Still {
    fn new(t0: f64, pos: f64) -> Self {
        Still {
            t0,
            rows: vec![pos],
        }
    }

    /// Whether the window is complete at `t`, and if so its median when
    /// still.
    fn judge(&mut self, t: f64, pos: f64, band: f64) -> Option<Option<f64>> {
        self.rows.push(pos);
        if t - self.t0 < WINDOW_S - 1e-9 {
            return None;
        }
        let m = median(&self.rows)?;
        Some(self.rows.iter().all(|x| (x - m).abs() <= band).then_some(m))
    }
}

/// The shaft coming to rest at zero duty: a still window of [`WINDOW_S`],
/// within [`SETTLE_MAX_S`].
#[derive(Clone, Debug)]
pub struct Settle {
    t0: f64,
    band: f64,
    window: Still,
}

impl Settle {
    pub fn new(t: f64, pos: f64, band: f64) -> Self {
        Settle {
            t0: t,
            band,
            window: Still::new(t, pos),
        }
    }

    /// The pot at `t`: the rest position once still, an error past
    /// [`SETTLE_MAX_S`], else None.
    pub fn feed(&mut self, t: f64, pos: f64) -> Option<Result<f64, String>> {
        if !pos.is_finite() {
            return Some(Err(dead(pos)));
        }
        match self.window.judge(t, pos, self.band) {
            Some(Some(m)) => return Some(Ok(m)),
            Some(None) => self.window = Still::new(t, pos),
            None => {}
        }
        (t - self.t0 > SETTLE_MAX_S).then(|| {
            Err(format!(
                "the shaft did not come to rest at zero duty within {SETTLE_MAX_S} s (at {pos:.0}): something drives the horn, or the pot is noisy. Nothing more drives; {HANDS}"
            ))
        })
    }
}

/// What the host does next for a go-to.
#[derive(Clone, Debug, PartialEq)]
pub enum Go {
    /// The duty for the next poll period; `price` the window it starts,
    /// which the host prices before the write.
    Drive { q15: i16, price: Option<f64> },
    /// Zero duty, and keep polling.
    Zero,
    /// At rest here.
    Arrived(f64),
    /// Refused: zero, nothing more drives.
    Stopped(String),
}

enum GoPhase {
    Idle,
    Drive { dir: f64 },
    Settle(Settle),
    Over,
}

/// The go-to: drive toward the target at one duty, zero within
/// [`ARRIVE_COUNTS`] of it or past it, then wait until still. A drive is at
/// most [`seek::LEG_MAX_S`](LEG_MAX_S); a still window short of the target
/// is a blocked shaft; a shaft running back [`WRONG_WAY_COUNTS`] from the
/// farthest it got runs the wrong way.
pub struct GoTo {
    target: f64,
    q15: i16,
    band: f64,
    phase: GoPhase,
    t0: f64,
    from: f64,
    best: f64,
    window: Still,
}

impl GoTo {
    /// A go-to to `target` at the duty magnitude `q15`, still within `band`.
    pub fn new(target: f64, q15: i16, band: f64) -> Self {
        GoTo {
            target,
            q15,
            band,
            phase: GoPhase::Idle,
            t0: 0.0,
            from: 0.0,
            best: 0.0,
            window: Still::new(0.0, 0.0),
        }
    }

    pub fn target(&self) -> f64 {
        self.target
    }

    fn stop(&mut self, why: String) -> Go {
        self.phase = GoPhase::Over;
        Go::Stopped(why)
    }

    fn settle(&mut self, t: f64, pos: f64) -> Go {
        self.phase = GoPhase::Settle(Settle::new(t, pos, self.band));
        Go::Zero
    }

    /// The pot read before any duty.
    pub fn start(&mut self, t: f64, pos: f64) -> Go {
        if !pos.is_finite() {
            return self.stop(dead(pos));
        }
        if (pos - self.target).abs() <= ARRIVE_COUNTS {
            return self.settle(t, pos);
        }
        let dir = (self.target - pos).signum();
        self.t0 = t;
        self.from = pos;
        self.best = 0.0;
        self.window = Still::new(t, pos);
        self.phase = GoPhase::Drive { dir };
        Go::Drive {
            q15: self.q15 * dir as i16,
            price: Some(WINDOW_S),
        }
    }

    /// The pot read at `t` after a poll period at the last duty given.
    pub fn feed(&mut self, t: f64, pos: f64) -> Go {
        match std::mem::replace(&mut self.phase, GoPhase::Over) {
            GoPhase::Drive { dir } => self.on_drive(dir, t, pos),
            GoPhase::Settle(mut s) => match s.feed(t, pos) {
                Some(Ok(at)) => Go::Arrived(at),
                Some(Err(why)) => self.stop(why),
                None => {
                    self.phase = GoPhase::Settle(s);
                    Go::Zero
                }
            },
            GoPhase::Idle => self.start(t, pos),
            GoPhase::Over => Go::Stopped("the go-to is over".into()),
        }
    }

    fn on_drive(&mut self, dir: f64, t: f64, pos: f64) -> Go {
        if !pos.is_finite() {
            return self.stop(dead(pos));
        }
        let to = self.target;
        if (pos - to).abs() <= ARRIVE_COUNTS || (pos - to) * dir >= 0.0 {
            return self.settle(t, pos);
        }
        if t - self.t0 > LEG_MAX_S {
            return self.stop(format!(
                "the go-to from {:.0} toward {to:.0} ran {LEG_MAX_S} s and ended at {pos:.0}: the shaft runs too slowly or something holds it. Nothing more drives; {HANDS}",
                self.from
            ));
        }
        let along = (pos - self.from) * dir;
        self.best = self.best.max(along);
        if self.best - along > WRONG_WAY_COUNTS {
            return self.stop(format!(
                "the go-to toward {to:.0} drove {} and the shaft ran {:.0} counts the other way: the drive's sign is not the one the probe proved, or something turns the horn. Nothing more drives; {HANDS}",
                if dir > 0.0 { "up" } else { "down" },
                self.best - along
            ));
        }
        let price = match self.window.judge(t, pos, self.band) {
            None => None,
            Some(Some(m)) => {
                return self.stop(format!(
                    "the go-to toward {to:.0} went still at {m:.0}, short of it: the shaft is blocked between the stops. Nothing more drives; {HANDS}, and check that nothing holds the horn"
                ));
            }
            Some(None) => {
                self.window = Still::new(t, pos);
                Some(WINDOW_S)
            }
        };
        self.phase = GoPhase::Drive { dir };
        Go::Drive {
            q15: self.q15 * dir as i16,
            price,
        }
    }
}

fn dead(pos: f64) -> String {
    format!(
        "the pot reads {pos}, no number: a dead or slipped pot. Nothing more drives; {HANDS}; if it is refused again, check the pot's wiring"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::steps::seek::POLL_S;

    const TRAVEL: Travel = Travel {
        lo: 82.0,
        hi: 3792.0,
    };
    /// The bench nameplate's 600 deg/s at 4.8 V over a 4096-count pot's
    /// 184.33 deg.
    const RATED: f64 = 600.0 * 4096.0 / 184.33;

    #[test]
    fn the_twelve_spins_go_lowest_level_first_alternating_up_first() {
        let o = order();
        assert_eq!(o.len(), 12);
        let levels: Vec<f64> = o.iter().map(|s| s.level).collect();
        assert_eq!(
            levels,
            [
                0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.5, 0.5, 0.75, 0.75, 0.75, 0.75
            ]
        );
        assert!(o.iter().all(|s| s.level <= 0.75));
        let dirs: Vec<i8> = o.iter().map(|s| s.dir).collect();
        assert_eq!(dirs, [1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1]);
        assert_eq!(
            o.iter().map(|s| s.n).collect::<Vec<_>>(),
            (1..=12).collect::<Vec<_>>()
        );
        let starts: Vec<f64> = o.iter().map(|s| TRAVEL.start(s.dir)).collect();
        assert_eq!(&starts[..2], &[332.0, 3542.0], "the guard it leaves");
        assert_eq!(TRAVEL.runway(1, 332.0), 3542.0 - 300.0 - 332.0);
        assert_eq!(TRAVEL.runway(-1, 3542.0), 3542.0 - 332.0 - 300.0);
    }

    fn seen(level: f64, dir: i8, speed: f64, coast: f64) -> Seen {
        Seen {
            level,
            dir,
            speed_cps: speed,
            coast_counts: coast,
        }
    }

    /// A table of runs: (level, sign, start, what was seen) -> T and its
    /// bounds, and the left-out cases.
    #[test]
    fn the_window_is_the_lesser_of_the_rise_and_the_runway() {
        let mut p = Planner::new(TRAVEL, RATED);
        // the pilot: no coast seen, the notebooks' spin-down rule
        let w = p.pilot(332.0);
        let bs = 1.25 * RATED * 0.25;
        assert!((w.bound_speed - bs).abs() < 1e-9);
        assert!((w.bound_coast - bs * 0.1).abs() < 1e-9);
        assert_eq!(w.from, BoundFrom::Nameplate);
        let runway = 2910.0;
        assert_eq!(w.runway, runway);
        let t_run = (runway - bs * 0.1) / bs - DRIVE_DEADMAN_S;
        assert!((w.t_runway_s - t_run).abs() < 1e-12);
        assert!((w.t_s - 0.5 * t_run).abs() < 1e-12);
        assert!(w.travel_bound() <= runway);

        // the bench run's pilot: 2207 counts/s up, rising in 11.5 ms,
        // coasting 70 counts
        p.feed_pilot(seen(0.25, 1, 2207.0, 70.0), 0.0115);
        let rise = 3.0 * 0.0115 + 0.1;
        // the lowest level keeps the nameplate's rule, which the pilot's
        // slower speed does not lower
        for (dir, start) in [(1, 332.0), (-1, 3542.0)] {
            let w = p.window(0.25, dir, start);
            let bs = 1.25 * RATED * 0.25;
            let bc = 1.5 * 70.0 * (bs / 2207.0).powi(2);
            assert!((w.bound_speed - bs).abs() < 1e-9 && (w.bound_coast - bc).abs() < 1e-9);
            assert_eq!(w.from, BoundFrom::Nameplate);
            assert_eq!(w.t_s, rise, "the rise binds: {}", w.said());
        }
        // a higher level waits for the level below it in its direction
        let w = p.window(0.5, -1, 3542.0);
        assert!(!w.fits(), "{}", w.said());
        assert_eq!(
            w.from,
            BoundFrom::Missing {
                level: 0.25,
                dir: -1
            }
        );
        assert!(w.said().contains("no spin of the 25% level ran down"));
        assert!(!p.window(0.75, 1, 332.0).fits(), "no 50 % spin yet");
        // the bench run's four 25 % spins
        for (d, v, c) in [
            (1, 2550.0, 43.0),
            (-1, 2063.0, 78.0),
            (1, 2427.0, 41.0),
            (-1, 2069.0, 82.0),
        ] {
            p.feed(seen(0.25, d, v, c));
        }
        let want = |dir: i8, seen: &[(f64, f64)]| {
            let fastest = seen.iter().map(|s| s.0).fold(0.0, f64::max);
            let bs = 1.25 * fastest * 2.0;
            let bc = seen
                .iter()
                .map(|(v, c)| 1.5 * c * (bs / v).powi(2))
                .fold(0.0, f64::max);
            (dir, bs, bc)
        };
        for (dir, bs, bc) in [
            want(1, &[(2207.0, 70.0), (2550.0, 43.0), (2427.0, 41.0)]),
            want(-1, &[(2063.0, 78.0), (2069.0, 82.0)]),
        ] {
            let w = p.window(0.5, dir, TRAVEL.start(dir));
            assert!((w.bound_speed - bs).abs() < 1e-9, "{dir}: {}", w.said());
            assert!((w.bound_coast - bc).abs() < 1e-9, "{dir}: {}", w.said());
            let n = if dir > 0 { 3 } else { 2 };
            assert_eq!(
                w.from,
                BoundFrom::Spins {
                    level: 0.25,
                    dir,
                    n
                }
            );
            let t_run = (2910.0 - bc) / bs - DRIVE_DEADMAN_S;
            assert!((w.t_s - t_run.min(rise)).abs() < 1e-9);
            assert!(
                w.fits() && w.travel_bound() <= w.runway + 1e-9,
                "{}",
                w.said()
            );
        }
        // the top level from the 50 % spins of its own direction only
        p.feed(seen(0.5, 1, 5200.0, 180.0));
        let w = p.window(0.75, 1, 332.0);
        let bs = 1.25 * 5200.0 * 1.5;
        assert!((w.bound_speed - bs).abs() < 1e-9);
        assert!((w.bound_coast - 1.5 * 180.0 * (bs / 5200.0).powi(2)).abs() < 1e-9);
        assert!(w.fits(), "{}", w.said());
        assert!(!p.window(0.75, -1, 3542.0).fits(), "no 50 % spin down");
        // a coast that leaves no runway leaves the level out
        p.feed(seen(0.5, 1, 5200.0, 1200.0));
        let w = p.window(0.75, 1, 332.0);
        assert!(!w.fits() && w.t_s < T_MIN_S, "{}", w.said());
        // from a start near the far guard nothing fits
        let w = p.window(0.25, 1, 3000.0);
        assert!(w.runway < 300.0 && !w.fits(), "{}", w.said());
    }

    fn row(tick: u64, pos: f64, duty: i16) -> TelFrame {
        TelFrame {
            tick,
            pos: Some(pos.round() as u16),
            duty_counts: Some(duty),
            ..TelFrame::default()
        }
    }

    /// A first-order rise to `v` counts/s with `tau`, `t` s of drive from
    /// `start`, then a constant-deceleration coast `decel` counts/s^2.
    fn spin_rows(
        start: f64,
        v: f64,
        tau: f64,
        t: f64,
        decel: f64,
        dir: f64,
    ) -> (Vec<TelFrame>, Vec<TelFrame>, f64) {
        let fs = 20_000.0;
        let n = (t * fs) as u64;
        let x = |s: f64| v * (s - tau * (1.0 - (-s / tau).exp()));
        let drive: Vec<TelFrame> = (0..n)
            .map(|k| row(k, start + dir * x(k as f64 / fs), 720))
            .collect();
        let v_cut = v * (1.0 - (-t / tau).exp());
        let at = start + dir * x(t);
        let stop = v_cut / decel;
        let coast: Vec<TelFrame> = (0..8000u64)
            .map(|k| {
                let s = (k as f64 / fs).min(stop);
                row(n + k, at + dir * (v_cut * s - 0.5 * decel * s * s), 0)
            })
            .collect();
        let end = at + dir * v_cut * v_cut / (2.0 * decel);
        (drive, coast, end)
    }

    #[test]
    fn a_spin_is_read_by_its_pot() {
        let (d, c, end) = spin_rows(332.0, 3000.0, 0.02, 0.3, 30_000.0, 1.0);
        let r = read_spin(1, 332.0, &d, &c, end.round(), 3.0, 20_000.0).expect("reads");
        assert!((r.speed_cps / 3000.0 - 1.0).abs() < 0.01, "{r:?}");
        // 90 % at tau ln 10, read at its 5 ms window's center
        let t90 = r.t90_s.expect("rises");
        assert!((t90 - 0.02 * 10f64.ln()).abs() < 0.004, "{t90}");
        assert!((r.coast_counts - 150.0).abs() < 2.0, "{r:?}");
        // within the 3-count band 14 ms before the stop at 100 ms
        assert!((r.coast_s.expect("stops") - 0.086).abs() < 0.005, "{r:?}");
        let (d, c, end) = spin_rows(3542.0, 3000.0, 0.02, 0.3, 30_000.0, -1.0);
        let r = read_spin(-1, 3542.0, &d, &c, end.round(), 3.0, 20_000.0).expect("reads");
        assert!((r.speed_cps / 3000.0 - 1.0).abs() < 0.01 && r.travel > 800.0);
        assert!(read_spin(1, 0.0, &d[..100], &c, 0.0, 3.0, 20_000.0).is_none());
    }

    /// After a spin: a shaft that did not turn, one past a guard, and one
    /// faster than its bound each end the step; one inside both passes.
    #[test]
    fn the_checks_after_a_spin() {
        let p = Planner::new(TRAVEL, RATED);
        let w = p.pilot(332.0);
        let r = SpinRead {
            start: 332.0,
            end: 2000.0,
            drive_travel: 1500.0,
            travel: 1668.0,
            speed_cps: 0.9 * w.bound_speed,
            t90_s: Some(0.05),
            cut_pos: 1850.0,
            coast_counts: 150.0,
            coast_s: Some(0.1),
        };
        assert_eq!(after(&w, &r, &TRAVEL, "the pilot"), Ok(()));
        let still = SpinRead {
            drive_travel: 5.0,
            ..r
        };
        assert!(
            after(&w, &still, &TRAVEL, "the pilot")
                .expect_err("still")
                .contains("did not turn")
        );
        let past = SpinRead { end: 3600.0, ..r };
        let why = after(&w, &past, &TRAVEL, "the pilot").expect_err("past");
        assert!(
            why.contains("outside the guards") && why.contains("nameplate's speed is too low"),
            "{why}"
        );
        let fast = SpinRead {
            speed_cps: 1.01 * w.bound_speed,
            ..r
        };
        let why = after(&w, &fast, &TRAVEL, "spin 3").expect_err("fast");
        assert!(
            why.contains("faster than its bound") && why.contains("against a bound of"),
            "{why}"
        );
    }

    /// The go-to drives toward its target, zeroes within 40 counts or past
    /// it, and arrives once still; a blocked shaft and a wrong way refuse.
    #[test]
    fn the_go_to_drives_zeroes_and_waits_until_still() {
        let run = |from: f64, to: f64, cps: f64, coast: f64, block: Option<f64>| {
            let mut g = GoTo::new(to, 1000, 3.0);
            let (mut t, mut pos, mut on) = (0.0, from, 0i16);
            let mut act = g.start(t, pos);
            let mut zero_at = None;
            let mut acts = Vec::new();
            for _ in 0..10_000 {
                acts.push(act.clone());
                match act {
                    Go::Drive { q15, .. } => {
                        on = q15;
                    }
                    Go::Zero => {
                        if on != 0 {
                            zero_at = Some(pos);
                            pos += (on as f64).signum() * coast;
                        }
                        on = 0;
                    }
                    Go::Arrived(at) => return (Ok(at), zero_at, acts),
                    Go::Stopped(why) => return (Err(why), zero_at, acts),
                }
                t += POLL_S;
                if on != 0 {
                    pos += (on as f64).signum() * cps * POLL_S;
                    if let Some(b) = block {
                        pos = pos.max(b);
                    }
                }
                act = g.feed(t, pos);
            }
            panic!("never ends");
        };
        let (got, zero, acts) = run(2000.0, 332.0, 1500.0, 12.0, None);
        let at = got.expect("arrives");
        let z = zero.expect("zeroed");
        assert!(z - 332.0 <= ARRIVE_COUNTS, "zeroed at {z}");
        assert!((at - (z - 12.0)).abs() < 1e-9);
        assert!(acts.iter().all(|a| match a {
            Go::Drive { q15, .. } => *q15 == -1000,
            _ => true,
        }));
        assert!(matches!(acts[0], Go::Drive { price: Some(_), .. }));
        // already there: no drive
        let (got, zero, acts) = run(340.0, 332.0, 1500.0, 0.0, None);
        assert_eq!(got, Ok(340.0));
        assert!(zero.is_none() && !acts.iter().any(|a| matches!(a, Go::Drive { .. })));
        // a fast shaft overshoots past the target: zeroed past it
        let (got, ..) = run(2000.0, 332.0, 4000.0, 30.0, None);
        assert!(got.expect("arrives") < 332.0);
        let (got, ..) = run(2000.0, 332.0, 1500.0, 0.0, Some(1500.0));
        assert!(got.expect_err("blocked").contains("blocked"));
        let (got, ..) = run(2000.0, 332.0, -1500.0, 0.0, None);
        assert!(got.expect_err("wrong way").contains("other way"));
        let mut s = Settle::new(0.0, 100.0, 3.0);
        let mut t = 0.0;
        let out = loop {
            t += POLL_S;
            if let Some(r) = s.feed(t, 100.0 + 10.0 * (t * 50.0).sin()) {
                break r;
            }
        };
        assert!(out.expect_err("restless").contains("did not come to rest"));
        assert!(t > SETTLE_MAX_S);
    }
}
