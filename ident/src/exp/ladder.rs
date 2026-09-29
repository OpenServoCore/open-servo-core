//! Steady-state duty ladder: constant-duty sweeps across the travel, both
//! directions, for the Ke fit and the kinetic friction line. Each rung
//! seeks the start band at one end of the guard, brakes, rests, then runs
//! free toward the other end collecting ident windows paired with pot
//! position; the steady segment (settle windows dropped, ends trimmed, slip
//! zone masked) gives omega as the pos-slope LS, plus the segment's mean
//! current and winding volts. Fits live in [`crate::fits`]; rungs whose
//! segment is too short or that stall mid-sweep are dropped with a warning,
//! not silently.
//!
//! Rungs run free: above the stall-safe duty, on the firmware limiter, the
//! stall timer, the travel guard and the [`Runway`]. They climb from the
//! bottom, and the first one whose need does not fit the runway ends the
//! ladder. A rung climbs on the current limit - its windows governed, the
//! applied duty under the goal - then settles and runs, and brakes at its
//! end position with the brake idiom, never a coast. What each rung
//! measured - speed, climb acceleration, stop, and how fast it was polled -
//! sizes the next: a poll yields at most one window, so the steady windows
//! take the poll cadence, not the window period, and a stretch of them in
//! a slip zone is travel the fit never sees. The brake points are planned
//! on the inset guard; the run aborts only on the soft limits, so a brake a
//! slow read made late is no abort.
//!
//! The limit governing a still shaft is the firmware's verdict that it is
//! blocked ([`limit_holds`]), as the stall fold is, and ends the run. A rung
//! still governed after twice its predicted climb ends the run when the
//! shaft is still (blocked), and declines the ladder when it is moving: the
//! load is more than the limit drives. A ladder of fewer than [`MIN_RUNGS`]
//! rungs, or whose top speed is under [`MIN_SPAN`] times its bottom one,
//! declines.
//!
//! Rungs alternate +duty then -duty at each level so the pot ends near the
//! next sweep's start and seeks stay short. A seek that comes to rest short
//! of both its target and a stop ends the run ([`super::seek::at_stop`]):
//! the shaft is blocked.

use core::fmt;

use super::seek;
use super::{
    AbortReason, Cmd, Experiment, GOVERNED, LIMIT_YIELD_FOLDED, RigParams, WindowSample,
    WindowStream, limit_holds,
};
use crate::fitmath::{linear_ls, mean};
use crate::fits::{FrictionFit, KeFit, RungPoint, friction_line, ke_fit};
use crate::frame::{SeqUnwrap, TelemetrySnapshot};
use crate::regs::control;
use crate::runway::{
    BRAKE_DUTY_Q15, BRAKE_POLL_MS, BRAKE_POLLS, BRAKE_REST_EPS, Need, Runway, STOP_MARGIN, fits,
};

const Q15: f64 = 32767.0;

/// Rungs, both ways, the fit needs.
pub const MIN_RUNGS: usize = 3;
/// The top rung's speed over the bottom one's the fit needs.
pub const MIN_SPAN: f64 = 2.0;
/// Speeds are read over at least this much of the drive, ms: over one poll
/// a count of pot noise is a large share of the travel.
const SPEED_SPAN_MS: f64 = 10.0;

/// A rung's settle and steady windows are sized this much over the time
/// the last rung's cadence gives them.
const STEADY_MARGIN: f64 = 1.25;

/// A rung is still when its reads over this long span no more than the
/// seek's drift: from rest a governed climb travels several times that in
/// this time even at [`crate::runway::ACCEL_PRIOR`], however fast it is
/// polled.
const STILL_MS: f64 = 100.0;

pub struct LadderCfg {
    /// Rung duties, q15, run bottom up as +d then -d each (26/33/40/47/55/64%).
    pub rungs_q15: Vec<i16>,
    /// Seek drive toward the start end, q15.
    pub seek_duty_q15: i16,
    /// Pause between a rung's reads: a poll yields at most one window, so
    /// a pause only spends travel.
    pub poll_ms: u32,
    /// One telemetry snapshot on the bus, ms: a rung polls at this plus
    /// `poll_ms` until a rung has timed its own polls.
    pub snapshot_ms: f64,
    pub seek_poll_ms: u32,
    /// Rest after every seek and every sweep.
    pub rest_ms: u32,
    /// Still detect: pos delta <= eps across this many consecutive polls.
    pub stall_eps: u16,
    pub stall_polls: u32,
    /// Seek gives up (with a warning) after this many polls.
    pub seek_cap_polls: u32,
    /// Fraction trimmed off each end of a sweep's accepted windows.
    pub trim_frac: f64,
    /// Minimum steady windows for a rung to enter the fits.
    pub min_steady: usize,
}

impl Default for LadderCfg {
    fn default() -> Self {
        Self {
            // 26/33/40/47/55/64% of 32767
            rungs_q15: vec![8520, 10813, 13107, 15400, 18022, 20971],
            seek_duty_q15: 8520,
            poll_ms: 0,
            // the bench bus: two reads
            snapshot_ms: 3.5,
            seek_poll_ms: 30,
            rest_ms: 300,
            stall_eps: 3,
            stall_polls: 10,
            seek_cap_polls: 400,
            trim_frac: 0.15,
            min_steady: 12,
        }
    }
}

/// Why the ladder gives the fit nothing.
#[derive(Clone, Debug, PartialEq)]
pub enum Declined {
    /// The rung at `duty_q15` was still governed after twice its predicted
    /// climb, the shaft moving.
    Heavy { duty_q15: i16 },
    /// `rungs` rungs ran both ways into the fit, under [`MIN_RUNGS`].
    Thin { rungs: usize },
    /// Bottom and top rung speeds, counts/ms: under [`MIN_SPAN`] apart.
    Narrow { lo: f64, hi: f64 },
}

impl fmt::Display for Declined {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Declined::Heavy { duty_q15 } => write!(
                f,
                "the {} rung was still held at the current limit after twice its predicted \
                 climb: the load is too heavy for the current limit",
                pct(*duty_q15)
            ),
            Declined::Thin { rungs } => write!(
                f,
                "{rungs} of the ladder's rungs ran both ways inside the travel, and the fit \
                 needs {MIN_RUNGS}"
            ),
            Declined::Narrow { lo, hi } => write!(
                f,
                "the top rung ran at {hi:.1} counts/ms, under twice the bottom rung's \
                 {lo:.1}: the rungs span too little speed to fit"
            ),
        }
    }
}

fn pct(duty_q15: i16) -> String {
    format!("{:+.0}%", duty_q15 as f64 / Q15 * 100.0)
}

/// One accepted sweep window with the pot position it was read with: raw
/// for the slip mask and the runway, the kernel's counts for the slope.
#[derive(Copy, Clone, Debug)]
struct SweepSample {
    w: WindowSample,
    pos: u16,
    counts: f64,
}

/// One rung reduced; `used` = false rungs carry their reason in `note`.
#[derive(Clone, Debug)]
pub struct RungSummary {
    pub duty_q15: i16,
    pub omega: f64,
    pub omega_r2: f64,
    pub i: f64,
    pub v: f64,
    pub windows: usize,
    pub used: bool,
    pub note: Option<String>,
}

#[derive(Clone, Debug)]
pub struct LadderResult {
    pub ke: KeFit,
    pub fric_fwd: Option<FrictionFit>,
    pub fric_rev: Option<FrictionFit>,
    pub rungs: Vec<RungSummary>,
    pub warnings: Vec<String>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum After {
    Rung,
    Next,
    Finish,
}

enum Phase {
    ModeWrite,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    BrakeWait,
    BrakeRead,
    BrakeEval,
    Rest(After),
    RungSet,
    RungRead,
    RungEval,
    FinishTorque,
    Finished,
}

/// The rung in flight.
struct Rung {
    goal: i16,
    need: Need,
    /// Where it started from rest.
    from: u16,
    t0: Option<f64>,
    climbing: bool,
}

/// A brake in progress, from `from` at speed `v`.
struct Brake {
    from: u16,
    v: f64,
    last: u16,
    polls: u32,
    seek: bool,
    after: After,
}

pub struct Ladder {
    cfg: LadderCfg,
    params: RigParams,
    runway: Runway,
    phase: Phase,
    /// Sweep index: rung level = sweep / 2, direction = +1 then -1.
    sweep: usize,
    clock: SeqUnwrap,
    /// (window time ms, raw pos) of every read of the drive in progress.
    reads: Vec<(f64, u16)>,
    last_pos: Option<u16>,
    still: u32,
    polls: u32,
    start: Option<u16>,
    halt: Option<AbortReason>,
    windows: WindowStream,
    sweep_samples: Vec<SweepSample>,
    stalled_note: bool,
    rung: Option<Rung>,
    brake: Option<Brake>,
    /// Where the last brake came to rest.
    rest_pos: Option<u16>,
    /// A rung measured its speed: from then on the seek sizes nothing.
    rung_measured: bool,
    /// Time between the last rung's polls, ms.
    cadence: Option<f64>,
    sized: Vec<(i16, Need)>,
    declined: Option<Declined>,
    rungs: Vec<RungSummary>,
    warnings: Vec<String>,
}

impl Ladder {
    pub fn new(mut cfg: LadderCfg, params: &RigParams, runway: Runway) -> Self {
        cfg.rungs_q15.sort_unstable();
        cfg.rungs_q15.dedup();
        Self {
            cfg,
            params: *params,
            runway,
            phase: Phase::ModeWrite,
            sweep: 0,
            clock: SeqUnwrap::default(),
            reads: Vec::new(),
            last_pos: None,
            still: 0,
            polls: 0,
            start: None,
            halt: None,
            windows: WindowStream::new(params),
            sweep_samples: Vec::new(),
            stalled_note: false,
            rung: None,
            brake: None,
            rest_pos: None,
            rung_measured: false,
            cadence: None,
            sized: Vec::new(),
            declined: None,
            rungs: Vec::new(),
            warnings: Vec::new(),
        }
    }

    /// Why the ladder gives the fit nothing, once it is done.
    pub fn declined(&self) -> Option<&Declined> {
        self.declined.as_ref()
    }

    /// Every sweep that ran, with the need it was sized to.
    pub fn sized(&self) -> &[(i16, Need)] {
        &self.sized
    }

    pub fn runway(&self) -> &Runway {
        &self.runway
    }

    pub fn warnings(&self) -> &[String] {
        &self.warnings
    }

    fn dir(&self) -> i8 {
        if self.sweep.is_multiple_of(2) { 1 } else { -1 }
    }

    fn duty(&self) -> i16 {
        let d = self.cfg.rungs_q15[self.sweep / 2];
        if self.dir() > 0 { d } else { -d }
    }

    fn time(&mut self, o: &TelemetrySnapshot) -> f64 {
        self.clock.push(o.agg_seq) as f64 * self.params.agg_period_ms
    }

    /// Speed over the last [`SPEED_SPAN_MS`] of reads, counts/ms.
    fn speed(&self) -> Option<f64> {
        let ((t0, p0), (t1, p1)) = self.span()?;
        (t1 > t0).then(|| p1.abs_diff(p0) as f64 / (t1 - t0))
    }

    /// The last read and the latest one at least [`SPEED_SPAN_MS`] before
    /// it, else the first.
    fn span(&self) -> Option<((f64, u16), (f64, u16))> {
        let &last = self.reads.last()?;
        let &back = self
            .reads
            .iter()
            .rev()
            .find(|r| r.0 <= last.0 - SPEED_SPAN_MS)
            .or(self.reads.first())?;
        Some((back, last))
    }

    /// The reads of the drive in progress over its last `ms` span no more
    /// than the seek's drift; false until it has run that long.
    fn still_over(&self, ms: f64) -> bool {
        let Some(&(t1, _)) = self.reads.last() else {
            return false;
        };
        if self.reads.first().is_none_or(|r| t1 - r.0 < ms) {
            return false;
        }
        let (lo, hi) = self
            .reads
            .iter()
            .rev()
            .take_while(|r| r.0 >= t1 - ms)
            .fold((u16::MAX, 0), |(lo, hi), r| (lo.min(r.1), hi.max(r.1)));
        hi - lo <= self.params.stall_eps * self.params.stall_polls as u16
    }

    /// A drive read every few ms moves a few counts a poll even when
    /// cruising slowly: stillness is judged over [`SPEED_SPAN_MS`].
    fn track_still_over_span(&mut self) {
        let still = self.span().is_some_and(|(a, b)| {
            b.0 - a.0 >= SPEED_SPAN_MS && b.1.abs_diff(a.1) <= self.cfg.stall_eps
        });
        self.still = if still { self.still + 1 } else { 0 };
    }

    /// Travel before the next read at speed `v`: one poll's worth.
    fn lead(&self, v: f64) -> f64 {
        match self.reads.as_slice() {
            [.., a, b] => v * (b.0 - a.0),
            _ => 0.0,
        }
    }

    /// Settle and steady windows at the cadence the last rung was polled
    /// at: a poll yields at most one window, however short the window.
    /// [`STEADY_MARGIN`] over that leaves room for a slow read or two.
    fn steady_ms(&self) -> f64 {
        let windows = self.params.settle_windows as f64
            + self.cfg.min_steady as f64 / (1.0 - 2.0 * self.cfg.trim_frac);
        let poll = self
            .cadence
            .unwrap_or(self.cfg.snapshot_ms + self.cfg.poll_ms as f64);
        STEADY_MARGIN * windows * self.params.agg_period_ms.max(poll)
    }

    fn reset_motion_track(&mut self) {
        self.last_pos = None;
        self.still = 0;
        self.polls = 0;
        self.start = None;
        self.reads.clear();
    }

    /// The shaft came to rest driving `dir`: fine at a stop, the end of
    /// the run anywhere else.
    fn rested(&mut self, pos: u16, dir: i8) -> bool {
        let start = self.start.unwrap_or(pos);
        match seek::at_stop(start, pos, dir, self.params.stops) {
            Ok(()) => true,
            Err(reason) => self.block(reason),
        }
    }

    fn block(&mut self, reason: AbortReason) -> bool {
        self.halt = Some(reason);
        self.phase = Phase::FinishTorque;
        false
    }

    fn track_still(&mut self, pos: u16) {
        self.start.get_or_insert(pos);
        if let Some(last) = self.last_pos
            && pos.abs_diff(last) <= self.cfg.stall_eps
        {
            self.still += 1;
        } else {
            self.still = 0;
        }
        self.last_pos = Some(pos);
    }

    /// Brake a drive moving `motion` from `pos` at `v`; `after` says what
    /// follows the rest.
    fn brake(&mut self, motion: i8, pos: u16, v: f64, seek: bool, after: After) -> Cmd {
        self.brake = Some(Brake {
            from: pos,
            v,
            last: pos,
            polls: 0,
            seek,
            after,
        });
        self.windows.mark_transition();
        self.phase = Phase::BrakeWait;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: -(motion as i32) * BRAKE_DUTY_Q15 as i32,
        }
    }

    fn off(&mut self) -> Cmd {
        self.phase = Phase::FinishTorque;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: 0,
        }
    }

    /// The slip zone's share of the stretch a rung sized `n` collects its
    /// steady windows over: windows there never reach the fit.
    fn masked(&self, n: &Need) -> f64 {
        let (lo, hi) = self.runway.guard();
        let (lo, hi) = (lo as f64, hi as f64);
        let stop = STOP_MARGIN * n.stop;
        let (from, to) = if self.dir() > 0 {
            (self.runway.start(1) as f64 + n.climb, hi - stop)
        } else {
            (lo + stop, self.runway.start(-1) as f64 - n.climb)
        };
        self.params
            .slip
            .map_or(0.0, |(s0, s1)| (s1 as f64).min(to) - (s0 as f64).max(from))
            .max(0.0)
    }

    /// Size the next rung; None ends the ladder below it.
    fn size(&mut self) -> Option<Need> {
        let duty = self.duty();
        let room = self.runway.room();
        let plan = self
            .runway
            .plan(self.dir(), duty as f64 / Q15, self.steady_ms())
            .map(|n| Need {
                run: n.run + self.masked(&n),
                ..n
            });
        let why = match plan {
            Some(n) if fits(&n, room) => return Some(n),
            Some(n) => format!(
                "the {} rung needs {:.0} counts of travel and {room:.0} are free: the ladder \
                 ends below it",
                pct(duty),
                n.total()
            ),
            None => format!(
                "the {} rung has no measured speed or stop to size it by: the ladder ends below \
                 it",
                pct(duty)
            ),
        };
        self.warnings.push(why);
        None
    }

    fn seek_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let t = self.time(o);
        self.reads.push((t, o.pos));
        self.track_still(o.pos);
        let motion = -self.dir();
        if o.limit_flags & LIMIT_YIELD_FOLDED != 0 {
            let start = self.start.unwrap_or(o.pos);
            self.block(seek::blocked(start, o.pos));
            return self.off();
        }
        let target = self.runway.start(self.dir()) as f64;
        let v = self.speed().unwrap_or(0.0);
        let stop = self.runway.stop(v).unwrap_or(0.0);
        let ahead = o.pos as f64 + motion as f64 * (self.lead(v) + stop);
        let arrived = if motion > 0 {
            ahead >= target
        } else {
            ahead <= target
        };
        if arrived {
            return self.brake(motion, o.pos, v, true, After::Rung);
        }
        if self.still >= self.cfg.stall_polls {
            if !self.rested(o.pos, motion) {
                return self.off();
            }
            return self.brake(motion, o.pos, 0.0, true, After::Rung);
        }
        self.polls += 1;
        if self.polls >= self.cfg.seek_cap_polls {
            self.warnings
                .push(format!("sweep {}: seek gave up, starting here", self.sweep));
            return self.brake(motion, o.pos, v, true, After::Rung);
        }
        self.phase = Phase::SeekRead;
        Cmd::Pause {
            ms: self.cfg.seek_poll_ms,
        }
    }

    fn brake_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let Some(mut b) = self.brake.take() else {
            return self.off();
        };
        b.polls += 1;
        if o.pos.abs_diff(b.last) >= BRAKE_REST_EPS && b.polls < BRAKE_POLLS {
            b.last = o.pos;
            self.brake = Some(b);
            self.phase = Phase::BrakeRead;
            return Cmd::Pause { ms: BRAKE_POLL_MS };
        }
        if !b.seek || !self.rung_measured {
            if b.seek {
                self.runway.ran(self.cfg.seek_duty_q15 as f64 / Q15, b.v);
            }
            self.runway.stopped(b.v, o.pos.abs_diff(b.from) as f64);
        }
        self.rest_pos = Some(o.pos);
        self.phase = Phase::Rest(b.after);
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: 0,
        }
    }

    fn rung_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let t = self.time(o);
        self.reads.push((t, o.pos));
        let window = self.windows.push(o);
        let (dir, duty) = (self.dir(), self.duty());
        let v_now = self.speed().unwrap_or(0.0);
        let Some(r) = self.rung.as_mut() else {
            return self.off();
        };
        let governed = o.duty_mean_q15 != r.goal;
        let t0 = *r.t0.get_or_insert(t);
        let (need, from, climbing, goal) = (r.need, r.from, r.climbing, r.goal);
        if climbing {
            r.climbing = governed;
        }
        let still = if climbing {
            self.still_over(STILL_MS)
        } else {
            self.track_still_over_span();
            self.still >= self.cfg.stall_polls
        };
        if limit_holds(o, still) {
            self.block(seek::blocked(from, o.pos));
            return self.off();
        }
        if let Some(w) = window {
            self.sweep_samples.push(SweepSample {
                w,
                pos: o.pos,
                counts: self.params.pot.counts(o.pos),
            });
        }
        let mut done = false;
        if climbing && !governed {
            // v^2 = 2 a d from rest: the start time never enters, and with
            // the acceleration falling as the shaft speeds up it reads the
            // lowest of the averages
            let d = o.pos.abs_diff(from) as f64;
            if t - t0 >= SPEED_SPAN_MS && d > 0.0 {
                self.runway.climbed(v_now * v_now / (2.0 * d) * 1000.0);
            }
            // the speed still settles after the limit lets go
            self.windows.mark_goal(goal);
        } else if climbing && t - t0 > 2.0 * need.climb_ms {
            if still {
                self.block(seek::blocked(from, o.pos));
                return self.off();
            }
            self.declined = Some(Declined::Heavy { duty_q15: duty });
            self.close_rung();
            return self.brake(dir, o.pos, v_now, false, After::Finish);
        } else if !climbing && still {
            self.stalled_note = true;
            done = true;
        }
        let stop = self.runway.stop(need.v.max(v_now)).unwrap_or(need.stop);
        let end = self.runway.brake_at(dir, stop);
        let ahead = o.pos as f64 + dir as f64 * self.lead(v_now);
        done |= if dir > 0 { ahead >= end } else { ahead <= end };
        if done {
            self.close_rung();
            return self.brake(dir, o.pos, v_now, false, After::Next);
        }
        self.phase = Phase::RungRead;
        Cmd::Pause {
            ms: self.cfg.poll_ms,
        }
    }

    /// Reduce the finished sweep into a rung summary; a used rung's speed
    /// sizes the next.
    fn close_rung(&mut self) {
        let duty = self.duty();
        let n_raw = self.sweep_samples.len();
        if let [first, .., last] = self.reads.as_slice() {
            self.cadence = Some((last.0 - first.0) / (self.reads.len() - 1) as f64);
        }
        let trim = (n_raw as f64 * self.cfg.trim_frac) as usize;
        let steady: Vec<&SweepSample> = self.sweep_samples[trim..n_raw.saturating_sub(trim)]
            .iter()
            .filter(|s| !self.params.in_slip(s.pos))
            .collect();
        let mut note = None;
        if self.stalled_note {
            note = Some("stalled mid-sweep".into());
        } else if self.windows.declined() {
            note = Some(GOVERNED.into());
        } else if steady.len() < self.cfg.min_steady {
            note = Some(format!("steady segment too short ({})", steady.len()));
        }
        let used = note.is_none();
        let mut rung = RungSummary {
            duty_q15: duty,
            omega: 0.0,
            omega_r2: 0.0,
            i: 0.0,
            v: 0.0,
            windows: steady.len(),
            used,
            note,
        };
        if used {
            let pos_t: Vec<(f64, f64)> = steady
                .iter()
                .map(|s| (s.w.t_ms / 1000.0, s.counts))
                .collect();
            let raw_t: Vec<(f64, f64)> = steady.iter().map(|s| (s.w.t_ms, s.pos as f64)).collect();
            let iv: Vec<f64> = steady.iter().map(|s| s.w.i).collect();
            // duty * vdiff / 32767 is |v|; re-sign by the drive direction
            let vv: Vec<f64> = steady
                .iter()
                .map(|s| s.w.duty_q15 * s.w.vdiff / Q15 * self.dir() as f64)
                .collect();
            match (linear_ls(&pos_t), mean(&iv), mean(&vv)) {
                (Some(slope), Some(i), Some(v)) => {
                    rung.omega = slope.b;
                    rung.omega_r2 = slope.r2;
                    rung.i = i;
                    rung.v = v;
                    if let Some(raw) = linear_ls(&raw_t) {
                        self.runway.ran(duty as f64 / Q15, raw.b.abs());
                        self.rung_measured = true;
                    }
                }
                _ => {
                    rung.used = false;
                    rung.note = Some("degenerate steady segment".into());
                }
            }
        }
        if let Some(n) = &rung.note {
            self.warnings.push(format!("rung {duty}: {n}"));
        }
        self.rungs.push(rung);
        self.sweep_samples.clear();
        self.stalled_note = false;
        self.rung = None;
    }

    /// The ladder as the fit would take it: enough rungs, run both ways,
    /// spanning enough speed.
    fn judge(&self) -> Option<Declined> {
        let used = |d: i16| {
            self.rungs
                .iter()
                .find(|r| r.duty_q15 == d && r.used)
                .map(|r| r.omega.abs() / 1000.0)
        };
        let speeds: Vec<f64> = self
            .cfg
            .rungs_q15
            .iter()
            .filter_map(|&d| Some((used(d)? + used(-d)?) / 2.0))
            .collect();
        match speeds.as_slice() {
            s if s.len() < MIN_RUNGS => Some(Declined::Thin { rungs: s.len() }),
            [lo, .., hi] if *hi < MIN_SPAN * lo => Some(Declined::Narrow { lo: *lo, hi: *hi }),
            _ => None,
        }
    }

    /// Ke + friction line over the used rungs; R comes from the resistance
    /// run. None until at least two usable rungs exist.
    pub fn fit(&self, r_vpc: f64) -> Option<LadderResult> {
        let pts: Vec<RungPoint> = self
            .rungs
            .iter()
            .filter(|r| r.used)
            .map(|r| RungPoint {
                omega: r.omega,
                i: r.i,
                v: r.v,
            })
            .collect();
        let ke = ke_fit(&pts, r_vpc)?;
        Some(LadderResult {
            ke,
            fric_fwd: friction_line(&pts, 1),
            fric_rev: friction_line(&pts, -1),
            rungs: self.rungs.clone(),
            warnings: self.warnings.clone(),
        })
    }
}

impl Experiment for Ladder {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            Phase::TorqueOn => {
                self.phase = Phase::SeekSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::SeekSet => {
                self.reset_motion_track();
                self.windows.mark_transition();
                self.phase = Phase::SeekRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: -(self.dir() as i32) * self.cfg.seek_duty_q15 as i32,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => match obs {
                Some(o) => self.seek_eval(o),
                None => {
                    self.phase = Phase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            },
            Phase::BrakeWait => {
                self.phase = Phase::BrakeRead;
                Cmd::Pause { ms: BRAKE_POLL_MS }
            }
            Phase::BrakeRead => {
                self.phase = Phase::BrakeEval;
                Cmd::Read
            }
            Phase::BrakeEval => match obs {
                Some(o) => self.brake_eval(o),
                None => {
                    self.phase = Phase::BrakeWait;
                    Cmd::Pause { ms: 0 }
                }
            },
            Phase::Rest(after) => {
                self.phase = match after {
                    After::Rung => Phase::RungSet,
                    After::Next => {
                        self.sweep += 1;
                        if self.sweep < 2 * self.cfg.rungs_q15.len() {
                            Phase::SeekSet
                        } else {
                            Phase::FinishTorque
                        }
                    }
                    After::Finish => Phase::FinishTorque,
                };
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::RungSet => {
                let Some(need) = self.size() else {
                    self.phase = Phase::FinishTorque;
                    return self.step(None);
                };
                let goal = self.duty();
                let from = self.rest_pos.or(self.last_pos).unwrap_or(0);
                self.sized.push((goal, need));
                self.reset_motion_track();
                self.rung = Some(Rung {
                    goal,
                    need,
                    from,
                    t0: None,
                    climbing: true,
                });
                self.windows.mark_goal(goal);
                self.phase = Phase::RungRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: goal as i32,
                }
            }
            Phase::RungRead => {
                self.phase = Phase::RungEval;
                Cmd::Read
            }
            Phase::RungEval => match obs {
                Some(o) => self.rung_eval(o),
                None => {
                    self.phase = Phase::RungRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            },
            Phase::FinishTorque => {
                if self.halt.is_none() && self.declined.is_none() {
                    self.declined = self.judge();
                }
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }

    fn halted(&self) -> Option<AbortReason> {
        self.halt
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{Bus, FakeServo, bench_mg90, bent_pot, pump, pump_on};
    use super::super::{Guarded, RigParams};
    use super::*;
    use crate::limits::{GUARD_INSET, duty_for};
    use crate::pot::Pot;
    use crate::runway::{Envelope, Line, Supply};

    /// A guard inside the fake's stops (200, 4000), those stops as
    /// calibrated: a rung that brakes at the guard never meets a stop.
    fn rig() -> RigParams {
        RigParams::new(Some((250, 3950)), 1100).with_stops((200, 4000))
    }

    fn physical_servo() -> FakeServo {
        let mut s = FakeServo::new(3.37);
        s.physical_motion = true;
        s.fv = 0.006;
        s
    }

    /// Aborts on the soft limits the guard in `params` was inset from, as
    /// the CLI's ladder does.
    fn guarded(exp: Ladder, params: RigParams) -> Guarded<Ladder> {
        let (lo, hi) = params.pos_guard.unwrap();
        let inset = GUARD_INSET as i32;
        let soft = (lo as i32 - inset, hi as i32 + inset);
        Guarded::new(exp, params.abort_at_soft(soft))
    }

    fn ladder(cfg: LadderCfg, params: &RigParams) -> Ladder {
        Ladder::new(cfg, params, Runway::new(params.pos_guard.unwrap()))
    }

    fn run_cfg(servo: &mut FakeServo, cfg: LadderCfg, params: RigParams) -> (Ladder, Vec<String>) {
        let mut exp = guarded(ladder(cfg, &params), params);
        let log = pump(&mut exp, servo, 2_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    fn run_e3(servo: &mut FakeServo, params: RigParams) -> (Ladder, Vec<String>) {
        run_cfg(servo, LadderCfg::default(), params)
    }

    #[test]
    fn recovers_planted_ke_and_friction_line() {
        let mut servo = physical_servo();
        let (exp, log) = run_e3(&mut servo, rig());
        assert!(!log.contains(&"OVERRUN".to_string()));
        assert_eq!(exp.declined(), None);
        let fit = exp.fit(3.37).expect("usable rungs");
        assert_eq!(fit.rungs.iter().filter(|r| r.used).count(), 12);
        assert!(
            (fit.ke.ke_vpc - 0.1731).abs() / 0.1731 < 0.01,
            "ke {}",
            fit.ke.ke_vpc
        );
        assert!(fit.ke.r2 > 0.999, "r2 {}", fit.ke.r2);
        for fr in [fit.fric_fwd.unwrap(), fit.fric_rev.unwrap()] {
            assert!((fr.fc - 20.0).abs() < 2.0, "fc {}", fr.fc);
            assert!((fr.fv - 0.006).abs() / 0.006 < 0.1, "fv {}", fr.fv);
        }
    }

    /// The sweeps cover raw 1450..2650, where the bent pot reads 1.2x
    /// wide: the raw slope over-reads omega and Ke lands ~17% low. Fitted
    /// in the table's counts, the kernel's domain, Ke is the planted one.
    #[test]
    fn nonlinear_pot_biases_raw_ke_and_the_live_table_recovers_it() {
        let table = bent_pot();
        assert_eq!(table.validate(200, 4000), Ok(()));
        let ke = |pot: Pot| {
            let mut servo = physical_servo();
            servo.pot = Some(table);
            let params = RigParams { pot, ..rig() };
            let (exp, _) = run_e3(&mut servo, params);
            exp.fit(3.37).expect("usable rungs").ke.ke_vpc
        };
        let raw = ke(Pot::RAW);
        let lin = ke(Pot::live(table));
        assert!((raw - 0.1731) / 0.1731 < -0.10, "raw ke {raw}");
        assert!((lin - 0.1731).abs() / 0.1731 < 0.01, "linearized ke {lin}");
    }

    #[test]
    fn slip_zone_samples_are_masked() {
        let params = RigParams {
            slip: Some((1250, 1650)),
            ..rig()
        };
        let mut clean = physical_servo();
        let (exp_clean, _) = run_e3(&mut clean, params);
        let mut glitched = physical_servo();
        // +80-count pot artifact strictly inside the masked slip zone, low
        // enough that the +80 readings also stay inside the mask
        glitched.glitch_zone = Some((1460.0, 1560.0));
        let (exp_glitch, _) = run_e3(&mut glitched, params);
        let a = exp_clean.fit(3.37).unwrap();
        let b = exp_glitch.fit(3.37).unwrap();
        assert!(
            (a.ke.ke_vpc - b.ke.ke_vpc).abs() < 1e-9,
            "masked glitch must not move the fit: {} vs {}",
            a.ke.ke_vpc,
            b.ke.ke_vpc
        );
    }

    /// A 900-count travel, the ladder told a snapshot costs no time: the
    /// first rung sizes its steady windows at the window period and runs,
    /// but read at the bus's 3.5 ms it collects too few and is dropped with
    /// a warning; sized at the cadence it was read at, the next rung does
    /// not fit, and the ladder declines.
    #[test]
    fn short_travel_rung_drops_with_warning() {
        let mut servo = physical_servo();
        servo.pos = 2050.0;
        servo.ends = (1600.0, 2500.0);
        let params = RigParams {
            pos_guard: Some((1600, 2500)),
            ..rig()
        };
        let cfg = LadderCfg {
            min_steady: 150,
            snapshot_ms: 0.0,
            ..LadderCfg::default()
        };
        let (exp, _) = run_cfg(&mut servo, cfg, params);
        let w = exp.warnings();
        assert!(
            w[0].starts_with("rung 8520: steady segment too short"),
            "{w:?}"
        );
        assert!(
            w[1].starts_with("the -26% rung needs") && w[1].ends_with("the ladder ends below it"),
            "{w:?}"
        );
        assert_eq!(exp.rungs.len(), 1);
        assert!(!exp.rungs[0].used);
        assert_eq!(exp.declined(), Some(&Declined::Thin { rungs: 0 }));
    }

    const R_MG90: f64 = 7270.0 / 4096.0;
    const GUARD: (u16, u16) = (532, 3526);
    const SOFT: (i32, i32) = (432, 3626);

    /// The bench guard, stops and abort: soft 432..3626 inset, a quarter
    /// over the 280-count limit.
    fn bench() -> RigParams {
        RigParams::new(Some(GUARD), 350).with_stops((209, 3849))
    }

    /// Seeks at 15%, the bench plan's breakaway + 2%.
    fn bench_cfg() -> LadderCfg {
        LadderCfg {
            seek_duty_q15: 4915,
            ..LadderCfg::default()
        }
    }

    /// The bench MG90's 2S pilot envelope: its v_ss fits and spindowns.
    fn mg90_2s() -> Envelope {
        let coast = [
            (3.16, 130.0),
            (5.62, 316.0),
            (7.55, 536.0),
            (11.67, 979.0),
            (13.15, 1333.0),
        ];
        Envelope {
            supply: Supply::TwoS,
            phys: (232, 3849),
            fwd: Line {
                slope: 0.2047,
                intercept: -0.721,
            },
            rev: Line {
                slope: 0.2079,
                intercept: -0.831,
            },
            coast: coast.iter().chain(coast.iter()).copied().collect(),
        }
    }

    fn sized_by_envelope(guard: (u16, u16)) -> Runway {
        let mut r = Runway::new(guard);
        r.size_by(mg90_2s(), Some(Supply::TwoS), (209, 3849))
            .unwrap();
        r
    }

    fn q15(pct: f64) -> i16 {
        (pct / 100.0 * Q15) as i16
    }

    /// Every observation the wrapped experiment was stepped with.
    struct Spy<E> {
        exp: E,
        seen: Vec<TelemetrySnapshot>,
    }

    impl<E: Experiment> Experiment for Spy<E> {
        fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
            if let Some(o) = obs {
                self.seen.push(*o);
            }
            self.exp.step(obs)
        }

        fn halted(&self) -> Option<AbortReason> {
            self.exp.halted()
        }
    }

    fn spy(servo: &mut FakeServo, exp: Ladder, params: RigParams) -> Spy<Guarded<Ladder>> {
        let mut s = Spy {
            exp: guarded(exp, params),
            seen: Vec::new(),
        };
        let log = pump(&mut s, servo, 4_000_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        s
    }

    fn brakes(log: &[String]) -> usize {
        log.iter()
            .filter(|l| {
                l.strip_prefix("write goal_duty ")
                    .and_then(|v| v.parse::<i32>().ok())
                    .is_some_and(|v| v.unsigned_abs() == BRAKE_DUTY_Q15 as u32)
            })
            .count()
    }

    /// The bench servo on both rails, sizing itself from its seek and its
    /// own rungs at the bench bus's cadence: every rung that runs is sized
    /// inside the room, runs up to its brake point inside the guard, brakes
    /// there and is used, and the top rungs run far above the duty whose
    /// stall the limit holds. On USB all of them run both ways; at the 2S
    /// speed the 64% rungs need all but a few counts of the room, so only
    /// the rungs to 55% are held to it. A slip zone's windows never reach
    /// the fit, so a
    /// rung that runs across one needs that much more travel: given a
    /// 400-count zone on 2S, the -64% rung does not fit, and every rung that
    /// ran is used.
    #[test]
    fn free_running_rungs_fit_the_runway() {
        let mut servo = bench_mg90(3204);
        servo.pos = 2029.0;
        let slipping = RigParams {
            slip: Some((1250, 1650)),
            ..bench()
        };
        let exp = Ladder::new(bench_cfg(), &slipping, Runway::new(GUARD));
        let mut g = guarded(exp, slipping);
        pump(&mut g, &mut servo, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        assert_eq!(exp.sized().len(), 11);
        let w = exp.warnings();
        assert!(
            w.len() == 1 && w[0].starts_with("the -64% rung needs"),
            "{w:?}"
        );
        assert!(exp.rungs.iter().all(|r| r.used));

        for vbus in [3204u16, 1780] {
            let mut servo = bench_mg90(vbus);
            servo.pos = 2029.0;
            let params = bench();
            let exp = Ladder::new(bench_cfg(), &params, Runway::new(GUARD));
            let mut g = guarded(exp, params);
            let log = pump(&mut g, &mut servo, 4_000_000);
            assert_eq!(g.abort(), None, "{vbus}");
            let exp = g.into_inner();
            assert_eq!(exp.declined(), None, "{vbus}: {:?}", exp.warnings());
            let room = exp.runway().room();
            let sized = exp.sized();
            let want = if vbus == 1780 { 12 } else { 10 };
            assert!(sized.len() >= want, "{vbus}: {:?}", exp.warnings());
            for (duty, n) in sized {
                println!(
                    "{vbus}: {:>4} needs {:4.0} (v {:5.2}, climb {:4.0}, run {:4.0}, stop {:4.0}) \
                     of {room}",
                    pct(*duty),
                    n.total(),
                    n.v,
                    n.climb,
                    n.run,
                    n.stop
                );
                assert!(fits(n, room));
            }
            assert_eq!(
                brakes(&log),
                2 * sized.len(),
                "every seek and every rung brakes"
            );
            let fit = exp.fit(R_MG90).expect("the ladder fits");
            assert_eq!(fit.rungs.iter().filter(|r| r.used).count(), sized.len());
            let stall_safe = duty_for(280.0, R_MG90, vbus as f64);
            assert!(20971.0 / Q15 > 2.0 * stall_safe, "{vbus}: {stall_safe}");
            assert!(!servo.torque);
        }
    }

    /// Envelope-sized on 2S, rungs to 100%: 55% fits, 80% does not, and
    /// nothing at or above 80% is ever driven.
    #[test]
    fn ladder_stops_climbing_at_the_first_rung_that_does_not_fit() {
        let mut servo = bench_mg90(3204);
        servo.pos = 2029.0;
        let params = bench();
        let cfg = LadderCfg {
            rungs_q15: [100.0, 26.0, 80.0, 40.0, 55.0].map(q15).to_vec(),
            ..bench_cfg()
        };
        let mut g = guarded(Ladder::new(cfg, &params, sized_by_envelope(GUARD)), params);
        let log = pump(&mut g, &mut servo, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        let ran: Vec<String> = exp.sized().iter().map(|(d, _)| pct(*d)).collect();
        assert_eq!(ran, ["+26%", "-26%", "+40%", "-40%", "+55%", "-55%"]);
        let last = exp.warnings().last().unwrap();
        assert!(
            last.starts_with("the +80% rung needs")
                && last.ends_with("are free: the ladder ends below it"),
            "{last}"
        );
        let top = log
            .iter()
            .filter_map(|l| l.strip_prefix("write goal_duty ")?.parse::<i32>().ok())
            .map(i32::unsigned_abs)
            .max()
            .unwrap();
        assert_eq!(top, q15(55.0) as u32);
        assert_eq!(exp.declined(), None);
        assert_eq!(exp.fit(R_MG90).unwrap().rungs.len(), 6);
    }

    /// A rung still governed after twice its predicted climb: a shaft that
    /// does not move is blocked and the run ends there, one that moves
    /// carries more load than the limit drives and the ladder declines.
    /// Either way it ends with torque off.
    #[test]
    fn a_climb_that_never_ends_is_blocked_or_declined() {
        let params = bench();
        let mut jammed = bench_mg90(3204);
        jammed.pos = 600.0;
        jammed.jam = Some(610.0);
        let mut g = guarded(
            Ladder::new(bench_cfg(), &params, sized_by_envelope(GUARD)),
            params,
        );
        let log = pump(&mut g, &mut jammed, 4_000_000);
        let abort = g.abort();
        let rest = g.into_inner().rest_pos.unwrap();
        assert!((532..=607).contains(&rest), "the seek rested at {rest}");
        assert_eq!(
            abort,
            Some(AbortReason::Blocked {
                pos: 610,
                moved: 610 - rest
            })
        );
        assert_eq!(log.last().unwrap(), "write torque_enable 0");
        assert!(!jammed.torque);

        // viscous load: the friction current reaches the limit at 2 counts/ms
        let mut heavy = bench_mg90(3204);
        heavy.pos = 2029.0;
        heavy.fv = 0.1;
        let mut g = guarded(
            Ladder::new(bench_cfg(), &params, sized_by_envelope(GUARD)),
            params,
        );
        let log = pump(&mut g, &mut heavy, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        let why = exp.declined().unwrap();
        assert_eq!(why, &Declined::Heavy { duty_q15: 8520 });
        assert_eq!(
            why.to_string(),
            "the +26% rung was still held at the current limit after twice its predicted \
             climb: the load is too heavy for the current limit"
        );
        assert_eq!(exp.sized().len(), 1);
        assert_eq!(brakes(&log), 2, "the rung brakes before the ladder ends");
        assert_eq!(log.last().unwrap(), "write torque_enable 0");
        assert!(!heavy.torque);
    }

    /// A weight on the horn that the limit cannot lift at speed: the +26%
    /// rung climbs against it on the limit all the way to its brake point,
    /// inside its climb budget, so it has no clean window. It is declined
    /// in the shared words and does not count; the ladder goes on, and the
    /// -26% rung, run with the weight, climbs clear of the limit and is
    /// fitted. The next rung up is still governed past its budget: the load
    /// is too heavy for it, and the ladder ends there. The runway expects
    /// the slow climb of a loaded servo.
    #[test]
    fn a_governed_rung_is_declined_and_the_ladder_goes_on() {
        let guard = (532, 1732);
        let params = RigParams::new(Some(guard), 350).with_stops((209, 3849));
        let mut servo = bench_mg90(3204);
        servo.pos = 1132.0;
        servo.fv = 0.05;
        servo.load = 36.0;
        let runway = Runway::new(guard).with_accel(15.0);
        let mut g = guarded(Ladder::new(bench_cfg(), &params, runway), params);
        pump(&mut g, &mut servo, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        let w = exp.warnings();
        assert_eq!(w[0], format!("rung 8520: {GOVERNED}"), "{w:?}");
        let second = &exp.rungs[1];
        assert_eq!(second.duty_q15, -8520);
        assert!(second.used && second.omega < 0.0, "{w:?}");
        assert!(second.windows >= bench_cfg().min_steady);
        assert_eq!(exp.declined(), Some(&Declined::Heavy { duty_q15: 10813 }));
        assert!(!servo.torque);
    }

    /// A jam just ahead of the rung's start, held at the limit: the stall
    /// timer folds the limit to the yield and the ladder takes the fold as
    /// its verdict at once, well inside the climb budget.
    #[test]
    fn a_yield_fold_on_a_rung_ends_the_run_blocked() {
        let params = bench();
        let mut servo = bench_mg90(3204);
        servo.pos = 600.0;
        servo.jam = Some(610.0);
        servo.stall_ms = Some(20.0);
        servo.stall_yield = 168;
        let s = spy(
            &mut servo,
            Ladder::new(bench_cfg(), &params, sized_by_envelope(GUARD)),
            params,
        );
        let abort = s.exp.abort();
        let rest = s.exp.into_inner().rest_pos.unwrap();
        assert_eq!(
            abort,
            Some(AbortReason::Blocked {
                pos: 610,
                moved: 610 - rest
            })
        );
        let last = s.seen.last().unwrap();
        assert_ne!(
            last.limit_flags & LIMIT_YIELD_FOLDED,
            0,
            "ended by the fold"
        );
        let rung = s.seen.iter().filter(|o| o.duty_mean_q15 > 5000).count();
        assert!(rung < 20, "{rung} rung reads");
        assert!(!servo.torque);
    }

    #[test]
    fn thin_ladder_is_declined() {
        // a short guard: two rungs fit
        let guard = (1000, 2200);
        let params = RigParams::new(Some(guard), 350).with_stops((209, 3849));
        let mut servo = bench_mg90(3204);
        servo.pos = 1600.0;
        let mut g = guarded(
            Ladder::new(bench_cfg(), &params, sized_by_envelope(guard)),
            params,
        );
        pump(&mut g, &mut servo, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        assert_eq!(exp.sized().len(), 4, "{:?}", exp.warnings());
        let why = exp.declined().unwrap();
        assert_eq!(why, &Declined::Thin { rungs: 2 });
        assert_eq!(
            why.to_string(),
            "2 of the ladder's rungs ran both ways inside the travel, and the fit needs 3"
        );

        // three rungs, but the top one under twice the bottom one's speed
        let params = bench();
        let mut servo = bench_mg90(3204);
        servo.pos = 2029.0;
        let cfg = LadderCfg {
            rungs_q15: [40.0, 47.0, 55.0].map(q15).to_vec(),
            ..bench_cfg()
        };
        let mut g = guarded(Ladder::new(cfg, &params, Runway::new(GUARD)), params);
        pump(&mut g, &mut servo, 4_000_000);
        assert_eq!(g.abort(), None);
        let exp = g.into_inner();
        assert_eq!(exp.sized().len(), 6);
        let Some(Declined::Narrow { lo, hi }) = exp.declined() else {
            panic!("{:?}", exp.declined());
        };
        assert!(*hi > *lo && *hi < MIN_SPAN * lo, "{lo} {hi}");
        assert!(
            exp.declined()
                .unwrap()
                .to_string()
                .ends_with("the rungs span too little speed to fit")
        );
    }

    /// The abort threshold, a quarter over the limit, is judged on the
    /// ident window mean. A governed climb holds the band's bottom, 0.875 of
    /// the limit; window peaks 15% over that reach past the limit and never
    /// reach the abort. Without a limiter holding the climb the first rung
    /// trips it.
    #[test]
    fn abort_threshold_rides_over_a_governed_climb() {
        let params = bench();
        let mut servo = bench_mg90(3204);
        servo.pos = 2029.0;
        servo.hold_ripple = 0.15;
        let s = spy(
            &mut servo,
            Ladder::new(bench_cfg(), &params, Runway::new(GUARD)),
            params,
        );
        assert_eq!(s.exp.abort(), None);
        let governed: Vec<i16> = s
            .seen
            .iter()
            .filter(|o| o.duty_mean_q15 != 0 && o.limit_flags & 1 != 0)
            .map(|o| o.i_mean_counts.abs())
            .collect();
        let top = governed.iter().copied().max().unwrap();
        assert!(governed.len() > 12, "{} governed windows", governed.len());
        assert!((280..=308).contains(&top), "top {top}");
        assert!(top < params.i_abort);

        let mut bare = bench_mg90(3204);
        bare.pos = 2029.0;
        bare.current_limit = None;
        let s = spy(
            &mut bare,
            Ladder::new(bench_cfg(), &params, Runway::new(GUARD)),
            params,
        );
        assert!(
            matches!(s.exp.abort(), Some(AbortReason::Overcurrent { i_mean }) if i_mean > 350),
            "{:?}",
            s.exp.abort()
        );
    }

    /// A rung that brakes a poll late - a slow read at its brake point -
    /// comes to rest past the inset guard it planned on, inside the soft
    /// limits. That is no abort: the run aborts on the soft limits, where
    /// the firmware brakes anyway, and the rung counts. Aborting on the
    /// guard, the same run would have ended there.
    #[test]
    fn a_late_brake_is_not_an_abort() {
        let run = |params: RigParams| {
            let mut servo = bench_mg90(3204);
            servo.pos = 2029.0;
            let mut s = Spy {
                exp: Guarded::new(
                    Ladder::new(bench_cfg(), &bench(), Runway::new(GUARD)),
                    params,
                ),
                seen: Vec::new(),
            };
            pump_on(
                &mut s,
                &mut servo,
                4_000_000,
                Bus::BENCH.with_slow_reads(11),
            );
            assert!(!servo.torque);
            s
        };
        let s = run(bench().abort_at_soft(SOFT));
        assert_eq!(s.exp.abort(), None);
        let past: Vec<u16> = s
            .seen
            .iter()
            .map(|o| o.pos)
            .filter(|p| !(GUARD.0..=GUARD.1).contains(p))
            .collect();
        assert!(!past.is_empty(), "no brake landed past the guard");
        let soft = (SOFT.0 as u16, SOFT.1 as u16);
        assert!(
            past.iter().all(|p| (soft.0..=soft.1).contains(p)),
            "{past:?}"
        );
        let exp = s.exp.into_inner();
        assert_eq!(exp.declined(), None);
        assert!(exp.rungs.iter().all(|r| r.used), "{:?}", exp.warnings());

        let s = run(bench());
        assert_eq!(s.exp.abort(), Some(AbortReason::PosGuard { pos: past[0] }));
    }

    /// Sized by the pilot envelope at the bench bus's cadence: the envelope
    /// gives the speed, but a braked stop is not its coast - three to five
    /// times shorter - so each rung is sized by the stops the run braked:
    /// the first on the seek's stop scaled up, under the coast, the rest on
    /// the rungs' own, under half of it. Every rung to 55% fits and runs
    /// both ways, the top one at over twice the bottom one's speed: the
    /// ladder is not narrow.
    #[test]
    fn envelope_sized_ladder_is_not_narrow() {
        for (vbus, seed) in [(3204, 1), (3204, 2), (3300, 3), (3300, 4)] {
            let mut servo = bench_mg90(vbus);
            servo.pos = 2029.0;
            let params = bench();
            let exp = Ladder::new(bench_cfg(), &params, sized_by_envelope(GUARD));
            let mut g = guarded(exp, params);
            pump_on(
                &mut g,
                &mut servo,
                4_000_000,
                Bus::BENCH.with_slow_reads(seed),
            );
            assert_eq!(g.abort(), None, "{vbus}");
            let exp = g.into_inner();
            assert_eq!(exp.declined(), None, "{vbus}: {:?}", exp.warnings());
            let env = mg90_2s();
            for (k, (duty, n)) in exp.sized().iter().enumerate() {
                let coast = env.coast(n.v);
                let under = if k == 0 { coast } else { coast / 2.0 };
                assert!(
                    n.stop < under,
                    "{vbus} {duty}: sized on a stop of {:.0}, coast {coast:.0}",
                    n.stop
                );
            }
            for d in &bench_cfg().rungs_q15[..5] {
                for d in [*d, -d] {
                    assert!(
                        exp.rungs.iter().any(|r| r.duty_q15 == d && r.used),
                        "{vbus}: {d} not fitted: {:?}",
                        exp.warnings()
                    );
                }
            }
        }
    }
}
