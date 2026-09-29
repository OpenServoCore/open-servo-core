//! The ripple traverse for `osc cal`: one constant-duty run from the low end
//! of the travel to the high end whose per-tick TEL current carries the
//! commutation ripple for the tachometer and the position table.
//!
//! The run is free-running - its duty is not capped by stall current - so it
//! takes the runway rule ([`Runway`]): it runs only when its climb and its
//! margined stop fit, and it brakes at `guard - 1.25 x stop` with the brake
//! idiom, never a coast. A seek at the stall-safe `seek_duty_q15` first
//! carries the shaft down to the start band and brakes there; its speed and
//! its braked stop are this slower first pass, which sizes the run when no
//! pilot envelope describes the servo.
//!
//! The run is polled through its climb until its speed reads over
//! [`SPEED_SPAN_MS`]. The capture then rides the duty already in force,
//! burst by burst ([`Cmd::Stream`]), each burst's frames seq-contiguous and
//! each sized to [`CAPTURE_SHARE`] of the travel left to the brake point at
//! the faster of the last speed read and the runway's. A burst's own frames
//! give the next one its start and speed, so the capture closes on the
//! brake point in a few chunks; then polling resumes, and the run brakes at
//! the brake point.
//!
//! Run it inside the stops just found ([`guard`]), with the pos guard off
//! and no permit.

use super::seek::{self, STOP_TOL, Watch};
use super::{AbortReason, Cmd, Experiment, LIMIT_YIELD_FOLDED, RigParams};
use crate::frame::{SeqUnwrap, TelFrame, TelemetrySnapshot};
use crate::limits::guards;
use crate::regs::control;
use crate::runway::{
    BRAKE_DUTY_Q15, BRAKE_POLL_MS, BRAKE_POLLS, BRAKE_REST_EPS, Need, Runway, fits,
};

const Q15: f64 = 32767.0;

/// Speeds are read over at least this much of the drive, ms.
pub const SPEED_SPAN_MS: f64 = 10.0;

/// The capture covers this share of the travel left to the brake point at
/// the speed read after the climb: the speed may still rise by a quarter
/// before the capture ends past the brake point.
pub const CAPTURE_SHARE: f64 = 0.8;

/// The traverse's guard inside the stops just found: each end more than
/// [`STOP_TOL`] in, so no rest of the run reads as a stop, and inside the
/// guard of the soft limits in force (`soft`, None on a servo never
/// calibrated), where the firmware's soft-limit clamp never meets it.
pub fn guard(stops: (u16, u16), soft: Option<(u16, u16)>) -> (u16, u16) {
    let lo = stops.0.saturating_add(STOP_TOL);
    let hi = stops.1.saturating_sub(STOP_TOL);
    match soft.map(guards) {
        Some((glo, ghi)) => (lo.max(glo), hi.min(ghi)),
        None => (lo, hi),
    }
}

pub struct SweepCfg {
    /// The traverse duty, q15.
    pub duty_q15: i16,
    /// The seek to the start band, q15: it points at a stop, so it is a
    /// stall-safe duty.
    pub seek_duty_q15: i16,
    /// TEL field selection, written before the arm (mask is sticky).
    pub mask: u16,
    /// Fast ticks a second: TEL samples per second of capture.
    pub tick_hz: f64,
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
    /// The seek, and the run, give up after this many polls.
    pub seek_cap_polls: u32,
    pub run_cap_polls: u32,
    /// Rest between the seek's brake and the run.
    pub rest_ms: u32,
}

impl Default for SweepCfg {
    fn default() -> Self {
        Self {
            duty_q15: 8520,
            seek_duty_q15: 3932,
            // pos|current|duty|vdiff
            mask: 0x1B,
            tick_hz: 20_000.0,
            poll_ms: 2,
            seek_poll_ms: 20,
            seek_cap_polls: 500,
            run_cap_polls: 4000,
            rest_ms: 300,
        }
    }
}

/// What the run captured.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Captured {
    pub bursts: u32,
    pub samples: u32,
    /// Where the shaft was at the first arm, and where the last burst's
    /// frames ended.
    pub from: u16,
    pub to: Option<u16>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum After {
    Run,
    Finish,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Phase {
    ModeWrite,
    TorqueOn,
    MaskOn,
    SeekRead,
    SeekEval,
    SeekWait,
    BrakeWait,
    BrakeRead,
    BrakeEval,
    Rest(After),
    RunSet,
    RunNext,
    RunRead,
    RunEval,
    RunWait,
    FinishDuty,
    FinishTorque,
    MaskOff,
    Finished,
}

struct Brake {
    from: u16,
    v: f64,
    last: u16,
    polls: u32,
    after: After,
}

/// The run in flight.
struct Rung {
    need: Need,
    from: u16,
    t0: Option<f64>,
    climbing: bool,
    watch: Watch,
    watched: u32,
    still: bool,
    /// When the limit let go of the climb.
    t_free: Option<f64>,
    /// The speed the capture was sized at, then the speed its frames ended
    /// at, counts/ms.
    v: f64,
    /// Time between polls before the capture, ms.
    cadence: f64,
    streamed: bool,
    /// Where the last burst's frames ended.
    at: Option<u16>,
    /// Consecutive polls past the climb that moved no more than the
    /// stillness threshold over [`SPEED_SPAN_MS`].
    still_polls: u32,
    polls: u32,
}

pub struct Sweep {
    cfg: SweepCfg,
    params: RigParams,
    runway: Runway,
    phase: Phase,
    clock: SeqUnwrap,
    /// (window time ms, raw pos) of every poll of the drive in progress.
    reads: Vec<(f64, u16)>,
    watch: Option<Watch>,
    polls: u32,
    seeking: bool,
    start: Option<u16>,
    brake: Option<Brake>,
    rung: Option<Rung>,
    need: Option<Need>,
    captured: Option<Captured>,
    rest: Option<u16>,
    declined: Option<String>,
    halt: Option<AbortReason>,
}

impl Sweep {
    pub fn new(cfg: SweepCfg, params: &RigParams, runway: Runway) -> Self {
        Self {
            cfg,
            params: *params,
            runway,
            phase: Phase::ModeWrite,
            clock: SeqUnwrap::default(),
            reads: Vec::new(),
            watch: None,
            polls: 0,
            seeking: false,
            start: None,
            brake: None,
            rung: None,
            need: None,
            captured: None,
            rest: None,
            declined: None,
            halt: None,
        }
    }

    /// Why the run captured nothing, in plain words.
    pub fn declined(&self) -> Option<&str> {
        self.declined.as_deref()
    }

    /// What the run was sized to.
    pub fn need(&self) -> Option<Need> {
        self.need
    }

    pub fn captured(&self) -> Option<Captured> {
        self.captured
    }

    /// Where the run's brake came to rest.
    pub fn rest(&self) -> Option<u16> {
        self.rest
    }

    pub fn runway(&self) -> &Runway {
        &self.runway
    }

    fn duty(&self) -> f64 {
        self.cfg.duty_q15 as f64 / Q15
    }

    fn time(&mut self, o: &TelemetrySnapshot) -> f64 {
        self.clock.push(o.agg_seq) as f64 * self.params.agg_period_ms
    }

    /// Speed over the last [`SPEED_SPAN_MS`] of polls, counts/ms.
    fn speed(&self) -> Option<f64> {
        let &(t1, p1) = self.reads.last()?;
        let &(t0, p0) = self
            .reads
            .iter()
            .rev()
            .find(|r| r.0 <= t1 - SPEED_SPAN_MS)?;
        Some(p1.abs_diff(p0) as f64 / (t1 - t0))
    }

    /// Time between the last two polls, ms.
    fn gap(&self) -> Option<f64> {
        match self.reads.as_slice() {
            [.., a, b] => Some(b.0 - a.0),
            _ => None,
        }
    }

    fn decline(&mut self, why: String) {
        self.declined.get_or_insert(why);
    }

    fn blocked(&mut self, from: u16, pos: u16) -> Cmd {
        self.halt = Some(seek::blocked(from, pos));
        self.phase = Phase::FinishDuty;
        Cmd::Pause { ms: 0 }
    }

    fn brake(&mut self, motion: i8, pos: u16, v: f64, after: After) -> Cmd {
        self.brake = Some(Brake {
            from: pos,
            v,
            last: pos,
            polls: 0,
            after,
        });
        self.phase = Phase::BrakeWait;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: -(motion as i32) * BRAKE_DUTY_Q15 as i32,
        }
    }

    fn seek_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let t = self.time(o);
        let target = self.runway.start(1) as f64;
        if !self.seeking && o.pos as f64 <= target {
            self.rest = Some(o.pos);
            self.phase = Phase::RunSet;
            return Cmd::Pause { ms: 0 };
        }
        self.reads.push((t, o.pos));
        let from = *self.start.get_or_insert(o.pos);
        if o.limit_flags & LIMIT_YIELD_FOLDED != 0 {
            return self.blocked(from, o.pos);
        }
        let v = self.speed().unwrap_or(0.0);
        let stop = self.runway.stop(v).unwrap_or(0.0);
        let lead = v * self.gap().unwrap_or(0.0);
        if self.seeking && o.pos as f64 - lead - stop <= target {
            return self.brake(-1, o.pos, v, After::Run);
        }
        let (eps, polls) = (self.params.stall_eps, self.params.stall_polls);
        if self
            .watch
            .get_or_insert(Watch::new(o.pos, eps, polls))
            .still(o.pos)
        {
            if seek::at_stop(from, o.pos, -1, self.params.stops).is_err() {
                return self.blocked(from, o.pos);
            }
            return self.brake(-1, o.pos, 0.0, After::Run);
        }
        self.polls += 1;
        if self.polls >= self.cfg.seek_cap_polls {
            self.decline("the seek to the start of the traverse did not arrive".into());
            return self.brake(-1, o.pos, v, After::Finish);
        }
        self.phase = Phase::SeekWait;
        if self.seeking {
            return Cmd::Pause {
                ms: self.cfg.seek_poll_ms,
            };
        }
        self.seeking = true;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: -(self.cfg.seek_duty_q15 as i32),
        }
    }

    fn brake_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let Some(mut b) = self.brake.take() else {
            self.phase = Phase::FinishDuty;
            return Cmd::Pause { ms: 0 };
        };
        b.polls += 1;
        if o.pos.abs_diff(b.last) >= BRAKE_REST_EPS && b.polls < BRAKE_POLLS {
            b.last = o.pos;
            self.brake = Some(b);
            self.phase = Phase::BrakeRead;
            return Cmd::Pause { ms: BRAKE_POLL_MS };
        }
        if b.after == After::Run {
            self.runway.ran(self.cfg.seek_duty_q15 as f64 / Q15, b.v);
        }
        self.runway.stopped(b.v, o.pos.abs_diff(b.from) as f64);
        self.rest = Some(o.pos);
        self.phase = Phase::Rest(b.after);
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: 0,
        }
    }

    /// Size the run; None declines it.
    fn size(&mut self) -> Option<Need> {
        let room = self.runway.room();
        let pct = self.duty() * 100.0;
        let why = match self.runway.plan(1, self.duty(), 0.0) {
            Some(n) if fits(&n, room) => return Some(n),
            Some(n) => format!(
                "the traverse at {pct:.0}% needs {:.0} counts to climb and stop and {room:.0} are \
                 free",
                n.total()
            ),
            None => format!("nothing measured sizes the traverse at {pct:.0}%"),
        };
        self.decline(why);
        None
    }

    fn run_set(&mut self) -> Cmd {
        let Some(need) = self.size() else {
            self.phase = Phase::FinishDuty;
            return Cmd::Pause { ms: 0 };
        };
        self.need = Some(need);
        let from = self.rest.unwrap_or(0);
        self.reads.clear();
        self.rung = Some(Rung {
            need,
            from,
            t0: None,
            climbing: true,
            watch: Watch::new(from, self.params.stall_eps, self.params.stall_polls),
            watched: 0,
            still: false,
            t_free: None,
            v: need.v,
            cadence: 0.0,
            streamed: false,
            at: None,
            still_polls: 0,
            polls: 0,
        });
        self.phase = Phase::RunWait;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: self.cfg.duty_q15 as i32,
        }
    }

    fn run_eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let t = self.time(o);
        self.reads.push((t, o.pos));
        let v_read = self.speed();
        let gap = self.gap();
        let (eps, stall_polls) = (self.params.stall_eps, self.params.stall_polls.max(1));
        let goal = self.cfg.duty_q15;
        let Some(r) = self.rung.as_mut() else {
            self.phase = Phase::FinishDuty;
            return Cmd::Pause { ms: 0 };
        };
        r.polls += 1;
        let governed = o.duty_mean_q15 != goal;
        let t0 = *r.t0.get_or_insert(t);
        if !r.streamed
            && let Some(g) = gap
        {
            r.cadence = g;
        }
        let was_climbing = r.climbing;
        if was_climbing {
            r.watched += 1;
            let still = r.watch.still(o.pos);
            if r.watched.is_multiple_of(stall_polls) {
                r.still = still;
            }
            r.climbing = governed;
        } else if v_read.is_some_and(|v| v * SPEED_SPAN_MS <= eps as f64) {
            r.still_polls += 1;
        } else {
            r.still_polls = 0;
        }
        let (need, from, polls) = (r.need, r.from, r.polls);
        let (still, stopped) = (r.still, r.still_polls >= stall_polls);
        if o.limit_flags & LIMIT_YIELD_FOLDED != 0 {
            return self.blocked(from, o.pos);
        }
        if was_climbing && !governed {
            let (d, dt) = (o.pos.abs_diff(from) as f64, t - t0);
            if dt >= SPEED_SPAN_MS && d > 0.0 {
                self.runway.climbed(2.0 * d / (dt * dt) * 1000.0);
            }
            if let Some(r) = self.rung.as_mut() {
                r.t_free = Some(t);
            }
        } else if was_climbing && t - t0 > 2.0 * need.climb_ms {
            if still {
                return self.blocked(from, o.pos);
            }
            self.decline(format!(
                "the traverse at {:.0}% was still held at the current limit after twice its \
                 predicted climb: the load is too heavy for the current limit",
                self.duty() * 100.0
            ));
            return self.brake(1, o.pos, v_read.unwrap_or(0.0), After::Finish);
        } else if stopped {
            return self.blocked(from, o.pos);
        }
        let Some(r) = self.rung.as_mut() else {
            return self.blocked(from, o.pos);
        };
        let v_now = v_read.unwrap_or(r.v).max(need.v);
        let stop = self.runway.stop(v_now).unwrap_or(need.stop);
        let end = self.runway.brake_at(1, stop);
        let lead = v_now * r.cadence;
        if o.pos as f64 + lead >= end {
            return self.brake(1, o.pos, v_now, After::Finish);
        }
        if polls >= self.cfg.run_cap_polls {
            self.decline("the traverse did not reach its brake point".into());
            return self.brake(1, o.pos, v_now, After::Finish);
        }
        let settled = r.t_free.is_some_and(|f| t - f >= SPEED_SPAN_MS);
        if settled && !r.streamed {
            r.streamed = true;
            if let Some(cmd) = self.arm(o.pos, v_now) {
                return cmd;
            }
        }
        self.phase = Phase::RunWait;
        Cmd::Pause {
            ms: self.cfg.poll_ms,
        }
    }

    /// The next burst from `pos` at `v`, counts/ms; None once one would
    /// be shorter than [`SPEED_SPAN_MS`].
    fn arm(&mut self, pos: u16, v: f64) -> Option<Cmd> {
        let r = self.rung.as_ref()?;
        let v = v.max(r.need.v);
        let end = self
            .runway
            .brake_at(1, self.runway.stop(v).unwrap_or(r.need.stop));
        let travel = (end - pos as f64 - v * r.cadence) * CAPTURE_SHARE;
        let ms = travel / v;
        if !ms.is_finite() || ms < SPEED_SPAN_MS {
            return None;
        }
        let samples = (ms * self.cfg.tick_hz / 1000.0).min(u16::MAX as f64) as u16;
        let c = self.captured.get_or_insert(Captured {
            bursts: 0,
            samples: 0,
            from: pos,
            to: None,
        });
        c.bursts += 1;
        c.samples += samples as u32;
        self.reads.clear();
        self.phase = Phase::RunNext;
        Some(Cmd::Stream {
            samples,
            goal: None,
        })
    }

    /// After a burst: the next one from where its frames ended while they
    /// show the shaft moving, else poll.
    fn run_next(&mut self) -> Cmd {
        let eps = self.params.stall_eps as f64;
        let next = self.rung.as_mut().and_then(|r| Some((r.at.take()?, r.v)));
        if let Some((pos, v)) = next
            && v * SPEED_SPAN_MS > eps
            && let Some(cmd) = self.arm(pos, v)
        {
            return cmd;
        }
        self.phase = Phase::RunEval;
        Cmd::Read
    }
}

impl Experiment for Sweep {
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
                self.phase = Phase::MaskOn;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::MaskOn => {
                self.phase = Phase::SeekRead;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: self.cfg.mask as i32,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => match obs {
                Some(o) => self.seek_eval(o),
                None => {
                    self.phase = Phase::FinishDuty;
                    Cmd::Pause { ms: 0 }
                }
            },
            Phase::SeekWait => {
                self.phase = Phase::SeekRead;
                Cmd::Pause {
                    ms: self.cfg.seek_poll_ms,
                }
            }
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
                    After::Run => Phase::RunSet,
                    After::Finish => Phase::FinishTorque,
                };
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::RunSet => self.run_set(),
            Phase::RunNext => self.run_next(),
            Phase::RunRead => {
                self.phase = Phase::RunEval;
                Cmd::Read
            }
            Phase::RunEval => match obs {
                Some(o) => self.run_eval(o),
                None => {
                    self.phase = Phase::RunRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            },
            Phase::RunWait => {
                self.phase = Phase::RunRead;
                Cmd::Pause {
                    ms: self.cfg.poll_ms,
                }
            }
            Phase::FinishDuty => {
                self.phase = Phase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::FinishTorque => {
                self.phase = Phase::MaskOff;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::MaskOff => {
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }

    /// The capture's own frames give the speed it ended at: the polls
    /// after it have none to read over yet.
    fn push_tel(&mut self, frames: &[TelFrame]) {
        let pos: Vec<(u64, u16)> = frames
            .iter()
            .filter_map(|f| Some((f.tick, f.pos?)))
            .collect();
        let span = (SPEED_SPAN_MS * self.cfg.tick_hz / 1000.0) as u64;
        let (Some(&(t1, p1)), Some(r)) = (pos.last(), self.rung.as_mut()) else {
            return;
        };
        if let Some(c) = self.captured.as_mut() {
            c.to = Some(p1);
        }
        if let Some(&(t0, p0)) = pos.iter().rev().find(|p| p.0 + span <= t1) {
            r.v = p1.abs_diff(p0) as f64 * self.cfg.tick_hz / 1000.0 / (t1 - t0) as f64;
            r.at = Some(p1);
        }
    }

    fn halted(&self) -> Option<AbortReason> {
        self.halt
    }
}

#[cfg(test)]
mod tests {
    use super::super::Guarded;
    use super::super::testkit::{FakeServo, bench_mg90, pump};
    use super::*;
    use crate::runway::{Envelope, Line, STOP_MARGIN, Supply};

    const STOPS: (u16, u16) = (209, 3849);
    const SOFT: (u16, u16) = (432, 3626);

    /// The bench MG90 on 2S as the pilot measured it.
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
            coast: coast.to_vec(),
        }
    }

    /// Where a traverse stands: the found stops, the permit off, the
    /// current abort a quarter over the 280-count limit.
    fn params() -> RigParams {
        RigParams::new(None, 350).with_stops(STOPS)
    }

    fn traverse(servo: &mut FakeServo, runway: Runway, seek: f64) -> (Sweep, Vec<String>) {
        let cfg = SweepCfg {
            seek_duty_q15: (seek * Q15) as i16,
            ..SweepCfg::default()
        };
        let mut exp = Guarded::new(Sweep::new(cfg, &params(), runway), params());
        let log = pump(&mut exp, servo, 2_000_000);
        assert_eq!(exp.abort(), None, "{log:?}");
        (exp.into_inner(), log)
    }

    #[test]
    fn the_guard_sits_inside_the_stops_and_the_soft_guard() {
        assert_eq!(guard(STOPS, None), (359, 3699));
        assert_eq!(guard(STOPS, Some(SOFT)), (532, 3526));
        assert_eq!(guard((600, 3000), Some(SOFT)), (750, 2850));
    }

    /// The bench servo on both rails, from rest by the high stop where the
    /// stop finder leaves it: the seek at the approach duty brakes in the
    /// start band, the 26% run is captured burst by burst, and the run
    /// brakes inside the guard by the runway rule - sized by the pilot
    /// envelope on 2S, by the slower first pass without one. The shaft is
    /// never driven into a stop and ends torque off.
    #[test]
    fn cal_traverse_brakes_inside_the_stops() {
        for (vbus, seek, envelope) in [
            (3204, 0.1551, true),
            (3204, 0.1551, false),
            (1780, 0.1913, false),
        ] {
            let mut servo = bench_mg90(vbus);
            servo.pos = 3690.0;
            let g = guard(STOPS, Some(SOFT));
            let mut runway = Runway::new(g);
            if envelope {
                runway
                    .size_by(mg90_2s(), Some(Supply::TwoS), (209, 3849))
                    .unwrap();
            }
            let (exp, log) = traverse(&mut servo, runway, seek);
            let what = format!("{vbus} envelope {envelope}");
            assert_eq!(exp.declined(), None, "{what}");
            let c = exp.captured().expect("a capture");
            let to = c.to.expect("a poll after the capture");
            assert!(
                c.from < g.0 + 400 && to > g.1 - 900 && to < g.1,
                "{what}: captured {c:?}"
            );
            let streams: Vec<u32> = log
                .iter()
                .filter_map(|l| l.strip_prefix("stream ")?.parse().ok())
                .collect();
            assert_eq!(streams.len() as u32, c.bursts, "{what}");
            assert_eq!(streams.iter().sum::<u32>(), c.samples, "{what}");
            // the run brakes, it never coasts: the brake write follows the
            // capture, before any zero duty
            let at = log.iter().rposition(|l| l.starts_with("stream ")).unwrap();
            let brake = format!("write goal_duty {}", -(BRAKE_DUTY_Q15 as i32));
            assert_eq!(
                log[at + 1..]
                    .iter()
                    .find(|l| l.starts_with("write goal_duty")),
                Some(&brake),
                "{what}"
            );
            let rest = exp.rest().unwrap();
            let need = exp.need().unwrap();
            assert!(
                rest <= g.1,
                "{what}: rests at {rest} past the guard {}",
                g.1
            );
            assert!(
                (g.1 as f64 - rest as f64) < STOP_MARGIN * need.stop + 300.0,
                "{what}: braked early at {rest}"
            );
            assert_eq!(servo.pressed_ms, 0.0, "{what}: driven into a stop");
            assert!(!servo.torque, "{what}");
            assert_eq!(
                &log[log.len() - 3..],
                [
                    "write goal_duty 0",
                    "write torque_enable 0",
                    "write tel_mask 0"
                ],
                "{what}"
            );
        }
    }

    /// Nothing measured and no envelope: the run cannot be sized and does
    /// not run.
    #[test]
    fn an_unsized_traverse_does_not_run() {
        let mut servo = bench_mg90(3204);
        let g = guard(STOPS, Some(SOFT));
        servo.pos = (g.0 + 20) as f64;
        let (exp, log) = traverse(&mut servo, Runway::new(g), 0.1551);
        assert!(
            exp.declined()
                .is_some_and(|w| w.starts_with("nothing measured sizes the traverse")),
            "{:?}",
            exp.declined()
        );
        assert!(!log.iter().any(|l| l.starts_with("stream")));
        assert!(!log.iter().any(|l| l == "write goal_duty 8520"));
        assert!(!servo.torque);
    }

    /// A shaft that locks mid run, the capture under way, is blocked where
    /// it stopped: the run ends there with torque off.
    #[test]
    fn a_jam_mid_run_is_blocked() {
        let mut servo = bench_mg90(3204);
        servo.pos = 600.0;
        servo.jam = Some(2500.0);
        let mut runway = Runway::new(guard(STOPS, Some(SOFT)));
        runway
            .size_by(mg90_2s(), Some(Supply::TwoS), (209, 3849))
            .unwrap();
        let cfg = SweepCfg {
            seek_duty_q15: (0.1551 * Q15) as i16,
            ..SweepCfg::default()
        };
        let mut exp = Guarded::new(Sweep::new(cfg, &params(), runway), params());
        let log = pump(&mut exp, &mut servo, 2_000_000);
        assert!(
            matches!(exp.abort(), Some(AbortReason::Blocked { pos: 2500, .. })),
            "{:?}",
            exp.abort()
        );
        assert!(log.iter().any(|l| l.starts_with("stream")));
        assert!(!servo.torque);
    }

    /// A shaft locked on the way to the start band is blocked, not a
    /// start: the run ends there with torque off.
    #[test]
    fn a_jam_on_the_way_to_the_start_is_blocked() {
        let mut servo = bench_mg90(3204);
        servo.pos = 3690.0;
        servo.jam = Some(2000.0);
        let runway = Runway::new(guard(STOPS, Some(SOFT)));
        let cfg = SweepCfg {
            seek_duty_q15: (0.1551 * Q15) as i16,
            ..SweepCfg::default()
        };
        let mut exp = Guarded::new(Sweep::new(cfg, &params(), runway), params());
        let log = pump(&mut exp, &mut servo, 2_000_000);
        assert!(matches!(
            exp.abort(),
            Some(AbortReason::Blocked { pos: 2000, .. })
        ));
        assert!(!log.iter().any(|l| l.starts_with("stream")));
        assert!(!servo.torque);
    }
}
