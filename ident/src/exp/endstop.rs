//! The stop finder for `osc cal`, and the drive-direction check: approach
//! each mechanical stop at one fixed duty, seat against it, back down and
//! read it, then leave it. The polarity follows from which stop a positive
//! duty reached.
//!
//! The approach never rises toward a stop: it is the duty the jam check
//! moved the shaft at plus a margin, capped where its stall draws the
//! current limit, so the stop is met at a speed and a current the limit
//! allows. A shaft still over a [`Watch`] window after [`SEEK_TRAVEL_MIN`]
//! counts of travel is seated. The duty then backs down to the seat duty,
//! half the limit, holds [`EndstopCfg::hold_ms`], and the mean of the last
//! [`EndstopCfg::read_polls`] polls is the stop: friction holds the seat,
//! so the stop reads with the spring's share relaxed, approached from the
//! same side every time. Leaving a stop may raise the duty by
//! [`SEEK_STEP_Q15`] per still window, up to the cap.
//!
//! A rest that is no stop is a sticky spot. With the stops known that is a
//! rest more than [`STOP_TOL`] from both; with them unknown, a rest short
//! of the travel. Clear of the stops ([`seek::clear_of_stops`]) the jam
//! check's rule applies there: the duty rises by [`NUDGE_STEP_Q15`] per
//! still window up to the cap, and the approach resumes at the raised duty.
//! Anywhere else, at the cap, or held by the stall fold, the shaft is
//! blocked and the run ends.
//!
//! Run it with the pos guard off and the stall permit held
//! ([`super::Permitted`]): a stop can sit past the soft limits. The two
//! stops must bracket where the shaft started with a real span between
//! them ([`EndstopResult::refusal`]).

use super::centre::NUDGE_STEP_Q15;
use super::seek::{self, SEEK_STEP_Q15, SEEK_TRAVEL_MIN, STOP_TOL, Watch};
use super::{AbortReason, Cmd, Experiment, LIMIT_YIELD_FOLDED, RigParams};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct EndstopCfg {
    /// The one duty each stop is approached at, q15.
    pub approach_q15: i16,
    /// The duty a seated shaft backs down to before the stop is read, q15.
    pub seat_q15: i16,
    /// The most a sticky spot or a leave raises the duty to, q15.
    pub cap_q15: i16,
    pub poll_ms: u32,
    /// How long the seat duty holds before the stop is read.
    pub hold_ms: u32,
    /// Polls at the end of the hold whose mean is the stop.
    pub read_polls: usize,
    /// Poll budget per leg: 1500 x 20 ms is 30 s of travel.
    pub polls_max: u32,
}

impl Default for EndstopCfg {
    fn default() -> Self {
        Self {
            approach_q15: 3932,
            seat_q15: 1966,
            cap_q15: 3932,
            poll_ms: 20,
            hold_ms: 300,
            read_polls: 8,
            polls_max: 1500,
        }
    }
}

/// The least span two stops can be apart and still be the stops, counts.
pub const SPAN_MIN: i32 = 1500;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EndstopResult {
    /// Where the shaft stood before the first approach.
    pub start: i32,
    pub pos_min_phys: i32,
    pub pos_max_phys: i32,
    /// true = positive duty increased position counts, under the drive
    /// polarity in force while it ran.
    pub drive_polarity: bool,
    /// Larger of the two currents the approach stalled at, counts: the
    /// motor loaded against a stop rather than settling idle.
    pub i_stall_counts: i16,
}

impl EndstopResult {
    /// Why these rests cannot be the stops: they must lie either side of
    /// the start, at least [`SPAN_MIN`] apart.
    pub fn refusal(&self) -> Option<String> {
        let (lo, hi, start) = (self.pos_min_phys, self.pos_max_phys, self.start);
        let span = hi - lo;
        (!(lo < start && start < hi && span >= SPAN_MIN)).then(|| {
            format!(
                "the stops found at {lo} and {hi} ({span} counts apart) do not lie either side \
                 of the start at {start} at least {SPAN_MIN} counts apart: the shaft is blocked \
                 or the position sensor is not reading"
            )
        })
    }
}

/// The low stop first, so the run ends off the high one.
const DIRS: [i8; 2] = [-1, 1];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Leg {
    /// Toward the stop `dir` drives at, travel counted from `from`: the
    /// leg's start, or the sticky spot it last rose at.
    Approach { dir: i8, from: u16 },
    /// Seated, backed down to the seat duty.
    Hold { dir: i8 },
    /// Off the stop read at `seat`, driving `dir`.
    Leave { dir: i8, seat: u16 },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Phase {
    ModeWrite,
    TorqueOn,
    Read,
    Eval,
    Wait,
    FinishDuty,
    FinishTorque,
    Finished,
}

pub struct Endstop {
    cfg: EndstopCfg,
    stops: Option<(u16, u16)>,
    stall_eps: u16,
    stall_polls: u32,
    phase: Phase,
    leg: Option<Leg>,
    /// The approach duty, raised only at a sticky spot.
    approach: i16,
    leave: i16,
    written: Option<i32>,
    watch: Option<Watch>,
    polls: u32,
    start: Option<u16>,
    held: Vec<u16>,
    i_seat: i16,
    halt: Option<AbortReason>,
    /// Stop and stall current per direction: index 0 the positive-duty
    /// rail, index 1 the negative-duty rail.
    rail: [Option<(u16, i16)>; 2],
}

fn rail_of(dir: i8) -> usize {
    if dir > 0 { 0 } else { 1 }
}

impl Endstop {
    pub fn new(cfg: EndstopCfg, params: &RigParams) -> Self {
        Self {
            approach: cfg.approach_q15,
            leave: cfg.approach_q15,
            cfg,
            stops: params.stops,
            stall_eps: params.stall_eps,
            stall_polls: params.stall_polls,
            phase: Phase::ModeWrite,
            leg: None,
            written: None,
            watch: None,
            polls: 0,
            start: None,
            held: Vec::new(),
            i_seat: 0,
            halt: None,
            rail: [None; 2],
        }
    }

    /// The approach duty as the run left it, a fraction of full scale:
    /// over the configured one only when a sticky spot raised it.
    pub fn approach(&self) -> f64 {
        self.approach as f64 / 32767.0
    }

    /// `None` until both stops were read.
    pub fn result(&self) -> Option<EndstopResult> {
        let (p_pos, i_pos) = self.rail[0]?;
        let (p_neg, i_neg) = self.rail[1]?;
        let (p_pos, p_neg) = (p_pos as i32, p_neg as i32);
        Some(EndstopResult {
            start: self.start? as i32,
            pos_min_phys: p_pos.min(p_neg),
            pos_max_phys: p_pos.max(p_neg),
            drive_polarity: p_pos > p_neg,
            i_stall_counts: i_pos.abs().max(i_neg.abs()),
        })
    }

    fn enter(&mut self, leg: Leg) {
        self.leg = Some(leg);
        self.watch = None;
        self.polls = 0;
    }

    /// The goal only when it changes; every drive polls on.
    fn drive(&mut self, value: i32) -> Cmd {
        if self.written == Some(value) {
            return self.wait();
        }
        self.written = Some(value);
        self.phase = Phase::Wait;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value,
        }
    }

    fn wait(&mut self) -> Cmd {
        self.phase = Phase::Read;
        Cmd::Pause {
            ms: self.cfg.poll_ms,
        }
    }

    fn finish(&mut self) -> Cmd {
        self.phase = Phase::FinishDuty;
        Cmd::Pause { ms: 0 }
    }

    fn blocked(&mut self, from: u16, pos: u16) -> Cmd {
        self.halt = Some(seek::blocked(from, pos));
        self.finish()
    }

    /// A whole window without motion, or the stall fold holding the shaft.
    fn still(&mut self, o: &TelemetrySnapshot) -> bool {
        let (eps, polls) = (self.stall_eps, self.stall_polls);
        let watch = self.watch.get_or_insert(Watch::new(o.pos, eps, polls));
        watch.still(o.pos) || o.limit_flags & LIMIT_YIELD_FOLDED != 0
    }

    fn near_a_stop(&self, pos: u16) -> bool {
        self.stops
            .is_some_and(|(lo, hi)| pos.abs_diff(lo) <= STOP_TOL || pos.abs_diff(hi) <= STOP_TOL)
    }

    fn approach_eval(&mut self, o: &TelemetrySnapshot, dir: i8, from: u16) -> Cmd {
        let pos = o.pos;
        if self.polls > self.cfg.polls_max {
            return self.blocked(from, pos);
        }
        if !self.still(o) {
            return self.drive(dir as i32 * self.approach as i32);
        }
        let travelled = pos.abs_diff(from) >= SEEK_TRAVEL_MIN;
        if travelled && (self.stops.is_none() || self.near_a_stop(pos)) {
            self.i_seat = o.i_mean_counts;
            self.held.clear();
            self.enter(Leg::Hold { dir });
            return self.drive(dir as i32 * self.cfg.seat_q15 as i32);
        }
        let next = self.approach.saturating_add(NUDGE_STEP_Q15);
        let folded = o.limit_flags & LIMIT_YIELD_FOLDED != 0;
        if !folded && seek::clear_of_stops(pos, self.stops) && next <= self.cfg.cap_q15 {
            self.approach = next;
            self.enter(Leg::Approach { dir, from: pos });
            return self.drive(dir as i32 * next as i32);
        }
        self.blocked(from, pos)
    }

    fn hold_eval(&mut self, pos: u16, dir: i8) -> Cmd {
        self.held.push(pos);
        if (self.held.len() as u32) * self.cfg.poll_ms < self.cfg.hold_ms {
            return self.wait();
        }
        let tail = &self.held[self.held.len().saturating_sub(self.cfg.read_polls.max(1))..];
        let mean = tail.iter().map(|p| *p as f64).sum::<f64>() / tail.len() as f64;
        let stop = mean.round() as u16;
        self.rail[rail_of(dir)] = Some((stop, self.i_seat));
        self.leave = self.approach;
        self.enter(Leg::Leave {
            dir: -dir,
            seat: stop,
        });
        self.drive(-(dir as i32) * self.leave as i32)
    }

    fn leave_eval(&mut self, o: &TelemetrySnapshot, dir: i8, seat: u16) -> Cmd {
        let pos = o.pos;
        if self.polls > self.cfg.polls_max {
            return self.blocked(seat, pos);
        }
        if pos.abs_diff(seat) > STOP_TOL {
            return match DIRS.into_iter().find(|d| self.rail[rail_of(*d)].is_none()) {
                Some(next) => {
                    self.enter(Leg::Approach {
                        dir: next,
                        from: pos,
                    });
                    self.drive(next as i32 * self.approach as i32)
                }
                None => self.finish(),
            };
        }
        if self.still(o) {
            let next = self.leave.saturating_add(SEEK_STEP_Q15);
            if next > self.cfg.cap_q15 {
                return self.blocked(seat, pos);
            }
            self.leave = next;
        }
        self.drive(dir as i32 * self.leave as i32)
    }

    fn eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        self.polls += 1;
        self.start.get_or_insert(o.pos);
        let leg = *self.leg.get_or_insert(Leg::Approach {
            dir: DIRS[0],
            from: o.pos,
        });
        match leg {
            Leg::Approach { dir, from } => self.approach_eval(o, dir, from),
            Leg::Hold { dir } => self.hold_eval(o.pos, dir),
            Leg::Leave { dir, seat } => self.leave_eval(o, dir, seat),
        }
    }
}

impl Experiment for Endstop {
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
                self.phase = Phase::Read;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::Read => {
                self.phase = Phase::Eval;
                Cmd::Read
            }
            Phase::Eval => match obs {
                Some(o) => self.eval(o),
                None => self.finish(),
            },
            Phase::Wait => self.wait(),
            Phase::FinishDuty => {
                self.phase = Phase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::FinishTorque => {
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
    use super::super::testkit::{FakeServo, bench_mg90, pump, rig};
    use super::super::{Guarded, Permitted};
    use super::*;

    /// A virgin servo: no stops known.
    fn virgin() -> RigParams {
        RigParams {
            stops: None,
            ..rig().without_pos_guard()
        }
    }

    fn run(servo: &mut FakeServo) -> (Endstop, Vec<String>) {
        let params = virgin();
        let mut exp = Guarded::new(Endstop::new(EndstopCfg::default(), &params), params);
        let log = pump(&mut exp, servo, 2_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    #[test]
    fn recovers_rails_and_polarity() {
        let mut servo = FakeServo::new(3.37);
        servo.ends = (321.0, 3702.0);
        servo.pos = 2000.0;
        let (exp, log) = run(&mut servo);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let r = exp.result().expect("both rails found");
        assert!((r.pos_min_phys - 321).abs() <= 3, "min {}", r.pos_min_phys);
        assert!((r.pos_max_phys - 3702).abs() <= 3, "max {}", r.pos_max_phys);
        assert!(r.drive_polarity, "positive duty should raise counts");
        assert!(r.i_stall_counts > 0, "stall current {}", r.i_stall_counts);
        assert_eq!(r.start, 2000);
        assert_eq!(r.refusal(), None);
    }

    /// Stops unknown: an approach that never travels ends the run, and
    /// rests that do not bracket the start with a real span are no stops.
    #[test]
    fn a_blocked_shaft_finds_no_stops() {
        let mut servo = FakeServo::new(3.37);
        servo.pos = 2048.0;
        servo.jam = Some(2048.0);
        let params = virgin();
        let mut exp = Guarded::new(Endstop::new(EndstopCfg::default(), &params), params);
        pump(&mut exp, &mut servo, 2_000_000);
        assert_eq!(
            exp.abort(),
            Some(AbortReason::Blocked {
                pos: 2048,
                moved: 0
            })
        );
        assert_eq!(exp.into_inner().result(), None);

        let r = EndstopResult {
            start: 2048,
            pos_min_phys: 1200,
            pos_max_phys: 2600,
            drive_polarity: true,
            i_stall_counts: 200,
        };
        assert_eq!(
            r.refusal().as_deref(),
            Some(
                "the stops found at 1200 and 2600 (1400 counts apart) do not lie either side of \
                 the start at 2048 at least 1500 counts apart: the shaft is blocked or the \
                 position sensor is not reading"
            )
        );
        let beside = EndstopResult {
            pos_min_phys: 2100,
            pos_max_phys: 3849,
            ..r
        };
        assert!(beside.refusal().is_some(), "the start is outside the stops");
        let good = EndstopResult {
            pos_min_phys: 209,
            pos_max_phys: 3849,
            ..r
        };
        assert_eq!(good.refusal(), None);
    }

    #[test]
    fn polarity_inference_flips_when_wiring_reversed() {
        let mut servo = FakeServo::new(3.37);
        servo.ends = (321.0, 3702.0);
        servo.pos = 2000.0;
        servo.drive_polarity = false;
        let (exp, _) = run(&mut servo);
        let r = exp.result().expect("both rails found");
        assert!(!r.drive_polarity, "reversed wiring must flip inference");
        // rails ordered regardless of which duty sign reached which end
        assert!(r.pos_min_phys < r.pos_max_phys);
        assert!((r.pos_min_phys - 321).abs() <= 3, "min {}", r.pos_min_phys);
        assert!((r.pos_max_phys - 3702).abs() <= 3, "max {}", r.pos_max_phys);
    }

    /// Known stops and reversed wiring: the approach meant for the low stop
    /// seats at the high one, which is still a stop.
    #[test]
    fn known_stops_seat_either_way_round() {
        let mut servo = FakeServo::new(3.37);
        servo.pos = 2000.0;
        servo.drive_polarity = false;
        let params = rig().without_pos_guard();
        let mut exp = Guarded::new(Endstop::new(EndstopCfg::default(), &params), params);
        pump(&mut exp, &mut servo, 2_000_000);
        assert_eq!(exp.abort(), None);
        let r = exp.into_inner().result().expect("both rails found");
        assert_eq!((r.pos_min_phys, r.pos_max_phys), (200, 4000));
        assert!(!r.drive_polarity);
    }

    #[test]
    fn command_choreography_is_safe() {
        let mut servo = FakeServo::new(3.37);
        let (_, log) = run(&mut servo);
        // torque on before any nonzero duty
        let torque_on = log
            .iter()
            .position(|l| l == "write torque_enable 1")
            .expect("torque on");
        let first_duty = log
            .iter()
            .position(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .expect("a drive command");
        assert!(torque_on < first_duty);
        // final safety pair
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
    }

    /// What the experiment saw and did, command by command.
    struct Spy {
        exp: Guarded<Permitted<Endstop>>,
        seen: Vec<(Option<u16>, Cmd)>,
    }

    impl Experiment for Spy {
        fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
            let cmd = self.exp.step(obs);
            self.seen.push((obs.map(|o| o.pos), cmd.clone()));
            cmd
        }
    }

    const Q15: f64 = 32767.0;

    /// The bench MG90 on 2S at its bench duties, stops known: approach at
    /// 15.5%, seat at 7.76%, both at a stall the 280-count limit holds.
    fn bench(servo: &mut FakeServo, cfg: EndstopCfg) -> Spy {
        let params = RigParams::new(None, 350).with_stops((209, 3849));
        let exp = Guarded::new(Permitted::new(Endstop::new(cfg, &params)), params);
        let mut spy = Spy {
            exp,
            seen: Vec::new(),
        };
        pump(&mut spy, servo, 2_000_000);
        spy
    }

    fn bench_cfg() -> EndstopCfg {
        EndstopCfg {
            approach_q15: (0.1551 * Q15) as i16,
            seat_q15: (0.0776 * Q15) as i16,
            cap_q15: (0.1551 * Q15) as i16,
            ..EndstopCfg::default()
        }
    }

    /// Seated at each stop, the duty backs down to the seat duty and holds
    /// there 300 ms; the stop is the mean of the last 8 polls of the hold,
    /// then the shaft leaves at the approach duty. The permit is held
    /// throughout and withdrawn at the end.
    #[test]
    fn cal_seats_at_half_the_limit_after_backing_down() {
        let mut servo = bench_mg90(3204);
        servo.pos = 2029.0;
        servo.pos_noise = 4.0;
        let cfg = bench_cfg();
        let (a, s) = (cfg.approach_q15 as i32, cfg.seat_q15 as i32);
        let spy = bench(&mut servo, cfg);
        assert_eq!(spy.exp.abort(), None);
        let writes: Vec<(usize, i32)> = spy
            .seen
            .iter()
            .enumerate()
            .filter_map(|(k, (_, c))| match c {
                Cmd::Write { reg, value } if *reg == control::GOAL_DUTY => Some((k, *value)),
                _ => None,
            })
            .collect();
        let values: Vec<i32> = writes.iter().map(|w| w.1).collect();
        assert_eq!(values, [-a, -s, a, s, -a, 0]);
        let r = spy.exp.into_inner().into_inner().result().unwrap();
        for (seat, stop) in [(1, r.pos_min_phys), (3, r.pos_max_phys)] {
            let (from, to) = (writes[seat].0, writes[seat + 1].0);
            let hold: u32 = spy.seen[from..to]
                .iter()
                .filter_map(|(_, c)| match c {
                    Cmd::Pause { ms } => Some(*ms),
                    _ => None,
                })
                .sum();
            assert!((300..320).contains(&hold), "held {hold} ms");
            let polls: Vec<f64> = spy.seen[from..=to]
                .iter()
                .filter_map(|(p, _)| p.map(|p| p as f64))
                .collect();
            let last8 = &polls[polls.len() - 8..];
            let mean = (last8.iter().sum::<f64>() / 8.0).round() as i32;
            assert_eq!(stop, mean);
        }
        assert!((r.pos_min_phys - 209).abs() <= 2 && (r.pos_max_phys - 3849).abs() <= 2);
        assert!(r.i_stall_counts > 250 && r.i_stall_counts <= 280);
        assert!(!servo.torque && !servo.permit_live());
    }

    /// A re-cal whose approach comes to rest at a sticky spot at mid travel:
    /// the jam check's rule raises the duty there, under the cap, and the
    /// approach carries on at the raised duty to the stop.
    #[test]
    fn recal_rides_a_sticky_spot_under_the_cap() {
        let mut servo = bench_mg90(1780);
        servo.pos = 2029.0;
        servo.sticky = Some((1200.0, 1400.0, 120.0));
        let cfg = EndstopCfg {
            approach_q15: (0.1913 * Q15) as i16,
            seat_q15: (0.1396 * Q15) as i16,
            cap_q15: (0.2792 * Q15) as i16,
            ..EndstopCfg::default()
        };
        let (a, cap) = (cfg.approach_q15 as i32, cfg.cap_q15 as i32);
        let spy = bench(&mut servo, cfg);
        assert_eq!(spy.exp.abort(), None);
        let exp = spy.exp.into_inner().into_inner();
        let raised = (exp.approach() * Q15).round() as i32;
        assert!(raised > a && raised <= cap, "{a} -> {raised}");
        let r = exp.result().unwrap();
        assert_eq!((r.pos_min_phys, r.pos_max_phys), (209, 3849));
        // toward the low stop through the spot: the base duty, then the
        // raise; never back down
        let drives: Vec<i32> = spy
            .seen
            .iter()
            .filter_map(|(_, c)| match c {
                Cmd::Write { reg, value } if *reg == control::GOAL_DUTY => Some(*value),
                _ => None,
            })
            .collect();
        assert_eq!(drives[0], -a);
        assert!(drives[1] < -a && drives[1] >= -cap, "{drives:?}");
        assert!(!servo.torque && !servo.permit_live());
    }

    /// The same spot at the cap already: the approach cannot rise, and the
    /// shaft is blocked where it rests.
    #[test]
    fn a_sticky_spot_over_the_cap_is_blocked() {
        let mut servo = bench_mg90(3204);
        servo.pos = 2029.0;
        servo.sticky = Some((1200.0, 1400.0, 300.0));
        let spy = bench(&mut servo, bench_cfg());
        assert!(matches!(
            spy.exp.abort(),
            Some(AbortReason::Blocked { pos, .. }) if (1200..=1400).contains(&pos)
        ));
        assert!(!servo.torque && !servo.permit_live());
    }
}
