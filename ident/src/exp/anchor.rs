//! The thermal hold: seek the low stop at the plan's seek duty, seat, hold
//! at the stall-safe stop duty (the current limit governs it) for
//! [`AnchorCfg::hold_ms`], then cool down seated for
//! [`AnchorCfg::cool_ms`]: duty 0, torque on, a measurement pulse at the
//! hold's duty every [`AnchorCfg::cool_every_ms`]. It polls at a bounded
//! rate throughout and keeps every poll and every aggregate window. Duty x
//! vdiff / i per window is the winding's own R, whose ratio to the R once
//! the hold's current settles reads the copper's rise; the pulses' decay
//! is the copper's own time constant, and
//! [`crate::thermal::fit_thermal`] regresses the hold's rise over the NTC
//! on the kernel's heat replayed at it for the winding estimate's rise
//! constant. Runs with the pos guard off and the stall permit held, like
//! the resistance stop ladder; a seek that rests anywhere but the stop
//! ends the run ([`super::seek::at_stop`]).

use super::{AbortReason, Cmd, Experiment, RigParams, WindowSample, WindowStream, seek};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;
use crate::thermal::{self, HoldPoll, RiseFit};
use crate::window::Watch;

#[derive(Copy, Clone, Debug)]
pub struct AnchorCfg {
    /// Seek toward the low stop, q15, applied negative.
    pub seek_duty_q15: i16,
    /// The hold at the stop, q15: the stall-safe cap, so the limit governs.
    pub hold_duty_q15: i16,
    pub hold_ms: u32,
    /// The cool-down after the hold, seated; 0 runs none.
    pub cool_ms: u32,
    /// A measurement pulse at the hold's duty starts this often into the
    /// cool-down, each ending inside it.
    pub cool_every_ms: u32,
    /// Each pulse's length: long enough for the current to settle and
    /// windows to land, short enough that its heat is a few percent of the
    /// hold's.
    pub cool_pulse_ms: u32,
    /// The hold's poll period: the fit bins by the second and the heat
    /// replay holds the last poll, so a fast poll buys nothing.
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
}

/// The shortest hold `osc ident thermal` takes, s: the rise fit needs
/// [`thermal::MIN_BINS`] seconds of it and the heat several of its own
/// time constants.
pub const HOLD_MIN_S: u32 = 60;

impl AnchorCfg {
    /// The cool-down's pulses: one every `cool_every_ms` from its start,
    /// each ending inside it.
    pub fn cool_pulses(&self) -> u32 {
        self.cool_ms.saturating_sub(self.cool_pulse_ms) / self.cool_every_ms.max(1)
    }
}

impl Default for AnchorCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 3932,
            hold_duty_q15: 5242,
            hold_ms: 480_000,
            cool_ms: 480_000,
            cool_every_ms: 10_000,
            cool_pulse_ms: 200,
            poll_ms: 20,
            seek_poll_ms: 30,
        }
    }
}

enum Phase {
    ModeWrite,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    HoldSet,
    HoldRead,
    HoldEval,
    CoolSet,
    CoolRead,
    CoolEval,
    FinishDuty,
    FinishTorque,
    Finished,
}

pub struct Anchor {
    cfg: AnchorCfg,
    stops: Option<(u16, u16)>,
    stall_eps: u16,
    stall_polls: u32,
    phase: Phase,
    halt: Option<AbortReason>,
    seek_start: Option<u16>,
    last_pos: Option<u16>,
    still: u32,
    polls_left: u32,
    windows: WindowStream,
    samples: Vec<WindowSample>,
    polls: Vec<HoldPoll>,
    t0_ms: Option<f64>,
    cool_ms: Option<f64>,
    pulse: bool,
    seat: Option<u16>,
}

/// Toward the low stop.
const DIR: i8 = -1;

fn poll(o: &TelemetrySnapshot) -> HoldPoll {
    HoldPoll {
        t_ms: o.host_ms,
        i_counts: o.i_tick_counts as f64,
        t_ntc_cc: o.t_ntc_cc as f64,
        t_winding_cc: o.t_winding_cc as f64,
    }
}

impl Anchor {
    pub fn new(cfg: AnchorCfg, params: &RigParams) -> Self {
        Self {
            cfg,
            stops: params.stops,
            stall_eps: params.stall_eps,
            stall_polls: params.stall_polls,
            phase: Phase::ModeWrite,
            halt: None,
            seek_start: None,
            last_pos: None,
            still: 0,
            polls_left: 0,
            windows: WindowStream::new(params),
            samples: Vec::new(),
            polls: Vec::new(),
            t0_ms: None,
            cool_ms: None,
            pulse: false,
            seat: None,
        }
    }

    /// Every aggregate window of the hold.
    pub fn samples(&self) -> &[WindowSample] {
        &self.samples
    }

    /// Every poll, the seek's and the hold's.
    pub fn polls(&self) -> &[HoldPoll] {
        &self.polls
    }

    /// Where the shaft seated, once it did.
    pub fn seat(&self) -> Option<u16> {
        self.seat
    }

    /// The hold's start: the first poll after its duty write, which the
    /// kernel applies on its next tick.
    pub fn t0_ms(&self) -> Option<f64> {
        self.t0_ms
    }

    /// The cool-down's start: the first poll after its duty 0 write.
    pub fn cool_ms(&self) -> Option<f64> {
        self.cool_ms
    }

    /// The copper's time constant from the cool-down and the rise over the
    /// hold at it ([`thermal::fit_thermal`]), on the servo's terminal and
    /// current scales.
    pub fn fit(&self, v_per_count: f64, a_per_count: f64) -> Result<RiseFit, String> {
        let t0 = self.t0_ms.ok_or("the hold never started")?;
        let cool = self.cool_ms.ok_or("the hold ran no cool-down")?;
        thermal::fit_thermal(
            &self.polls,
            &self.samples,
            t0,
            cool,
            v_per_count,
            a_per_count,
        )
    }

    /// The rise over the hold alone at a given `thermal_tau_s`
    /// ([`thermal::fit_rise`]).
    pub fn fit_at(
        &self,
        thermal_tau_s: u16,
        v_per_count: f64,
        a_per_count: f64,
    ) -> Result<RiseFit, String> {
        let t0 = self.t0_ms.ok_or("the hold never started")?;
        thermal::fit_rise(
            &self.polls,
            &self.samples,
            t0,
            self.cool_ms.unwrap_or(f64::INFINITY),
            thermal_tau_s,
            v_per_count,
            a_per_count,
        )
    }

    /// Whether `e` ms into the cool-down falls inside a pulse.
    fn pulse_at(&self, e: f64) -> bool {
        let (every, pulse) = (
            self.cfg.cool_every_ms.max(1) as f64,
            self.cfg.cool_pulse_ms as f64,
        );
        let k = (e / every).floor();
        k >= 1.0 && k <= self.cfg.cool_pulses() as f64 && e - k * every < pulse
    }
}

impl Experiment for Anchor {
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
                self.phase = Phase::SeekRead;
                self.windows.mark_transition();
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: DIR as i32 * self.cfg.seek_duty_q15 as i32,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => {
                if let Some(o) = obs {
                    self.polls.push(poll(o));
                    if let Some(last) = self.last_pos
                        && o.pos.abs_diff(last) <= self.stall_eps
                    {
                        self.still += 1;
                    } else {
                        self.still = 0;
                    }
                    self.last_pos = Some(o.pos);
                    let start = *self.seek_start.get_or_insert(o.pos);
                    if self.still >= self.stall_polls {
                        if let Err(reason) = seek::at_stop(start, o.pos, DIR, self.stops) {
                            self.halt = Some(reason);
                            self.phase = Phase::FinishDuty;
                            return Cmd::Pause { ms: 0 };
                        }
                        self.seat = Some(o.pos);
                    }
                }
                if self.seat.is_some() {
                    self.phase = Phase::HoldSet;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            Phase::HoldSet => {
                self.phase = Phase::HoldRead;
                // a backstop: every poll pauses poll_ms, so the clock ends
                // the hold first
                self.polls_left = self.cfg.hold_ms / self.cfg.poll_ms.max(1) + 1;
                // no goal mark: the limiter governs the hold on purpose,
                // and R per window holds whatever duty it applied
                self.windows.mark_transition();
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: DIR as i32 * self.cfg.hold_duty_q15 as i32,
                }
            }
            Phase::HoldRead => {
                self.phase = Phase::HoldEval;
                Cmd::Read
            }
            Phase::HoldEval => {
                let mut over = false;
                if let Some(o) = obs {
                    self.polls.push(poll(o));
                    self.samples
                        .extend(self.windows.drain().into_iter().map(|(w, _)| w));
                    let t0 = *self.t0_ms.get_or_insert(o.host_ms);
                    over = o.host_ms - t0 >= self.cfg.hold_ms as f64;
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if over || self.polls_left == 0 {
                    self.phase = if self.cfg.cool_ms > 0 {
                        Phase::CoolSet
                    } else {
                        Phase::FinishDuty
                    };
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::HoldRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            Phase::CoolSet => {
                self.phase = Phase::CoolRead;
                // a backstop as the hold's, each pulse's two writes polling
                // without a pause
                self.polls_left =
                    self.cfg.cool_ms / self.cfg.poll_ms.max(1) + 1 + 2 * self.cfg.cool_pulses();
                self.pulse = false;
                self.windows.mark_transition();
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::CoolRead => {
                self.phase = Phase::CoolEval;
                Cmd::Read
            }
            Phase::CoolEval => {
                let (mut over, mut pulse) = (false, self.pulse);
                if let Some(o) = obs {
                    self.polls.push(poll(o));
                    self.samples
                        .extend(self.windows.drain().into_iter().map(|(w, _)| w));
                    let e = o.host_ms - *self.cool_ms.get_or_insert(o.host_ms);
                    over = e >= self.cfg.cool_ms as f64;
                    pulse = self.pulse_at(e);
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                self.phase = Phase::CoolRead;
                if over || self.polls_left == 0 {
                    self.phase = Phase::FinishDuty;
                    Cmd::Pause { ms: 0 }
                } else if pulse != self.pulse {
                    self.pulse = pulse;
                    self.windows.mark_transition();
                    Cmd::Write {
                        reg: control::GOAL_DUTY,
                        value: if pulse {
                            DIR as i32 * self.cfg.hold_duty_q15 as i32
                        } else {
                            0
                        },
                    }
                } else {
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
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
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }

    fn push_watch(&mut self, w: &Watch) {
        self.windows.watch(w);
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

    fn run(servo: &mut FakeServo, cfg: AnchorCfg) -> (Anchor, Vec<String>, Option<AbortReason>) {
        let params = rig().without_pos_guard();
        let mut exp = Guarded::new(Permitted::new(Anchor::new(cfg, &params)), params);
        let log = pump(&mut exp, servo, 2_000_000);
        let abort = exp.abort();
        (exp.into_inner().into_inner(), log, abort)
    }

    /// The bench fake breaks away at 13%: a seek over it, as the plan's
    /// seek is in a run.
    fn bench_cfg(hold_ms: u32) -> AnchorCfg {
        AnchorCfg {
            seek_duty_q15: 4915,
            hold_ms,
            cool_ms: 0,
            ..AnchorCfg::default()
        }
    }

    fn short() -> AnchorCfg {
        AnchorCfg {
            hold_ms: 3000,
            cool_ms: 3000,
            cool_every_ms: 1000,
            ..AnchorCfg::default()
        }
    }

    /// Seated at the low stop, the hold runs its time at the bounded poll
    /// rate and keeps the planted R in every window.
    #[test]
    fn holds_its_time_at_the_low_stop() {
        let mut servo = FakeServo::new(3.37);
        let (exp, log, abort) = run(&mut servo, short());
        assert_eq!(abort, None);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let seat = exp.seat().expect("seated");
        assert!(seat <= 200 + seek::STOP_TOL, "seat {seat}");
        let t0 = exp.t0_ms().expect("a hold");
        let cool = exp.cool_ms().expect("a cool-down");
        assert!(
            (3000.0..3100.0).contains(&(cool - t0)),
            "held {}",
            cool - t0
        );
        let hold_polls = exp
            .polls()
            .iter()
            .filter(|p| p.t_ms >= t0 && p.t_ms < cool)
            .count();
        assert!(hold_polls <= 3000 / 20 + 1, "{hold_polls} polls");
        let held: Vec<&WindowSample> = exp.samples().iter().filter(|w| w.t_ms < cool).collect();
        assert!(held.len() > 50, "{} windows", held.len());
        for w in held {
            let r = (w.duty_q15 * w.vdiff / 32767.0) / (w.i * w.duty_q15.signum());
            assert!((r - 3.37).abs() < 0.05, "r {r}");
        }
        // the polls before t0 are the seek's
        assert!(exp.polls().iter().any(|p| p.t_ms < t0));
    }

    /// At the bench limit the hold draws the limit, not the cap's stall.
    #[test]
    fn the_limit_governs_the_hold() {
        let mut servo = bench_mg90(3204);
        servo.r = 3.37;
        // 30%: a 3.37 winding stalls 285 counts on 3204, over the 280 limit
        let cfg = AnchorCfg {
            hold_duty_q15: 9830,
            ..bench_cfg(3000)
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let t0 = exp.t0_ms().unwrap();
        let late: Vec<f64> = exp
            .polls()
            .iter()
            .filter(|p| p.t_ms >= t0 + 1000.0)
            .map(|p| p.i_counts.abs())
            .collect();
        assert!(!late.is_empty());
        assert!(late.iter().all(|i| *i <= 281.0), "{late:?}");
    }

    /// A winding that does not warm over an NTC that does not move reads
    /// no rise against the heat the hold carried.
    #[test]
    fn a_winding_that_does_not_warm_reads_no_rise() {
        let mut servo = FakeServo::new(3.37);
        let cfg = AnchorCfg {
            hold_ms: 40_000,
            cool_ms: 60_000,
            ..AnchorCfg::default()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let f = exp.fit_at(4, 4.0e-3, 0.9e-3).expect("a fit");
        assert!(f.bins >= thermal::MIN_BINS, "{f:?}");
        assert!(f.bins <= 41, "the cool-down is not the hold: {f:?}");
        assert!(f.k.abs() < 1e-3, "{f:?}");
        assert!(f.settled_rise_c.abs() < 0.5, "{f:?}");
        let why = exp.fit(4.0e-3, 0.9e-3).unwrap_err();
        assert!(why.contains("never read over"), "{why}");
    }

    /// Seated through the cool-down: duty 0 with torque on between pulses
    /// at the hold's duty, the permit kept, polls at the hold's rate, the
    /// pulses' polls carrying current and the gaps' none.
    #[test]
    fn the_cool_down_pulses_seated_at_the_hold_duty() {
        let mut servo = FakeServo::new(3.37);
        let cfg = AnchorCfg {
            hold_ms: 3000,
            cool_ms: 30_000,
            cool_every_ms: 5000,
            ..AnchorCfg::default()
        };
        let hold = -(cfg.hold_duty_q15 as i32);
        let (exp, log, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let duties: Vec<i32> = log
            .iter()
            .filter_map(|l| l.strip_prefix("write goal_duty "))
            .map(|v| v.parse().unwrap())
            .collect();
        // the seek, the hold, then 0 and five pulses, and the finish
        let mut want = vec![hold, 0];
        for _ in 0..5 {
            want.extend([hold, 0]);
        }
        want.push(0);
        assert_eq!(duties[1..], want[..], "{duties:?}");
        let torque_off = log.iter().position(|l| l == "write torque_enable 0");
        let permit_off = log.iter().position(|l| l == "write stall_permit 0");
        assert_eq!(torque_off, Some(log.len() - 2), "{log:?}");
        assert_eq!(permit_off, Some(log.len() - 1), "{log:?}");
        let cool = exp.cool_ms().expect("a cool-down");
        let polls: Vec<&HoldPoll> = exp.polls().iter().filter(|p| p.t_ms >= cool).collect();
        let last = polls.last().unwrap().t_ms;
        assert!(
            (30_000.0..30_100.0).contains(&(last - cool)),
            "{}",
            last - cool
        );
        assert!(
            polls.windows(2).all(|w| w[1].t_ms - w[0].t_ms < 60.0),
            "a poll gap in the cool-down"
        );
        let mut runs = Vec::new();
        for p in &polls {
            match (p.i_counts != 0.0, runs.last_mut()) {
                (true, Some((_, end, true))) => *end = p.t_ms,
                (on, _) => runs.push((p.t_ms, p.t_ms, on)),
            }
        }
        let pulses: Vec<(f64, f64)> = runs
            .iter()
            .filter(|r| r.2)
            .map(|r| (r.0 - cool, r.1 - r.0))
            .collect();
        assert_eq!(pulses.len(), 5, "{pulses:?}");
        for (k, (at, len)) in pulses.iter().enumerate() {
            let start = 5000.0 * (k + 1) as f64;
            assert!((start..start + 60.0).contains(at), "{pulses:?}");
            assert!((150.0..=230.0).contains(len), "{pulses:?}");
        }
        let pulse_windows: Vec<&WindowSample> =
            exp.samples().iter().filter(|w| w.t_ms >= cool).collect();
        assert!(pulse_windows.len() > 5 * 50, "{}", pulse_windows.len());
        for w in pulse_windows.iter().filter(|w| w.duty_q15 != 0.0) {
            let r = (w.duty_q15 * w.vdiff / 32767.0) / (w.i * w.duty_q15.signum());
            assert!((r - 3.37).abs() < 0.05, "r {r}");
        }
    }

    /// A fault in the cool-down ends the run like one in the hold: duty 0,
    /// torque off, the permit withdrawn.
    #[test]
    fn a_fault_in_the_cool_down_ends_the_run() {
        let mut servo = FakeServo::new(3.37);
        let cfg = AnchorCfg {
            hold_ms: 3000,
            cool_ms: 30_000,
            cool_every_ms: 5000,
            ..AnchorCfg::default()
        };
        servo.fault_at_ms = Some(12_000.0);
        let (exp, log, abort) = run(&mut servo, cfg);
        assert!(
            matches!(abort, Some(AbortReason::Fault { .. })),
            "{abort:?}"
        );
        assert!(
            exp.cool_ms().is_some(),
            "the fault landed before the cool-down"
        );
        let tail: Vec<&String> = log.iter().rev().take(3).collect();
        assert_eq!(*tail[2], "write goal_duty 0");
        assert_eq!(*tail[1], "write torque_enable 0");
        assert_eq!(*tail[0], "write stall_permit 0");
    }

    #[test]
    fn command_choreography_is_safe() {
        let mut servo = FakeServo::new(3.37);
        let (_, log, _) = run(&mut servo, short());
        let idx = |s: &str| log.iter().position(|l| l == s);
        let torque_on = idx("write torque_enable 1").expect("torque on");
        let permit_on = idx("write stall_permit 1").expect("the permit");
        let first_duty = log
            .iter()
            .position(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .expect("a drive command");
        assert!(torque_on < permit_on && permit_on < first_duty);
        // every drive points at the low stop
        assert!(
            log.iter()
                .filter_map(|l| l.strip_prefix("write goal_duty "))
                .all(|v| v.parse::<i32>().unwrap() <= 0),
            "{log:?}"
        );
        let tail: Vec<&String> = log.iter().rev().take(3).collect();
        assert_eq!(*tail[2], "write goal_duty 0");
        assert_eq!(*tail[1], "write torque_enable 0");
        assert_eq!(*tail[0], "write stall_permit 0");
    }

    /// A shaft that rests short of the stop is blocked: no hold is
    /// commanded against a jam, and nothing is fitted.
    #[test]
    fn a_mid_travel_jam_is_not_a_stop() {
        let mut servo = FakeServo::new(3.37);
        servo.jam = Some(1500.0);
        let (exp, log, abort) = run(&mut servo, short());
        assert!(
            matches!(abort, Some(AbortReason::Blocked { pos: 1500, .. })),
            "{abort:?}"
        );
        let seek = AnchorCfg::default().seek_duty_q15 as i32;
        assert!(
            log.iter()
                .filter_map(|l| l.strip_prefix("write goal_duty "))
                .all(|v| v.parse::<i32>().unwrap().abs() <= seek),
            "a hold was commanded: {log:?}"
        );
        assert!(exp.samples().is_empty());
        assert!(exp.t0_ms().is_none());
        assert!(exp.fit(4.0e-3, 0.9e-3).is_err());
        assert!(exp.fit_at(4, 4.0e-3, 0.9e-3).is_err());
    }
}
