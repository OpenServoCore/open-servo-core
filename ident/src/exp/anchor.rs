//! The thermal hold: seek the low stop at the plan's seek duty, seat, hold
//! at the stall-safe stop duty (the current limit governs it) for
//! [`AnchorCfg::hold_ms`], polling at a bounded rate, and keep every poll
//! and every aggregate window. Duty x vdiff / i per window is the winding's
//! own R, whose ratio to the R just after the hold starts reads the
//! copper's rise; [`crate::thermal::fit_rise`] regresses it over the NTC on
//! the kernel's replayed heat for the winding estimate's rise constant.
//! Runs with the pos guard off and the stall permit held, like the
//! resistance stop ladder; a seek that rests anywhere but the stop ends
//! the run ([`super::seek::at_stop`]).

use super::{AbortReason, Cmd, Experiment, RigParams, WindowSample, WindowStream, seek};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;
use crate::thermal::{self, HoldPoll, RiseFit};
use crate::window::Watch;

pub struct AnchorCfg {
    /// Seek toward the low stop, q15, applied negative.
    pub seek_duty_q15: i16,
    /// The hold at the stop, q15: the stall-safe cap, so the limit governs.
    pub hold_duty_q15: i16,
    pub hold_ms: u32,
    /// The hold's poll period: the fit bins by the second and the heat
    /// replay holds the last poll, so a fast poll buys nothing.
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
}

/// The shortest hold `osc ident thermal` takes, s: the rise fit needs
/// [`thermal::MIN_BINS`] seconds of it and the heat several of its own
/// time constants.
pub const HOLD_MIN_S: u32 = 60;

impl Default for AnchorCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 3932,
            hold_duty_q15: 5242,
            hold_ms: 480_000,
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

    /// The rise over the hold ([`thermal::fit_rise`]) on the servo's
    /// `thermal_tau_s` and terminal and current scales.
    pub fn fit(
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
            thermal_tau_s,
            v_per_count,
            a_per_count,
        )
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
                    self.phase = Phase::FinishDuty;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::HoldRead;
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
            ..AnchorCfg::default()
        }
    }

    fn short() -> AnchorCfg {
        AnchorCfg {
            hold_ms: 3000,
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
        let last = exp.polls().last().unwrap().t_ms;
        assert!(
            (3000.0..3100.0).contains(&(last - t0)),
            "held {}",
            last - t0
        );
        let hold_polls = exp.polls().iter().filter(|p| p.t_ms >= t0).count();
        assert!(hold_polls <= 3000 / 20 + 1, "{hold_polls} polls");
        assert!(exp.samples().len() > 50, "{} windows", exp.samples().len());
        for w in exp.samples() {
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
            ..AnchorCfg::default()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let f = exp.fit(4, 4.0e-3, 0.9e-3).expect("a fit");
        assert!(f.bins >= thermal::MIN_BINS, "{f:?}");
        assert!(f.k.abs() < 1e-3, "{f:?}");
        assert!(f.settled_rise_c.abs() < 0.5, "{f:?}");
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
        assert!(exp.fit(4, 4.0e-3, 0.9e-3).is_err());
    }
}
