//! The thermal hold: seek the low stop at the plan's seek duty, seat, hold
//! there for [`AnchorCfg::hold_ms`] at a duty whose cold current must sit
//! [`thermal::HOLD_MARGIN`] under the current ceiling, then step down to
//! [`AnchorCfg::cool_duty_q15`] for [`AnchorCfg::cool_ms`], torque on and
//! the rotor never released. It polls at a bounded rate throughout and
//! keeps every poll and every aggregate window, each judged as it lands
//! ([`ThermWindow`]). Duty x vdiff / i per window is the winding's own R;
//! [`crate::thermal::fit_rise`] fits the winding estimate's rise over the
//! hold at the servo's `thermal_tau_s`. A hold whose cold current crowds the
//! ceiling ends once its current's floor is read
//! ([`thermal::cold_current`]). The hold presses the horn further into the
//! stop than the seek seated it, so a window reads seated against the
//! hold's own seat ([`Anchor::hold_seat`]), through the cool-down too. Runs
//! with the pos guard off and the stall permit held, like the resistance
//! stop ladder; a seek that rests anywhere but the stop ends the run
//! ([`super::seek::at_stop`]).

use super::{AbortReason, Cmd, Experiment, RigParams, WindowStream, seek};
use crate::frame::{TelemetrySnapshot, counts_q15, duty_counts};
use crate::regs::control;
use crate::thermal::{self, CoolSummary, HoldPoll, RiseFit, ThermWindow};
use crate::window::Watch;

#[derive(Copy, Clone, Debug)]
pub struct AnchorCfg {
    /// Seek toward the low stop, q15, applied negative.
    pub seek_duty_q15: i16,
    /// The hold at the stop, q15, applied negative.
    pub hold_duty_q15: i16,
    /// The step-down after the hold, q15, applied negative.
    pub cool_duty_q15: i16,
    /// The higher published window floor, q15: a window whose applied duty
    /// is under it does not read the winding.
    pub read_floor_q15: i16,
    pub hold_ms: u32,
    /// The step-down after the hold; 0 runs none.
    pub cool_ms: u32,
    /// The hold's poll period: the fit bins by the second and the heat
    /// replay holds the last poll, so a fast poll buys nothing.
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
}

/// The shortest hold `osc ident thermal` takes, s: the rise fit needs
/// [`thermal::MIN_BINS`] seconds of it, R_ref the current's floor through
/// [`thermal::FLOOR_TO_MS`], and the heat several of its own time
/// constants.
pub const HOLD_MIN_S: u32 = 60;

/// The hold's seat is the median pot over this long of its windows at the
/// commanded duty, ms.
pub const SEAT_MS: f64 = 1000.0;

impl Default for AnchorCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 3932,
            hold_duty_q15: 5242,
            // 0.4 of the hold, `osc ident thermal`'s default
            cool_duty_q15: 2096,
            // dev-v006's terminal floor under Fast decay, 13.3%
            read_floor_q15: 4356,
            hold_ms: 480_000,
            cool_ms: 960_000,
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
    /// Why the hold ended before its time.
    refused: Option<String>,
    cold_checked: bool,
    seek_start: Option<u16>,
    last_pos: Option<u16>,
    still: u32,
    polls_left: u32,
    windows: WindowStream,
    samples: Vec<ThermWindow>,
    polls: Vec<HoldPoll>,
    t0_ms: Option<f64>,
    cool_ms: Option<f64>,
    seat: Option<u16>,
    hold_seat: Option<u16>,
    /// The goal duty last written, q15, signed.
    cmd_q15: i16,
    pwm_period_ticks: u16,
}

/// Toward the low stop.
const DIR: i16 = -1;

fn poll(o: &TelemetrySnapshot) -> HoldPoll {
    HoldPoll {
        t_ms: o.host_ms,
        i_counts: o.i_tick_counts as f64,
        t_ntc_cc: o.t_ntc_cc as f64,
        t_winding_cc: o.t_winding_cc as f64,
        i_lim_counts: o.i_lim_counts as f64,
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
            refused: None,
            cold_checked: false,
            seek_start: None,
            last_pos: None,
            still: 0,
            polls_left: 0,
            windows: WindowStream::new(params),
            samples: Vec::new(),
            polls: Vec::new(),
            t0_ms: None,
            cool_ms: None,
            seat: None,
            hold_seat: None,
            cmd_q15: 0,
            pwm_period_ticks: 0,
        }
    }

    /// Every aggregate window of the hold and the cool-down.
    pub fn samples(&self) -> &[ThermWindow] {
        &self.samples
    }

    /// Every poll, the seek's, the hold's and the cool-down's.
    pub fn polls(&self) -> &[HoldPoll] {
        &self.polls
    }

    /// Where the shaft seated, once it did.
    pub fn seat(&self) -> Option<u16> {
        self.seat
    }

    /// Where the hold pressed the shaft: the median pot over its first
    /// [`SEAT_MS`] at the commanded duty. Every hold and cool-down window
    /// is judged seated against it.
    pub fn hold_seat(&self) -> Option<u16> {
        self.hold_seat
    }

    /// The hold's start: the first poll after its duty write, which the
    /// kernel applies on its next tick.
    pub fn t0_ms(&self) -> Option<f64> {
        self.t0_ms
    }

    /// The cool-down's start: the first poll after its step-down write.
    pub fn cool_ms(&self) -> Option<f64> {
        self.cool_ms
    }

    /// Why the hold ended early, when its cold current crowded the
    /// ceiling, it never settled, or it never read at the command.
    pub fn refused(&self) -> Option<&str> {
        self.refused.as_deref()
    }

    /// The rise over the hold at the servo's `thermal_tau_s`
    /// ([`thermal::fit_rise`]), on its terminal and current scales.
    pub fn fit(
        &self,
        thermal_tau_s: u16,
        v_per_count: f64,
        a_per_count: f64,
    ) -> Result<RiseFit, String> {
        if let Some(why) = &self.refused {
            return Err(why.clone());
        }
        let t0 = self.t0_ms.ok_or("the hold never started")?;
        let seat = self.hold_seat.ok_or("the hold never seated")?;
        thermal::fit_rise(
            &self.polls,
            &self.samples,
            t0,
            self.cool_ms.unwrap_or(f64::INFINITY),
            seat,
            thermal_tau_s,
            v_per_count,
            a_per_count,
        )
    }

    /// The step-down to the run's last poll, once it started.
    pub fn cool_summary(&self) -> Option<CoolSummary> {
        thermal::cool_summary(&self.samples, self.cool_ms?, self.polls.last()?.t_ms)
    }

    /// Write the goal duty `q15`: every window from here is judged
    /// against it.
    fn goal(&mut self, q15: i16) -> Cmd {
        self.cmd_q15 = q15;
        // no goal mark: the limiter may govern on purpose, and each window
        // is judged against the command as it lands
        self.windows.mark_transition();
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: q15 as i32,
        }
    }

    fn torque(on: bool) -> Cmd {
        Cmd::Write {
            reg: control::TORQUE_ENABLE,
            value: on as i32,
        }
    }

    /// The windows watched since the last poll `o`, each judged against
    /// the duty in force while it was watched.
    fn take_windows(&mut self, o: &TelemetrySnapshot) {
        let pwm = self.pwm_period_ticks;
        let applied = |q15: i16| counts_q15(duty_counts(q15, pwm), pwm);
        let (cmd, floor) = (applied(self.cmd_q15), applied(self.cfg.read_floor_q15));
        for (w, pos) in self.windows.drain() {
            let pos = pos.unwrap_or(o.pos);
            self.samples.push(ThermWindow {
                w,
                pos,
                commanded: w.duty_q15 == cmd,
                floors: w.duty_q15.abs() >= floor,
                // until the hold's seat is read, judged when it is
                still: self.hold_seat.is_none_or(|_| self.seated(pos)),
            });
        }
    }

    /// `pos` sits within the stillness tolerance of the hold's seat.
    fn seated(&self, pos: u16) -> bool {
        self.hold_seat
            .is_some_and(|s| pos.abs_diff(s) <= self.stall_eps)
    }

    /// Once the hold's windows at its commanded duty cover [`SEAT_MS`],
    /// their median pot is the hold's seat, and every hold window so far is
    /// judged against it.
    fn read_hold_seat(&mut self, t0: f64) {
        if self.hold_seat.is_some() {
            return;
        }
        let from = self.samples.partition_point(|x| x.w.t_ms < t0);
        let at_duty = |x: &&ThermWindow| x.commanded && x.floors && x.w.i != 0.0;
        let held = &self.samples[from..];
        let Some(first) = held.iter().find(at_duty).map(|x| x.w.t_ms) else {
            return;
        };
        if held.last().is_none_or(|x| x.w.t_ms < first + SEAT_MS) {
            return;
        }
        let mut pos: Vec<u16> = held
            .iter()
            .take_while(|x| x.w.t_ms < first + SEAT_MS)
            .filter(at_duty)
            .map(|x| x.pos)
            .collect();
        pos.sort_unstable();
        self.hold_seat = Some(pos[pos.len() / 2]);
        for k in from..self.samples.len() {
            self.samples[k].still = self.seated(self.samples[k].pos);
        }
    }

    /// Once the hold's windows at its command cover the current's floor
    /// and the R_ref line, its cold current against the ceiling; a hold
    /// that crowds it, never settles, or never reads at the command is
    /// refused.
    fn check_cold(&mut self, t0: f64) {
        if self.cold_checked {
            return;
        }
        match thermal::cold_current(&self.polls, &self.samples, t0, self.hold_seat) {
            None => {}
            Some(Ok(_)) => self.cold_checked = true,
            Some(Err(why)) => {
                self.cold_checked = true;
                self.refused = Some(why);
            }
        }
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
                Self::torque(true)
            }
            Phase::SeekSet => {
                self.phase = Phase::SeekRead;
                self.goal(DIR * self.cfg.seek_duty_q15)
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
                        if let Err(reason) = seek::at_stop(start, o.pos, DIR as i8, self.stops) {
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
                self.goal(DIR * self.cfg.hold_duty_q15)
            }
            Phase::HoldRead => {
                self.phase = Phase::HoldEval;
                Cmd::Read
            }
            Phase::HoldEval => {
                let mut over = false;
                if let Some(o) = obs {
                    self.polls.push(poll(o));
                    self.take_windows(o);
                    let t0 = *self.t0_ms.get_or_insert(o.host_ms);
                    over = o.host_ms - t0 >= self.cfg.hold_ms as f64;
                    self.read_hold_seat(t0);
                    self.check_cold(t0);
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.refused.is_some() {
                    self.phase = Phase::FinishDuty;
                    Cmd::Pause { ms: 0 }
                } else if over || self.polls_left == 0 {
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
                // a backstop as the hold's
                self.polls_left = self.cfg.cool_ms / self.cfg.poll_ms.max(1) + 1;
                self.goal(DIR * self.cfg.cool_duty_q15)
            }
            Phase::CoolRead => {
                self.phase = Phase::CoolEval;
                Cmd::Read
            }
            Phase::CoolEval => {
                let mut over = false;
                if let Some(o) = obs {
                    self.polls.push(poll(o));
                    self.take_windows(o);
                    let e = o.host_ms - *self.cool_ms.get_or_insert(o.host_ms);
                    over = e >= self.cfg.cool_ms as f64;
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if over || self.polls_left == 0 {
                    self.phase = Phase::FinishDuty;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::CoolRead;
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
                Self::torque(false)
            }
            Phase::Finished => Cmd::Done,
        }
    }

    fn push_watch(&mut self, w: &Watch) {
        self.pwm_period_ticks = w.pwm_period_ticks;
        self.windows.watch(w);
    }

    fn halted(&self) -> Option<AbortReason> {
        self.halt
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, PWM_PERIOD_TICKS, bench_mg90, pump, rig};
    use super::super::{Guarded, Permitted};
    use super::*;

    fn run(servo: &mut FakeServo, cfg: AnchorCfg) -> (Anchor, Vec<String>, Option<AbortReason>) {
        let params = rig().without_pos_guard();
        let mut exp = Guarded::new(Permitted::new(Anchor::new(cfg, &params)), params);
        let log = pump(&mut exp, servo, 2_000_000);
        let abort = exp.abort();
        (exp.into_inner().into_inner(), log, abort)
    }

    /// The fake winding under a 280 count ceiling: the default hold draws
    /// 82, far under it.
    fn servo() -> FakeServo {
        let mut s = FakeServo::new(3.37);
        s.current_limit = Some(280);
        s
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
            ..AnchorCfg::default()
        }
    }

    fn r_of(w: &WindowSample) -> f64 {
        (w.duty_q15 * w.vdiff / 32767.0) / (w.i * w.duty_q15.signum())
    }

    use crate::exp::WindowSample;

    /// The writes after the hold's duty, the permit's refreshes left out.
    fn after_hold(log: &[String], hold_q15: i16) -> Vec<String> {
        let hold = format!("write goal_duty {}", -(hold_q15 as i32));
        let at = log.iter().position(|l| *l == hold).expect("a hold");
        log[at + 1..]
            .iter()
            .filter(|l| l.starts_with("write ") && *l != "write stall_permit 1")
            .cloned()
            .collect()
    }

    /// Seated at the low stop, the hold runs its time at the bounded poll
    /// rate and keeps the planted R in every window, each judged at the
    /// commanded duty, over the floors and seated.
    #[test]
    fn holds_its_time_at_the_low_stop() {
        let mut servo = servo();
        let (exp, log, abort) = run(&mut servo, short());
        assert_eq!(abort, None);
        assert_eq!(exp.refused(), None);
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
        let held: Vec<&ThermWindow> = exp.samples().iter().filter(|x| x.w.t_ms < cool).collect();
        assert!(held.len() > 50, "{} windows", held.len());
        for x in held {
            assert!((r_of(&x.w) - 3.37).abs() < 0.05, "r {}", r_of(&x.w));
            assert!(x.commanded && x.reads(), "{x:?}");
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

    /// A hold whose cold current sits under the ceiling by less than the
    /// margin ends once its current's floor is read, before the cool-down,
    /// and the fit names why.
    #[test]
    fn a_hold_inside_the_ceiling_margin_ends_once_its_floor_is_read() {
        let mut servo = FakeServo::new(3.37);
        // the hold draws 82 counts, 7 % under 88
        servo.current_limit = Some(88);
        let cfg = AnchorCfg {
            hold_ms: 120_000,
            ..short()
        };
        let (exp, log, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let why = exp.refused().expect("refused");
        assert!(why.contains("inside the 10 % margin"), "{why}");
        assert!(why.contains("88 counts current ceiling"), "{why}");
        assert!(why.contains("keeps the I2t fold off the hold"), "{why}");
        let t0 = exp.t0_ms().unwrap();
        let last = exp.polls().last().unwrap().t_ms;
        assert!(
            (thermal::FLOOR_TO_MS..thermal::FLOOR_TO_MS + 1500.0).contains(&(last - t0)),
            "held {} ms",
            last - t0
        );
        assert!(exp.cool_ms().is_none() && exp.cool_summary().is_none());
        assert_eq!(exp.fit(4, 4.0e-3, 0.9e-3).unwrap_err(), why);
        let tail: Vec<&String> = log.iter().rev().take(3).collect();
        assert_eq!(*tail[2], "write goal_duty 0");
        assert_eq!(*tail[1], "write torque_enable 0");
        assert_eq!(*tail[0], "write stall_permit 0");
    }

    /// A winding that does not warm over an NTC that does not move reads
    /// no rise against the heat the hold carried.
    #[test]
    fn a_winding_that_does_not_warm_reads_no_rise() {
        let mut servo = servo();
        let cfg = AnchorCfg {
            hold_ms: HOLD_MIN_S * 1000,
            cool_ms: 30_000,
            ..AnchorCfg::default()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let f = exp.fit(4, 4.0e-3, 0.9e-3).expect("a fit");
        assert!(f.bins >= thermal::MIN_BINS, "{f:?}");
        assert!(f.bins <= 61, "the cool-down is not the hold: {f:?}");
        assert!(f.k.abs() < 1e-3, "{f:?}");
        assert!(f.settled_rise_c.abs() < 0.5, "{f:?}");
        assert!(f.margin() > 0.6, "{f:?}");
        assert!(f.r_ref_settle_ms < thermal::R_REF_WITHIN_MS, "{f:?}");
    }

    /// After the hold the duty steps down to the cool-down's and stays
    /// there, torque on: one goal write, then nothing until the finish's
    /// duty 0 and torque off.
    #[test]
    fn the_cool_down_steps_down_with_torque_on() {
        let mut servo = servo();
        let cfg = AnchorCfg {
            hold_ms: 3000,
            cool_ms: 30_000,
            ..AnchorCfg::default()
        };
        let (exp, log, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        assert_eq!(
            after_hold(&log, cfg.hold_duty_q15),
            [
                format!("write goal_duty {}", -(cfg.cool_duty_q15 as i32)),
                "write goal_duty 0".into(),
                "write torque_enable 0".into(),
                "write stall_permit 0".into(),
            ],
            "{log:?}"
        );
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
        let s = exp.cool_summary().expect("a summary");
        assert!((s.s - (last - cool) / 1000.0).abs() < 1e-9, "{s:?}");
        assert!(s.windows > 30 * 300, "{s:?}");
    }

    /// Stepped down to a duty over the window floor, every cool-down window
    /// past the step reads the winding at the cool-down's commanded duty,
    /// its current down with the duty and R the hold's.
    #[test]
    fn cool_down_windows_read_commanded_at_the_cool_duty() {
        let mut servo = servo();
        let cfg = AnchorCfg {
            hold_duty_q15: 9830,
            cool_duty_q15: 4915,
            hold_ms: 3000,
            cool_ms: 10_000,
            ..AnchorCfg::default()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!((abort, exp.refused()), (None, None));
        let cool = exp.cool_ms().unwrap();
        let pwm = PWM_PERIOD_TICKS;
        let applied = |q15: i16| counts_q15(duty_counts(q15, pwm), pwm);
        let held: Vec<&ThermWindow> = exp
            .samples()
            .iter()
            .filter(|x| x.w.t_ms < cool && x.commanded)
            .collect();
        let late: Vec<&ThermWindow> = exp
            .samples()
            .iter()
            .filter(|x| x.w.t_ms >= cool + 100.0)
            .collect();
        assert!(late.len() > 10 * 300, "{}", late.len());
        for x in &late {
            assert!(x.commanded && x.reads(), "{x:?}");
            assert_eq!(x.w.duty_q15, applied(-cfg.cool_duty_q15), "{x:?}");
            assert!((r_of(&x.w) - 3.37).abs() < 0.05, "r {}", r_of(&x.w));
        }
        let mean =
            |xs: &[&ThermWindow]| xs.iter().map(|x| x.w.i.abs()).sum::<f64>() / xs.len() as f64;
        let ratio = mean(&late) / mean(&held);
        assert!((ratio - 0.5).abs() < 0.02, "{ratio}");
        let s = exp.cool_summary().unwrap();
        assert!(s.read >= late.len(), "{s:?}");
        assert!((s.i_counts / mean(&held) - 0.5).abs() < 0.02, "{s:?}");
    }

    /// The bench horn on D: the seek seats it, the hold presses it 14
    /// counts further in for the whole hold, and at the default step-down,
    /// under the give, it springs back to the seek's seat. Every window is
    /// judged against the hold's own seat: every hold window after the
    /// first second reads still, the cool-down's sprung windows and
    /// anything 2 x stall_eps off the hold's seat do not.
    #[test]
    fn stillness_is_judged_against_the_hold_seat() {
        let mut servo = servo();
        servo.low_give = Some((5000, 14.0));
        let cfg = AnchorCfg {
            hold_ms: 3000,
            cool_ms: 30_000,
            ..AnchorCfg::default()
        };
        assert!(cfg.cool_duty_q15 < 5000 && cfg.hold_duty_q15 >= 5000);
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!((abort, exp.refused()), (None, None));
        let (seat, hold) = (exp.seat().unwrap(), exp.hold_seat().unwrap());
        assert_eq!(seat - hold, 14, "seek seat {seat}, hold seat {hold}");
        let (t0, cool) = (exp.t0_ms().unwrap(), exp.cool_ms().unwrap());
        let at_duty = |x: &&ThermWindow| x.commanded && x.floors;
        let first = exp
            .samples()
            .iter()
            .filter(|x| x.w.t_ms >= t0)
            .find(at_duty)
            .unwrap()
            .w
            .t_ms;
        let held: Vec<&ThermWindow> = exp
            .samples()
            .iter()
            .filter(|x| x.w.t_ms >= first + 1000.0 && x.w.t_ms < cool)
            .collect();
        assert!(held.len() > 1000, "{}", held.len());
        assert!(
            held.iter().all(|x| x.still && x.reads()),
            "a hold window off its seat"
        );
        let sprung: Vec<&ThermWindow> = exp
            .samples()
            .iter()
            .filter(|x| x.w.t_ms >= cool + 100.0)
            .collect();
        assert!(sprung.len() > 1000, "{}", sprung.len());
        assert!(sprung.iter().all(|x| x.pos == seat && !x.still));
        let eps = rig().stall_eps;
        assert!(exp.seated(hold + eps) && exp.seated(hold - eps));
        assert!(!exp.seated(hold + 2 * eps) && !exp.seated(hold - 2 * eps));
    }

    /// A fault in the cool-down ends the run like one in the hold: duty 0,
    /// torque off, the permit withdrawn.
    #[test]
    fn a_fault_in_the_cool_down_ends_the_run() {
        let mut servo = servo();
        let cfg = AnchorCfg {
            hold_ms: 3000,
            cool_ms: 30_000,
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
        let mut servo = servo();
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
        let mut servo = servo();
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
