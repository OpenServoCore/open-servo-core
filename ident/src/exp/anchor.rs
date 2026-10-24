//! The thermal hold: seek the low stop at the plan's seek duty, seat, and
//! hold there for [`AnchorCfg::hold_ms`] at one duty, torque on and the
//! rotor never released, whose cold current must sit
//! [`thermal::HOLD_MARGIN`] under the current ceiling. It polls at a
//! bounded rate throughout and keeps every poll and every aggregate window,
//! each judged as it lands ([`ThermWindow`]). Duty x vdiff / i per window
//! is the winding's own R; [`crate::thermal::fit_two_node`] fits the
//! copper's rise over the hold with two heat nodes and their octaves. The
//! hold presses the horn further into the stop than the seek seated it, so
//! a window reads seated against the hold's own seat
//! ([`Anchor::hold_seat`]).
//!
//! Three watches guard the hold as it runs. A cold current that crowds the
//! ceiling refuses it once its current's floor is read
//! ([`thermal::cold_current`]). A current ceiling under
//! [`AnchorCfg::cap_counts`] at any poll is the limiter governing, a rail
//! fold or an NTC fault, and refuses it. A step in R with the pot still is
//! a brush contact event ([`thermal::contact_events`]) and refuses it. Once
//! R_ref is read the copper's temperature is watched against
//! [`AnchorCfg::guard_c`], the servo's own derate start, and reaching it
//! ends the hold early with what it has ([`Anchor::ended_early`]). Runs with
//! the pos guard off and the stall permit held, like the resistance stop
//! ladder; a seek that rests anywhere but the stop ends the run
//! ([`super::seek::at_stop`]).

use super::{AbortReason, Cmd, Experiment, RigParams, WindowStream, seek};
use crate::frame::{TelemetrySnapshot, counts_q15, duty_counts};
use crate::regs::control;
use crate::thermal::{self, HoldPoll, ThermWindow, TwoNodeFit};
use crate::window::Watch;

#[derive(Copy, Clone, Debug)]
pub struct AnchorCfg {
    /// Seek toward the low stop, q15, applied negative.
    pub seek_duty_q15: i16,
    /// The hold at the stop, q15, applied negative.
    pub hold_duty_q15: i16,
    /// The higher published window floor, q15: a window whose applied duty
    /// is under it does not read the winding.
    pub read_floor_q15: i16,
    pub hold_ms: u32,
    /// The hold's poll period: the fit bins by the second and the heat
    /// replay holds the last poll, so a fast poll buys nothing.
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
    /// The servo's saved current limit, counts: a polled ceiling under it is
    /// the limiter governing and refuses the hold. 0 watches nothing.
    pub cap_counts: u16,
    /// The copper temperature, C, that ends the hold: the servo's saved
    /// derate start. Infinity watches nothing.
    pub guard_c: f64,
}

/// The shortest hold `osc ident thermal` takes, s: the body octave the hold
/// decides needs it to cover the faster body tau [`thermal::BODY_COVER`]
/// times over.
pub const HOLD_MIN_S: u32 = 480;

/// The hold's seat is the median pot over this long of its windows at the
/// commanded duty, ms.
pub const SEAT_MS: f64 = 1000.0;

/// The copper the guard reads is the mean over the windows this far back,
/// ms.
pub const GUARD_MS: f64 = 1000.0;

impl Default for AnchorCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 3932,
            hold_duty_q15: 5242,
            // dev-v006's terminal floor under Fast decay, 13.3%
            read_floor_q15: 4356,
            hold_ms: 960_000,
            poll_ms: 20,
            seek_poll_ms: 30,
            cap_counts: 0,
            guard_c: f64::INFINITY,
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
    /// Why the hold was refused before its time.
    refused: Option<String>,
    /// Why the guard ended the hold before its time.
    ended: Option<String>,
    cold_checked: bool,
    /// R_ref, vcounts per ccount, and T_0, C, once the reference is read.
    guard: Option<(f64, f64)>,
    /// When the contact detector next runs over the hold, ms.
    contact_at: f64,
    seek_start: Option<u16>,
    last_pos: Option<u16>,
    still: u32,
    polls_left: u32,
    windows: WindowStream,
    samples: Vec<ThermWindow>,
    polls: Vec<HoldPoll>,
    t0_ms: Option<f64>,
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
            ended: None,
            cold_checked: false,
            guard: None,
            contact_at: 0.0,
            seek_start: None,
            last_pos: None,
            still: 0,
            polls_left: 0,
            windows: WindowStream::new(params),
            samples: Vec::new(),
            polls: Vec::new(),
            t0_ms: None,
            seat: None,
            hold_seat: None,
            cmd_q15: 0,
            pwm_period_ticks: 0,
        }
    }

    /// Every aggregate window of the hold.
    pub fn samples(&self) -> &[ThermWindow] {
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

    /// Where the hold pressed the shaft: the median pot over its first
    /// [`SEAT_MS`] at the commanded duty. Every hold window is judged seated
    /// against it.
    pub fn hold_seat(&self) -> Option<u16> {
        self.hold_seat
    }

    /// The hold's start: the first poll after its duty write, which the
    /// kernel applies on its next tick.
    pub fn t0_ms(&self) -> Option<f64> {
        self.t0_ms
    }

    /// Why the hold was refused: its cold current crowded the ceiling, it
    /// never settled or never read at the command, the limiter governed it,
    /// or a brush contact event stepped its R.
    pub fn refused(&self) -> Option<&str> {
        self.refused.as_deref()
    }

    /// Why the guard ended the hold before its time: the copper reached
    /// [`AnchorCfg::guard_c`].
    pub fn ended_early(&self) -> Option<&str> {
        self.ended.as_deref()
    }

    /// The two heat nodes and their octaves over the hold
    /// ([`thermal::fit_two_node`]), on its terminal and current scales. A
    /// hold the guard ended under [`HOLD_MIN_S`] is not fitted.
    pub fn fit(&self, v_per_count: f64, a_per_count: f64) -> Result<TwoNodeFit, String> {
        if let Some(why) = &self.refused {
            return Err(why.clone());
        }
        let t0 = self.t0_ms.ok_or("the hold never started")?;
        let seat = self.hold_seat.ok_or("the hold never seated")?;
        if let Some(why) = &self.ended
            && let Some(last) = self.polls.last()
            && last.t_ms - t0 < (HOLD_MIN_S * 1000) as f64
        {
            return Err(format!("{why}, under the {HOLD_MIN_S} s a fit needs"));
        }
        thermal::fit_two_node(
            &self.polls,
            &self.samples,
            t0,
            f64::INFINITY,
            seat,
            v_per_count,
            a_per_count,
        )
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
    /// refused. One that passes reads its reference for the guard.
    fn check_cold(&mut self, t0: f64) {
        if self.cold_checked {
            return;
        }
        match thermal::cold_current(&self.polls, &self.samples, t0, self.hold_seat) {
            None => {}
            Some(Ok(_)) => {
                self.cold_checked = true;
                let seat = self.hold_seat.unwrap_or_default();
                match thermal::hold_reference(&self.polls, &self.samples, t0, seat) {
                    Ok(g) => self.guard = Some(g),
                    Err(why) => self.refused = Some(why),
                }
            }
            Some(Err(why)) => {
                self.cold_checked = true;
                self.refused = Some(why);
            }
        }
    }

    /// The limiter governing the hold at this poll refuses it.
    fn watch_limiter(&mut self, o: &TelemetrySnapshot, t0: f64) {
        let cap = self.cfg.cap_counts;
        if cap != 0 && o.i_lim_counts < cap && self.refused.is_none() {
            self.refused = Some(format!(
                "the limiter governed the hold {:.0} s in: the current ceiling read {} counts \
                 under the {cap} counts limit",
                (o.host_ms - t0) / 1000.0,
                o.i_lim_counts
            ));
        }
    }

    /// Every [`thermal::CONTACT_BLOCK_MS`], the hold's windows so far for a
    /// brush contact event; the first final one refuses the hold.
    fn watch_contact(&mut self, now: f64, t0: f64) {
        if now < self.contact_at || self.refused.is_some() {
            return;
        }
        self.contact_at = now + thermal::CONTACT_BLOCK_MS;
        if let Some(e) = thermal::contact_events(&self.samples, t0, now)
            .iter()
            .find(|e| e.to_ms + 2.0 * thermal::CONTACT_BLOCK_MS <= now)
        {
            self.refused = Some(format!(
                "a brush contact event {:.0} s in: R stepped {:+.1} % with the pot still, \
                 {:.0} to {:.0} counts of current at pot {:.1} to {:.1}",
                (e.t_ms - t0) / 1000.0,
                e.step_frac * 100.0,
                e.i_before,
                e.i_after,
                e.pos_before,
                e.pos_after
            ));
        }
    }

    /// Once the reference is read, the copper over the last [`GUARD_MS`]
    /// against the guard; reaching it ends the hold.
    fn watch_guard(&mut self, now: f64, t0: f64) {
        let Some((r_ref, t0_c)) = self.guard else {
            return;
        };
        if let Some(c) = thermal::copper_c(&self.samples, now - GUARD_MS, r_ref, t0_c)
            && c >= self.cfg.guard_c
            && self.ended.is_none()
        {
            self.ended = Some(format!(
                "the thermometer guard ended the hold {:.0} s in: the copper read {c:.1} C \
                 against the {:.1} C derate start",
                (now - t0) / 1000.0,
                self.cfg.guard_c
            ));
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
                    self.watch_limiter(o, t0);
                    self.watch_contact(o.host_ms, t0);
                    self.watch_guard(o.host_ms, t0);
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.refused.is_some() || self.ended.is_some() || over || self.polls_left == 0 {
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
    use super::super::testkit::{FakeServo, bench_mg90, pump, rig};
    use super::super::{Guarded, Permitted};
    use super::*;
    use crate::exp::WindowSample;

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
            ..AnchorCfg::default()
        }
    }

    fn short() -> AnchorCfg {
        AnchorCfg {
            hold_ms: 3000,
            ..AnchorCfg::default()
        }
    }

    fn r_of(w: &WindowSample) -> f64 {
        (w.duty_q15 * w.vdiff / 32767.0) / (w.i * w.duty_q15.signum())
    }

    /// The hold's span, ms: its start to the run's last poll.
    fn held_ms(exp: &Anchor) -> f64 {
        exp.polls().last().unwrap().t_ms - exp.t0_ms().unwrap()
    }

    fn assert_safe_tail(log: &[String]) {
        let tail: Vec<&String> = log.iter().rev().take(3).collect();
        assert_eq!(*tail[2], "write goal_duty 0");
        assert_eq!(*tail[1], "write torque_enable 0");
        assert_eq!(*tail[0], "write stall_permit 0");
    }

    /// Seated at the low stop, the hold runs its time at the bounded poll
    /// rate and keeps the planted R in every window, each judged at the
    /// commanded duty, over the floors and seated.
    #[test]
    fn holds_its_time_at_the_low_stop() {
        let mut servo = servo();
        let (exp, log, abort) = run(&mut servo, short());
        assert_eq!(abort, None);
        assert_eq!((exp.refused(), exp.ended_early()), (None, None));
        assert!(!log.contains(&"OVERRUN".to_string()));
        let seat = exp.seat().expect("seated");
        assert!(seat <= 200 + seek::STOP_TOL, "seat {seat}");
        let t0 = exp.t0_ms().expect("a hold");
        let held = held_ms(&exp);
        assert!((3000.0..3100.0).contains(&held), "held {held}");
        let hold_polls = exp.polls().iter().filter(|p| p.t_ms >= t0).count();
        assert!(hold_polls <= 3000 / 20 + 2, "{hold_polls} polls");
        assert!(exp.samples().len() > 50, "{} windows", exp.samples().len());
        for x in exp.samples() {
            assert!((r_of(&x.w) - 3.37).abs() < 0.05, "r {}", r_of(&x.w));
            assert!(x.commanded && x.reads(), "{x:?}");
        }
        // the polls before t0 are the seek's
        assert!(exp.polls().iter().any(|p| p.t_ms < t0));
        assert_safe_tail(&log);
    }

    /// A ceiling that drops under the saved limit mid hold is the limiter
    /// governing it, here to an NTC fault's 0: the hold ends at that poll,
    /// refused.
    #[test]
    fn a_ceiling_under_the_limit_refuses_the_hold() {
        let mut servo = servo();
        servo.limit_drop = Some((8000.0, 0));
        let cfg = AnchorCfg {
            hold_ms: 30_000,
            cap_counts: 280,
            ..AnchorCfg::default()
        };
        let (exp, log, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let why = exp.refused().expect("refused");
        assert!(why.contains("the limiter governed the hold"), "{why}");
        assert!(
            why.contains("read 0 counts under the 280 counts limit"),
            "{why}"
        );
        let last = exp.polls().last().unwrap().t_ms;
        assert!((8000.0..8100.0).contains(&last), "ended at {last} ms");
        assert_eq!(exp.fit(4.0e-3, 0.9e-3).unwrap_err(), why);
        assert_safe_tail(&log);
    }

    /// Without the cap the same hold runs on, the limit governing its
    /// current.
    #[test]
    fn without_a_cap_the_limit_governs_the_hold() {
        let mut servo = bench_mg90(3204);
        servo.r = 3.37;
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
    /// margin ends once its current's floor is read, and the fit names why.
    #[test]
    fn a_hold_inside_the_ceiling_margin_is_refused_once_its_floor_is_read() {
        let mut servo = FakeServo::new(3.37);
        // the hold draws 82 counts, 7 % under 88
        servo.current_limit = Some(88);
        let cfg = AnchorCfg {
            hold_ms: 120_000,
            ..AnchorCfg::default()
        };
        let (exp, log, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let why = exp.refused().expect("refused");
        assert!(why.contains("inside the 10 % margin"), "{why}");
        assert!(why.contains("88 counts current ceiling"), "{why}");
        assert!(why.contains("keeps the limiter off the hold"), "{why}");
        let held = held_ms(&exp);
        assert!(
            (thermal::FLOOR_TO_MS..thermal::FLOOR_TO_MS + 1500.0).contains(&held),
            "held {held} ms"
        );
        assert_eq!(exp.ended_early(), None);
        assert_eq!(exp.fit(4.0e-3, 0.9e-3).unwrap_err(), why);
        assert_safe_tail(&log);
    }

    /// A winding that does not warm over an NTC that does not move fits no
    /// gain on either node against the heat the hold carried, and the
    /// undecided body writes the octave a shortest hold covers.
    #[test]
    fn a_winding_that_does_not_warm_reads_no_rise() {
        let mut servo = servo();
        let cfg = AnchorCfg {
            hold_ms: HOLD_MIN_S * 1000,
            ..AnchorCfg::default()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        assert_eq!((exp.refused(), exp.ended_early()), (None, None));
        let f = exp.fit(4.0e-3, 0.9e-3).expect("a fit");
        assert!(f.bins >= thermal::MIN_BINS, "{f:?}");
        let h = &f.holds[0];
        assert!((h.hold_s - HOLD_MIN_S as f64).abs() <= 1.0, "{f:?}");
        assert!(f.rotor.k.abs() < 1e-3 && f.body.k.abs() < 1e-3, "{f:?}");
        assert_eq!((f.rotor.tau_s, f.body.tau_s), (33, 262));
        assert!(h.settled_rise_c.abs() < 0.5, "{f:?}");
        assert!(h.margin() > 0.6, "{f:?}");
        assert!(h.r_ref_settle_ms < thermal::R_REF_WITHIN_MS, "{f:?}");
    }

    /// With the guard under the copper's own reading, the hold ends as
    /// soon as its reference is read, torque off like any end; a hold that
    /// short is not fitted.
    #[test]
    fn the_guard_ends_the_hold_once_the_copper_reaches_it() {
        let mut servo = servo();
        let cfg = AnchorCfg {
            hold_ms: 120_000,
            guard_c: -1.0,
            ..AnchorCfg::default()
        };
        let (exp, log, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        assert_eq!(exp.refused(), None);
        let why = exp.ended_early().expect("ended");
        assert!(
            why.contains("the thermometer guard ended the hold"),
            "{why}"
        );
        assert!(why.contains("against the -1.0 C derate start"), "{why}");
        let held = held_ms(&exp);
        assert!(
            (thermal::FLOOR_TO_MS..thermal::FLOOR_TO_MS + 5000.0).contains(&held),
            "held {held} ms"
        );
        let err = exp.fit(4.0e-3, 0.9e-3).unwrap_err();
        assert!(err.starts_with(why), "{err}");
        assert!(err.ends_with("under the 480 s a fit needs"), "{err}");
        assert_safe_tail(&log);
    }

    /// A step in R with the pot still is a brush contact event: the hold
    /// ends once the event is final, refused with the whole step. The fake
    /// rounds its current to a count, so a 5 % step reads within one.
    #[test]
    fn a_contact_event_refuses_the_hold() {
        let mut servo = servo();
        servo.r_step = Some((20_000.0, 1.05));
        let cfg = AnchorCfg {
            hold_ms: 60_000,
            ..AnchorCfg::default()
        };
        let (exp, log, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let why = exp.refused().expect("refused");
        assert!(why.contains("a brush contact event"), "{why}");
        let step: f64 = why
            .split("R stepped +")
            .nth(1)
            .and_then(|r| r.split(" %").next())
            .and_then(|n| n.parse().ok())
            .expect(why);
        assert!((4.0..6.0).contains(&step), "{why}");
        let last = exp.polls().last().unwrap().t_ms;
        assert!(
            (20_000.0..20_000.0 + 5.0 * thermal::CONTACT_BLOCK_MS).contains(&last),
            "ended at {last} ms"
        );
        assert_eq!(exp.fit(4.0e-3, 0.9e-3).unwrap_err(), why);
        assert_safe_tail(&log);
    }

    /// The bench horn on D: the seek seats it and the hold presses it 14
    /// counts further in for the whole hold. Every window is judged
    /// against the hold's own seat: every hold window after the first
    /// second reads still, anything 2 x stall_eps off the hold's seat does
    /// not.
    #[test]
    fn stillness_is_judged_against_the_hold_seat() {
        let mut servo = servo();
        servo.low_give = Some((5000, 14.0));
        let cfg = short();
        assert!(cfg.hold_duty_q15 >= 5000);
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!((abort, exp.refused()), (None, None));
        let (seat, hold) = (exp.seat().unwrap(), exp.hold_seat().unwrap());
        assert_eq!(seat - hold, 14, "seek seat {seat}, hold seat {hold}");
        let t0 = exp.t0_ms().unwrap();
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
            .filter(|x| x.w.t_ms >= first + 1000.0)
            .collect();
        assert!(held.len() > 500, "{}", held.len());
        assert!(
            held.iter().all(|x| x.still && x.reads()),
            "a hold window off its seat"
        );
        let eps = rig().stall_eps;
        assert!(exp.seated(hold + eps) && exp.seated(hold - eps));
        assert!(!exp.seated(hold + 2 * eps) && !exp.seated(hold - 2 * eps));
    }

    /// A fault in the hold ends the run: duty 0, torque off, the permit
    /// withdrawn.
    #[test]
    fn a_fault_in_the_hold_ends_the_run() {
        let mut servo = servo();
        let cfg = AnchorCfg {
            hold_ms: 30_000,
            ..AnchorCfg::default()
        };
        servo.fault_at_ms = Some(12_000.0);
        let (exp, log, abort) = run(&mut servo, cfg);
        assert!(
            matches!(abort, Some(AbortReason::Fault { .. })),
            "{abort:?}"
        );
        assert!(exp.t0_ms().is_some(), "the fault landed before the hold");
        assert!(held_ms(&exp) < 15_000.0);
        assert_safe_tail(&log);
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
        assert_safe_tail(&log);
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
        assert!(exp.fit(4.0e-3, 0.9e-3).is_err());
    }
}
