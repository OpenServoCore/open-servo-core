//! The short-hold session: the rotor node from short holds at fresh brush
//! seats, the body node pinned from the servo's long hold
//! ([`thermal::fit_rotor_pinned`]). Each hold is one [`Anchor`], the holds
//! alternating between the low stop and the high stop: the seek between
//! them turns the rotor hundreds of times, so every hold sits on a fresh
//! seat, each brush takes both polarities, and a sign-dependent sense
//! offset shows as a split between the stops rather than a bias. Between
//! holds the servo rests torque off for [`SessionCfg::rest_ms`], polled
//! every [`SessionCfg::rest_poll_ms`].
//!
//! The whole session is one record ([`ShortHolds::polls`],
//! [`ShortHolds::samples`]): the kernel replay runs from its first poll and
//! carries the heat across holds and rests, and each hold's fit reads its
//! NTC base from the polls before its own start, so the rests are short.
//! The pool decides when it has enough: the user cannot know the motor's
//! segment count, so no count is fixed. After each hold the hold alone is
//! fitted with the body pinned; a hold the anchor refused or the fit
//! rejected is dropped from the pool with its reason and still counts
//! toward [`SessionCfg::holds_max`]. The accepted holds are pooled and
//! their single fits' spread read ([`thermal::pool_spread`]); the session
//! ends decided once [`SessionCfg::holds_min`] holds are in, the pooled
//! fit stands and the pool's standard error at the ceiling sits inside
//! [`SessionCfg::within_c`], undecided at `holds_max` or after two refused
//! holds in a row. A pooled fit can fail where every single passed (seats
//! far enough apart read as an unsettled contact under one shared gain):
//! the last pool that stood is kept and the failure reported.

use super::anchor::{Anchor, AnchorCfg, HoldMarks, poll};
use super::{AbortReason, Cmd, Experiment, RigParams};
use crate::frame::TelemetrySnapshot;
use crate::thermal::{self, Hold, HoldPoll, NodeFit, PoolSpread, ThermWindow, TwoNodeFit};
use crate::window::Watch;

#[derive(Clone, Debug)]
pub struct SessionCfg {
    /// Each hold, `hold_ms` the short hold; `dir` is set per hold by the
    /// alternation, the first toward the low stop.
    pub anchor: AnchorCfg,
    /// The body node the servo's long hold wrote.
    pub body: NodeFit,
    pub v_per_count: f64,
    pub a_per_count: f64,
    /// The current ceiling the spread is read at, counts.
    pub ceiling_counts: f64,
    /// The pool decides once its mean's standard error at the ceiling sits
    /// inside this, C.
    pub within_c: f64,
    /// Accepted holds before the pool may decide; the spread itself needs
    /// [`thermal::SPREAD_MIN_HOLDS`].
    pub holds_min: usize,
    /// Holds run, accepted or dropped, before the session ends undecided.
    pub holds_max: usize,
    pub rest_ms: u32,
    pub rest_poll_ms: u32,
}

/// The seek between the stops is allowed this long in the session's time
/// estimate, ms: a seek crosses the whole travel at the plan's seek duty
/// and then sits still for the stall polls, a few seconds on any servo the
/// plan drives. The estimate is for the user's clock, never a timeout.
pub const SEEK_ALLOWANCE_MS: f64 = 5000.0;

/// How long `holds` holds take, ms: each a seek, the hold and the rest.
pub fn planned_ms(cfg: &SessionCfg, holds: usize) -> f64 {
    holds as f64 * (SEEK_ALLOWANCE_MS + cfg.anchor.hold_ms as f64 + cfg.rest_ms as f64)
}

/// What the session read.
#[derive(Clone, Debug, PartialEq)]
pub struct SessionFit {
    /// The rotor over every accepted hold, the body as pinned: the last
    /// pooled fit that succeeded.
    pub pooled: TwoNodeFit,
    /// Why the pooled fit over every accepted hold failed, when the last
    /// one did (`pooled` is then an earlier pool, and `decided` is false).
    pub pool_failed: Option<String>,
    /// Each accepted hold fitted alone, in order.
    pub singles: Vec<TwoNodeFit>,
    /// The accepted holds' marks, one per single, in order.
    pub accepted: Vec<HoldMarks>,
    /// The singles' spread; None under [`thermal::SPREAD_MIN_HOLDS`]
    /// accepted holds.
    pub spread: Option<PoolSpread>,
    /// Every hold that started and seated, accepted or dropped, in order.
    pub marks: Vec<HoldMarks>,
    /// Each dropped hold by its number as the log names it (the first hold
    /// is 1), with why.
    pub dropped: Vec<(usize, String)>,
    /// The pool's spread sat inside [`SessionCfg::within_c`].
    pub decided: bool,
    /// Both gains free over the accepted holds: the body's drift across
    /// the session makes it visible again. Reported, never written.
    pub cross_check: Option<TwoNodeFit>,
}

enum Phase {
    /// The live anchor runs.
    Hold,
    RestRead,
    RestEval,
    Finished,
}

pub struct ShortHolds {
    cfg: SessionCfg,
    params: RigParams,
    phase: Phase,
    anchor: Anchor,
    dir: i8,
    halt: Option<AbortReason>,
    polls: Vec<HoldPoll>,
    samples: Vec<ThermWindow>,
    marks: Vec<HoldMarks>,
    /// The accepted holds' marks, their single fits and their stops.
    accepted: Vec<HoldMarks>,
    singles: Vec<TwoNodeFit>,
    dirs: Vec<i8>,
    dropped: Vec<(usize, String)>,
    /// Holds the anchor refused in a row.
    refused_run: usize,
    holds_run: usize,
    /// The last pooled fit that succeeded, and why the latest failed.
    pooled: Option<TwoNodeFit>,
    pool_failed: Option<String>,
    spread: Option<PoolSpread>,
    decided: bool,
    verdict: Option<String>,
    log: Vec<String>,
    /// When the rest ends, ms on the host's clock.
    rest_end_ms: f64,
}

impl ShortHolds {
    pub fn new(cfg: SessionCfg, params: &RigParams) -> Self {
        let dir = -1;
        Self {
            anchor: Anchor::new(AnchorCfg { dir, ..cfg.anchor }, params),
            cfg,
            params: *params,
            phase: Phase::Hold,
            dir,
            halt: None,
            polls: Vec::new(),
            samples: Vec::new(),
            marks: Vec::new(),
            accepted: Vec::new(),
            singles: Vec::new(),
            dirs: Vec::new(),
            dropped: Vec::new(),
            refused_run: 0,
            holds_run: 0,
            pooled: None,
            pool_failed: None,
            spread: None,
            decided: false,
            verdict: None,
            log: Vec::new(),
            rest_end_ms: 0.0,
        }
    }

    /// Every poll of the session: each hold's seek and hold, and each rest.
    pub fn polls(&self) -> &[HoldPoll] {
        &self.polls
    }

    /// Every aggregate window of every hold.
    pub fn samples(&self) -> &[ThermWindow] {
        &self.samples
    }

    /// One line per hold as it ended.
    pub fn log_lines(&self) -> &[String] {
        &self.log
    }

    /// Why the session ended, once it did.
    pub fn verdict(&self) -> Option<&str> {
        self.verdict.as_deref()
    }

    /// Every hold that started and seated, in order.
    pub fn marks(&self) -> &[HoldMarks] {
        &self.marks
    }

    pub fn result(&self) -> Result<SessionFit, String> {
        if self.accepted.is_empty() {
            let why: Vec<String> = self
                .dropped
                .iter()
                .map(|(n, why)| format!("hold {n}: {why}"))
                .collect();
            return Err(format!("no hold was accepted: {}", why.join("; ")));
        }
        let pooled = self
            .pooled
            .clone()
            .ok_or_else(|| self.pool_failed.clone().unwrap_or_default())?;
        let holds: Vec<Hold<'_>> = self.accepted.iter().map(|m| self.hold_of(m)).collect();
        let cross_check =
            thermal::fit_two_node_pooled(&holds, self.cfg.v_per_count, self.cfg.a_per_count).ok();
        Ok(SessionFit {
            pooled,
            pool_failed: self.pool_failed.clone(),
            singles: self.singles.clone(),
            accepted: self.accepted.clone(),
            spread: self.spread.clone(),
            marks: self.marks.clone(),
            dropped: self.dropped.clone(),
            decided: self.decided,
            cross_check,
        })
    }

    /// The hold at `m` over the session's record.
    fn hold_of(&self, m: &HoldMarks) -> Hold<'_> {
        Hold {
            polls: &self.polls,
            windows: &self.samples,
            t0_ms: m.t0_ms,
            end_ms: m.end_ms,
            seat: m.hold_seat,
        }
    }

    fn toward(dir: i8) -> &'static str {
        if dir > 0 {
            "the high stop"
        } else {
            "the low stop"
        }
    }

    /// The finished anchor's record into the session's, its hold fitted
    /// alone and pooled, and the stop rule.
    fn end_hold(&mut self) {
        let (v, a) = (self.cfg.v_per_count, self.cfg.a_per_count);
        self.polls.extend_from_slice(self.anchor.polls());
        self.samples.extend_from_slice(self.anchor.samples());
        self.holds_run += 1;
        let n = self.holds_run;
        let marks = self.anchor.marks();
        if let Some(m) = marks {
            self.marks.push(m);
        }
        let refused = self.anchor.refused().map(str::to_string);
        let fitted = match (&refused, marks) {
            (Some(why), _) => Err(why.clone()),
            (None, None) => Err("the hold never seated".to_string()),
            (None, Some(m)) => {
                let hold = self.hold_of(&m);
                thermal::fit_rotor_pinned(&[hold], &self.cfg.body, v, a)
                    .map(|single| (single, m))
                    .map_err(|why| match self.anchor.ended_early() {
                        Some(ended) => format!("{ended}; {why}"),
                        None => why,
                    })
            }
        };
        let mut line = format!("hold {n} toward {}: ", Self::toward(self.dir));
        match fitted {
            Ok((single, m)) => {
                line.push_str(&format!(
                    "seat {}, rotor {} q016 from {:.0} s in",
                    m.hold_seat,
                    single.rotor.rise_q016.raw,
                    single.holds[0].fit_from_ms / 1000.0
                ));
                self.accepted.push(m);
                self.singles.push(single);
                self.dirs.push(self.dir);
                self.refused_run = 0;
                let holds: Vec<Hold<'_>> = self.accepted.iter().map(|m| self.hold_of(m)).collect();
                match thermal::fit_rotor_pinned(&holds, &self.cfg.body, v, a) {
                    Ok(p) => {
                        self.pooled = Some(p);
                        self.pool_failed = None;
                    }
                    Err(why) => self.pool_failed = Some(why),
                }
                self.spread =
                    thermal::pool_spread(&self.singles, self.cfg.ceiling_counts, &self.dirs);
                line.push_str(&self.pool_text());
            }
            Err(why) => {
                line.push_str(&format!("dropped, {why}"));
                self.dropped.push((n, why));
                self.refused_run = if refused.is_some() {
                    self.refused_run + 1
                } else {
                    0
                };
            }
        }
        self.log.push(line);
        self.judge();
    }

    /// The pool as it stands, for the hold's line.
    fn pool_text(&self) -> String {
        let n = self.accepted.len();
        match (&self.pool_failed, &self.pooled, &self.spread) {
            (Some(why), _, _) => format!("; pool of {n} fails: {why}"),
            (None, None, _) => String::new(),
            (None, Some(p), None) => format!(
                "; pool of {n}: {} q016, the spread needs {} holds",
                p.rotor.rise_q016.raw,
                thermal::SPREAD_MIN_HOLDS
            ),
            (None, Some(p), Some(s)) => {
                let split = match s.split_c {
                    Some(c) => format!("split {c:+.1} C"),
                    None => "no split yet".to_string(),
                };
                format!(
                    "; pool of {n}: {} +/- {:.0}, se {:.1} C at the ceiling, {split}",
                    p.rotor.rise_q016.raw,
                    s.k_sd * 65536.0,
                    s.se_c_at_ceiling
                )
            }
        }
    }

    /// The stop rule after a hold: never decided on a step whose pooled
    /// fit failed.
    fn judge(&mut self) {
        let n = self.holds_run;
        let se = self.spread.as_ref().map(|s| s.se_c_at_ceiling);
        if self.accepted.len() >= self.cfg.holds_min
            && self.pool_failed.is_none()
            && let Some(se) = se
            && se <= self.cfg.within_c
        {
            self.decided = true;
            self.verdict = Some(format!(
                "decided after {n} holds: the pool's se {se:.2} C at the ceiling sits inside {:.2} C",
                self.cfg.within_c
            ));
        } else if self.refused_run >= 2 {
            self.verdict = Some("two holds in a row refused".to_string());
        } else if n >= self.cfg.holds_max {
            self.verdict = Some(match se {
                Some(se) => format!(
                    "undecided after {n} holds: the pool's se {se:.2} C at the ceiling sits over \
                     {:.2} C",
                    self.cfg.within_c
                ),
                None => format!(
                    "undecided after {n} holds: {} accepted, under the {} the spread needs",
                    self.accepted.len(),
                    thermal::SPREAD_MIN_HOLDS
                ),
            });
        }
    }

    fn next_hold(&mut self) {
        self.dir = -self.dir;
        self.anchor = Anchor::new(
            AnchorCfg {
                dir: self.dir,
                ..self.cfg.anchor
            },
            &self.params,
        );
        self.phase = Phase::Hold;
    }
}

impl Experiment for ShortHolds {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::Hold => {
                let cmd = self.anchor.step(obs);
                if let Some(reason) = self.anchor.halted() {
                    self.halt = Some(reason);
                    return cmd;
                }
                if cmd != Cmd::Done {
                    return cmd;
                }
                self.end_hold();
                if self.verdict.is_some() {
                    self.phase = Phase::Finished;
                    return Cmd::Done;
                }
                // the rest's first poll lands at once: the replay holds the
                // last poll's current until the next one
                self.rest_end_ms =
                    self.polls.last().map_or(0.0, |p| p.t_ms) + self.cfg.rest_ms as f64;
                self.phase = Phase::RestEval;
                Cmd::Read
            }
            Phase::RestRead => {
                self.phase = Phase::RestEval;
                Cmd::Read
            }
            Phase::RestEval => {
                let Some(o) = obs else {
                    return Cmd::Read;
                };
                self.polls.push(poll(o));
                if o.host_ms >= self.rest_end_ms {
                    self.next_hold();
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::RestRead;
                    Cmd::Pause {
                        ms: self.cfg.rest_poll_ms,
                    }
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }

    fn push_watch(&mut self, w: &Watch) {
        if matches!(self.phase, Phase::Hold) {
            self.anchor.push_watch(w);
        }
    }

    fn halted(&self) -> Option<AbortReason> {
        self.halt.or_else(|| self.anchor.halted())
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeHeat, FakeServo, pump, rig};
    use super::super::{Guarded, Permitted, seek};
    use super::*;

    /// Board D's gains, q016, rotor on 33 s and body on 262 s.
    const Q: [u16; 2] = [719, 777];
    /// How close a single hold's rotor reads to its planted gain, and the
    /// pool's mean: the fake heats on its true current while the replay
    /// heats on whole-count polls, and the R_ref line reads through the
    /// window dither, which together cost it a percent or two.
    const SINGLE_WITHIN: f64 = 0.03;
    const MEAN_WITHIN: f64 = 0.02;
    const TAUS: [u16; 2] = [33, 262];
    const V_PER_COUNT: f64 = 4.0e-3;
    const A_PER_COUNT: f64 = 0.9e-3;
    /// The fake's ceiling, counts: the hold draws 231, 17 % under it.
    const CEILING: u16 = 280;
    /// The winding's R, vcounts per ccount: the default hold duty draws 231
    /// counts, where board D's gains warm the copper 12 C and the current
    /// reported in whole counts resolves the rise.
    const R: f64 = 1.2;

    /// The body node the long hold wrote, as the short holds take it.
    fn pinned_body(raw: u16) -> NodeFit {
        NodeFit::pinned(raw, TAUS[1])
    }

    /// A winding warming by board D's gains under the ceiling.
    fn servo() -> FakeServo {
        let mut s = FakeServo::new(R);
        s.current_limit = Some(CEILING);
        s.heat = Some(FakeHeat::new(Q, TAUS));
        s.i_dither = true;
        s
    }

    fn cfg(holds_min: usize, holds_max: usize, within_c: f64) -> SessionCfg {
        SessionCfg {
            anchor: AnchorCfg {
                hold_ms: 120_000,
                ..AnchorCfg::default()
            },
            body: pinned_body(Q[1]),
            v_per_count: V_PER_COUNT,
            a_per_count: A_PER_COUNT,
            ceiling_counts: CEILING as f64,
            within_c,
            holds_min,
            holds_max,
            rest_ms: 10_000,
            rest_poll_ms: 1000,
        }
    }

    fn run(
        servo: &mut FakeServo,
        cfg: SessionCfg,
    ) -> (ShortHolds, Vec<String>, Option<AbortReason>) {
        let params = rig().without_pos_guard();
        let mut exp = Guarded::new(Permitted::new(ShortHolds::new(cfg, &params)), params);
        let log = pump(&mut exp, servo, 4_000_000);
        let abort = exp.abort();
        (exp.into_inner().into_inner(), log, abort)
    }

    /// The hold's safe tail: duty 0 then torque off, right after it. Every
    /// hold ends so, and the run ends with the permit withdrawn too.
    fn assert_safe_tails(log: &[String], holds: usize) {
        let ends = log
            .windows(2)
            .filter(|w| w[0] == "write goal_duty 0" && w[1] == "write torque_enable 0")
            .count();
        assert_eq!(ends, holds, "{log:?}");
        let tail: Vec<&String> = log.iter().rev().take(3).collect();
        assert_eq!(*tail[2], "write goal_duty 0");
        assert_eq!(*tail[1], "write torque_enable 0");
        assert_eq!(*tail[0], "write stall_permit 0");
    }

    /// The drives between consecutive torque enables, in order: each hold's
    /// seek and hold duties.
    fn drives_per_hold(log: &[String]) -> Vec<Vec<i32>> {
        let mut holds: Vec<Vec<i32>> = Vec::new();
        for l in log {
            if l == "write torque_enable 1" {
                holds.push(Vec::new());
            } else if let Some(v) = l.strip_prefix("write goal_duty ")
                && let Some(h) = holds.last_mut()
            {
                let v: i32 = v.parse().unwrap();
                if v != 0 {
                    h.push(v);
                }
            }
        }
        holds
    }

    fn near_stop(seat: u16, dir: i8) -> bool {
        let stop = seek::stop_for(dir, (200, 4000));
        seat.abs_diff(stop) <= seek::STOP_TOL
    }

    /// Three holds at one gain: low stop, high stop, low stop, the pool of
    /// three deciding on identical seats, the pooled rotor the planted one,
    /// one log line per hold, every hold ending safe.
    #[test]
    fn holds_alternate_between_the_stops_and_pool() {
        let mut servo = servo();
        let (exp, log, abort) = run(&mut servo, cfg(3, 6, 1.0));
        assert_eq!(abort, None);
        assert_eq!(exp.marks().len(), 3, "{:?}", exp.log_lines());
        for (m, dir) in exp.marks().iter().zip([-1, 1, -1]) {
            assert_eq!(m.dir, dir);
            assert!(near_stop(m.seat, dir), "{m:?}");
            assert!(near_stop(m.hold_seat, dir), "{m:?}");
        }
        for (drives, dir) in drives_per_hold(&log).iter().zip([-1, 1, -1]) {
            assert!(!drives.is_empty());
            assert!(drives.iter().all(|d| d.signum() == dir), "{drives:?}");
        }
        let f = exp.result().expect("a session fit");
        assert!(f.decided, "{:?}", exp.verdict());
        assert!(exp.verdict().unwrap().starts_with("decided after 3 holds"));
        let planted = Q[0] as f64;
        let rotor = f.pooled.rotor.rise_q016.raw as f64;
        assert!(
            (rotor / planted - 1.0).abs() < 0.02,
            "rotor {rotor} q016 against {planted}: {:?}",
            f.pooled
        );
        assert_eq!(f.pooled.body, pinned_body(Q[1]));
        assert_eq!(f.singles.len(), 3);
        assert_eq!(f.accepted, f.marks);
        for s in &f.singles {
            let off = (s.rotor.rise_q016.raw as f64 / planted - 1.0) * 100.0;
            assert!(
                off.abs() < SINGLE_WITHIN * 100.0,
                "a single read {off:+.1} % off the planted, -0.8 % cold and about -2.2 % on a \
                 warm start on this fake: {s:?}"
            );
        }
        let spread = f.spread.as_ref().expect("a spread");
        assert_eq!(spread.n, 3);
        assert!(spread.split_c.is_some(), "{spread:?}");
        assert!(spread.se_c_at_ceiling <= 1.0, "{spread:?}");
        assert!(f.dropped.is_empty(), "{:?}", f.dropped);
        let cross = f.cross_check.as_ref().expect("the free fit");
        assert_eq!(cross.holds.len(), 3);
        assert_eq!(exp.log_lines().len(), 3, "{:?}", exp.log_lines());
        for (k, l) in exp.log_lines().iter().enumerate() {
            assert!(l.starts_with(&format!("hold {} toward ", k + 1)), "{l}");
            assert!(l.contains("q016 from 20 s in"), "{l}");
        }
        assert!(
            exp.log_lines()[2].contains("pool of 3:"),
            "{}",
            exp.log_lines()[2]
        );
        assert_safe_tails(&log, 3);
    }

    /// A rotor gain that differs seat to seat by +/-5 %: three identical
    /// seats never come, the pool's error stays over a tight bound and the
    /// session runs out its holds undecided, the spread reported.
    #[test]
    fn the_pool_keeps_going_while_the_seats_disagree() {
        let mut servo = servo();
        servo.heat.as_mut().unwrap().rotor_q_per_enable = vec![683, 755, 683, 755];
        let (exp, log, abort) = run(&mut servo, cfg(3, 4, 0.05));
        assert_eq!(abort, None);
        assert_eq!(exp.marks().len(), 4, "{:?}", exp.log_lines());
        let f = exp.result().expect("a session fit");
        assert!(!f.decided);
        assert!(
            exp.verdict()
                .unwrap()
                .starts_with("undecided after 4 holds"),
            "{:?}",
            exp.verdict()
        );
        assert_eq!(f.singles.len(), 4);
        let spread = f.spread.as_ref().expect("a spread");
        assert_eq!(spread.n, 4);
        assert!(spread.se_c_at_ceiling > 0.05, "{spread:?}");
        let planted = Q[0] as f64 / 65536.0;
        assert!(
            (spread.k_sd / planted - 0.05).abs() < 0.01,
            "sd {:.4} of the mean against 5 %: {spread:?}",
            spread.k_sd / planted
        );
        assert!(
            (spread.k_mean / planted - 1.0).abs() < MEAN_WITHIN,
            "the mean read {:+.1} % off the planted mean, about -1.7 % on this fake: {spread:?}",
            (spread.k_mean / planted - 1.0) * 100.0
        );
        let split = spread.split_c.expect("both stops held");
        assert!(
            split > 0.0,
            "the high stop's seats carry the higher gain: {spread:?}"
        );
        assert_safe_tails(&log, 4);
    }

    /// Seats +/-5 % apart under one shared gain: after the third hold the
    /// pooled fit refuses itself (hold 2's early residual reads as an
    /// unsettled contact) while every single passed. The session ends on
    /// that step with the two-hold pool kept, the failure named and no
    /// decision.
    #[test]
    fn a_late_pool_failure_keeps_the_last_good_pool() {
        let mut servo = servo();
        servo.heat.as_mut().unwrap().rotor_q_per_enable = vec![683, 755, 683];
        let (exp, log, abort) = run(&mut servo, cfg(3, 3, 1.0));
        assert_eq!(abort, None);
        let f = exp.result().expect("the two-hold pool stands");
        assert_eq!(f.singles.len(), 3);
        assert_eq!(f.pooled.holds.len(), 2, "{:?}", exp.log_lines());
        let why = f.pool_failed.as_deref().expect("the pool of three failed");
        assert!(why.contains("the contact had not settled"), "{why}");
        assert!(!f.decided);
        assert!(
            exp.verdict()
                .unwrap()
                .starts_with("undecided after 3 holds"),
            "{:?}",
            exp.verdict()
        );
        assert!(
            exp.log_lines()[2].contains("pool of 3 fails: "),
            "{}",
            exp.log_lines()[2]
        );
        assert_safe_tails(&log, 3);
    }

    /// The second hold's cold current crowds a lowered ceiling: it is
    /// refused once its floor is read and dropped with that reason, the
    /// third and fourth holds run and the pool decides on the other three.
    #[test]
    fn a_refused_hold_is_dropped_not_fatal() {
        let mut servo = servo();
        // the hold draws 231 counts, 7 % under 248
        servo.limit_per_enable = vec![None, Some(248)];
        let (exp, log, abort) = run(&mut servo, cfg(3, 6, 1.0));
        assert_eq!(abort, None);
        let f = exp.result().expect("a session fit");
        assert!(f.decided, "{:?}", exp.verdict());
        assert_eq!(f.singles.len(), 3);
        assert_eq!(f.marks.len(), 4);
        assert_eq!(f.accepted.len(), 3);
        assert!(f.accepted.iter().all(|m| f.marks.contains(m)));
        assert_eq!(f.dropped.len(), 1, "{:?}", f.dropped);
        let (n, why) = &f.dropped[0];
        assert_eq!(*n, 2);
        assert!(why.contains("inside the 10 % margin"), "{why}");
        assert!(why.contains("248 counts current ceiling"), "{why}");
        assert!(
            exp.log_lines()[1].contains("dropped, "),
            "{}",
            exp.log_lines()[1]
        );
        let planted = Q[0] as f64;
        assert!(
            (f.pooled.rotor.rise_q016.raw as f64 / planted - 1.0).abs() < 0.02,
            "{:?}",
            f.pooled
        );
        assert_safe_tails(&log, 4);
    }

    /// Two refused holds in a row end the session undecided, with the one
    /// hold before them fitted alone and no spread.
    #[test]
    fn two_refusals_in_a_row_end_the_session() {
        let mut servo = servo();
        servo.limit_per_enable = vec![None, Some(248), Some(248)];
        let (exp, log, abort) = run(&mut servo, cfg(3, 6, 1.0));
        assert_eq!(abort, None);
        assert_eq!(exp.verdict(), Some("two holds in a row refused"));
        let f = exp.result().expect("the first hold's fit");
        assert!(!f.decided);
        assert_eq!(f.singles.len(), 1);
        assert_eq!(f.spread, None);
        assert_eq!(f.dropped.iter().map(|d| d.0).collect::<Vec<_>>(), [2, 3]);
        assert_eq!(exp.log_lines().len(), 3);
        assert_safe_tails(&log, 3);
    }

    /// Nothing accepted: the result names every hold's reason.
    #[test]
    fn a_session_with_no_accepted_hold_names_every_reason() {
        let mut servo = servo();
        servo.limit_per_enable = vec![Some(248), Some(248)];
        let (exp, _, abort) = run(&mut servo, cfg(3, 6, 1.0));
        assert_eq!(abort, None);
        let why = exp.result().unwrap_err();
        assert!(why.starts_with("no hold was accepted: hold 1: "), "{why}");
        assert!(why.contains("; hold 2: "), "{why}");
    }

    /// One record on one clock: the polls climb through every hold and
    /// rest, and each hold starts after the one before ended and rested.
    #[test]
    fn the_session_record_is_one_clock() {
        let mut servo = servo();
        let cfg = cfg(3, 6, 1.0);
        let rest = cfg.rest_ms as f64;
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let polls = exp.polls();
        // a poll is its 20 ms pause and the bus: about 4400 per hold
        assert!(polls.len() > 3 * 4000, "{} polls", polls.len());
        assert!(
            polls.windows(2).all(|p| p[1].t_ms > p[0].t_ms),
            "a poll out of order"
        );
        let marks = exp.marks();
        assert_eq!(marks.len(), 3);
        for pair in marks.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            assert!(b.t0_ms >= a.end_ms + rest, "{a:?} then {b:?}");
            // the rest's polls carry no current and sit a second apart
            let resting: Vec<&HoldPoll> = polls
                .iter()
                .filter(|p| p.t_ms > a.end_ms && p.t_ms < a.end_ms + rest)
                .collect();
            assert!(resting.len() >= 9, "{} rest polls", resting.len());
            assert!(resting.iter().all(|p| p.i_counts == 0.0));
        }
        // the first hold's polls start with its seek
        assert!(polls[0].t_ms < marks[0].t0_ms);
        let f = exp.result().unwrap();
        assert_eq!(f.marks, marks);
    }

    #[test]
    fn the_plan_counts_a_seek_a_hold_and_a_rest_per_hold() {
        let cfg = cfg(3, 6, 1.0);
        assert_eq!(planned_ms(&cfg, 3), 3.0 * (5000.0 + 120_000.0 + 10_000.0));
    }
}
