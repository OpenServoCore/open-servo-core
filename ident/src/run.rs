//! The order `osc ident run` takes its stages in, and when it stops. The
//! driver runs whatever [`Run::next_stage`] names and reports how it ended; the
//! first stage that aborts ends the run, whatever the reason: a blocked
//! shaft stays blocked for every stage after it.

use crate::exp::AbortReason;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Stage {
    Bias,
    Burst,
    /// Only when the burst declined: the winding R from stop stalls.
    Resistance,
    Breakaway,
    Ladder,
    Inertia,
}

impl Stage {
    pub fn name(self) -> &'static str {
        match self {
            Stage::Bias => "bias",
            Stage::Burst => "burst",
            Stage::Resistance => "resistance",
            Stage::Breakaway => "breakaway",
            Stage::Ladder => "ladder",
            Stage::Inertia => "inertia",
        }
    }
}

/// How a stage ended.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Ended {
    Done,
    /// Ran, but its result does not feed the gains: the burst's routes
    /// both declined, or it produced nothing to fit.
    Declined,
    Aborted(AbortReason),
}

#[derive(Clone, Debug, Default)]
pub struct Run {
    at: Option<Stage>,
    ended: Option<Ended>,
    aborted: Option<(Stage, AbortReason)>,
}

impl Run {
    pub fn new() -> Self {
        Self::default()
    }

    /// The stage to run now; None once the run is over.
    pub fn next_stage(&mut self) -> Option<Stage> {
        if self.aborted.is_some() {
            return None;
        }
        let next = match (self.at, self.ended) {
            (None, _) => Some(Stage::Bias),
            (Some(Stage::Bias), _) => Some(Stage::Burst),
            (Some(Stage::Burst), Some(Ended::Declined)) => Some(Stage::Resistance),
            (Some(Stage::Burst | Stage::Resistance), _) => Some(Stage::Breakaway),
            (Some(Stage::Breakaway), _) => Some(Stage::Ladder),
            (Some(Stage::Ladder), _) => Some(Stage::Inertia),
            (Some(Stage::Inertia), _) => None,
        };
        if next.is_some() {
            self.at = next;
            self.ended = None;
        }
        next
    }

    /// Report how the stage [`Run::next_stage`] last named ended.
    pub fn ended(&mut self, how: Ended) {
        self.ended = Some(how);
        if let (Ended::Aborted(reason), Some(stage)) = (how, self.at) {
            self.aborted = Some((stage, reason));
        }
    }

    /// The stage that ended the run early, and why.
    pub fn aborted(&self) -> Option<(Stage, AbortReason)> {
        self.aborted
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exp::bias::{Bias, BiasCfg};
    use crate::exp::breakaway::{Breakaway, BreakawayCfg};
    use crate::exp::held::{Held, HeldCfg};
    use crate::exp::inductance::{Cfg as InductanceCfg, Inductance, fit_captures};
    use crate::exp::inertia::{Inertia, InertiaCfg};
    use crate::exp::ladder::{Ladder, LadderCfg};
    use crate::exp::resistance::{Resistance, ResistanceCfg};
    use crate::exp::rl::Scales;
    use crate::exp::testkit::{FakeServo, pump, rig};
    use crate::exp::{Experiment, Guarded, Permitted};
    use crate::units::SenseParams;

    fn stages(run: &mut Run, how: impl Fn(Stage) -> Ended) -> Vec<Stage> {
        let mut seen = Vec::new();
        while let Some(s) = run.next_stage() {
            seen.push(s);
            run.ended(how(s));
        }
        seen
    }

    #[test]
    fn run_all_stops_at_the_first_abort() {
        use Stage::*;
        let blocked = AbortReason::Blocked {
            pos: 2048,
            moved: 0,
        };
        let full = stages(&mut Run::new(), |s| match s {
            Burst => Ended::Declined,
            _ => Ended::Done,
        });
        assert_eq!(full, [Bias, Burst, Resistance, Breakaway, Ladder, Inertia]);
        let promoted = stages(&mut Run::new(), |_| Ended::Done);
        assert_eq!(promoted, [Bias, Burst, Breakaway, Ladder, Inertia]);
        for (k, stage) in full.iter().enumerate() {
            let mut run = Run::new();
            let seen = stages(&mut run, |s| match s {
                s if s == *stage => Ended::Aborted(blocked),
                Burst => Ended::Declined,
                _ => Ended::Done,
            });
            assert_eq!(seen, full[..=k], "aborted in {}", stage.name());
            assert_eq!(run.aborted(), Some((*stage, blocked)));
            assert_eq!(run.next_stage(), None, "the run stays over");
        }
    }

    fn scales() -> Scales {
        let s = SenseParams {
            shunt_r_mohm: 60,
            gain_milli: 15_000,
            vmotor_div_top: 6_800,
            vmotor_div_bot: 3_300,
            vdd_mv: 3_300,
            tick_hz: 20_100,
        };
        Scales::from_sense(&s, 15_000, 10_000).unwrap()
    }

    /// One stage the way the CLI runs it, against the fake; the command log
    /// is appended to `log`.
    fn execute(stage: Stage, servo: &mut FakeServo, log: &mut Vec<String>) -> Ended {
        fn go<E: Experiment>(
            exp: E,
            params: crate::exp::RigParams,
            servo: &mut FakeServo,
            log: &mut Vec<String>,
        ) -> (Guarded<E>, Ended) {
            let mut g = Guarded::new(exp, params);
            log.extend(pump(&mut g, servo, 2_000_000));
            let how = g.abort().map_or(Ended::Done, Ended::Aborted);
            (g, how)
        }
        let params = rig();
        match stage {
            Stage::Bias => go(Bias::new(BiasCfg::default(), &params), params, servo, log).1,
            Stage::Burst => {
                let held = Held::new(HeldCfg::default(), &params, scales());
                let (held, how) = go(held, params.without_pos_guard(), servo, log);
                if how != Ended::Done {
                    return how;
                }
                let caps = held.into_inner().captures().to_vec();
                if fit_captures(&caps, &scales(), &Default::default())
                    .is_some_and(|r| r.held.promotable())
                {
                    return Ended::Done;
                }
                let free = Inductance::new(InductanceCfg::default(), &params, scales());
                match go(free, params, servo, log) {
                    (_, Ended::Done) => Ended::Declined,
                    (_, how) => how,
                }
            }
            Stage::Resistance => {
                let p = params.without_pos_guard();
                let exp = Permitted::new(Resistance::new(ResistanceCfg::default(), &p));
                go(exp, p, servo, log).1
            }
            Stage::Breakaway => go(Breakaway::new(BreakawayCfg::default()), params, servo, log).1,
            Stage::Ladder => {
                go(
                    Ladder::new(LadderCfg::default(), &params),
                    params,
                    servo,
                    log,
                )
                .1
            }
            Stage::Inertia => {
                go(
                    Inertia::new(InertiaCfg::default(), &params),
                    params,
                    servo,
                    log,
                )
                .1
            }
        }
    }

    /// The bench run on a shaft locked at mid travel, replayed: the first
    /// seek comes to rest where it started and the run ends there. No stall
    /// dwell, no ladder rung and no burst is ever commanded after it.
    #[test]
    fn a_mid_travel_jam_ends_the_run_at_the_first_seek() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        servo.soft = Some((230.0, 3970.0));
        servo.lease_ms = Some(1008.0);
        servo.pos = 2048.0;
        servo.jam = Some(2048.0);
        let mut run = Run::new();
        let mut log = Vec::new();
        let mut ran = Vec::new();
        while let Some(stage) = run.next_stage() {
            ran.push(stage);
            run.ended(execute(stage, &mut servo, &mut log));
        }
        assert_eq!(ran, [Stage::Bias, Stage::Burst]);
        assert_eq!(
            run.aborted(),
            Some((
                Stage::Burst,
                AbortReason::Blocked {
                    pos: 2048,
                    moved: 0
                }
            ))
        );
        let hold = (HeldCfg::default().hold_pct as i32) * 32767 / 100;
        let first_seek = log
            .iter()
            .position(|l| *l == format!("write goal_duty {}", -hold))
            .expect("the seek");
        let after = &log[first_seek + 1..];
        assert!(
            after
                .iter()
                .filter_map(|l| l.strip_prefix("write goal_duty "))
                .all(|v| v == "0"),
            "a drive after the failed seek: {after:?}"
        );
        assert!(after.iter().all(|l| !l.starts_with("burst")), "{after:?}");
        assert_eq!(log.last().map(String::as_str), Some("write stall_permit 0"));
        assert!(!servo.torque && !servo.permit_live());
    }
}
