//! The order `osc ident run` takes its stages in, the duty each one drives
//! at, and when the run stops. The driver runs whatever [`Run::next_stage`]
//! names, feeds back what it measured, and reports how the stage ended; the
//! first stage that aborts ends the run - a blocked shaft stays blocked for
//! every stage after it.
//!
//! The jam check comes first: it proves the shaft free, raising its duty
//! while nothing moves, and the duty that moved it seeds every later seek.
//! Winding R and L come next, from free-shaft bursts at mid travel inside
//! [`BurstAllowance`]: a burst is too short to load the train and too short
//! for the limiter to see, so it is bounded by the volts it applies, not by
//! stall current. Once R is known, every drive that could stall - seeks,
//! centring, the breakaway ramp - is planned from R and the rail so its
//! stall stays at or under the current limit ([`DutyPlan`]); the ladder and
//! inertia rungs run free on the firmware limiter, the stall timer and the
//! travel guard and the [`crate::runway`], inertia stepping from a moving
//! base by what the limit leaves over the base's running current. A stage
//! that declines once there is a plan still ends with the closing
//! centring. Nothing in the default run stalls a stop: a burst that
//! declines ends it. Asked for ([`Run::with_stall_ladder`]), the resistance
//! stop ladder measures R instead, its dwells stalling between the current
//! sensor's window floor and the limit ([`DutyPlan::stall_ladder`]) with
//! the permit held; a supply that leaves no room between the two ends the
//! run before it moves.

use crate::exp::AbortReason;
use crate::exp::breakaway::BreakawayCfg;
use crate::exp::centre::CentreCfg;
use crate::exp::inductance::Cfg as BurstCfg;
use crate::exp::inertia::{BASE_OVER_SEEK, InertiaCfg};
use crate::exp::ladder::LadderCfg;
use crate::exp::resistance::ResistanceCfg;
use crate::exp::rl::Scales;
use crate::limits::{
    BurstAllowance, CLASS_R_MIN, DutyPlan, NUDGE_MAX_MV, Refusal, ServoLimits, duty_of_mv,
    pct_floor, q15_floor,
};

const Q15: f64 = 32767.0;

#[derive(Clone, Debug, PartialEq)]
pub enum Stage {
    Bias,
    /// Into the band at mid travel from `duty`, raising up to `cap` while
    /// nothing moves; `nudge` also drives the shaft out and back once it is
    /// there, the jam check.
    Centre {
        duty: f64,
        cap: f64,
        nudge: bool,
    },
    /// Free-shaft bursts from rest at mid travel at the two `rungs`, the
    /// from-a-hold control stepping from `pre`; `seek` brings the shaft back
    /// to the band between arms.
    Burst {
        rungs: [f64; 2],
        pre: f64,
        seek: f64,
    },
    /// The resistance stop ladder, on request once the burst declined:
    /// `seek` to each stop, then dwell at each of the `rungs`.
    Resistance {
        seek: f64,
        rungs: Vec<f64>,
    },
    Breakaway {
        cap: f64,
    },
    Ladder {
        seek: f64,
        rungs: Vec<f64>,
    },
    /// Steps from a moving `base`, sized once its running current is
    /// read ([`InertiaCfg::step_duties`]).
    Inertia {
        seek: f64,
        base: f64,
    },
}

impl Stage {
    pub fn name(&self) -> &'static str {
        match self {
            Stage::Bias => "bias",
            Stage::Centre { .. } => "centring",
            Stage::Burst { .. } => "burst",
            Stage::Resistance { .. } => "resistance",
            Stage::Breakaway { .. } => "breakaway",
            Stage::Ladder { .. } => "ladder",
            Stage::Inertia { .. } => "inertia",
        }
    }
}

/// How a stage ended.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Ended {
    Done,
    /// Ran, but its result does not feed the gains: the burst's routes
    /// declined, the ladder fitted too few rungs, or it produced nothing to
    /// fit. Nothing after it can be planned; a run with a plan still ends
    /// with its closing centring.
    Declined,
    Aborted(AbortReason),
}

/// Why the run ended before its last stage.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Over {
    Aborted(&'static str, AbortReason),
    /// The named stage gave nothing to fit: nothing after it can be
    /// planned. The burst's decline leaves no winding R.
    Declined(&'static str),
    /// The resistance stop ladder, asked for, fitted no R either.
    ResistanceDeclined,
    /// The resistance stop ladder, asked for, has no room between the
    /// window floor and the stall-safe cap: duties, fractions of full
    /// scale.
    NoLadderRoom {
        floor: f64,
        cap: f64,
    },
    /// The jam check never proved the shaft free, so no burst may arm.
    Unproven,
    /// The rail is too high for the burst's two rungs to span the fit's
    /// pair inside the volts cap.
    RailTooHigh {
        rail_mv: f64,
    },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Step {
    Bias,
    Nudge,
    Burst,
    Resistance,
    Breakaway,
    Ladder,
    Inertia,
    Park,
}

/// Every planned stage starts from rest at mid travel, and the run ends
/// there.
const ORDER: [Step; 10] = [
    Step::Bias,
    Step::Nudge,
    Step::Burst,
    Step::Park,
    Step::Breakaway,
    Step::Park,
    Step::Ladder,
    Step::Park,
    Step::Inertia,
    Step::Park,
];

#[derive(Clone, Debug)]
pub struct Run {
    lim: ServoLimits,
    rail_mv: f64,
    bootstrap: f64,
    class_r_vpc: f64,
    stall_ladder: bool,
    /// A step the run inserted ahead of the order.
    pending: Option<Step>,
    next: usize,
    free: Option<f64>,
    plan: Option<DutyPlan>,
    at: Option<&'static str>,
    over: Option<Over>,
    /// Why the run ends once its closing centring is done.
    cut: Option<Over>,
}

impl Run {
    /// `sc` turns the class's [`CLASS_R_MIN`] and the rail into the
    /// servo's own units.
    pub fn new(lim: ServoLimits, sc: &Scales) -> Self {
        Self {
            bootstrap: lim.bootstrap_duty(sc.r_vpc(CLASS_R_MIN)),
            rail_mv: lim.vbus as f64 * sc.v_term_per_count * 1000.0,
            class_r_vpc: sc.r_vpc(CLASS_R_MIN),
            stall_ladder: false,
            pending: None,
            lim,
            next: 0,
            free: None,
            plan: None,
            at: None,
            over: None,
            cut: None,
        }
    }

    /// A burst that declines hands over to the resistance stop ladder
    /// instead of ending the run.
    pub fn with_stall_ladder(self) -> Self {
        Self {
            stall_ladder: true,
            ..self
        }
    }

    /// The class-safe duty every drive starts from until R is measured.
    pub fn bootstrap(&self) -> f64 {
        self.bootstrap
    }

    /// The most the jam check raises to: [`NUDGE_MAX_MV`] on this rail.
    pub fn nudge_cap(&self) -> f64 {
        duty_of_mv(NUDGE_MAX_MV, self.rail_mv).max(self.bootstrap)
    }

    pub fn rail_mv(&self) -> f64 {
        self.rail_mv
    }

    /// The stage to run now; None once the run is over.
    pub fn next_stage(&mut self) -> Option<Stage> {
        if self.over.is_some() {
            return None;
        }
        let step = match self.pending.take() {
            Some(s) => s,
            None => {
                let Some(&s) = ORDER.get(self.next) else {
                    self.over = self.cut.take();
                    return None;
                };
                self.next += 1;
                s
            }
        };
        let boot = self.bootstrap;
        let stage = match (step, self.plan) {
            (Step::Bias, _) => Stage::Bias,
            (Step::Nudge, _) => Stage::Centre {
                duty: boot,
                cap: self.nudge_cap(),
                nudge: true,
            },
            (Step::Burst, _) => {
                let Some(moved) = self.free else {
                    return self.end(Over::Unproven);
                };
                match BurstAllowance::rungs(self.rail_mv) {
                    Some(rungs) => Stage::Burst {
                        rungs,
                        pre: boot,
                        seek: moved,
                    },
                    None if self.stall_ladder => return self.stop_ladder(),
                    None => {
                        let rail_mv = self.rail_mv;
                        return self.end(Over::RailTooHigh { rail_mv });
                    }
                }
            }
            (Step::Resistance, _) => return self.stop_ladder(),
            (_, None) => return self.end(Over::Declined("burst")),
            (Step::Breakaway, Some(p)) => Stage::Breakaway { cap: p.stop_cap },
            (Step::Ladder, Some(p)) => Stage::Ladder {
                seek: p.seek,
                rungs: fractions(&LadderCfg::default().rungs_q15),
            },
            (Step::Inertia, Some(p)) => Stage::Inertia {
                seek: p.seek,
                base: p.seek + BASE_OVER_SEEK,
            },
            (Step::Park, Some(p)) => Stage::Centre {
                duty: p.seek,
                cap: p.stop_cap,
                nudge: false,
            },
        };
        self.at = Some(stage.name());
        Some(stage)
    }

    /// The stop ladder, planned from what R the servo carries, else the
    /// class's lowest: the burst in this run measured none.
    fn stop_ladder(&mut self) -> Option<Stage> {
        let plan = self.lim.stall_plan(self.class_r_vpc, self.free);
        match plan.stall_ladder(self.lim.window_floor()) {
            Ok(rungs) => {
                self.at = Some("resistance");
                Some(Stage::Resistance {
                    seek: plan.seek,
                    rungs,
                })
            }
            Err(Refusal::NoLadderRoom { floor, cap }) => {
                self.end(Over::NoLadderRoom { floor, cap })
            }
            Err(_) => self.end(Over::ResistanceDeclined),
        }
    }

    fn end(&mut self, why: Over) -> Option<Stage> {
        self.over = Some(why);
        None
    }

    /// Report how the stage [`Run::next_stage`] last named ended.
    pub fn ended(&mut self, how: Ended) {
        match how {
            Ended::Done => {}
            Ended::Declined => match self.at {
                Some("burst") if self.stall_ladder => self.pending = Some(Step::Resistance),
                Some("resistance") => self.over = Some(Over::ResistanceDeclined),
                Some(stage) if self.plan.is_some() && stage != "burst" => {
                    self.cut = Some(Over::Declined(stage));
                    self.finish();
                }
                at => self.over = Some(Over::Declined(at.unwrap_or("run"))),
            },
            Ended::Aborted(reason) => {
                self.over = Some(Over::Aborted(self.at.unwrap_or("run"), reason))
            }
        }
    }

    /// The jam check's result: the duty the shaft first travelled at, None
    /// when it never did.
    pub fn nudged(&mut self, moved_at: Option<f64>) {
        self.free = moved_at;
    }

    /// The winding R the burst, or the stop ladder, measured: every later
    /// stall-safe duty plans from it.
    pub fn measured(&mut self, r_vpc: f64) {
        self.plan = Some(DutyPlan::new(&self.lim, r_vpc, self.free));
    }

    /// The larger of the two breakaway duties, when either direction moved.
    pub fn broke_away(&mut self, duty: Option<f64>) {
        if let (Some(p), Some(d)) = (self.plan, duty) {
            self.plan = Some(p.with_breakaway(d));
        }
    }

    /// Skip what is left to the closing centring: a run cut short by
    /// request still ends at mid travel.
    pub fn finish(&mut self) {
        self.next = self.next.max(ORDER.len() - 1);
        self.pending = None;
    }

    pub fn plan(&self) -> Option<DutyPlan> {
        self.plan
    }

    pub fn over(&self) -> Option<Over> {
        self.over
    }

    /// The stage that ended the run early, and why.
    pub fn aborted(&self) -> Option<(&'static str, AbortReason)> {
        match self.over {
            Some(Over::Aborted(stage, reason)) => Some((stage, reason)),
            _ => None,
        }
    }
}

fn fractions(q15: &[i16]) -> Vec<f64> {
    q15.iter().map(|q| *q as f64 / Q15).collect()
}

/// The centring as a stage names it.
pub fn centre_cfg(duty: f64, cap: f64, nudge: bool) -> CentreCfg {
    CentreCfg {
        duty_q15: q15_floor(duty),
        cap_q15: q15_floor(cap),
        nudge,
        ..CentreCfg::default()
    }
}

/// `base` with a stage's burst: both rungs from rest, the from-a-hold
/// control from `pre` to the top rung, repeats cut to the allowance's
/// count.
pub fn burst_cfg(rungs: &[f64], pre: f64, seek: f64, base: BurstCfg) -> BurstCfg {
    let step_pct: Vec<u8> = rungs.iter().map(|d| pct_floor(*d)).collect();
    let top = step_pct.iter().copied().max().unwrap_or(0);
    let hold_pct = (pct_floor(pre) > 0 && top > pct_floor(pre)).then(|| (pct_floor(pre), top));
    let arms =
        (step_pct.len() + hold_pct.is_some() as usize) as u32 * if base.both_signs { 2 } else { 1 };
    BurstCfg {
        repeats: BurstAllowance::repeats(base.repeats, arms),
        hold_pct,
        step_pct,
        seek_duty_q15: q15_floor(seek),
        ..base
    }
}

/// `base` with the stop ladder's seek and dwells.
pub fn resistance_cfg(seek: f64, rungs: &[f64], base: ResistanceCfg) -> ResistanceCfg {
    ResistanceCfg {
        seek_duty_q15: q15_floor(seek),
        ladder_q15: rungs.iter().map(|d| q15_floor(*d)).collect(),
        ..base
    }
}

pub fn breakaway_cfg(cap: f64) -> BreakawayCfg {
    let base = BreakawayCfg::default();
    let cap_q15 = q15_floor(cap);
    BreakawayCfg {
        start_q15: base.start_q15.min(cap_q15),
        cap_q15,
        ..base
    }
}

pub fn ladder_cfg(seek: f64, rungs: &[f64]) -> LadderCfg {
    let mut rungs_q15: Vec<i16> = rungs.iter().map(|d| q15_floor(*d)).collect();
    rungs_q15.dedup();
    LadderCfg {
        rungs_q15,
        seek_duty_q15: q15_floor(seek),
        ..LadderCfg::default()
    }
}

pub fn inertia_cfg(seek: f64, base: f64, cfg: InertiaCfg) -> InertiaCfg {
    InertiaCfg {
        seek_duty_q15: q15_floor(seek),
        base_q15: q15_floor(base),
        ..cfg
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::burst::Capture;
    use crate::exp::bias::{Bias, BiasCfg};
    use crate::exp::breakaway::Breakaway;
    use crate::exp::centre::Centre;
    use crate::exp::inductance::{FitCfg, Inductance, fit_captures};
    use crate::exp::inertia::Inertia;
    use crate::exp::ladder::{Ladder, LadderResult};
    use crate::exp::resistance::Resistance;
    use crate::exp::testkit::{FakeServo, bench_mg90, pump};
    use crate::exp::{Experiment, Guarded, Permitted, RigParams};
    use crate::fits::InertiaPriors;
    use crate::runway::Runway;
    use crate::sources;
    use crate::units::SenseParams;

    /// The bench MG90: limit 280, 4.9 ohm = 1.775 vcounts per ccount.
    const LIM: u16 = 280;
    const R: f64 = 7270.0 / 4096.0;
    const RAIL_2S: u16 = 3204;
    const RAIL_USB: u16 = 1780;

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

    /// The bench servo as the CLI reads it on a rail of `vbus`: its stops,
    /// its soft limits, R not yet identified.
    fn limits(vbus: u16) -> ServoLimits {
        ServoLimits {
            i_lim: LIM,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 0,
            vbus,
            window_floor_q15: 4356,
            amps_per_count: scales().amps_per_count,
        }
    }

    fn run(vbus: u16) -> Run {
        Run::new(limits(vbus), &scales())
    }

    /// Drive the sequencer, answering each stage with `how` and feeding
    /// what the stage would have measured.
    fn stages(run: &mut Run, mut how: impl FnMut(&Stage) -> Ended) -> Vec<&'static str> {
        let mut seen = Vec::new();
        while let Some(s) = run.next_stage() {
            seen.push(s.name());
            match s {
                Stage::Centre { nudge: true, .. } => run.nudged(Some(0.12)),
                Stage::Burst { .. } | Stage::Resistance { .. } => run.measured(R),
                _ => {}
            }
            run.ended(how(&s));
        }
        seen
    }

    const ORDER_NAMES: [&str; 10] = [
        "bias",
        "centring",
        "burst",
        "centring",
        "breakaway",
        "centring",
        "ladder",
        "centring",
        "inertia",
        "centring",
    ];

    #[test]
    fn run_all_stops_at_the_first_abort() {
        let blocked = AbortReason::Blocked {
            pos: 2048,
            moved: 0,
        };
        let full = stages(&mut run(RAIL_2S), |_| Ended::Done);
        assert_eq!(full, ORDER_NAMES);
        for k in 0..full.len() {
            let mut r = run(RAIL_2S);
            let mut n = 0;
            let seen = stages(&mut r, |_| {
                n += 1;
                if n == k + 1 {
                    Ended::Aborted(blocked)
                } else {
                    Ended::Done
                }
            });
            assert_eq!(seen, full[..=k], "aborted at stage {k}");
            assert_eq!(r.aborted(), Some((full[k], blocked)));
            assert_eq!(r.next_stage(), None, "the run stays over");
        }
        // a declined burst leaves nothing to plan from
        let mut r = run(RAIL_2S);
        let seen = stages(&mut r, |s| match s {
            Stage::Burst { .. } => Ended::Declined,
            _ => Ended::Done,
        });
        assert_eq!(seen, full[..3]);
        assert_eq!(r.over(), Some(Over::Declined("burst")));
        // a declined ladder still ends with the closing centring
        let mut r = run(RAIL_2S);
        let seen = stages(&mut r, |s| match s {
            Stage::Ladder { .. } => Ended::Declined,
            _ => Ended::Done,
        });
        assert_eq!(seen, [&full[..7], &full[9..]].concat());
        assert_eq!(r.over(), Some(Over::Declined("ladder")));
        assert_eq!(r.next_stage(), None);
    }

    /// A declined burst ends the run unless the stop ladder was asked for;
    /// asked for, it runs once, in the burst's place, and a stop ladder
    /// that fits nothing ends the run too. On 2S, with R unknown, the cap
    /// the class's lowest R allows sits under the window floor: the ladder
    /// has no room and the run ends before it moves.
    #[test]
    fn the_stop_ladder_runs_only_on_request() {
        let declined = |s: &Stage| match s {
            Stage::Burst { .. } | Stage::Resistance { .. } => Ended::Declined,
            _ => Ended::Done,
        };
        let mut r = run(RAIL_2S);
        assert_eq!(stages(&mut r, declined), ORDER_NAMES[..3]);
        assert_eq!(r.over(), Some(Over::Declined("burst")));

        let mut r = run(RAIL_2S).with_stall_ladder();
        assert_eq!(stages(&mut r, declined), ORDER_NAMES[..3]);
        assert!(matches!(r.over(), Some(Over::NoLadderRoom { floor, cap }) if cap < floor));

        let mut r = run(RAIL_USB).with_stall_ladder();
        assert_eq!(
            stages(&mut r, declined),
            ["bias", "centring", "burst", "resistance"]
        );
        assert_eq!(r.over(), Some(Over::ResistanceDeclined));

        let mut r = run(RAIL_USB).with_stall_ladder();
        let mut ladder = None;
        let seen = stages(&mut r, |s| match s {
            Stage::Burst { .. } => Ended::Declined,
            Stage::Resistance { seek, rungs } => {
                ladder = Some((*seek, rungs.clone()));
                Ended::Done
            }
            _ => Ended::Done,
        });
        assert_eq!(
            &seen[..5],
            ["bias", "centring", "burst", "resistance", "centring"]
        );
        assert_eq!(&seen[5..], &ORDER_NAMES[4..]);
        assert_eq!(r.over(), None);
        // R unknown: the class's lowest R plans it, errs safe on this one
        let (seek, rungs) = ladder.unwrap();
        let lim = limits(RAIL_USB);
        let class = lim.stall_plan(scales().r_vpc(CLASS_R_MIN), Some(0.12));
        assert_eq!(seek, class.seek);
        assert_eq!(rungs, class.stall_ladder(lim.window_floor()).unwrap());
        assert_eq!(rungs.len(), 4);
        assert!(q15_floor(rungs[0]) >= lim.window_floor());
        for d in &rungs {
            let stall = crate::limits::stall_counts(*d, R, RAIL_USB as f64);
            assert!(stall <= LIM as f64, "{d}: {stall}");
        }

        // a rail too high for the burst hands over the same way, and has no
        // room either
        let mut r = run(3800).with_stall_ladder();
        assert_eq!(stages(&mut r, |_| Ended::Done), ORDER_NAMES[..2]);
        assert!(matches!(r.over(), Some(Over::NoLadderRoom { .. })));
    }

    /// The whole default run on the bench servo, both rails, and a rail
    /// too high for the burst: nothing is ever driven into a stop and the
    /// stall permit is never written.
    #[test]
    fn default_run_never_stalls_a_stop() {
        for vbus in [RAIL_2S, RAIL_USB, 3800] {
            let mut servo = bench_servo(vbus);
            servo.pos = 2600.0;
            let mut run = run(vbus);
            let mut rig = Rig::new(&mut servo, vbus);
            rig.run(&mut run);
            let expect = if vbus == 3800 {
                Some(Over::RailTooHigh {
                    rail_mv: run.rail_mv(),
                })
            } else {
                None
            };
            assert_eq!(run.over(), expect, "{vbus}");
            assert!(
                !rig.log.iter().any(|l| l.starts_with("write stall_permit")),
                "{vbus}: the permit was written"
            );
            assert_eq!(servo.pressed_ms, 0.0, "{vbus}: driven into a stop");
            assert!(!servo.torque && !servo.permit_live());
        }
    }

    /// Asked for, on USB with a burst that declines: the stop ladder stalls
    /// both stops over the window floor and under the limit with the permit
    /// held, measures R, and the run plans from it to the end, centred
    /// with torque off.
    #[test]
    fn the_stop_ladder_measures_r_when_asked() {
        let vbus = RAIL_USB;
        let mut servo = bench_servo(vbus);
        servo.pos = 2600.0;
        let mut run = run(vbus).with_stall_ladder();
        let mut rig = Rig::new(&mut servo, vbus);
        rig.decline_burst = true;
        rig.run(&mut run);
        assert_eq!(run.over(), None, "{:?}", run.over());
        let r = run.plan().expect("planned").r_vpc;
        assert!((r / R - 1.0).abs() < 0.02, "R {r}");
        let at = |name: &str| rig.marks.iter().position(|m| m.0 == name).unwrap();
        let k = at("resistance");
        let permit = |l: &String| l.starts_with("write stall_permit");
        assert!(rig.span(k, k + 1).iter().any(permit));
        assert!(
            !rig.span(0, k).iter().any(permit)
                && !rig.span(k + 1, rig.marks.len()).iter().any(permit)
        );
        assert!(
            rig.span(k, k + 1)
                .last()
                .is_some_and(|l| l == "write stall_permit 0")
        );
        assert!(servo.pressed_ms > 1000.0, "the stops were never stalled");
        assert!(!servo.torque && !servo.permit_live());
        assert!((servo.pos - 2029.0).abs() <= 300.0, "ends at {}", servo.pos);
    }

    /// No burst arms before the jam check proved the shaft free, and none
    /// on a rail the volts cap leaves no pair on.
    #[test]
    fn burst_waits_for_the_nudge() {
        let mut r = run(RAIL_2S);
        assert_eq!(r.next_stage(), Some(Stage::Bias));
        assert!(matches!(
            r.next_stage(),
            Some(Stage::Centre { nudge: true, .. })
        ));
        r.nudged(None);
        r.ended(Ended::Done);
        assert_eq!(r.next_stage(), None);
        assert_eq!(r.over(), Some(Over::Unproven));

        let mut r = run(RAIL_2S);
        r.next_stage();
        r.next_stage();
        r.nudged(Some(0.145));
        let Some(Stage::Burst { rungs, pre, seek }) = r.next_stage() else {
            panic!("no burst after the jam check");
        };
        assert_eq!(rungs, [0.25, 0.40]);
        assert_eq!((pre, seek), (r.bootstrap(), 0.145));

        // 3800 vcounts is 9.4 V: 3.2 V caps the top rung at 34%
        let mut r = run(3800);
        r.next_stage();
        r.next_stage();
        r.nudged(Some(0.145));
        assert_eq!(r.next_stage(), None);
        assert!(matches!(r.over(), Some(Over::RailTooHigh { .. })));
    }

    #[test]
    fn nudge_cap_is_two_volts_on_the_rail() {
        let r = run(RAIL_2S);
        assert!((r.bootstrap() - 0.0952).abs() < 1e-3);
        assert!((r.nudge_cap() - 0.253).abs() < 1e-3);
        let r = run(RAIL_USB);
        assert!((r.bootstrap() - 0.1713).abs() < 1e-3);
        assert!((r.nudge_cap() - 0.4557).abs() < 1e-3);
    }

    #[test]
    fn burst_cfg_is_the_allowance() {
        let base = BurstCfg {
            repeats: 5,
            ..BurstCfg::default()
        };
        let cfg = burst_cfg(&[0.25, 0.40], 0.0952, 0.145, base);
        assert_eq!(cfg.step_pct, [25, 40]);
        assert_eq!(cfg.hold_pct, Some((9, 40)));
        assert_eq!(cfg.repeats, 4);
        assert_eq!(
            crate::exp::inductance::plan(&cfg).len() as u32,
            BurstAllowance::MAX_ARMS
        );
    }

    /// The stages the way the CLI runs them, against the fake, feeding back
    /// what the CLI feeds back.
    struct Rig<'a> {
        servo: &'a mut FakeServo,
        vbus: u16,
        log: Vec<String>,
        /// Log length when each stage ended.
        marks: Vec<(&'static str, usize)>,
        caps: Vec<Capture>,
        r_ohm: Option<f64>,
        ladder: Option<LadderResult>,
        /// What the ladder measured, handed on to inertia.
        runway: Option<Runway>,
        inertia_fit: bool,
        /// The burst measures but its result is taken as declined.
        decline_burst: bool,
    }

    impl Rig<'_> {
        fn new(servo: &mut FakeServo, vbus: u16) -> Rig<'_> {
            Rig {
                servo,
                vbus,
                log: Vec::new(),
                marks: Vec::new(),
                caps: Vec::new(),
                r_ohm: None,
                ladder: None,
                runway: None,
                inertia_fit: false,
                decline_burst: false,
            }
        }

        fn go<E: Experiment>(&mut self, exp: E, params: RigParams) -> (E, Ended) {
            let mut g = Guarded::new(exp, params);
            self.log.extend(pump(&mut g, self.servo, 4_000_000));
            let how = g.abort().map_or(Ended::Done, Ended::Aborted);
            (g.into_inner(), how)
        }

        fn run(&mut self, run: &mut Run) {
            while let Some(stage) = run.next_stage() {
                let how = self.execute(run, &stage);
                run.ended(how);
            }
        }

        fn execute(&mut self, run: &mut Run, stage: &Stage) -> Ended {
            let lim = limits(self.vbus);
            let params =
                RigParams::new(Some(lim.guard().unwrap()), lim.abort_default()).with_stops(lim.raw);
            let base = BurstCfg {
                repeats: 5,
                i_max_a: BurstAllowance::i_max_a(),
                ..BurstCfg::default()
            };
            let how = match stage {
                Stage::Bias => {
                    let exp = Bias::new(BiasCfg::default(), &params);
                    self.go(exp, params.without_pos_guard()).1
                }
                Stage::Centre { duty, cap, nudge } => {
                    let exp = Centre::new(centre_cfg(*duty, *cap, *nudge), &params);
                    let (exp, how) = self.go(exp, params.without_pos_guard());
                    if *nudge {
                        run.nudged(exp.moved_at());
                    }
                    how
                }
                Stage::Burst { rungs, pre, seek } => {
                    let cfg = burst_cfg(rungs, *pre, *seek, base);
                    let (exp, how) = self.go(Inductance::new(cfg, &params, scales()), params);
                    self.caps.extend_from_slice(exp.captures());
                    let fit = fit_captures(&self.caps, &scales(), &FitCfg::default());
                    match sources::winding(fit.as_ref(), None, Some(&scales()), 0.0) {
                        _ if self.decline_burst && how == Ended::Done => Ended::Declined,
                        Some(w) if how == Ended::Done => {
                            self.r_ohm = w.r_ohm;
                            run.measured(w.r_vpc);
                            how
                        }
                        _ if how == Ended::Done => Ended::Declined,
                        _ => how,
                    }
                }
                Stage::Resistance { seek, rungs } => {
                    let cfg = resistance_cfg(*seek, rungs, ResistanceCfg::default());
                    let exp = Permitted::new(Resistance::new(cfg, &params));
                    let (exp, how) = self.go(exp, params.without_pos_guard());
                    match exp.into_inner().fit() {
                        Some(r) if how == Ended::Done => {
                            run.measured(r.r_vpc);
                            how
                        }
                        _ if how == Ended::Done => Ended::Declined,
                        _ => how,
                    }
                }
                Stage::Breakaway { cap } => {
                    let (exp, how) = self.go(Breakaway::new(breakaway_cfg(*cap)), params);
                    let bk = exp.fit(R, self.vbus as f64);
                    let top = bk.duty_bk_fwd.into_iter().chain(bk.duty_bk_rev).max();
                    run.broke_away(top.map(|d| d as f64 / Q15));
                    how
                }
                Stage::Ladder { seek, rungs } => {
                    let runway = Runway::new(lim.guard().unwrap());
                    let ladder = Ladder::new(ladder_cfg(*seek, rungs), &params, runway);
                    let (exp, how) = self.go(ladder, params);
                    self.ladder = exp.fit(R);
                    self.runway = Some(exp.runway().clone());
                    match exp.declined() {
                        Some(_) if how == Ended::Done => Ended::Declined,
                        _ => how,
                    }
                }
                Stage::Inertia { seek, base } => {
                    let cfg = inertia_cfg(*seek, *base, InertiaCfg::default());
                    let plan = run.plan().expect("planned");
                    let runway = self.runway.clone().expect("the ladder's runway");
                    let (exp, how) = self.go(Inertia::new(cfg, plan, runway, &params), params);
                    if let Some(l) = &self.ladder {
                        let priors = InertiaPriors {
                            r_vpc: R,
                            ke_vpc: l.ke.ke_vpc,
                            fc: l.fric_fwd.map_or(0.0, |f| f.fc),
                            fv: l.fric_fwd.map_or(0.0, |f| f.fv),
                            tick_hz: 20_100.0,
                        };
                        self.inertia_fit = exp.fit(&priors).is_some();
                    }
                    how
                }
            };
            self.marks.push((stage.name(), self.log.len()));
            how
        }

        /// The log between the ends of stages `a` and `b` (0 = the start).
        fn span(&self, a: usize, b: usize) -> &[String] {
            let at = |k: usize| if k == 0 { 0 } else { self.marks[k - 1].1 };
            &self.log[at(a)..at(b)]
        }
    }

    /// Every duty a log span commands, goals and burst steps alike.
    fn drives(log: &[String]) -> Vec<f64> {
        log.iter()
            .filter_map(|l| {
                let q: i32 = if let Some(v) = l.strip_prefix("burst ") {
                    v.split(' ').next()?.parse().ok()?
                } else {
                    let v = l
                        .strip_prefix("write goal_duty ")
                        .or_else(|| l.strip_prefix("stream "))?;
                    v.split(' ').next_back()?.parse().ok()?
                };
                Some((q as f64 / Q15).abs())
            })
            .collect()
    }

    /// The bench MG90 with the burst plant on the same winding and rail.
    fn bench_servo(vbus: u16) -> FakeServo {
        let sc = scales();
        let mut s = bench_mg90(vbus);
        s.burst.r = R * sc.v_term_per_count / sc.amps_per_count;
        s.burst.v_rail = vbus as f64 * sc.v_term_per_count;
        s.burst.v0 = 0.2;
        s
    }

    /// The default run on the bench servo, both rails, from a start off
    /// mid travel: it reaches the fit - R and L from the burst, a ladder
    /// that fits Ke, inertia steps that fit b - inside the allowance.
    #[test]
    fn bench_mg90_run_reaches_the_fit_on_2s_and_usb() {
        for vbus in [RAIL_2S, RAIL_USB] {
            let mut servo = bench_servo(vbus);
            servo.pos = 2600.0;
            let mut run = run(vbus);
            let mut rig = Rig::new(&mut servo, vbus);
            rig.run(&mut run);
            assert_eq!(run.over(), None, "{vbus}: {:?}", run.over());
            let names: Vec<&str> = rig.marks.iter().map(|m| m.0).collect();
            assert_eq!(names, ORDER_NAMES);
            let plan = run.plan().expect("planned");
            println!(
                "{vbus} vcounts: bootstrap {:.4}, nudge cap {:.4}, R {:?} ohm, seek {:.4}, \
                 hold {:.4}, stop cap {:.4}, ladder {} of {} rungs used",
                run.bootstrap(),
                run.nudge_cap(),
                rig.r_ohm,
                plan.seek,
                plan.hold,
                plan.stop_cap,
                rig.ladder
                    .as_ref()
                    .map_or(0, |l| l.rungs.iter().filter(|r| r.used).count()),
                rig.ladder.as_ref().map_or(0, |l| l.rungs.len()),
            );
            let r = rig.r_ohm.expect("R from the burst");
            let r_true = R * scales().v_term_per_count / scales().amps_per_count;
            assert!((r / r_true - 1.0).abs() < 0.1, "{vbus}: R {r} of {r_true}");
            let arms = rig.log.iter().filter(|l| l.starts_with("burst ")).count();
            assert!(arms as u32 <= BurstAllowance::MAX_ARMS, "{arms} bursts");
            let ladder = rig.ladder.as_ref().expect("the ladder fits");
            assert!(ladder.rungs.iter().filter(|r| r.used).count() >= 3);
            assert!(rig.inertia_fit, "{vbus}: inertia did not fit");
            assert!(!servo.torque);
            assert!((servo.pos - 2029.0).abs() <= 300.0, "ends at {}", servo.pos);
        }
    }

    /// Nothing drives at a planned duty until R and L are in: before the
    /// burst the jam check stays under its 2 V cap, the bursts under the
    /// 3.2 V allowance, and after it every stall-safe drive - centring,
    /// the breakaway ramp - stays at or under the plan's stop cap.
    #[test]
    fn r_and_l_are_measured_before_any_planned_drive() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2600.0;
        let mut run = run(RAIL_2S);
        let (nudge_cap, rail_mv) = (run.nudge_cap(), run.rail_mv());
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.run(&mut run);
        assert_eq!(run.over(), None);
        let plan = run.plan().expect("planned");
        let top = |span: &[String]| drives(span).into_iter().fold(0.0, f64::max);
        assert!(top(rig.span(0, 2)) <= nudge_cap + 1e-9, "the jam check");
        assert!(top(rig.span(2, 3)) * rail_mv <= BurstAllowance::MAX_MV + 1e-6);
        for k in [3, 4, 5, 7, 9] {
            let span = rig.span(k, k + 1);
            assert!(
                top(span) <= plan.stop_cap + 1e-9,
                "{}: {} over the stop cap {}",
                rig.marks[k].0,
                top(span),
                plan.stop_cap
            );
        }
        let stall = plan.stop_cap * RAIL_2S as f64 / plan.r_vpc;
        assert!(stall <= LIM as f64 + 1e-6);
    }

    /// The bench MG90's inertia steps as planned: limit 280, 4.9 ohm, a
    /// shaft that breaks away at 8%. Seeks at 10%, the base at 15%, and a
    /// base drawing 80 counts leaves 9.1% of room on 2S: steps to 19.5,
    /// 21.8 and 24.1%. On USB the same room is 16.4%.
    #[test]
    fn bench_inertia_base_and_steps() {
        for (vbus, want) in [
            (RAIL_2S, [0.1955, 0.2183, 0.2410]),
            (RAIL_USB, [0.2322, 0.2733, 0.3145]),
        ] {
            let mut run = run(vbus);
            let mut base = None;
            while let Some(s) = run.next_stage() {
                match s {
                    Stage::Centre { nudge: true, .. } => run.nudged(Some(0.12)),
                    Stage::Burst { .. } => run.measured(R),
                    Stage::Breakaway { .. } => run.broke_away(Some(0.08)),
                    Stage::Inertia { seek, base: b } => {
                        assert!((seek - 0.10).abs() < 1e-12);
                        base = Some(b);
                    }
                    _ => {}
                }
                run.ended(Ended::Done);
            }
            let base = base.expect("an inertia stage");
            assert!((base - 0.15).abs() < 1e-12);
            let cfg = inertia_cfg(0.10, base, InertiaCfg::default());
            let steps = cfg.step_duties(&run.plan().unwrap(), 80.0);
            for (got, want) in steps.iter().zip(want) {
                assert!((got - want).abs() < 5e-4, "{vbus}: {steps:?}");
            }
        }
    }

    /// A virgin servo whose shaft needs more than the class-safe duty: the
    /// jam check raises until it moves, then the burst measures R.
    #[test]
    fn virgin_run_escalates_the_nudge_and_measures_r() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2100.0;
        let mut run = run(RAIL_2S);
        let boot = run.bootstrap();
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        while let Some(stage) = run.next_stage() {
            let burst = matches!(stage, Stage::Burst { .. });
            let how = rig.execute(&mut run, &stage);
            run.ended(how);
            if burst {
                run.finish();
            }
        }
        assert_eq!(run.over(), None);
        let moved = run.plan().expect("R measured").seek - DutyPlan::SEEK_MARGIN;
        assert!(moved > 0.13 && moved > boot, "moved at {moved}");
        assert!(rig.r_ohm.is_some());
    }

    /// The bench servo with its shaft locked at mid travel: the jam
    /// check raises to its cap, the shaft never moves, and the run ends
    /// there. No stall dwell, no ladder rung and no burst is ever
    /// commanded.
    #[test]
    fn a_mid_travel_jam_ends_the_run_at_the_first_seek() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2048.0;
        servo.jam = Some(2048.0);
        let mut run = run(RAIL_2S);
        let cap = run.nudge_cap();
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.run(&mut run);
        let names: Vec<&str> = rig.marks.iter().map(|m| m.0).collect();
        assert_eq!(names, ["bias", "centring"]);
        assert_eq!(
            run.aborted(),
            Some((
                "centring",
                AbortReason::Blocked {
                    pos: 2048,
                    moved: 0
                }
            ))
        );
        let log = rig.log;
        assert!(drives(&log).iter().all(|d| *d <= cap));
        assert!(
            log.iter()
                .all(|l| !l.starts_with("burst") && !l.starts_with("stream"))
        );
        assert_eq!(
            log.last().map(String::as_str),
            Some("write torque_enable 0")
        );
        assert!(!servo.torque && !servo.permit_live());
    }
}
