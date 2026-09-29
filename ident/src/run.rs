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
//! centring. Nothing in the default run stalls a stop. Asked for
//! ([`Run::with_stall_ladder`]), the resistance stop ladder measures R when
//! the burst declines, its dwells stalling between the current sensor's
//! window floor and the limit ([`DutyPlan::stall_ladder`]) with the permit
//! held. When neither measures R the run plans from the winding the servo
//! carries from an earlier identification ([`Run::with_stored_winding`]):
//! R and L belong to the motor, not to the position table or the gear
//! train. Without one the run ends, after a closing centring that starts
//! at the class-safe duty.
//!
//! `osc cal` takes the same front ([`Run::for_cal`]) - bias, the jam check,
//! the burst - then finds the stops: each approached at the one duty the
//! jam check moved the shaft at plus the seek margin, capped where its
//! stall draws the limit, and seated at half the limit
//! ([`crate::exp::endstop`]). A shaft whose breakaway alone stalls over
//! the limit is refused before anything drives at a stop. A burst that
//! declines leaves R unknown and cal goes on without it: the stops are
//! approached at the duty that moved the shaft and seated at half the
//! class-safe duty. The ripple traverse follows, free-running on the
//! runway ([`crate::exp::sweep`]), and the run ends at mid travel.

use crate::exp::AbortReason;
use crate::exp::breakaway::BreakawayCfg;
use crate::exp::centre::CentreCfg;
use crate::exp::endstop::EndstopCfg;
use crate::exp::inductance::Cfg as BurstCfg;
use crate::exp::inertia::{BASE_OVER_SEEK, InertiaCfg};
use crate::exp::ladder::LadderCfg;
use crate::exp::resistance::ResistanceCfg;
use crate::exp::rl::Scales;
use crate::exp::sweep::SweepCfg;
use crate::limits::{
    BurstAllowance, CLASS_R_MIN, DutyPlan, NUDGE_MAX_MV, Refusal, STOP_LADDER, ServoLimits,
    duty_of_mv, pct_floor, q15_floor,
};
use crate::sources::Winding;

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
    /// Cal: each stop approached at `approach`, seated at `seat`; a sticky
    /// spot or a leave may raise the duty up to `cap`.
    Stops {
        approach: f64,
        seat: f64,
        cap: f64,
    },
    /// Cal: the ripple traverse, its first pass to the start band at
    /// `seek`.
    Traverse {
        seek: f64,
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
            Stage::Stops { .. } => "stops",
            Stage::Traverse { .. } => "traverse",
        }
    }
}

/// How a stage ended.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Ended {
    Done,
    /// Ran, but its result does not feed the gains: the burst's routes
    /// declined, the ladder fitted too few rungs, or it produced nothing to
    /// fit. Nothing after it can be planned unless a stored winding stands
    /// in for the burst; the run still ends with its closing centring.
    Declined,
    Aborted(AbortReason),
}

/// Why the run ended before its last stage.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Over {
    Aborted(&'static str, AbortReason),
    /// The named stage gave nothing to fit: nothing after it can be
    /// planned. The burst's decline, with no stored winding, leaves no
    /// winding R.
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
    /// Cal: the duty that first moved the shaft stalls over the limit,
    /// drawing `need` counts ([`Refusal::BreakawayOverLimit`]).
    BreakawayOverLimit {
        need: f64,
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
    Stops,
    Traverse,
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

/// Cal: the jam check and the burst at mid travel, the stops, the traverse
/// from the low end, and back to mid travel.
const CAL_ORDER: [Step; 6] = [
    Step::Bias,
    Step::Nudge,
    Step::Burst,
    Step::Stops,
    Step::Traverse,
    Step::Park,
];

#[derive(Clone, Debug)]
pub struct Run {
    lim: ServoLimits,
    rail_mv: f64,
    bootstrap: f64,
    class_r_vpc: f64,
    stall_ladder: bool,
    cal: bool,
    /// What an earlier identification left on the servo.
    stored: Option<Winding>,
    /// The ending the stored winding stood in for, once it did.
    reused: Option<Over>,
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
            cal: false,
            stored: None,
            reused: None,
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

    /// The winding the servo carries, read before the run: when neither the
    /// burst nor the stop ladder measures R, the run plans from it.
    pub fn with_stored_winding(self, stored: Option<Winding>) -> Self {
        Self { stored, ..self }
    }

    /// The stored winding, and the ending it stood in for, once the run
    /// plans from it.
    pub fn reused(&self) -> Option<(Winding, Over)> {
        self.stored.zip(self.reused)
    }

    /// The run `osc cal` takes: the front of the default run, then the
    /// stops, the traverse and the closing centring. A burst that declines
    /// leaves R unknown and the run goes on.
    pub fn for_cal(lim: ServoLimits, sc: &Scales) -> Self {
        Self {
            cal: true,
            ..Self::new(lim, sc)
        }
    }

    fn order(&self) -> &'static [Step] {
        if self.cal { &CAL_ORDER } else { &ORDER }
    }

    /// Cal's approach, seat and cap: by the plan once R is measured, else
    /// the duty that moved the shaft, half the class-safe duty and the jam
    /// check's cap. None before the jam check moved the shaft.
    pub fn stop_duties(&self) -> Option<(f64, f64, f64)> {
        let moved = self.free?;
        Some(match self.plan {
            Some(p) => (p.seek, p.hold, p.stop_cap),
            None => (moved, self.bootstrap / 2.0, self.nudge_cap()),
        })
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
                let Some(&s) = self.order().get(self.next) else {
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
                    None if self.cal => return self.next_stage(),
                    None if self.stall_ladder => match self.stop_ladder() {
                        Ok(stage) => stage,
                        Err(why) => return self.end(why),
                    },
                    None => {
                        let rail_mv = self.rail_mv;
                        return self.end(Over::RailTooHigh { rail_mv });
                    }
                }
            }
            (Step::Resistance, _) => match self.stop_ladder() {
                Ok(stage) => stage,
                Err(why) => {
                    self.no_r(why);
                    return self.next_stage();
                }
            },
            (Step::Stops, plan) => {
                let (Some(moved), Some((approach, seat, cap))) = (self.free, self.stop_duties())
                else {
                    return self.end(Over::Unproven);
                };
                if let Some(p) = plan
                    && let Err(Refusal::BreakawayOverLimit { need, .. }) =
                        self.lim.check_breakaway(moved, p.r_vpc)
                {
                    return self.end(Over::BreakawayOverLimit { need });
                }
                Stage::Stops {
                    approach,
                    seat,
                    cap,
                }
            }
            (Step::Traverse, _) => match self.stop_duties() {
                Some((approach, _, _)) => Stage::Traverse { seek: approach },
                None => return self.end(Over::Unproven),
            },
            (Step::Park, _) if self.cal => match self.stop_duties() {
                Some((approach, _, cap)) => Stage::Centre {
                    duty: approach,
                    cap,
                    nudge: false,
                },
                None => return self.end(Over::Unproven),
            },
            // the jam check's own start and cap: nothing measured R
            (Step::Park, None) => Stage::Centre {
                duty: boot,
                cap: self.nudge_cap(),
                nudge: false,
            },
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
    fn stop_ladder(&self) -> Result<Stage, Over> {
        let plan = self.lim.stall_plan(self.class_r_vpc, self.free);
        match plan.stall_ladder(STOP_LADDER, self.lim.window_floor()) {
            Ok(rungs) => Ok(Stage::Resistance {
                seek: plan.seek,
                rungs,
            }),
            Err(Refusal::NoLadderRoom { floor, cap, .. }) => Err(Over::NoLadderRoom { floor, cap }),
            Err(_) => Err(Over::ResistanceDeclined),
        }
    }

    /// Nothing in this run measured R: plan from the stored winding and go
    /// on, else end with `why` after the closing centring.
    fn no_r(&mut self, why: Over) {
        match self.stored {
            Some(w) => {
                self.plan = Some(DutyPlan::new(&self.lim, w.r_vpc, self.free));
                self.reused = Some(why);
            }
            None => {
                self.cut = Some(why);
                self.finish();
            }
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
                Some("burst" | "traverse") if self.cal => {}
                Some(stage) if self.cal => {
                    self.cut = Some(Over::Declined(stage));
                    self.finish();
                }
                Some("burst") if self.stall_ladder => self.pending = Some(Step::Resistance),
                Some("burst") => self.no_r(Over::Declined("burst")),
                Some("resistance") => self.no_r(Over::ResistanceDeclined),
                Some(stage) if self.plan.is_some() => {
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
        self.next = self.next.max(self.order().len() - 1);
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

/// Cal's stop finder at a stage's duties.
pub fn endstop_cfg(approach: f64, seat: f64, cap: f64) -> EndstopCfg {
    EndstopCfg {
        approach_q15: q15_floor(approach),
        seat_q15: q15_floor(seat),
        cap_q15: q15_floor(cap),
        ..EndstopCfg::default()
    }
}

/// `base` with the traverse's first pass at a stage's seek.
pub fn sweep_cfg(seek: f64, base: SweepCfg) -> SweepCfg {
    SweepCfg {
        seek_duty_q15: q15_floor(seek),
        ..base
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
    use crate::exp::endstop::{Endstop, EndstopResult};
    use crate::exp::inductance::{FitCfg, Inductance, fit_captures};
    use crate::exp::inertia::Inertia;
    use crate::exp::ladder::{Ladder, LadderResult};
    use crate::exp::resistance::Resistance;
    use crate::exp::sweep::{Captured, Sweep};
    use crate::exp::testkit::{Bus, FakeServo, bench_mg90, pump_on};
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
        // a declined burst leaves nothing to plan from: the closing
        // centring, then the end
        let mut r = run(RAIL_2S);
        let seen = stages(&mut r, |s| match s {
            Stage::Burst { .. } => Ended::Declined,
            _ => Ended::Done,
        });
        assert_eq!(seen, [&full[..3], &full[9..]].concat());
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
    /// has no room and never moves. Each ends with the closing centring.
    #[test]
    fn the_stop_ladder_runs_only_on_request() {
        let declined = |s: &Stage| match s {
            Stage::Burst { .. } | Stage::Resistance { .. } => Ended::Declined,
            _ => Ended::Done,
        };
        let closed = ["bias", "centring", "burst", "centring"];
        let mut r = run(RAIL_2S);
        assert_eq!(stages(&mut r, declined), closed);
        assert_eq!(r.over(), Some(Over::Declined("burst")));

        let mut r = run(RAIL_2S).with_stall_ladder();
        assert_eq!(stages(&mut r, declined), closed);
        assert!(matches!(r.over(), Some(Over::NoLadderRoom { floor, cap }) if cap < floor));

        let mut r = run(RAIL_USB).with_stall_ladder();
        assert_eq!(
            stages(&mut r, declined),
            ["bias", "centring", "burst", "resistance", "centring"]
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
        assert_eq!(
            rungs,
            class.stall_ladder(STOP_LADDER, lim.window_floor()).unwrap()
        );
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

        // 3800 vcounts is 9.4 V: 3.2 V less the margin caps the top rung at 33%
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
        bus: Bus,
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
        /// What the burst's captures fitted, declined or not.
        e8: Option<crate::exp::inductance::InductanceResult>,
        /// What the jam check first moved the shaft at.
        moved: Option<f64>,
        stops: Option<EndstopResult>,
        captured: Option<Captured>,
        /// A servo never calibrated: no soft limits, no stops known.
        virgin: bool,
    }

    impl Rig<'_> {
        fn new(servo: &mut FakeServo, vbus: u16) -> Rig<'_> {
            Rig {
                servo,
                vbus,
                bus: Bus::BENCH,
                log: Vec::new(),
                marks: Vec::new(),
                caps: Vec::new(),
                r_ohm: None,
                ladder: None,
                runway: None,
                inertia_fit: false,
                decline_burst: false,
                e8: None,
                moved: None,
                stops: None,
                captured: None,
                virgin: false,
            }
        }

        fn go<E: Experiment>(&mut self, exp: E, params: RigParams) -> (E, Ended) {
            let mut g = Guarded::new(exp, params);
            self.log
                .extend(pump_on(&mut g, self.servo, 4_000_000, self.bus));
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
            let params = if self.virgin {
                RigParams::new(None, lim.abort_default())
            } else {
                RigParams::new(Some(lim.guard().unwrap()), lim.abort_default()).with_stops(lim.raw)
            };
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
                        self.moved = exp.moved_at();
                        run.nudged(exp.moved_at());
                    }
                    how
                }
                Stage::Burst { rungs, pre, seek } => {
                    let cfg = burst_cfg(rungs, *pre, *seek, base);
                    let (exp, how) = self.go(Inductance::new(cfg, &params, scales()), params);
                    self.caps.extend_from_slice(exp.captures());
                    let fit = fit_captures(&self.caps, &scales(), &FitCfg::default());
                    self.e8 = fit.clone();
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
                    let (exp, how) = self.go(ladder, params.abort_at_soft(lim.soft));
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
                    let inertia = Inertia::new(cfg, plan, runway, &params);
                    let (exp, how) = self.go(inertia, params.abort_at_soft(lim.soft));
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
                Stage::Stops {
                    approach,
                    seat,
                    cap,
                } => {
                    let p = params.without_pos_guard();
                    let exp = Endstop::new(endstop_cfg(*approach, *seat, *cap), &p);
                    let (exp, how) = self.go(Permitted::new(exp), p);
                    match exp.into_inner().result() {
                        Some(r) if how == Ended::Done && r.refusal().is_none() => {
                            self.stops = Some(r);
                            how
                        }
                        _ if how == Ended::Done => Ended::Declined,
                        _ => how,
                    }
                }
                Stage::Traverse { seek } => {
                    let r = self.stops.expect("the stops");
                    let stops = (r.pos_min_phys as u16, r.pos_max_phys as u16);
                    let soft = (!self.virgin).then_some((lim.soft.0 as u16, lim.soft.1 as u16));
                    let runway = Runway::new(crate::exp::sweep::guard(stops, soft));
                    let p = RigParams::new(None, lim.abort_default()).with_stops(stops);
                    let cfg = sweep_cfg(*seek, SweepCfg::default());
                    let (exp, how) = self.go(Sweep::new(cfg, &p, runway), p);
                    self.captured = exp.captured();
                    match exp.declined() {
                        Some(_) if how == Ended::Done => Ended::Declined,
                        _ => how,
                    }
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

    /// The whole default run on the bench servo at its measured speed and
    /// the bench bus's cadence, its slow reads seeded eight ways, on a
    /// 7.9 V pack and a fuller one, no slip zone: no drive leaves the travel
    /// or trips the abort, the ladder fits every rung to 55% both ways, its
    /// top speed over 2.2 times the bottom one's - a tenth of margin on the
    /// twice the fit needs - and the run ends centred with torque off.
    #[test]
    fn ladder_runs_at_bench_bus_timing() {
        let to_55 = &LadderCfg::default().rungs_q15[..5];
        for vbus in [RAIL_2S, 3300] {
            for seed in 1..=8 {
                let mut servo = bench_servo(vbus);
                servo.pos = 2600.0;
                let mut run = run(vbus);
                let mut rig = Rig::new(&mut servo, vbus);
                rig.bus = Bus::BENCH.with_slow_reads(seed);
                rig.run(&mut run);
                assert_eq!(run.aborted(), None, "{vbus} seed {seed}");
                assert_eq!(run.over(), None, "{vbus} seed {seed}");
                let ladder = rig.ladder.as_ref().expect("the ladder fits");
                for d in to_55 {
                    for d in [*d, -d] {
                        assert!(
                            ladder.rungs.iter().any(|r| r.duty_q15 == d && r.used),
                            "{vbus} seed {seed}: {d} not fitted: {:?}",
                            ladder.warnings
                        );
                    }
                }
                let speed = |d: i16| {
                    ladder
                        .rungs
                        .iter()
                        .filter(|r| r.used && r.duty_q15.abs() == d)
                        .map(|r| r.omega.abs())
                        .sum::<f64>()
                };
                let (lo, hi) = (speed(to_55[0]), speed(to_55[4]));
                assert!(
                    hi > 1.1 * crate::exp::ladder::MIN_SPAN * lo,
                    "{vbus} seed {seed}: {lo} {hi}"
                );
                assert!(!servo.torque && !servo.permit_live());
                assert!((servo.pos - 2029.0).abs() <= 300.0, "ends at {}", servo.pos);
            }
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

    const CAL_NAMES: [&str; 6] = ["bias", "centring", "burst", "stops", "traverse", "centring"];

    fn cal(vbus: u16) -> Run {
        Run::for_cal(limits(vbus), &scales())
    }

    /// The goal duties a log span writes, q15.
    fn goals(log: &[String]) -> Vec<i32> {
        log.iter()
            .filter_map(|l| l.strip_prefix("write goal_duty ")?.parse().ok())
            .collect()
    }

    /// Drive cal's sequencer, the jam check moving the shaft at 12%,
    /// answering each stage with `how`; the burst measures R only when it
    /// ends Done.
    fn cal_stages(run: &mut Run, mut how: impl FnMut(&Stage) -> Ended) -> Vec<&'static str> {
        let mut seen = Vec::new();
        while let Some(s) = run.next_stage() {
            seen.push(s.name());
            let h = how(&s);
            match s {
                Stage::Centre { nudge: true, .. } => run.nudged(Some(0.12)),
                Stage::Burst { .. } if h == Ended::Done => run.measured(R),
                _ => {}
            }
            run.ended(h);
        }
        seen
    }

    /// Cal's order and how it ends: a burst that declines, or a rail too
    /// high for one, goes on without R; stops that are no stops and a
    /// traverse that declines still end at mid travel; an abort ends where
    /// it stands.
    #[test]
    fn cal_order_and_its_ends() {
        let done = |_: &Stage| Ended::Done;
        let mut r = cal(RAIL_2S);
        assert_eq!(cal_stages(&mut r, done), CAL_NAMES);
        assert_eq!(r.over(), None);
        assert!(r.plan().is_some());

        let mut r = cal(RAIL_2S);
        let seen = cal_stages(&mut r, |s| match s {
            Stage::Burst { .. } => Ended::Declined,
            _ => Ended::Done,
        });
        assert_eq!(seen, CAL_NAMES);
        assert_eq!((r.over(), r.plan()), (None, None));

        let mut r = cal(3800);
        assert_eq!(
            cal_stages(&mut r, done),
            ["bias", "centring", "stops", "traverse", "centring"]
        );
        assert_eq!((r.over(), r.plan()), (None, None));

        let mut r = cal(RAIL_2S);
        let seen = cal_stages(&mut r, |s| match s {
            Stage::Stops { .. } => Ended::Declined,
            _ => Ended::Done,
        });
        assert_eq!(seen, ["bias", "centring", "burst", "stops", "centring"]);
        assert_eq!(r.over(), Some(Over::Declined("stops")));

        let mut r = cal(RAIL_2S);
        let seen = cal_stages(&mut r, |s| match s {
            Stage::Traverse { .. } => Ended::Declined,
            _ => Ended::Done,
        });
        assert_eq!(seen, CAL_NAMES);
        assert_eq!(r.over(), None);

        let blocked = AbortReason::Blocked {
            pos: 1300,
            moved: 0,
        };
        let mut r = cal(RAIL_2S);
        let seen = cal_stages(&mut r, |s| match s {
            Stage::Stops { .. } => Ended::Aborted(blocked),
            _ => Ended::Done,
        });
        assert_eq!(seen, CAL_NAMES[..4]);
        assert_eq!(r.aborted(), Some(("stops", blocked)));
    }

    /// The bench numbers: limit 280, r 1.775 vcounts per ccount, a shaft
    /// that first moved at 13%. On 2S the approach is 15%, under the 15.5%
    /// cap, and the seat 7.76%; on USB the approach is 15% too, under the
    /// 27.9% cap, and the seat 14.0%.
    #[test]
    fn cal_duties_on_the_bench_servo() {
        for (vbus, want_seat, want_cap) in [(RAIL_2S, 0.0776, 0.1551), (RAIL_USB, 0.1396, 0.2792)] {
            let mut r = cal(vbus);
            let mut stops = None;
            while let Some(s) = r.next_stage() {
                match s {
                    Stage::Centre { nudge: true, .. } => r.nudged(Some(0.13)),
                    Stage::Burst { .. } => r.measured(R),
                    Stage::Stops {
                        approach,
                        seat,
                        cap,
                    } => stops = Some((approach, seat, cap)),
                    _ => {}
                }
                r.ended(Ended::Done);
            }
            let (approach, seat, cap) = stops.expect("a stops stage");
            assert!((approach - 0.15).abs() < 1e-9, "{vbus}: {approach}");
            assert!((seat - want_seat).abs() < 1e-4, "{vbus}: {seat}");
            assert!((cap - want_cap).abs() < 1e-4, "{vbus}: {cap}");
            let stall = |d: f64| crate::limits::stall_counts(d, R, vbus as f64);
            assert!(stall(approach) <= LIM as f64);
            assert!((stall(seat) - LIM as f64 / 2.0).abs() < 1e-9);
        }
    }

    /// A normal cal on the bench servo, both rails, re-calibrating a servo
    /// whose stops are known: the jam check moves the shaft, the burst
    /// measures R, and each stop is approached at the duty that moved it
    /// plus the seek margin, capped at the stall-safe duty: on 2S the jam
    /// check's 14.5% plus 2% is over the cap the measured R allows, on USB
    /// 17.1% plus 2% is under it. The stall permit is held only while the
    /// stops are found; the run captures the ripple and ends at mid travel
    /// with torque off.
    #[test]
    fn cal_approaches_at_the_duty_that_moved() {
        for (vbus, moved, capped) in [(RAIL_2S, 0.1452, true), (RAIL_USB, 0.1713, false)] {
            let mut servo = bench_servo(vbus);
            servo.pos = 2600.0;
            let mut run = cal(vbus);
            let mut rig = Rig::new(&mut servo, vbus);
            rig.run(&mut run);
            assert_eq!(run.over(), None, "{vbus}: {:?}", run.over());
            let names: Vec<&str> = rig.marks.iter().map(|m| m.0).collect();
            assert_eq!(names, CAL_NAMES);
            let m = rig.moved.expect("the jam check moved the shaft");
            assert!((m - moved).abs() < 5e-4, "{vbus}: moved at {m}");
            let plan = run.plan().expect("R from the burst");
            let want = (m + DutyPlan::SEEK_MARGIN).min(plan.stop_cap);
            assert_eq!(want == plan.stop_cap, capped, "{vbus}: {want}");
            assert_eq!(
                goals(rig.span(3, 4))[0],
                -(q15_floor(want) as i32),
                "{vbus}"
            );
            let r = rig.stops.expect("the stops");
            assert_eq!((r.pos_min_phys, r.pos_max_phys), (209, 3849));
            assert!(r.drive_polarity);
            assert!(rig.captured.is_some(), "{vbus}: no ripple captured");
            let permit = |l: &String| l.starts_with("write stall_permit");
            assert!(rig.span(3, 4).iter().any(permit));
            assert!(!rig.span(0, 3).iter().any(permit) && !rig.span(4, 6).iter().any(permit));
            assert!(!servo.torque && !servo.permit_live());
            assert!((servo.pos - 2029.0).abs() <= 310.0, "ends at {}", servo.pos);
        }
    }

    /// A servo never calibrated: no stops known, no soft limits. The same
    /// run finds them, captures the ripple, and ends at mid travel.
    #[test]
    fn cal_finds_the_stops_on_a_virgin_servo() {
        let mut servo = bench_servo(RAIL_2S);
        servo.soft = None;
        servo.pos = 2600.0;
        let mut run = cal(RAIL_2S);
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.virgin = true;
        rig.run(&mut run);
        assert_eq!(run.over(), None, "{:?}", run.over());
        let r = rig.stops.expect("the stops");
        assert_eq!((r.pos_min_phys, r.pos_max_phys), (209, 3849));
        assert_eq!(r.refusal(), None);
        assert!(rig.captured.is_some());
        assert!(!servo.torque && !servo.permit_live());
        assert!((servo.pos - 2048.0).abs() <= 310.0, "ends at {}", servo.pos);
    }

    /// Replayed on the bench servo: toward a stop the only duty ever
    /// commanded is the one approach duty. The stop finder writes it, the
    /// lower seat duty and nothing else; the traverse's first pass to the
    /// low end runs at it, and so does the closing centring.
    #[test]
    fn cal_never_escalates_toward_a_stop() {
        for vbus in [RAIL_2S, RAIL_USB] {
            let mut servo = bench_servo(vbus);
            servo.pos = 2600.0;
            let mut run = cal(vbus);
            let mut rig = Rig::new(&mut servo, vbus);
            rig.run(&mut run);
            assert_eq!(run.over(), None, "{vbus}");
            let (approach, seat, _) = run.stop_duties().unwrap();
            let (a, s) = (q15_floor(approach) as i32, q15_floor(seat) as i32);
            let mut stops: Vec<i32> = goals(rig.span(3, 4)).iter().map(|d| d.abs()).collect();
            stops.sort_unstable();
            stops.dedup();
            assert_eq!(stops, [0, s, a], "{vbus}");
            let traverse = goals(rig.span(4, 5));
            assert_eq!(traverse[0], -a, "{vbus}: the first pass");
            let brake = crate::runway::BRAKE_DUTY_Q15 as i32;
            let run_q15 = SweepCfg::default().duty_q15 as i32;
            for d in &traverse {
                assert!([0, -a, brake, -brake, run_q15].contains(d), "{vbus}: {d}");
            }
            assert!(
                goals(rig.span(5, 6))
                    .iter()
                    .all(|d| d.abs() == a || *d == 0),
                "{vbus}: the closing centring"
            );
        }
    }

    /// A shaft that sticks until 15% on 2S: the jam check's raise past
    /// 14.5% moves it at 17%, whose stall draws 307 counts, over the
    /// 280-count limit. Once R is known cal refuses, before anything drives
    /// at a stop, and nothing holds the permit.
    #[test]
    fn cal_refuses_when_breakaway_is_over_the_limit() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2600.0;
        servo.breakaway_q15 = 4915;
        let mut run = cal(RAIL_2S);
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.run(&mut run);
        let names: Vec<&str> = rig.marks.iter().map(|m| m.0).collect();
        assert_eq!(names, CAL_NAMES[..3]);
        let Some(Over::BreakawayOverLimit { need }) = run.over() else {
            panic!("{:?}", run.over());
        };
        let m = rig.moved.unwrap();
        let r = run.plan().expect("R from the burst").r_vpc;
        assert!((m - 0.1702).abs() < 5e-4, "moved at {m}");
        let stall = crate::limits::stall_counts(m, r, RAIL_2S as f64);
        assert!((need - stall).abs() < 1e-9 && need > LIM as f64, "{need}");
        // by the true 4.9 ohm: 307 counts
        let lim = limits(RAIL_2S);
        let why = Refusal::BreakawayOverLimit {
            need: crate::limits::stall_counts(m, R, RAIL_2S as f64),
            i_lim: lim.i_lim,
            ma: lim.ma(),
        };
        assert_eq!(
            why.to_string(),
            "this servo needs about 275 mA to start moving, over its current limit of 251 mA: \
             raise the limit or free the mechanism"
        );
        assert!(!rig.log.iter().any(|l| l.starts_with("write stall_permit")));
        assert_eq!(servo.pressed_ms, 0.0);
        assert!(!servo.torque);
        assert!((servo.pos - 2029.0).abs() <= 310.0, "ends at {}", servo.pos);
    }

    /// The burst declines, so R is unknown: cal takes the stops without it,
    /// approached at the duty that moved the shaft and seated at half the
    /// class-safe duty, 4.76% on 2S, and still ends at mid travel.
    #[test]
    fn cal_without_r_seats_at_the_class_half_duty() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2600.0;
        let mut run = cal(RAIL_2S);
        let boot = run.bootstrap();
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.decline_burst = true;
        rig.run(&mut run);
        assert_eq!((run.over(), run.plan()), (None, None));
        let names: Vec<&str> = rig.marks.iter().map(|m| m.0).collect();
        assert_eq!(names, CAL_NAMES);
        let m = rig.moved.unwrap();
        assert!((boot / 2.0 - 0.0476).abs() < 1e-3);
        assert_eq!(
            goals(rig.span(3, 4))[..2],
            [-(q15_floor(m) as i32), -(q15_floor(boot / 2.0) as i32)]
        );
        let r = rig.stops.expect("the stops");
        assert_eq!((r.pos_min_phys, r.pos_max_phys), (209, 3849));
        assert!(!servo.torque && !servo.permit_live());
        assert!((servo.pos - 2029.0).abs() <= 310.0, "ends at {}", servo.pos);
    }

    /// A shaft locked at mid travel: the jam check raises to its cap and
    /// gives up, and cal ends there with the blocked message and torque
    /// off. Nothing but the drive's own control fields was ever written:
    /// no config, no calib, no position table, no permit.
    #[test]
    fn cal_on_a_jammed_shaft_writes_nothing() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2048.0;
        servo.jam = Some(2048.0);
        let mut run = cal(RAIL_2S);
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.run(&mut run);
        let names: Vec<&str> = rig.marks.iter().map(|m| m.0).collect();
        assert_eq!(names, ["bias", "centring"]);
        let (stage, reason) = run.aborted().expect("aborted");
        assert_eq!(stage, "centring");
        assert_eq!(
            reason.to_string(),
            "the shaft is blocked or the position sensor is not reading (pos 2048, moved 0 \
             counts)"
        );
        let controls =
            crate::regs::control::TORQUE_ENABLE.addr..=crate::regs::control::BURST_CHANS.addr;
        for l in &rig.log {
            let Some(name) = l.strip_prefix("write ").and_then(|w| w.split(' ').next()) else {
                panic!("not a write: {l}");
            };
            let reg = crate::regs::ALL.iter().find(|r| r.0 == name).unwrap().1;
            assert!(controls.contains(&reg.addr), "{l}");
            assert_ne!(name, "stall_permit");
        }
        assert!(!servo.torque && !servo.permit_live());
    }

    /// The bench servo as the CLI reads it once identified: R 7270 in its
    /// table.
    fn identified(vbus: u16) -> ServoLimits {
        ServoLimits {
            r_q12: 7270,
            ..limits(vbus)
        }
    }

    /// The winding an earlier identification left on the bench servo: R
    /// `r_vpc`, L 0.6 mH.
    fn stored(r_vpc: f64) -> Winding {
        let sc = scales();
        Winding {
            r_ohm: Some(r_vpc * sc.v_term_per_count / sc.amps_per_count),
            r_vpc,
            r_from: sources::Source::Stored,
            l_h: 0.6e-3,
            l_from: sources::Source::Stored,
        }
    }

    fn reusing(vbus: u16, w: Winding) -> Run {
        Run::new(identified(vbus), &scales()).with_stored_winding(Some(w))
    }

    fn names(rig: &Rig) -> Vec<&'static str> {
        rig.marks.iter().map(|m| m.0).collect()
    }

    /// The bench MG90 on 2S, identified before, its burst declined: the run
    /// plans from the winding it carries and goes on to the fit - the
    /// breakaway, a ladder that fits Ke, inertia steps that fit b - and
    /// ends centred with torque off. Nothing stalls a stop.
    #[test]
    fn a_declined_burst_uses_the_stored_winding() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2600.0;
        let mut run = reusing(RAIL_2S, stored(R));
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.decline_burst = true;
        rig.run(&mut run);
        assert_eq!(run.over(), None, "{:?}", run.over());
        assert_eq!(run.reused(), Some((stored(R), Over::Declined("burst"))));
        assert_eq!(names(&rig), ORDER_NAMES);
        assert_eq!(run.plan().expect("planned").r_vpc, R);
        assert_eq!(rig.r_ohm, None, "the burst supplied R");
        let ladder = rig.ladder.as_ref().expect("the ladder fits");
        assert!(ladder.rungs.iter().filter(|r| r.used).count() >= 3);
        assert!(rig.inertia_fit, "inertia did not fit");
        assert!(!rig.log.iter().any(|l| l.starts_with("write stall_permit")));
        assert_eq!(servo.pressed_ms, 0.0);
        assert!(!servo.torque && !servo.permit_live());
        assert!((servo.pos - 2029.0).abs() <= 300.0, "ends at {}", servo.pos);
    }

    /// Asked for, the stop ladder comes before the stored winding: on USB
    /// it has room and measures R from the stops, and the stored winding,
    /// a third off here, is never taken. On 2S the stored R leaves the
    /// ladder no room between the window floor and the stall-safe cap, so
    /// the run takes the stored winding instead, stalls nothing and goes on.
    #[test]
    fn stall_ladder_comes_before_the_stored_winding() {
        let mut servo = bench_servo(RAIL_USB);
        servo.pos = 2600.0;
        let mut run = reusing(RAIL_USB, stored(R * 1.3)).with_stall_ladder();
        let mut rig = Rig::new(&mut servo, RAIL_USB);
        rig.decline_burst = true;
        rig.run(&mut run);
        assert_eq!(run.over(), None, "{:?}", run.over());
        assert_eq!(run.reused(), None);
        assert_eq!(
            &names(&rig)[..4],
            ["bias", "centring", "burst", "resistance"]
        );
        let r = run.plan().expect("planned").r_vpc;
        assert!((r / R - 1.0).abs() < 0.02, "R {r}");
        assert!(rig.log.iter().any(|l| l.starts_with("write stall_permit")));
        assert!(!servo.torque && !servo.permit_live());

        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2600.0;
        let mut run = reusing(RAIL_2S, stored(R)).with_stall_ladder();
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.decline_burst = true;
        rig.run(&mut run);
        assert_eq!(run.over(), None, "{:?}", run.over());
        assert!(matches!(
            run.reused(),
            Some((w, Over::NoLadderRoom { .. })) if w == stored(R)
        ));
        assert_eq!(names(&rig), ORDER_NAMES);
        assert_eq!(run.plan().expect("planned").r_vpc, R);
        assert!(!rig.log.iter().any(|l| l.starts_with("write stall_permit")));
        assert_eq!(servo.pressed_ms, 0.0);
        assert!(!servo.torque && !servo.permit_live());
    }

    /// A servo that carries no winding: a declined burst ends the run
    /// after its closing centring with nothing planned, the stop ladder
    /// asked for or not - 2S leaves it no room.
    #[test]
    fn no_stored_winding_stops_the_run() {
        for ladder in [false, true] {
            let mut servo = bench_servo(RAIL_2S);
            servo.pos = 2600.0;
            let mut run = Run::new(identified(RAIL_2S), &scales()).with_stored_winding(None);
            if ladder {
                run = run.with_stall_ladder();
            }
            let mut rig = Rig::new(&mut servo, RAIL_2S);
            rig.decline_burst = true;
            rig.run(&mut run);
            assert_eq!(names(&rig), ["bias", "centring", "burst", "centring"]);
            if ladder {
                assert!(matches!(run.over(), Some(Over::NoLadderRoom { .. })));
            } else {
                assert_eq!(run.over(), Some(Over::Declined("burst")));
            }
            assert_eq!((run.plan(), run.reused()), (None, None));
            assert_eq!(run.next_stage(), None, "the run stays over");
            assert!(!rig.log.iter().any(|l| l.starts_with("write stall_permit")));
            assert!(!servo.torque && !servo.permit_live());
        }
    }

    /// The declined burst still carries a rough R from its pairs: a stored
    /// winding it agrees with passes quietly, one more than a quarter away
    /// either way is called stale.
    #[test]
    fn a_stale_stored_winding_warns() {
        let mut servo = bench_servo(RAIL_2S);
        servo.pos = 2600.0;
        let mut run = reusing(RAIL_2S, stored(R));
        let mut rig = Rig::new(&mut servo, RAIL_2S);
        rig.decline_burst = true;
        while let Some(stage) = run.next_stage() {
            let burst = matches!(stage, Stage::Burst { .. });
            let how = rig.execute(&mut run, &stage);
            run.ended(how);
            if burst {
                break;
            }
        }
        let e8 = rig.e8.as_ref().expect("the burst fitted");
        let rough = e8.volts.r_pair_ohm.or(e8.r_pair_ohm).expect("a rough R");
        let r_ohm = stored(R).r_ohm.unwrap();
        assert!(
            (rough / r_ohm - 1.0).abs() < sources::STALE_R,
            "{rough} of {r_ohm}"
        );
        assert_eq!(sources::stale(Some(e8), &stored(R)), None);
        assert_eq!(sources::stale(Some(e8), &stored(R * 1.4)), Some(rough));
        assert_eq!(sources::stale(Some(e8), &stored(R / 1.4)), Some(rough));
        assert_eq!(sources::stale(None, &stored(R * 1.4)), None);
    }

    /// A declined burst that leaves the shaft off mid travel, as on the
    /// bench at 1457. With no stored winding the closing centring starts at
    /// the class-safe duty and raises no higher than the jam check's cap;
    /// with one it drives at the plan's seek, under its stop cap. Either
    /// way the run ends centred with torque off.
    #[test]
    fn a_declined_burst_still_ends_centred() {
        for w in [None, Some(stored(R))] {
            let mut servo = bench_servo(RAIL_2S);
            servo.pos = 2600.0;
            let mut run = Run::new(identified(RAIL_2S), &scales()).with_stored_winding(w);
            let (boot, cap) = (run.bootstrap(), run.nudge_cap());
            let mut rig = Rig::new(&mut servo, RAIL_2S);
            rig.decline_burst = true;
            let mut closing = None;
            while let Some(stage) = run.next_stage() {
                if names(&rig).last() == Some(&"burst") {
                    closing = Some(stage.clone());
                }
                let burst = matches!(stage, Stage::Burst { .. });
                let how = rig.execute(&mut run, &stage);
                run.ended(how);
                if burst {
                    rig.servo.pos = 1457.0;
                    run.finish();
                }
            }
            assert_eq!(names(&rig), ["bias", "centring", "burst", "centring"]);
            let top = drives(rig.span(3, 4)).into_iter().fold(0.0, f64::max);
            match (w, run.plan()) {
                (None, None) => {
                    let want = Stage::Centre {
                        duty: boot,
                        cap,
                        nudge: false,
                    };
                    assert_eq!(closing, Some(want));
                    assert!(top <= cap + 1e-9, "{top} over {cap}");
                    assert_eq!(run.over(), Some(Over::Declined("burst")));
                }
                (Some(_), Some(p)) => {
                    let want = Stage::Centre {
                        duty: p.seek,
                        cap: p.stop_cap,
                        nudge: false,
                    };
                    assert_eq!(closing, Some(want));
                    assert!(top <= p.stop_cap + 1e-9, "{top} over {}", p.stop_cap);
                    assert_eq!(run.over(), None);
                }
                other => panic!("{other:?}"),
            }
            assert!(!servo.torque && !servo.permit_live());
            assert!((servo.pos - 2029.0).abs() <= 300.0, "ends at {}", servo.pos);
        }
    }
}
