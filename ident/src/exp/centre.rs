//! Centre: drive the shaft into a band at mid travel, then duty 0 and
//! torque off. With `nudge` the shaft is also driven [`SEEK_TRAVEL_MIN`]
//! out and back once it is in the band: the jam check, the cheapest proof,
//! before anything else drives, that the shaft moves and the pot reads it.
//!
//! A still window raises the duty instead of ending the run - by
//! [`NUDGE_STEP_Q15`] while the shaft is clear of both stops
//! ([`seek::clear_of_stops`]), by [`SEEK_STEP_Q15`] while leaving one - up
//! to `cap_q15`. The first travel holds the duty: it is the run's estimate
//! of what moves the shaft ([`Centre::moved_at`]). A shaft still at the cap,
//! still after it travelled, still anywhere a raise is not allowed, or held
//! by the firmware's stall fold, is blocked and ends the run. With the stops
//! unknown the nudge tries the other direction once before that. Run it
//! with the pos guard off: the shaft may start outside it.

use super::seek::{self, SEEK_STEP_Q15, SEEK_TRAVEL_MIN, Watch};
use super::{AbortReason, Cmd, Experiment, LIMIT_YIELD_FOLDED, RigParams};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

/// Mid travel with no soft guard configured: the pot's own midpoint.
const POT_MID: u16 = 2048;

/// Raise per still window while clear of the stops: 2.5% of full scale.
pub const NUDGE_STEP_Q15: i16 = 819;

#[derive(Clone, Debug)]
pub struct CentreCfg {
    pub duty_q15: i16,
    /// The most a raise may reach.
    pub cap_q15: i16,
    pub nudge: bool,
    /// Half-width of the band around mid travel, counts.
    pub margin: u16,
    pub poll_ms: u32,
    /// Poll budget: 250 x 20 ms is 5 s of travel.
    pub polls_max: u32,
}

impl Default for CentreCfg {
    fn default() -> Self {
        Self {
            duty_q15: 3932,
            cap_q15: 3932,
            nudge: false,
            margin: 300,
            poll_ms: 20,
            polls_max: 250,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Leg {
    Toward,
    /// The nudge: out from `from` driving `dir`, then back to it.
    Out {
        from: u16,
        dir: i8,
    },
    Back {
        from: u16,
        dir: i8,
    },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Phase {
    FirstRead,
    Look,
    ModeWrite,
    TorqueOn,
    Read,
    Eval,
    Wait,
    FinishDuty,
    FinishTorque,
    Finished,
}

pub struct Centre {
    cfg: CentreCfg,
    params: RigParams,
    band: (u16, u16),
    phase: Phase,
    leg: Leg,
    mag: i16,
    watch: Option<Watch>,
    polls: u32,
    nudged: bool,
    flipped: bool,
    moved_at: Option<i16>,
    arrived: bool,
    halt: Option<AbortReason>,
}

impl Centre {
    pub fn new(cfg: CentreCfg, params: &RigParams) -> Self {
        let mid = params.pos_guard.map_or(POT_MID, |(lo, hi)| (lo + hi) / 2);
        let band = (
            mid.saturating_sub(cfg.margin),
            mid.saturating_add(cfg.margin),
        );
        Self {
            mag: cfg.duty_q15,
            cfg,
            params: *params,
            band,
            phase: Phase::FirstRead,
            leg: Leg::Toward,
            watch: None,
            polls: 0,
            nudged: false,
            flipped: false,
            moved_at: None,
            arrived: false,
            halt: None,
        }
    }

    /// The shaft ended in the band (nudged out and back, when asked).
    pub fn arrived(&self) -> bool {
        self.arrived
    }

    /// The largest duty the shaft first travelled at, a fraction of full
    /// scale; None until it travelled.
    pub fn moved_at(&self) -> Option<f64> {
        self.moved_at.map(|q| q as f64 / 32767.0)
    }

    pub fn band(&self) -> (u16, u16) {
        self.band
    }

    fn in_band(&self, pos: u16) -> bool {
        (self.band.0..=self.band.1).contains(&pos)
    }

    /// Out toward mid travel, so the nudge stays in the band.
    fn nudge_from(&mut self, pos: u16) {
        let mid = (self.band.0 + self.band.1) / 2;
        let dir = if pos < mid { 1 } else { -1 };
        self.leg = Leg::Out { from: pos, dir };
        self.watch = None;
    }

    fn drive(&mut self, dir: i8) -> Cmd {
        self.phase = Phase::Wait;
        Cmd::Write {
            reg: control::GOAL_DUTY,
            value: dir as i32 * self.mag as i32,
        }
    }

    fn finish(&mut self, arrived: bool) -> Cmd {
        self.arrived = arrived;
        self.phase = Phase::FinishDuty;
        Cmd::Pause { ms: 0 }
    }

    fn moved(&mut self) {
        self.moved_at = Some(self.moved_at.map_or(self.mag, |m| m.max(self.mag)));
    }

    fn blocked(&mut self, start: u16, pos: u16) -> Cmd {
        self.halt = Some(seek::blocked(start, pos));
        self.finish(false)
    }

    /// One poll of the leg under way.
    fn eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        let pos = o.pos;
        self.polls += 1;
        if self.polls > self.cfg.polls_max {
            return self.finish(false);
        }
        let dir = match self.leg {
            Leg::Toward if self.in_band(pos) && self.cfg.nudge && !self.nudged => {
                self.nudge_from(pos);
                self.polls -= 1;
                return self.eval(o);
            }
            Leg::Toward if self.in_band(pos) => return self.finish(true),
            Leg::Toward => {
                if pos < self.band.0 {
                    1
                } else {
                    -1
                }
            }
            Leg::Out { from, dir } if pos.abs_diff(from) >= SEEK_TRAVEL_MIN => {
                self.moved();
                self.leg = Leg::Back { from, dir: -dir };
                self.watch = None;
                -dir
            }
            Leg::Out { dir, .. } => dir,
            Leg::Back { from, dir } if (pos as i32 - from as i32) * dir as i32 >= 0 => {
                self.moved();
                self.nudged = true;
                self.leg = Leg::Toward;
                self.watch = None;
                self.polls -= 1;
                return self.eval(o);
            }
            Leg::Back { dir, .. } => dir,
        };
        let (eps, polls) = (self.params.stall_eps, self.params.stall_polls);
        let watch = self.watch.get_or_insert(Watch::new(pos, eps, polls));
        let start = watch.start();
        let still = watch.still(pos);
        let travelled = pos.abs_diff(start) >= SEEK_TRAVEL_MIN;
        if travelled {
            self.moved();
        }
        if o.limit_flags & LIMIT_YIELD_FOLDED != 0 {
            return self.blocked(start, pos);
        }
        if still {
            let stops = self.params.stops;
            let step = if seek::clear_of_stops(pos, stops) {
                Some(NUDGE_STEP_Q15)
            } else if seek::leaves_stop(start, dir, stops) {
                Some(SEEK_STEP_Q15)
            } else {
                None
            };
            match step.map(|s| self.mag.saturating_add(s)) {
                Some(next) if !travelled && next <= self.cfg.cap_q15 => self.mag = next,
                _ => match self.leg {
                    Leg::Out { from, dir } if stops.is_none() && !travelled && !self.flipped => {
                        self.flipped = true;
                        self.mag = self.cfg.duty_q15;
                        self.leg = Leg::Out { from, dir: -dir };
                        self.watch = None;
                        return self.drive(-dir);
                    }
                    _ => return self.blocked(start, pos),
                },
            }
        }
        self.drive(dir)
    }
}

impl Experiment for Centre {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::FirstRead => {
                self.phase = Phase::Look;
                Cmd::Read
            }
            // Nothing to do leaves torque alone.
            Phase::Look => {
                let Some(o) = obs else {
                    self.phase = Phase::Finished;
                    return Cmd::Done;
                };
                if self.in_band(o.pos) && !self.cfg.nudge {
                    self.arrived = true;
                    self.phase = Phase::Finished;
                    return Cmd::Done;
                }
                self.phase = Phase::ModeWrite;
                Cmd::Pause { ms: 0 }
            }
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
                None => self.finish(false),
            },
            Phase::Wait => {
                self.phase = Phase::Read;
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
    use super::super::Guarded;
    use super::super::testkit::{FakeServo, pump, rig};
    use super::*;

    const CAP: i16 = 8192;

    fn run(servo: &mut FakeServo, nudge: bool) -> (Guarded<Centre>, Vec<String>) {
        run_with(servo, nudge, rig())
    }

    fn run_with(
        servo: &mut FakeServo,
        nudge: bool,
        params: RigParams,
    ) -> (Guarded<Centre>, Vec<String>) {
        let cfg = CentreCfg {
            nudge,
            cap_q15: CAP,
            ..CentreCfg::default()
        };
        let mut exp = Guarded::new(Centre::new(cfg, &params), params.without_pos_guard());
        let log = pump(&mut exp, servo, 100_000);
        (exp, log)
    }

    fn duties(log: &[String]) -> Vec<i32> {
        log.iter()
            .filter_map(|l| l.strip_prefix("write goal_duty "))
            .map(|v| v.parse().unwrap())
            .collect()
    }

    #[test]
    fn centres_from_either_side_and_torques_off() {
        for start in [600.0, 3500.0] {
            let mut s = FakeServo::new(3.37);
            s.pos = start;
            let (exp, log) = run(&mut s, false);
            assert!(exp.abort().is_none(), "{:?}", exp.abort());
            let band = exp.into_inner();
            assert!(band.arrived());
            assert!((band.band().0 as f64..=band.band().1 as f64).contains(&s.pos));
            assert_eq!(
                &log[log.len() - 2..],
                ["write goal_duty 0", "write torque_enable 0"]
            );
            assert!(!s.torque);
        }
    }

    #[test]
    fn a_centred_shaft_is_left_alone_unless_nudged() {
        let mut s = FakeServo::new(3.37);
        s.pos = 2050.0;
        let (exp, log) = run(&mut s, false);
        assert!(exp.into_inner().arrived());
        assert!(log.is_empty(), "{log:?}");

        let (exp, log) = run(&mut s, true);
        assert!(exp.abort().is_none());
        let exp = exp.into_inner();
        assert!(exp.arrived());
        assert_eq!(exp.moved_at(), Some(3932.0 / 32767.0));
        let d = duties(&log);
        assert!(
            d.contains(&3932) && d.contains(&-3932),
            "out and back: {d:?}"
        );
    }

    /// A shaft that needs more than the start duty: clear of the stops the
    /// still window raises it, and the duty that first moved the shaft is
    /// held and reported. Near a stop nothing raises it.
    #[test]
    fn nudge_raises_the_duty_only_clear_of_the_stops() {
        let mut s = FakeServo::new(3.37);
        s.pos = 2050.0;
        s.breakaway_q15 = 5500;
        let (exp, log) = run(&mut s, true);
        assert!(exp.abort().is_none(), "{:?}", exp.abort());
        let exp = exp.into_inner();
        let moved = (3932 + 2 * NUDGE_STEP_Q15) as f64 / 32767.0;
        assert_eq!(exp.moved_at(), Some(moved));
        let d = duties(&log);
        assert_eq!(d.iter().map(|d| d.abs()).max(), Some(3932 + 2 * 819));
        assert!(exp.arrived());

        // 250 counts from the low stop: not clear of it, not at it
        let mut s = FakeServo::new(3.37);
        s.pos = 450.0;
        s.breakaway_q15 = 5500;
        let (exp, log) = run(&mut s, true);
        assert_eq!(
            exp.abort(),
            Some(AbortReason::Blocked { pos: 450, moved: 0 })
        );
        assert!(duties(&log).iter().all(|d| d.abs() <= 3932), "{log:?}");
    }

    /// The jam check on a shaft locked at mid travel: raised window by
    /// window to the cap, then blocked. The run ends at the cap, never over.
    #[test]
    fn nudge_gives_up_at_the_cap() {
        let mut s = FakeServo::new(3.37);
        s.pos = 2048.0;
        s.jam = Some(2048.0);
        let (exp, log) = run(&mut s, true);
        assert_eq!(
            exp.abort(),
            Some(AbortReason::Blocked {
                pos: 2048,
                moved: 0
            })
        );
        let d = duties(&log);
        let top = d.iter().map(|d| d.abs()).max().unwrap();
        assert!(
            top <= CAP as i32 && top + NUDGE_STEP_Q15 as i32 > CAP as i32,
            "{d:?}"
        );
        assert!(!s.torque);
        assert_eq!(exp.into_inner().moved_at(), None);
    }

    /// A jam the limiter holds at the current limit: the stall timer folds
    /// it to the yield before the raises reach the cap, and the jam check
    /// takes the fold as its verdict at once.
    #[test]
    fn a_yield_fold_ends_the_nudge_blocked() {
        let mut s = FakeServo::new(3.37);
        s.pos = 2048.0;
        s.jam = Some(2048.0);
        // 3932 stalls at 62 counts, the first raise at 75
        s.current_limit = Some(70);
        s.stall_ms = Some(100.0);
        s.stall_yield = 40;
        let (exp, log) = run(&mut s, true);
        assert_eq!(
            exp.abort(),
            Some(AbortReason::Blocked {
                pos: 2048,
                moved: 0
            })
        );
        let top = duties(&log).iter().map(|d| d.abs()).max().unwrap();
        assert_eq!(top, 3932 + NUDGE_STEP_Q15 as i32, "ended by the fold");
        assert_eq!(
            &log[log.len() - 2..],
            ["write goal_duty 0", "write torque_enable 0"]
        );
        assert!(!s.torque && s.limit_flags() & LIMIT_YIELD_FOLDED == 0);
    }

    /// A shaft creeping under the stillness speed at the start duty (USB at
    /// 7%: 130 counts/s) gets a raise, not a verdict.
    #[test]
    fn slow_seek_is_raised_not_blocked() {
        let mut s = FakeServo::new(3.37);
        s.pos = 1500.0;
        // 12% creeps at 110 counts/s, under the window's 150; 14.5% runs at 360
        s.physical_motion = true;
        s.fc = 56.0;
        let (exp, log) = run(&mut s, false);
        assert!(exp.abort().is_none(), "{:?}", exp.abort());
        assert!(exp.into_inner().arrived());
        assert!(duties(&log).iter().any(|d| *d > 3932), "never raised");
    }

    /// Stops unknown: the nudge that cannot move one way tries the other.
    #[test]
    fn unknown_stops_try_the_other_direction() {
        let mut s = FakeServo::new(3.37);
        s.pos = 2050.0;
        s.jam = Some(2050.0);
        let params = RigParams {
            stops: None,
            ..rig()
        };
        let (exp, log) = run_with(&mut s, true, params);
        assert!(matches!(exp.abort(), Some(AbortReason::Blocked { .. })));
        let d = duties(&log);
        assert!(d.iter().any(|d| *d > 0));
        assert!(d.iter().any(|d| *d < 0), "never tried the other way: {d:?}");
    }
}
