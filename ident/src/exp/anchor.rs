//! The thermometer's anchor: the kernel's own winding R at rest. Seek the
//! low stop at the plan's seek duty, seat, hold at the stall-safe stop
//! duty (the current limit governs it) and read the ident aggregate's
//! windows: duty x vdiff / i per window is the quantity the kernel's
//! thermometer LMS settles on (core `estimator::thermal`), so their median
//! is `r0_q12` on the thermometer's own scale ([`crate::thermometer`]).
//! Runs with the pos guard off and the stall permit held, like the
//! resistance stop ladder; a seek that rests anywhere but the stop ends
//! the run ([`super::seek::at_stop`]). The hold is about a second: 0.3 W
//! into the winding for a second is well under a degree of self-heating,
//! and the current at the limit is where a loaded hold sits in service.

use super::{AbortReason, Cmd, Experiment, RigParams, WindowSample, WindowStream, seek};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

const Q15: f64 = 32767.0;

pub struct AnchorCfg {
    /// Seek toward the low stop, q15, applied negative.
    pub seek_duty_q15: i16,
    /// The hold at the stop, q15: the stall-safe cap, so the limit governs.
    pub hold_duty_q15: i16,
    /// Polls at the hold; with `poll_ms` about a second.
    pub hold_polls: u32,
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
}

impl Default for AnchorCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 3932,
            hold_duty_q15: 5242,
            hold_polls: 400,
            poll_ms: 2,
            seek_poll_ms: 30,
        }
    }
}

/// Windows a hold must read before its median stands: a hold the limiter
/// is still slewing into, or one the poll budget cut, reads fewer.
pub const MIN_WINDOWS: usize = 100;

/// Per-window scatter of R over this fraction of the median says the
/// shaft was not at rest against the stop (back-EMF in the windows) or
/// the limiter hunted: healthy holds read under 2%.
pub const MAX_SPREAD: f64 = 0.05;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AnchorResult {
    /// The median window R, vcounts per ccount: `r0_q12`.
    pub r_vpc: f64,
    /// Per-window standard deviation of R over the median.
    pub spread: f64,
    pub n: usize,
    /// The median hold current, counts, into the stop.
    pub i_counts: f64,
    /// The median applied duty at the hold, fraction of full scale.
    pub duty: f64,
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
    seat: Option<u16>,
}

/// Toward the low stop.
const DIR: i8 = -1;

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
            seat: None,
        }
    }

    pub fn samples(&self) -> &[WindowSample] {
        &self.samples
    }

    /// Where the shaft seated, once it did.
    pub fn seat(&self) -> Option<u16> {
        self.seat
    }

    pub fn result(&self) -> Result<AnchorResult, String> {
        Self::fit_samples(&self.samples)
    }

    /// The median window R and its scatter; Err says why the hold is no
    /// anchor. Associated so the CLI refits offline from recorded samples.
    pub fn fit_samples(samples: &[WindowSample]) -> Result<AnchorResult, String> {
        // duty and vdiff carry the drive's sign, i is folded by it, so
        // every term is positive into the stop
        let rs: Vec<f64> = samples
            .iter()
            .filter(|w| w.i != 0.0 && w.duty_q15 != 0.0)
            .map(|w| (w.duty_q15 * w.vdiff / Q15) / (w.i * w.duty_q15.signum()))
            .collect();
        if rs.len() < MIN_WINDOWS {
            return Err(format!(
                "the hold read {} windows with current, under the {MIN_WINDOWS} an anchor needs",
                rs.len()
            ));
        }
        let r = median(&rs);
        let sd = (rs.iter().map(|x| (x - r).powi(2)).sum::<f64>() / (rs.len() - 1) as f64).sqrt();
        let spread = sd / r;
        if !r.is_finite() || r <= 0.0 || spread > MAX_SPREAD {
            return Err(format!(
                "the hold's windows scatter {:.1}% around {r:.4} vcounts/ccount, over the \
                 {:.0}% of a shaft at rest against the stop",
                spread * 100.0,
                MAX_SPREAD * 100.0
            ));
        }
        let i: Vec<f64> = samples.iter().map(|w| w.i.abs()).collect();
        let duty: Vec<f64> = samples.iter().map(|w| w.duty_q15.abs() / Q15).collect();
        Ok(AnchorResult {
            r_vpc: r,
            spread,
            n: rs.len(),
            i_counts: median(&i),
            duty: median(&duty),
        })
    }
}

fn median(xs: &[f64]) -> f64 {
    let mut v = xs.to_vec();
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n == 0 {
        0.0
    } else if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
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
                self.polls_left = self.cfg.hold_polls;
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
                if let Some(o) = obs
                    && let Some(w) = self.windows.push(o)
                {
                    self.samples.push(w);
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.polls_left == 0 {
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
    fn bench_cfg() -> AnchorCfg {
        AnchorCfg {
            seek_duty_q15: 4915,
            ..AnchorCfg::default()
        }
    }

    /// The fake stalls through R alone: the hold's windows read it back
    /// as duty x vdiff / i, seated at the low stop, whether or not the
    /// limiter governs the hold.
    #[test]
    fn reads_the_planted_r_at_the_low_stop() {
        for (mut servo, cfg, name) in [
            (FakeServo::new(3.37), AnchorCfg::default(), "free limit"),
            (bench_mg90(3204), bench_cfg(), "limit 280"),
        ] {
            servo.r = 3.37;
            let (exp, log, abort) = run(&mut servo, cfg);
            assert_eq!(abort, None, "{name}");
            assert!(!log.contains(&"OVERRUN".to_string()), "{name}");
            let seat = exp.seat().expect("seated");
            assert!(seat <= 200 + seek::STOP_TOL, "{name}: seat {seat}");
            let r = exp.result().unwrap_or_else(|e| panic!("{name}: {e}"));
            assert!((r.r_vpc - 3.37).abs() < 0.02, "{name}: r {}", r.r_vpc);
            assert!(r.spread < 0.01, "{name}: spread {}", r.spread);
            assert!(r.n >= MIN_WINDOWS, "{name}: n {}", r.n);
            assert!(r.i_counts > 0.0 && r.duty > 0.0, "{name}: {r:?}");
        }
    }

    /// At the bench limit the hold draws the limit, not the cap's stall:
    /// the hold current the anchor reports is what the thermometer samples
    /// in service.
    #[test]
    fn the_limit_governs_the_hold() {
        let mut servo = bench_mg90(3204);
        servo.r = 3.37;
        // 30%: a 3.37 winding stalls 285 counts on 3204, over the 280 limit
        let cfg = AnchorCfg {
            hold_duty_q15: 9830,
            ..bench_cfg()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let r = exp.result().unwrap();
        assert!(r.i_counts <= 281.0, "hold current {}", r.i_counts);
        assert!((r.r_vpc - 3.37).abs() < 0.02, "r {}", r.r_vpc);
    }

    #[test]
    fn command_choreography_is_safe() {
        let mut servo = FakeServo::new(3.37);
        let (_, log, _) = run(&mut servo, AnchorCfg::default());
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
        let tail: Vec<&String> = log.iter().rev().take(4).collect();
        assert_eq!(*tail[3], "write goal_duty 0");
        assert_eq!(*tail[2], "write torque_enable 0");
        assert_eq!(*tail[1], "write stall_permit 0");
        assert_eq!(*tail[0], "write ident_agg 0");
    }

    /// A shaft that rests short of the stop is blocked: no hold is
    /// commanded against a jam, and nothing is anchored.
    #[test]
    fn a_mid_travel_jam_is_not_a_stop() {
        let mut servo = FakeServo::new(3.37);
        servo.jam = Some(1500.0);
        let (exp, log, abort) = run(&mut servo, AnchorCfg::default());
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
        assert!(exp.result().is_err());
    }

    #[test]
    fn too_few_or_scattered_windows_are_no_anchor() {
        let w = |i: f64, vdiff: f64| WindowSample {
            t_ms: 0.0,
            i,
            vdiff,
            duty_q15: -16_000.0,
        };
        let few: Vec<WindowSample> = (0..MIN_WINDOWS - 1).map(|_| w(-200.0, -1380.0)).collect();
        assert!(Anchor::fit_samples(&few).unwrap_err().contains("windows"));
        let mut scattered = few.clone();
        scattered.extend((0..40).map(|k| w(-200.0 - 40.0 * (k % 2) as f64, -1380.0)));
        assert!(
            Anchor::fit_samples(&scattered)
                .unwrap_err()
                .contains("scatter")
        );
        let mut tidy = few;
        tidy.push(w(-200.0, -1380.0));
        let r = Anchor::fit_samples(&tidy).unwrap();
        // 16000/32767 x 1380 / 200 = 3.3686
        assert!((r.r_vpc - 3.3686).abs() < 1e-3, "{}", r.r_vpc);
        assert_eq!(r.spread, 0.0);
        assert_eq!(r.i_counts, 200.0);
    }
}
