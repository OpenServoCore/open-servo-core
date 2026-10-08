//! The thermometer's hold: the kernel's own winding R at a seat. Seek the
//! low stop at the plan's seek duty, seat, hold at the stall-safe stop
//! duty (the current limit governs it) and read the ident aggregate's
//! windows: duty x vdiff / i per window is the quantity the kernel's
//! thermometer LMS settles on (core `estimator::thermal`), so their median
//! is a seat's R on the thermometer's own scale; after a long idle it
//! becomes the stored cold R ([`crate::thermometer::cold_r_q12`]).
//! Runs with the pos guard off and the stall permit held, like the
//! resistance stop ladder; a seek that rests anywhere but the stop ends
//! the run ([`super::seek::at_stop`]). The hold is about a second: 0.3 W
//! into the winding for a second is well under a degree of self-heating,
//! and the current at the limit is where a loaded hold sits in service.
//! With `hold_ms` the hold runs that long and its reads are the thermal
//! fit's rows ([`crate::thermal`]).

use super::{AbortReason, Cmd, Experiment, RigParams, WindowSample, WindowStream, seek};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;
use crate::thermal::Row;

const Q15: f64 = 32767.0;

pub struct AnchorCfg {
    /// Seek toward the low stop, q15, applied negative.
    pub seek_duty_q15: i16,
    /// The hold at the stop, q15: the stall-safe cap, so the limit governs.
    pub hold_duty_q15: i16,
    /// Polls at the hold; with `poll_ms` about a second.
    pub hold_polls: u32,
    /// The hold ends this long after its first read, if sooner, ms.
    pub hold_ms: Option<f64>,
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
}

impl Default for AnchorCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 3932,
            hold_duty_q15: 5242,
            hold_polls: 400,
            hold_ms: None,
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

/// A hold whose R sits this far off the identified winding's, or whose
/// duty at the hold current sits this far under what that R and the rail
/// predict, is not the winding: a brush bridging two segments read R0
/// 0.7705 vcounts/ccount at 12.3% duty on the bench MG90, 23% under the
/// 0.97-1.01 of every other anchor that day.
pub const CONTACT_BAND: f64 = 0.15;

/// The hold against the identified winding `r_vpc` on the rail `vbus`,
/// vcounts: `hold_r` vcounts/ccount, `duty` a fraction, at `i_counts`.
/// Err is the refusal.
pub fn contact_state(
    hold_r: f64,
    duty: f64,
    i_counts: f64,
    r_vpc: f64,
    vbus: f64,
) -> Result<(), String> {
    let predicted = crate::limits::duty_for(i_counts, r_vpc, vbus);
    let low = hold_r < r_vpc * (1.0 - CONTACT_BAND) || duty < predicted * (1.0 - CONTACT_BAND);
    if low {
        return Err(format!(
            "the hold read R0 {hold_r:.4} against the identified R {r_vpc:.4}: a \
             low-resistance contact state (a brush bridging two segments); re-seat and retry"
        ));
    }
    if hold_r > r_vpc * (1.0 + CONTACT_BAND) {
        return Err(format!(
            "the hold read R0 {hold_r:.4} against the identified R {r_vpc:.4}, over {:.0}% \
             above it; re-seat and retry",
            CONTACT_BAND * 100.0
        ));
    }
    Ok(())
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AnchorResult {
    /// The median window R, vcounts per ccount, at the hold's temperature.
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
    rows: Vec<Row>,
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
            rows: Vec::new(),
            seat: None,
        }
    }

    pub fn samples(&self) -> &[WindowSample] {
        &self.samples
    }

    /// Every read of the hold, in order.
    pub fn rows(&self) -> &[Row] {
        &self.rows
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
                let mut timed_out = false;
                if let Some(o) = obs {
                    if let Some(w) = self.windows.push(o) {
                        self.samples.push(w);
                    }
                    let start = self.rows.first().map_or(o.host_ms, |r| r.t_s * 1000.0);
                    timed_out = self.cfg.hold_ms.is_some_and(|ms| o.host_ms - start >= ms);
                    self.rows.push(Row::from_snapshot(o));
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.polls_left == 0 || timed_out {
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

    /// The hold the limit governs at 280 leaves the thermometer's floor at
    /// 7/8 of it.
    #[test]
    fn anchor_sets_the_floor_under_its_hold() {
        let mut servo = bench_mg90(3204);
        servo.r = 3.37;
        let cfg = AnchorCfg {
            hold_duty_q15: 9830,
            ..bench_cfg()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let r = exp.result().unwrap();
        let hold = r.i_counts.round() as u16;
        assert!((270..=281).contains(&hold), "hold current {}", r.i_counts);
        let floor = crate::thermometer::floor_counts(r.i_counts);
        assert_eq!(floor.raw, 7 * hold / 8);
        assert!(!floor.saturated);
    }

    /// A timed hold runs its span on the host's clock, whatever the poll
    /// budget, and keeps every read as a thermal row.
    #[test]
    fn a_timed_hold_runs_its_span_and_keeps_its_reads() {
        let mut servo = FakeServo::new(3.37);
        let cfg = AnchorCfg {
            hold_polls: u32::MAX,
            hold_ms: Some(3000.0),
            poll_ms: 16,
            ..AnchorCfg::default()
        };
        let (exp, _, abort) = run(&mut servo, cfg);
        assert_eq!(abort, None);
        let rows = exp.rows();
        let span = rows.last().unwrap().t_s - rows[0].t_s;
        assert!((3.0..3.1).contains(&span), "{span}");
        assert!(rows.len() > 100, "{}", rows.len());
        assert!(rows.iter().skip(10).all(|r| r.p > 0.0), "{:?}", &rows[..12]);
    }

    /// The bench's bridged anchor (R0 0.7705 at 12.3% for 275 counts)
    /// against the identified 1.00 vcounts/ccount on a 1780-count rail is
    /// refused on its R and on its duty alike; an ordinary seat passes.
    #[test]
    fn a_hold_in_a_low_resistance_contact_state_is_refused() {
        let (r, vbus, i) = (1.0, 1780.0, 275.0);
        let ok = crate::limits::duty_for(i, 0.99, vbus);
        assert_eq!(contact_state(0.99, ok, i, r, vbus), Ok(()));
        let why = contact_state(0.7705, 0.123, i, r, vbus).unwrap_err();
        assert_eq!(
            why,
            "the hold read R0 0.7705 against the identified R 1.0000: a low-resistance contact \
             state (a brush bridging two segments); re-seat and retry"
        );
        // the duty alone, 18% under the 15.4% the identified R predicts
        assert!(contact_state(0.95, 0.126, i, r, vbus).is_err());
        assert!(contact_state(1.2, ok, i, r, vbus).is_err());
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
