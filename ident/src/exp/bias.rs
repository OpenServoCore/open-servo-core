//! Bias: torque off, the raw sensors at rest. The ident aggregate block is
//! stale at torque-off (last-valid feed), so the polls read the raw
//! snapshot fields: current bias sanity and noise, vbus sanity. The pot's
//! rest noise, sigma_theta for the fusion synthesis, comes from a TEL
//! stream instead: a polled read lands on the bus's own disturbance and
//! reads about twice as noisy. It is taken in raw counts and scaled by the
//! position table's travel-average gain, never by the gain of the one
//! interval the shaft rests in.
//!
//! The ADC skips codes at its quarter-scale carries: a rest within
//! [`CARRY_BAND`] of one reads a noise that is not the sensor's. Such a
//! rest is jogged off first, at a stall-safe duty ([`BiasCfg::jog_q15`]),
//! then left torque off; a shaft that does not move off it is reported in
//! plain words.

use super::{Cmd, Experiment, RigParams};
use crate::fitmath::{mean, stddev};
use crate::frame::{TEL_BIT_POS, TelBurst, TelemetrySnapshot};
use crate::pot::Pot;
use crate::regs::control;

/// A rest this close to a carry, counts, is moved off it.
pub const CARRY_BAND: u16 = 8;

/// The ADC's quarter-scale carries, raw counts.
const CARRIES: [u16; 3] = [1024, 2048, 3072];

const JOG_POLL_MS: u32 = 10;
const JOG_POLLS: u32 = 30;
/// The jogged shaft settles this long, ms, torque off, before the polls.
const JOG_REST_MS: u32 = 300;

pub struct BiasCfg {
    /// Poll count; with `poll_ms` pacing this sets the observation span.
    pub polls: u32,
    pub poll_ms: u32,
    /// TEL samples the rest noise is read over: half a second of ticks.
    pub tel_samples: u16,
    /// The duty a rest on a carry is jogged off at, q15: one whose stall
    /// the current limit holds. 0 never jogs.
    pub jog_q15: i16,
}

impl Default for BiasCfg {
    /// ~2 s of polls at the default pacing.
    fn default() -> Self {
        Self {
            polls: 400,
            poll_ms: 5,
            tel_samples: 10_000,
            jog_q15: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BiasResult {
    /// Pot noise sigma in the counts the kernel controls on - the fusion
    /// synthesis input: `sigma_raw` x `gain`.
    pub sigma_theta: f64,
    /// Pot noise sigma over the TEL stream, raw counts.
    pub sigma_raw: f64,
    /// The position table's travel-average gain; 1 without a table.
    pub gain: f64,
    /// Where the shaft rested for the stream, raw counts.
    pub rest: f64,
    pub tel_n: usize,
    /// Mean polled position, the kernel's counts.
    pub pos_mean: f64,
    /// Raw current-channel noise sigma, counts.
    pub i_noise: f64,
    /// Mean raw current minus the published bias. The bias is the zero
    /// with the driver awake, so torque off this reads minus the driver's
    /// own supply-current step once the servo has learned it, near zero
    /// before.
    pub i_bias_delta: f64,
    pub vbus_mean: f64,
    pub vbus_sd: f64,
    pub n: usize,
    pub warnings: Vec<String>,
}

/// The carry `raw` rests within [`CARRY_BAND`] of.
fn carry_at(raw: u16) -> Option<u16> {
    CARRIES.into_iter().find(|c| raw.abs_diff(*c) <= CARRY_BAND)
}

enum Phase {
    TorqueOff,
    DutyOff,
    RestRead,
    RestEval,
    JogMode,
    JogTorque,
    JogSet,
    JogWait,
    JogRead,
    JogEval,
    JogTorqueOff,
    JogRest,
    Poll,
    Collect,
    MaskOn,
    Stream,
    MaskOff,
    Finished,
}

pub struct Bias {
    cfg: BiasCfg,
    pot: Pot,
    stops: Option<(u16, u16)>,
    phase: Phase,
    /// The carry the rest sat on and the way the jog drives off it.
    jog: Option<(u16, i8)>,
    jog_polls: u32,
    jogged: bool,
    pos: Vec<f64>,
    current: Vec<f64>,
    bias: Vec<f64>,
    vbus: Vec<f64>,
    tel: Vec<f64>,
    warnings: Vec<String>,
}

impl Bias {
    pub fn new(cfg: BiasCfg, params: &RigParams) -> Self {
        Self {
            cfg,
            pot: params.pot,
            stops: params.stops,
            phase: Phase::TorqueOff,
            jog: None,
            jog_polls: 0,
            jogged: false,
            pos: Vec::new(),
            current: Vec::new(),
            bias: Vec::new(),
            vbus: Vec::new(),
            tel: Vec::new(),
            warnings: Vec::new(),
        }
    }

    /// None until enough polls and stream samples landed for the
    /// statistics to mean anything.
    pub fn result(&self) -> Option<BiasResult> {
        if self.pos.len() < 16 || self.tel.len() < 16 {
            return None;
        }
        let sigma_raw = stddev(&self.tel)?;
        let gain = self.pot.travel_gain(self.stops);
        Some(BiasResult {
            sigma_theta: sigma_raw * gain,
            sigma_raw,
            gain,
            rest: mean(&self.tel)?,
            tel_n: self.tel.len(),
            pos_mean: mean(&self.pos)?,
            i_noise: stddev(&self.current)?,
            i_bias_delta: mean(&self.current)? - mean(&self.bias)?,
            vbus_mean: mean(&self.vbus)?,
            vbus_sd: stddev(&self.vbus)?,
            n: self.pos.len(),
            warnings: self.warnings.clone(),
        })
    }

    /// The rest just read: on a carry it is jogged off once, and one it
    /// could not leave is reported.
    fn rest_eval(&mut self, raw: u16) -> Cmd {
        match carry_at(raw) {
            Some(carry) if !self.jogged && self.cfg.jog_q15 != 0 => {
                let dir = if raw >= carry { 1 } else { -1 };
                self.jog = Some((carry, dir));
                self.phase = Phase::JogMode;
            }
            Some(carry) => {
                let why = if self.jogged {
                    format!(
                        "and the jog at {:.1}% did not move it off",
                        self.cfg.jog_q15 as f64 * 100.0 / 32767.0
                    )
                } else {
                    "and nothing jogged it off".into()
                };
                self.warnings.push(format!(
                    "the shaft rests at {raw}, within {CARRY_BAND} counts of the position \
                     sensor's carry at {carry}, where its converter skips codes, {why}: the rest \
                     noise may read wide"
                ));
                self.phase = Phase::Poll;
            }
            None => self.phase = Phase::Poll,
        }
        Cmd::Pause { ms: 0 }
    }

    fn jog_eval(&mut self, raw: u16) -> Cmd {
        self.jog_polls += 1;
        let off = self
            .jog
            .is_none_or(|(carry, _)| raw.abs_diff(carry) >= 2 * CARRY_BAND);
        if off || self.jog_polls >= JOG_POLLS {
            self.phase = Phase::JogTorqueOff;
            return Cmd::Write {
                reg: control::GOAL_DUTY,
                value: 0,
            };
        }
        self.phase = Phase::JogWait;
        Cmd::Pause { ms: 0 }
    }
}

impl Experiment for Bias {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::TorqueOff => {
                self.phase = Phase::DutyOff;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::DutyOff => {
                self.phase = Phase::RestRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::RestRead => {
                self.phase = Phase::RestEval;
                Cmd::Read
            }
            Phase::RestEval => match obs {
                Some(o) => self.rest_eval(o.pos),
                None => {
                    self.phase = Phase::RestRead;
                    Cmd::Pause { ms: 0 }
                }
            },
            Phase::JogMode => {
                self.phase = Phase::JogTorque;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            Phase::JogTorque => {
                self.phase = Phase::JogSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::JogSet => {
                let dir = self.jog.map_or(1, |(_, d)| d) as i32;
                self.jogged = true;
                self.phase = Phase::JogWait;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: dir * self.cfg.jog_q15 as i32,
                }
            }
            Phase::JogWait => {
                self.phase = Phase::JogRead;
                Cmd::Pause { ms: JOG_POLL_MS }
            }
            Phase::JogRead => {
                self.phase = Phase::JogEval;
                Cmd::Read
            }
            Phase::JogEval => match obs {
                Some(o) => self.jog_eval(o.pos),
                None => {
                    self.phase = Phase::JogRead;
                    Cmd::Pause { ms: 0 }
                }
            },
            Phase::JogTorqueOff => {
                self.phase = Phase::JogRest;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::JogRest => {
                self.phase = Phase::RestRead;
                Cmd::Pause { ms: JOG_REST_MS }
            }
            Phase::Poll => {
                self.phase = Phase::Collect;
                Cmd::Read
            }
            Phase::Collect => {
                if let Some(o) = obs {
                    self.pos.push(self.pot.counts(o.pos));
                    self.current.push(o.current as f64);
                    self.bias.push(o.current_bias_counts as f64);
                    self.vbus.push(o.vbus_counts as f64);
                }
                if self.pos.len() as u32 >= self.cfg.polls {
                    self.phase = Phase::MaskOn;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::Poll;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            Phase::MaskOn => {
                self.phase = Phase::Stream;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: TEL_BIT_POS as i32,
                }
            }
            Phase::Stream => {
                self.phase = Phase::MaskOff;
                Cmd::Stream {
                    samples: self.cfg.tel_samples,
                    goal: None,
                }
            }
            Phase::MaskOff => {
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }

    fn push_tel(&mut self, burst: &TelBurst) {
        self.tel
            .extend(burst.frames.iter().filter_map(|f| f.pos.map(f64::from)));
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, bent_pot, pump};
    use super::*;

    #[test]
    fn bias_sigma_theta_recovered() {
        let mut servo = FakeServo::new(3.37);
        // uniform width 5 counts -> sigma = 5/sqrt(12) ~ 1.44
        servo.pos_noise = 5.0;
        let mut exp = Bias::new(BiasCfg::default(), &crate::exp::testkit::rig());
        let log = pump(&mut exp, &mut servo, 10_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let r = exp.result().expect("enough polls");
        assert_eq!(r.n, 400);
        assert!(
            (1.0..2.0).contains(&r.sigma_theta),
            "sigma_theta {}",
            r.sigma_theta
        );
        assert!(r.i_bias_delta.abs() < 1.0, "delta {}", r.i_bias_delta);
        assert!((r.vbus_mean - 1731.0).abs() < 1.0);
    }

    /// The bent pot reads 0.8x at the middle of the travel and 1.2x at the
    /// ends, 1x across it. At rest mid travel with a 1.2-count sensor the
    /// noise comes from the stream, in raw counts, and scales by the
    /// travel's gain, where the gain of the interval the shaft rests in
    /// would read it a fifth low. The stream is the one the bias arms,
    /// torque off and its mask cleared after.
    #[test]
    fn rest_noise_is_measured_in_raw_counts() {
        let table = bent_pot();
        let mut servo = FakeServo::new(3.37);
        servo.pot = Some(table);
        servo.pos = 2100.0;
        servo.pos_noise = 1.2 * 12f64.sqrt();
        let params = RigParams {
            pot: Pot::live(table),
            ..crate::exp::testkit::rig()
        };
        let mut exp = Bias::new(BiasCfg::default(), &params);
        let log = pump(&mut exp, &mut servo, 10_000);
        let r = exp.result().expect("a result");
        assert_eq!(r.tel_n, 10_000);
        // the converter's rounding adds its own 1/12
        let want = (1.2f64.powi(2) + 1.0 / 12.0).sqrt();
        assert!((r.sigma_raw / want - 1.0).abs() < 0.05, "{}", r.sigma_raw);
        assert!((r.gain - 1.0).abs() < 1e-9, "{}", r.gain);
        assert_eq!(r.sigma_theta, r.sigma_raw * r.gain);
        let local = (table.counts(2116) - table.counts(2084)) / 32.0;
        assert!(local < 0.85, "{local}");
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);
        let tail: Vec<&str> = log.iter().rev().take(3).map(String::as_str).collect();
        assert_eq!(
            tail,
            ["write tel_mask 0", "stream 10000", "write tel_mask 1"]
        );
        assert!(!servo.torque);
    }

    /// A rest on the mid-scale carry is jogged off it at the stall-safe
    /// duty and left torque off before anything is read; a shaft that
    /// cannot leave it is reported in plain words, and one never jogged is
    /// too.
    #[test]
    fn a_rest_on_the_mid_scale_carry_is_moved_off_it() {
        let run = |jam: bool, jog_q15: i16| {
            let mut servo = crate::exp::testkit::bench_mg90(3204);
            servo.pos = 2050.0;
            if jam {
                servo.jam = Some(2050.0);
            }
            let cfg = BiasCfg {
                jog_q15,
                ..BiasCfg::default()
            };
            let params = crate::exp::testkit::rig().without_pos_guard();
            let mut exp = super::super::Guarded::new(Bias::new(cfg, &params), params);
            let log = pump(&mut exp, &mut servo, 10_000);
            assert_eq!(exp.abort(), None);
            assert!(!servo.torque);
            (exp.into_inner().result().expect("a result"), log)
        };
        let (r, log) = run(false, 5082);
        assert!(log.contains(&"write goal_duty 5082".to_string()), "{log:?}");
        assert!(
            (r.rest.round() as u16).abs_diff(2048) >= 2 * CARRY_BAND,
            "rests at {}",
            r.rest
        );
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);

        let (r, _) = run(true, 5082);
        assert_eq!(
            r.warnings,
            [
                "the shaft rests at 2050, within 8 counts of the position sensor's carry at 2048, \
              where its converter skips codes, and the jog at 15.5% did not move it off: the \
              rest noise may read wide"
            ]
        );

        let (r, log) = run(false, 0);
        assert!(!log.iter().any(|l| l.starts_with("write goal_duty 5")));
        assert!(r.warnings[0].ends_with("and nothing jogged it off: the rest noise may read wide"));
    }
}
