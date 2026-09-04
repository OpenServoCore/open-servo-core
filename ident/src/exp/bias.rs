//! E0 bias: torque off, poll raw sensors. The ident aggregate block is
//! stale at torque-off (last-valid feed), so this experiment reads the raw
//! snapshot fields instead: current bias sanity, pot noise sigma (feeds
//! fusion gain synthesis), vbus sanity.

use super::{Cmd, Experiment};
use crate::fitmath::{mean, stddev};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct BiasCfg {
    /// Poll count; with `poll_ms` pacing this sets the observation span.
    pub polls: u32,
    pub poll_ms: u32,
}

impl Default for BiasCfg {
    /// ~2 s at the default pacing.
    fn default() -> Self {
        Self {
            polls: 400,
            poll_ms: 5,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BiasResult {
    /// Pot noise sigma, counts - the fusion synthesis input.
    pub sigma_theta: f64,
    pub pos_mean: f64,
    /// Raw current-channel noise sigma, counts.
    pub i_noise: f64,
    /// Mean raw current minus the boot-measured bias; near zero when the
    /// boot bias capture was healthy.
    pub i_bias_delta: f64,
    pub vbus_mean: f64,
    pub vbus_sd: f64,
    pub n: usize,
}

enum Phase {
    TorqueOff,
    DutyOff,
    Poll,
    Collect,
    Finished,
}

pub struct Bias {
    cfg: BiasCfg,
    phase: Phase,
    pos: Vec<f64>,
    current: Vec<f64>,
    bias: Vec<f64>,
    vbus: Vec<f64>,
}

impl Bias {
    pub fn new(cfg: BiasCfg) -> Self {
        Self {
            cfg,
            phase: Phase::TorqueOff,
            pos: Vec::new(),
            current: Vec::new(),
            bias: Vec::new(),
            vbus: Vec::new(),
        }
    }

    /// None until enough polls landed for the statistics to mean anything.
    pub fn result(&self) -> Option<BiasResult> {
        if self.pos.len() < 16 {
            return None;
        }
        Some(BiasResult {
            sigma_theta: stddev(&self.pos)?,
            pos_mean: mean(&self.pos)?,
            i_noise: stddev(&self.current)?,
            i_bias_delta: mean(&self.current)? - mean(&self.bias)?,
            vbus_mean: mean(&self.vbus)?,
            vbus_sd: stddev(&self.vbus)?,
            n: self.pos.len(),
        })
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
                self.phase = Phase::Poll;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::Poll => {
                self.phase = Phase::Collect;
                Cmd::Read
            }
            Phase::Collect => {
                if let Some(o) = obs {
                    self.pos.push(o.pos as f64);
                    self.current.push(o.current as f64);
                    self.bias.push(o.current_bias_counts as f64);
                    self.vbus.push(o.vbus_counts as f64);
                }
                if self.pos.len() as u32 >= self.cfg.polls {
                    self.phase = Phase::Finished;
                    Cmd::Done
                } else {
                    self.phase = Phase::Poll;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, pump};
    use super::*;

    #[test]
    fn bias_sigma_theta_recovered() {
        let mut servo = FakeServo::new(3.37);
        // uniform width 5 counts -> sigma = 5/sqrt(12) ~ 1.44
        servo.pos_noise = 5.0;
        let mut exp = Bias::new(BiasCfg::default());
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
}
