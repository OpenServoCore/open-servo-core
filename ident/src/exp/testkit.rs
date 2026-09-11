//! Test-only fake servo + driver pump: a scripted plant that answers the
//! engine's commands the way the rig would, so experiments run end to end
//! in-process. Electrical model: stalled i = v/R; free-running i = fc; the
//! first windows after a duty change are inflated to imitate the L
//! transient the settle discard exists for.

use super::{Cmd, Experiment};
use crate::burst::{
    CHAN_VBUS, CHAN_VMOTOR_A, CHAN_VMOTOR_B, Capture, Meta, SAMPLE_HCLK, SAMPLE_US, SAMPLES,
    frame_len,
};
use crate::frame::{TelFrame, TelemetrySnapshot};
use crate::regs::{ALL, Reg, control};

pub struct FakeServo {
    pub r: f64,
    pub vbus: f64,
    pub ke: f64,
    pub fc: f64,
    /// Viscous friction, ccounts per c/s (physical model only).
    pub fv: f64,
    pub free_speed: f64,
    /// Steady omega from the motor equation instead of the free_speed
    /// shortcut: omega = (|v| - R*fc) / (Ke + R*fv), signed by duty.
    pub physical_motion: bool,
    /// First-order dynamics for the inertia transient: omega integrates
    /// alpha = b * f_med * (i - fc*sgn - fv*omega) with i = (v - Ke*w)/R,
    /// so the planted `b` is exactly what the estimators must recover.
    /// Steady state matches `physical_motion` by construction.
    pub dynamic: bool,
    /// B, (c/s per medium tick) per ccount (dynamic model).
    pub b: f64,
    /// Medium rate, tick_hz / 10.
    pub f_med: f64,
    /// Static friction: no motion below this |duty| (0 = none).
    pub breakaway_q15: i16,
    /// Reported pos gains +80 counts inside this zone (slip artifact).
    pub glitch_zone: Option<(f64, f64)>,
    pub pos: f64,
    pub ends: (f64, f64),
    /// Wiring convention: false inverts duty's effect on motion, so the
    /// endstop experiment must infer the flipped drive_polarity.
    pub drive_polarity: bool,
    pub pos_noise: f64,
    pub fault_at_ms: Option<f64>,
    /// Latch a fault once this many [`Cmd::Burst`]s have been captured.
    pub fault_after_bursts: Option<u32>,
    pub bursts: u32,
    /// The kernel's soft endstop: outbound duty at or past a limit is zeroed
    /// unless the stall permit is set (and honored).
    pub soft: Option<(f64, f64)>,
    pub permit: bool,
    pub honors_permit: bool,
    pub torque: bool,
    pub duty: i16,
    pub tel_mask: u16,
    omega_dyn: f64,
    pub t_ms: f64,
    t_duty_change: f64,
    pub transient_windows: f64,
    pub transient_gain: f64,
    /// The plant a [`Cmd::Burst`] captures from; the burst's own mask
    /// replaces `chans`.
    pub burst: SynthBurst,
    lcg: u64,
}

impl FakeServo {
    pub fn new(r: f64) -> Self {
        Self {
            r,
            vbus: 1731.0,
            ke: 0.1731,
            fc: 20.0,
            fv: 0.0,
            free_speed: 10_000.0,
            physical_motion: false,
            dynamic: false,
            b: 0.1,
            f_med: 2010.0,
            breakaway_q15: 0,
            glitch_zone: None,
            pos: 2400.0,
            ends: (200.0, 4000.0),
            drive_polarity: true,
            pos_noise: 0.0,
            fault_at_ms: None,
            fault_after_bursts: None,
            bursts: 0,
            soft: None,
            permit: false,
            honors_permit: true,
            torque: false,
            duty: 0,
            tel_mask: 0,
            omega_dyn: 0.0,
            t_ms: 0.0,
            t_duty_change: -1e9,
            transient_windows: 3.0,
            transient_gain: 1.5,
            // r 8 ohm keeps the whole default duty ladder inside the 0.4 A
            // envelope, so the plan is not pruned by accident
            burst: SynthBurst {
                r: 8.0,
                ..SynthBurst::board_d()
            },
            lcg: 0x9E3779B97F4A7C15,
        }
    }

    fn noise(&mut self) -> f64 {
        self.lcg = self
            .lcg
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        // uniform in [-0.5, 0.5) scaled by pos_noise
        ((self.lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * self.pos_noise
    }

    /// The duty the bridge actually sees: zero with torque off, and zero
    /// driving outward at a soft limit the permit does not open.
    fn applied(&self) -> i16 {
        let out = self.duty as f64 * if self.drive_polarity { 1.0 } else { -1.0 };
        let clamped = self
            .soft
            .is_some_and(|(lo, hi)| (self.pos <= lo && out < 0.0) || (self.pos >= hi && out > 0.0))
            && !(self.permit && self.honors_permit);
        if !self.torque || clamped {
            0
        } else {
            self.duty
        }
    }

    fn omega(&self) -> f64 {
        if self.dynamic {
            return self.omega_dyn;
        }
        let duty = self.applied();
        if duty == 0 || duty.unsigned_abs() < self.breakaway_q15 as u16 {
            return 0.0;
        }
        let vsign = duty.signum() as f64 * if self.drive_polarity { 1.0 } else { -1.0 };
        let stalled =
            (self.pos <= self.ends.0 && vsign < 0.0) || (self.pos >= self.ends.1 && vsign > 0.0);
        if stalled {
            return 0.0;
        }
        if self.physical_motion {
            let v = duty.unsigned_abs() as f64 / 32767.0 * self.vbus;
            let mag = ((v - self.r * self.fc) / (self.ke + self.r * self.fv)).max(0.0);
            mag * vsign
        } else {
            duty.unsigned_abs() as f64 / 32767.0 * self.free_speed * vsign
        }
    }

    /// The winding current the dynamic model carries right now: ohmic on
    /// the applied volts minus bemf. Friction is mechanical - it consumes
    /// torque, not extra current - so nothing else is added.
    fn i_dyn(&self) -> f64 {
        let duty = self.applied();
        if duty == 0 {
            return 0.0;
        }
        let v = duty as f64 / 32767.0 * self.vbus;
        (v - self.ke * self.omega_dyn) / self.r
    }

    pub fn write(&mut self, reg: Reg, value: i32) {
        if reg == control::TORQUE_ENABLE {
            self.torque = value != 0;
        } else if reg == control::GOAL_DUTY {
            self.duty = value as i16;
            self.t_duty_change = self.t_ms;
        } else if reg == control::TEL_MASK {
            self.tel_mask = value as u16;
        } else if reg == control::STALL_PERMIT {
            self.permit = value != 0;
        }
    }

    /// One dynamic-model integration substep.
    fn substep(&mut self, dt: f64) {
        let i = self.i_dyn();
        let w = self.omega_dyn;
        let fric = if w != 0.0 {
            self.fc * w.signum() + self.fv * w
        } else if i.abs() > self.fc {
            self.fc * i.signum()
        } else {
            i // no net torque below stiction: alpha = 0
        };
        let alpha = self.b * self.f_med * (i - fric);
        let w2 = w + alpha * dt;
        // coasting friction never reverses the spin through zero
        self.omega_dyn = if self.applied() == 0 {
            if w != 0.0 && w.signum() != w2.signum() {
                0.0
            } else {
                w2
            }
        } else {
            w2
        };
        self.pos = (self.pos + self.omega_dyn * dt).clamp(self.ends.0, self.ends.1);
        if (self.pos <= self.ends.0 && self.omega_dyn < 0.0)
            || (self.pos >= self.ends.1 && self.omega_dyn > 0.0)
        {
            self.omega_dyn = 0.0;
        }
    }

    /// One fast-tick advance shared by [`advance`] and [`stream`].
    fn tick(&mut self, dt: f64) {
        if self.dynamic {
            self.substep(dt);
        } else {
            self.pos = (self.pos + self.omega() * dt).clamp(self.ends.0, self.ends.1);
        }
    }

    pub fn advance(&mut self, ms: u32) {
        if self.dynamic {
            // tick-sized substeps keep the ~tens-of-ms tau integration exact
            let dt = 1.0 / (self.f_med * 10.0);
            let n = (ms as f64 / 1000.0 / dt).round() as u64;
            for _ in 0..n {
                self.substep(dt);
            }
        } else {
            let dt = ms as f64 / 1000.0;
            self.pos = (self.pos + self.omega() * dt).clamp(self.ends.0, self.ends.1);
        }
        self.t_ms += ms as f64;
    }

    /// One armed TEL burst: `samples` fast ticks integrated from t0 (the
    /// arm instant - any goal write is already applied), one frame per tick
    /// with the mask-selected fields. Mask 0 streams nothing, like the
    /// firmware's disarmed producer.
    pub fn stream(&mut self, samples: u16, sink: &mut Vec<TelFrame>) {
        let dt = 1.0 / (self.f_med * 10.0);
        for k in 0..samples {
            self.tick(dt);
            if self.tel_mask == 0 {
                continue;
            }
            let noise = self.noise();
            let driving = self.torque && self.duty != 0;
            let sel = |bit: u16| self.tel_mask & bit != 0;
            let i = if self.dynamic {
                self.i_dyn()
            } else {
                let v = self.duty as f64 / 32767.0 * self.vbus;
                if driving {
                    (v - self.ke * self.omega()) / self.r
                } else {
                    0.0
                }
            };
            sink.push(TelFrame {
                tick: k as u64,
                window_valid: driving,
                pos: sel(1 << 0).then(|| (self.pos + noise).round().clamp(0.0, 4095.0) as u16),
                current: sel(1 << 1).then(|| i.round() as i16),
                current_trough: sel(1 << 2).then_some(512),
                duty_q15: sel(1 << 3).then_some(if driving { self.duty } else { 0 }),
                vdiff: sel(1 << 4).then(|| {
                    if driving {
                        (self.vbus * self.duty.signum() as f64) as i16
                    } else {
                        0
                    }
                }),
                vbus: sel(1 << 5).then_some(self.vbus as u16),
                // raw terminal fakes: driven side carries the rail, the
                // other sits low; raw current rides a 512-count bias and
                // is a MAGNITUDE - the low-side shunt sees drive current
                // the same way whichever way the bridge is pointed
                // (window.rs applies the direction sign downstream)
                current_raw: sel(1 << 6).then(|| (512.0 + i.abs()).round() as u16),
                vmotor_a: sel(1 << 7).then_some(if driving && self.duty > 0 {
                    self.vbus as u16
                } else {
                    0
                }),
                vmotor_b: sel(1 << 8).then_some(if driving && self.duty < 0 {
                    self.vbus as u16
                } else {
                    0
                }),
                vbus_raw: sel(1 << 9).then_some(self.vbus as u16),
                ntc_raw: sel(1 << 10).then_some(2048),
            });
        }
        self.t_ms += samples as f64 * dt * 1000.0;
    }

    pub fn read(&mut self) -> TelemetrySnapshot {
        let duty = self.applied();
        let driving = duty != 0;
        let (i, vdiff) = if driving {
            let v = duty as f64 / 32767.0 * self.vbus;
            let omega = self.omega();
            // friction current only while moving: stalled current is ohmic.
            // The physical and dynamic models need no extra term - their
            // (v - ke*omega)/r IS the winding current at every instant.
            let fric = if omega != 0.0 && !self.physical_motion && !self.dynamic {
                self.fc * duty.signum() as f64
            } else {
                0.0
            };
            let mut i = (v - self.ke * omega) / self.r + fric;
            if (self.t_ms - self.t_duty_change) / 0.8 < self.transient_windows {
                i *= self.transient_gain;
            }
            (i, self.vbus * duty.signum() as f64)
        } else {
            (0.0, 0.0)
        };
        let fault = matches!(self.fault_at_ms, Some(at) if self.t_ms >= at)
            || matches!(self.fault_after_bursts, Some(n) if self.bursts >= n);
        let glitch = match self.glitch_zone {
            Some((lo, hi)) if (lo..=hi).contains(&self.pos) => 80.0,
            _ => 0.0,
        };
        let pos = (self.pos + glitch + self.noise())
            .round()
            .clamp(0.0, 4095.0) as u16;
        TelemetrySnapshot {
            fault_flags: if fault { 32 } else { 0 },
            fault_code: if fault { 6 } else { 0 },
            pos,
            current: (512.0 + i).round() as u16,
            current_bias_counts: 512,
            vbus_counts: self.vbus as u16,
            i_mean_counts: i.round() as i16,
            vdiff_mean: vdiff.round() as i16,
            duty_mean_q15: duty,
            duty_applied_q15: duty,
            agg_seq: (self.t_ms / 0.8) as u64 as u16,
            ..Default::default()
        }
    }
}

/// A switched RL winding sampled the way the firmware burst samples it: a
/// centre-aligned PWM period of `2 * arr` HCLK, the drive window straddling
/// the crest, the shunt live only during ON, and a first-order amplifier
/// lag on the edge. Separate from [`FakeServo`], which has no electrical
/// model at microsecond resolution.
///
/// The bridge is explicit when `rds` and `r_shunt` are set: during ON the
/// driven terminal sits one high-side drop under the rail node and the idle
/// terminal one low-side drop plus the shunt above ground; during the brake
/// both low sides short the winding and nothing crosses the shunt. With
/// both zero, `r` is the whole loop, the model the older tests use.
///
/// `c_island` is decoupling inside the shunt's ground island, fed from the
/// rail node through `r_feed`: it supplies part of each pulse and is
/// repaid through the shunt during OFF, so the shunt carries the island's
/// feed current rather than the winding's.
#[derive(Clone, Debug)]
pub struct SynthBurst {
    /// Winding resistance, ohms; the whole loop when `rds` and `r_shunt`
    /// are zero.
    pub r: f64,
    pub l: f64,
    /// Open-circuit source voltage.
    pub v_rail: f64,
    /// Source resistance behind `c_bulk`; zero is a stiff rail.
    pub r_src: f64,
    pub c_bulk: f64,
    pub c_island: f64,
    pub r_feed: f64,
    /// Per bridge FET, ohms.
    pub rds: f64,
    pub r_shunt: f64,
    /// Brush drop against the winding current, volts.
    pub v0: f64,
    /// Back-EMF growth after the step latches, volts per ms.
    pub emf_v_per_ms: f64,
    /// The chopping leg's low side never turns on: its body diode carries
    /// the whole OFF phase.
    pub body_diode: bool,
    pub arr: u16,
    pub bias: f64,
    /// Shunt amplifier edge time constant, microseconds.
    pub settle_us: f64,
    /// Terminal tap RC, microseconds.
    pub tap_us: f64,
    /// Terminal divider bias node, raw counts.
    pub vb: f64,
    /// Tap B's code over tap A's with the winding at rest, counts: the two
    /// dividers never match.
    pub split: f64,
    /// Peak-to-peak measurement noise, counts.
    pub noise_counts: f64,
    pub amps_per_count: f64,
    pub v_rail_per_count: f64,
    pub v_term_per_count: f64,
    pub adc_lsb_v: f64,
    pub step_index: u16,
    /// Samples from the step to the crest whose update event latches the
    /// new compare value. The window straddling that crest is a HALF
    /// window - the bench captures show it and so must this.
    pub latch_delay: usize,
    pub chans: u8,
}

/// Body diode forward drop, volts.
const DIODE_V: f64 = 0.7;

/// Periods a from-a-hold capture runs at its pre-step duty before sample 0,
/// so the pre-step half and the pre-arm rail read are settled.
const PRIME_PERIODS: usize = 40;

/// Where the servo's scan samples the rail: slot 5 of the crest scan,
/// 2.16 us after the crest, in conversion periods.
const PRE_ARM_RAIL_SAMPLES: f64 = 2.0;

impl SynthBurst {
    /// Board D as fitted: 60 mohm shunt at G 15, 15k/10k rail tap, 6k8/3k3
    /// terminal taps to a 779-count bias node, a 4.37 V USB rail, ARR 1200.
    pub fn board_d() -> Self {
        let lsb = 3.3 / 4096.0;
        Self {
            r: 4.0,
            l: 0.6e-3,
            v_rail: 4.37,
            r_src: 0.0,
            c_bulk: 100e-6,
            c_island: 0.0,
            r_feed: 0.0,
            rds: 0.0,
            r_shunt: 0.0,
            v0: 0.0,
            emf_v_per_ms: 0.0,
            body_diode: false,
            arr: 1200,
            bias: 112.0,
            settle_us: 0.9,
            tap_us: 0.33,
            vb: 779.0,
            split: 0.0,
            noise_counts: 2.0,
            amps_per_count: lsb / (15.0 * 0.060),
            v_rail_per_count: lsb * 2.5,
            v_term_per_count: lsb * 10_100.0 / 3_300.0,
            adc_lsb_v: lsb,
            step_index: 485,
            latch_delay: 21,
            chans: 0,
        }
    }

    /// Board D's bridge made explicit: DRV8212P FETs and the 60 mohm shunt.
    pub fn with_bridge(self) -> Self {
        Self {
            rds: 0.140,
            r_shunt: 0.060,
            ..self
        }
    }

    /// Samples per PWM period at the burst's own sample clock.
    pub fn period(&self) -> f64 {
        2.0 * self.arr as f64 / SAMPLE_HCLK
    }

    pub fn capture(&self, step_q15: i16, pre_q15: i16) -> Capture {
        const SUBSTEPS: usize = 8;
        let p = self.period();
        let crest0 = self.step_index as f64 + self.latch_delay as f64;
        let duty_at = |k: f64| {
            let q = if k >= 0.0 { step_q15 } else { pre_q15 };
            (q as f64 / 32767.0).abs()
        };
        // (on, duty of the half period x falls in, offset from its crest)
        let phase = |x: f64| {
            let k = ((x - crest0) / p).round();
            let u = x - (crest0 + k * p);
            let d = if u >= 0.0 {
                duty_at(k)
            } else {
                duty_at(k - 1.0)
            };
            (u.abs() <= d * p / 2.0, d, u)
        };
        let dt = SAMPLE_US * 1e-6 / SUBSTEPS as f64;
        let a_amp = 1.0 - (-SAMPLE_US / SUBSTEPS as f64 / self.settle_us).exp();
        let a_tap = 1.0 - (-SAMPLE_US / SUBSTEPS as f64 / self.tap_us).exp();
        let vb_v = self.vb * self.adc_lsb_v;
        let fwd = step_q15 >= 0;
        let slots: Vec<u8> = [CHAN_VMOTOR_A, CHAN_VMOTOR_B, CHAN_VBUS]
            .into_iter()
            .filter(|b| self.chans & b != 0)
            .collect();
        let fl = frame_len(self.chans);

        let mut pre_arm_rail = self.v_rail;
        // winding current, rail node, island, amplifier, tap A, tap B
        let step = |x: f64, st: &mut [f64; 6]| {
            let [i, vc, isl, m, ta, tb] = st;
            let (on, d, _) = phase(x);
            let drive = on && d != 0.0;
            let e = self.emf_v_per_ms * ((x - crest0).max(0.0) * SAMPLE_US * 1e-3);
            let v0 = if *i > 0.0 { self.v0 } else { 0.0 };
            let i_bridge = if drive { *i } else { 0.0 };
            // (bridge supply above PGND, shunt current)
            let (vm, i_sh) = if self.c_island > 0.0 {
                (*isl, (*vc - *isl) / (self.r_feed + self.r_shunt))
            } else {
                (*vc - i_bridge * self.r_shunt, i_bridge)
            };
            let pgnd = i_sh * self.r_shunt;
            // terminals against ground, in the drive frame: hi is the
            // chopping leg
            let (hi, lo) = if d == 0.0 {
                (vb_v, vb_v)
            } else if on {
                (vm + pgnd - *i * self.rds, pgnd + *i * self.rds)
            } else if self.body_diode && *i > 0.0 {
                (-DIODE_V, pgnd + *i * self.rds)
            } else {
                (pgnd - *i * self.rds, pgnd + *i * self.rds)
            };
            if d != 0.0 {
                *i += (hi - lo - *i * self.r - v0 - e) / self.l * dt;
                if !on {
                    *i = i.max(0.0);
                }
            }
            if self.c_island > 0.0 {
                *isl += (i_sh - i_bridge) / self.c_island * dt;
            }
            if self.r_src > 0.0 {
                *vc += ((self.v_rail - *vc) / self.r_src - i_sh) / self.c_bulk * dt;
            }
            *m += (self.bias + i_sh / self.amps_per_count - *m) * a_amp;
            let (va, vbv) = if fwd { (hi, lo) } else { (lo, hi) };
            *ta += (va - *ta) * a_tap;
            *tb += (vbv - *tb) * a_tap;
        };
        let mut st = [0.0, self.v_rail, self.v_rail, self.bias, vb_v, vb_v];
        if pre_q15 != 0 {
            let x0 = -(PRIME_PERIODS as f64) * p;
            let n = (PRIME_PERIODS as f64 * p * SUBSTEPS as f64) as usize;
            for s in 0..n {
                let x = x0 + s as f64 / SUBSTEPS as f64;
                step(x, &mut st);
                let (_, _, u) = phase(x);
                if (PRE_ARM_RAIL_SAMPLES..PRE_ARM_RAIL_SAMPLES + 1.0 / SUBSTEPS as f64).contains(&u)
                {
                    pre_arm_rail = st[1];
                }
            }
        }
        let mut lcg = 0x2545F4914F6CDD1Du64;
        let mut noise = || {
            lcg = lcg
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * self.noise_counts
        };
        let tap_code = |v: f64| self.vb + (v - vb_v) / self.v_term_per_count;
        let mut samples = Vec::with_capacity(SAMPLES);
        for n in 0..SAMPLES {
            for s in 0..SUBSTEPS {
                let x = n as f64 + s as f64 / SUBSTEPS as f64;
                step(x, &mut st);
            }
            let code = match n % fl {
                0 => st[3],
                slot => match slots[slot - 1] {
                    CHAN_VMOTOR_A => tap_code(st[4]),
                    CHAN_VMOTOR_B => tap_code(st[5]) + self.split,
                    _ => st[1] / self.v_rail_per_count,
                },
            };
            samples.push((code + noise()).round().clamp(0.0, 4095.0) as u16);
        }
        Capture {
            samples,
            meta: Meta {
                pre_q15,
                step_q15,
                step_index: self.step_index,
                start_cnt: 1094,
                pwm_arr: self.arr,
                start_dir: 1,
                restore_dir: 0,
                vbus_raw: (pre_arm_rail / self.v_rail_per_count).round() as u16,
                bias: self.bias.round() as u16,
                chans: self.chans,
                frame_len: fl as u8,
                vmotor_bias: self.vb.round() as u16,
                ..Meta::default()
            },
        }
    }
}

fn reg_name(reg: Reg) -> &'static str {
    ALL.iter()
        .find(|(_, r)| *r == reg)
        .map(|(n, _)| *n)
        .unwrap_or("?")
}

/// Drive an experiment against the fake servo; returns the command log
/// ("write <field> <value>" entries, "stream <samples> [<field> <value>]"
/// per burst, plus a trailing marker on overrun). A Stream arm applies its
/// goal at t0, synthesizes the burst's per-tick frames from the plant, and
/// hands them back through `push_tel` - the driver contract.
pub fn pump<E: Experiment>(exp: &mut E, servo: &mut FakeServo, max_steps: u32) -> Vec<String> {
    let mut log = Vec::new();
    let mut pending: Option<TelemetrySnapshot> = None;
    let mut frames = Vec::new();
    for _ in 0..max_steps {
        match exp.step(pending.take().as_ref()) {
            Cmd::Write { reg, value } => {
                servo.write(reg, value);
                log.push(format!("write {} {}", reg_name(reg), value));
            }
            Cmd::Read => pending = Some(servo.read()),
            Cmd::Pause { ms } => servo.advance(ms),
            Cmd::Stream { samples, goal } => {
                match goal {
                    Some((reg, value)) => {
                        servo.write(reg, value);
                        log.push(format!("stream {} {} {}", samples, reg_name(reg), value));
                    }
                    None => log.push(format!("stream {samples}")),
                }
                frames.clear();
                servo.stream(samples, &mut frames);
                exp.push_tel(&frames);
            }
            Cmd::Burst {
                duty_q15,
                pre_q15,
                chans,
                seated,
            } => {
                let pos = servo.pos.round() as u16;
                log.push(format!(
                    "burst {duty_q15} pre {pre_q15} chans {chans} pos {pos}{}",
                    if seated { " seated" } else { "" }
                ));
                let plant = SynthBurst {
                    chans,
                    ..servo.burst.clone()
                };
                let mut cap = plant.capture(duty_q15, pre_q15);
                cap.meta.pos = pos;
                cap.meta.seated = seated;
                exp.push_burst(&cap);
                servo.bursts += 1;
                servo.advance(2);
            }
            Cmd::Done => return log,
        }
    }
    log.push("OVERRUN".into());
    log
}
