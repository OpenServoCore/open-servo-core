//! The bench MG90 as a plant, in linearized counts: an R-L winding with
//! back-EMF (Brake shorts it, Coast opens it), Coulomb plus viscous
//! friction, 5.7 counts of gear lash between the rotor and the pot, and a
//! pot with 1.2 raw counts of noise that wanders +/-5 counts off the position
//! table, the residual linearization leaves. A load torque, a heavier
//! output, a held rotor, leads wired the other way round and a brush
//! contact's drop while the rotor turns ([`Contact`]) are the scenarios'
//! knobs. [`rig`] is the bench MG90's saved gains and limits (2S
//! rail, 280-count limit, deadband 12) over the plant's own motor model and
//! [`kernel`] its kernel, both on the
//! osc-dev-v006 board's window floors and shunt settle bands: the floor
//! decides what the current loop reads on every small command, so a rig on
//! another board's floors runs another loop. The plant's shunt reads a
//! drive window short of the amplifier's settle low by the board's band for
//! that width, the bands the kernel scales it back up by.

use osc_servo_core::estimator::thermal::{ALPHA_Q26, NtcCfg, R_REF_CC, ntc_cc};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::pos_lut::{GRID, POINTS};
use osc_servo_core::regions::config::ms_ticks;
use osc_servo_core::{
    ControlTable, Kernel, KernelTiming, MotorCmd, RegionStorage, SensorFrame, Shared, StallResponse,
};

use crate::plant::{self, BIAS, FakeIo, FakeMotor, FakeSensors, lut_live, seed, vbus_raw};
use crate::sim::{DEV_V006_BEMF_MIN_TICKS, DEV_V006_I_SETTLE_GAIN, DEV_V006_SENSE};

/// 7.1 V in vmotor-tap counts.
pub const VBUS: f64 = 2880.0;
pub const R: f64 = 1.775;
pub const KE: f64 = 0.14162;
/// c/s per FAST tick per current count.
pub const B: f64 = 0.03083;
pub const FC: f64 = 53.19;
pub const FV: f64 = 0.005135;
/// L/R = 0.87 mH / 4.89 ohm, in FAST ticks.
pub const TAU_E: f64 = 3.56;
pub const TICK_HZ: f64 = 20_000.0;
/// The pot's noise, raw counts rms: a plant's `sigma_raw` unless set.
pub const SIGMA_RAW: f64 = 1.2;
pub const LASH: f64 = 5.7;
/// The pot's residual as `(amplitude, period)` sine lines, linearized
/// counts: a plant's `residual` unless set.
pub const RESIDUAL: &[(f64, f64)] = &[(5.0, 60.0)];
pub const ARR: u32 = 1200;
/// The board NTC's reading, raw counts: steady, the board never warms.
pub const NTC_RAW: u16 = 2048;

/// A plant's rail and motor in the board's counts: the rail in vcounts, R
/// in vcounts per ccount, Ke in vcounts per c/s, B in c/s per FAST tick
/// per ccount, the Coulomb term in ccounts and the viscous one in ccounts
/// per c/s.
#[derive(Clone, Copy, Debug)]
pub struct Motor {
    pub vbus: f64,
    pub r: f64,
    pub ke: f64,
    pub b: f64,
    pub fc: f64,
    pub fv: f64,
}

/// What the brush contact drops beyond the winding's resistance while the
/// rotor turns, odd in the current: nothing up to `from` ccounts, `slope`
/// vcounts per ccount from there to `to`, and flat past it. At rest the
/// contact holds still and drops nothing.
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub from: f64,
    pub to: f64,
    pub slope: f64,
}

impl Contact {
    /// The drop at current `i`, vcounts, and its slope there.
    pub fn at(&self, i: f64) -> (f64, f64) {
        let a = i.abs();
        let drop = i.signum() * self.slope * (a.min(self.to) - self.from).max(0.0);
        (
            drop,
            if a > self.from && a < self.to {
                self.slope
            } else {
                0.0
            },
        )
    }
}

/// The bench MG90's, on its 2S rail.
pub const MOTOR: Motor = Motor {
    vbus: VBUS,
    r: R,
    ke: KE,
    b: B,
    fc: FC,
    fv: FV,
};

/// The generic rig's timing with the board's back-EMF observer floor and
/// its shunt settle bands.
pub const TIMING: KernelTiming = KernelTiming {
    bemf_min_ticks: DEV_V006_BEMF_MIN_TICKS,
    i_settle_gain: DEV_V006_I_SETTLE_GAIN,
    ..plant::TIMING
};

pub fn kernel() -> Kernel<FakeIo> {
    Kernel::new(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        TIMING,
    )
}

pub struct Rng(pub u64);

impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    pub fn unit(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    pub fn gauss(&mut self) -> f64 {
        let (u, v) = (self.unit(), self.unit());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// The table's inverse: the raw count it maps onto `lin`.
pub fn raw_of(lut: &[i16; POINTS], lin: f64) -> f64 {
    let at = |k: usize| (k * GRID) as f64 + lut[k] as f64;
    let k = (0..POINTS - 1)
        .find(|&k| at(k + 1) > lin)
        .unwrap_or(POINTS - 2);
    (k * GRID) as f64 + GRID as f64 * (lin - at(k)) / (at(k + 1) - at(k))
}

/// Rotor and pot shaft in linearized counts, the pot riding the rotor
/// through the lash.
pub struct Mg90 {
    pub rotor: f64,
    pub out: f64,
    pub omega: f64,
    pub i: f64,
    pub motor: Motor,
    rng: Rng,
    lut: [i16; POINTS],
    /// A torque on the output as a winding current, counts: positive turns
    /// the counts up.
    pub load: f64,
    /// Output inertia over the bench MG90's.
    pub inertia: f64,
    pub locked: bool,
    /// The motor leads swapped at the motor: the bridge and its sensing
    /// see the same wiring, the rotor turns the other way.
    pub swapped: bool,
    /// An ADC wide code, `(stuck, skipped)`: the reading sticks at `stuck`
    /// while the input rises through the `skipped` codes above it.
    pub wide: Option<(u16, u16)>,
    /// The pot's noise, raw counts rms.
    pub sigma_raw: f64,
    /// The residual the pot wanders off the table by, `(amplitude,
    /// period)` sine lines in linearized counts, summed: `&[]` is none.
    pub residual: &'static [(f64, f64)],
    /// The winding's L over its resistance, FAST ticks.
    pub tau_e: f64,
    /// The brush contact's drop while the rotor turns: `None` is none.
    pub contact: Option<Contact>,
    /// The last write's volts, `None` an open winding. The chip's compares
    /// are preloaded and the kernel writes before the trough, so a write
    /// loads there, between pulses: the period up to the frame after a
    /// write runs half on the write before it and half on it, and a period
    /// with an open half carries no current.
    written: Option<f64>,
    /// The volts the last period stepped ran on.
    drive: Option<f64>,
    /// The iron's loss conductance across the inductance, ccounts per
    /// vcount: the shunt reads it times the drive window's volts across
    /// the inductance on top of the winding current, along the drive. It
    /// carries no torque and averages to nothing over a period. 0 is none.
    pub window_g: f64,
}

impl Mg90 {
    pub fn new(lin: f64, lut: [i16; POINTS], seed: u64) -> Self {
        Self {
            rotor: lin,
            out: lin,
            omega: 0.0,
            i: 0.0,
            motor: MOTOR,
            rng: Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1),
            lut,
            load: 0.0,
            inertia: 1.0,
            locked: false,
            swapped: false,
            wide: None,
            sigma_raw: SIGMA_RAW,
            residual: RESIDUAL,
            tau_e: TAU_E,
            contact: None,
            written: None,
            drive: None,
            window_g: 0.0,
        }
    }

    /// The same plant on `motor`.
    pub fn with_motor(self, motor: Motor) -> Self {
        Self { motor, ..self }
    }

    /// The linearized position the kernel reads, noise aside.
    pub fn sensed(&self) -> f64 {
        self.residual.iter().fold(self.out, |s, &(a, period)| {
            s - a * (std::f64::consts::TAU * self.out / period).sin()
        })
    }

    /// One FAST tick after the kernel wrote `cmd`, with the chip's mapping:
    /// a zero width coasts, Brake shorts the winding, Coast and Disabled
    /// open it. The crest sample sits in the window `cmd` drives.
    pub fn step(&mut self, cmd: Option<MotorCmd>) -> SensorFrame {
        let m = self.motor;
        let wiring = if self.swapped { -1.0 } else { 1.0 };
        let volts = match cmd {
            Some(MotorCmd::Drive { ticks, reverse, .. }) => (ticks != 0).then(|| {
                let v = ticks as f64 / ARR as f64 * m.vbus;
                wiring * if reverse { -v } else { v }
            }),
            Some(MotorCmd::Brake) => Some(0.0),
            _ => None,
        };
        self.drive = core::mem::replace(&mut self.written, volts)
            .zip(volts)
            .map(|(a, b)| (a + b) / 2.0);
        self.i = match self.drive {
            // the contact's drop on its tangent at this current: the winding
            // relaxes on R plus its slope, the step exact for the tangent
            Some(v) => match self.contact.filter(|_| self.omega != 0.0) {
                None => {
                    self.i
                        + ((v - m.ke * self.omega) / m.r - self.i)
                            * (1.0 - (-1.0 / self.tau_e).exp())
                }
                Some(c) => {
                    let (x, s) = c.at(self.i);
                    let r = m.r + s;
                    let target = (v - m.ke * self.omega - x + s * self.i) / r;
                    self.i + (target - self.i) * (1.0 - (-r / m.r / self.tau_e).exp())
                }
            },
            None => 0.0,
        };
        let torque = self.i + self.load;
        if self.locked {
            self.omega = 0.0;
        } else if self.omega != 0.0 || torque.abs() > m.fc {
            let dir = if self.omega != 0.0 {
                self.omega.signum()
            } else {
                torque.signum()
            };
            let next = self.omega + m.b / self.inertia * (torque - m.fc * dir - m.fv * self.omega);
            // friction stops a coasting rotor, it never reverses it
            self.omega = if self.omega != 0.0 && next.signum() != self.omega.signum() {
                0.0
            } else {
                next
            };
        }
        self.rotor += self.omega / TICK_HZ;
        self.out = self
            .out
            .clamp(self.rotor - LASH / 2.0, self.rotor + LASH / 2.0);
        let raw = raw_of(&self.lut, self.sensed()) + self.sigma_raw * self.rng.gauss();
        let mut pos = raw.round().clamp(0.0, 4095.0) as u16;
        if let Some((stuck, skipped)) = self.wide
            && pos > stuck
            && pos <= stuck + skipped
        {
            pos = stuck;
        }
        let fwd = !matches!(cmd, Some(MotorCmd::Drive { reverse: true, .. }));
        let bridge_i = wiring * self.i * Self::settle(cmd);
        let (mut sample, va, vb) = if fwd {
            (bridge_i, m.vbus as u16, 0)
        } else {
            (-bridge_i, 0, m.vbus as u16)
        };
        if self.window_g != 0.0 {
            let along = if fwd { wiring } else { -wiring };
            sample += self.window(cmd, along) * Self::settle(cmd);
        }
        SensorFrame {
            pos,
            current: (BIAS as f64 + sample).round().clamp(0.0, 4095.0) as u16,
            current_trough: BIAS,
            vmotor_a: va,
            vmotor_a_trough: va,
            vmotor_b: vb,
            vmotor_b_trough: vb,
            vref_raw: 1489,
            vbus_raw: vbus_raw(m.vbus as u16),
            ntc_raw: NTC_RAW,
        }
    }

    /// The current the shunt reads in `cmd`'s drive window beyond the
    /// winding's, along the drive (`along` is the drive's sign in the
    /// winding's frame): `window_g` times the volts the window leaves
    /// across the inductance, the rail less the winding's resistive drop,
    /// back-EMF and contact along the drive. Settled, that is `(1 - D)`
    /// times the rail; a start from rest reads the whole rail. 0 with no
    /// drive window.
    fn window(&self, cmd: Option<MotorCmd>, along: f64) -> f64 {
        match cmd {
            Some(MotorCmd::Drive { ticks, .. }) if ticks != 0 => {
                let m = self.motor;
                let x = self
                    .contact
                    .filter(|_| self.omega != 0.0)
                    .map_or(0.0, |c| c.at(self.i).0);
                self.window_g * (m.vbus - along * (m.r * self.i + m.ke * self.omega + x))
            }
            _ => 0.0,
        }
    }

    /// What the shunt reads of the winding current in the drive window
    /// `cmd` sets: low by the board's settle band for its width.
    fn settle(cmd: Option<MotorCmd>) -> f64 {
        match cmd {
            Some(MotorCmd::Drive { ticks, .. }) if ticks != 0 => {
                32768.0 / DEV_V006_I_SETTLE_GAIN.q15_at(ticks as u32) as f64
            }
            _ => 1.0,
        }
    }

    /// What the rail supplied over the last period stepped: the winding
    /// current over the on fraction.
    pub fn supply(&self) -> f64 {
        self.drive
            .map_or(0.0, |v| (self.i * v / self.motor.vbus).abs())
    }
}

/// The bench MG90's saved gains and limits with the plant's own motor
/// model, on the board's shunt and terminal window floors, `lut` live, then
/// `cfg`.
pub fn rig(lut: &[i16; POINTS], cfg: impl FnOnce(&mut ControlTable)) -> Shared {
    let q = |x: f64, frac: u32| (x * (1u32 << frac) as f64).round() as u16;
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        let c = &mut t.config;
        c.loop_current.i_kp_q88 = 508;
        c.loop_current.i_ki_q412 = 2284;
        c.loop_current.i_kaw_q412 = 4568;
        c.loop_velocity.v_kp_q88 = 522;
        c.loop_velocity.v_ki_q412 = 1312;
        c.loop_velocity.v_kaw_q412 = 2624;
        c.loop_velocity.j_ff_q88 = 831;
        c.loop_position.p_kp_q88 = 40212;
        c.loop_position.pos_deadband_counts = 12;
        c.loop_position.speed_ceiling_cps = 1500;
        c.loop_position.velocity_limit_cps = 1500;
        c.loop_position.accel_limit_q88 = 15 << 8;
        c.limits.current_limit_counts = 280;
        c.limits.stall_response = StallResponse::Yield;
        c.limits.stall_time_ticks = ms_ticks(500);
        c.limits.i_cont_counts = 168;
        c.limits.stall_release_counts = 84;
        c.limits.stall_tau_trip_counts = 2182;
        c.limits.oc_trip_counts = 1843;
        c.limits.oc_trip_ticks = 8;
        c.limits.openloop_zero_brake = false;
        c.thermal.v_undervolt_counts = 1200;
        c.fusion.l1_q016 = 8640;
        c.fusion.l2_q88 = 3180;
        c.fusion.l3_q88 = 81;
        c.fault_cfg.pos_error_counts = 400;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = DEV_V006_SENSE.i_window_min_ticks;
        cal.sense.v_window_min_ticks = DEV_V006_SENSE.v_window_min_ticks;
        cal.motor.r_q12 = q(R, 12);
        cal.motor.ke_vpc_q = q(KE, 12);
        cal.motor.recip_ke_q = q(1.0 / KE, 10);
        cal.motor.b_i_q313 = q(B * DECIM_MED as f64, 13);
        cal.motor.fric_fc_counts = q(FC, 0);
        cal.motor.fric_fv_q016 = q(FV, 16);
    });
    sh.table.with_mut(cfg);
    lut_live(&sh, lut);
    sh
}

/// The winding resistance a set's fields hold at the plant's board,
/// vcounts per count: `r_q12`, the winding at 25.00 C, along copper's
/// slope to the temperature the set's NTC curve reads at [`NTC_RAW`], as
/// the kernel reads it; `r_q12` itself with no curve.
pub fn stored_r(t: &ControlTable) -> f64 {
    let th = &t.calib.thermal;
    let curve = NtcCfg {
        raw_ref: th.ntc_raw_ref,
        t_ref_cc: th.ntc_t_ref_cc,
        k1_q88: th.ntc_k1_q88,
        k2_q24: th.ntc_k2_q24,
    };
    let d = ntc_cc(NTC_RAW, &curve).map_or(0, |t| t as i32 - R_REF_CC);
    let alpha = ALPHA_Q26 as f64 / (1u64 << 26) as f64;
    t.calib.motor.r_q12 as f64 / 4096.0 * (1.0 + alpha * d as f64)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    fn drive(ticks: u16) -> Option<MotorCmd> {
        Some(MotorCmd::Drive {
            ticks,
            reverse: false,
        })
    }

    /// The rig's motor model is the plant's: a held rotor's current pins R,
    /// the steady current and speed at two widths pin Ke and the friction
    /// past the brush contact's drop, one MEDIUM tick's rise from rest pins
    /// B. Over every width from the
    /// board's shunt floor past the last settle band, the held current the
    /// shunt reads, scaled by the kernel's band, is the winding's and the
    /// plant's window term to the sample's rounding.
    #[test]
    fn the_rigs_motor_model_is_the_plants() {
        motor_model_is_the_plants(
            &rig(&[0; POINTS], |_| {}),
            || Mg90::new(2048.0, [0; POINTS], 1),
            None,
        );
    }

    /// The pin above for the motor model `sh` stores against the plants
    /// `plant` builds at rest; the drag `(Coulomb, viscous)` where the
    /// plant's is not the stored friction line.
    pub(crate) fn motor_model_is_the_plants(
        sh: &Shared,
        plant: impl Fn() -> Mg90,
        drag: Option<(f64, f64)>,
    ) {
        let (m, r) = sh.table.with(|t| (t.calib.motor, stored_r(t)));
        let ke = m.ke_vpc_q as f64 / 4096.0;
        let recip = m.recip_ke_q as f64 / 1024.0;
        let (fc, fv) = drag.unwrap_or((m.fric_fc_counts as f64, m.fric_fv_q016 as f64 / 65536.0));
        let fric = |omega: f64| fc + fv * omega;
        // the winding's current, the shunt's sample and the window term the
        // sample carries on top
        let held = |ticks: u16| {
            let mut p = plant();
            p.locked = true;
            let mut f = p.step(drive(ticks));
            for _ in 0..200 {
                f = p.step(drive(ticks));
            }
            let w = p.window(drive(ticks), 1.0);
            (p.i, f.current as f64 - BIAS as f64, w)
        };
        for ticks in [300u16, 600] {
            let v = ticks as f64 / ARR as f64 * plant().motor.vbus;
            let (i, _, _) = held(ticks);
            assert!(
                (i - v / r).abs() <= 1.0,
                "{ticks} ticks: held {i:.1} counts, the model {:.1}",
                v / r
            );

            let mut p = plant();
            for _ in 0..20_000 {
                p.step(drive(ticks));
            }
            let model = fric(p.omega);
            assert!(
                (p.i - model).abs() <= 1.0,
                "{ticks} ticks: steady {:.1} counts at {:.0} c/s, the model holds {model:.1}",
                p.i,
                p.omega
            );
            let contact = p.contact.map_or(0.0, |c| c.at(p.i).0);
            let e = v - r * p.i - contact;
            assert!(
                (ke * p.omega - e).abs() <= 1.0,
                "{ticks} ticks: {e:.1} vcounts of back-EMF at {:.0} c/s",
                p.omega
            );
            assert!(
                (recip * e - p.omega).abs() <= recip,
                "{ticks} ticks: {e:.1} vcounts of back-EMF read {:.0} c/s at {:.0} c/s",
                recip * e,
                p.omega
            );

            let mut p = plant();
            // the write's first period runs half on the one before: the
            // MEDIUM tick is read from the period after it
            p.step(drive(ticks));
            let start = p.omega;
            let mut model = 0.0;
            for _ in 0..DECIM_MED {
                let omega = p.omega;
                p.step(drive(ticks));
                model += m.b_i_q313 as f64 / 8192.0 * (p.i - fric(omega)) / DECIM_MED as f64;
            }
            let rise = p.omega - start;
            assert!(
                (model - rise).abs() <= rise / 16.0,
                "{ticks} ticks: one MEDIUM tick from rest rose {rise:.1} c/s, the model {model:.1}"
            );
        }

        let floor = sh.table.with(|t| t.calib.sense.i_window_min_ticks);
        let past = DEV_V006_I_SETTLE_GAIN.start_ticks + 8 * DEV_V006_I_SETTLE_GAIN.q15.len() as u16;
        for ticks in floor..=past {
            let (i, sample, w) = held(ticks);
            let gain = TIMING.i_settle_gain.q15_at(ticks as u32) as f64 / 32768.0;
            assert!(
                (sample * gain - (i + w)).abs() <= 0.5 * gain,
                "{ticks} ticks: the shunt reads {sample}, x{gain:.4} of {i:.2} counts and {w:.2} of window"
            );
        }
    }
}
