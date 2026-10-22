//! The bench MG90 as a plant, in linearized counts: an R-L winding with
//! back-EMF (Brake shorts it, Coast opens it), Coulomb plus viscous
//! friction, 5.7 counts of gear lash between the rotor and the pot, and a
//! pot with 1.2 raw counts of noise that wanders +/-5 counts off the position
//! table, the residual linearization leaves. A load torque, a heavier
//! output, a held rotor and leads wired the other way round are the
//! scenarios' knobs. [`rig`] is the bench MG90's identified set (2S rail,
//! 280-count limit, deadband 12).

use osc_servo_core::pos_lut::{GRID, POINTS};
use osc_servo_core::regions::config::ms_ticks;
use osc_servo_core::{ControlTable, MotorCmd, RegionStorage, SensorFrame, Shared, StallResponse};

use crate::plant::{BIAS, lut_live, seed, vbus_raw};

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
pub const SIGMA_RAW: f64 = 1.2;
pub const LASH: f64 = 5.7;
/// Amplitude and period, linearized counts.
pub const RESIDUAL: (f64, f64) = (5.0, 60.0);
pub const ARR: u32 = 1200;

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
}

impl Mg90 {
    pub fn new(lin: f64, lut: [i16; POINTS], seed: u64) -> Self {
        Self {
            rotor: lin,
            out: lin,
            omega: 0.0,
            i: 0.0,
            rng: Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1),
            lut,
            load: 0.0,
            inertia: 1.0,
            locked: false,
            swapped: false,
        }
    }

    /// The linearized position the kernel reads, noise aside.
    pub fn sensed(&self) -> f64 {
        self.out - RESIDUAL.0 * (std::f64::consts::TAU * self.out / RESIDUAL.1).sin()
    }

    /// One FAST tick under `cmd`, with the chip's mapping: a zero width
    /// coasts, Brake shorts the winding, Coast and Disabled open it.
    pub fn step(&mut self, cmd: Option<MotorCmd>) -> SensorFrame {
        let wiring = if self.swapped { -1.0 } else { 1.0 };
        let volts = match cmd {
            Some(MotorCmd::Drive { ticks, reverse, .. }) => (ticks != 0).then(|| {
                let v = ticks as f64 / ARR as f64 * VBUS;
                wiring * if reverse { -v } else { v }
            }),
            Some(MotorCmd::Brake) => Some(0.0),
            _ => None,
        };
        self.i = match volts {
            Some(v) => self.i + ((v - KE * self.omega) / R - self.i) * (1.0 - (-1.0 / TAU_E).exp()),
            None => 0.0,
        };
        let torque = self.i + self.load;
        if self.locked {
            self.omega = 0.0;
        } else if self.omega != 0.0 || torque.abs() > FC {
            let dir = if self.omega != 0.0 {
                self.omega.signum()
            } else {
                torque.signum()
            };
            let next = self.omega + B / self.inertia * (torque - FC * dir - FV * self.omega);
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
        let raw = raw_of(&self.lut, self.sensed()) + SIGMA_RAW * self.rng.gauss();
        let fwd = !matches!(cmd, Some(MotorCmd::Drive { reverse: true, .. }));
        let bridge_i = wiring * self.i;
        let (sample, va, vb) = if fwd {
            (bridge_i, VBUS as u16, 0)
        } else {
            (-bridge_i, 0, VBUS as u16)
        };
        SensorFrame {
            pos: raw.round().clamp(0.0, 4095.0) as u16,
            current: (BIAS as f64 + sample).round().clamp(0.0, 4095.0) as u16,
            current_trough: BIAS,
            vmotor_a: va,
            vmotor_a_trough: va,
            vmotor_b: vb,
            vmotor_b_trough: vb,
            vcal: 1200,
            vbus_raw: vbus_raw(VBUS as u16),
            ntc_raw: 2048,
        }
    }

    /// What the rail supplies: the winding current over the on fraction.
    pub fn supply(&self, cmd: Option<MotorCmd>) -> f64 {
        match cmd {
            Some(MotorCmd::Drive { ticks, .. }) => (self.i * ticks as f64 / ARR as f64).abs(),
            _ => 0.0,
        }
    }
}

/// The bench MG90's saved set, identified in linearized counts, `lut` live,
/// then `cfg`.
pub fn rig(lut: &[i16; POINTS], cfg: impl FnOnce(&mut ControlTable)) -> Shared {
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
        cal.sense.i_window_min_ticks = 160;
        cal.sense.v_window_min_ticks = 160;
        cal.motor.r_q12 = 7270;
        cal.motor.ke_vpc_q = 580;
        cal.motor.recip_ke_q = 7228;
        cal.motor.b_i_q313 = 2524;
        cal.motor.fric_fc_counts = 53;
        cal.motor.fric_fv_q016 = 336;
        cal.winding.r0_q12 = 7270;
    });
    sh.table.with_mut(cfg);
    lut_live(&sh, lut);
    sh
}
