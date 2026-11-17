//! Crude integer plant and the fake `ControlIo` that drive `Kernel::on_tick`
//! without a wire - the core kernel tests' closed-loop smoke rig, lifted here
//! so integration tests can script the kernel against it. Identification-
//! model shape: `omega[k+1] = a*omega[k] + b*duty - coulomb`, theta
//! integrates, pot = `theta >> 16` clamped to the 12-bit span,
//! `i = (duty*vbus >> 15 - ke*omega) / R`. Units loose on purpose -
//! deterministic qualitative behaviour, not fidelity.

use crate::sim::DEV_V006_SENSE;
use osc_servo_core::estimator::bemf::{self, RECIP_ARR_SHIFT};
use osc_servo_core::estimator::window;
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::pos_lut::{self, POINTS};
use osc_servo_core::regions::config::ms_ticks;
use osc_servo_core::stamp;
use osc_servo_core::{
    ControlIo, ControlTable, ImageState, Kernel, KernelTiming, Mode, Motor, MotorCmd,
    RegionStorage, SensorFrame, Sensors, Shared, StallResponse, TelStream,
};

/// Shunt zero-current offset the plant's current samples sit on.
pub const BIAS: u16 = 2048;
const ARR: u16 = 1200;

/// A TEL duty in compare counts as the Q15 the table publishes.
pub fn duty_q15(counts: i16) -> i16 {
    (counts as i32 * i16::MAX as i32 / ARR as i32) as i16
}

/// A Q15 duty as the compare counts TEL carries at the plant's period.
pub fn duty_counts(q15: i16) -> i16 {
    window::duty_counts(q15, ARR)
}
/// A non-unity rail scale (22k/10k tap over 20k/10k terminals), so the rig
/// exercises the rescale.
const VBUS_SCALE_Q15: u32 = 34952;
const VBUS: u16 = 3000;

/// `Plant::step` per FAST tick: the duty adds `duty * DUTY_GAIN >> 15` c/s,
/// omega decays by `omega >> DECAY_SHIFT`, Coulomb friction takes
/// `COULOMB_CPS` toward zero; the winding reads back-EMF
/// `omega >> KE_SHIFT` vcounts over R = `1 << R_SHIFT` vcounts per ccount.
const DUTY_GAIN: u32 = 200;
const DECAY_SHIFT: u32 = 6;
const COULOMB_CPS: u32 = 8;
const KE_SHIFT: u32 = 4;
const R_SHIFT: u32 = 1;

/// `Plant::step` in the kernel's motion model, current counts per MEDIUM
/// tick. The duty term is `DUTY_GAIN * (R i + e) / VBUS`, so per FAST tick
/// omega gains `B_f i` with `B_f = DUTY_GAIN R / VBUS` and loses
/// `(2^-DECAY_SHIFT - DUTY_GAIN 2^-KE_SHIFT / VBUS) omega + COULOMB_CPS`:
/// B is `DECIM_MED` of `B_f`, the frictions are their c/s terms over `B_f`.
pub const B_I_Q313: u16 =
    (((DUTY_GAIN << R_SHIFT) * DECIM_MED as u32 * 8192 + VBUS as u32 / 2) / VBUS as u32) as u16;
pub const FRIC_FC_COUNTS: u16 = (COULOMB_CPS * VBUS as u32 / (DUTY_GAIN << R_SHIFT)) as u16;
pub const FRIC_FV_Q016: u16 = (((VBUS as u32) << 16) / (DUTY_GAIN << (DECAY_SHIFT + R_SHIFT))
    - (1 << (16 - KE_SHIFT - R_SHIFT))) as u16;

/// The chip-side const-eval for a 20 kHz FAST rate (MED 2 kHz).
pub const TIMING: KernelTiming = KernelTiming {
    pwm_arr: ARR,
    recip_arr_q24: (1 << RECIP_ARR_SHIFT) / ARR as u32,
    tick_hz: 20_000,
    dt_med_q32: ((1u64 << 32) / 2000) as u32,
    vbus_scale_q15: VBUS_SCALE_Q15,
    bias_brake_min_ticks: 960,
    bemf_min_ticks: 0,
    bridge_min_ticks: 27,
    i_settle_gain: window::SettleGain::UNITY,
    brush_dev_max: bemf::brush_dev_max(
        DEV_V006_SENSE.shunt_r_mohm,
        DEV_V006_SENSE.gain_milli,
        DEV_V006_SENSE.vdd_mv,
    ),
};

pub struct FakeSensors;
impl Sensors for FakeSensors {
    fn land(&mut self, _: &Shared) {}
}

pub struct FakeMotor {
    pub last: Option<MotorCmd>,
}
impl Motor for FakeMotor {
    fn write(&mut self, cmd: MotorCmd) {
        self.last = Some(cmd);
    }
}

pub struct FakeIo {
    pub sensors: FakeSensors,
    pub motor: FakeMotor,
}
impl ControlIo for FakeIo {
    type Sensors = FakeSensors;
    type Motor = FakeMotor;
    fn parts(&mut self) -> (&mut FakeSensors, &mut FakeMotor) {
        (&mut self.sensors, &mut self.motor)
    }
}

pub fn kernel() -> Kernel<FakeIo> {
    Kernel::new(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        TIMING,
    )
}

pub fn last_cmd(k: &Kernel<FakeIo>) -> MotorCmd {
    k.io.motor.last.expect("a motor write happened")
}

/// Hand-stable rig baseline: the gains the core kernel tests settle the
/// plant with, the stored cold R and the motion model at the plant's own,
/// position tracking screen out (the plant/gain pair is qualitative), zero
/// open-loop duty braking. A fully loaded servo: both images loaded, Ke set, the set
/// stamped, so the data state opens every mode.
pub fn seed(shared: &Shared) {
    shared.table.with_mut(|t| {
        let c = &mut t.config;
        c.pos_soft.pos_min_soft_counts = 0;
        c.pos_soft.pos_max_soft_counts = 4095;
        c.loop_current.i_kp_q88 = 256;
        c.loop_current.i_ki_q412 = 200;
        c.loop_current.i_kaw_q412 = 2048;
        c.loop_current.duty_max_q15 = 32767;
        c.loop_velocity.v_kp_q88 = 64;
        c.loop_velocity.v_ki_q412 = 40;
        c.loop_velocity.v_kaw_q412 = 2048;
        c.loop_velocity.j_ff_q88 = 0;
        c.loop_position.p_kp_q88 = 1024;
        c.loop_position.pos_deadband_counts = 8;
        c.loop_position.speed_ceiling_cps = 3000;
        c.loop_position.velocity_limit_cps = 3000;
        c.loop_position.accel_limit_q88 = 50 << 8;
        c.limits.current_limit_counts = 1200;
        c.limits.stall_response = StallResponse::Yield;
        c.limits.drive_polarity = true;
        c.limits.stall_omega_max_cps = 500;
        c.limits.stall_time_ticks = ms_ticks(50);
        c.limits.i_cont_counts = 300;
        c.limits.stall_release_counts = 150;
        c.limits.stall_tau_trip_counts = 1200;
        // inside the 12-bit span above BIAS: the plant's samples clamp at 4095
        c.limits.oc_trip_counts = 1800;
        c.limits.oc_trip_ticks = 4;
        c.limits.openloop_zero_brake = true;
        c.limits.thermal_tau_s = 4;
        c.thermal.cutoff_cc = 10000;
        c.thermal.v_undervolt_counts = 2200;
        // 1200 counts over 200 vcounts: 6 counts per vcount
        c.fault_cfg.rail_fold_q8 = 6 << 8;
        c.fusion.l1_q016 = 16384;
        c.fusion.l2_q88 = 1024;
        c.fusion.l3_q88 = 256;
        c.fault_cfg.pos_error_counts = u16::MAX;
        c.fault_cfg.pos_error_time_ticks = ms_ticks(500);
        c.fault_cfg.sensor_delta_max = 256;
        c.fault_cfg.sensor_bad_count = 4;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = 100;
        cal.sense.v_window_min_ticks = 100;
        cal.motor.ke_vpc_q = 256;
        cal.motor.r_q12 = 8192;
        cal.motor.recip_ke_q = 16 << 10;
        cal.motor.b_i_q313 = B_I_Q313;
        cal.motor.fric_fc_counts = FRIC_FC_COUNTS;
        cal.motor.fric_fv_q016 = FRIC_FV_Q016;
        t.telemetry.refs.current_bias_counts = BIAS;
        t.control.lifecycle.torque_enable = false;
        t.control.lifecycle.mode = Mode::Position;
    });
    stamp(shared);
    shared.publish_data_state(ImageState::Loaded, ImageState::Loaded);
}

/// The host's stamp over the live set, as `osc ident` writes it last.
pub fn stamp(shared: &Shared) {
    shared
        .table
        .with_mut(|t| t.calib.stamp.plant_stamp = stamp::compute(t, None));
}

/// `points` in the array and LIVE, as a COMMIT lands them, restamped over
/// the array so the data state stays open. After `seed`.
pub fn lut_live(shared: &Shared, points: &[i16; POINTS]) {
    shared.with_pos_lut_mut(|k| *k = *points);
    shared.table.with_mut(|t| {
        t.control.pos_lut.pos_lut_state = pos_lut::state::LIVE;
        t.calib.stamp.plant_stamp = stamp::compute(t, points.first_chunk());
    });
    shared.data_state_checkpoint();
}

/// Rail-tap counts that scale to exactly `vmotor` terminal counts.
pub fn vbus_raw(vmotor: u16) -> u16 {
    ((vmotor as u32 * 32768).div_ceil(VBUS_SCALE_Q15)) as u16
}

pub struct Plant {
    theta_q16: i64,
    omega_cps: i32,
    /// The last write's duty. The chip's compares are preloaded and the
    /// kernel writes before the trough, so a write loads there, between
    /// pulses: the period up to the frame after a write runs half on the
    /// write before it and half on it.
    written: i16,
    /// Added to every current sample: a winding short or a sense fault.
    pub current_offset: i32,
}

impl Plant {
    pub fn new(pos: u16) -> Self {
        Self {
            theta_q16: (pos as i64) << 16,
            omega_cps: 0,
            written: 0,
            current_offset: 0,
        }
    }

    /// One FAST tick after the kernel wrote `duty`. The trough scan lands
    /// in the brake phase, so the shunt reads its offset there. The crest
    /// sample sits in the window `duty` drives.
    pub fn step(&mut self, duty: i16) -> SensorFrame {
        let drive = (core::mem::replace(&mut self.written, duty) as i32 + duty as i32) / 2;
        self.omega_cps += ((drive * DUTY_GAIN as i32) >> 15) - (self.omega_cps >> DECAY_SHIFT);
        let coulomb = COULOMB_CPS as i32;
        if self.omega_cps > 0 {
            self.omega_cps = (self.omega_cps - coulomb).max(0);
        } else {
            self.omega_cps = (self.omega_cps + coulomb).min(0);
        }
        self.theta_q16 += ((self.omega_cps as i64) << 16) / 20_000;
        let pos = (self.theta_q16 >> 16).clamp(0, 4095) as u16;
        let v = (drive * VBUS as i32) >> 15;
        let i = (v - (self.omega_cps >> KE_SHIFT)) >> R_SHIFT;
        let mag = if duty >= 0 { i } else { -i };
        let sample = (BIAS as i32 + mag + self.current_offset).clamp(0, 4095) as u16;
        let (va, vb) = if duty >= 0 { (VBUS, 40) } else { (40, VBUS) };
        SensorFrame {
            pos,
            current: sample,
            current_trough: BIAS,
            vmotor_a: va,
            vmotor_a_trough: va,
            vmotor_b: vb,
            vmotor_b_trough: vb,
            vref_raw: 1489,
            vbus_raw: vbus_raw(VBUS),
            ntc_raw: 2048,
        }
    }

    pub fn pos(&self) -> i32 {
        (self.theta_q16 >> 16) as i32
    }

    pub fn omega_cps(&self) -> i32 {
        self.omega_cps
    }
}

/// `RlPlant` rail: 7.9 V (2S) in vmotor-tap counts.
pub const RL_VBUS: u16 = 3204;
/// `RlPlant` winding R, vcounts per ccount Q4.12: the mg90-a `r_q12`
/// (1.775 vcounts per count, 4.9 ohm on the 60 mohm chain).
pub const RL_R_Q12: u16 = 7270;
/// Back-EMF `omega >> RL_KE_SHIFT` vcounts, omega in c/s.
const RL_KE_SHIFT: u32 = 3;
/// Coulomb friction as a current, counts.
const RL_FRIC_COUNTS: i32 = 53;
/// Rotor acceleration per count of net current: 7.58 c/s Q8 per tick.
const RL_ACCEL_Q8_X100: i32 = 758;

/// `RlPlant` in the kernel's motor model: Ke `2^-RL_KE_SHIFT` vcounts per
/// c/s, B `DECIM_MED` ticks of the acceleration per count, the Coulomb
/// current, no viscous term.
pub const RL_KE_VPC_Q12: u16 = 4096 >> RL_KE_SHIFT;
pub const RL_RECIP_KE_Q10: u16 = 1024 << RL_KE_SHIFT;
pub const RL_B_I_Q313: u16 =
    ((RL_ACCEL_Q8_X100 * DECIM_MED as i32 * 8192 + 12_800) / 25_600) as u16;
pub const RL_FRIC_FC_COUNTS: u16 = RL_FRIC_COUNTS as u16;
pub const RL_FRIC_FV_Q016: u16 = 0;

/// The `RlPlant` motor model over `seed`'s, at the plant's default winding.
pub fn rl_motor(t: &mut ControlTable) {
    let m = &mut t.calib.motor;
    m.r_q12 = RL_R_Q12;
    m.ke_vpc_q = RL_KE_VPC_Q12;
    m.recip_ke_q = RL_RECIP_KE_Q10;
    m.b_i_q313 = RL_B_I_Q313;
    m.fric_fc_counts = RL_FRIC_FC_COUNTS;
    m.fric_fv_q016 = RL_FRIC_FV_Q016;
}

/// MG90-scale R-L-back-EMF plant at the FAST tick for the current-limit
/// pins: the winding current closes 1/4 of its gap to `(v - e) / R` each
/// tick (tau_e ~ 3.5 ticks) and the shunt reads it at the crest. Coulomb
/// friction as a current, no load. `locked` holds the rotor anywhere; the
/// hard stops stop it dead while the current pushes into them.
pub struct RlPlant {
    theta_q16: i64,
    omega_q8: i32,
    i: i32,
    /// `i` in 1/256 counts, so the lag settles on `i_ss` from either side.
    i_q8: i32,
    /// The last write's duty, as [`Plant`]'s.
    written: i16,
    pub locked: bool,
    pub stop_lo: i32,
    pub stop_hi: i32,
    /// Load resistance, `r_q12` units; a load under the stored R is a
    /// winding short.
    pub r_q12: u16,
    /// L/R far under the PWM period (a bare resistor): the period-average
    /// current follows the duty within the tick, and the drive-window
    /// sample reads the ON current `(V_bus - e) / R` instead of it.
    pub low_l: bool,
}

impl RlPlant {
    pub fn new(pos: u16) -> Self {
        Self {
            theta_q16: (pos as i64) << 16,
            omega_q8: 0,
            i: 0,
            i_q8: 0,
            written: 0,
            locked: false,
            stop_lo: 0,
            stop_hi: 4095,
            r_q12: RL_R_Q12,
            low_l: false,
        }
    }

    /// One FAST tick after the kernel wrote `duty`. Zero duty is a shorted
    /// winding: the back-EMF alone drives the current. The crest sample
    /// sits in the window `duty` drives.
    pub fn step(&mut self, duty: i16) -> SensorFrame {
        let drive = (core::mem::replace(&mut self.written, duty) as i32 + duty as i32) / 2;
        let v = (drive * RL_VBUS as i32) >> 15;
        let e = (self.omega_q8 >> 8) >> RL_KE_SHIFT;
        let current_at = |v: i32| ((((v - e) as i64) << 12) / self.r_q12 as i64) as i32;
        let i_ss = current_at(v);
        if self.low_l {
            self.i_q8 = i_ss << 8;
        } else {
            self.i_q8 += ((i_ss << 8) - self.i_q8) >> 2;
        }
        self.i = (self.i_q8 + 128) >> 8;
        let rail = RL_VBUS as i32;
        let sensed = if self.low_l && duty != 0 {
            current_at(if duty > 0 { rail } else { -rail })
        } else {
            self.i
        };

        let pos = self.pos();
        let into_stop = (pos >= self.stop_hi && self.i > 0) || (pos <= self.stop_lo && self.i < 0);
        if self.locked || into_stop {
            self.omega_q8 = 0;
        } else {
            let net = if self.omega_q8 > 0 || (self.omega_q8 == 0 && self.i > RL_FRIC_COUNTS) {
                self.i - RL_FRIC_COUNTS
            } else if self.omega_q8 < 0 || self.i < -RL_FRIC_COUNTS {
                self.i + RL_FRIC_COUNTS
            } else {
                0
            };
            let next = self.omega_q8 + net * RL_ACCEL_Q8_X100 / 100;
            // friction stops a coasting rotor, it never reverses it
            let reversed = self.omega_q8 != 0 && (next > 0) != (self.omega_q8 > 0);
            self.omega_q8 = if reversed { 0 } else { next };
            self.theta_q16 += (((self.omega_q8 >> 8) as i64) << 16) / 20_000;
            let (lo, hi) = ((self.stop_lo as i64) << 16, (self.stop_hi as i64) << 16);
            if self.theta_q16 > hi || self.theta_q16 < lo {
                self.theta_q16 = self.theta_q16.clamp(lo, hi);
                self.omega_q8 = 0;
            }
        }

        let mag = if duty >= 0 { sensed } else { -sensed };
        let (va, vb) = if duty >= 0 {
            (RL_VBUS, 40)
        } else {
            (40, RL_VBUS)
        };
        SensorFrame {
            pos: self.pos().clamp(0, 4095) as u16,
            current: (BIAS as i32 + mag).clamp(0, 4095) as u16,
            current_trough: BIAS,
            vmotor_a: va,
            vmotor_a_trough: va,
            vmotor_b: vb,
            vmotor_b_trough: vb,
            vref_raw: 1489,
            vbus_raw: vbus_raw(RL_VBUS),
            ntc_raw: 2048,
        }
    }

    /// Signed period-average winding current, counts: what sets the
    /// torque, and the drive-window sample unless `low_l`.
    pub fn current(&self) -> i32 {
        self.i
    }

    pub fn pos(&self) -> i32 {
        (self.theta_q16 >> 16) as i32
    }

    /// Rotor speed, whole c/s.
    pub fn omega_cps(&self) -> i32 {
        self.omega_q8 >> 8
    }
}

/// The duty the bridge sees after the last write, in the wiring's frame:
/// the logical duty under the Drive's sign, 0 for everything else and
/// before the first write.
pub fn duty_of<T: TelStream>(k: &Kernel<FakeIo, T>) -> i16 {
    match k.io.motor.last {
        Some(MotorCmd::Drive { reverse, .. }) => {
            let duty = k.duty_q15();
            if (duty < 0) == reverse {
                duty
            } else {
                duty.saturating_neg()
            }
        }
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The seed's motion model is the plant it drives: the steady current at
    /// two duties pins the Coulomb and viscous terms, one MEDIUM tick's rise
    /// from rest pins B. Both duties make a whole c/s drive term, so the
    /// plant's truncation stays out of the steady state.
    #[test]
    fn the_seeds_motion_model_is_the_plants() {
        let sh = Shared::new();
        seed(&sh);
        let m = sh.table.with(|t| t.calib.motor);
        let fric =
            |omega: i32| m.fric_fc_counts as f64 + m.fric_fv_q016 as f64 * omega as f64 / 65536.0;
        let current = |f: SensorFrame| f.current as i32 - BIAS as i32;
        for duty in [4096, 8192] {
            let mut p = Plant::new(0);
            let mut i = 0;
            for _ in 0..20_000 {
                i = current(p.step(duty));
            }
            let model = fric(p.omega_cps());
            assert!(
                (i as f64 - model).abs() <= 1.0,
                "duty {duty}: steady {i} counts at {} c/s, the model holds {model:.1}",
                p.omega_cps()
            );

            let mut p = Plant::new(0);
            // the write's first period runs half on the one before: the
            // MEDIUM tick is read from the period after it
            p.step(duty);
            let start = p.omega_cps();
            let mut model = 0.0;
            for _ in 0..DECIM_MED {
                let omega = p.omega_cps();
                let i = current(p.step(duty));
                model += m.b_i_q313 as f64 / 8192.0 * (i as f64 - fric(omega)) / DECIM_MED as f64;
            }
            let rise = (p.omega_cps() - start) as f64;
            assert!(
                (model - rise).abs() <= rise / 16.0,
                "duty {duty}: one MEDIUM tick from rest rose {rise} c/s, the model {model:.1}"
            );
        }
    }

    /// The `RlPlant` rigs' motor model is the plant's: a held bare-resistor
    /// rotor's current pins R, the steady current and speed at two duties
    /// pin Ke and the friction, one MEDIUM tick's rise from rest pins B.
    /// The winding's quarter-gap closure stands up to 3 counts under its
    /// target, so the steady state is read to that.
    #[test]
    fn the_rl_rigs_motor_model_is_the_plants() {
        const CLOSURE: f64 = 3.0;
        const MID: u16 = 2050;
        let sh = Shared::new();
        seed(&sh);
        sh.table.with_mut(rl_motor);
        let m = sh.table.with(|t| t.calib.motor);
        let r = m.r_q12 as f64 / 4096.0;
        let ke = m.ke_vpc_q as f64 / 4096.0;
        let recip = m.recip_ke_q as f64 / 1024.0;
        let fric =
            |omega: i32| m.fric_fc_counts as f64 + m.fric_fv_q016 as f64 * omega as f64 / 65536.0;
        for duty in [8192i16, 16384] {
            let v = ((duty as i32 * RL_VBUS as i32) >> 15) as f64;
            let mut p = RlPlant::new(MID);
            p.locked = true;
            p.low_l = true;
            // the write's first period runs half on the one before
            p.step(duty);
            p.step(duty);
            assert!(
                (p.current() as f64 - v / r).abs() <= 1.0,
                "duty {duty}: held {} counts, the model {:.1}",
                p.current(),
                v / r
            );

            let mut p = RlPlant::new(MID);
            p.stop_hi = i32::MAX >> 16;
            for _ in 0..20_000 {
                p.step(duty);
            }
            let (i, omega) = (p.current(), p.omega_cps());
            let model = fric(omega);
            assert!(
                (i as f64 - model).abs() <= CLOSURE,
                "duty {duty}: steady {i} counts at {omega} c/s, the model holds {model:.1}"
            );
            let e = v - r * i as f64;
            assert!(
                (ke * omega as f64 - e).abs() <= r * CLOSURE + 1.0,
                "duty {duty}: {e:.1} vcounts of back-EMF at {omega} c/s"
            );
            assert!(
                (recip * e - omega as f64).abs() <= recip * (r * CLOSURE + 1.0),
                "duty {duty}: {e:.1} vcounts of back-EMF read {:.0} c/s at {omega} c/s",
                recip * e
            );

            let mut p = RlPlant::new(MID);
            p.step(duty);
            let start = p.omega_cps();
            let mut model = 0.0;
            for _ in 0..DECIM_MED {
                let omega = p.omega_cps();
                p.step(duty);
                model += m.b_i_q313 as f64 / 8192.0 * (p.current() as f64 - fric(omega))
                    / DECIM_MED as f64;
            }
            let rise = (p.omega_cps() - start) as f64;
            assert!(
                (model - rise).abs() <= rise / 16.0,
                "duty {duty}: one MEDIUM tick from rest rose {rise} c/s, the model {model:.1}"
            );
        }
    }
}
