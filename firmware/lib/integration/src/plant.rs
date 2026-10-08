//! Crude integer plant and the fake `ControlIo` that drive `Kernel::on_tick`
//! without a wire - the core kernel tests' closed-loop smoke rig, lifted here
//! so integration tests can script the kernel against it. Identification-
//! model shape: `omega[k+1] = a*omega[k] + b*duty - coulomb`, theta
//! integrates, pot = `theta >> 16` clamped to the 12-bit span,
//! `i = (duty*vbus >> 15 - ke*omega) / R`. Units loose on purpose -
//! deterministic qualitative behaviour, not fidelity.

use osc_servo_core::estimator::bemf::RECIP_ARR_SHIFT;
use osc_servo_core::estimator::window;
use osc_servo_core::pos_lut::{self, POINTS};
use osc_servo_core::stamp;
use osc_servo_core::{
    ControlIo, DecaySelect, ImageState, Kernel, KernelTiming, Mode, Motor, MotorCmd, RegionStorage,
    SensorFrame, Sensors, Shared, StallResponse,
};

/// Shunt zero-current offset the plant's current samples sit on.
pub const BIAS: u16 = 2048;
const ARR: u16 = 1200;
/// A non-unity rail scale (22k/10k tap over 20k/10k terminals), so the rig
/// exercises the rescale.
const VBUS_SCALE_Q15: u32 = 34952;
const VBUS: u16 = 3000;

/// The chip-side const-eval for a 20 kHz FAST rate (MED 2 kHz).
pub const TIMING: KernelTiming = KernelTiming {
    pwm_arr: ARR,
    recip_arr_q24: (1 << RECIP_ARR_SHIFT) / ARR as u32,
    tick_hz: 20_000,
    dt_med_q32: ((1u64 << 32) / 2000) as u32,
    dt_tick_q32: ((1u64 << 32) / 20_000) as u32,
    ticks_per_ms_q16: 20 << 16,
    vbus_scale_q15: VBUS_SCALE_Q15,
    bias_brake_min_ticks: 960,
    v_trough_min_ticks: 0,
    bemf_min_ticks: 0,
    i_settle_gain: window::SettleGain::UNITY,
};

pub struct FakeSensors;
impl Sensors for FakeSensors {
    fn frame(&mut self) -> SensorFrame {
        SensorFrame::default()
    }
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
/// plant with, the stored cold R at the plant's R, position tracking
/// screen out (the plant/gain pair is qualitative), zero open-loop duty
/// braking. A fully loaded servo: both images loaded, Ke set, the set
/// stamped, so the data state opens every mode.
pub fn seed(shared: &Shared) {
    shared.table.with_mut(|t| {
        let c = &mut t.config;
        c.pos_limits.pos_min_soft_counts = 0;
        c.pos_limits.pos_max_soft_counts = 4095;
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
        c.loop_position.velocity_limit_cps = 3000;
        c.loop_position.accel_limit_q88 = 50 << 8;
        c.limits.current_limit_counts = 1200;
        c.limits.stall_response = StallResponse::Yield;
        c.limits.drive_polarity = true;
        c.limits.stall_omega_max_cps = 500;
        c.limits.stall_time_ms = 50;
        c.limits.stall_yield_counts = 300;
        c.limits.stall_release_counts = 150;
        c.limits.stall_tau_trip_counts = 1200;
        // inside the 12-bit span above BIAS: the plant's samples clamp at 4095
        c.limits.oc_trip_counts = 1800;
        c.limits.oc_trip_ticks = 4;
        c.limits.openloop_decay = DecaySelect::Slow;
        c.limits.openloop_zero_brake = true;
        c.thermal.derate_start_cc = 8000;
        c.thermal.cutoff_cc = 10000;
        c.thermal.recover_cc = 9000;
        c.thermal.v_undervolt_counts = 2200;
        c.thermal.rtherm_i_min_counts = 300;
        c.fusion.l1_q016 = 16384;
        c.fusion.l2_q88 = 1024;
        c.fusion.l3_q88 = 256;
        c.fault_cfg.pos_error_counts = u16::MAX;
        c.fault_cfg.pos_error_time_ms = 500;
        c.fault_cfg.sensor_delta_max = 256;
        c.fault_cfg.sensor_bad_count = 4;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = 100;
        cal.sense.v_window_min_ticks = 100;
        cal.winding.r0_q12 = 8192;
        cal.motor.ke_vpc_q = 256;
        cal.motor.r_q12 = 8192;
        cal.motor.recip_ke_q = 16 << 10;
        cal.motor.b_i_q313 = 8192;
        t.telemetry.sensors.current_bias_counts = BIAS;
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
    /// Added to every current sample: a winding short or a sense fault.
    pub current_offset: i32,
}

impl Plant {
    pub fn new(pos: u16) -> Self {
        Self {
            theta_q16: (pos as i64) << 16,
            omega_cps: 0,
            current_offset: 0,
        }
    }

    /// One FAST tick under the duty the kernel last wrote: a = 1 - 1/64,
    /// b*duty = duty*200>>15, coulomb 8 c/s toward zero. Slow-decay frames:
    /// the trough scan lands in the brake phase, so the shunt reads its
    /// offset there.
    pub fn step(&mut self, duty: i16) -> SensorFrame {
        self.omega_cps += ((duty as i32 * 200) >> 15) - (self.omega_cps >> 6);
        if self.omega_cps > 0 {
            self.omega_cps = (self.omega_cps - 8).max(0);
        } else {
            self.omega_cps = (self.omega_cps + 8).min(0);
        }
        self.theta_q16 += ((self.omega_cps as i64) << 16) / 20_000;
        let pos = (self.theta_q16 >> 16).clamp(0, 4095) as u16;
        // ke = 1/16 vcounts per c/s, R = 2 vcounts/ccount
        let v = (duty as i32 * VBUS as i32) >> 15;
        let i = (v - (self.omega_cps >> 4)) >> 1;
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
            vcal: 1200,
            vbus_raw: vbus_raw(VBUS),
            ntc_raw: 2048,
            tick: 0,
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

/// MG90-scale R-L-back-EMF plant at the FAST tick for the current-limit
/// pins: the winding current closes 1/4 of its gap to `(v - e) / R` each
/// tick (tau_e ~ 3.5 ticks) and the shunt reads it at the crest. Coulomb
/// friction as a current, no load. `locked` holds the rotor anywhere; the
/// hard stops stop it dead while the current pushes into them.
pub struct RlPlant {
    theta_q16: i64,
    omega_q8: i32,
    i: i32,
    pub locked: bool,
    pub stop_lo: i32,
    pub stop_hi: i32,
}

impl RlPlant {
    pub fn new(pos: u16) -> Self {
        Self {
            theta_q16: (pos as i64) << 16,
            omega_q8: 0,
            i: 0,
            locked: false,
            stop_lo: 0,
            stop_hi: 4095,
        }
    }

    /// One FAST tick under the duty the kernel last wrote. Zero duty is a
    /// shorted winding: the back-EMF alone drives the current.
    pub fn step(&mut self, duty: i16) -> SensorFrame {
        let v = (duty as i32 * RL_VBUS as i32) >> 15;
        let e = (self.omega_q8 >> 8) >> RL_KE_SHIFT;
        let i_ss = (((v - e) as i64) << 12) / RL_R_Q12 as i64;
        self.i += (i_ss as i32 - self.i) >> 2;

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

        let mag = if duty >= 0 { self.i } else { -self.i };
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
            vcal: 1200,
            vbus_raw: vbus_raw(RL_VBUS),
            ntc_raw: 2048,
            tick: 0,
        }
    }

    /// Signed winding current, counts: what the shunt would read in the
    /// drive window.
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

/// The duty the bridge sees for `cmd`: Drive's, 0 for everything else.
pub fn duty_of(cmd: MotorCmd) -> i16 {
    match cmd {
        MotorCmd::Drive { duty, .. } => duty.0,
        MotorCmd::Disabled | MotorCmd::Coast | MotorCmd::Brake => 0,
    }
}
