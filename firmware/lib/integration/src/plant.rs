//! Crude integer plant and the fake `ControlIo` that drive `Kernel::on_tick`
//! without a wire - the core kernel tests' closed-loop smoke rig, lifted here
//! so integration tests can script the kernel against it. Identification-
//! model shape: `omega[k+1] = a*omega[k] + b*duty - coulomb`, theta
//! integrates, pot = `theta >> 16` clamped to the 12-bit span,
//! `i = (duty*vbus >> 15 - ke*omega) / R`. Units loose on purpose -
//! deterministic qualitative behaviour, not fidelity.

use osc_servo_core::estimator::bemf::RECIP_ARR_SHIFT;
use osc_servo_core::pos_lut::{self, POINTS};
use osc_servo_core::stamp;
use osc_servo_core::{
    ControlIo, DecaySelect, ImageState, Kernel, KernelTiming, Mode, Motor, MotorCmd, RegionStorage,
    SensorFrame, Sensors, Shared, StallResponse,
};

/// Shunt zero-current offset the plant's current samples sit on.
pub const BIAS: u16 = 2048;
const ARR: u16 = 1200;
/// osc-dev-v006 board D rail scale (22k/10k tap over 20k/10k terminals).
const VBUS_SCALE_Q15: u32 = 34952;
const VBUS: u16 = 3000;

/// The chip-side const-eval for a 20 kHz FAST rate (MED 2 kHz).
pub const TIMING: KernelTiming = KernelTiming {
    pwm_arr: ARR,
    recip_arr_q24: (1 << RECIP_ARR_SHIFT) / ARR as u32,
    tick_hz: 20_000,
    dt_med_q32: ((1u64 << 32) / 2000) as u32,
    med_ticks_per_ms_q16: 2 << 16,
    vbus_scale_q15: VBUS_SCALE_Q15,
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
/// plant with, the winding anchor at the plant's R, position tracking
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
        c.thermal.rtherm_omega_max_cps = 400;
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
        cal.winding.t0_cc = 2500;
        cal.winding.k_r2t_q88 = 512;
        cal.winding.mu_q016 = 6554;
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

/// The duty the bridge sees for `cmd`: Drive's, 0 for everything else.
pub fn duty_of(cmd: MotorCmd) -> i16 {
    match cmd {
        MotorCmd::Drive { duty, .. } => duty.0,
        MotorCmd::Disabled | MotorCmd::Coast | MotorCmd::Brake => 0,
    }
}
