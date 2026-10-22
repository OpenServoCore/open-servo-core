//! Kernel-level orchestration tests plus the closed-loop plant smoke suite.
//! The per-block tests carry the numeric precision; these pin the wiring:
//! gate, ack, dispatch shapes, publishes, and qualitative closed-loop
//! behavior against a crude integer plant.

use super::phases::fast;
use super::phases::loops::{duty_limit, faults, limits};
use super::*;
use crate::budget::Events;
use crate::estimator::{OmegaSource, bemf, bias, window};
use crate::math::q_mul;
use crate::pos_lut;
use crate::regions::config::{StallResponse, ms_ticks};
use crate::regions::control::Mode;
use crate::regions::telemetry::tick_flag;
use crate::traits::{ControlIo, DecayMode, Motor, MotorCmd, Sensors, TelStream};
use crate::{ControlTable, RegionStorage, SensorFrame, Shared, TelemetryTick};

const BIAS: u16 = 2048;
const ARR: u16 = 1200;
/// A non-unity rail scale (22k/10k tap over 20k/10k terminals), so the rig
/// exercises the rescale.
const VBUS_SCALE_Q15: u32 = 34952;

// The chip-side const-eval for a 20 kHz FAST rate (MED 2 kHz).
const TIMING: KernelTiming = KernelTiming {
    pwm_arr: ARR,
    recip_arr_q24: (1 << bemf::RECIP_ARR_SHIFT) / ARR as u32,
    tick_hz: 20_000,
    dt_med_q32: ((1u64 << 32) / 2000) as u32,
    vbus_scale_q15: VBUS_SCALE_Q15,
    bias_brake_min_ticks: 600,
    v_trough_min_ticks: 0,
    bemf_min_ticks: 0,
    i_settle_gain: window::SettleGain::UNITY,
};

struct FakeSensors;
impl Sensors for FakeSensors {
    fn land(&mut self, _: &Shared) {}
}

struct FakeMotor {
    last: Option<MotorCmd>,
}
impl Motor for FakeMotor {
    fn write(&mut self, cmd: MotorCmd) {
        self.last = Some(cmd);
    }
}

struct FakeIo {
    sensors: FakeSensors,
    motor: FakeMotor,
}
impl ControlIo for FakeIo {
    type Sensors = FakeSensors;
    type Motor = FakeMotor;
    fn parts(&mut self) -> (&mut FakeSensors, &mut FakeMotor) {
        (&mut self.sensors, &mut self.motor)
    }
}

fn kernel() -> Kernel<FakeIo> {
    Kernel::new(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        TIMING,
    )
}

fn last_cmd(k: &Kernel<FakeIo>) -> MotorCmd {
    k.io.motor.last.expect("a motor write happened")
}

/// Hand-stable rig baseline; tests override per case via `with_mut`.
fn seed(shared: &Shared) {
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
        c.loop_position.p_kp_q88 = 256;
        c.loop_position.pos_deadband_counts = 8;
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
        c.limits.oc_trip_counts = 2400;
        c.limits.oc_trip_ticks = 4;
        c.limits.openloop_decay = DecayMode::Slow;
        c.limits.openloop_zero_brake = false;
        c.limits.thermal_tau_ms = 4096;
        c.thermal.derate_start_cc = 8000;
        c.thermal.cutoff_cc = 10000;
        c.thermal.recover_cc = 9000;
        c.thermal.v_undervolt_counts = 2200;
        c.thermal.rtherm_i_min_counts = 300;
        // 1200 counts over 200 vcounts: 6 counts per vcount
        c.fault_cfg.rail_fold_q8 = 6 << 8;
        c.fusion.l1_q016 = 16384;
        c.fusion.l2_q88 = 1024;
        // l3 * B is the tau_d loop gain: at the b_i-encoded B (1.0 below)
        // 0.5 cc per count is stable, 2.0 rails the filter
        c.fusion.l3_q88 = 128;
        c.fault_cfg.pos_error_counts = 400;
        c.fault_cfg.pos_error_time_ticks = ms_ticks(500);
        c.fault_cfg.sensor_delta_max = 256;
        c.fault_cfg.sensor_bad_count = 4;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = 100;
        cal.sense.v_window_min_ticks = 100;
        cal.motor.ke_vpc_q = 256; // 0.0625 vcounts per c/s
        cal.motor.r_q12 = 8192; // 2.0 vcounts/ccount
        cal.motor.recip_ke_q = 16 << 10; // 16 c/s per vcount
        // plant B ~= 1.33 c/s per ccount per medium tick (200/1500 x 10);
        // encoded 1.0 (the old Q0.16-ceiling dynamics, kept so the pinned
        // integer sims stand); correction gains absorb the gap
        cal.motor.b_i_q313 = 8192;
        t.telemetry.refs.current_bias_counts = BIAS;
        t.control.lifecycle.torque_enable = false;
        t.control.lifecycle.mode = Mode::Position;
    });
}

/// A committed CONFIG or CALIB write, as the dispatcher lands it: the table
/// changes under an odd configuration count, even again after.
fn write_config(sh: &Shared, f: impl FnOnce(&mut ControlTable)) {
    sh.config_commit_begin();
    sh.table.with_mut(f);
    sh.config_commit_end();
}

/// Rail-tap counts that scale to exactly `vmotor` terminal counts.
fn vbus_raw(vmotor: u16) -> u16 {
    ((vmotor as u32 * 32768).div_ceil(VBUS_SCALE_Q15)) as u16
}

/// Slow-decay frames: the trough scan lands in the brake phase, so the shunt
/// reads its offset there.
fn frame(pos: u16, current: u16) -> SensorFrame {
    SensorFrame {
        pos,
        current,
        current_trough: BIAS,
        vmotor_a: 3000,
        vmotor_a_trough: 3000,
        vmotor_b: 40,
        vmotor_b_trough: 40,
        vcal: 1200,
        vbus_raw: vbus_raw(3000),
        ntc_raw: 2048,
    }
}

/// Ticks `f` until the applied duty equals `goal_duty`: OpenLoop duty above
/// the window floor slews there at `duty_limit::UP_Q15` per tick.
fn settle<T: TelStream>(k: &mut Kernel<FakeIo, T>, sh: &Shared, f: SensorFrame) {
    let goal = sh.table.with(|t| t.control.lifecycle.goal_duty);
    for _ in 0..1000 {
        if k.fast.duty_q15 == goal {
            return;
        }
        k.on_scan(f, sh);
    }
    panic!("duty {} never reached the goal {goal}", k.fast.duty_q15);
}

/// Ticks `f` until the next tick runs medium phase `p`.
fn run_to<T: TelStream>(k: &mut Kernel<FakeIo, T>, sh: &Shared, f: SensorFrame, p: u8) {
    while k.phase != p {
        k.on_scan(f, sh);
    }
}

/// Ticks `f` through the first period's VELOCITY phase, where the band and
/// the current reference first land: a servo enabled from boot drives from
/// the last of these ticks on.
fn to_first_command(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame) {
    for _ in 0..=phase::VELOCITY {
        k.on_scan(f, sh);
    }
}

// --- Gate / ack / dispatch ------------------------------------------------

#[test]
fn torque_off_disables_and_estimators_still_track() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..40 {
        k.on_scan(frame(2500, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // boot seeded the fusion at the first measurement
    assert_eq!(k.medium.fusion.theta_q16(), 2500 << 16);
    // the pot moves while disabled: the observer follows it anyway (the
    // live b_i bleed at the plant's saturated B settles the 1500-count
    // step in ~920 medium ticks, integer-sim verified)
    for _ in 0..12000 {
        k.on_scan(frame(1000, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    let err = (k.medium.fusion.theta_q16() - (1000 << 16)).abs();
    assert!(err < 5 << 16, "theta_hat={}", k.medium.fusion.theta_q16());
    // published while disabled
    sh.table.with(|t| {
        assert_eq!(
            t.telemetry.estimates.theta_hat_q16,
            k.medium.fusion.theta_q16()
        );
    });
}

#[test]
fn enable_edge_reseeds_without_transient() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..2000 {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
    });
    // at-goal enable: every command in the first stretch is at rest or a
    // bounded nudge - no integrator kick, no profile teleport
    for _ in 0..50 {
        k.on_scan(frame(2000, BIAS), &sh);
        match last_cmd(&k) {
            MotorCmd::Brake | MotorCmd::Disabled => {}
            MotorCmd::Drive { .. } => {
                let duty = written_duty(&k);
                assert!(duty.unsigned_abs() < 2000, "duty={duty}")
            }
            MotorCmd::Coast => panic!("coast is never commanded"),
        }
    }
    assert_eq!(k.medium.traj.theta_star_q16(), 2000 << 16);
}

#[test]
fn enable_edge_clears_hand_era_tau_d() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    // torque off, shaft hand-swept then snapped back: with the b_i bleed
    // live, steady-rate motion is model-explained and leaves tau_d near
    // zero - only the snap transient (e pinned at the clamp) pumps it
    for i in 0..4000u32 {
        let pos = 1000 + ((i / 4) % 2000) as u16;
        k.on_scan(frame(pos, BIAS), &sh);
    }
    for _ in 0..150 {
        k.on_scan(frame(1000, BIAS), &sh);
    }
    assert!(
        k.medium.fusion.tau_d_counts().unsigned_abs() > 200,
        "precondition: hand motion railed tau_d, got {}",
        k.medium.fusion.tau_d_counts()
    );
    // enable at rest: fusion reseeds, so the collision check never sees the
    // stale disturbance and STALL must not latch
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 0;
    });
    for _ in 0..200 {
        k.on_scan(frame(1500, BIAS), &sh);
    }
    assert_eq!(k.faults.mask(), 0, "stale tau_d re-latched STALL");
    assert!(k.medium.fusion.tau_d_counts().unsigned_abs() < 50);
}

#[test]
fn endstop_allows_retreat_from_the_wall() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 120;
    });
    let mut k = kernel();
    // parked hard against the top wall (bench: horn ratcheted to the rail)
    for _ in 0..400 {
        k.on_scan(frame(4095, BIAS), &sh);
    }
    // inward goal is banded to zero: the winding short holds at the wall
    assert!(
        matches!(last_cmd(&k), MotorCmd::Brake),
        "expected brake at the wall, got {:?}",
        last_cmd(&k)
    );
    // the door back out must be open: a retreat goal drives immediately
    // (the magnitude-fold deadlock read the zeroed i_ref as inward forever).
    // Reverse drive puts the rail on vmotor_b, so va - vb flips with the
    // command (the bemf sign convention).
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = -120);
    let mut rev = frame(4095, BIAS);
    (rev.vmotor_a, rev.vmotor_a_trough) = (40, 40);
    (rev.vmotor_b, rev.vmotor_b_trough) = (3000, 3000);
    let mut drove = false;
    for _ in 0..400 {
        k.on_scan(rev, &sh);
        if let Some(duty) = drive_duty(&k) {
            drove |= duty < 0;
        }
    }
    assert!(drove, "retreat from the wall never drove");
    assert_eq!(k.faults.mask(), 0);
}

// --- Stall permit lease ----------------------------------------------------

/// FAST ticks per second at the 20 kHz rig rate.
const SEC: u32 = 20_000;
/// Past the top soft wall, at rest: the endstop closes the band's top side
/// unless the permit lease is live.
const PAST_WALL: u16 = 3500;

fn permit_rig(sh: &Shared) -> Kernel<FakeIo> {
    seed(sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.torque_enable = true;
        t.config.pos_soft.pos_max_soft_counts = 3000;
    });
    let mut k = kernel();
    for _ in 0..400 {
        k.on_scan(frame(PAST_WALL, BIAS), sh);
    }
    // stopped just ahead of the CONTROL phase, which reads a grant for the
    // LIMITS phase after it
    run_to(&mut k, sh, frame(PAST_WALL, BIAS), phase::CONTROL);
    assert_eq!(
        k.medium.i_band.hi, 0,
        "the endstop is closed before any permit"
    );
    k
}

/// What the dispatcher does for a committed write of `stall_permit = 1`.
fn write_permit(sh: &Shared) {
    use crate::regions::control::addr::lifecycle::STALL_PERMIT;
    sh.table
        .with_mut(|t| t.control.lifecycle.stall_permit = true);
    sh.permit_after_commit(STALL_PERMIT, 1);
}

fn permit_live(k: &Kernel<FakeIo>) -> bool {
    k.medium.i_band.hi != 0
}

/// Ticks at the wall until the lease, once live, lapses; the ticks it took.
fn ticks_to_lapse(k: &mut Kernel<FakeIo>, sh: &Shared, max: u32) -> u32 {
    let mut live = false;
    (1..=max)
        .find(|_| {
            k.on_scan(frame(PAST_WALL, BIAS), sh);
            live |= permit_live(k);
            live && !permit_live(k)
        })
        .unwrap_or(max + 1)
}

#[test]
fn permit_lease_expires_without_a_host() {
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    write_permit(&sh);
    let lapse = ticks_to_lapse(&mut k, &sh, 2 * SEC);
    assert!(
        (SEC * 99 / 100..=SEC * 101 / 100).contains(&lapse),
        "lease lasted {lapse} ticks"
    );
    assert!(
        sh.table.with(|t| t.control.lifecycle.stall_permit),
        "the request byte is the host's: the kernel never clears it"
    );
    for _ in 0..SEC {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
        assert!(!permit_live(&k), "a lapsed lease stays lapsed");
    }
}

#[test]
fn permit_rewrite_extends_the_lease() {
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    write_permit(&sh);
    for _ in 0..SEC * 8 / 10 {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(permit_live(&k));
    write_permit(&sh);
    for _ in 0..SEC * 8 / 10 {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
        assert!(permit_live(&k), "the rewrite renewed the lease");
    }
    let lapse = ticks_to_lapse(&mut k, &sh, SEC);
    assert!(
        (SEC * 19 / 100..=SEC * 21 / 100).contains(&lapse),
        "a second from the rewrite, not the first write: {lapse} ticks more"
    );
    // a write of false revokes at the next MEDIUM pass
    write_permit(&sh);
    for _ in 0..DECIM_MED {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(permit_live(&k));
    sh.table
        .with_mut(|t| t.control.lifecycle.stall_permit = false);
    for _ in 0..DECIM_MED {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(!permit_live(&k), "false revokes inside a MEDIUM tick");
}

#[test]
fn torque_off_drops_the_permit() {
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    write_permit(&sh);
    for _ in 0..SEC / 10 {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(permit_live(&k));
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..2 * DECIM_MED {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    // re-enabled well inside the second the grant had left
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..SEC {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
        assert!(!permit_live(&k), "the lease outlived torque off");
    }
    assert!(sh.table.with(|t| t.control.lifecycle.stall_permit));
}

#[test]
fn permit_written_with_torque_off_never_grants() {
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..2 * DECIM_MED {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    write_permit(&sh);
    // torque back on before the next MEDIUM pass: the request alone, never
    // rewritten under torque, is no grant
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..2 * SEC {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
        assert!(!permit_live(&k), "a torque-off request granted a lease");
    }
}

fn limit_flags(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.limits.limit_flags)
}

#[test]
fn limit_flags_name_the_governor() {
    use limits::flag;

    // ceiling: OpenLoop over the limit at the window floor (R unset keeps
    // the base there, so every window reads the overage)
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_duty = 8000;
        t.config.limits.current_limit_counts = 280;
        t.calib.motor.r_q12 = 0;
    });
    let mut k = kernel();
    for _ in 0..2 * DECIM_MED {
        k.on_scan(frame(2000, BIAS + 290), &sh);
    }
    for _ in 0..4 {
        run_to(&mut k, &sh, frame(2000, BIAS + 290), phase::PUBLISH + 1);
        assert_eq!(limit_flags(&sh), flag::CEILING);
        k.on_scan(frame(2000, BIAS + 290), &sh);
    }
    assert!(k.fast.duty_q15 < 8000, "governed");

    // yield: Current mode at the limit, the held current over i_cont on a
    // still shaft, folds; then a goal under the fold leaves only the fold
    // standing (the held current stays over the release)
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_current = 1200;
        t.config.limits.stall_release_counts = 0;
        // the mg90-a observer set: `seed`'s gains ring for over a second
        // on a held current against a still pot, and the press needs the
        // observer still for the whole timer
        t.config.fusion.l1_q016 = 8640;
        t.config.fusion.l2_q88 = 3180;
        t.config.fusion.l3_q88 = 84;
        t.calib.motor.b_i_q313 = 2427;
    });
    let mut k = kernel();
    // the timer plus the observer's settle on the held current
    for _ in 0..8_000 {
        k.on_scan(frame(2000, BIAS + 400), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::CEILING | flag::YIELD);
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = 100);
    for _ in 0..2 * DECIM_MED {
        k.on_scan(frame(2000, BIAS + 400), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::YIELD);
    assert_eq!(k.faults.mask(), 0);

    // the terms: the goal caps a Velocity drive and not an OpenLoop one,
    // the I2t budget a long hold over i_cont, the rail a sag into the
    // band; each names itself
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_velocity = 1000;
        t.control.lifecycle.goal_current = -400;
        t.config.limits.stall_time_ticks = ms_ticks(60000);
    });
    let mut k = kernel();
    for _ in 0..2 * DECIM_MED {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    assert_eq!(limit_flags(&sh) & !flag::CEILING, flag::GOAL);
    assert_eq!(sh.table.with(|t| t.telemetry.estimates.i_lim_counts), 400);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    settle(&mut k, &sh, frame(2000, BIAS));
    for _ in 0..2 * DECIM_MED {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    assert_eq!(
        limit_flags(&sh),
        0,
        "a left-behind goal caps no OpenLoop drive"
    );
    assert_eq!(sh.table.with(|t| t.telemetry.estimates.i_lim_counts), 1200);
    // 400 counts held for 156 SLOW periods (2.5 s) against a 300-count
    // rating on a 4 s constant: the heat sits in the fold's ramp
    for _ in 0..DECIM_MED as u32 * DECIM_SLOW as u32 * 156 {
        k.on_scan(frame(2000, BIAS + 400), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::I2T);
    let folded = sh.table.with(|t| t.telemetry.estimates.i_lim_counts);
    assert!((300..1200).contains(&folded), "folded={folded}");
    // the rail halfway into the band: the fold under the I2t ceiling
    let mut sag = frame(2000, BIAS + 400);
    sag.vbus_raw = vbus_raw(2300);
    for _ in 0..200 * DECIM_MED as u32 {
        k.on_scan(sag, &sh);
    }
    assert_eq!(limit_flags(&sh), flag::RAIL);
    let (sagged, vbus) = sh.table.with(|t| {
        let e = &t.telemetry.estimates;
        (e.i_lim_counts as i32, e.vbus_counts as i32)
    });
    // 1200 counts over the 200-vcount band: 6 counts per vcount over the floor
    let want = (vbus - 2200) * 6;
    assert!(
        (want - 1..=want).contains(&sagged),
        "sagged={sagged} vbus={vbus}"
    );

    // endstop: at rest past the top soft wall
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    assert_eq!(limit_flags(&sh), flag::ENDSTOP);

    // permit: the lease drops the endstop, so it stands alone
    write_permit(&sh);
    for _ in 0..DECIM_MED {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::PERMIT);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..DECIM_MED {
        k.on_scan(frame(PAST_WALL, BIAS), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::ENDSTOP, "torque off ends the lease");
}

#[test]
fn undervolt_follows_the_rail_tap_through_bridge_off() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8192;
    });
    let mut k = kernel();
    // sagged rail on the direct tap; the terminal taps are irrelevant
    let mut sagged = frame(2000, BIAS);
    sagged.vbus_raw = vbus_raw(1000);
    for _ in 0..400 {
        k.on_scan(sagged, &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_UNDER_VOLT, "sag latches");
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // the tap keeps sampling with the bridge off: an ack against a still
    // sagged rail re-latches, an ack against a recovered rail sticks
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..400 {
        k.on_scan(sagged, &sh);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..2000 {
        k.on_scan(sagged, &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_UNDER_VOLT, "sag re-latches");
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..400 {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..2000 {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    assert_eq!(k.faults.mask(), 0, "recovered rail re-latched undervolt");
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // EWMA truncation tail: settles within 1 count of the rail
    let v = sh.table.with(|t| t.telemetry.estimates.vbus_counts) as i32;
    assert!((v - 3000).abs() <= 1, "vbus_counts={v}");
}

#[test]
fn oc_latch_forces_disabled_despite_torque_enable() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 16000;
        // over the limit on purpose: the OpenLoop ceiling stays out of it
        t.config.limits.current_limit_counts = u16::MAX;
    });
    let mut k = kernel();
    // the first command drives (windows still invalid: previous duty was 0)
    to_first_command(&mut k, &sh, frame(2000, BIAS + 3000));
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // 4 consecutive valid over-trip samples latch on the 4th
    for _ in 0..3 {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
        assert_eq!(k.faults.mask(), 0);
    }
    k.on_scan(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // torque_enable still set: stays disabled, publish lands at medium
    for _ in 0..10 {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
        assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    }
    sh.table.with(|t| {
        assert_eq!(t.telemetry.common.fault_flags, faults::BIT_OVER_CURRENT);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_OVER_CURRENT);
        assert_eq!(t.telemetry.mode.mode_active, Mode::OpenLoop as u8);
    });
}

#[test]
fn ack_clears_then_relatches_while_condition_persists() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 16000;
        // over the limit on purpose: the OpenLoop ceiling stays out of it
        t.config.limits.current_limit_counts = u16::MAX;
    });
    let mut k = kernel();
    to_first_command(&mut k, &sh, frame(2000, BIAS + 3000));
    for _ in 0..8 {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    // ack: drop then raise torque_enable, each seen at a CONTROL phase; the
    // ack lands at the CONTROL phase after the raise, not before
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..DECIM_MED {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    while k.phase != 0 {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
        assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    }
    k.on_scan(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), 0);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // the overcurrent persists: a fresh window re-latches
    for _ in 0..5 {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
}

#[test]
fn oc_gap_rearms_the_window() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 16000;
        // over the limit on purpose: the OpenLoop ceiling stays out of it
        t.config.limits.current_limit_counts = u16::MAX;
    });
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS + 3000), &sh); // warmup: invalid window
    for _ in 0..3 {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
    }
    // one clean sample re-arms; three more over-samples do not latch
    k.on_scan(frame(2000, BIAS + 100), &sh);
    for _ in 0..3 {
        k.on_scan(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), 0);
    // the 4th consecutive does
    k.on_scan(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
}

#[test]
fn openloop_duty_passthrough_clamped_with_decay() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        t.config.limits.openloop_decay = DecayMode::Fast;
    });
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS));
    match last_cmd(&k) {
        MotorCmd::Drive { decay, .. } => {
            assert_eq!(written_duty(&k), 8000);
            assert!(matches!(decay, DecayMode::Fast));
        }
        other => panic!("expected Drive, got {other:?}"),
    }
    // duty_max clamps the passthrough at the next medium boundary, at once
    write_config(&sh, |t| t.config.loop_current.duty_max_q15 = 5000);
    while k.phase != 0 {
        k.on_scan(frame(2000, BIAS), &sh);
        assert_eq!(written_duty(&k), 8000);
    }
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(written_duty(&k), 5000);
}

#[test]
fn duty_max_write_caps_the_closed_loop_at_the_rail_phase() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 1200;
    });
    let mut k = kernel();
    // no current ever reads back: the loop saturates at duty_max x vbus
    for _ in 0..10 * DECIM_MED {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    assert!(written_duty(&k) > 32000, "duty={}", written_duty(&k));
    // the take leaves the clamp alone; the next RAIL phase prepares it
    write_config(&sh, |t| t.config.loop_current.duty_max_q15 = 8000);
    run_to(&mut k, &sh, frame(2000, BIAS), phase::SPARE_A);
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.cfg.loop_current.duty_max_q15, 8000, "the take ran");
    assert!(written_duty(&k) > 32000, "the take tick");
    while k.phase != phase::RAIL {
        k.on_scan(frame(2000, BIAS), &sh);
        assert!(written_duty(&k) > 32000);
    }
    k.on_scan(frame(2000, BIAS), &sh);
    let d = written_duty(&k);
    assert!((7900..=8000).contains(&d), "duty={d}");
}

#[test]
fn openloop_zero_duty_drives_zero_when_brake_flag_clear() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 0;
    });
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { decay, .. } => {
            assert_eq!(written_duty(&k), 0);
            assert!(matches!(decay, DecayMode::Slow));
        }
        other => panic!("expected Drive, got {other:?}"),
    }
}

#[test]
fn openloop_zero_goal_drops_the_held_current() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS + 200));
    k.on_scan(frame(2000, BIAS + 200), &sh);
    assert_eq!(k.fast.i_meas_last, 200);
    // the coast drives nothing: the held current reads 0 from the tick
    // the zero goal lands, not the last driven sample
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 0);
    run_to(&mut k, &sh, frame(2000, BIAS + 200), 0);
    k.on_scan(frame(2000, BIAS + 200), &sh);
    assert_eq!(drive_duty(&k), Some(0));
    assert_eq!(k.fast.i_meas_last, 0);
}

#[test]
fn openloop_zero_duty_brakes_when_flag_set() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 0;
        t.config.limits.openloop_zero_brake = true;
    });
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS), &sh);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(k.fast.duty_q15, 0);
}

#[test]
fn openloop_endstop_brakes_instead_of_coasting() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    let mut k = kernel();
    for _ in 0..400 {
        k.on_scan(frame(4095, BIAS), &sh);
    }
    assert!(
        matches!(last_cmd(&k), MotorCmd::Brake),
        "expected brake at the wall, got {:?}",
        last_cmd(&k)
    );
    assert_eq!(k.fast.duty_q15, 0);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
    k.on_scan(frame(4095, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { .. } => assert!(written_duty(&k) < 0, "duty {}", written_duty(&k)),
        other => panic!("expected retreat Drive, got {other:?}"),
    }
    settle(&mut k, &sh, frame(4095, BIAS));
    assert_eq!(written_duty(&k), -8000);
}

#[test]
fn openloop_nonzero_duty_drives_despite_brake_flag() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        t.config.limits.openloop_zero_brake = true;
    });
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS));
    match last_cmd(&k) {
        MotorCmd::Drive { .. } => assert_eq!(written_duty(&k), 8000),
        other => panic!("expected Drive, got {other:?}"),
    }
}

/// The width the motor write carries is the one the next tick's window
/// select reads; a command that drives nothing leaves none.
#[test]
fn the_next_measure_reads_the_written_width() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS));
    let MotorCmd::Drive { ticks, .. } = last_cmd(&k) else {
        panic!("expected Drive, got {:?}", last_cmd(&k));
    };
    assert_eq!(ticks as u32, window::drive_ticks(8000, ARR));
    assert_eq!(k.fast.ticks, ticks);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    run_to(&mut k, &sh, frame(2000, BIAS), 0);
    k.on_scan(frame(2000, BIAS), &sh);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    assert_eq!(k.fast.ticks, 0);
}

/// The wiring-frame duty the last Drive programs: the logical duty under
/// the command's sign, the command's width checked against the formula.
fn drive_duty(k: &Kernel<FakeIo>) -> Option<i16> {
    match last_cmd(k) {
        MotorCmd::Drive { ticks, reverse, .. } => {
            let duty = k.fast.duty_q15;
            assert_eq!(ticks as u32, window::drive_ticks(duty, ARR), "duty {duty}");
            Some(if (duty < 0) == reverse {
                duty
            } else {
                duty.saturating_neg()
            })
        }
        _ => None,
    }
}

fn written_duty(k: &Kernel<FakeIo>) -> i16 {
    drive_duty(k).unwrap_or_else(|| panic!("expected Drive, got {:?}", last_cmd(k)))
}

fn reversed(sh: &Shared, mode: Mode) {
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = mode;
        t.config.limits.drive_polarity = false;
    });
}

#[test]
fn reversed_polarity_endstop_stays_positional() {
    let sh = Shared::new();
    seed(&sh);
    reversed(&sh, Mode::OpenLoop);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    let mut k = kernel();
    // +duty is still the outbound push at the top wall
    for _ in 0..400 {
        k.on_scan(frame(4095, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(k.fast.duty_q15, 0);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
    for _ in 0..400 {
        k.on_scan(frame(4095, BIAS), &sh);
    }
    assert_eq!(k.fast.duty_q15, -8000);
    assert_eq!(written_duty(&k), 8000);
    assert_eq!(k.faults.mask(), 0);
}

#[test]
fn reversed_polarity_negates_the_openloop_write() {
    let sh = Shared::new();
    seed(&sh);
    reversed(&sh, Mode::OpenLoop);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS));
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(written_duty(&k), -8000);
    assert_eq!(k.fast.duty_q15, 8000);
    // the tick block's duty is logical, the previous tick's command
    assert_eq!(tick_block(&sh).duty_ticks, window::duty_counts(8000, ARR));
}

#[test]
fn reversed_polarity_negates_the_closed_loop_write() {
    let sh = Shared::new();
    seed(&sh);
    reversed(&sh, Mode::Current);
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = 500);
    let mut k = kernel();
    for _ in 0..20 {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    assert!(k.fast.duty_q15 > 0, "logical duty={}", k.fast.duty_q15);
    assert_eq!(written_duty(&k), -k.fast.duty_q15);
}

#[test]
fn reversed_polarity_negates_vdiff() {
    let sh = Shared::new();
    seed(&sh);
    reversed(&sh, Mode::OpenLoop);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    let mut k = kernel();
    // the tick after the first command samples the window it drove
    to_first_command(&mut k, &sh, frame(2000, BIAS));
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.fast.vdiff_last, -(3000 - 40));
    // the rewire lands with the fast half of the configuration
    write_config(&sh, |t| t.config.limits.drive_polarity = true);
    while k.phase != phase::SPARE_A {
        k.on_scan(frame(2000, BIAS), &sh);
        assert_eq!(k.fast.vdiff_last, -(3000 - 40));
    }
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.fast.vdiff_last, 3000 - 40);
}

#[test]
fn current_mode_clamps_goal_to_i_lim() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 5000;
    });
    let mut k = kernel();
    to_first_command(&mut k, &sh, frame(2000, BIAS));
    assert_eq!(k.medium.i_ref_cc, 1200);
    assert!(matches!(
        last_cmd(&k),
        MotorCmd::Drive {
            decay: DecayMode::Slow,
            ..
        }
    ));
    // the goal is read at the next CONTROL phase and clamped at the VELOCITY
    // phase after it
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = -5000);
    run_to(&mut k, &sh, frame(2000, BIAS), phase::CONTROL);
    run_to(&mut k, &sh, frame(2000, BIAS), phase::VELOCITY);
    assert_eq!(k.medium.i_ref_cc, 1200);
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.medium.i_ref_cc, -1200);
}

/// The thermometer's bench MG90 model on the test rig: tau 128 s, a small
/// `g` so the carry's steady excess at the rig's 722 x 400 vcount-ccount
/// hold is 4.4 C, mu for a 2 s settle at 400 counts, the dev board's
/// 10K/10K/3950 NTC reading 25.00 C at the rig's 2048 counts.
fn seed_thermometer(sh: &Shared, r_cold_q12: u16) {
    write_config(sh, |t| {
        t.calib.thermal.th_alpha_q24 = 2097;
        t.calib.thermal.th_g_q016 = 100;
        t.calib.thermal.th_mu_q016 = 5369;
        t.calib.winding.r0_q12 = r_cold_q12;
        t.calib.thermal.ntc_raw_ref = 2048;
        t.calib.thermal.ntc_t_ref_cc = 2500;
        t.calib.thermal.ntc_k1_q88 = 563;
        t.calib.thermal.ntc_k2_q24 = 5800;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
}

/// The rig hold's own R, 722 x 4096 / 400, as the stored cold R at 25 C.
const RIG_R_COLD_Q12: u16 = 7393;

fn therm(sh: &Shared) -> (i16, u8, u16, i16) {
    sh.table.with(|t| {
        (
            t.telemetry.estimates.t_winding_cc,
            t.telemetry_ext.therm.therm_flags,
            t.telemetry.estimates.i_lim_counts,
            t.telemetry_ext.therm.t_ntc_cc,
        )
    })
}

/// SLOW ticks in fast ticks; a reference needs 192 SLOW ticks at a seat.
const SLOW: u32 = DECIM_MED as u32 * DECIM_SLOW as u32;
const BASE_FAST_TICKS: u32 = 200 * SLOW;

/// Without model constants the thermometer reads its sentinel and nothing
/// derates, however hot the hold would read through the retired anchor.
#[test]
fn thermometer_without_a_model_reads_the_sentinel_and_never_derates() {
    let sh = Shared::new();
    seed(&sh);
    write_config(&sh, |t| {
        t.calib.thermal.ntc_raw_ref = 2048;
        t.calib.thermal.ntc_t_ref_cc = 2500;
        t.calib.thermal.ntc_k1_q88 = 563;
        t.calib.thermal.ntc_k2_q24 = 5800;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        // the hold sits over the continuous rating: the I2t budget and the
        // press are not under test
        t.config.limits.i_cont_counts = 0;
    });
    let mut k = kernel();
    let hot = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, BASE_FAST_TICKS);
    let (t, flags, i_lim, _) = therm(&sh);
    assert_eq!(t, i16::MIN);
    assert_eq!(flags & crate::estimator::thermal::flag::UNSET, 1);
    assert_eq!(i_lim, 1200);
    assert_eq!(k.faults.mask(), 0);
}

/// A seat change with +1% of contact (the retired anchor read it as +2.6 C)
/// re-bases on the carried temperature: the step is under 0.5 C.
#[test]
fn seat_change_rebases_the_ratio_without_a_temperature_step() {
    let sh = Shared::new();
    seed(&sh);
    seed_thermometer(&sh, RIG_R_COLD_Q12);
    let mut k = kernel();
    let seat_a = frame(2000, BIAS + 400);
    settle(&mut k, &sh, seat_a);
    ticks(&mut k, &sh, seat_a, BASE_FAST_TICKS);
    let (t_a, flags, _, t_ntc) = therm(&sh);
    assert_eq!(flags & crate::estimator::thermal::flag::TRACK, 2);
    assert_eq!(t_ntc, 2500);
    // the base inherits the boot prior: half the steady excess at the
    // rig's 1200-count limit, 19.8 C
    assert!((4400..4500).contains(&t_a), "{t_a}");
    // 100 counts away, 396 counts at the same duty: R +1.0%
    let seat_b = frame(2100, BIAS + 396);
    ticks(&mut k, &sh, seat_b, BASE_FAST_TICKS);
    let (t_b, flags, _, _) = therm(&sh);
    assert_eq!(flags & crate::estimator::thermal::flag::TRACK, 2);
    assert!((t_b - t_a).abs() < 50, "{t_b} from {t_a}");
}

/// A derate step at the seat (the current drops an eighth or more while
/// the duty holds) would read +17% of R as heat through the ratio; the
/// base is dropped and the carry holds the temperature within 0.5 C.
#[test]
fn a_derate_step_at_a_seat_reads_under_half_a_degree() {
    let sh = Shared::new();
    seed(&sh);
    seed_thermometer(&sh, RIG_R_COLD_Q12);
    let mut k = kernel();
    let hold = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hold);
    ticks(&mut k, &sh, hold, BASE_FAST_TICKS);
    let (t0, _, _, _) = therm(&sh);
    ticks(&mut k, &sh, frame(2000, BIAS + 340), 20 * SLOW);
    let (t1, flags, _, _) = therm(&sh);
    assert_eq!(flags & crate::estimator::thermal::flag::TRACK, 0);
    assert!((t1 - t0).abs() < 50, "{t1} from {t0}");
}

/// A +1.5% contact step inside one hold (+3.9 C by the ratio) rises faster
/// than heating can at a limit near the hold (500 counts: 0.3 C/s): the
/// base is dropped, the carry keeps the temperature from before the step,
/// and neither the re-base nor the next seat reads it.
#[test]
fn a_contact_step_inside_a_hold_is_rejected_on_its_rise_rate() {
    let sh = Shared::new();
    seed(&sh);
    seed_thermometer(&sh, RIG_R_COLD_Q12);
    write_config(&sh, |t| t.config.limits.current_limit_counts = 500);
    let mut k = kernel();
    let hold = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hold);
    ticks(&mut k, &sh, hold, BASE_FAST_TICKS);
    let (t0, _, _, _) = therm(&sh);
    ticks(&mut k, &sh, frame(2000, BIAS + 394), 400 * SLOW);
    let (t1, flags, _, _) = therm(&sh);
    assert_eq!(flags & crate::estimator::thermal::flag::TRACK, 2);
    assert!((t1 - t0).abs() < 50, "{t1} from {t0}");
    ticks(&mut k, &sh, frame(2100, BIAS + 394), BASE_FAST_TICKS);
    let (t2, _, _, _) = therm(&sh);
    assert!((t2 - t1).abs() < 50, "{t2} from {t1}");
}

/// Torque off: the carry decays the excess toward the NTC on tau, so a
/// derate taken on the boot prior (the bench proxy band, 35 C) lifts at
/// rest instead of latching.
#[test]
fn torque_off_carries_the_estimate_to_the_ntc_and_the_derate_lifts() {
    let sh = Shared::new();
    seed(&sh);
    seed_thermometer(&sh, RIG_R_COLD_Q12);
    write_config(&sh, |t| {
        t.config.thermal.derate_start_cc = 3500;
        t.config.thermal.cutoff_cc = 6500;
        t.config.thermal.recover_cc = 4000;
    });
    let mut k = kernel();
    let hold = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hold);
    ticks(&mut k, &sh, hold, BASE_FAST_TICKS);
    let (t_hot, _, i_lim, _) = therm(&sh);
    assert!(t_hot > 4400 && i_lim < 1200, "{t_hot} {i_lim}");
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    // five tau: 40000 SLOW ticks
    ticks(&mut k, &sh, frame(2000, BIAS), 40_000 * SLOW);
    let (t, flags, i_lim, t_ntc) = therm(&sh);
    assert!((t - t_ntc).abs() < 40, "{t} vs {t_ntc}");
    assert_eq!(flags & crate::estimator::thermal::flag::TRACK, 0);
    assert_eq!(i_lim, 1200);
}

#[test]
fn position_error_latches_after_persistence() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3500;
        t.config.fault_cfg.pos_error_counts = 100;
        t.config.fault_cfg.pos_error_time_ticks = ms_ticks(10); // 20 medium ticks
        // keep the stall path out of this test
        t.config.limits.stall_time_ticks = ms_ticks(60000);
    });
    let mut k = kernel();
    // pot frozen at 500 while the profile marches away
    for _ in 0..4000 {
        k.on_scan(frame(500, BIAS), &sh);
        if k.faults.mask() != 0 {
            break;
        }
    }
    assert_eq!(k.faults.mask(), faults::BIT_POSITION_ERROR);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
}

#[test]
fn publishes_land_in_the_table() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..20 {
        k.on_scan(frame(1234, 2100), &sh);
    }
    sh.table.with(|t| {
        assert_eq!(t.telemetry.tick.pos, 1234);
        assert_eq!(t.telemetry.tick.current_raw, 2100);
        assert_eq!(t.telemetry.tick.vmotor_a, 3000);
        assert_eq!(t.telemetry.tick.duty_ticks, 0);
        assert_eq!(
            t.telemetry.estimates.theta_hat_q16,
            k.medium.fusion.theta_q16()
        );
        assert_eq!(
            t.telemetry.estimates.omega_hat_cps,
            k.medium.fusion.omega_q16()
        );
        assert_eq!(t.telemetry.estimates.i_lim_counts, 1200);
        assert_eq!(t.telemetry.estimates.vbus_counts, 3000);
        assert_eq!(t.telemetry.mode.mode_active, Mode::Position as u8);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_NONE);
        assert_eq!(t.telemetry.common.fault_flags, 0);
    });
}

fn published_bias(sh: &Shared) -> u16 {
    sh.table.with(|t| t.telemetry.refs.current_bias_counts)
}

/// Ticks for the bias tracker to settle a step to the count: 12 time
/// constants.
const BIAS_SETTLE_TICKS: u32 = 12 << bias::ALPHA_SHIFT;

fn ticks(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame, n: u32) {
    for _ in 0..n {
        k.on_scan(f, sh);
    }
}

/// A frame with no drive: both scans read `trough`.
fn rest(trough: u16) -> SensorFrame {
    let mut f = frame(2000, trough);
    f.current_trough = trough;
    f
}

#[test]
fn trough_bias_tracks_only_inside_slow_drive_windows() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    let shifted = || {
        let mut f = frame(2000, BIAS);
        f.current_trough = BIAS + 500;
        f
    };
    // Fast decay: the trough IS the drive window. The decay lands at a
    // medium boundary, so it goes in ahead of the drive.
    write_config(&sh, |t| t.config.limits.openloop_decay = DecayMode::Fast);
    ticks(&mut k, &sh, frame(2000, BIAS), DECIM_MED as u32);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    ticks(&mut k, &sh, shifted(), 3000);
    assert!(matches!(
        last_cmd(&k),
        MotorCmd::Drive {
            decay: DecayMode::Fast,
            ..
        }
    ));
    assert_eq!(published_bias(&sh), BIAS);
    // full-scale Slow: idle leg never leaves CCR 0, no off phase. Reached
    // under Fast: a Slow slew up to it passes through brake troughs.
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_duty = i16::MAX);
    settle(&mut k, &sh, shifted());
    assert_eq!(published_bias(&sh), BIAS);
    write_config(&sh, |t| t.config.limits.openloop_decay = DecayMode::Slow);
    ticks(&mut k, &sh, shifted(), 3000);
    assert_eq!(drive_duty(&k), Some(i16::MAX));
    assert_eq!(published_bias(&sh), BIAS);
    // a partial Slow window: the trough is the brake phase, the tracker follows
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    ticks(&mut k, &sh, shifted(), BIAS_SETTLE_TICKS);
    assert!(matches!(
        last_cmd(&k),
        MotorCmd::Drive {
            decay: DecayMode::Slow,
            ..
        }
    ));
    assert_eq!(published_bias(&sh), BIAS + 500);
}

#[test]
fn drifting_trough_bias_leaves_i_meas_flat() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    let mut k = kernel();
    // offset walks 40 counts at 1 count per 4096 ticks, 2 counts of lag
    // at the 8192-tick time constant (the bench drift, 8 counts per
    // degree C, would need a degree every 1.6 s); the true current is a
    // constant 300 counts above it
    let mut b = BIAS;
    for n in 0..40u32 << 12 {
        b = BIAS + (n >> 12) as u16;
        let mut f = frame(2000, b + 300);
        f.current_trough = b;
        k.on_scan(f, &sh);
        if n >= 300 {
            let i = k.fast.i_meas_last as i32;
            assert!((i - 300).abs() <= 3, "tick {n}: i_meas {i}, bias {b}");
        }
    }
    assert!(
        published_bias(&sh).abs_diff(b) <= 3,
        "{}",
        published_bias(&sh)
    );
    // the untracked boot value would have read 339 here
    assert_eq!(b, BIAS + 39);
}

#[test]
fn bias_feed_ignores_the_current_floor() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.calib.sense.i_window_min_ticks = 40;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 20000;
    });
    let mut k = kernel();
    let shifted = || {
        let mut f = frame(2000, BIAS + 300);
        f.current_trough = BIAS + 500;
        f
    };
    // the slew up passes wide brake halves; the trough holds the bias there
    settle(&mut k, &sh, frame(2000, BIAS + 300));
    // 732 drive ticks leave a 468-tick brake half: under the brake
    // minimum, however low the current floor sits
    ticks(&mut k, &sh, shifted(), 3000);
    assert_eq!(published_bias(&sh), BIAS);
    // 293 drive ticks leave 907: the tracker follows
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    ticks(&mut k, &sh, shifted(), BIAS_SETTLE_TICKS);
    assert_eq!(published_bias(&sh), BIAS + 500);
}

#[test]
fn torque_off_bias_follows_a_drifting_rest() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    // the board warms with torque off: the rest walks 200 counts down at
    // 1 count per 1024 ticks, 8 counts of lag
    for n in 0..200u32 << 10 {
        let b = BIAS - (n >> 10) as u16;
        k.on_scan(rest(b), &sh);
        if n >= 1 << 10 {
            let p = published_bias(&sh);
            assert!(p >= b && p - b <= 9, "tick {n}: bias {p}, rest {b}");
        }
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // never awake: no step learned, the bias is the rest itself
    ticks(&mut k, &sh, rest(BIAS - 200), BIAS_SETTLE_TICKS);
    assert_eq!(published_bias(&sh), BIAS - 200);
}

#[test]
fn the_awake_step_is_learned_and_rides_the_torque_off_drift() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 0;
        t.config.limits.openloop_zero_brake = true;
    });
    let mut k = kernel();
    // torque off at the boot rest, then the brake: the driver wakes and
    // its own supply current lifts the zero 10 counts
    ticks(&mut k, &sh, rest(BIAS), 1000);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    ticks(&mut k, &sh, rest(BIAS + 10), BIAS_SETTLE_TICKS);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(published_bias(&sh), BIAS + 10);
    // torque off past the stale window, then the board warms: the bias
    // keeps the step over the rest
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    ticks(&mut k, &sh, rest(BIAS), bias::STALE_TICKS as u32);
    assert_eq!(published_bias(&sh), BIAS + 10);
    ticks(&mut k, &sh, rest(BIAS - 200), BIAS_SETTLE_TICKS);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    assert_eq!(published_bias(&sh), BIAS - 190);
    // awake again, coasting on a zero goal: the zero it reads is the one
    // already in use, so nothing moves
    write_config(&sh, |t| t.config.limits.openloop_zero_brake = false);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..BIAS_SETTLE_TICKS {
        k.on_scan(rest(BIAS - 190), &sh);
        assert_eq!(published_bias(&sh), BIAS - 190);
    }
    assert_eq!(drive_duty(&k), Some(0));
}

#[test]
fn a_draining_current_is_never_learned_as_zero() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = i16::MAX;
        // the drive sits over the continuous rating: the press is not
        // under test
        t.config.limits.i_cont_counts = 0;
    });
    let mut k = kernel();
    // the slew up passes brake troughs at the zero; at full duty there is
    // no brake half, and the trough scan reads the drive at the top of the
    // range. Held until the awake zero goes stale, so it would carry a
    // leak into the asleep zero to the bias.
    let slew = frame(2000, BIAS + 500);
    let mut driven = slew;
    driven.current_trough = 4095;
    settle(&mut k, &sh, slew);
    ticks(&mut k, &sh, driven, bias::STALE_TICKS as u32);
    assert_eq!(published_bias(&sh), BIAS);
    // torque off, then the brake: the shunt still reads the drive through
    // the settle and none of it reaches the tracker; past it, the rest does
    for (zero_brake, cmd, rest_counts) in [
        (false, MotorCmd::Disabled, BIAS - 20),
        (true, MotorCmd::Brake, BIAS + 20),
    ] {
        write_config(&sh, |t| {
            t.control.lifecycle.torque_enable = zero_brake;
            t.control.lifecycle.goal_duty = 0;
            t.config.limits.openloop_zero_brake = zero_brake;
        });
        while core::mem::discriminant(&last_cmd(&k)) != core::mem::discriminant(&cmd) {
            k.on_scan(driven, &sh);
        }
        ticks(&mut k, &sh, driven, fast::ZERO_SETTLE_TICKS as u32);
        assert_eq!(published_bias(&sh), BIAS);
        ticks(&mut k, &sh, rest(BIAS), BIAS_SETTLE_TICKS);
        assert_eq!(published_bias(&sh), BIAS);
        ticks(&mut k, &sh, rest(rest_counts), BIAS_SETTLE_TICKS);
        assert_eq!(published_bias(&sh), rest_counts);
        ticks(&mut k, &sh, rest(BIAS), BIAS_SETTLE_TICKS);
        sh.table.with_mut(|t| {
            t.control.lifecycle.torque_enable = true;
            t.control.lifecycle.goal_duty = i16::MAX;
        });
        settle(&mut k, &sh, slew);
    }
}

#[test]
fn settle_gain_lifts_a_short_window_and_leaves_the_raw_sample() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.calib.sense.i_window_min_ticks = 64;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        // 100 drive ticks
        t.control.lifecycle.goal_duty = 2731;
    });
    let timing = KernelTiming {
        i_settle_gain: window::SettleGain {
            start_ticks: 40,
            q15: &[35143, 34194, 33734, 33575, 33456, 33376, 33314, 33259],
        },
        ..TIMING
    };
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        (),
        timing,
    );
    let f = frame(2000, BIAS + 300);
    settle(&mut k, &sh, f);
    k.on_scan(f, &sh);
    let t = tick_block(&sh);
    assert_ne!(t.tick_flags & tick_flag::I_VALID, 0);
    // the 96..103 band: 300 x 33259 / 32768 = 304.49
    assert_eq!(t.i_tick_counts, 304);
    assert_eq!(t.current_raw, BIAS + 300);
    // past the table the gain is unity
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    settle(&mut k, &sh, f);
    k.on_scan(f, &sh);
    assert_eq!(tick_block(&sh).i_tick_counts, 300);
}

// --- Held stream samples ---------------------------------------------------

/// OpenLoop at 8000, a duty whose windows the 100-tick floors read.
fn drive_setup(sh: &Shared) {
    seed(sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        // drive_ticks(8000) = 293 >= the 100-tick floors; the slew there
        // starts at the floor, so windows are valid from tick 2 on (tick 1
        // measures the boot duty of 0)
        t.control.lifecycle.goal_duty = 8000;
    });
}

/// A TEL kernel settled at 8000 and +200 counts, its next tick the CONTROL
/// phase where a goal write lands, the recorder empty.
fn tel_settled_at_control(sh: &Shared) -> Kernel<FakeIo, RecTel> {
    drive_setup(sh);
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        RecTel::new(sh),
        TIMING,
    );
    settle(&mut k, sh, frame(2000, BIAS + 200));
    while k.phase != phase::CONTROL {
        k.on_scan(frame(2000, BIAS + 200), sh);
    }
    k.tel.rows.clear();
    k
}

/// The (held current, differential, duty) of each row the recorder took.
fn rows(k: &Kernel<FakeIo, RecTel>) -> heapless::Vec<(i16, i16, i16), 64> {
    k.tel
        .rows
        .iter()
        .map(|t| (t.i_tick_counts, t.vdiff_counts, t.duty_ticks))
        .collect()
}

#[test]
fn stream_rows_hold_the_last_valid_window_while_the_drive_pushes() {
    let sh = Shared::new();
    let mut k = tel_settled_at_control(&sh);
    // drive_ticks(1000) = 37, under the 100-tick floors: windows go
    // invalid one tick later (the first tick still measures the period the
    // last valid command drove) while the drive still pushes
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 1000);
    for _ in 0..16 {
        k.on_scan(frame(2000, BIAS + 300), &sh);
    }
    let r = rows(&k);
    assert_eq!(r[0], (300, 2960, 293));
    assert!(r[1..].iter().all(|&row| row == (300, 2960, 37)), "{r:?}");
    assert!(
        k.tel.rows[1..]
            .iter()
            .all(|t| t.tick_flags & tick_flag::I_VALID == 0)
    );
}

#[test]
fn stream_current_reads_zero_once_nothing_drives() {
    let sh = Shared::new();
    let mut k = tel_settled_at_control(&sh);
    // disable: the first tick measures the period the last drive command
    // drove, the 15 after it read no current; vdiff keeps its hold
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..16 {
        k.on_scan(frame(2000, BIAS + 200), &sh);
    }
    let r = rows(&k);
    assert_eq!(r[0], (200, 2960, 293));
    assert!(r[1..].iter().all(|&row| row == (0, 2960, 0)), "{r:?}");
    assert_eq!(k.fast.i_meas_last, 0);
}

// --- Back-EMF boxcar ------------------------------------------------------

#[test]
fn bemf_boxcar_lands_on_the_closed_form_after_20_ticks() {
    let sh = Shared::new();
    drive_setup(&sh);
    let mut k = kernel();
    // from the OBSERVER phase after the slew every tick measures duty
    // 8000: drive_ticks 293, vdiff 2960, i 100. The half closed 10 ticks
    // later is the first clean one, 20 ticks pairs it with a second, and the
    // PUBLISH phase after it lands the boxcar.
    settle(&mut k, &sh, frame(2000, BIAS + 100));
    run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::OBSERVER + 1);
    for _ in 0..20 {
        k.on_scan(frame(2000, BIAS + 100), &sh);
    }
    run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::PUBLISH + 1);
    // closed form: (293 * 2960 / 1200 - 2.0 * 100) * 16 c/s per vcount
    let ticks = window::drive_ticks(8000, ARR) as i64;
    let v_sum = (bemf::BOXCAR_TICKS as i64 * ticks * 2960 * TIMING.recip_arr_q24 as i64) >> 24;
    let r_sum = (8192i64 * bemf::BOXCAR_TICKS as i64 * 100) >> 12;
    let expect = ((v_sum - r_sum) * 16) / bemf::BOXCAR_TICKS as i64;
    let got = sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps) as i64;
    assert!((got - expect).abs() <= 1, "got {got} expect {expect}");
    assert_eq!(got, 8363, "pin");
    // torque off lands at the next CONTROL phase, which still measures the
    // last drive; the OBSERVER tick after it measures sub-floor, so the half
    // it closes voids, and the PUBLISH phase after that drops to 0
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..DECIM_MED - 1 {
        k.on_scan(frame(2000, BIAS + 100), &sh);
    }
    assert_eq!(
        sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps),
        8363
    );
    k.on_scan(frame(2000, BIAS + 100), &sh);
    assert_eq!(sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps), 0);
}

/// The back-EMF floor gates the observer alone: a 293-tick window under a
/// 294-tick floor still feeds the terminal differential everywhere else.
#[test]
fn bemf_floor_voids_the_observer_and_keeps_the_differential() {
    for (floor, want) in [(293, 8363), (294, 0)] {
        let sh = Shared::new();
        drive_setup(&sh);
        let mut k = Kernel::new(
            FakeIo {
                sensors: FakeSensors,
                motor: FakeMotor { last: None },
            },
            KernelTiming {
                bemf_min_ticks: floor,
                ..TIMING
            },
        );
        settle(&mut k, &sh, frame(2000, BIAS + 100));
        run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::OBSERVER + 1);
        for _ in 0..20 {
            k.on_scan(frame(2000, BIAS + 100), &sh);
        }
        run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::PUBLISH + 1);
        assert_eq!(
            sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps),
            want,
            "floor {floor}"
        );
        assert_eq!(k.fast.vdiff_last, 2960, "floor {floor}");
    }
}

#[test]
fn velocity_feedback_switches_to_the_bemf_and_back() {
    let sh = Shared::new();
    drive_setup(&sh);
    let mut k = kernel();
    let published = |sh: &Shared| {
        sh.table.with(|t| {
            (
                t.telemetry.estimates.omega_hat_cps,
                t.telemetry.mode.omega_hat_src,
            )
        })
    };
    // the fourth clean boxcar flips the source; until then the published
    // source is the observer
    let mut n = 0;
    while k.medium.omega_sw.source() != OmegaSource::Bemf {
        k.on_scan(frame(2000, BIAS + 100), &sh);
        assert_eq!(published(&sh).1, 0);
        n += 1;
        assert!(n < 100, "the source never flipped");
    }
    // the PUBLISH phase lands it
    run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::PUBLISH + 1);
    let bemf = sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps) as i32;
    assert!(bemf > 0);
    assert_eq!(published(&sh), (bemf << 16, 1));
    // torque off lands at the next CONTROL phase; the half the OBSERVER
    // phase after it closes voids, and the source rides the held boxcar
    // through that one result, then falls back at the next period's void
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    k.on_scan(frame(2000, BIAS + 100), &sh);
    run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::PUBLISH + 1);
    assert_eq!(published(&sh), (bemf << 16, 1));
    k.on_scan(frame(2000, BIAS + 100), &sh);
    run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::PUBLISH + 1);
    assert_eq!(published(&sh), (k.medium.fusion.omega_q16(), 0));
}

// --- Current-loop feedforward ---------------------------------------------

#[test]
fn ke_feedforward_rides_the_profile_never_an_estimate() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        // current PI inert: the duty IS the Ke feedforward
        t.config.loop_current.i_kp_q88 = 0;
        t.config.loop_current.i_ki_q412 = 0;
        t.config.loop_current.i_kaw_q412 = 0;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 1600;
    });
    let mut k = kernel();
    // profile ramps 50 c/s per medium tick to 1600 and holds; the pot sits
    // still, so both velocity estimates read ~0 - the feed must not
    for _ in 0..1000 {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    assert_eq!(k.medium.traj.omega_star_q16(), 1600 << 16);
    // ke 0.0625 vcounts per c/s * 1600 c/s = 100 vcounts on a 3000 rail
    let u_ff = q_mul(1600 << 16, 256, 28);
    assert_eq!(u_ff, 100);
    let expect = q_mul(u_ff, k.medium.vbus.recip_q15() as i32, 15) as i16;
    assert_eq!(k.fast.duty_q15, expect);
    assert!((expect as i32 - 1092).abs() <= 1, "duty={expect}");
    // Current mode: no profile, no feed - even with the shaft spinning
    // (the pot observer would read ~2000 c/s here)
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 0;
    });
    for n in 0..2000u16 {
        k.on_scan(frame(1000 + n / 10, BIAS), &sh);
    }
    assert!(
        k.medium.fusion.omega_q16() > 1000 << 16,
        "pot omega {}",
        k.medium.fusion.omega_q16() >> 16
    );
    assert_eq!(k.fast.duty_q15, 0);
}

// --- Closed-loop plant ----------------------------------------------------

/// Crude integer plant in the identification-model shape:
/// `omega[k+1] = a*omega[k] + b*duty - coulomb`, theta integrates, pot =
/// theta >> 16 clamped 0..4095, `i = (duty*vbus>>15 - ke*omega)/R`. Units
/// loose on purpose - qualitative closed-loop behavior, not fidelity.
struct Plant {
    theta_q16: i64,
    omega_cps: i32,
    vbus: u16,
    frozen: bool,
}

impl Plant {
    fn new(pos: u16) -> Self {
        Self {
            theta_q16: (pos as i64) << 16,
            omega_cps: 0,
            vbus: 3000,
            frozen: false,
        }
    }

    /// One FAST tick: a = 1 - 1/64, b*duty = duty*200>>15, coulomb 8 c/s
    /// toward zero; frozen = hard wall (theta pinned, omega 0).
    fn step(&mut self, duty: i16) -> SensorFrame {
        if self.frozen {
            self.omega_cps = 0;
        } else {
            self.omega_cps += ((duty as i32 * 200) >> 15) - (self.omega_cps >> 6);
            if self.omega_cps > 0 {
                self.omega_cps = (self.omega_cps - 8).max(0);
            } else {
                self.omega_cps = (self.omega_cps + 8).min(0);
            }
            self.theta_q16 += ((self.omega_cps as i64) << 16) / 20_000;
        }
        let pos = (self.theta_q16 >> 16).clamp(0, 4095) as u16;
        // ke = 1/16 vcounts per c/s, R = 2 vcounts/ccount
        let v = (duty as i32 * self.vbus as i32) >> 15;
        let i = (v - (self.omega_cps >> 4)) >> 1;
        let mag = if duty >= 0 { i } else { -i };
        let sample = (BIAS as i32 + mag).clamp(0, 4095) as u16;
        let (va, vb) = if duty >= 0 {
            (self.vbus, 40)
        } else {
            (40, self.vbus)
        };
        SensorFrame {
            pos,
            current: sample,
            current_trough: BIAS,
            vmotor_a: va,
            vmotor_a_trough: va,
            vmotor_b: vb,
            vmotor_b_trough: vb,
            vcal: 1200,
            vbus_raw: vbus_raw(self.vbus),
            ntc_raw: 2048,
        }
    }

    fn pos(&self) -> i32 {
        (self.theta_q16 >> 16) as i32
    }
}

fn run_plant(k: &mut Kernel<FakeIo>, sh: &Shared, plant: &mut Plant, ticks: u32) {
    for _ in 0..ticks {
        let f = plant.step(k.fast.duty_q15);
        k.on_scan(f, sh);
    }
}

#[test]
fn position_step_settles_without_limit_cycle() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3000;
        // the plant/gain pair is qualitative; keep the tracking screen out
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        // this plant accelerates far beyond what the (deliberately
        // degenerate) b_i predict explains, so a large l3 rails tau_d into
        // the collision trip on every ramp; keep it gentle here
        t.config.fusion.l3_q88 = 256;
        // tighter position loop so the post-profile residual closes inside
        // the run instead of creeping on a 1 s time constant
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    run_plant(&mut k, &sh, &mut plant, 40_000); // 2 s
    assert_eq!(k.faults.mask(), 0);
    assert!(
        (plant.pos() - 3000).abs() <= 16,
        "settled at {}",
        plant.pos()
    );
    assert_eq!(k.medium.traj.theta_star_q16(), 3000 << 16, "profile landed");
    // trailing window: position parked, motor overwhelmingly braked
    let (mut lo, mut hi, mut parked) = (i32::MAX, i32::MIN, 0u32);
    for _ in 0..2000 {
        let f = plant.step(k.fast.duty_q15);
        k.on_scan(f, &sh);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
        if matches!(last_cmd(&k), MotorCmd::Brake) {
            parked += 1;
        }
    }
    assert!(hi - lo <= 2, "limit cycle: spread {}", hi - lo);
    assert!(parked >= 1500, "parked ticks {parked}");
}

#[test]
fn velocity_mode_tracks_a_ramp() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        // plenty of travel so the pot never rails
        t.config.pos_soft.pos_max_soft_counts = 1_000_000;
    });
    let mut k = kernel();
    let mut plant = Plant::new(100);
    for seg in 1..=4i32 {
        sh.table
            .with_mut(|t| t.control.lifecycle.goal_velocity = 500 * seg);
        run_plant(&mut k, &sh, &mut plant, 4000); // 200 ms per segment
    }
    assert_eq!(k.faults.mask(), 0);
    let omega_hat = k.medium.fusion.omega_q16() >> 16;
    assert!((omega_hat - 2000).abs() <= 300, "omega_hat={omega_hat}");
    assert!(
        (plant.omega_cps - 2000).abs() <= 300,
        "plant omega={}",
        plant.omega_cps
    );
}

#[test]
fn hard_wall_stall_yields() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3500;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(500);
    plant.frozen = true;
    // The live observer first books the wall current as tau_d, so the
    // wind-up is integrator-paced (~230 ms to the full limit); the press
    // verdict then folds i_lim to i_cont and the fold holds while the
    // held current sits at it. The observer's settle makes any
    // single-phase pin flaky - assert the run's invariants instead.
    let mut saw_full = false;
    let mut saw_fold = false;
    for _ in 0..24_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.medium.i_ref_cc == 1200 {
            saw_full = true;
        }
        let mut lim = 0u16;
        sh.table.with(|t| lim = t.telemetry.estimates.i_lim_counts);
        if lim == 300 {
            saw_fold = true;
        }
    }
    assert!(saw_full, "wind-up reached i_lim");
    assert!(saw_fold, "stall verdict folded to yield");
    assert_eq!(k.faults.mask(), 0, "Yield never latches");
}

#[test]
fn hard_wall_stall_faults() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3500;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_response = StallResponse::Fault;
    });
    let mut k = kernel();
    let mut plant = Plant::new(500);
    plant.frozen = true;
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert_eq!(k.faults.mask(), faults::BIT_STALL);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    sh.table
        .with(|t| assert_eq!(t.telemetry.mode.fault_code, faults::CODE_STALL));
}

#[test]
fn hold_parks_releases_and_reparks() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.fusion.l3_q88 = 256;
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1990);
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake), "never parked");
    assert_eq!(k.faults.mask(), 0);
    // a goal write releases the park (omega_star != 0 drops hold); the whole
    // commanded move must run without a single park, then it re-parks once
    // the profile lands inside the deadband
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = 2100);
    let mut saw_move = false;
    let mut reparked = false;
    for _ in 0..20_000u32 {
        run_plant(&mut k, &sh, &mut plant, 1);
        let moving = k.medium.traj.omega_star_q16() != 0;
        if moving {
            saw_move = true;
            assert!(!matches!(last_cmd(&k), MotorCmd::Brake), "park mid-move");
        }
        if saw_move && !moving && matches!(last_cmd(&k), MotorCmd::Brake) {
            reparked = true;
        }
    }
    assert!(saw_move, "goal write never started the profile");
    assert!(reparked, "never re-parked after the move");
    assert_eq!(k.faults.mask(), 0);
    // an external push out of the deadband wakes the park, then it re-parks
    plant.theta_q16 -= 60i64 << 16;
    let mut woke = false;
    for _ in 0..2000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        woke |= !matches!(last_cmd(&k), MotorCmd::Brake);
    }
    assert!(woke, "push never woke the hold");
    let mut parked = 0u32;
    for _ in 0..30_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if matches!(last_cmd(&k), MotorCmd::Brake) {
            parked += 1;
        }
    }
    assert!(
        parked > 15_000,
        "never re-parked after the push: parked {parked}"
    );
    assert_eq!(k.faults.mask(), 0);
    assert!((plant.pos() - 2100).abs() <= 16, "rest {}", plant.pos());
}

#[test]
fn hold_stays_parked_under_pot_noise() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.fusion.l3_q88 = 256;
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1990);
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake), "never parked");
    // parked hold under +-3 counts of pot noise: gating on position rest
    // (not the noisy omega_hat) keeps the park engaged - a wake per blip is
    // the shake fix2 removes
    let mut rng: u32 = 0xdead_beef;
    let (mut parked, mut lo, mut hi) = (0u32, i32::MAX, i32::MIN);
    for _ in 0..40_000 {
        let mut f = plant.step(k.fast.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let n = ((rng >> 24) as i32 % 7) - 3;
        f.pos = (f.pos as i32 + n).clamp(0, 4095) as u16;
        k.on_scan(f, &sh);
        if matches!(last_cmd(&k), MotorCmd::Brake) {
            parked += 1;
        }
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(hi - lo <= 4, "hold limit cycle: spread {}", hi - lo);
    assert!(parked >= 36_000, "parked ticks {parked} of 40000");
}

/// A still pot at `raw` under a goal of `goal`: whether the loop parks.
fn parks(sh: &Shared, raw: u16, goal: i32) -> bool {
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = goal;
        t.config.loop_position.pos_deadband_counts = 12;
    });
    let mut k = kernel();
    for _ in 0..10_000 {
        k.on_scan(frame(raw, BIAS), sh);
    }
    assert_eq!(k.faults.mask(), 0);
    assert_eq!(k.medium.traj.theta_star_q16(), goal << 16, "profile landed");
    k.medium.hold
}

/// Local gain 26/16 over raw 1600..2240: raw 1924 linearizes to 2126.5,
/// where 12 raw counts are 19.5 linearized ones.
#[test]
fn hold_band_is_judged_in_raw_counts() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = [0i16; pos_lut::POINTS];
    for (n, c) in k.iter_mut().enumerate().take(141).skip(100) {
        *c = 10 * (n as i16 - 100);
    }
    sh.with_pos_lut_mut(|a| *a = k);
    sh.table
        .with_mut(|t| t.control.pos_lut.pos_lut_state = pos_lut::state::LIVE);
    assert_eq!(sh.pos_lut_q4(1924), 34024);
    assert!(parks(&sh, 1924, 2146));
    assert!(!parks(&sh, 1924, 2147));
    assert!(parks(&sh, 1924, 2107));
    assert!(!parks(&sh, 1924, 2106));
    // the identity: the band is the deadband
    sh.table
        .with_mut(|t| t.control.pos_lut.pos_lut_state = pos_lut::state::IDENTITY);
    assert!(parks(&sh, 1924, 1936));
    assert!(!parks(&sh, 1924, 1937));
    assert!(parks(&sh, 1924, 1912));
    assert!(!parks(&sh, 1924, 1911));
}

#[test]
fn hold_freezes_and_drains_the_velocity_loop() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.fusion.l3_q88 = 256;
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1990);
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(k.medium.hold, "never parked");
    let mut rng: u32 = 0xdead_beef;
    for _ in 0..20_000 {
        let mut f = plant.step(k.fast.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f.pos = (f.pos as i32 + ((rng >> 24) as i32 % 7) - 3).clamp(0, 4095) as u16;
        k.on_scan(f, &sh);
        if k.medium.hold && k.phase == phase::VELOCITY + 1 {
            assert_eq!(k.medium.i_ref_cc, 0, "parked with a current command");
        }
    }
    // pushed out of the band: the first velocity step after the park is a
    // fresh loop's on the same inputs, so no charge survived the hold
    plant.theta_q16 -= 60i64 << 16;
    for _ in 0..2000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.phase == phase::VELOCITY + 1 && !k.medium.hold {
            let (lv, motor) = sh.table.with(|t| (t.config.loop_velocity, t.calib.motor));
            let fresh = VelocityLoop::new().step(
                k.medium.omega_ref_q16,
                k.medium.omega_hat,
                k.medium.traj.alpha_star_q16(),
                k.medium.traj.omega_star_q16(),
                k.medium.i_band,
                &lv,
                &motor,
            );
            assert_ne!(fresh, 0, "the wake must command something");
            assert_eq!(k.medium.i_ref_cc, fresh, "the hold left integrator charge");
            return;
        }
    }
    panic!("push never woke the hold");
}

#[test]
fn current_mode_endstop_unwinds_duty_to_zero() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 300;
        t.config.pos_soft.pos_max_soft_counts = 3000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    // free flight into the soft wall with a wound-up loop
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert_eq!(k.faults.mask(), 0);
    assert!(
        plant.pos() >= 3000,
        "never reached the wall: {}",
        plant.pos()
    );
    // past the wall the endstop zeroes i_ref; the loop must unwind ALL the
    // way to zero, not freeze at the sub-floor duty the honest-zero feed
    // leaves behind (bench: 18% duty held grinding into the rail)
    assert_eq!(k.fast.duty_q15, 0, "duty froze above zero at the wall");
    let mut zero = 0u32;
    for _ in 0..2000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.fast.duty_q15 == 0 {
            zero += 1;
        }
    }
    assert!(zero >= 1990, "duty kept firing at the wall: {zero} of 2000");
}

#[test]
fn velocity_mode_brakes_at_the_soft_wall() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 2000;
        t.config.pos_soft.pos_max_soft_counts = 3000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    run_plant(&mut k, &sh, &mut plant, 30_000);
    assert_eq!(k.faults.mask(), 0);
    // the wall ramp shrinks the inward goal with distance: the profile
    // decelerates INTO the wall instead of leaving momentum to coast on an
    // open-loop current cut, and parks without a flip-flop limit cycle
    assert!(
        plant.pos() >= 2950 && plant.pos() <= 3020,
        "rest pos {}",
        plant.pos()
    );
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    let mut star_max = 0i32;
    for _ in 0..4000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
        star_max = star_max.max(k.medium.traj.omega_star_q16());
    }
    assert!(hi - lo <= 4, "limit cycle at the wall: spread {}", hi - lo);
    assert!(
        star_max <= 64 << 16,
        "profile still commands inward speed: {}",
        star_max >> 16
    );
    // the door back out stays open
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_velocity = -500);
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(
        plant.pos() < 2900,
        "retreat from the wall failed: {}",
        plant.pos()
    );
}

#[test]
fn openloop_endstop_zeroes_outbound_duty() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        t.config.pos_soft.pos_max_soft_counts = 3000;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    // free flight into the top soft wall: the cut leaves only coast
    // momentum past it (the open-loop sweep that crashed the horn)
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert_eq!(k.faults.mask(), 0);
    assert!(
        plant.pos() >= 3000 && plant.pos() <= 3050,
        "wall not respected: {}",
        plant.pos()
    );
    assert_eq!(k.fast.duty_q15, 0, "outbound duty still firing at the wall");
    // retreat duty applies untouched and drives back off the wall
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
    run_plant(&mut k, &sh, &mut plant, 2_000);
    assert_eq!(k.fast.duty_q15, -8000, "retreat from the wall blocked");
    // mirrored at the min wall, crossed in free flight like the top one
    write_config(&sh, |t| t.config.pos_soft.pos_min_soft_counts = 500);
    run_plant(&mut k, &sh, &mut plant, 25_000);
    assert!(
        plant.pos() >= 450 && plant.pos() <= 500,
        "min wall not respected: {}",
        plant.pos()
    );
    assert_eq!(
        k.fast.duty_q15, 0,
        "outbound duty still firing at the min wall"
    );
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    run_plant(&mut k, &sh, &mut plant, 5_000);
    assert_eq!(k.fast.duty_q15, 8000, "retreat from the min wall blocked");
    assert!(plant.pos() > 600, "never drove off the min wall");
    assert_eq!(k.faults.mask(), 0);
}

/// An open-loop push held at the soft wall is not a stall: the endstop
/// closed that side of the band, so the duty limiter holds a zero ceiling
/// with no current flowing, and the stall timer never runs there.
#[test]
fn openloop_held_at_the_soft_wall_never_stalls() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        t.config.pos_soft.pos_max_soft_counts = 3000;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
        t.config.limits.stall_response = StallResponse::Fault;
    });
    let mut k = kernel();
    let mut plant = Plant::new(2800);
    // to the wall, then held there for ten stall windows
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(
        plant.pos() >= 3000,
        "never reached the wall: {}",
        plant.pos()
    );
    run_plant(&mut k, &sh, &mut plant, 10 * ms_ticks(50));
    assert_eq!(k.faults.mask(), 0, "the wall tripped the stall timer");
    assert_eq!(k.fast.duty_q15, 0);
}

/// The same push pinned by the current limit mid travel is a stall.
#[test]
fn openloop_pinned_mid_travel_still_stalls() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 30000;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
        t.config.limits.stall_response = StallResponse::Fault;
    });
    let mut k = kernel();
    let mut plant = Plant::new(2000);
    plant.frozen = true;
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert_eq!(k.faults.mask(), faults::BIT_STALL);
    sh.table
        .with(|t| assert_eq!(t.telemetry.mode.fault_code, faults::CODE_STALL));
}

#[test]
fn position_step_survives_tick_deletion() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.fusion.l3_q88 = 256;
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    // ~10% of ISR invocations vanish, as lost ticks do; the plant keeps
    // running on the stale duty and the move still settles and parks
    let mut rng: u32 = 0x1357_9bdf;
    for _ in 0..40_000 {
        let f = plant.step(k.fast.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        if !rng.is_multiple_of(10) {
            k.on_scan(f, &sh);
        }
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(
        (plant.pos() - 3000).abs() <= 24,
        "settled at {}",
        plant.pos()
    );
    // still parks: trailing spread stays flat
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    for _ in 0..2000 {
        let f = plant.step(k.fast.duty_q15);
        k.on_scan(f, &sh);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
    }
    assert!(hi - lo <= 4, "limit cycle: spread {}", hi - lo);
}

// --- Tick block and TEL stream ----------------------------------------------

/// The table's tick block as the last tick left it.
fn tick_block(sh: &Shared) -> TelemetryTick {
    sh.table.with(|t| t.telemetry.tick)
}

/// The tick block as each row's first chunk sees it: read at `on_publish`.
struct RecTel {
    table: *const crate::ControlTableCell,
    rows: heapless::Vec<TelemetryTick, 64>,
}
impl RecTel {
    fn new(sh: &Shared) -> Self {
        Self {
            table: &sh.table,
            rows: heapless::Vec::new(),
        }
    }
}
impl TelStream for RecTel {
    fn on_publish(&mut self, _: impl FnOnce() -> bool) {
        // SAFETY: the test's `Shared` outlives the kernel that owns this.
        let t = unsafe { (*self.table).with(|t| t.telemetry.tick) };
        let _ = self.rows.push(t);
    }
}

/// Whether the kernel held the TX lane on the last tick.
#[derive(Default)]
struct HoldTel {
    held: bool,
}
impl TelStream for HoldTel {
    fn on_publish(&mut self, _: impl FnOnce() -> bool) {}
    fn hold(&mut self) {
        self.held = true;
    }
}

/// The kernel holds the TX lane on every tick that runs a SLOW stage (due
/// or finish) or a thermometer stage (an install or a side, each consuming
/// its pending bit) and on no other: never a configuration take's tick, never a
/// plain one. A tracked seat runs every stage, an install included. The
/// stages are the budget probe's (`Events::between` on the tags).
#[test]
fn slow_and_thermometer_stage_ticks_hold_the_tel_lane() {
    let sh = Shared::new();
    seed(&sh);
    seed_thermometer(&sh, RIG_R_COLD_Q12);
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        HoldTel::default(),
        TIMING,
    );
    let hot = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hot);
    // due, finish, install, side, configuration take
    let mut seen = [0u32; 5];
    for t in 0..BASE_FAST_TICKS {
        if t == BASE_FAST_TICKS / 2 {
            write_config(&sh, |_| {});
        }
        let before = k.probe_tags();
        k.tel.held = false;
        k.on_scan(hot, &sh);
        let ev = Events::between(before, k.probe_tags());
        let kind = [ev.slow, ev.finish, ev.install, ev.side, ev.refresh];
        for (n, &hit) in seen.iter_mut().zip(&kind) {
            *n += hit as u32;
        }
        let want = kind[..4].iter().any(|&h| h);
        assert_eq!(k.tel.held, want, "tick {t}: {kind:?}");
    }
    assert!(seen.iter().all(|&n| n > 0), "{seen:?}");
}

/// The bench probe books each tick from the tags it carried from the tick
/// before (`KernelProbe::tick`) exactly as from tags read on either side of
/// the body: every stage, take, drive-arm change, TEL bank, window close
/// and halt included.
#[test]
fn the_probe_books_from_carried_tags_as_from_tags_around_the_body() {
    use crate::budget::TICK_PERIOD;
    use crate::budget::probe::{HALT_TICKS, KernelProbe};
    let sh = Shared::new();
    seed(&sh);
    seed_thermometer(&sh, RIG_R_COLD_Q12);
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        HoldTel::default(),
        TIMING,
    );
    let hot = frame(2000, BIAS + 400);
    let mut around = KernelProbe::ZERO;
    let mut carried = KernelProbe::ZERO;
    let mut banks = 0u32;
    let mut events = [0u32; 9];
    let ticks = BASE_FAST_TICKS + 2000;
    for t in 0..ticks {
        match t {
            1000 => sh.table.with_mut(|c| {
                c.control.lifecycle.mode = Mode::Velocity;
                c.control.lifecycle.goal_velocity = 1000;
            }),
            1500 => sh
                .table
                .with_mut(|c| c.control.lifecycle.mode = Mode::OpenLoop),
            _ if t == ticks / 2 => write_config(&sh, |_| {}),
            _ => {}
        }
        let entry = t * TICK_PERIOD;
        let body = match t % 997 {
            0 => HALT_TICKS + 1,
            n => 600 + n,
        };
        let tel = t % 3000 < 1000;
        let was = banks;
        banks += (tel && t % 16 == 0) as u32;
        let health = t % 20 == 7;
        let before = k.probe_tags();
        k.on_scan(hot, &sh);
        let after = k.probe_tags();
        let ev = Events {
            tel,
            bank: banks != was,
            health,
            ..Events::between(before, after)
        };
        let kinds = [
            ev.slow,
            ev.finish,
            ev.close,
            ev.install,
            ev.side,
            ev.refresh,
            ev.bank,
            ev.health,
            before.regime != after.regime,
        ];
        for (n, &hit) in events.iter_mut().zip(&kinds) {
            *n += hit as u32;
        }
        let r = before.regime.booked(after.regime);
        around.book(body, before.phase as usize, r, ev, !before.booted);
        around.ended(entry, entry + body);
        carried.tick(body, || after, tel, banks, health);
        carried.ended(entry, entry + body);
    }
    assert!(events.iter().all(|&n| n > 0), "{events:?}");
    assert_eq!(carried.booked(), around.booked());
}

/// The row's first chunk is taken at `on_publish`: the tick block is whole by
/// then - this scan's frame, the stamp the ISR published, and the
/// conclusions on the window the previous command drove - and nothing in
/// the tick writes it after, so both chunks copy one tick.
#[test]
fn tick_block_is_whole_when_the_row_is_taken() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        RecTel::new(&sh),
        TIMING,
    );
    settle(&mut k, &sh, frame(2100, BIAS + 40));
    for n in 0..2 * DECIM_MED as u32 {
        k.tel.rows.clear();
        sh.table
            .with_mut(|t| t.telemetry.tick.sample_tick = 0x1_0000 + n);
        let f = frame(2100 + n as u16, BIAS + 40);
        k.on_scan(f, &sh);
        let row = k.tel.rows[0];
        assert_eq!(row, tick_block(&sh), "tick {n}: written after the row");
        assert_eq!((row.pos, row.sample_tick), (f.pos, 0x1_0000 + n));
        assert_eq!(row.current_trough, BIAS);
        assert_eq!(row.duty_ticks, 293, "8000/32768 of ARR {ARR}");
        assert_eq!(row.i_tick_counts, 40);
        assert_eq!(
            row.tick_flags,
            tick_flag::I_VALID | tick_flag::V_VALID,
            "duty 8000 is far above the 100-tick window floors"
        );
    }
}

/// The tick block every tick, streaming or not: the duty is the command
/// whose window this frame measured, in the compare counts the bridge
/// applied, logical sign; with nothing driving, no window and no current.
#[test]
fn tick_block_carries_the_measured_window() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS), &sh);
    let t = tick_block(&sh);
    assert_eq!((t.duty_ticks, t.i_tick_counts, t.tick_flags), (0, 0, 0));
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    settle(&mut k, &sh, frame(2100, BIAS + 40));
    for _ in 0..20 {
        k.on_scan(frame(2100, BIAS + 40), &sh);
    }
    let t = tick_block(&sh);
    assert_eq!(t.duty_ticks, window::duty_counts(8000, ARR));
    assert_eq!(t.i_tick_counts, 40);
    // reverse rides negative, as the host's goal quantization has it
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
    settle(&mut k, &sh, frame(2100, BIAS + 40));
    for _ in 0..20 {
        k.on_scan(frame(2100, BIAS + 40), &sh);
    }
    let t = tick_block(&sh);
    assert_eq!(t.duty_ticks, -293);
    assert_eq!(t.duty_ticks, window::duty_counts(-8000, ARR));
    assert_eq!(t.i_tick_counts, -40);
}

// --- Configuration take ---------------------------------------------------

/// The OpenLoop duty clamp the kernel holds.
fn ol_max(k: &Kernel<FakeIo>) -> u16 {
    k.cfg.loop_current.duty_max_q15
}

#[test]
fn first_tick_takes_the_configuration_without_a_write() {
    let sh = Shared::new();
    seed(&sh);
    sh.table
        .with_mut(|t| t.config.loop_current.duty_max_q15 = 5000);
    let mut k = kernel();
    assert_eq!(sh.config_seq(), k.config_seq, "nothing moved the count");
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(ol_max(&k), 5000);
    assert_eq!(k.cfg.limits.stall_time_ticks, 1000, "50 ms");
    assert_eq!(
        sh.table.with(|t| t.telemetry.limits.window_floor_q15),
        k.cfg.ol_floor_q15
    );
}

#[test]
fn floor_inputs_are_read_only() {
    use crate::regions::calib::CalibRegs;
    use crate::regions::calib::addr::sense::{I_WINDOW_MIN_TICKS, V_WINDOW_MIN_TICKS};
    for input in [I_WINDOW_MIN_TICKS, V_WINDOW_MIN_TICKS] {
        assert!(
            CalibRegs::CT_WRITABLE_ABS
                .iter()
                .all(|&(a, n)| input + 2 <= a || a + n <= input),
            "{input:#x} is writable"
        );
    }
}

#[test]
fn a_take_keeps_the_floors_of_the_first_tick() {
    let f = frame(2000, BIAS);
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    k.on_scan(f, &sh);
    let floors = |k: &Kernel<FakeIo>| {
        let c = &k.cfg;
        (c.ol_floor_q15, c.v_floor_ticks, c.v_floor_q15)
    };
    let published = || {
        sh.table.with(|t| {
            (
                t.telemetry.limits.window_floor_q15,
                t.telemetry_ext.limits_ext.window_v_floor_q15,
            )
        })
    };
    let booted = (floors(&k), published());
    write_config(&sh, |t| {
        t.calib.sense.i_window_min_ticks = 240;
        t.calib.sense.v_window_min_ticks = 240;
        t.config.loop_current.duty_max_q15 = 5000;
    });
    for _ in 0..2 * DECIM_MED {
        k.on_scan(f, &sh);
    }
    assert_eq!(ol_max(&k), 5000, "the take ran");
    assert_eq!((floors(&k), published()), booted);
}

#[test]
fn unset_rating_and_goal_follow_a_committed_cap_change() {
    use crate::regions::config::addr::limits::CURRENT_LIMIT_COUNTS;
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.config.limits.i_cont_counts = 0;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_current = 0;
    });
    let mut k = kernel();
    for cap in [1200u16, 600, 1800] {
        crate::services::bus::tests::write_committed(&sh, CURRENT_LIMIT_COUNTS, &cap.to_le_bytes());
        for _ in 0..2 * DECIM_MED as u32 * DECIM_SLOW as u32 {
            k.on_scan(frame(2000, BIAS), &sh);
        }
        assert_eq!(sh.table.with(|t| t.config.limits.i_cont_counts), 0);
        assert_eq!(sh.table.with(|t| t.telemetry.estimates.i_lim_counts), cap);
        assert_eq!(limit_flags(&sh), 0);
    }
}

#[test]
fn dispatched_config_write_lands_at_the_take() {
    use crate::regions::config::addr::loop_current::DUTY_MAX_Q15;
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(ol_max(&k), 32767);
    crate::services::bus::tests::write_committed(&sh, DUTY_MAX_Q15, &5000u16.to_le_bytes());
    assert_eq!(sh.table.with(|t| t.config.loop_current.duty_max_q15), 5000);
    while k.phase != phase::SPARE_A {
        k.on_scan(frame(2000, BIAS), &sh);
        assert_eq!(ol_max(&k), 32767, "before the take");
    }
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(ol_max(&k), 5000);
}

/// A write across the span: a fast-path field, a medium field and the Ke
/// pair the belt and the boxcar read.
fn write_across_the_span(sh: &Shared) {
    write_config(sh, |t| {
        t.config.loop_current.duty_max_q15 = 5000;
        t.config.limits.stall_time_ticks = ms_ticks(20);
        t.calib.motor.recip_ke_q = 0;
    });
}

/// Which of the three fields the kernel holds new.
fn fields_new(k: &Kernel<FakeIo>) -> [bool; 3] {
    let c = &k.cfg;
    assert_eq!(
        c.ke_unset,
        c.motor.recip_ke_q == 0,
        "the belt reads the boxcar's Ke"
    );
    [
        ol_max(k) == 5000,
        c.limits.stall_time_ticks == 400,
        c.motor.recip_ke_q == 0,
    ]
}

#[test]
fn config_write_lands_whole_at_one_tick_within_a_medium_period() {
    let f = frame(2000, BIAS);
    for at in 0..DECIM_MED {
        let sh = Shared::new();
        seed(&sh);
        let mut k = kernel();
        for _ in 0..DECIM_MED + at {
            k.on_scan(f, &sh);
        }
        write_across_the_span(&sh);
        let mut ticks = 0;
        while fields_new(&k) != [true; 3] {
            let p = k.phase;
            k.on_scan(f, &sh);
            ticks += 1;
            let after = fields_new(&k);
            assert!(
                after == [false; 3] || (after == [true; 3] && p == phase::SPARE_A),
                "write at {at}: {after:?} after phase {p}"
            );
        }
        assert!(ticks <= DECIM_MED as u32, "write at {at}: {ticks} ticks");
    }
}

#[test]
fn unmoved_count_keeps_the_configuration() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS), &sh);
    // a table edit no commit announced never reaches the kernel
    sh.table
        .with_mut(|t| t.config.loop_current.duty_max_q15 = 5000);
    for _ in 0..4 * DECIM_MED {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    assert_eq!(ol_max(&k), 32767);
}

#[test]
fn a_write_after_the_take_takes_again() {
    let f = frame(2000, BIAS);
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    k.on_scan(f, &sh);
    write_config(&sh, |t| t.config.loop_current.duty_max_q15 = 6000);
    run_to(&mut k, &sh, f, phase::SPARE_A);
    k.on_scan(f, &sh);
    assert_eq!(ol_max(&k), 6000);
    // a commit after the take: the count it recorded is behind, so the next
    // SPARE_A copies the span again
    write_config(&sh, |t| t.config.limits.stall_time_ticks = ms_ticks(20));
    sh.table
        .with_mut(|t| t.config.loop_current.duty_max_q15 = 5000);
    assert_ne!(k.config_seq, sh.config_seq());
    for _ in 0..DECIM_MED {
        k.on_scan(f, &sh);
    }
    assert_eq!(ol_max(&k), 5000);
    assert_eq!(k.cfg.limits.stall_time_ticks, 400);
    assert_eq!(k.config_seq, sh.config_seq());
}

/// The kernel's span as its bytes.
fn span_bytes(k: &Kernel<FakeIo>) -> [u8; SPAN_BYTES] {
    // SAFETY: the span is the front of the repr(C) `KernelConfig`
    // (`config` const asserts), every byte initialized.
    unsafe { (&raw const k.cfg).cast::<[u8; SPAN_BYTES]>().read() }
}

const SPAN_BYTES: usize = (SPAN_END - SPAN_START) as usize;

/// The next committed set: the table's span new in every word, every value
/// still valid for its field.
fn next_set(sh: &Shared) -> [u32; SPAN_BYTES / 4] {
    use crate::regions::config::addr::limits::OPENLOOP_ZERO_BRAKE;
    let mut words = [0u32; SPAN_BYTES / 4];
    sh.table.with(|t| {
        // SAFETY: the table is repr(C) bytes, the span word-aligned.
        let p = (t as *const ControlTable).cast::<u32>();
        for (i, w) in words.iter_mut().enumerate() {
            *w = unsafe { p.add(SPAN_START as usize / 4 + i).read() };
        }
    });
    let bool_word = (OPENLOOP_ZERO_BRAKE - SPAN_START) as usize / 4;
    for (i, w) in words.iter_mut().enumerate() {
        // byte 0 of each word is a plain integer, but for the one bool, whose
        // neighbour is reserved
        let shift = if i == bool_word { 8 } else { 0 };
        *w = w.wrapping_add(1 << shift);
    }
    words
}

fn store_word(sh: &Shared, i: usize, w: u32) {
    // SAFETY: as `next_set`; the word holds valid values for its fields.
    sh.table.with_mut(|t| unsafe {
        (t as *mut ControlTable)
            .cast::<u32>()
            .add(SPAN_START as usize / 4 + i)
            .write(w)
    });
}

/// A SPARE_A tick after every store of a whole-span commit, with the
/// set committed before it not yet taken: the kernel keeps the set it holds
/// while the count is odd, never a mix, and takes the newest whole set at
/// the first SPARE_A after the commit ends.
#[test]
fn kernel_never_reads_a_set_under_commit() {
    let f = frame(2000, BIAS);
    for stores in 0..=SPAN_BYTES / 4 {
        let sh = Shared::new();
        seed(&sh);
        let mut k = kernel();
        k.on_scan(f, &sh);
        let held = span_bytes(&k);
        // a whole set committed and not yet taken
        let b = next_set(&sh);
        sh.config_commit_begin();
        for (i, &w) in b.iter().enumerate() {
            store_word(&sh, i, w);
        }
        sh.config_commit_end();
        // the next one under way at SPARE_A, `stores` words in
        run_to(&mut k, &sh, f, phase::SPARE_A);
        let c = next_set(&sh);
        sh.config_commit_begin();
        for (i, &w) in c.iter().enumerate().take(stores) {
            store_word(&sh, i, w);
        }
        k.on_scan(f, &sh);
        assert_eq!(span_bytes(&k), held, "{stores} stores in");
        for (i, &w) in c.iter().enumerate().skip(stores) {
            store_word(&sh, i, w);
        }
        sh.config_commit_end();
        run_to(&mut k, &sh, f, phase::SPARE_A);
        k.on_scan(f, &sh);
        let want: heapless::Vec<u8, SPAN_BYTES> = c.iter().flat_map(|w| w.to_ne_bytes()).collect();
        assert_eq!(span_bytes(&k)[..], want[..], "{stores} stores in");
        assert_eq!(k.config_seq, sh.config_seq());
    }
}

#[test]
fn a_take_moves_the_probe_tag_at_therm_install_alone() {
    let f = frame(2000, BIAS);
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    k.on_scan(f, &sh);
    run_to(&mut k, &sh, f, phase::CONTROL);
    write_across_the_span(&sh);
    let mut moves = [0u32; DECIM_MED as usize];
    for _ in 0..2 * DECIM_MED {
        let before = k.probe_tags();
        k.on_scan(f, &sh);
        if k.probe_tags().config_seq != before.config_seq {
            moves[before.phase as usize] += 1;
        }
    }
    let mut want = [0; DECIM_MED as usize];
    want[phase::SPARE_A as usize] = 1;
    assert_eq!(moves, want);
}

/// The kernel's `hold_start` tick by tick.
#[derive(Default)]
struct StartTel {
    held: heapless::Vec<bool, 64>,
}
impl TelStream for StartTel {
    fn on_publish(&mut self, hold_start: impl FnOnce() -> bool) {
        let _ = self.held.push(hold_start());
    }
}

/// From a commit to the SPARE_A take that ends it, every tick holds
/// a burst's start, the take's own included, and after it only the take's
/// phase does; an odd count holds too, and its take waits for the next
/// period's SPARE_A.
#[test]
fn a_pending_take_holds_a_burst_start_through_its_tick() {
    let f = frame(2000, BIAS);
    for odd_at_install in [false, true] {
        let sh = Shared::new();
        seed(&sh);
        let mut k = Kernel::with_tel(
            FakeIo {
                sensors: FakeSensors,
                motor: FakeMotor { last: None },
            },
            StartTel::default(),
            TIMING,
        );
        k.on_scan(f, &sh);
        run_to(&mut k, &sh, f, phase::LIMITS);
        k.tel.held.clear();
        sh.config_commit_begin();
        if !odd_at_install {
            sh.config_commit_end();
        }
        let mut take_at = None;
        for i in 0..3 * DECIM_MED as usize {
            if odd_at_install && i == DECIM_MED as usize {
                sh.config_commit_end();
            }
            let seq = k.config_seq;
            k.on_scan(f, &sh);
            if take_at.is_none() && k.config_seq != seq {
                take_at = Some(i);
            }
        }
        let take_at = take_at.expect("no take");
        let want = if odd_at_install { 15 } else { 5 };
        assert_eq!(take_at, want, "odd at install: {odd_at_install}");
        // tick i runs phase LIMITS + i
        let install =
            |i: usize| (phase::LIMITS as usize + i) % DECIM_MED as usize == phase::SPARE_A as usize;
        let want: heapless::Vec<bool, 64> = (0..k.tel.held.len())
            .map(|i| i <= take_at || install(i))
            .collect();
        assert_eq!(k.tel.held, want);
    }
}

// --- Host writes land at the CONTROL phase ---------------------------------

/// Ticks ahead of every phased write: a whole number of medium periods.
const PHASED_LEAD: u32 = 40 * DECIM_MED as u32;
/// Ticks recorded after it: two medium periods.
const PHASED_TICKS: usize = 2 * DECIM_MED as usize;

fn cmd_key(k: &Kernel<FakeIo>) -> (u8, i16) {
    match last_cmd(k) {
        MotorCmd::Disabled => (0, 0),
        MotorCmd::Coast => (1, 0),
        MotorCmd::Brake => (2, 0),
        MotorCmd::Drive { .. } => (3, written_duty(k)),
    }
}

/// Two medium periods after a write landing `phase` ticks into the
/// medium period, beside a twin that never sees it.
struct Phased {
    k: Kernel<FakeIo>,
    cmds: [(u8, i16); PHASED_TICKS],
    twin: [(u8, i16); PHASED_TICKS],
    /// Ticks run before the first CONTROL phase after the write.
    boundary: usize,
}

fn phased(setup: fn(&mut ControlTable), write: fn(&mut ControlTable), phase: u8) -> Phased {
    let f = frame(2000, BIAS);
    let (sh, twin_sh) = (Shared::new(), Shared::new());
    for s in [&sh, &twin_sh] {
        seed(s);
        s.table.with_mut(setup);
    }
    let (mut k, mut twin) = (kernel(), kernel());
    for _ in 0..PHASED_LEAD + phase as u32 {
        k.on_scan(f, &sh);
        twin.on_scan(f, &twin_sh);
    }
    sh.table.with_mut(write);
    let (mut cmds, mut twin_cmds) = ([(0, 0); PHASED_TICKS], [(0, 0); PHASED_TICKS]);
    for n in 0..PHASED_TICKS {
        k.on_scan(f, &sh);
        twin.on_scan(f, &twin_sh);
        cmds[n] = cmd_key(&k);
        twin_cmds[n] = cmd_key(&twin);
    }
    Phased {
        k,
        cmds,
        twin: twin_cmds,
        boundary: ((DECIM_MED - phase) % DECIM_MED) as usize,
    }
}

/// Every phase: the ticks ahead of the boundary run as if nothing was
/// written; `after` holds from the boundary on.
fn assert_lands_at_the_boundary(
    setup: fn(&mut ControlTable),
    write: fn(&mut ControlTable),
    after: impl Fn(&Phased, usize) -> bool,
) {
    for phase in 0..DECIM_MED {
        let r = phased(setup, write, phase);
        assert_eq!(
            r.cmds[..r.boundary],
            r.twin[..r.boundary],
            "phase {phase}: before the boundary"
        );
        for n in r.boundary..PHASED_TICKS {
            assert!(after(&r, n), "phase {phase} tick {n}: {:?}", r.cmds[n]);
        }
    }
}

fn openloop_8000(t: &mut ControlTable) {
    t.control.lifecycle.torque_enable = true;
    t.control.lifecycle.mode = Mode::OpenLoop;
    t.control.lifecycle.goal_duty = 8000;
}

/// The OpenLoop restart: from the window floor, one slew step per tick.
fn restarts_from_the_floor(r: &Phased, n: usize) -> bool {
    let floor = r.k.cfg.ol_floor_q15 as i16;
    let slew = (n - r.boundary + 1) as i16 * duty_limit::UP_Q15 as i16;
    r.cmds[n] == (3, (floor + slew).min(8000))
}

#[test]
fn torque_off_lands_at_the_control_phase() {
    assert_lands_at_the_boundary(
        openloop_8000,
        |t| t.control.lifecycle.torque_enable = false,
        |r, n| r.cmds[n] == (0, 0),
    );
}

#[test]
fn torque_on_lands_at_the_control_phase() {
    assert_lands_at_the_boundary(
        |t| {
            openloop_8000(t);
            t.control.lifecycle.torque_enable = false;
        },
        |t| t.control.lifecycle.torque_enable = true,
        restarts_from_the_floor,
    );
}

#[test]
fn mode_change_lands_at_the_control_phase() {
    assert_lands_at_the_boundary(
        |t| {
            t.control.lifecycle.torque_enable = true;
            t.control.lifecycle.mode = Mode::Current;
            t.control.lifecycle.goal_current = 300;
            t.control.lifecycle.goal_duty = 8000;
        },
        |t| t.control.lifecycle.mode = Mode::OpenLoop,
        restarts_from_the_floor,
    );
}

#[test]
fn goal_duty_lands_at_the_control_phase() {
    assert_lands_at_the_boundary(
        openloop_8000,
        |t| t.control.lifecycle.goal_duty = 4000,
        // a goal cut applies at once
        |r, n| r.cmds[n] == (3, 4000),
    );
}

// --- Medium phases ----------------------------------------------------------

/// The first period after boot, one phase per tick: each stage's output
/// shows up at its own phase and feeds the next, and the command's parts
/// land where the phase table says.
#[test]
fn the_chain_runs_in_phase_order() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 1000;
    });
    let mut k = kernel();
    let f = frame(2000, BIAS);
    let published = |sh: &Shared| {
        sh.table.with(|t| {
            (
                t.telemetry.estimates.vbus_counts,
                t.telemetry.mode.mode_active,
            )
        })
    };
    // CONTROL: the drive kind is set, nothing downstream has run
    k.on_scan(f, &sh);
    assert!(matches!(k.cmd.drive, fast::Drive::Closed { .. }));
    assert_eq!(k.medium.traj.omega_star_q16(), 0);
    // OBSERVER, then TRAJECTORY: the profile steps on the observer's
    // estimate
    k.on_scan(f, &sh);
    assert_eq!(k.medium.traj.omega_star_q16(), 0);
    k.on_scan(f, &sh);
    let omega_star = k.medium.traj.omega_star_q16();
    assert!(omega_star > 0);
    // LIMITS: the band exists but has not reached the command
    assert_eq!(k.medium.i_band, IBand { lo: 0, hi: 0 });
    k.on_scan(f, &sh);
    assert_eq!(
        k.medium.i_band,
        IBand {
            lo: -1200,
            hi: 1200
        }
    );
    assert_eq!(k.cmd.band, IBand { lo: 0, hi: 0 });
    assert_eq!(k.cmd.i_ref_cc, 0);
    // VELOCITY: the reference the profile asks for lands with the band
    k.on_scan(f, &sh);
    assert_eq!(
        k.cmd.band,
        IBand {
            lo: -1200,
            hi: 1200
        }
    );
    assert!(k.cmd.i_ref_cc > 0);
    assert_eq!(k.cmd.i_ref_cc, k.medium.i_ref_cc);
    // RAIL: the rail estimate reaches the command
    assert_eq!(k.cmd.vbus_counts, 0);
    k.on_scan(f, &sh);
    assert_eq!(k.cmd.vbus_counts, 3000);
    // SLOW, then PUBLISH
    k.on_scan(f, &sh);
    assert_eq!(published(&sh), (0, 0));
    k.on_scan(f, &sh);
    assert_eq!(published(&sh), (3000, Mode::Velocity as u8));
    // and once per period: the profile moves only at TRAJECTORY, by one
    // step each period
    let mut last = omega_star;
    for tick in 0..3 * DECIM_MED as u32 {
        let next = k.phase;
        k.on_scan(f, &sh);
        let now = k.medium.traj.omega_star_q16();
        if next == phase::TRAJECTORY {
            assert_eq!(now - last, omega_star, "tick {tick}");
            last = now;
        } else {
            assert_eq!(now, last, "tick {tick}");
        }
    }
}

/// The slow block keeps its cadence: once every DECIM_SLOW periods, at the
/// SLOW phase.
#[test]
fn the_slow_block_runs_every_32_periods() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    let f = frame(2000, BIAS);
    let mut runs = [0u32; 4];
    let mut n = 0;
    for tick in 0..4 * DECIM_SLOW as u32 * DECIM_MED as u32 {
        let next = k.phase;
        let before = k.medium.slow_periods;
        k.on_scan(f, &sh);
        if k.medium.slow_periods < before {
            assert_eq!(next, phase::SLOW, "tick {tick}");
            runs[n] = tick;
            n += 1;
        }
    }
    assert_eq!(n, 4);
    for w in runs.windows(2) {
        assert_eq!(w[1] - w[0], DECIM_SLOW as u32 * DECIM_MED as u32);
    }
}

#[test]
fn a_take_owns_therm_install_and_the_install_waits_a_period() {
    use crate::estimator::thermal::stage;
    let f = frame(2000, BIAS);
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    // past the first SLOW finish's period, which the thermometer sits out
    for _ in 0..=2 * DECIM_MED {
        k.on_scan(f, &sh);
    }
    write_config(&sh, |t| t.config.limits.stall_time_ticks = ms_ticks(20));
    run_to(&mut k, &sh, f, phase::SPARE_A);
    k.medium.thermal.request(stage::INSTALL | stage::NTC);
    k.on_scan(f, &sh);
    assert_eq!(k.cfg.limits.stall_time_ticks, 400, "the take ran");
    let pending = k.medium.thermal.pending();
    assert_ne!(pending & stage::INSTALL, 0, "the install waits");
    k.on_scan(f, &sh);
    assert_eq!(
        k.medium.thermal.pending() & stage::NTC,
        0,
        "SPARE_B is its own"
    );
    run_to(&mut k, &sh, f, phase::SPARE_A);
    k.on_scan(f, &sh);
    assert_eq!(
        k.medium.thermal.pending() & stage::INSTALL,
        0,
        "a period later"
    );
}

/// The SLOW block's stages land one per period: the due stage, the finish
/// a period later, then the thermometer's install and derate on the spare
/// phases of the period after that.
#[test]
fn the_slow_stages_take_one_period_each() {
    use crate::estimator::thermal::stage;
    let f = frame(2000, BIAS);
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    run_to(&mut k, &sh, f, phase::SLOW);
    k.on_scan(f, &sh);
    assert_eq!(k.medium.slow_periods, 0, "the due stage ran");
    run_to(&mut k, &sh, f, phase::SLOW);
    k.on_scan(f, &sh);
    assert_ne!(
        k.medium.thermal.pending() & stage::DERATE,
        0,
        "the finish asked"
    );
    run_to(&mut k, &sh, f, phase::CONTROL);
    assert_ne!(
        k.medium.thermal.pending() & stage::DERATE,
        0,
        "not in its period"
    );
    k.on_scan(f, &sh);
    run_to(&mut k, &sh, f, phase::CONTROL);
    assert_eq!(
        k.medium.thermal.pending() & stage::DERATE,
        0,
        "the next one"
    );
}
