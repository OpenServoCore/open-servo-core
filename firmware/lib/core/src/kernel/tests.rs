//! Kernel-level orchestration tests plus the closed-loop plant smoke suite.
//! The per-block tests carry the numeric precision; these pin the wiring:
//! gate, ack, dispatch shapes, publishes, and qualitative closed-loop
//! behavior against a crude integer plant.

use super::*;
use crate::estimator::OmegaSource;
use crate::regions::config::StallResponse;
use crate::traits::Sensors;
use crate::{RegionStorage, Shared};

const BIAS: u16 = 2048;
const ARR: u16 = 1200;
/// osc-dev-v006 board D rail scale (22k/10k tap over 20k/10k terminals).
const VBUS_SCALE_Q15: u32 = 34952;

// The chip-side const-eval for a 20 kHz FAST rate (MED 2 kHz).
const TIMING: KernelTiming = KernelTiming {
    pwm_arr: ARR,
    recip_arr_q24: (1 << bemf::RECIP_ARR_SHIFT) / ARR as u32,
    tick_hz: 20_000,
    dt_med_q32: ((1u64 << 32) / 2000) as u32,
    med_ticks_per_ms_q16: 2 << 16,
    vbus_scale_q15: VBUS_SCALE_Q15,
};

struct FakeSensors;
impl Sensors for FakeSensors {
    fn frame(&mut self) -> SensorFrame {
        SensorFrame::default()
    }
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
        c.loop_position.p_kp_q88 = 256;
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
        c.limits.oc_trip_counts = 2400;
        c.limits.oc_trip_ticks = 4;
        c.limits.openloop_decay = DecaySelect::Slow;
        c.limits.openloop_zero_brake = false;
        c.thermal.derate_start_cc = 8000;
        c.thermal.cutoff_cc = 10000;
        c.thermal.recover_cc = 9000;
        c.thermal.v_undervolt_counts = 2200;
        c.thermal.rtherm_i_min_counts = 300;
        c.thermal.rtherm_omega_max_cps = 400;
        c.fusion.l1_q016 = 16384;
        c.fusion.l2_q88 = 1024;
        // l3 * B is the tau_d loop gain: at the b_i-encoded B (1.0 below)
        // 0.5 cc per count is stable, 2.0 rails the filter
        c.fusion.l3_q88 = 128;
        c.fault_cfg.pos_error_counts = 400;
        c.fault_cfg.pos_error_time_ms = 500;
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
        t.telemetry.sensors.current_bias_counts = BIAS;
        t.control.lifecycle.torque_enable = false;
        t.control.lifecycle.mode = Mode::Position;
    });
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
        tick: 0,
    }
}

/// Ticks `f` until the applied duty equals `goal_duty`: OpenLoop duty above
/// the window floor slews there at `duty_limit::UP_Q15` per tick.
fn settle<T: TelStream>(k: &mut Kernel<FakeIo, T>, sh: &Shared, f: SensorFrame) {
    let goal = sh.table.with(|t| t.control.lifecycle.goal_duty);
    for _ in 0..1000 {
        if k.duty_q15 == goal {
            return;
        }
        k.on_tick(f, sh);
    }
    panic!("duty {} never reached the goal {goal}", k.duty_q15);
}

/// Ticks `f` until a MEDIUM pass has just run: the next tick opens a boxcar
/// half.
fn to_medium_boundary(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame) {
    while k.decim_med != 0 {
        k.on_tick(f, sh);
    }
}

/// Ticks `f` until an ident window publishes; returns its `agg_seq`. The
/// next tick opens a fresh window.
fn to_ident_boundary(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame) -> u16 {
    let seq = sh.table.with(|t| t.telemetry.ident.agg_seq);
    loop {
        k.on_tick(f, sh);
        let now = sh.table.with(|t| t.telemetry.ident.agg_seq);
        if now != seq {
            return now;
        }
    }
}

// --- Gate / ack / dispatch ------------------------------------------------

#[test]
fn torque_off_disables_and_estimators_still_track() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..40 {
        k.on_tick(frame(2500, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // boot seeded the fusion at the first measurement
    assert_eq!(k.fusion.theta_q16(), 2500 << 16);
    // the pot moves while disabled: the observer follows it anyway (the
    // live b_i bleed at the plant's saturated B settles the 1500-count
    // step in ~920 medium ticks, integer-sim verified)
    for _ in 0..12000 {
        k.on_tick(frame(1000, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    let err = (k.fusion.theta_q16() - (1000 << 16)).abs();
    assert!(err < 5 << 16, "theta_hat={}", k.fusion.theta_q16());
    // published while disabled
    sh.table.with(|t| {
        assert_eq!(t.telemetry.estimates.theta_hat_q16, k.fusion.theta_q16());
    });
}

#[test]
fn enable_edge_reseeds_without_transient() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..2000 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
    });
    // at-goal enable: every command in the first stretch is at rest or a
    // bounded nudge - no integrator kick, no profile teleport
    for _ in 0..50 {
        k.on_tick(frame(2000, BIAS), &sh);
        match last_cmd(&k) {
            MotorCmd::Coast | MotorCmd::Disabled => {}
            MotorCmd::Drive { duty, .. } => {
                assert!(duty.0.unsigned_abs() < 2000, "duty={}", duty.0)
            }
            MotorCmd::Brake => panic!("brake is never commanded"),
        }
    }
    assert_eq!(k.traj.theta_star_q16(), 2000 << 16);
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
        k.on_tick(frame(pos, BIAS), &sh);
    }
    for _ in 0..150 {
        k.on_tick(frame(1000, BIAS), &sh);
    }
    assert!(
        k.fusion.tau_d_counts().unsigned_abs() > 200,
        "precondition: hand motion railed tau_d, got {}",
        k.fusion.tau_d_counts()
    );
    // enable at rest: fusion reseeds, so the collision check never sees the
    // stale disturbance and STALL must not latch
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 0;
    });
    for _ in 0..200 {
        k.on_tick(frame(1500, BIAS), &sh);
    }
    assert_eq!(k.faults.mask(), 0, "stale tau_d re-latched STALL");
    assert!(k.fusion.tau_d_counts().unsigned_abs() < 50);
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
        k.on_tick(frame(4095, BIAS), &sh);
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
        k.on_tick(rev, &sh);
        if let MotorCmd::Drive { duty, .. } = last_cmd(&k) {
            drove |= duty.0 < 0;
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
        t.config.pos_limits.pos_max_soft_counts = 3000;
    });
    let mut k = kernel();
    for _ in 0..400 {
        k.on_tick(frame(PAST_WALL, BIAS), sh);
    }
    assert_eq!(k.i_band.hi, 0, "the endstop is closed before any permit");
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
    k.i_band.hi != 0
}

/// Ticks at the wall until the lease lapses; the ticks it took.
fn ticks_to_lapse(k: &mut Kernel<FakeIo>, sh: &Shared, max: u32) -> u32 {
    (1..=max)
        .find(|_| {
            k.on_tick(frame(PAST_WALL, BIAS), sh);
            !permit_live(k)
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
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
        assert!(!permit_live(&k), "a lapsed lease stays lapsed");
    }
}

#[test]
fn permit_rewrite_extends_the_lease() {
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    write_permit(&sh);
    for _ in 0..SEC * 8 / 10 {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(permit_live(&k));
    write_permit(&sh);
    for _ in 0..SEC * 8 / 10 {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
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
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(permit_live(&k));
    sh.table
        .with_mut(|t| t.control.lifecycle.stall_permit = false);
    for _ in 0..DECIM_MED {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(!permit_live(&k), "false revokes inside a MEDIUM tick");
}

#[test]
fn torque_off_drops_the_permit() {
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    write_permit(&sh);
    for _ in 0..SEC / 10 {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
    }
    assert!(permit_live(&k));
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..2 * DECIM_MED {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
    }
    // re-enabled well inside the second the grant had left
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..SEC {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
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
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
    }
    write_permit(&sh);
    // torque back on before the next MEDIUM pass: the request alone, never
    // rewritten under torque, is no grant
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..2 * SEC {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
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
        k.on_tick(frame(2000, BIAS + 290), &sh);
    }
    for _ in 0..4 {
        to_medium_boundary(&mut k, &sh, frame(2000, BIAS + 290));
        assert_eq!(limit_flags(&sh), flag::CEILING);
        k.on_tick(frame(2000, BIAS + 290), &sh);
    }
    assert!(k.duty_q15 < 8000, "governed");

    // yield: Current mode pinned at the limit on a still shaft folds, then
    // a goal under the fold leaves only the fold standing
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_current = 1200;
        t.config.limits.stall_release_counts = 0;
    });
    let mut k = kernel();
    for _ in 0..2_000 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::CEILING | flag::YIELD);
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = 100);
    for _ in 0..2 * DECIM_MED {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::YIELD);
    assert_eq!(k.faults.mask(), 0);

    // endstop: at rest past the top soft wall
    let sh = Shared::new();
    let mut k = permit_rig(&sh);
    assert_eq!(limit_flags(&sh), flag::ENDSTOP);

    // permit: the lease drops the endstop, so it stands alone
    write_permit(&sh);
    for _ in 0..DECIM_MED {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
    }
    assert_eq!(limit_flags(&sh), flag::PERMIT);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..DECIM_MED {
        k.on_tick(frame(PAST_WALL, BIAS), &sh);
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
        k.on_tick(sagged, &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_UNDER_VOLT, "sag latches");
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // the tap keeps sampling with the bridge off: an ack against a still
    // sagged rail re-latches, an ack against a recovered rail sticks
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..400 {
        k.on_tick(sagged, &sh);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..2000 {
        k.on_tick(sagged, &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_UNDER_VOLT, "sag re-latches");
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..400 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..2000 {
        k.on_tick(frame(2000, BIAS), &sh);
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
    // tick 1 drives (windows still invalid: previous duty was 0)
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // 4 consecutive valid over-trip samples latch on the 4th
    for _ in 0..3 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
        assert_eq!(k.faults.mask(), 0);
    }
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // torque_enable still set: stays disabled, publish lands at medium
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
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
    for _ in 0..8 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    // ack: drop then raise torque_enable
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), 0);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // the overcurrent persists: a fresh window re-latches
    for _ in 0..5 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
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
    k.on_tick(frame(2000, BIAS + 3000), &sh); // warmup: invalid window
    for _ in 0..3 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
    }
    // one clean sample re-arms; three more over-samples do not latch
    k.on_tick(frame(2000, BIAS + 100), &sh);
    for _ in 0..3 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), 0);
    // the 4th consecutive does
    k.on_tick(frame(2000, BIAS + 3000), &sh);
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
        t.config.limits.openloop_decay = DecaySelect::Fast;
    });
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS));
    match last_cmd(&k) {
        MotorCmd::Drive { duty, decay } => {
            assert_eq!(duty.0, 8000);
            assert!(matches!(decay, DecayMode::Fast));
        }
        other => panic!("expected Drive, got {other:?}"),
    }
    // duty_max clamps the passthrough, at once
    sh.table.with_mut(|t| {
        t.config.loop_current.duty_max_q15 = 5000;
        t.control.lifecycle.goal_duty = 8000;
    });
    k.on_tick(frame(2000, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { duty, .. } => assert_eq!(duty.0, 5000),
        other => panic!("expected Drive, got {other:?}"),
    }
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
    k.on_tick(frame(2000, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { duty, decay } => {
            assert_eq!(duty.0, 0);
            assert!(matches!(decay, DecayMode::Slow));
        }
        other => panic!("expected Drive, got {other:?}"),
    }
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
    k.on_tick(frame(2000, BIAS), &sh);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(k.duty_q15, 0);
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
        k.on_tick(frame(4095, BIAS), &sh);
    }
    assert!(
        matches!(last_cmd(&k), MotorCmd::Brake),
        "expected brake at the wall, got {:?}",
        last_cmd(&k)
    );
    assert_eq!(k.duty_q15, 0);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
    k.on_tick(frame(4095, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { duty, .. } => assert!(duty.0 < 0, "duty {}", duty.0),
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
        MotorCmd::Drive { duty, .. } => assert_eq!(duty.0, 8000),
        other => panic!("expected Drive, got {other:?}"),
    }
}

fn written_duty(k: &Kernel<FakeIo>) -> i16 {
    match last_cmd(k) {
        MotorCmd::Drive { duty, .. } => duty.0,
        other => panic!("expected Drive, got {other:?}"),
    }
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
        k.on_tick(frame(4095, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(k.duty_q15, 0);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
    for _ in 0..400 {
        k.on_tick(frame(4095, BIAS), &sh);
    }
    assert_eq!(k.duty_q15, -8000);
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
    // past the next medium publish, which reports the previous tick's duty
    for _ in 0..=DECIM_MED {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    assert_eq!(written_duty(&k), -8000);
    assert_eq!(k.duty_q15, 8000);
    sh.table
        .with(|t| assert_eq!(t.telemetry.estimates.duty_applied_q15, 8000));
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
        k.on_tick(frame(2000, BIAS), &sh);
    }
    assert!(k.duty_q15 > 0, "logical duty={}", k.duty_q15);
    assert_eq!(written_duty(&k), -k.duty_q15);
}

#[test]
fn reversed_polarity_negates_vdiff() {
    let sh = Shared::new();
    seed(&sh);
    reversed(&sh, Mode::OpenLoop);
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    let mut k = kernel();
    // the second tick samples the window the first tick's duty drove
    k.on_tick(frame(2000, BIAS), &sh);
    k.on_tick(frame(2000, BIAS), &sh);
    assert_eq!(k.vdiff_last, -(3000 - 40));
    sh.table.with_mut(|t| t.config.limits.drive_polarity = true);
    k.on_tick(frame(2000, BIAS), &sh);
    assert_eq!(k.vdiff_last, 3000 - 40);
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
    k.on_tick(frame(2000, BIAS), &sh);
    assert_eq!(k.i_ref_cc, 1200);
    assert!(matches!(
        last_cmd(&k),
        MotorCmd::Drive {
            decay: DecayMode::Slow,
            ..
        }
    ));
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = -5000);
    k.on_tick(frame(2000, BIAS), &sh);
    assert_eq!(k.i_ref_cc, -1200);
}

#[test]
fn position_error_latches_after_persistence() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3500;
        t.config.fault_cfg.pos_error_counts = 100;
        t.config.fault_cfg.pos_error_time_ms = 10; // 20 medium ticks
        // keep the stall path out of this test
        t.config.limits.stall_time_ms = 60000;
    });
    let mut k = kernel();
    // pot frozen at 500 while the profile marches away
    for _ in 0..4000 {
        k.on_tick(frame(500, BIAS), &sh);
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
        k.on_tick(frame(1234, 2100), &sh);
    }
    sh.table.with(|t| {
        assert_eq!(t.telemetry.sensors.pos, 1234);
        assert_eq!(t.telemetry.sensors.current, 2100);
        assert_eq!(t.telemetry.sensors.vmotor_a, 3000);
        assert_eq!(t.telemetry.estimates.theta_hat_q16, k.fusion.theta_q16());
        assert_eq!(t.telemetry.estimates.omega_hat_cps, k.fusion.omega_q16());
        assert_eq!(t.telemetry.estimates.i_lim_counts, 1200);
        assert_eq!(t.telemetry.estimates.duty_applied_q15, 0);
        assert_eq!(t.telemetry.estimates.vbus_counts, 3000);
        assert_eq!(t.telemetry.mode.mode_active, Mode::Position as u8);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_NONE);
        assert_eq!(t.telemetry.common.fault_flags, 0);
    });
}

fn published_bias(sh: &Shared) -> u16 {
    sh.table.with(|t| t.telemetry.sensors.current_bias_counts)
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
    // tick 1 already runs on the boot seed
    k.on_tick(shifted(), &sh);
    assert_eq!(published_bias(&sh), BIAS);
    // Disabled: no PWM, no brake phase
    for _ in 0..300 {
        k.on_tick(shifted(), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    assert_eq!(published_bias(&sh), BIAS);
    // Fast decay: the trough IS the drive window
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        t.config.limits.openloop_decay = DecaySelect::Fast;
    });
    for _ in 0..300 {
        k.on_tick(shifted(), &sh);
    }
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
    sh.table
        .with_mut(|t| t.config.limits.openloop_decay = DecaySelect::Slow);
    for _ in 0..300 {
        k.on_tick(shifted(), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { duty, .. } if duty.0 == i16::MAX));
    assert_eq!(published_bias(&sh), BIAS);
    // Brake as a command
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_duty = 0;
        t.config.limits.openloop_zero_brake = true;
    });
    for _ in 0..300 {
        k.on_tick(shifted(), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(published_bias(&sh), BIAS);
    // a partial Slow window: the trough is the brake phase, the tracker follows
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    for _ in 0..1500 {
        k.on_tick(shifted(), &sh);
    }
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
    // offset walks 100 counts over 4000 ticks (1 count per 40 ticks, well
    // under the 128-tick tracker time constant); the true current is a
    // constant 300 counts above it
    let mut b = BIAS;
    for n in 0..4000u16 {
        b = BIAS + n / 40;
        let mut f = frame(2000, b + 300);
        f.current_trough = b;
        k.on_tick(f, &sh);
        if n >= 300 {
            let i = k.i_meas_last as i32;
            assert!((i - 300).abs() <= 5, "tick {n}: i_meas {i}, bias {b}");
        }
    }
    assert!(
        published_bias(&sh).abs_diff(b) <= 5,
        "{}",
        published_bias(&sh)
    );
    // the untracked boot value would have read 400 here
    assert_eq!(b, BIAS + 99);
}

// --- Ident aggregates -----------------------------------------------------

fn ident_setup(sh: &Shared) {
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

#[test]
fn ident_window_pins_aggregates() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    // the windows up to the goal duty are slew-mixed; spend them
    settle(&mut k, &sh, frame(2000, BIAS + 100));
    let seq = to_ident_boundary(&mut k, &sh, frame(2000, BIAS + 100));
    // next window: 8 ticks at +100, 4 at +300, 4 at -60, all windows valid
    for _ in 0..8 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
    }
    // seq holds mid-window: publish only at the /16 boundary
    sh.table
        .with(|t| assert_eq!(t.telemetry.ident.agg_seq, seq));
    for _ in 0..4 {
        k.on_tick(frame(2000, BIAS + 300), &sh);
    }
    for _ in 0..4 {
        k.on_tick(frame(2000, BIAS - 60), &sh);
    }
    sh.table.with(|t| {
        let d = &t.telemetry.ident;
        // (8*100 + 4*300 + 4*-60) >> 4 = 1760/16 = 110
        assert_eq!(d.i_mean_counts, 110);
        assert_eq!(d.i_min_counts, -60);
        assert_eq!(d.i_max_counts, 300);
        assert_eq!(d.vdiff_mean, 2960); // va 3000 - vb 40, every tick
        assert_eq!(d.duty_mean_q15, 8000);
        assert_eq!(d.agg_seq, seq + 1);
    });
}

#[test]
fn ident_invalid_ticks_hold_last_valid() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS + 200));
    let seq = to_ident_boundary(&mut k, &sh, frame(2000, BIAS + 200));
    // disable: windows go invalid one tick later (the first tick still
    // measures the period the last drive command drove)
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..16 {
        k.on_tick(frame(2000, BIAS + 200), &sh);
    }
    sh.table.with(|t| {
        let d = &t.telemetry.ident;
        // 15 invalid ticks held the last valid i/vdiff: means stay put
        assert_eq!(d.i_mean_counts, 200);
        assert_eq!(d.i_min_counts, 200);
        assert_eq!(d.i_max_counts, 200);
        assert_eq!(d.vdiff_mean, 2960);
        // duty is per-tick truth: 8000 on the first tick only -> 8000>>4 = 500
        assert_eq!(d.duty_mean_q15, 500);
        assert_eq!(d.agg_seq, seq + 1);
    });
}

#[test]
fn ident_accumulators_reset_between_windows() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    // windows 1 (boot-mixed) and 2 at a big current
    for _ in 0..32 {
        k.on_tick(frame(2000, BIAS + 1000), &sh);
    }
    sh.table
        .with(|t| assert_eq!(t.telemetry.ident.i_max_counts, 1000));
    // window 3: 15 ticks at 0, one at -5; window 2's 1000 must not leak
    for _ in 0..15 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    k.on_tick(frame(2000, BIAS - 5), &sh);
    sh.table.with(|t| {
        let d = &t.telemetry.ident;
        assert_eq!(d.i_max_counts, 0);
        assert_eq!(d.i_min_counts, -5);
        // sum -5 >> 4: arithmetic shift floors to -1
        assert_eq!(d.i_mean_counts, -1);
        assert_eq!(d.agg_seq, 3);
    });
}

// --- Back-EMF boxcar ------------------------------------------------------

#[test]
fn bemf_boxcar_lands_on_the_closed_form_after_20_ticks() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    // from the medium boundary after the slew every tick measures duty
    // 8000: drive_ticks 293, vdiff 2960, i 100. The half closed 10 ticks
    // later is the first clean one, 20 ticks pairs it with a second.
    settle(&mut k, &sh, frame(2000, BIAS + 100));
    to_medium_boundary(&mut k, &sh, frame(2000, BIAS + 100));
    for _ in 0..20 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
    }
    // closed form: (293 * 2960 / 1200 - 2.0 * 100) * 16 c/s per vcount
    let ticks = window::drive_ticks(8000, ARR) as i64;
    let v_sum = (bemf::BOXCAR_TICKS as i64 * ticks * 2960 * TIMING.recip_arr_q24 as i64) >> 24;
    let r_sum = (8192i64 * bemf::BOXCAR_TICKS as i64 * 100) >> 12;
    let expect = ((v_sum - r_sum) * 16) / bemf::BOXCAR_TICKS as i64;
    let got = sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps) as i64;
    assert!((got - expect).abs() <= 1, "got {got} expect {expect}");
    assert_eq!(got, 8363, "pin");
    // torque off: the first tick still measures the last drive, the rest
    // are sub-floor, so the next half closed voids and the publish drops to
    // 0 with it
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..9 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
    }
    assert_eq!(
        sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps),
        8363
    );
    k.on_tick(frame(2000, BIAS + 100), &sh);
    assert_eq!(sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps), 0);
}

#[test]
fn velocity_feedback_switches_to_the_bemf_and_back() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    let published = |sh: &Shared| {
        sh.table.with(|t| {
            (
                t.telemetry.estimates.omega_hat_cps,
                t.telemetry.mode.omega_hat_src,
            )
        })
    };
    // the slew starts at the window floor, so valid boxcars close at ticks
    // 20, 30, 40, 50: the fourth flips the source; until then omega_hat is
    // the observer's omega
    for _ in 0..50 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
        assert_eq!(published(&sh), (k.fusion.omega_q16(), 0));
    }
    k.on_tick(frame(2000, BIAS + 100), &sh);
    let bemf = sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps) as i32;
    assert!(bemf > 0);
    assert_eq!(published(&sh), (bemf << 16, 1));
    assert_eq!(k.omega_sw.source(), OmegaSource::Bemf);
    // torque off: tick 51 still measures the last drive; the half closed
    // at tick 60 voids and the source rides the held boxcar through that
    // one result, then falls back at tick 70
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
    }
    assert_eq!(published(&sh), (bemf << 16, 1));
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
    }
    assert_eq!(published(&sh), (k.fusion.omega_q16(), 0));
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
        k.on_tick(frame(2000, BIAS), &sh);
    }
    assert_eq!(k.traj.omega_star_q16(), 1600 << 16);
    // ke 0.0625 vcounts per c/s * 1600 c/s = 100 vcounts on a 3000 rail
    let u_ff = q_mul(1600 << 16, 256, 28);
    assert_eq!(u_ff, 100);
    let expect = q_mul(u_ff, k.vbus.recip_q15() as i32, 15) as i16;
    assert_eq!(k.duty_q15, expect);
    assert!((expect as i32 - 1092).abs() <= 1, "duty={expect}");
    // Current mode: no profile, no feed - even with the shaft spinning
    // (the pot observer would read ~2000 c/s here)
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 0;
    });
    for n in 0..2000u16 {
        k.on_tick(frame(1000 + n / 10, BIAS), &sh);
    }
    assert!(
        k.fusion.omega_q16() > 1000 << 16,
        "pot omega {}",
        k.fusion.omega_q16() >> 16
    );
    assert_eq!(k.duty_q15, 0);
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
            tick: 0,
        }
    }

    fn pos(&self) -> i32 {
        (self.theta_q16 >> 16) as i32
    }
}

fn run_plant(k: &mut Kernel<FakeIo>, sh: &Shared, plant: &mut Plant, ticks: u32) {
    for _ in 0..ticks {
        let f = plant.step(k.duty_q15);
        k.on_tick(f, sh);
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
    assert_eq!(k.traj.theta_star_q16(), 3000 << 16, "profile landed");
    // trailing window: position parked, motor overwhelmingly coasting
    let (mut lo, mut hi, mut coast) = (i32::MAX, i32::MIN, 0u32);
    for _ in 0..2000 {
        let f = plant.step(k.duty_q15);
        k.on_tick(f, &sh);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
        if matches!(last_cmd(&k), MotorCmd::Coast) {
            coast += 1;
        }
    }
    assert!(hi - lo <= 2, "limit cycle: spread {}", hi - lo);
    assert!(coast >= 1500, "coast ticks {coast}");
}

#[test]
fn velocity_mode_tracks_a_ramp() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        // plenty of travel so the pot never rails
        t.config.pos_limits.pos_max_soft_counts = 1_000_000;
    });
    let mut k = kernel();
    let mut plant = Plant::new(100);
    for seg in 1..=4i32 {
        sh.table
            .with_mut(|t| t.control.lifecycle.goal_velocity = 500 * seg);
        run_plant(&mut k, &sh, &mut plant, 4000); // 200 ms per segment
    }
    assert_eq!(k.faults.mask(), 0);
    let omega_hat = k.fusion.omega_q16() >> 16;
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
    // wind-up is integrator-paced (~230 ms to the full limit); the stall
    // verdict then folds i_lim to yield, the weaker drive lets tau_d
    // bleed below release, and the fold opens into another probe. That
    // probe-the-wall cycle (bench: re-trips through grip-strength dips)
    // makes any single-phase pin flaky - assert the cycle's invariants
    // across a run instead.
    let mut saw_full = false;
    let mut saw_fold = false;
    for _ in 0..24_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.i_ref_cc == 1200 {
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
    assert!(matches!(last_cmd(&k), MotorCmd::Coast), "never parked");
    assert_eq!(k.faults.mask(), 0);
    // a goal write releases the park (omega_star != 0 drops hold); the whole
    // commanded move must run without a single Coast, then it re-parks once
    // the profile lands inside the deadband
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = 2100);
    let mut saw_move = false;
    let mut reparked = false;
    for _ in 0..20_000u32 {
        run_plant(&mut k, &sh, &mut plant, 1);
        let moving = k.traj.omega_star_q16() != 0;
        if moving {
            saw_move = true;
            assert!(!matches!(last_cmd(&k), MotorCmd::Coast), "coast mid-move");
        }
        if saw_move && !moving && matches!(last_cmd(&k), MotorCmd::Coast) {
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
        woke |= !matches!(last_cmd(&k), MotorCmd::Coast);
    }
    assert!(woke, "push never woke the hold");
    let mut coast = 0u32;
    for _ in 0..30_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if matches!(last_cmd(&k), MotorCmd::Coast) {
            coast += 1;
        }
    }
    assert!(
        coast > 15_000,
        "never re-parked after the push: coast {coast}"
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
    assert!(matches!(last_cmd(&k), MotorCmd::Coast), "never parked");
    // parked hold under +-3 counts of pot noise: gating on position rest
    // (not the noisy omega_hat) keeps the park engaged - a wake per blip is
    // the shake fix2 removes
    let mut rng: u32 = 0xdead_beef;
    let (mut coast, mut lo, mut hi) = (0u32, i32::MAX, i32::MIN);
    for _ in 0..40_000 {
        let mut f = plant.step(k.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let n = ((rng >> 24) as i32 % 7) - 3;
        f.pos = (f.pos as i32 + n).clamp(0, 4095) as u16;
        k.on_tick(f, &sh);
        if matches!(last_cmd(&k), MotorCmd::Coast) {
            coast += 1;
        }
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(hi - lo <= 4, "hold limit cycle: spread {}", hi - lo);
    assert!(coast >= 36_000, "coast ticks {coast} of 40000");
}

#[test]
fn current_mode_endstop_unwinds_duty_to_zero() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 300;
        t.config.pos_limits.pos_max_soft_counts = 3000;
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
    assert_eq!(k.duty_q15, 0, "duty froze above zero at the wall");
    let mut zero = 0u32;
    for _ in 0..2000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.duty_q15 == 0 {
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
        t.config.pos_limits.pos_max_soft_counts = 3000;
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
        star_max = star_max.max(k.traj.omega_star_q16());
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
        t.config.pos_limits.pos_max_soft_counts = 3000;
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
    assert_eq!(k.duty_q15, 0, "outbound duty still firing at the wall");
    // retreat duty applies untouched and drives back off the wall
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
    run_plant(&mut k, &sh, &mut plant, 2_000);
    assert_eq!(k.duty_q15, -8000, "retreat from the wall blocked");
    // mirrored at the min wall, crossed in free flight like the top one
    sh.table
        .with_mut(|t| t.config.pos_limits.pos_min_soft_counts = 500);
    run_plant(&mut k, &sh, &mut plant, 25_000);
    assert!(
        plant.pos() >= 450 && plant.pos() <= 500,
        "min wall not respected: {}",
        plant.pos()
    );
    assert_eq!(k.duty_q15, 0, "outbound duty still firing at the min wall");
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    run_plant(&mut k, &sh, &mut plant, 5_000);
    assert_eq!(k.duty_q15, 8000, "retreat from the min wall blocked");
    assert!(plant.pos() > 600, "never drove off the min wall");
    assert_eq!(k.faults.mask(), 0);
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
    // ~10% of ISR invocations vanish; the plant keeps running on the stale
    // duty (tick-indexed contract: dilation, never compensation)
    let mut rng: u32 = 0x1357_9bdf;
    for _ in 0..40_000 {
        let f = plant.step(k.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        if !rng.is_multiple_of(10) {
            k.on_tick(f, &sh);
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
        let f = plant.step(k.duty_q15);
        k.on_tick(f, &sh);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
    }
    assert!(hi - lo <= 4, "limit cycle: spread {}", hi - lo);
}

// --- TEL stream ------------------------------------------------------------

#[derive(Default)]
struct RecTel {
    active: bool,
    samples: heapless::Vec<crate::tel::TelSample, 64>,
}
impl crate::tel::TelStream for RecTel {
    fn active(&self) -> bool {
        self.active
    }
    fn on_tick(&mut self, sample: &crate::tel::TelSample) {
        let _ = self.samples.push(*sample);
    }
}

#[test]
fn tel_stream_gated_by_sink_active() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        RecTel::default(),
        TIMING,
    );
    // sink inactive: no emission, torque off or driving
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    settle(&mut k, &sh, frame(2100, BIAS + 40));
    assert!(k.tel.samples.is_empty());

    // sink active: one sample per tick
    k.tel.active = true;
    for _ in 0..20 {
        k.on_tick(frame(2100, BIAS + 40), &sh);
    }
    assert_eq!(k.tel.samples.len(), 20);
    let s = k.tel.samples.last().unwrap();
    assert_eq!(s.pos, 2100);
    assert_eq!(s.pos_lin_q4, 2100 << 4, "identity: no table live");
    assert_eq!(s.current_trough, BIAS);
    // previous-tick alignment: the sampled duty is the command whose window
    // this frame measured
    assert_eq!(s.duty_q15, 8000);
    // duty 8000/32767 of ARR is far above the 100-tick window floors
    assert!(s.window_valid);
    assert_eq!(s.current, 40);
    assert!(!s.fault);
}
