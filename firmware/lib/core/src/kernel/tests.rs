//! Kernel-level orchestration tests plus the closed-loop plant smoke suite.
//! The per-block tests carry the numeric precision; these pin the wiring:
//! gate, ack, dispatch shapes, publishes, and qualitative closed-loop
//! behavior against a crude integer plant.

use super::phases::fast;
use super::phases::loops::{self, duty_limit, faults, limits};
use super::*;
use crate::budget::Events;
use crate::estimator::{FusionObs, OmegaBlend, bemf, bias, omega_blend, window};
use crate::math::q_mul;
use crate::regions::config::{StallResponse, ms_ticks};
use crate::regions::control::Mode;
use crate::regions::telemetry::tick_flag;
use crate::traits::{ControlIo, Motor, MotorCmd, Sensors, TelStream};
use crate::{ControlTable, RegionStorage, SensorFrame, Shared, TelemetryTick};
use crate::{adc, pos_lut};

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
    bemf_min_ticks: 0,
    bridge_min_ticks: 27,
    i_settle_gain: window::SettleGain::UNITY,
    // the development board's sense chain
    brush_dev_max: bemf::brush_dev_max(60, 14_884, 3300),
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
        c.limits.oc_trip_counts = 2400;
        c.limits.oc_trip_ticks = 4;
        c.limits.openloop_zero_brake = false;
        c.limits.thermal_tau_s = 4;
        c.thermal.cutoff_cc = 10000;
        c.thermal.v_undervolt_counts = 2200;
        // 1200 counts over 200 vcounts: 6 counts per vcount
        c.fault_cfg.rail_fold_q8 = 6 << 8;
        c.fusion.l1_q016 = 16384;
        c.fusion.l2_q88 = 1024;
        // l3 * B is the tau_d loop gain
        c.fusion.l3_q88 = 128;
        c.fault_cfg.pos_error_counts = 400;
        c.fault_cfg.pos_error_time_ticks = ms_ticks(500);
        c.fault_cfg.sensor_delta_max = 256;
        c.fault_cfg.sensor_bad_count = 4;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = 100;
        cal.sense.v_window_min_ticks = 100;
        cal.motor.ke_vpc_q = 4096 >> PLANT_KE_SHIFT;
        cal.motor.r_q12 = 4096 << PLANT_R_SHIFT;
        cal.motor.recip_ke_q = 1024 << PLANT_KE_SHIFT;
        cal.motor.b_i_q313 = PLANT_B_I_Q313;
        cal.motor.fric_fc_counts = PLANT_FRIC_FC_COUNTS;
        cal.motor.fric_fv_q016 = PLANT_FRIC_FV_Q016;
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

/// The trough scan lands in the brake phase, so the shunt reads its offset
/// there.
fn frame(pos: u16, current: u16) -> SensorFrame {
    SensorFrame {
        pos,
        current,
        current_trough: BIAS,
        vmotor_a: 3000,
        vmotor_a_trough: 3000,
        vmotor_b: 40,
        vmotor_b_trough: 40,
        vref_raw: 1489,
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
/// the current reference first land, and the tick after it, whose write
/// takes them first: a servo enabled from boot drives from the last of
/// these ticks on.
fn to_first_command(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame) {
    for _ in 0..=phase::VELOCITY + 1 {
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

// --- Bus dead-man -----------------------------------------------------------

/// OpenLoop under torque from the boot tick, `bus_deadman_ticks` at `field`.
fn deadman_seed(sh: &Shared, field: u16) {
    seed(sh);
    sh.table.with_mut(|t| {
        let l = &mut t.control.lifecycle;
        l.torque_enable = true;
        l.mode = Mode::OpenLoop;
        l.goal_duty = 8000;
        l.bus_deadman_ticks = field;
    });
}

/// The production half's guard: the countdown steps DECIM_MED ticks on
/// each CONTROL phase after the enable edge's, so a field of 400 lapses on
/// the 40th CONTROL after the edge, tick 400 from an enable at the boot
/// tick.
mod deadman {
    use super::*;

    /// A TEL stream the test switches on and off.
    #[derive(Default)]
    struct StreamTel {
        streaming: bool,
    }
    impl TelStream for StreamTel {
        fn on_publish(&mut self, _: impl FnOnce() -> bool) {}
        fn streaming(&self) -> bool {
            self.streaming
        }
    }

    type K = Kernel<FakeIo, StreamTel>;

    fn rig(sh: &Shared, field: u16) -> K {
        deadman_seed(sh, field);
        Kernel::with_tel(
            FakeIo {
                sensors: FakeSensors,
                motor: FakeMotor { last: None },
            },
            StreamTel::default(),
            TIMING,
        )
    }

    fn scan(k: &mut K, sh: &Shared, f: SensorFrame, n: u32) {
        for _ in 0..n {
            k.on_scan(f, sh);
        }
    }

    fn drives(k: &K) -> bool {
        k.faults.mask() == 0 && matches!(k.io.motor.last, Some(MotorCmd::Drive { .. }))
    }

    fn lapsed(k: &K) -> bool {
        k.faults.mask() == faults::BIT_DEADMAN
            && matches!(k.io.motor.last, Some(MotorCmd::Disabled))
    }

    /// The tick the countdown runs out on: CONTROL raises the lapse after
    /// that tick's write, and the next write is off.
    fn lapses(k: &mut K, sh: &Shared, f: SensorFrame) {
        k.on_scan(f, sh);
        assert_eq!(k.faults.mask(), faults::BIT_DEADMAN);
        assert!(matches!(k.io.motor.last, Some(MotorCmd::Drive { .. })));
        k.on_scan(f, sh);
        assert!(lapsed(k));
    }

    /// Bus silence under torque latches `deadman` and the drive goes off,
    /// published as code 9 on bit 7; it stays off under torque until the
    /// enable edge acks it, which restarts the countdown from the field.
    #[test]
    fn the_deadman_lapses_bus_silence_and_the_enable_edge_acks() {
        let f = frame(2000, BIAS);
        let sh = Shared::new();
        let mut k = rig(&sh, 400);
        scan(&mut k, &sh, f, 400);
        assert!(drives(&k));
        lapses(&mut k, &sh, f);
        run_to(&mut k, &sh, f, phase::SPARE_A);
        sh.table.with(|t| {
            assert_eq!(t.telemetry.mode.fault_code, faults::CODE_DEADMAN);
            assert_eq!(t.telemetry.common.fault_flags, faults::BIT_DEADMAN);
        });
        assert_eq!(faults::CODE_DEADMAN, 9);
        assert_eq!(faults::BIT_DEADMAN, 1 << 7);
        for _ in 0..1000 {
            k.on_scan(f, &sh);
            assert!(lapsed(&k), "the lapse holds under torque");
        }

        // torque off, then the enable edge at the CONTROL of tick 1420:
        // the 40th CONTROL after it is tick 1820
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        run_to(&mut k, &sh, f, phase::SLOW);
        assert!(matches!(k.io.motor.last, Some(MotorCmd::Disabled)));
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        run_to(&mut k, &sh, f, phase::CONTROL);
        assert_eq!(k.faults.mask(), faults::BIT_DEADMAN);
        scan(&mut k, &sh, f, 400);
        assert!(drives(&k), "the ack clears the fault");
        lapses(&mut k, &sh, f);
    }

    /// An instruction dispatched ahead of tick 300 reloads: the CONTROL at
    /// 300 starts 400 afresh, so tick 700 is the lapse.
    #[test]
    fn the_deadman_reloads_on_a_dispatched_instruction() {
        let f = frame(2000, BIAS);
        let sh = Shared::new();
        let mut k = rig(&sh, 400);
        scan(&mut k, &sh, f, 300);
        sh.bus_touch();
        scan(&mut k, &sh, f, 400);
        assert!(drives(&k));
        lapses(&mut k, &sh, f);
    }

    /// A stream from tick 100 holds the 310 ticks left; released at 1100,
    /// they run out on the CONTROL of tick 1400.
    #[test]
    fn the_deadman_holds_while_a_burst_streams() {
        let f = frame(2000, BIAS);
        let sh = Shared::new();
        let mut k = rig(&sh, 400);
        scan(&mut k, &sh, f, 100);
        k.tel.streaming = true;
        scan(&mut k, &sh, f, 1000);
        assert!(drives(&k));
        k.tel.streaming = false;
        scan(&mut k, &sh, f, 300);
        assert!(drives(&k));
        lapses(&mut k, &sh, f);
    }

    /// From 0 at the enable edge, a write of 400 landing with its touch at
    /// tick 2 is carried and loaded by the CONTROL at 10, so the silence
    /// lapses at tick 410; a zero field never trips.
    #[test]
    fn the_deadman_takes_a_field_written_under_torque() {
        let f = frame(2000, BIAS);
        let sh = Shared::new();
        let mut k = rig(&sh, 0);
        scan(&mut k, &sh, f, 5000);
        assert!(drives(&k), "a zero field never trips");

        let sh = Shared::new();
        let mut k = rig(&sh, 0);
        scan(&mut k, &sh, f, 2);
        sh.table
            .with_mut(|t| t.control.lifecycle.bus_deadman_ticks = 400);
        sh.bus_touch();
        scan(&mut k, &sh, f, 2 * DECIM_MED as u32);
        assert!(drives(&k), "a field written under torque trips nothing");
        scan(&mut k, &sh, f, 410 - 22);
        assert!(drives(&k));
        lapses(&mut k, &sh, f);
    }

    /// The loop-margin stimulus, built with `loop-margin`.
    #[cfg(feature = "loop-margin")]
    mod stimulus {
        use super::*;

        /// The cruise, c/s, and the stimulus the rig sets: 2 c/s per period
        /// over half-periods of 10, a 10 c/s peak.
        const CRUISE: i32 = 1000;
        const SLOPE: i32 = 2;
        const HALF: u16 = 10;
        /// Periods in one cycle of the triangle.
        const CYCLE: usize = 2 * HALF as usize;

        /// Velocity mode at the cruise from the boot tick, a burst streaming
        /// under a loaded dead-man, the velocity loop's gains 0 so the current
        /// reference is the feed-forward alone and no limit binds, then `cfg`;
        /// run to the cruise.
        fn inject_rig(sh: &Shared, cfg: impl FnOnce(&mut ControlTable)) -> K {
            let mut k = rig(sh, u16::MAX);
            sh.table.with_mut(|t| {
                let l = &mut t.control.lifecycle;
                l.mode = Mode::Velocity;
                l.goal_velocity = CRUISE;
                t.config.loop_velocity.v_kp_q88 = 0;
                t.config.loop_velocity.v_ki_q412 = 0;
                t.control.inject.inject_accel_q88 = (SLOPE << 8) as u16;
                t.control.inject.inject_half_periods = HALF;
                cfg(t);
            });
            k.tel.streaming = true;
            let f = frame(2000, BIAS);
            for _ in 0..40 {
                period(&mut k, sh, f);
            }
            k
        }

        /// One medium period to its PUBLISH; the reference less the profile's
        /// speed, csQ16.
        fn period(k: &mut K, sh: &Shared, f: SensorFrame) -> i32 {
            k.on_scan(f, sh);
            run_to(k, sh, f, phase::SPARE_A);
            let u = sh.table.with(|t| t.telemetry_ext.inject.omega_ref_cps);
            assert_eq!(u, k.medium.omega_ref_q16, "PUBLISH carries the reference");
            u - k.medium.traj.omega_star_q16()
        }

        fn injected<const N: usize>(k: &mut K, sh: &Shared) -> [i32; N] {
            core::array::from_fn(|_| period(k, sh, frame(2000, BIAS)))
        }

        /// The stimulus is the triangle on the cruise: steps of the slope every
        /// period, turning at +/-slope x half / 2, a cycle of two halves, mean
        /// 0; the profile stays at the cruise under it.
        #[test]
        fn the_stimulus_rides_the_velocity_reference_as_a_triangle() {
            let sh = Shared::new();
            let mut k = inject_rig(&sh, |_| {});
            assert_eq!(k.medium.traj.omega_star_q16(), CRUISE << 16);
            let d = injected::<{ 2 * CYCLE }>(&mut k, &sh);
            let peak = (SLOPE * HALF as i32 / 2) << 16;
            assert_eq!(d.iter().max(), Some(&peak));
            assert_eq!(d.iter().min(), Some(&-peak));
            assert!(d.windows(2).all(|w| (w[1] - w[0]).abs() == SLOPE << 16));
            assert!(d.windows(CYCLE + 1).all(|w| w[0] == w[CYCLE]));
            assert_eq!(d[..CYCLE].iter().sum::<i32>(), 0);
            assert_eq!(k.medium.traj.omega_star_q16(), CRUISE << 16);
        }

        /// Off is the profile's speed exactly: no burst, no loaded dead-man,
        /// either field 0, a limit flag but the permit, a mode but Velocity.
        #[test]
        fn the_stimulus_needs_a_burst_a_loaded_deadman_and_no_limit_but_the_permit() {
            let off = |cfg: fn(&mut ControlTable), streaming: bool| {
                let sh = Shared::new();
                let mut k = inject_rig(&sh, cfg);
                k.tel.streaming = streaming;
                // the gate is RAIL's, for the next period
                period(&mut k, &sh, frame(2000, BIAS));
                injected::<{ 2 * CYCLE }>(&mut k, &sh)
            };
            let none = [0; 2 * CYCLE];
            assert_eq!(off(|_| {}, false), none, "no burst");
            assert_eq!(
                off(|t| t.control.lifecycle.bus_deadman_ticks = 0, true),
                none,
                "no dead-man"
            );
            assert_eq!(off(|t| t.control.inject.inject_accel_q88 = 0, true), none);
            assert_eq!(
                off(|t| t.control.inject.inject_half_periods = 0, true),
                none
            );
            // a goal under the feed-forward caps the reference: GOAL and CEILING
            assert_eq!(
                off(|t| t.control.lifecycle.goal_current = 1, true),
                none,
                "a limit holds the command"
            );
            assert_ne!(off(|_| {}, true), none);

            // the permit lease alone leaves it on
            let sh = Shared::new();
            let mut k = inject_rig(&sh, |_| {});
            write_permit(&sh);
            period(&mut k, &sh, frame(2000, BIAS));
            assert_eq!(limit_flags(&sh), limits::flag::PERMIT);
            assert_ne!(injected::<{ 2 * CYCLE }>(&mut k, &sh), none);

            // Position mode never injects: the reference is the position loop's
            let f = frame(2000, BIAS);
            let run = |accel: u16| {
                let sh = Shared::new();
                let mut k = inject_rig(&sh, |t| {
                    t.control.lifecycle.mode = Mode::Position;
                    t.control.lifecycle.goal_position = 2400;
                    t.control.inject.inject_accel_q88 = accel;
                });
                core::array::from_fn::<i32, { 2 * CYCLE }, _>(|_| {
                    run_to(&mut k, &sh, f, phase::SPARE_A);
                    k.on_scan(f, &sh);
                    k.medium.omega_ref_q16
                })
            };
            assert_eq!(run((SLOPE << 8) as u16), run(0));
        }

        /// The phase steps only on a period that injects: a gated stretch
        /// holds it, and the triangle resumes where it stopped.
        #[test]
        fn a_gated_period_holds_the_stimulus_phase() {
            let sh = Shared::new();
            let mut k = inject_rig(&sh, |_| {});
            let before = injected::<7>(&mut k, &sh);
            k.tel.streaming = false;
            // the period whose CONTROL first reads no burst injects nothing
            assert_eq!(injected::<6>(&mut k, &sh), [0; 6]);
            k.tel.streaming = true;
            // and the first one after the burst resumes injects again
            let after = injected::<CYCLE>(&mut k, &sh);

            let sh = Shared::new();
            let mut k = inject_rig(&sh, |_| {});
            let plain = injected::<{ 7 + CYCLE }>(&mut k, &sh);
            assert_eq!(plain[..7], before[..]);
            assert_eq!(plain[7..], after[..]);
        }

        /// The summed reference stays inside the speed ceiling at every field
        /// value.
        #[test]
        fn the_stimulus_is_clamped_to_the_speed_ceiling() {
            let lim = CRUISE + 5;
            let sh = Shared::new();
            let mut k = inject_rig(&sh, |t| {
                t.config.loop_position.speed_ceiling_cps = lim as u16;
            });
            let d = injected::<{ 2 * CYCLE }>(&mut k, &sh);
            assert_eq!(d.iter().max(), Some(&((lim - CRUISE) << 16)));
            assert_eq!(d.iter().min(), Some(&((-SLOPE * HALF as i32 / 2) << 16)));

            for (accel, half) in [
                (u16::MAX, u16::MAX),
                (u16::MAX, 512),
                (u16::MAX, 511),
                (1, 1),
            ] {
                let sh = Shared::new();
                let mut k = inject_rig(&sh, |t| {
                    t.control.inject.inject_accel_q88 = accel;
                    t.control.inject.inject_half_periods = half;
                });
                for _ in 0..600 {
                    period(&mut k, &sh, frame(2000, BIAS));
                    let u = k.medium.omega_ref_q16;
                    assert!(u.unsigned_abs() <= 3000 << 16, "{accel} {half}: {u}");
                }
            }
        }

        /// The kernel reads the slope at most 0x7FFF and the half-period at
        /// most 512: at both ceilings the triangle's peak is 0x7FFF x 256 in
        /// Q8.8, exactly i32's last whole c/s in csQ16 (32767 c/s), and a field
        /// above either ceiling runs as the ceiling. A 0 cruise under the
        /// table's widest speed ceiling shows the whole triangle unclamped.
        #[test]
        fn the_stimulus_reads_its_fields_at_most_at_their_ceilings() {
            let wide = |accel: u16, half: u16| {
                let sh = Shared::new();
                let mut k = inject_rig(&sh, |t| {
                    t.control.lifecycle.goal_velocity = 0;
                    t.config.loop_position.speed_ceiling_cps = 32767;
                    t.control.inject.inject_accel_q88 = accel;
                    t.control.inject.inject_half_periods = half;
                });
                injected::<1100>(&mut k, &sh)
            };
            let peak = 0x7FFF << 16;
            for (accel, half) in [(0x7FFF, 512), (0x8000, 512), (u16::MAX, 600)] {
                let d = wide(accel, half);
                assert_eq!(d.iter().max(), Some(&peak), "{accel} {half}");
                assert_eq!(d.iter().min(), Some(&-peak), "{accel} {half}");
                assert!(d.windows(1025).all(|w| w[0] == w[1024]), "{accel} {half}");
                assert!(d.windows(2).all(|w| (w[1] - w[0]).abs() == 0x7FFF << 8));
            }
            assert_eq!(wide(u16::MAX, 2), wide(0x7FFF, 2));
            assert_eq!(wide(256, 600), wide(256, 512));
        }

        /// Either field 0 is the plain Velocity arm exactly, the gate's other
        /// inputs all open: nothing sums and nothing clamps, so a profile above
        /// a speed ceiling lowered mid-run comes down at the accel limit.
        #[test]
        fn a_stimulus_not_asked_for_leaves_the_velocity_arm_alone() {
            for (accel, half) in [(0, 0), (0, HALF), ((SLOPE << 8) as u16, 0)] {
                let sh = Shared::new();
                let mut k = inject_rig(&sh, |t| {
                    t.control.inject.inject_accel_q88 = accel;
                    t.control.inject.inject_half_periods = half;
                });
                assert_eq!(k.medium.traj.omega_star_q16(), CRUISE << 16);
                write_config(&sh, |t| t.config.loop_position.speed_ceiling_cps = 500);
                let mut above = 0;
                for _ in 0..40 {
                    assert_eq!(period(&mut k, &sh, frame(2000, BIAS)), 0, "{accel} {half}");
                    above += (k.medium.omega_ref_q16 > 500 << 16) as u32;
                }
                assert!(above > 1, "{accel} {half}: the profile came down at once");
                assert_eq!(k.medium.omega_ref_q16, 500 << 16);
            }
        }
    }
}

/// The production image accepts the loop-margin stimulus's fields and
/// ignores them: a Velocity cruise under a loaded dead-man keeps the
/// profile's speed as its reference, and `omega_ref_cps` reads 0.
#[cfg(not(feature = "loop-margin"))]
#[test]
fn the_injection_fields_are_ignored_without_the_feature() {
    let f = frame(2000, BIAS);
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        let l = &mut t.control.lifecycle;
        l.torque_enable = true;
        l.mode = Mode::Velocity;
        l.goal_velocity = 1000;
        l.bus_deadman_ticks = u16::MAX;
        t.control.inject.inject_accel_q88 = 2 << 8;
        t.control.inject.inject_half_periods = 10;
    });
    let mut k = kernel();
    for _ in 0..100 {
        run_to(&mut k, &sh, f, phase::SPARE_A);
        k.on_scan(f, &sh);
        assert_eq!(k.medium.omega_ref_q16, k.medium.traj.omega_star_q16());
        sh.table
            .with(|t| assert_eq!(t.telemetry_ext.inject.omega_ref_cps, 0));
    }
    assert_eq!(k.medium.omega_ref_q16, 1000 << 16);
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
    // a long hold over i_cont folds nothing (bit 4 is retired), the rail a
    // sag into the band; each names itself
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
    // rating: nothing folds on the heat alone
    for _ in 0..DECIM_MED as u32 * DECIM_SLOW as u32 * 156 {
        k.on_scan(frame(2000, BIAS + 400), &sh);
    }
    assert_eq!(limit_flags(&sh), 0);
    assert_eq!(sh.table.with(|t| t.telemetry.estimates.i_lim_counts), 1200);
    // the rail halfway into the band
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
    // the ack lands after that tick's write; the next one drives
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    k.on_scan(frame(2000, BIAS + 3000), &sh);
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
    // up to the first drive: no window read yet
    to_first_command(&mut k, &sh, frame(2000, BIAS + 3000));
    assert_eq!(k.faults.mask(), 0);
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
fn openloop_duty_passthrough_clamped() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    let mut k = kernel();
    settle(&mut k, &sh, frame(2000, BIAS));
    match last_cmd(&k) {
        MotorCmd::Drive { .. } => assert_eq!(written_duty(&k), 8000),
        other => panic!("expected Drive, got {other:?}"),
    }
    // duty_max clamps the passthrough at the next medium boundary, at once:
    // the write after its CONTROL phase
    write_config(&sh, |t| t.config.loop_current.duty_max_q15 = 5000);
    while k.phase != 0 {
        k.on_scan(frame(2000, BIAS), &sh);
        assert_eq!(written_duty(&k), 8000);
    }
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(written_duty(&k), 8000);
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(written_duty(&k), 5000);
}

#[test]
fn duty_max_write_caps_the_closed_loop_at_the_rail_phase() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        unfolded(t);
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
    // the take leaves the clamp alone; the next RAIL phase prepares it for
    // the write after it
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
    assert!(written_duty(&k) > 32000, "the RAIL tick");
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
    // the boot tick writes off; its CONTROL phase's command drives the next
    k.on_scan(frame(2000, BIAS), &sh);
    k.on_scan(frame(2000, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { .. } => assert_eq!(written_duty(&k), 0),
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
    // the goal lands at this CONTROL phase, after its write
    k.on_scan(frame(4095, BIAS), &sh);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
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
    // the rewire lands with the take, after the take tick's measurement
    write_config(&sh, |t| t.config.limits.drive_polarity = true);
    while k.phase != phase::SPARE_A {
        k.on_scan(frame(2000, BIAS), &sh);
        assert_eq!(k.fast.vdiff_last, -(3000 - 40));
    }
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.fast.vdiff_last, -(3000 - 40));
    k.on_scan(frame(2000, BIAS), &sh);
    assert_eq!(k.fast.vdiff_last, 3000 - 40);
}

/// The seed's 3000 c/s ceiling at its speed gain folds Current mode's band
/// at every speed (the fold binds from `ceiling - cap / kp`): the ceiling
/// for a Current-mode test of something else, where the fold never binds.
fn unfolded(t: &mut ControlTable) {
    t.config.loop_position.speed_ceiling_cps = loops::trajectory::VEL_CAP_CPS;
}

#[test]
fn current_mode_clamps_goal_to_i_lim() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        unfolded(t);
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 5000;
    });
    let mut k = kernel();
    to_first_command(&mut k, &sh, frame(2000, BIAS));
    assert_eq!(k.medium.i_ref_cc, 1200);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
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

/// With no speed gain the speed fold reads 0 at every speed, so Current
/// mode delivers nothing: at rest the band closes both sides, the goal
/// clamps to 0, and `limit_flags` names the fold.
#[test]
fn current_mode_with_no_speed_gain_delivers_nothing() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.config.loop_velocity.v_kp_q88 = 0;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 5000;
    });
    let mut k = kernel();
    to_first_command(&mut k, &sh, frame(2000, BIAS));
    assert_eq!(k.medium.i_band, IBand { lo: 0, hi: 0 });
    assert_eq!(k.medium.i_ref_cc, 0);
    for _ in 0..4 * DECIM_MED {
        k.on_scan(frame(2000, BIAS), &sh);
        assert_eq!(k.medium.i_ref_cc, 0);
    }
    assert_eq!(limit_flags(&sh), limits::flag::SPEED);
    assert_eq!(
        sh.table.with(|t| t.telemetry.estimates.i_lim_counts),
        1200,
        "the composed limit is published, not the folded sides"
    );
}

/// One Current-mode lift across a 40-tick shunt floor, and the three ticks
/// from the first command at or over it.
struct Lift {
    /// The width commanded on the tick before the first command at or over
    /// the floor.
    before: u16,
    /// The duty of that first command, of the tick that measured its
    /// window, and of the tick after.
    duty: [i16; 3],
    /// The tick block and the fault mask on the tick that measured the
    /// first read window.
    i_tick_counts: i16,
    i_valid: bool,
    faults: u8,
}

/// The reference steps to 120 counts, whose volts clear the floor, from
/// one whose volts sit under it: 10 counts with `dropped`, a pulse of a few
/// ticks the bridge drops, otherwise 45, a pulse the bridge puts out. The
/// first read window reads `first` counts over the zero and the next one
/// `second`.
fn lift(dropped: bool, first: u16, second: u16, oc_trip_counts: u16) -> Lift {
    const FLOOR: u16 = 40;
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = if dropped { 10 } else { 45 };
        t.calib.sense.i_window_min_ticks = FLOOR;
        t.config.limits.oc_trip_counts = oc_trip_counts;
        t.config.limits.oc_trip_ticks = 1;
    });
    let mut k = kernel();
    let rest = frame(2000, BIAS);
    ticks(&mut k, &sh, rest, 20);
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = 120);
    let mut before = 0;
    for _ in 0..200 {
        before = k.fast.ticks;
        k.on_scan(rest, &sh);
        if k.fast.ticks >= FLOOR {
            break;
        }
    }
    assert!(k.fast.ticks >= FLOOR, "never over the floor");
    let d0 = k.fast.duty_q15;
    k.on_scan(frame(2000, BIAS + first), &sh);
    let t = tick_block(&sh);
    let (d1, faults) = (k.fast.duty_q15, k.faults.mask());
    k.on_scan(frame(2000, BIAS + second), &sh);
    Lift {
        before,
        duty: [d0, d1, k.fast.duty_q15],
        i_tick_counts: t.i_tick_counts,
        i_valid: t.tick_flags & tick_flag::I_VALID != 0,
        faults,
    }
}

/// The first shunt window read after a commanded pulse too narrow for the
/// bridge to put out reads far over the winding current (bench: a median
/// of 145 counts after a row of 1 to 15 ticks against 63 inside a train of
/// the same width), and on the proportional gain that one reading steps
/// the duty across zero. The current loop does not take it: the command
/// that follows is the one no reading gives, whatever the window read, and
/// the window after it is taken as any other. Everything else keeps
/// the reading: the held current, the tick flags and the overcurrent trip.
/// A first read window after a pulse the bridge did put out is winding
/// current still rising, and the loop takes it.
#[test]
fn the_first_reading_after_a_dropped_pulse_never_reaches_the_current_loop() {
    const HIGH: u16 = 300;
    const NO_TRIP: u16 = 2400;
    let over = lift(true, HIGH, 100, NO_TRIP);
    let zero = lift(true, 0, 100, NO_TRIP);
    assert!((1..=15).contains(&over.before), "{}", over.before);
    assert_eq!(over.before, zero.before);
    // the reading is over the reference: taken, it reverses the drive
    assert!(over.duty[0] > 0);
    assert_eq!(over.duty[1], zero.duty[1], "the loop took the reading");
    assert_eq!(over.duty[1], over.duty[0], "the command moved");
    // the strict view of the same window
    assert_eq!(over.i_tick_counts, HIGH as i16);
    assert!(over.i_valid);
    assert_eq!(over.faults, 0);
    let trip = lift(true, HIGH, 100, HIGH - 1);
    assert_eq!(trip.faults, faults::BIT_OVER_CURRENT);
    // one reading, not the visit: the next window moves the loop
    let next = lift(true, HIGH, 0, NO_TRIP);
    assert_eq!(next.duty[1], over.duty[1]);
    assert!(
        over.duty[2] < next.duty[2],
        "the second reading was not taken"
    );
    assert!(over.duty[2] > 0);

    // a lift from a pulse under the floor that the bridge put out: the
    // first read window's reading is taken
    let put_out_high = lift(false, HIGH, 100, NO_TRIP);
    let put_out_zero = lift(false, 0, 100, NO_TRIP);
    assert!(
        (31..40).contains(&put_out_high.before),
        "{}",
        put_out_high.before
    );
    assert!(put_out_high.duty[1] < put_out_zero.duty[1]);
}

/// One Current-mode drive reversal over the shunt floor, and the four ticks
/// from the command that reversed.
struct Reversal {
    /// The width before the reversal, and the duty of the reversing
    /// command and of the three ticks after it.
    before: u16,
    duty: [i16; 4],
    /// The no-reading command on the reversal's first window's tick
    /// (`blind_duty`), its reference and held current after it.
    blind: i16,
    i_ref: i32,
    held: i16,
    /// The tick flags on that tick, and the fault mask after the last.
    flags: u16,
    faults: u8,
}

/// Current mode on a still shaft at a reference of 120 counts, read as 120,
/// until one window reads 300: on the proportional gain the command crosses
/// zero to a width over the floor. The reversal's first window reads
/// `first`, the next `second`, the one after `third`, each in its own drive's
/// sign. The reference never moves, so no tick depends on when the medium
/// step lands its command.
fn reversal(first: u16, second: u16, third: u16, oc_trip_counts: u16) -> Reversal {
    const FLOOR: u16 = 64;
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 120;
        t.config.loop_current.i_kp_q88 = 3 << 8;
        t.calib.sense.i_window_min_ticks = FLOOR;
        t.calib.sense.v_window_min_ticks = FLOOR;
        t.config.limits.oc_trip_counts = oc_trip_counts;
        t.config.limits.oc_trip_ticks = 2;
    });
    let mut k = kernel();
    let along = frame(2000, BIAS + 120);
    to_first_command(&mut k, &sh, along);
    ticks(&mut k, &sh, along, 40);
    let before = k.fast.ticks;
    k.on_scan(frame(2000, BIAS + 300), &sh);
    let d0 = k.fast.duty_q15;
    k.on_scan(frame(2000, BIAS + first), &sh);
    let (d1, blind, i_ref, held) = (
        k.fast.duty_q15,
        blind_duty(&k, 8192),
        k.cmd.i_ref_cc,
        k.fast.i_meas_last,
    );
    let flags = tick_block(&sh).tick_flags;
    k.on_scan(frame(2000, BIAS + second), &sh);
    let d2 = k.fast.duty_q15;
    k.on_scan(frame(2000, BIAS + third), &sh);
    Reversal {
        before,
        duty: [d0, d1, d2, k.fast.duty_q15],
        blind,
        i_ref,
        held,
        flags,
        faults: k.faults.mask(),
    }
}

/// The first drive window after the drive reverses runs against a winding
/// that still carries the old direction's current (its L/R is about four
/// periods), and the return-path shunt reads a current against the drive
/// as about zero (bench: +1 count on 3,352 of 3,355 reversals). That window
/// carries no reading for any consumer: the loop commands the reference's
/// volts as under the floor, the held current is the reference, the tick
/// flags name no reading, and the overcurrent trip holds its count. Here
/// the no-reading command for the unchanged reference reverses the drive
/// back, so the window after is a reversal's first too; the one after that
/// is taken as any other.
#[test]
fn the_first_window_after_a_drive_reversal_is_no_reading() {
    const NO_TRIP: u16 = 2400;
    let near_zero = reversal(1, 1, 60, NO_TRIP);
    let high = reversal(300, 1, 60, NO_TRIP);
    assert!(near_zero.before >= 64, "{} ticks", near_zero.before);
    assert!(near_zero.duty[0] < 0, "the drive did not reverse");
    assert!(
        window::drive_ticks(near_zero.duty[0], ARR) >= 64,
        "the reversal landed under the floor"
    );
    assert_eq!(
        near_zero.duty[1], high.duty[1],
        "the loop took the reading of the first window after a drive reversal"
    );
    assert_eq!(near_zero.duty[1], near_zero.blind);
    assert_eq!(near_zero.held as i32, near_zero.i_ref);
    assert_eq!(
        near_zero.flags & (tick_flag::I_VALID | tick_flag::V_VALID),
        0
    );
    assert!(near_zero.duty[1] > 0 && window::drive_ticks(near_zero.duty[1], ARR) >= 64);
    assert_eq!(near_zero.duty[2], reversal(1, 300, 60, NO_TRIP).duty[2]);
    // the window after a drive that kept its direction is a reading
    let third_high = reversal(1, 1, 200, NO_TRIP);
    assert_ne!(
        near_zero.duty[3], third_high.duty[3],
        "the window was not taken"
    );

    // an overcurrent read on both sides of the reversal trips across it
    let trip = reversal(1, 1, 300, 250);
    assert_eq!(trip.faults, faults::BIT_OVER_CURRENT);
}

/// What the bridge does between two opposite drives.
#[derive(Copy, Clone, Debug)]
enum Stop {
    Coast,
    Brake,
    Off,
}

/// OpenLoop at 8000 on a window reading 100 counts, then `stop` for at
/// least `held` ticks, then -8000: the ticks the stop wrote, and whether
/// the reversed drive's first window and the one after it read.
fn reversal_across(stop: Stop, held: u32) -> (u32, [bool; 2]) {
    let sh = Shared::new();
    drive_setup(&sh);
    sh.table
        .with_mut(|t| t.config.limits.openloop_zero_brake = matches!(stop, Stop::Brake));
    let mut k = kernel();
    let f = frame(2000, BIAS + 100);
    settle(&mut k, &sh, f);
    sh.table.with_mut(|t| match stop {
        Stop::Off => t.control.lifecycle.torque_enable = false,
        _ => t.control.lifecycle.goal_duty = 0,
    });
    let mut stopped = 0;
    loop {
        if stopped >= held {
            sh.table.with_mut(|t| {
                t.control.lifecycle.torque_enable = true;
                t.control.lifecycle.goal_duty = -8000;
            });
        }
        k.on_scan(f, &sh);
        match k.fast.duty_q15 {
            0 => stopped += 1,
            d if d < 0 => break,
            _ => {}
        }
    }
    assert!(k.fast.ticks >= 100, "the reversal landed under the floor");
    let read = core::array::from_fn(|_| {
        k.on_scan(f, &sh);
        tick_block(&sh).tick_flags & tick_flag::I_VALID != 0
    });
    (stopped, read)
}

/// A drive that reverses across a brake, a coast or the drive off pushes
/// against the last drive's current while the winding may still carry it:
/// the brake carries it on L/R, the coast and the off drain it into the
/// rail, each within the zero's settle (`fast::ZERO_SETTLE_TICKS`). Inside
/// it the reversal's first window is no reading; past it the drive is a
/// first one and its first window reads. So does the first drive from
/// boot, either way.
#[test]
fn a_reversal_across_a_stop_is_no_reading_until_the_winding_settles() {
    let settle_ticks = fast::ZERO_SETTLE_TICKS as u32;
    for stop in [Stop::Coast, Stop::Brake, Stop::Off] {
        let (n, read) = reversal_across(stop, 1);
        assert!((1..settle_ticks).contains(&n), "{stop:?}: {n} ticks");
        assert_eq!(read, [false, true], "{stop:?} for {n} ticks");
        let (n, read) = reversal_across(stop, 2 * settle_ticks);
        assert!(n > settle_ticks + 1, "{stop:?}: {n} ticks");
        assert_eq!(read, [true, true], "{stop:?} for {n} ticks");
    }
    for goal in [8000, -8000] {
        let sh = Shared::new();
        drive_setup(&sh);
        sh.table.with_mut(|t| t.control.lifecycle.goal_duty = goal);
        let mut k = kernel();
        let f = frame(2000, BIAS + 100);
        while k.fast.ticks == 0 {
            k.on_scan(f, &sh);
        }
        k.on_scan(f, &sh);
        assert_ne!(
            tick_block(&sh).tick_flags & tick_flag::I_VALID,
            0,
            "the first drive from boot at {goal}"
        );
    }
}

/// The timer loads a motor write at the next update event, at the crest or
/// the trough, so a write past the trough lands inside the drive pulse the
/// crest scan samples and builds that pulse from two commands. A write that
/// lands before the trough builds every pulse from one. The write is the
/// tick's first act after the window take and cannot wait for the medium
/// step: it drives the command the previous tick's medium step left, so a
/// reference that lands at VELOCITY drives from the next tick.
#[test]
fn the_motor_write_never_waits_for_the_medium_step() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 120;
        t.calib.sense.i_window_min_ticks = 64;
    });
    let mut k = kernel();
    // the window reads the reference: the loop holds its command
    let along = frame(2000, BIAS + 120);
    to_first_command(&mut k, &sh, along);
    ticks(&mut k, &sh, along, 40);
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = 200);
    run_to(&mut k, &sh, along, phase::CONTROL);
    run_to(&mut k, &sh, along, phase::VELOCITY);
    let held = k.fast.duty_q15;
    k.on_scan(along, &sh);
    assert_eq!(k.cmd.i_ref_cc, 200, "the reference did not land");
    assert_eq!(
        k.fast.duty_q15, held,
        "the motor write took the reference its own tick's medium step landed"
    );
    k.on_scan(along, &sh);
    assert!(k.fast.duty_q15 > held, "the new reference never drove");
}

/// Under the shunt floor no window reports, and the command is the
/// reference through the winding's resistance on top of the Ke
/// feed-forward: `r_q12 x i_ref`, nothing integrated. Fed a zero current
/// there, the integrator moves by `ki x i_ref` a tick whatever the winding
/// carries, so a reference of a few counts walks the command across the
/// whole band and out its far edge (bench: a decelerating shaft asked for
/// -38 counts went from +62 to -64 ticks in 10 ticks and read about -110
/// there, driven in reverse), and a reference back at zero leaves the
/// walked width standing. A reference whose volts clear the floor lifts
/// over it on its first command, so the band cannot deadlock.
#[test]
fn the_command_never_integrates_under_the_shunt_floor() {
    const FLOOR: u16 = 64;
    // r_q12 2.0 vcounts per ccount x 10 counts = 20 vcounts of a 3000 rail
    const HELD: u16 = 8;
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = -10;
        t.calib.sense.i_window_min_ticks = FLOOR;
    });
    let mut k = kernel();
    // a shaft at rest inside its friction: no window ever reads a current
    let rest = frame(2000, BIAS);
    to_first_command(&mut k, &sh, rest);
    let (mut widest, mut narrowest) = (0, u16::MAX);
    for _ in 0..1000 {
        k.on_scan(rest, &sh);
        assert!(k.fast.duty_q15 < 0, "the drive left the reference's side");
        widest = widest.max(k.fast.ticks);
        narrowest = narrowest.min(k.fast.ticks);
    }
    assert!(
        widest <= HELD + 1,
        "a reference of 10 counts walked the command to {widest} ticks"
    );
    assert!(narrowest >= HELD - 1, "{narrowest} ticks");

    // back at zero nothing stands
    let goal = |g: i16, k: &mut Kernel<FakeIo>| {
        sh.table.with_mut(|t| t.control.lifecycle.goal_current = g);
        run_to(k, &sh, rest, phase::CONTROL);
        run_to(k, &sh, rest, phase::VELOCITY);
        ticks(k, &sh, rest, 2);
    };
    goal(0, &mut k);
    assert_eq!(k.fast.duty_q15, 0, "a zero reference kept a width");

    // 2.0 x 100 counts = 200 vcounts = 80 ticks: over the floor at once
    goal(100, &mut k);
    assert!(k.fast.ticks >= FLOOR, "{} ticks", k.fast.ticks);
    assert!(k.fast.duty_q15 > 0);
}

/// The duty the current loop commands on a tick with no reading: the
/// reference through `r_q12` on top of the feed-forward, over the rail.
fn blind_duty(k: &Kernel<FakeIo>, r_q12: i32) -> i16 {
    let fast::Drive::Closed { u_ff_vc } = k.cmd.drive else {
        panic!("not the current loop");
    };
    let u = q_mul(r_q12, k.cmd.i_ref_cc, 12) + u_ff_vc;
    q_mul(u, k.cmd.vbus_recip_q15 as i32, 15) as i16
}

/// On a tick with no reading the command is the reference's volts through
/// the winding on top of the Ke feed-forward, to the count, whatever the
/// ticks before it commanded: the velocity loop moves the reference across
/// its band on a still shaft and the command follows it tick for tick. The
/// held current is that reference, with the tick flags still naming no
/// reading.
#[test]
fn under_the_floor_the_command_is_the_reference_through_the_winding() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 160;
        // 2.0 x 49 counts + 0.0625 x 160 c/s = 108 vcounts = 43 ticks: the
        // whole band sits under the 100-tick floor
        t.config.limits.current_limit_counts = 49;
    });
    let mut k = kernel();
    let rest = frame(2000, BIAS);
    to_first_command(&mut k, &sh, rest);
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    for _ in 0..4000 {
        // the write takes the command the last medium step left
        let (want, i_ref) = (blind_duty(&k, 8192), k.cmd.i_ref_cc);
        k.on_scan(rest, &sh);
        assert_eq!(k.fast.duty_q15, want);
        assert!(k.fast.ticks < 100, "{} ticks", k.fast.ticks);
        assert_eq!(k.fast.i_meas_last as i32, i_ref);
        lo = lo.min(k.cmd.i_ref_cc);
        hi = hi.max(k.cmd.i_ref_cc);
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(hi - lo >= 20, "the reference stood still: {lo}..{hi}");
    assert!(matches!(k.cmd.drive, fast::Drive::Closed { u_ff_vc: 10 }));
    let held = k.fast.i_meas_last;
    assert_ne!(held, 0);
    k.on_scan(rest, &sh);
    let t = tick_block(&sh);
    assert_eq!(t.i_tick_counts, held);
    assert_eq!(t.tick_flags & tick_flag::I_VALID, 0);
}

/// Under the floor the command's stall current is the reference, so the
/// band holds no more than the floor's volts over the winding's resistance,
/// the bound OpenLoop's base duty has there; a reference past it opens a
/// window on its first command.
#[test]
fn the_band_under_the_floor_holds_at_most_the_floors_volts_over_r() {
    const FLOOR: u16 = 64;
    let rest = frame(2000, BIAS);
    let mut held = 0;
    for goal in (-120..=120).filter(|&g| g != 0) {
        let sh = Shared::new();
        seed(&sh);
        sh.table.with_mut(|t| {
            t.control.lifecycle.torque_enable = true;
            t.control.lifecycle.mode = Mode::Current;
            t.calib.sense.i_window_min_ticks = FLOOR;
        });
        let mut k = kernel();
        ticks(&mut k, &sh, rest, 2 * DECIM_MED as u32);
        sh.table
            .with_mut(|t| t.control.lifecycle.goal_current = goal);
        run_to(&mut k, &sh, rest, phase::CONTROL);
        run_to(&mut k, &sh, rest, phase::VELOCITY);
        k.on_scan(rest, &sh);
        assert_eq!(k.cmd.i_ref_cc, goal as i32);
        let want = blind_duty(&k, 8192);
        k.on_scan(rest, &sh);
        assert_eq!(k.fast.duty_q15, want, "{goal}");
        if k.fast.ticks >= FLOOR {
            continue;
        }
        held = held.max(goal.unsigned_abs());
        let width = k.fast.ticks;
        for _ in 0..200 {
            k.on_scan(rest, &sh);
            assert_eq!(k.fast.ticks, width, "{goal}");
        }
    }
    // 64 ticks of 1200 on a 3000 rail = 160 vcounts, 80 counts at 2.0
    assert_eq!(held, 79);
}

/// The reference held under the floor is the loop's own command, not a
/// reading: the overcurrent trip and the back-EMF sums take a reading or
/// nothing. A held reference over the trip raises nothing and closes no
/// back-EMF half, with the terminal taps reading the window all along;
/// the same drive with the shunt's floor under it trips on its reading.
#[test]
fn the_reference_held_under_the_floor_reaches_no_trip_and_no_back_emf_sum() {
    let rig = |i_floor: u16| {
        let sh = Shared::new();
        seed(&sh);
        sh.table.with_mut(|t| {
            t.control.lifecycle.torque_enable = true;
            t.control.lifecycle.mode = Mode::Current;
            // 2.0 x 40 counts = 80 vcounts = 32 ticks
            t.control.lifecycle.goal_current = 40;
            t.config.limits.oc_trip_counts = 30;
            t.config.limits.oc_trip_ticks = 1;
            t.calib.sense.i_window_min_ticks = i_floor;
            t.calib.sense.v_window_min_ticks = 20;
        });
        let mut k = kernel();
        for _ in 0..400 {
            k.on_scan(frame(2000, BIAS + 35), &sh);
        }
        (k, sh)
    };
    let (k, sh) = rig(100);
    assert_eq!(k.fast.ticks, 32);
    assert_eq!(k.fast.i_meas_last, 40);
    let t = tick_block(&sh);
    assert_eq!(t.i_tick_counts, 40);
    assert_eq!(
        t.tick_flags & (tick_flag::I_VALID | tick_flag::V_VALID),
        tick_flag::V_VALID
    );
    assert_eq!(k.faults.mask(), 0);
    // a closed half would read (32 x 2960 / 1200 - 2.0 x 35) x 16 c/s
    let published = sh.table.with(|t| {
        (
            t.telemetry.estimates.omega_bemf_cps,
            t.telemetry.mode.omega_hat_src,
        )
    });
    assert_eq!(published, (0, 0));

    let (k, _) = rig(20);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
}

/// The held current is an i16: a reference past it, which only a current
/// limit over the shunt's range allows, holds at the field's end instead
/// of wrapping to the other sign.
#[test]
fn the_reference_held_under_the_floor_saturates_at_the_fields_width() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    k.on_scan(frame(2000, BIAS), &sh);
    k.cmd.drive = fast::Drive::Closed { u_ff_vc: 0 };
    for (i_ref, held) in [(40_000, i16::MAX), (-40_000, i16::MIN), (-7, -7)] {
        k.cmd.i_ref_cc = i_ref;
        let none = fast::Measured {
            i_meas: None,
            sel: window::select(0, 0, 0),
        };
        k.fast.drive(&none, &k.cfg, &k.cmd, &k.faults);
        assert_eq!(k.fast.i_meas_last, held, "{i_ref}");
    }
}

/// With no identified resistance the loop's own proportional gain stands
/// in under the floor, so an unidentified winding still lifts over it; the
/// choice is made where the configuration is taken, and a commit moves it.
#[test]
fn an_unidentified_winding_is_commanded_through_the_loops_own_gain() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 40;
        t.calib.motor.r_q12 = 0;
        // 1.5 vcounts per count
        t.config.loop_current.i_kp_q88 = 384;
    });
    let mut k = kernel();
    let rest = frame(2000, BIAS);
    to_first_command(&mut k, &sh, rest);
    assert_eq!(k.cfg.r_loop_q12, 384 << 4);
    // 1.5 x 40 counts = 60 vcounts = 24 ticks, and it stays there
    for _ in 0..200 {
        k.on_scan(rest, &sh);
        assert_eq!(k.fast.duty_q15, blind_duty(&k, 384 << 4));
        assert_eq!(k.fast.ticks, 24);
    }
    // an identified winding is the one commanded through: 1.0 x 40 counts
    // = 40 vcounts = 16 ticks
    write_config(&sh, |t| t.calib.motor.r_q12 = 4096);
    ticks(&mut k, &sh, rest, 2 * DECIM_MED as u32);
    assert_eq!(k.cfg.r_loop_q12, 4096);
    assert_eq!(k.fast.duty_q15, blind_duty(&k, 4096));
    assert_eq!(k.fast.ticks, 16);
    write_config(&sh, |t| t.calib.motor.r_q12 = 0);
    ticks(&mut k, &sh, rest, 2 * DECIM_MED as u32);
    assert_eq!(k.cfg.r_loop_q12, 384 << 4);
    assert_eq!(k.fast.ticks, 24);

    // 1.5 x 200 counts = 300 vcounts = 120 ticks: over the floor at once
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = 200);
    run_to(&mut k, &sh, rest, phase::CONTROL);
    run_to(&mut k, &sh, rest, phase::VELOCITY);
    ticks(&mut k, &sh, rest, 2);
    assert_eq!(k.fast.ticks, 120);
}

/// The take resolves the brush term from the stored pair: the size capped
/// at half the stored winding, off under the fade's span, and the fade's
/// low end under the full-weight speed; a commit moves both.
#[test]
fn the_take_resolves_the_brush_term() {
    let sh = Shared::new();
    seed(&sh);
    let r = sh.table.with(|t| t.calib.motor.r_q12);
    let mut k = kernel();
    let rest = frame(2000, BIAS);
    ticks(&mut k, &sh, rest, 2 * DECIM_MED as u32);
    let span = bemf::BRUSH_SPAN_CPS;
    assert_eq!((k.cfg.brush_rc_q12, k.cfg.brush_lo_cps), (0, -span));
    for (x, cps, want) in [
        (1093, 2540, (1093, 1516)),
        (r, 2540, ((r >> 1) as i32, 1516)),
        (1093, 1023, (0, -1)),
        (1093, 1024, (1093, 0)),
        (0, 2540, (0, 1516)),
        (0, 0, (0, -span)),
    ] {
        write_config(&sh, |t| {
            t.calib.motor.r_brush_q12 = x;
            t.calib.motor.brush_cps = cps;
        });
        ticks(&mut k, &sh, rest, 2 * DECIM_MED as u32);
        assert_eq!(
            (k.cfg.brush_rc_q12, k.cfg.brush_lo_cps),
            want,
            "{x} at {cps}"
        );
    }
}

/// The set of record's window conductance, under the rig's winding.
const G_WINDOW: u16 = 1423;

/// An OpenLoop drive at `goal` over the trip-free ceiling with `g` stored.
fn window_rig(goal: i16, g: u16) -> Shared {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = goal;
        // over the limit on purpose: the OpenLoop ceiling stays out of it
        t.config.limits.current_limit_counts = u16::MAX;
        t.calib.motor.g_window_q016 = g;
    });
    sh
}

/// The window term the kernel's take and rail put on a `ticks` width, Q16
/// counts.
fn window_term(k: &Kernel<FakeIo>, ticks: u32) -> u32 {
    let v = k.cmd.vbus_counts.max(k.cfg.thermal.v_undervolt_counts);
    let mut w = window::WindowTerm::default();
    w.set_rail(k.cfg.g_arr, v, ARR);
    w.q16(ticks)
}

/// The published current is the sample less the window term at the
/// measured width and the rail, signed by the drive: each reading takes
/// the term's whole count or the next one up, and over a run they take the
/// Q16 term within one count, the carry's span; with no conductance the
/// sample's, bit for bit.
#[test]
fn the_reading_takes_the_term_off_along_the_drive() {
    for g in [0, G_WINDOW] {
        for goal in [8000i16, -8000] {
            let sh = window_rig(goal, g);
            let mut k = kernel();
            let f = frame(2000, BIAS + 300);
            settle(&mut k, &sh, f);
            ticks(&mut k, &sh, f, 2 * DECIM_MED as u32);
            let term = window_term(&k, k.fast.ticks as u32);
            assert_eq!(term == 0, g == 0, "{g} at {goal}: {term}");
            let n = 100 * DECIM_MED as i64;
            let mut taken = 0;
            for _ in 0..n {
                k.on_scan(f, &sh);
                let i = sh.table.with(|t| t.telemetry.tick.i_tick_counts) as i32;
                let off = 300 - goal.signum() as i32 * i;
                let whole = (term >> 16) as i32;
                assert!(off == whole || off == whole + 1, "{g} at {goal}: {off}");
                assert!(g != 0 || off == 0, "{g} at {goal}: {off}");
                taken += (off as i64) << 16;
            }
            assert!(
                (taken - n * term as i64).abs() < 1 << 16,
                "{g} at {goal}: {} counts over the term",
                (taken - n * term as i64) as f64 / 65536.0
            );
            assert_eq!(k.faults.mask(), 0);
        }
    }
}

/// The trip compares the terminal current the bridge carries: a sample
/// over the trip latches it on the fourth window though the reading the
/// loops take sits under it.
#[test]
fn the_overcurrent_trip_reads_the_uncorrected_current() {
    let sh = window_rig(16000, G_WINDOW);
    let mut k = kernel();
    let trip = sh.table.with(|t| t.config.limits.oc_trip_counts);
    let f = frame(2000, BIAS + trip + 10);
    to_first_command(&mut k, &sh, f);
    for _ in 0..3 {
        k.on_scan(f, &sh);
        assert_eq!(k.faults.mask(), 0);
        let i = sh.table.with(|t| t.telemetry.tick.i_tick_counts);
        assert!(i > 0 && (i as u16) < trip, "reads {i} under a {trip} trip");
    }
    k.on_scan(f, &sh);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
}

/// The take runs the stored conductance only while the stored winding
/// bounds it, `0 < g x r_q12 <= 2^25`; a commit moves it, and the next
/// RAIL phase puts it on the rail.
#[test]
fn the_take_fences_the_conductance_by_the_winding() {
    let sh = window_rig(8000, 0);
    let mut k = kernel();
    let f = frame(2000, BIAS + 300);
    settle(&mut k, &sh, f);
    assert_eq!(k.cfg.g_arr, 0);
    let recip = TIMING.recip_arr_q24;
    for (g, r, runs) in [
        (G_WINDOW, 8192, true),
        (4096, 8192, true),
        (4097, 8192, false),
        (u16::MAX, 512, true),
        (u16::MAX, 513, false),
        (G_WINDOW, 0, false),
        (0, 8192, false),
    ] {
        write_config(&sh, |t| {
            t.calib.motor.g_window_q016 = g;
            t.calib.motor.r_q12 = r;
        });
        ticks(&mut k, &sh, f, 2 * DECIM_MED as u32);
        let want = if runs { g as u32 * recip } else { 0 };
        assert_eq!(k.cfg.g_arr, want, "{g} on {r}");
        assert_eq!(k.fast.win.q16(64) != 0, runs, "{g} on {r}: on the rail");
    }
}

/// The term follows the rail estimate at the RAIL phase and holds between
/// two: a rail that sags takes a smaller term, in step with the clamp.
#[test]
fn the_term_follows_the_rail_at_rail() {
    let sh = window_rig(8000, G_WINDOW);
    let mut k = kernel();
    let mut f = frame(2000, BIAS + 300);
    settle(&mut k, &sh, f);
    ticks(&mut k, &sh, f, 2 * DECIM_MED as u32);
    let width = k.fast.ticks as u32;
    let high = k.fast.win.q16(width);
    assert_eq!(high, window_term(&k, width));
    f.vbus_raw = vbus_raw(2600);
    for _ in 0..200 * DECIM_MED as u32 {
        let (was, rail) = (k.fast.win.q16(width), k.phase == phase::RAIL);
        k.on_scan(f, &sh);
        let now = k.fast.win.q16(width);
        assert_eq!(now, window_term(&k, width));
        if !rail {
            assert_eq!(now, was, "moved off the RAIL phase");
        }
    }
    assert_eq!(k.cmd.vbus_counts, 2600);
    let low = k.fast.win.q16(width);
    assert!(low < high, "{low} at 2600 vcounts, {high} at 3000");
}

/// The winding estimate's bench MG90 constant (the NTC on a board in room
/// air) and the dev board's 10K/10K/3950 NTC reading 25.00 C at the rig's
/// 2048 counts, on an OpenLoop hold.
fn seed_estimate(sh: &Shared) {
    write_config(sh, |t| {
        t.calib.thermal.th_rise_q016 = 1044;
        t.calib.thermal.ntc_raw_ref = 2048;
        t.calib.thermal.ntc_t_ref_cc = 2500;
        t.calib.thermal.ntc_k1_q88 = 563;
        t.calib.thermal.ntc_k2_q24 = 5800;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        // the hold sits over the continuous rating: the press is not under
        // test
        t.config.limits.i_cont_counts = 0;
    });
}

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

/// A SLOW period in fast ticks.
const SLOW: u32 = DECIM_MED as u32 * DECIM_SLOW as u32;
const BASE_FAST_TICKS: u32 = 200 * SLOW;
/// SLOW periods for the seed's 4096 ms leak (shift 8) to settle a step in
/// the held current to the integer EWMA's fixed point: 2501 from zero.
const HEAT_SETTLE: u32 = 2600 * SLOW;

/// Without an NTC curve the estimate reads its sentinel, under the lowest
/// cutoff a host can write over it, and nothing faults, however long the
/// hold; with a curve and no gain it is the NTC alone.
#[test]
fn the_estimate_without_a_curve_reads_the_sentinel_and_without_a_gain_the_ntc() {
    use crate::estimator::thermal::flag;
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    write_config(&sh, |t| {
        t.calib.thermal.ntc_k1_q88 = 0;
        t.config.thermal.cutoff_cc = i16::MIN + 1;
    });
    let mut k = kernel();
    let hot = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, BASE_FAST_TICKS);
    let (t, flags, i_lim, _) = therm(&sh);
    assert_eq!((t, flags, i_lim), (i16::MIN, flag::UNSET, 1200));
    assert_eq!(k.faults.mask(), 0);
    assert_eq!(k.fast.duty_q15, 8000, "the hold drives");
    write_config(&sh, |t| {
        t.calib.thermal.ntc_k1_q88 = 563;
        t.calib.thermal.th_rise_q016 = 0;
        t.config.thermal.cutoff_cc = 10000;
    });
    ticks(&mut k, &sh, hot, BASE_FAST_TICKS);
    let (t, flags, i_lim, t_ntc) = therm(&sh);
    assert_eq!((t, flags, i_lim, t_ntc), (2500, 0, 1200, 2500));
    assert_eq!(k.faults.mask(), 0);
}

/// `n` ticks of `f`, the bridge off on every one.
fn never_drives(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame, n: u32) {
    for at in 0..n {
        k.on_scan(f, sh);
        assert!(matches!(last_cmd(k), MotorCmd::Disabled), "tick {at}");
    }
}

/// A failed NTC, a dead divider reading a rail or a curve reading outside
/// the band: the seed latches the overtemperature fault on the boot tick,
/// torque off as a servo boots, and the estimate reads i16::MAX with the
/// fault flag. Nothing folds the ceiling, so the latch alone holds the
/// bridge: the host's enable is an ack, the CONTROL phase that takes it
/// raises the fault again on the estimate it already has, and the bridge
/// stays off on every tick from boot, across the ack and through the
/// finishes after it. Once the NTC reads again an ack clears and the hold
/// drives.
#[test]
fn a_failed_ntc_faults_at_the_seed_and_never_drives() {
    use crate::estimator::thermal::flag;
    for (ntc_raw, k1_q88) in [(6, 563), (4090, 563), (1000, 563 * 8)] {
        let sh = Shared::new();
        seed(&sh);
        seed_estimate(&sh);
        sh.table.with_mut(|t| {
            t.control.lifecycle.torque_enable = false;
            t.calib.thermal.ntc_k1_q88 = k1_q88;
        });
        let mut k = kernel();
        let dead = SensorFrame {
            ntc_raw,
            ..frame(2000, BIAS)
        };
        never_drives(&mut k, &sh, dead, 1);
        assert_eq!(k.faults.mask(), faults::BIT_OVER_TEMP, "{ntc_raw}");
        assert_eq!(k.faults.code(), faults::CODE_OVER_TEMP);
        never_drives(&mut k, &sh, dead, 2 * SLOW);
        let (t, flags, i_lim, _) = therm(&sh);
        assert_eq!((t, flags, i_lim), (i16::MAX, flag::NTC_FAULT, 1200));
        // the enable, taken at every phase of the SLOW block in turn
        for offset in [0, 1, SLOW / 2, SLOW - 1] {
            sh.table
                .with_mut(|t| t.control.lifecycle.torque_enable = false);
            never_drives(&mut k, &sh, dead, SLOW + offset);
            sh.table
                .with_mut(|t| t.control.lifecycle.torque_enable = true);
            never_drives(&mut k, &sh, dead, 3 * SLOW);
            assert_eq!(k.faults.mask(), faults::BIT_OVER_TEMP, "{ntc_raw}");
            assert_eq!(k.faults.code(), faults::CODE_OVER_TEMP);
        }
        // the NTC back at 25.00 C on the right curve
        write_config(&sh, |t| {
            t.calib.thermal.ntc_k1_q88 = 563;
            t.control.lifecycle.torque_enable = false;
        });
        let good = frame(2000, BIAS);
        never_drives(&mut k, &sh, good, 2 * SLOW);
        assert_eq!(therm(&sh).0, 2500);
        assert_eq!(
            k.faults.mask(),
            faults::BIT_OVER_TEMP,
            "latched until an ack"
        );
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        settle(&mut k, &sh, good);
        assert_eq!(k.faults.mask(), 0);
        assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    }
}

/// An NTC that fails mid-hold: the next finish reads i16::MAX and the
/// fault latches, the bridge off with it.
#[test]
fn a_rail_mid_hold_latches_on_the_next_finish() {
    use crate::estimator::thermal::flag;
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    let mut k = kernel();
    let hot = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, HEAT_SETTLE);
    assert_eq!(therm(&sh).0, 5331);
    assert_eq!(k.faults.mask(), 0);
    let dead = SensorFrame {
        ntc_raw: 4090,
        ..hot
    };
    ticks(&mut k, &sh, dead, 2 * SLOW);
    let (t, flags, i_lim, t_ntc) = therm(&sh);
    assert_eq!(
        (t, flags, i_lim, t_ntc),
        (i16::MAX, flag::NTC_FAULT, 1200, 2500)
    );
    assert_eq!(k.faults.mask(), faults::BIT_OVER_TEMP);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
}

/// A 400-count hold settles the heat and the warm input it feeds back at
/// their joint fixed point: the cold square 160000 raised by alpha x the
/// rise it produces, 177769 counts squared at 28.3 C over, and the estimate
/// reads the NTC plus the rise on it: 2500 + (177769 x 1044 >> 16) = 5331.
/// The cold square alone would read 5048; the copper 11 % warmer than
/// cold adds the rest. Under the cutoff the estimate shapes nothing: the
/// ceiling is the cap.
#[test]
fn the_estimate_reads_the_ntc_plus_the_rise_on_the_heat() {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    let mut k = kernel();
    let hot = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, HEAT_SETTLE);
    assert_eq!(k.fast.i_meas_last(), 400);
    let (t, flags, i_lim, t_ntc) = therm(&sh);
    assert_eq!((t, flags, t_ntc), (5331, 0, 2500));
    assert_eq!(i_lim, 1200, "under the cutoff");
    assert_eq!(limit_flags(&sh), 0);
    assert_eq!(k.faults.mask(), 0);
}

/// The body node adds its gain on its own leak: with the step-down's
/// candidate body gain (825) on a 16 s leak (shift 10) the same hold reads
/// the rotor's 5331 plus the body's share, most of it after 2.5 of its
/// tau and all of it after 7, each node's heat settled at the joint fixed
/// point (194880 counts squared, 2500 + (194880 x 1869 >> 16) = 8056 up to
/// the EWMA's dead band); a zero `thermal_tau2_s` turns the node off
/// whatever its gain.
#[test]
fn the_body_node_adds_its_gain_on_its_own_leak_and_a_zero_tau_turns_it_off() {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    write_config(&sh, |t| {
        t.calib.thermal.th_rise2_q016 = 825;
        t.config.thermal.thermal_tau2_s = 16;
    });
    let mut k = kernel();
    let hot = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, HEAT_SETTLE);
    let (t_mid, flags, _, _) = therm(&sh);
    ticks(&mut k, &sh, hot, 5 * HEAT_SETTLE);
    let (t_end, _, _, _) = therm(&sh);
    assert_eq!((t_mid, t_end, flags), (7767, 8056, 0));
    write_config(&sh, |t| t.config.thermal.thermal_tau2_s = 0);
    ticks(&mut k, &sh, hot, HEAT_SETTLE);
    let (t, flags, _, _) = therm(&sh);
    assert_eq!((t, flags), (5331, 0));
}

/// Torque off: the held current drops to zero and the heat leaks on tau.
/// Five tau (1280 SLOW periods) leave 1177 counts squared of the hot
/// hold's heat, 2518.
#[test]
fn torque_off_lets_the_heat_leak() {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    let mut k = kernel();
    let hold = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hold);
    ticks(&mut k, &sh, hold, HEAT_SETTLE);
    assert_eq!(therm(&sh).0, 5331);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    ticks(&mut k, &sh, frame(2000, BIAS), 1280 * SLOW);
    let (t, flags, i_lim, t_ntc) = therm(&sh);
    assert_eq!((t, flags, t_ntc), (2518, 0, 2500));
    assert_eq!(i_lim, 1200);
    assert_eq!(k.faults.mask(), 0);
}

/// The OpenLoop hold with no rise (`th_rise_q016` 0, so the warm factor is
/// 0) on a one-period leak (`thermal_tau_s` 0, shift 0): the rotor heat
/// after each due reads that cycle's mean square exactly.
fn bare_heat_rig(sh: &Shared) -> Kernel<FakeIo> {
    seed(sh);
    seed_estimate(sh);
    write_config(sh, |t| {
        t.calib.thermal.th_rise_q016 = 0;
        t.config.limits.thermal_tau_s = 0;
    });
    kernel()
}

/// Ticks `f` until the next tick is the CONTROL tick of the period `n`
/// periods after a SLOW due.
fn to_slow_period(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame, n: u8) {
    k.on_scan(f, sh);
    while !(k.phase == phase::CONTROL && k.medium.slow_periods == n) {
        k.on_scan(f, sh);
    }
}

/// The heat reads the SLOW cycle's mean of the held current squared, not
/// one sample of it: a hold whose current alternates 400 counts and 0
/// every MEDIUM period heats as 400^2 / 2 = 80000 counts squared on every
/// cycle, whichever parity of the cycle's periods carries the current.
#[test]
fn a_current_alternating_each_period_heats_as_half_its_square_at_either_phase() {
    let hot = frame(2000, BIAS + 400);
    let cold = frame(2000, BIAS);
    for parity in [0, 1] {
        let sh = Shared::new();
        let mut k = bare_heat_rig(&sh);
        settle(&mut k, &sh, hot);
        to_slow_period(&mut k, &sh, hot, 0);
        for due in 0..8 {
            for _ in 0..DECIM_SLOW {
                let (f, i) = if k.medium.slow_periods % 2 == parity {
                    (hot, 400)
                } else {
                    (cold, 0)
                };
                periods(&mut k, &sh, f, 1);
                assert_eq!(k.fast.i_meas_last(), i);
            }
            assert_eq!(k.medium.slow_periods, 0);
            assert_eq!(
                k.medium.limits.heats()[0],
                80_000,
                "parity {parity}, due {due}"
            );
        }
        assert_eq!(k.fast.duty_q15, 8000, "the hold drives");
    }
}

/// With nothing driving the held current is 0, so the cycle's mean falls
/// on its own: torque off 15 periods into a cycle of a 400-count hold
/// leaves that cycle's due the 15 driven periods' share, 15 x 160000 / 32
/// = 75000 counts squared, and the next due 0, the shunt still reading
/// 400 counts on every frame.
#[test]
fn torque_off_drops_the_heat_mean_within_one_cycle() {
    let hot = frame(2000, BIAS + 400);
    let sh = Shared::new();
    let mut k = bare_heat_rig(&sh);
    settle(&mut k, &sh, hot);
    slow_periods(&mut k, &sh, hot, 2);
    to_slow_period(&mut k, &sh, hot, 0);
    assert_eq!(k.medium.limits.heats()[0], 160_000);
    to_slow_period(&mut k, &sh, hot, 15);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    to_slow_period(&mut k, &sh, hot, 0);
    assert_eq!(k.medium.limits.heats()[0], 75_000);
    assert_eq!(k.fast.i_meas_last(), 0);
    to_slow_period(&mut k, &sh, hot, 1);
    to_slow_period(&mut k, &sh, hot, 0);
    assert_eq!(k.medium.limits.heats()[0], 0);
}

/// One thermal limit: a hold at the 1200-count cap heads for a fixed point
/// far over the 90.00 C cutoff, and until the estimate reads the cutoff
/// nothing takes current away: the ceiling is the cap and no term binds on
/// every period. The heats step on a finished cycle's mean, so the first
/// due has no current behind it: the 75th finish, 1.2 s in, reads 90.38 C
/// after 89.51 C, latches the fault, and the bridge is off from that tick.
/// The winding
/// then cools with the fault standing: an ack while the estimate still
/// reads the cutoff or more never drives, and once it reads less an ack
/// clears and the hold drives again.
#[test]
fn a_hold_that_heats_to_the_cutoff_runs_at_the_cap_and_ends_in_the_latch() {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    write_config(&sh, |t| {
        t.config.thermal.cutoff_cc = 9000;
        // the collision trip off, its boot value: under the rig's
        // 1200-count trip this hold on a still pot sets YIELD
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let hold = frame(2000, BIAS + 1200);
    let mut finishes = 0u32;
    let mut last_under = 0;
    while k.faults.mask() == 0 {
        ticks(&mut k, &sh, hold, SLOW);
        finishes += 1;
        assert!(finishes < 10_000, "never latched");
        let (t, flags, i_lim, _) = therm(&sh);
        if k.faults.mask() == 0 {
            assert!(t < 9000 && t >= last_under, "{t}");
            last_under = t;
            assert_eq!((flags, i_lim), (0, 1200));
            assert_eq!(limit_flags(&sh) & !limits::flag::CEILING, 0);
            assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
        }
    }
    let t_latch = therm(&sh).0;
    assert_eq!((finishes, last_under, t_latch), (75, 8951, 9038));
    assert_eq!(k.faults.mask(), faults::BIT_OVER_TEMP);
    assert_eq!(k.faults.code(), faults::CODE_OVER_TEMP);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));

    // the bridge is off: the winding carries nothing and cools. An ack
    // inside the period of the latch, the estimate still at the cutoff
    let rest = frame(2000, BIAS);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    never_drives(&mut k, &sh, rest, DECIM_MED as u32);
    assert!(therm(&sh).0 >= 9000);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    never_drives(&mut k, &sh, rest, DECIM_MED as u32);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_TEMP, "the ack relatches");
    assert_eq!(k.faults.code(), faults::CODE_OVER_TEMP);
    for _ in 0..4 {
        never_drives(&mut k, &sh, rest, SLOW);
    }
    assert!(therm(&sh).0 < 9000);
    assert_eq!(
        k.faults.mask(),
        faults::BIT_OVER_TEMP,
        "latched until an ack"
    );
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    never_drives(&mut k, &sh, rest, SLOW);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    settle(&mut k, &sh, rest);
    assert_eq!(k.faults.mask(), 0);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
}

fn r_hat(sh: &Shared) -> u16 {
    sh.table.with(|t| t.telemetry.estimates.r_hat_q12)
}

/// Ticks `f` through the next SLOW finish and the PUBLISH phase behind it.
fn to_finish(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame) {
    k.on_scan(f, sh);
    while !(k.medium.slow_periods == 1 && k.phase == phase::PUBLISH + 1) {
        k.on_scan(f, sh);
    }
}

/// The resistance the control path reads is the stored one wherever the
/// heat model adds nothing: from the boot tick with no NTC curve, through
/// a hot hold there; on a curve with nothing held; and on a failed NTC,
/// whose estimate feeds no rise back, the finish after a settled hold had
/// it at 2.222 vcounts per count.
#[test]
fn r_hat_is_the_stored_resistance_with_no_heat_unset_or_on_a_failed_ntc() {
    use crate::estimator::thermal::flag;
    let hot = frame(2000, BIAS + 400);
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    write_config(&sh, |t| t.calib.thermal.ntc_k1_q88 = 0);
    let mut k = kernel();
    k.on_scan(hot, &sh);
    assert_eq!(r_hat(&sh), 8192, "the boot tick");
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, HEAT_SETTLE);
    assert_eq!(k.fast.i_meas_last(), 400);
    assert_eq!((therm(&sh).1, r_hat(&sh)), (flag::UNSET, 8192));

    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    let mut k = kernel();
    ticks(&mut k, &sh, frame(2000, BIAS), BASE_FAST_TICKS);
    assert_eq!((therm(&sh).0, r_hat(&sh)), (2500, 8192), "nothing held");

    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, HEAT_SETTLE);
    assert_eq!((therm(&sh).0, r_hat(&sh)), (5331, 9103));
    let dead = SensorFrame {
        ntc_raw: 4090,
        ..hot
    };
    to_finish(&mut k, &sh, dead);
    assert_eq!((therm(&sh).1, r_hat(&sh)), (flag::NTC_FAULT, 8192));
}

/// A held current raises the resistance with the estimate, finish by
/// finish, to the value the warm factor gives: the 400-count hold settles
/// 28.31 C over the NTC, where copper is 0.393 % x 28.31 = 11.12 % over
/// cold (7290 in Q0.16), and 2.0 vcounts per count reads 8192 + (8192 x
/// 7290 >> 16) = 9103, 2.222. Torque off, it falls back with the heat:
/// five tau on, 0.18 C over reads 8197.
#[test]
fn r_hat_rises_with_a_held_current_to_the_warm_factors_value_and_falls_as_it_cools() {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    let mut k = kernel();
    let hot = frame(2000, BIAS + 400);
    k.on_scan(hot, &sh);
    let mut last = r_hat(&sh);
    assert_eq!(last, 8192, "the boot tick");
    settle(&mut k, &sh, hot);
    for _ in 0..HEAT_SETTLE / SLOW {
        ticks(&mut k, &sh, hot, SLOW);
        let r = r_hat(&sh);
        assert!(r >= last, "{r} after {last}");
        last = r;
    }
    assert_eq!(therm(&sh).0, 5331);
    assert_eq!(k.medium.thermal.warm_q16(), 7290);
    assert_eq!(r_hat(&sh), 9103);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    ticks(&mut k, &sh, frame(2000, BIAS), 1280 * SLOW);
    assert_eq!((therm(&sh).0, r_hat(&sh)), (2518, 8197));
}

/// A new `r_q12` is the one warmed from the configuration take on, a
/// period after its commit and with no finish in between: at the settled
/// hold's 11.12 % a write of 1.0 reads 4096 + 455, a write the factor
/// carries past the field saturates at its top, an unidentified winding
/// reads 0 at any heat, and the first value back reads 9103 again.
#[test]
fn r_hat_follows_a_new_r_q12_at_the_take_and_saturates() {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    let mut k = kernel();
    let hot = frame(2000, BIAS + 400);
    settle(&mut k, &sh, hot);
    ticks(&mut k, &sh, hot, HEAT_SETTLE);
    to_finish(&mut k, &sh, hot);
    assert_eq!(r_hat(&sh), 9103);
    for (r_q12, want) in [(4096, 4551), (60000, u16::MAX), (0, 0), (8192, 9103)] {
        write_config(&sh, |t| t.calib.motor.r_q12 = r_q12);
        periods(&mut k, &sh, hot, 2);
        assert_eq!(k.cfg.motor.r_q12, r_q12, "taken");
        assert_eq!(r_hat(&sh), want, "{r_q12}");
    }
    assert!(
        (2..DECIM_SLOW).contains(&k.medium.slow_periods),
        "a finish ran: the take alone did not move it"
    );
}

/// The back-EMF boxcar reads the same speed on a warm winding as on a
/// cold one. The frames stand in for the plant: at one duty and one speed
/// the window's volts and the back-EMF are the same, so a winding an
/// eighth over cold carries eight ninths of the current, and the drop the
/// boxcar subtracts is the same 360 vcounts, cold 2.0 x 180 counts and
/// warm 2.25 x 160. A rise gain of 7246 settles the 160-count hold 31.83
/// C over the NTC, where the factor is an eighth and `r_hat` reads 9216.
/// Both read (293 x 2960 / 1200 - 360) x 16 = 5803 c/s; the stored
/// resistance alone reads the warm winding 0.25 x 160 x 16 = 640 c/s
/// fast.
#[test]
fn the_boxcar_reads_a_warm_winding_at_the_speed_it_read_cold() {
    let read = |i: u16, th_rise_q016: u16| {
        let sh = Shared::new();
        seed(&sh);
        seed_estimate(&sh);
        write_config(&sh, |t| t.calib.thermal.th_rise_q016 = th_rise_q016);
        let mut k = kernel();
        let f = frame(2000, BIAS + i);
        settle(&mut k, &sh, f);
        ticks(&mut k, &sh, f, HEAT_SETTLE);
        run_to(&mut k, &sh, f, phase::PUBLISH + 1);
        let omega = sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps);
        (therm(&sh).0, r_hat(&sh), omega)
    };
    assert_eq!(read(180, 0), (2500, 8192, 5803), "cold");
    assert_eq!(read(160, 7246), (5683, 9216, 5803), "warm");
    assert_eq!(read(160, 0), (2500, 8192, 6443), "warm on the stored R");
}

/// A current reference held under the shunt floor on a warm winding: a
/// 40-count limit under a rise gain at the top of its field, which settles
/// the held reference 16.57 C over the NTC, `r_hat` 8725.
fn warm_blind_rig(sh: &Shared) -> Kernel<FakeIo> {
    seed(sh);
    seed_estimate(sh);
    write_config(sh, |t| {
        t.calib.thermal.th_rise_q016 = u16::MAX;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 40;
        t.config.limits.current_limit_counts = 40;
    });
    let mut k = kernel();
    let rest = frame(2000, BIAS);
    to_first_command(&mut k, sh, rest);
    k.on_scan(rest, sh);
    // cold: 2.0 x 40 counts = 80 vcounts = 32 ticks
    assert_eq!((r_hat(sh), k.fast.ticks), (8192, 32));
    assert_eq!(k.fast.duty_q15, blind_duty(&k, 8192));
    ticks(&mut k, sh, rest, HEAT_SETTLE);
    assert_eq!((therm(sh).0, r_hat(sh)), (4157, 8725));
    k
}

/// On a tick with no reading the loop commands the reference through the
/// resistance at the estimate's temperature, `r_hat x i_ref`: warm, 2.130
/// x 40 counts = 85 vcounts = 34 ticks where the stored 2.0 commanded 32,
/// tick for tick as the reference stands. An unidentified winding keeps
/// the loop's own gain as the take chose it, unscaled by the heat.
#[test]
fn under_the_floor_a_warm_winding_is_commanded_through_r_hat() {
    let sh = Shared::new();
    let mut k = warm_blind_rig(&sh);
    let rest = frame(2000, BIAS);
    assert_eq!(k.cmd.r_loop_q12, 8725);
    for _ in 0..200 {
        k.on_scan(rest, &sh);
        assert_eq!(k.fast.duty_q15, blind_duty(&k, 8725));
        assert_eq!(k.fast.ticks, 34);
    }
    assert_eq!(k.fast.i_meas_last, 40);
    // 1.5 x 40 counts = 60 vcounts = 24 ticks, the estimate still warm
    write_config(&sh, |t| {
        t.calib.motor.r_q12 = 0;
        t.config.loop_current.i_kp_q88 = 384;
    });
    periods(&mut k, &sh, rest, 2);
    assert_eq!((r_hat(&sh), k.cmd.r_loop_q12), (0, 384 << 4));
    assert_eq!(k.fast.duty_q15, blind_duty(&k, 384 << 4));
    assert_eq!(k.fast.ticks, 24);
    assert!(therm(&sh).0 > 4000, "{}", therm(&sh).0);
}

/// OpenLoop's stall-safe base duty is a ceiling, so it stays on the
/// stored resistance, which a warm winding only ever exceeds: with
/// `r_hat` at 8725 the base is still the 40-count limit through 2.0, 80
/// vcounts of the 3000 rail.
#[test]
fn the_stall_safe_open_loop_duty_stays_on_the_stored_resistance() {
    let sh = Shared::new();
    let k = warm_blind_rig(&sh);
    assert_eq!(r_hat(&sh), 8725);
    let cold = q_mul(80, k.cmd.vbus_recip_q15 as i32, 15);
    assert_eq!((k.cmd.ol_base_q15 as i32, cold), (873, 873));
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
    // raised in the medium step, after this tick's write: the next is off
    k.on_scan(frame(500, BIAS), &sh);
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

/// Control periods for the bias tracker to settle a step to the count: 12
/// time constants, one sample a period.
const BIAS_SETTLE_PERIODS: u32 = 12 << bias::ALPHA_SHIFT;

fn ticks(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame, n: u32) {
    for _ in 0..n {
        k.on_scan(f, sh);
    }
}

/// `n` control periods of `f`.
fn periods(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame, n: u32) {
    ticks(k, sh, f, n * DECIM_MED as u32);
}

/// `n` SLOW periods of `f`.
fn slow_periods(k: &mut Kernel<FakeIo>, sh: &Shared, f: SensorFrame, n: u32) {
    ticks(k, sh, f, n * SLOW);
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
    // full scale: the idle leg never leaves CCR 0, no off phase. The slew
    // up to it passes through brake troughs, so it runs on the bias.
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = i16::MAX;
    });
    settle(&mut k, &sh, frame(2000, BIAS));
    ticks(&mut k, &sh, shifted(), 3000);
    assert_eq!(drive_duty(&k), Some(i16::MAX));
    assert_eq!(published_bias(&sh), BIAS);
    // a partial window: the trough is the brake phase, the tracker follows
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 8000);
    periods(&mut k, &sh, shifted(), BIAS_SETTLE_PERIODS);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
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
    // offset walks 40 counts at 1 count per 4096 ticks, 409.6 periods: 2.5
    // counts of lag at the 1024-period time constant (the bench drift, 8
    // counts per degree C, would need a degree every 1.6 s); the true
    // current is a constant 300 counts above it
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
    periods(&mut k, &sh, shifted(), BIAS_SETTLE_PERIODS);
    assert_eq!(published_bias(&sh), BIAS + 500);
}

/// The board warms with torque off and the bias follows the rest. The rest
/// walks 200 counts down at 1 count per 1024 ticks, 102.4 control periods.
/// A first-order tracker follows a ramp one time constant behind: 1024
/// periods at 1 count per 102.4 periods is 10 counts of lag. The walk's
/// whole-count steps put the lag half a count either side of that, 9.5
/// just before a step and 10.5 just after, and the published zero rounds
/// to the count, half a count more: settled, the lag reads 9 to 11, and
/// never more on the way there.
#[test]
fn torque_off_bias_follows_a_drifting_rest() {
    /// The tracker's time constant in ticks.
    const TAU_TICKS: u32 = (1 << bias::ALPHA_SHIFT) * DECIM_MED as u32;
    /// Ticks per count of the walk.
    const WALK_SHIFT: u32 = 10;
    const LAG: u16 = (TAU_TICKS >> WALK_SHIFT) as u16;
    const _: () = assert!(LAG == 10);
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for n in 0..200u32 << WALK_SHIFT {
        let b = BIAS - (n >> WALK_SHIFT) as u16;
        k.on_scan(rest(b), &sh);
        if n >= 1 << WALK_SHIFT {
            let p = published_bias(&sh);
            assert!(p >= b && p - b <= LAG + 1, "tick {n}: bias {p}, rest {b}");
            // 8 time constants in, the lag has settled to within the count
            if n >= 8 * TAU_TICKS {
                assert!(p - b >= LAG - 1, "tick {n}: bias {p}, rest {b}");
            }
        }
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // never awake: no step learned, the bias is the rest itself
    periods(&mut k, &sh, rest(BIAS - 200), BIAS_SETTLE_PERIODS);
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
    periods(&mut k, &sh, rest(BIAS + 10), BIAS_SETTLE_PERIODS);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(published_bias(&sh), BIAS + 10);
    // torque off past the stale window, then the board warms: the bias
    // keeps the step over the rest
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    slow_periods(&mut k, &sh, rest(BIAS), bias::STALE_PERIODS as u32);
    assert_eq!(published_bias(&sh), BIAS + 10);
    periods(&mut k, &sh, rest(BIAS - 200), BIAS_SETTLE_PERIODS);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    assert_eq!(published_bias(&sh), BIAS - 190);
    // awake again, coasting on a zero goal: the zero it reads is the one
    // already in use, so nothing moves
    write_config(&sh, |t| t.config.limits.openloop_zero_brake = false);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..BIAS_SETTLE_PERIODS * DECIM_MED as u32 {
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
    slow_periods(&mut k, &sh, driven, bias::STALE_PERIODS as u32);
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
        periods(&mut k, &sh, rest(BIAS), BIAS_SETTLE_PERIODS);
        assert_eq!(published_bias(&sh), BIAS);
        periods(&mut k, &sh, rest(rest_counts), BIAS_SETTLE_PERIODS);
        assert_eq!(published_bias(&sh), rest_counts);
        periods(&mut k, &sh, rest(BIAS), BIAS_SETTLE_PERIODS);
        sh.table.with_mut(|t| {
            t.control.lifecycle.torque_enable = true;
            t.control.lifecycle.goal_duty = i16::MAX;
        });
        settle(&mut k, &sh, slew);
    }
}

/// The tracker takes one sample a control period, on the PUBLISH tick, and
/// none on the other nine phases, asleep and under drive alike: against a
/// trough far from the zero every sample moves the zero in use by a count
/// or more, so it steps on the PUBLISH ticks alone, by what one sample
/// moves the tracker, and the same tick's publish carries it.
#[test]
fn the_zero_takes_one_sample_a_period_at_publish() {
    const PERIODS: u32 = 5;
    for (torque, zero) in [(false, bias::Zero::Asleep), (true, bias::Zero::Awake)] {
        let sh = Shared::new();
        seed(&sh);
        sh.table.with_mut(|t| {
            t.control.lifecycle.torque_enable = torque;
            t.control.lifecycle.mode = Mode::OpenLoop;
            t.control.lifecycle.goal_duty = 8000;
        });
        let mut k = kernel();
        // the zero's own value until the bridge state has settled: torque
        // off, or a Slow drive whose trough is a wide brake half
        let near = frame(2000, BIAS + 300);
        if torque {
            settle(&mut k, &sh, near);
        }
        periods(&mut k, &sh, near, fast::ZERO_SETTLE_TICKS as u32);
        assert_eq!(k.fast.bias_counts(), BIAS);
        let mut far = near;
        far.current_trough = BIAS - 1600;
        let mut model = bias::BiasTracker::new();
        model.seed(BIAS);
        let mut samples = 0;
        for tick in 0..PERIODS * DECIM_MED as u32 {
            let next = k.phase;
            let before = k.fast.bias_counts();
            k.on_scan(far, &sh);
            let after = k.fast.bias_counts();
            if next == phase::PUBLISH {
                samples += 1;
                assert_ne!(after, before, "{zero:?} tick {tick}");
                assert_eq!(
                    after,
                    model.update(zero, (far.current_trough as u32) << adc::ADC_Q4_SHIFT),
                    "{zero:?} tick {tick}"
                );
                assert_eq!(published_bias(&sh), after, "{zero:?} tick {tick}");
            } else {
                assert_eq!(after, before, "{zero:?} tick {tick}");
            }
        }
        assert_eq!(samples, PERIODS, "{zero:?}");
        assert_eq!(drive_duty(&k).is_some(), torque);
    }
}

/// An unfed zero ages one step a SLOW period, at the block's finish, and
/// is stale after `STALE_PERIODS` of them. The brake feeds the awake zero,
/// then torque goes off. Through the last SLOW period of the horizon the
/// rest sits far off, so every sample moves the asleep zero by a count or
/// more: the awake zero in use holds until the finish that ends the
/// horizon and moves with the next sample.
#[test]
fn an_unfed_zero_is_stale_after_stale_periods_of_slow() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 0;
        t.config.limits.openloop_zero_brake = true;
    });
    let mut k = kernel();
    // the last awake sample is the PUBLISH ahead of the CONTROL phase that
    // reads torque off
    periods(&mut k, &sh, rest(BIAS), 100);
    assert!(matches!(last_cmd(&k), MotorCmd::Brake));
    assert_eq!(k.phase, phase::CONTROL);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    // one tick of `f`; whether it ran the SLOW block's finish
    let step = |k: &mut Kernel<FakeIo>, f: SensorFrame| {
        let before = k.probe_tags();
        k.on_scan(f, &sh);
        Events::between(before, k.probe_tags()).finish
    };
    let mut aged = 0;
    while aged < bias::STALE_PERIODS - 1 {
        aged += step(&mut k, rest(BIAS)) as u8;
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    let far = rest(BIAS - 1600);
    let mut samples = 0;
    loop {
        samples += (k.phase == phase::PUBLISH) as u32;
        let finish = step(&mut k, far);
        assert_eq!(k.fast.bias_counts(), BIAS, "sample {samples}");
        if finish {
            break;
        }
    }
    assert_eq!(samples, DECIM_SLOW as u32);
    assert_eq!(k.phase, phase::PUBLISH);
    k.on_scan(far, &sh);
    assert!(k.fast.bias_counts() < BIAS, "{}", k.fast.bias_counts());
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
    // invalid two ticks later (the CONTROL tick's write still drives the
    // last valid command, and the tick after measures its period) while
    // the drive still pushes
    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = 1000);
    for _ in 0..16 {
        k.on_scan(frame(2000, BIAS + 300), &sh);
    }
    let r = rows(&k);
    assert_eq!(r[..2], [(300, 2960, 293); 2]);
    assert!(r[2..].iter().all(|&row| row == (300, 2960, 37)), "{r:?}");
    assert!(
        k.tel.rows[2..]
            .iter()
            .all(|t| t.tick_flags & tick_flag::I_VALID == 0)
    );
}

#[test]
fn stream_current_reads_zero_once_nothing_drives() {
    let sh = Shared::new();
    let mut k = tel_settled_at_control(&sh);
    // disable: the first two ticks measure the periods the last drive
    // command drove (the CONTROL tick's write still drives it), the 14
    // after read no current; vdiff keeps its hold
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..16 {
        k.on_scan(frame(2000, BIAS + 200), &sh);
    }
    let r = rows(&k);
    assert_eq!(r[..2], [(200, 2960, 293); 2]);
    assert!(r[2..].iter().all(|&row| row == (0, 2960, 0)), "{r:?}");
    assert_eq!(k.fast.i_meas_last, 0);
}

// --- Back-EMF boxcar ------------------------------------------------------

#[test]
fn bemf_boxcar_lands_on_the_closed_form_after_20_ticks() {
    let sh = Shared::new();
    drive_setup(&sh);
    let mut k = kernel();
    // from the CONTROL phase after the slew every tick measures duty
    // 8000: drive_ticks 293, vdiff 2960, i 100. The half the SPEED phase
    // closes 10 ticks later is the first clean one, 20 ticks pairs it with
    // a second, and the PUBLISH phase after it lands the boxcar.
    settle(&mut k, &sh, frame(2000, BIAS + 100));
    run_to(&mut k, &sh, frame(2000, BIAS + 100), phase::CONTROL);
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
    // last drive; the tick after it measures sub-floor, in the half that
    // period's SPEED phase closes, so the boxcar the PUBLISH phase lands in
    // between is the clean one closed ahead of the edge, and the next
    // period's PUBLISH drops to 0
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..2 * DECIM_MED - 1 {
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
        // the slew ran under the floor: the start-up half, then the
        // boxcar's two
        for _ in 0..30 {
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

/// A back-EMF row as the fast half summed it: the width, the terminal
/// differential and the current of a tick's window.
type BemfRow = (u32, Option<i32>, Option<i32>);

/// The back-EMF chain with the half's close on the OBSERVER tick itself,
/// ahead of the blend.
struct CloseAtObserver {
    obs: bemf::BemfObs,
    blend: OmegaBlend,
    boxcar: Option<i32>,
    /// The duty that drove the last row sampled.
    duty: i16,
}

impl CloseAtObserver {
    fn new() -> Self {
        Self {
            obs: bemf::BemfObs::new(TIMING.brush_dev_max),
            blend: OmegaBlend::new(),
            boxcar: None,
            duty: 0,
        }
    }

    fn sample(&mut self, (ticks, vdiff, i): BemfRow, duty: i16) {
        self.obs.sample(ticks, vdiff, i);
        self.duty = duty;
    }

    /// The kernel's close on the operands this tick reads and the duty of
    /// the half's last row, then the blend on the kernel's own observer
    /// speed.
    fn observe(&mut self, k: &Kernel<FakeIo>, sh: &Shared) -> i32 {
        self.boxcar = self.obs.close(
            self.duty,
            r_hat(sh),
            0,
            k.cfg.motor.recip_ke_q,
            TIMING.recip_arr_q24,
        );
        self.blend
            .step(self.boxcar, self.obs.flipped(), k.medium.fusion.omega_q16())
    }
}

/// The boxcar closes on SPEED, two ticks ahead of the OBSERVER that reads
/// it, and its place changes nothing else: the speed OBSERVER hands on and
/// the boxcar PUBLISH lands are the ones a close on the OBSERVER tick
/// itself gives on the same rows two ticks late, with the resistance and
/// the Ke reciprocal as that tick reads them. A half is still ten rows,
/// each in one half: the rows of phases 0 to 9 of one period. The same
/// close on the rows as they come hands on another speed, so the two
/// ticks show. The plant runs up, reverses, crawls under the floors and
/// stops, a 160-tick back-EMF floor over the 100-tick windows, with a
/// resistance and a Ke write landing mid run.
#[test]
fn the_observer_reads_the_boxcar_speed_closed_two_ticks_ahead() {
    const BEMF_FLOOR: u16 = 160;
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.config.pos_soft.pos_max_soft_counts = 1_000_000;
    });
    let mut k = Kernel::new(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        KernelTiming {
            bemf_min_ticks: BEMF_FLOOR,
            ..TIMING
        },
    );
    let mut plant = Plant::new(2000);
    let (mut late, mut now) = (CloseAtObserver::new(), CloseAtObserver::new());
    // ahead of the first tick no window read
    let mut held: [(BemfRow, i16); 2] = [((0, None, None), 0); 2];
    let (mut boxcars, mut voids, mut moved) = (0u32, 0u32, 0u32);
    let mut sources = [false; 2];
    let mut void_at = [false; DECIM_MED as usize];
    for tick in 0..9000u32 {
        match tick {
            0 => sh
                .table
                .with_mut(|t| t.control.lifecycle.goal_velocity = 2500),
            3000 => sh
                .table
                .with_mut(|t| t.control.lifecycle.goal_velocity = -2500),
            4003 => write_config(&sh, |t| {
                t.calib.motor.r_q12 = 7000;
                t.calib.motor.recip_ke_q = 15 << 10;
            }),
            6000 => sh
                .table
                .with_mut(|t| t.control.lifecycle.goal_velocity = 200),
            8000 => {
                sh.table
                    .with_mut(|t| t.control.lifecycle.torque_enable = false);
                // the edge lands on this tick's CONTROL and clears the blend
                late.blend.reset();
                now.blend.reset();
            }
            _ => {}
        }
        let next = k.phase;
        let drove = k.fast.duty_q15;
        let f = plant.step(drove);
        k.on_scan(f, &sh);
        let t = tick_block(&sh);
        let width = t.duty_ticks.unsigned_abs() as u32;
        let row: BemfRow = (
            width,
            (t.tick_flags & tick_flag::V_VALID != 0 && width >= BEMF_FLOOR as u32)
                .then_some(t.vdiff_counts as i32),
            (t.tick_flags & tick_flag::I_VALID != 0).then_some(t.i_tick_counts as i32),
        );
        void_at[next as usize] |= width != 0 && (row.1.is_none() || row.2.is_none());
        late.sample(held[0].0, held[0].1);
        held = [held[1], (row, drove)];
        now.sample(row, drove);
        if next == phase::OBSERVER {
            let speed = late.observe(&k, &sh);
            assert_eq!(k.medium.omega_hat, speed, "tick {tick}");
            moved += (now.observe(&k, &sh) != speed) as u32;
            sources[late.boxcar.is_some() as usize] = true;
            match late.boxcar {
                Some(_) => boxcars += 1,
                None => voids += 1,
            }
        }
        if next == phase::PUBLISH {
            let boxcar = bemf::omega_cps_i16(late.boxcar.unwrap_or(0));
            let published = sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps);
            assert_eq!(published, boxcar, "tick {tick}");
            let src = sh.table.with(|t| t.telemetry.mode.omega_hat_src);
            assert_eq!(src, late.boxcar.is_some() as u8, "tick {tick}");
        }
    }
    assert_eq!(k.faults.mask(), 0);
    assert_eq!(k.cfg.motor.r_q12, 7000, "the write was taken");
    assert!(
        boxcars > 300 && voids > 300,
        "{boxcars} boxcars, {voids} voids"
    );
    assert_eq!(sources, [true; 2], "both sources fed the loop");
    assert_eq!(
        void_at, [true; DECIM_MED as usize],
        "a void row on every phase"
    );
    assert!(moved > 0, "a close on the OBSERVER tick reads the same");
}

/// The back-EMF row the fast half just summed, off the tick block: no
/// differential under the back-EMF `floor`.
fn bemf_row(sh: &Shared, floor: u16) -> BemfRow {
    let t = tick_block(sh);
    let width = t.duty_ticks.unsigned_abs() as u32;
    (
        width,
        (t.tick_flags & tick_flag::V_VALID != 0 && width >= floor as u32)
            .then_some(t.vdiff_counts as i32),
        (t.tick_flags & tick_flag::I_VALID != 0).then_some(t.i_tick_counts as i32),
    )
}

/// The kernel's rows through the plain pairing of two valid halves, with
/// the valid halves in a row counted beside it.
struct PlainPairing {
    obs: bemf::BemfObs,
    valid: bool,
    run: u32,
}

impl PlainPairing {
    fn new() -> Self {
        Self {
            obs: bemf::BemfObs::new(TIMING.brush_dev_max),
            valid: true,
            run: 0,
        }
    }

    fn sample(&mut self, (ticks, vdiff, i): BemfRow) {
        self.valid &= vdiff.is_some() && i.is_some();
        self.obs.sample(ticks, vdiff, i);
    }

    /// The plain boxcar at this close, and the valid halves in a row up to
    /// and with the one it closes.
    fn close(&mut self, k: &Kernel<FakeIo>, sh: &Shared) -> (Option<i32>, u32) {
        self.run = if self.valid { self.run + 1 } else { 0 };
        self.valid = true;
        let boxcar = self
            .obs
            .close_half(r_hat(sh), k.cfg.motor.recip_ke_q, TIMING.recip_arr_q24);
        (boxcar, self.run)
    }
}

/// A plant held still: the taps and the shunt follow the duty's sign and
/// size, the pot stays.
fn held_plant() -> Plant {
    Plant {
        frozen: true,
        ..Plant::new(2000)
    }
}

/// One tick of `plant` under the duty that drives its window; the phase
/// the tick ran.
fn plant_tick(k: &mut Kernel<FakeIo>, sh: &Shared, plant: &mut Plant) -> u8 {
    let ran = k.phase;
    let f = plant.step(k.fast.duty_q15);
    k.on_scan(f, sh);
    ran
}

/// A boxcar waits out the start-up half: the first rows of a drive read
/// fast while the current rises, so the close hands on a boxcar only when
/// the half ahead of its two was valid as well. From rest the second valid
/// half closes a boxcar under the plain pairing and none here; from the
/// third on the two are the same boxcar. The slew crosses a swept back-EMF
/// floor, so the drive's first valid row lands on every phase, and the
/// first boxcar starts 10 to 19 rows behind it.
#[test]
fn a_boxcar_waits_out_the_startup_half() {
    let mut held_off = [false; 2 * DECIM_MED as usize];
    for floor in 100..=160u16 {
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
        let mut plant = held_plant();
        let mut plain = PlainPairing::new();
        let mut first_valid = None;
        let mut boxcars = 0;
        for tick in 0..120u32 {
            let ran = plant_tick(&mut k, &sh, &mut plant);
            let row = bemf_row(&sh, floor);
            if row.1.is_some() && row.2.is_some() {
                first_valid.get_or_insert(tick);
            } else {
                assert_eq!(first_valid, None, "floor {floor}: a void row in the drive");
            }
            plain.sample(row);
            if ran != phase::SPEED {
                continue;
            }
            let (pair, run) = plain.close(&k, &sh);
            let got = k.medium.omega_bemf;
            match run {
                0 | 1 => assert_eq!((got, pair), (None, None), "floor {floor} tick {tick}"),
                2 => {
                    assert!(pair.is_some(), "floor {floor}: the plain pairing reads");
                    assert_eq!(got, None, "floor {floor}: the start-up boxcar");
                }
                _ => {
                    assert!(got.is_some(), "floor {floor} tick {tick}");
                    assert_eq!(got, pair, "floor {floor} tick {tick}");
                    if boxcars == 0 {
                        // the boxcar's twenty rows end on this tick
                        let start = tick + 1 - bemf::BOXCAR_TICKS as u32;
                        held_off[(start - first_valid.unwrap()) as usize] = true;
                    }
                    boxcars += 1;
                }
            }
        }
        assert!(boxcars > 4, "floor {floor}: {boxcars} boxcars");
    }
    let want: [bool; 2 * DECIM_MED as usize] = core::array::from_fn(|n| n >= DECIM_MED as usize);
    assert_eq!(held_off, want, "rows held off, each of 10 to 19");
}

/// The next SPEED close on `plant`: the boxcar it handed on, and whether
/// it saw the drive flip.
fn to_close(k: &mut Kernel<FakeIo>, sh: &Shared, plant: &mut Plant) -> (Option<i32>, bool) {
    while plant_tick(k, sh, plant) != phase::SPEED {}
    (k.medium.omega_bemf, k.fast.bemf.flipped())
}

/// A half that holds a drive flip is void, and the next is a start-up
/// half. An OpenLoop reversal restarts at the window floor on the other
/// side, so every row through it reads but the reversal's first window,
/// which carries no reading, and the plain pairing hands on nothing on
/// the two closes that pair its half. Here the close that sees the duty's
/// sign differ from the last close's hands on nothing and flags the flip
/// for that one period; the two closes behind it hand on nothing, where
/// the plain pairing reads again on the second, and from the third on the
/// boxcar is the plain pairing's again, down to the speed the other way.
#[test]
fn a_half_that_holds_a_drive_flip_is_void() {
    const FROM: u32 = 200;
    let sh = Shared::new();
    drive_setup(&sh);
    let mut k = kernel();
    let mut plant = Plant::new(2000);
    let mut plain = PlainPairing::new();
    let mut closes: heapless::Vec<(Option<i32>, bool, Option<i32>), 64> = heapless::Vec::new();
    let mut signs = [false; 2];
    let mut flip_at = None;
    let mut reversed = false;
    for tick in 0..700u32 {
        if tick == 300 {
            sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -8000);
        }
        let ran = plant_tick(&mut k, &sh, &mut plant);
        let row = bemf_row(&sh, 0);
        plain.sample(row);
        let back = tick_block(&sh).duty_ticks < 0;
        signs[back as usize] = true;
        if tick >= FROM {
            // the first window of the reversal is no reading
            let first = back && !reversed;
            reversed |= back;
            assert_eq!(
                row.1.is_some() && row.2.is_some(),
                !first,
                "tick {tick}: {row:?}"
            );
        }
        if ran != phase::SPEED {
            continue;
        }
        let (pair, _) = plain.close(&k, &sh);
        if tick >= FROM + DECIM_MED as u32 {
            if signs == [true; 2] {
                assert_eq!(flip_at.replace(closes.len()), None, "one flip");
            }
            closes
                .push((k.medium.omega_bemf, k.fast.bemf.flipped(), pair))
                .unwrap();
        }
        signs = [false; 2];
    }
    let at = flip_at.expect("a half held both signs");
    assert!(
        at >= 5 && closes.len() > at + 20,
        "{at} of {}",
        closes.len()
    );
    for (n, &(got, flipped, pair)) in closes.iter().enumerate() {
        assert_eq!(
            pair.is_some(),
            !(at..at + 2).contains(&n),
            "close {n}: the plain pairing"
        );
        assert_eq!(flipped, n == at, "close {n}");
        let want = if (at..at + 3).contains(&n) {
            None
        } else {
            pair
        };
        assert_eq!(got, want, "close {n}");
    }
    assert!(closes[at - 1].0 > Some(1000), "{:?}", closes[at - 1]);
    let last = closes[closes.len() - 1];
    assert!(last.0.is_some_and(|w| w < -1000), "{last:?}");
}

/// A zero duty drives nothing and has no sign: a close on it is no flip,
/// and it leaves the sign a later drive is compared with at the last one
/// that drove. A first drive, either way, flips nothing; a drive that
/// stops and resumes the same way flips nothing; one that stops and
/// resumes the other way flips once, on the first close that drives. The
/// resumed drive's first boxcar comes on the same close either way: the
/// half the drive resumes in holds a zero row, so it is void already.
#[test]
fn a_zero_duty_has_no_sign_to_flip() {
    let goal = |sh: &Shared, duty: i16| sh.table.with_mut(|t| t.control.lifecycle.goal_duty = duty);
    for first in [8000i16, -8000] {
        let sh = Shared::new();
        drive_setup(&sh);
        goal(&sh, first);
        let mut k = kernel();
        let mut plant = Plant::new(2000);
        let mut last = None;
        for n in 0..20 {
            let (got, flipped) = to_close(&mut k, &sh, &mut plant);
            assert!(!flipped, "first drive {first}, close {n}");
            last = got;
        }
        assert_eq!(last.map(i32::signum), Some(first.signum() as i32));
        let mut waits = [0u32; 2];
        for (leg, resume) in [first, -first].into_iter().enumerate() {
            goal(&sh, 0);
            for n in 0..20 {
                let (got, flipped) = to_close(&mut k, &sh, &mut plant);
                assert!(!flipped, "{first} to zero, close {n}");
                assert!(n < 1 || got.is_none(), "{first} to zero, close {n}");
            }
            assert_eq!(k.fast.duty_q15, 0);
            goal(&sh, resume);
            let mut flips: heapless::Vec<u32, 20> = heapless::Vec::new();
            let mut first_boxcar = None;
            for n in 0..20 {
                let (got, flipped) = to_close(&mut k, &sh, &mut plant);
                if flipped {
                    flips.push(n).unwrap();
                }
                if got.is_some() {
                    first_boxcar.get_or_insert(n);
                }
                last = got;
            }
            let want: &[u32] = if resume == first { &[] } else { &[0] };
            assert_eq!(flips.as_slice(), want, "{first} then {resume}");
            assert_eq!(last.map(i32::signum), Some(resume.signum() as i32));
            waits[leg] = first_boxcar.expect("the resumed drive reads");
        }
        assert_eq!(waits, [3; 2], "first drive {first}");
    }
}

/// What the loop's speed carries over the observer's: the blend's lead.
fn lead(k: &Kernel<FakeIo>) -> i32 {
    k.medium.omega_hat - k.medium.fusion.omega_q16()
}

/// FAST ticks for the blend's standing part to form on a boxcar every
/// period: sixteen of its corners.
const STANDING_FORM_TICKS: u32 = (16 << omega_blend::D_SHIFT) * DECIM_MED as u32;

/// The loop's speed is the observer's plus the boxcar's fast part, and
/// `omega_hat_src` says whether a boxcar fed it this period. An OpenLoop
/// drive over a still pot, so the boxcar reads thousands of c/s over the
/// observer. Until a boxcar closes the loop reads the observer bit for bit
/// and the source reads 0. The first boxcar moves the loop's speed a
/// quarter of its gap, and that period's PUBLISH lands the loop's speed,
/// the source at 1 and the boxcar as it closed. Once the standing part
/// has formed, under a boxcar still 8363 c/s over the observer, the loop
/// reads within 16 c/s of the observer and the standing part within 16 c/s
/// of the gap, both as means over the blend's last corner of periods, since
/// on this still pot the observer's speed alternates period to period. The
/// torque edge clears the blend; the boxcar closed ahead of the edge still
/// feeds once, so the next enable edge has a standing part to clear, and
/// with no boxcar left the loop reads the observer bit for bit again.
#[test]
fn the_loops_speed_is_the_observers_plus_the_boxcars_fast_part() {
    let sh = Shared::new();
    drive_setup(&sh);
    let mut k = kernel();
    let f = frame(2000, BIAS + 100);
    let published = |sh: &Shared| {
        sh.table.with(|t| {
            (
                t.telemetry.estimates.omega_hat_cps,
                t.telemetry.mode.omega_hat_src,
                t.telemetry.estimates.omega_bemf_cps,
            )
        })
    };
    let mut n = 0;
    while k.medium.omega_bemf.is_none() {
        k.on_scan(f, &sh);
        assert_eq!(lead(&k), 0, "tick {n}");
        assert_eq!(published(&sh).1, 0, "tick {n}");
        n += 1;
        assert!(n < 100, "no boxcar closed");
    }
    let boxcar = k.medium.omega_bemf.unwrap();
    run_to(&mut k, &sh, f, phase::OBSERVER + 1);
    let gap = (boxcar << 16) - k.medium.fusion.omega_q16();
    assert!(gap > 1000 << 16, "a gap of {} c/s", gap >> 16);
    assert!(
        lead(&k) <= gap / 4 + (1 << 16) && lead(&k) > gap / 4 - gap / 64,
        "the first boxcar moved the loop {} of {gap}",
        lead(&k)
    );
    run_to(&mut k, &sh, f, phase::PUBLISH + 1);
    assert_eq!(published(&sh), (k.medium.omega_hat, 1, boxcar as i16));

    let corner = 1u32 << omega_blend::D_SHIFT;
    let (mut leads, mut gaps) = (0i64, 0i64);
    for n in 0..STANDING_FORM_TICKS {
        let ran = k.phase;
        k.on_scan(f, &sh);
        if ran == phase::OBSERVER && n >= STANDING_FORM_TICKS - corner * DECIM_MED as u32 {
            leads += lead(&k) as i64;
            gaps += ((8363 << 16) - k.medium.fusion.omega_q16()) as i64;
        }
    }
    assert_eq!(published(&sh).1, 1);
    assert_eq!(published(&sh).2, 8363);
    let lead_mean = leads / corner as i64;
    assert!(lead_mean.abs() < 16 << 16, "a mean lead of {lead_mean}");
    let standing = k.medium.omega_blend.standing_q16() as i64;
    let gap = gaps / corner as i64;
    assert!(
        (standing - gap).abs() < 16 << 16,
        "{standing} of a mean gap {gap}"
    );

    // torque off lands on the next CONTROL
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    run_to(&mut k, &sh, f, phase::CONTROL);
    k.on_scan(f, &sh);
    let cleared = |k: &Kernel<FakeIo>| {
        (
            k.medium.omega_blend.standing_q16(),
            k.medium.omega_blend.lead_q16(),
        )
    };
    assert_eq!(cleared(&k), (0, 0), "the torque edge");
    periods(&mut k, &sh, f, 80);
    assert_eq!(published(&sh), (k.medium.fusion.omega_q16(), 0, 0));
    assert_eq!(lead(&k), 0);
    assert_ne!(
        k.medium.omega_blend.standing_q16(),
        0,
        "the boxcar closed ahead of the edge fed once"
    );
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    run_to(&mut k, &sh, f, phase::CONTROL);
    k.on_scan(f, &sh);
    assert_eq!(cleared(&k), (0, 0), "the enable edge");
}

/// The park clears the blend with the velocity loop's integrator. A
/// position step on the plant forms a standing part on the way; from the
/// first parked VELOCITY on, every parked period leaves it and the lead at
/// zero, so the loop that leaves the park starts on the observer alone.
#[test]
fn the_park_clears_the_blend() {
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
    let (mut formed, mut fed, mut parked) = (0i32, 0u32, 0u32);
    for tick in 0..40_000u32 {
        let ran = plant_tick(&mut k, &sh, &mut plant);
        let (standing, c) = (
            k.medium.omega_blend.standing_q16(),
            k.medium.omega_blend.lead_q16(),
        );
        if k.medium.life != phases::Life::Parked {
            formed = formed.max(standing.abs());
            fed += (ran == phase::OBSERVER && k.medium.omega_bemf.is_some()) as u32;
        } else if ran == phase::VELOCITY {
            assert_eq!((standing, c), (0, 0), "tick {tick}");
            parked += 1;
        }
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(fed > 100, "{fed} boxcars fed the move");
    assert!(formed > 16 << 16, "a standing part of {formed} on the way");
    assert!(parked > 1000, "{parked} parked periods");
}

/// The standing part turns at the OBSERVER behind a drive flip. A still
/// plant whose shunt reads 20 counts under the winding's, so the boxcar
/// reads over 500 c/s along the drive over a pot that stays. Driven one
/// way the standing part forms at that gap. Through an OpenLoop reversal
/// the close flags one flip, the OBSERVER behind it negates the standing
/// part, bit for bit, and the boxcars of the drive the other way, as far
/// the other side of the observer, move the loop's speed under 16 c/s off
/// it; a standing part that kept its sign would hand it most of twice the
/// gap.
#[test]
fn the_standing_part_turns_at_the_observer_behind_a_flip() {
    const SHORT: u16 = 20;
    let sh = Shared::new();
    drive_setup(&sh);
    sh.table.with_mut(|t| {
        // under the continuous rating on this plant, so the press stays out
        t.control.lifecycle.goal_duty = 5000;
        // no model: the observer reads the still pot at rest, where the
        // rig's would read the held current as a speed for a second
        t.calib.motor.b_i_q313 = 0;
    });
    let mut k = kernel();
    let mut plant = held_plant();
    let mut tick = |k: &mut Kernel<FakeIo>| {
        let ran = k.phase;
        let duty = k.fast.duty_q15;
        let mut f = plant.step(duty);
        if duty != 0 {
            // the winding's current at this plant's 2.0 vcounts a count,
            // the same size either way round, less the shortfall
            let i = (duty.unsigned_abs() as u32 * plant.vbus as u32) >> 16;
            f.current = BIAS + i as u16 - SHORT;
        }
        k.on_scan(f, &sh);
        ran
    };
    let boxcar = |sh: &Shared| sh.table.with(|t| t.telemetry.estimates.omega_bemf_cps) as i32;
    for _ in 0..STANDING_FORM_TICKS {
        tick(&mut k);
    }
    let formed = k.medium.omega_blend.standing_q16();
    let over = boxcar(&sh);
    assert!(over > 500, "a boxcar of {over}");
    assert!((formed - (over << 16)).abs() < 16 << 16, "{formed}");
    assert!(lead(&k).abs() < 16 << 16, "a lead of {}", lead(&k));

    sh.table.with_mut(|t| t.control.lifecycle.goal_duty = -5000);
    let (mut flips, mut worst, mut fed) = (0u32, 0i32, 0u32);
    let mut before = None;
    for n in 0..4000u32 {
        let ran = tick(&mut k);
        worst = worst.max(lead(&k).abs());
        if ran == phase::SPEED && k.fast.bemf.flipped() {
            flips += 1;
            before = Some(k.medium.omega_blend.standing_q16());
        }
        if ran == phase::OBSERVER {
            if let Some(d) = before.take() {
                assert!(d > 0, "tick {n}: {d}");
                assert_eq!(k.medium.omega_blend.standing_q16(), -d, "tick {n}");
            }
            fed += (flips == 1 && k.medium.omega_bemf.is_some()) as u32;
        }
    }
    assert_eq!(flips, 1);
    assert!(fed > 300, "{fed} boxcars the other way");
    assert!(
        (boxcar(&sh) + over).abs() <= 16,
        "a boxcar of {}",
        boxcar(&sh)
    );
    assert!(worst < 16 << 16, "the loop's speed moved {worst} off");
}

// --- Current-loop feedforward ---------------------------------------------

#[test]
fn ke_feedforward_rides_the_profile_never_an_estimate() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        // current loop inert, no gain and no resistance to command the
        // reference through under the floor: the duty IS the Ke feedforward
        t.config.loop_current.i_kp_q88 = 0;
        t.config.loop_current.i_ki_q412 = 0;
        t.config.loop_current.i_kaw_q412 = 0;
        t.calib.motor.r_q12 = 0;
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

/// `Plant::step` per FAST tick: the duty adds `duty * PLANT_DUTY_GAIN >> 15`
/// c/s, omega decays by `omega >> PLANT_DECAY_SHIFT`, Coulomb friction takes
/// `PLANT_COULOMB_CPS` toward zero; the winding reads back-EMF
/// `omega >> PLANT_KE_SHIFT` vcounts over R = `1 << PLANT_R_SHIFT` vcounts
/// per ccount, on a `PLANT_VBUS` rail.
const PLANT_DUTY_GAIN: u32 = 200;
const PLANT_DECAY_SHIFT: u32 = 6;
const PLANT_COULOMB_CPS: u32 = 8;
const PLANT_KE_SHIFT: u32 = 4;
const PLANT_R_SHIFT: u32 = 1;
const PLANT_VBUS: u16 = 3000;

/// `Plant::step` in the kernel's motion model, current counts per MEDIUM
/// tick. The duty term is `DUTY_GAIN * (R i + e) / VBUS`, so per FAST tick
/// omega gains `B_f i` with `B_f = DUTY_GAIN R / VBUS` and loses
/// `(2^-DECAY_SHIFT - DUTY_GAIN 2^-KE_SHIFT / VBUS) omega + COULOMB_CPS`:
/// B is `DECIM_MED` of `B_f`, the frictions are their c/s terms over `B_f`.
const PLANT_B_I_Q313: u16 = (((PLANT_DUTY_GAIN << PLANT_R_SHIFT) * DECIM_MED as u32 * 8192
    + PLANT_VBUS as u32 / 2)
    / PLANT_VBUS as u32) as u16;
const PLANT_FRIC_FC_COUNTS: u16 =
    (PLANT_COULOMB_CPS * PLANT_VBUS as u32 / (PLANT_DUTY_GAIN << PLANT_R_SHIFT)) as u16;
const PLANT_FRIC_FV_Q016: u16 = (((PLANT_VBUS as u32) << 16)
    / (PLANT_DUTY_GAIN << (PLANT_DECAY_SHIFT + PLANT_R_SHIFT))
    - (1 << (16 - PLANT_KE_SHIFT - PLANT_R_SHIFT))) as u16;

/// Crude integer plant in the identification-model shape:
/// `omega[k+1] = a*omega[k] + b*duty - coulomb`, theta integrates, pot =
/// theta >> 16 clamped 0..4095, `i = (duty*vbus>>15 - ke*omega)/R`. Units
/// loose on purpose - qualitative closed-loop behavior, not fidelity.
struct Plant {
    theta_q16: i64,
    omega_cps: i32,
    vbus: u16,
    frozen: bool,
    /// A standing torque on the shaft, c/s per FAST tick: the drive term
    /// of a held current, which Coulomb friction meets as it meets the
    /// drive.
    load_cps: i32,
    /// Static friction, c/s per FAST tick: a shaft at rest stays there
    /// while the drive and the load together stay within it.
    stick_cps: i32,
}

impl Plant {
    fn new(pos: u16) -> Self {
        Self {
            theta_q16: (pos as i64) << 16,
            omega_cps: 0,
            vbus: PLANT_VBUS,
            frozen: false,
            load_cps: 0,
            stick_cps: 0,
        }
    }

    /// One FAST tick; frozen = hard wall (theta pinned, omega 0).
    fn step(&mut self, duty: i16) -> SensorFrame {
        let push = ((duty as i32 * PLANT_DUTY_GAIN as i32) >> 15) + self.load_cps;
        if self.frozen || self.omega_cps == 0 && push.abs() <= self.stick_cps {
            self.omega_cps = 0;
        } else {
            self.omega_cps += push - (self.omega_cps >> PLANT_DECAY_SHIFT);
            let coulomb = PLANT_COULOMB_CPS as i32;
            if self.omega_cps > 0 {
                self.omega_cps = (self.omega_cps - coulomb).max(0);
            } else {
                self.omega_cps = (self.omega_cps + coulomb).min(0);
            }
            self.theta_q16 += ((self.omega_cps as i64) << 16) / 20_000;
        }
        let pos = (self.theta_q16 >> 16).clamp(0, 4095) as u16;
        let v = (duty as i32 * self.vbus as i32) >> 15;
        let i = (v - (self.omega_cps >> PLANT_KE_SHIFT)) >> PLANT_R_SHIFT;
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
            vref_raw: 1489,
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

/// The seed is the plant it drives: a held rotor's current pins R, the
/// steady current and speed at two duties pin Ke and the Coulomb and
/// viscous terms, one MEDIUM tick's rise from rest pins B. Both duties make
/// a whole c/s drive term, so the plant's truncation stays out of the
/// steady state.
#[test]
fn the_seeds_motor_model_is_the_plants() {
    let sh = Shared::new();
    seed(&sh);
    let m = sh.table.with(|t| t.calib.motor);
    let r = m.r_q12 as f64 / 4096.0;
    let ke = m.ke_vpc_q as f64 / 4096.0;
    let recip = m.recip_ke_q as f64 / 1024.0;
    let fric =
        |omega: i32| m.fric_fc_counts as f64 + m.fric_fv_q016 as f64 * omega as f64 / 65536.0;
    let current = |f: SensorFrame| f.current as i32 - BIAS as i32;
    for duty in [4096i16, 8192] {
        let v = ((duty as i32 * PLANT_VBUS as i32) >> 15) as f64;
        let mut p = held_plant();
        let held = current(p.step(duty));
        assert!(
            (held as f64 - v / r).abs() <= 1.0,
            "duty {duty}: held {held} counts, the model {:.1}",
            v / r
        );

        let mut p = Plant::new(0);
        let mut i = 0;
        for _ in 0..20_000 {
            i = current(p.step(duty));
        }
        let model = fric(p.omega_cps);
        assert!(
            (i as f64 - model).abs() <= 1.0,
            "duty {duty}: steady {i} counts at {} c/s, the model holds {model:.1}",
            p.omega_cps
        );
        let e = v - r * i as f64;
        assert!(
            (ke * p.omega_cps as f64 - e).abs() <= 1.0,
            "duty {duty}: {e} vcounts of back-EMF at {} c/s",
            p.omega_cps
        );
        assert!(
            (recip * e - p.omega_cps as f64).abs() <= recip,
            "duty {duty}: {e} vcounts of back-EMF read {:.0} c/s at {} c/s",
            recip * e,
            p.omega_cps
        );

        let mut p = Plant::new(0);
        let mut model = 0.0;
        for _ in 0..DECIM_MED {
            let omega = p.omega_cps;
            let i = current(p.step(duty));
            model += m.b_i_q313 as f64 / 8192.0 * (i as f64 - fric(omega)) / DECIM_MED as f64;
        }
        let rise = p.omega_cps as f64;
        assert!(
            (model - rise).abs() <= rise / 16.0,
            "duty {duty}: one MEDIUM tick from rest rose {rise} c/s, the model {model:.1}"
        );
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
        // a gentle tau_d loop gain (l3 * B)
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

/// A standing load of 3 Coulomb levels against the counts: the park
/// drops the shaft.
const PLANT_LOAD_CPS: i32 = -3 * PLANT_COULOMB_CPS as i32;

/// The plant at rest at 2000 under `load_cps` and a goal on it, torque on,
/// for 6 s. Torque on at rest parks at once, and a load past the friction
/// level drops the shaft out of the zone before the loop drives it back:
/// the return is slow, the observer's speed reading on a still shaft
/// whose current ramps.
fn loaded_hold(sh: &Shared, load_cps: i32) -> (Kernel<FakeIo>, Plant) {
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant {
        load_cps,
        ..Plant::new(2000)
    };
    run_plant(&mut k, sh, &mut plant, 120_000);
    (k, plant)
}

/// The plant of [`loaded_hold`] once its shaft has static friction past the
/// load's own and a goal where it stuck, 0.5 s on: the press fold stays out
/// of the way (the gravity arm's hold with `i_cont` at the cap).
fn stuck_loaded_hold(sh: &Shared) -> (Kernel<FakeIo>, Plant) {
    let (mut k, mut plant) = loaded_hold(sh, PLANT_LOAD_CPS);
    plant.stick_cps = 6 * PLANT_COULOMB_CPS as i32;
    write_config(sh, |t| {
        t.config.limits.i_cont_counts = t.config.limits.current_limit_counts;
    });
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = plant.pos());
    run_plant(&mut k, sh, &mut plant, 10_000);
    (k, plant)
}

/// Under a load past the friction level the hold never parks: the still
/// shaft rests in the band on the current its velocity integral held,
/// frozen from the first held period to the last, instead of dropping out
/// of the zone at each park.
#[test]
fn a_loaded_hold_keeps_its_current_inside_the_band() {
    let sh = Shared::new();
    seed(&sh);
    let (mut k, mut plant) = stuck_loaded_hold(&sh);
    let db = sh
        .table
        .with(|t| t.config.loop_position.pos_deadband_counts) as i32;
    // the load's 3 levels less or plus the friction's 1 hold the shaft,
    // and the loop's ripple rides on them
    let fc = PLANT_FRIC_FC_COUNTS as i32;
    let goal = sh.table.with(|t| t.control.lifecycle.goal_position);
    let integ = k.medium.vel.integ_ccq16();
    for n in 0..20_000 {
        let f = plant.step(k.fast.duty_q15);
        k.on_scan(f, &sh);
        assert!(k.medium.life == phases::Life::Held, "not held at tick {n}");
        assert!(
            !matches!(last_cmd(&k), MotorCmd::Brake),
            "braked at tick {n}"
        );
        assert_eq!(k.medium.vel.integ_ccq16(), integ, "tick {n}: integrated");
        assert!(
            (plant.pos() - goal).abs() <= db,
            "out of the band at tick {n}: {}",
            plant.pos()
        );
        assert!(
            (2 * fc..=5 * fc).contains(&k.medium.i_ref_cc),
            "tick {n}: i_ref {} is not the load's",
            k.medium.i_ref_cc
        );
        assert!(
            !phases::loops::position::unloaded(k.medium.fusion.tau_d_q16(), fc as u16),
            "tick {n}: the load reads {} counts",
            k.medium.fusion.tau_d_counts()
        );
    }
    assert_eq!(k.faults.mask(), 0);
}

/// A still shaft under a load, at a steady error inside the band, keeps the
/// current it settled on. Integrating that error winds the current up until
/// the shaft breaks loose of its static friction and jumps past the goal: a
/// stick-slip.
#[test]
fn a_stuck_loaded_hold_does_not_wind_its_current_up() {
    let sh = Shared::new();
    seed(&sh);
    let (mut k, mut plant) = stuck_loaded_hold(&sh);
    // half the band off the shaft
    let db = sh
        .table
        .with(|t| t.config.loop_position.pos_deadband_counts) as i32;
    let goal = plant.pos() + if plant.pos() > 2000 { -db / 2 } else { db / 2 };
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = goal);
    run_plant(&mut k, &sh, &mut plant, 10_000);
    let pos = plant.pos();
    assert!((pos - goal).abs() <= db, "precondition: still at {pos}");
    // the mean current over the first and over each later 0.25 s, for 3 s
    const WINDOW: i32 = 5_000;
    let mut first = None;
    for w in 0..12 {
        let mut sum = 0;
        for n in 0..WINDOW {
            let f = plant.step(k.fast.duty_q15);
            k.on_scan(f, &sh);
            assert_eq!(plant.pos(), pos, "broke loose at tick {}", w * WINDOW + n);
            sum += k.medium.i_ref_cc;
        }
        let mean = sum / WINDOW;
        let at = *first.get_or_insert(mean);
        assert!(
            (mean - at).abs() <= 2,
            "the current wound from {at} to {mean} by {} ms, the error {}",
            (w + 1) * WINDOW / 20,
            goal - pos
        );
    }
    assert_eq!(k.faults.mask(), 0);
}

/// A held shaft pushed past the band stays held inside the zone; pushed out
/// of the zone it drives again on the integral it held: no reset on the way
/// out, and it integrates on.
#[test]
fn a_held_shaft_out_of_the_zone_integrates_again() {
    let sh = Shared::new();
    seed(&sh);
    let (mut k, mut plant) = stuck_loaded_hold(&sh);
    assert!(k.medium.life == phases::Life::Held, "precondition: held");
    let held = k.medium.vel.integ_ccq16();
    let db = sh
        .table
        .with(|t| t.config.loop_position.pos_deadband_counts) as i32;
    // past the band on the load's side, inside the zone: still held
    let goal = sh.table.with(|t| t.control.lifecycle.goal_position);
    plant.theta_q16 = ((goal - db - db / 2) as i64) << 16;
    for n in 0..2_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        assert!(k.medium.life == phases::Life::Held, "released at tick {n}");
        assert_eq!(k.medium.vel.integ_ccq16(), held, "tick {n}: integrated");
    }
    // past the zone
    plant.theta_q16 = ((goal - 2 * db - db / 2) as i64) << 16;
    // the observer's estimate follows within 50 ms
    for _ in 0..1_000 {
        if k.medium.life == phases::Life::Run {
            break;
        }
        run_plant(&mut k, &sh, &mut plant, 1);
    }
    assert!(k.medium.life == phases::Life::Run, "still held");
    for _ in 0..DECIM_MED {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.phase == phase::VELOCITY + 1 {
            break;
        }
    }
    let first = k.medium.vel.integ_ccq16();
    assert!(
        first > held && first - held < 1 << 16,
        "the first step out charged {held} to {first}"
    );
    run_plant(&mut k, &sh, &mut plant, 2_000);
    assert!(
        k.medium.vel.integ_ccq16() > first,
        "integrated {first} to {}",
        k.medium.vel.integ_ccq16()
    );
    assert_eq!(k.faults.mask(), 0);
}

/// A held shaft past the band, settled inside the zone, whose load falls
/// inside the friction level (the level raised past the held current)
/// parks where it is: the zone is a free shaft's park region once settled.
#[test]
fn a_held_shaft_that_loses_its_load_inside_the_zone_parks_there() {
    let sh = Shared::new();
    seed(&sh);
    let (mut k, mut plant) = stuck_loaded_hold(&sh);
    let db = sh
        .table
        .with(|t| t.config.loop_position.pos_deadband_counts) as i32;
    let goal = sh.table.with(|t| t.control.lifecycle.goal_position);
    let pos = goal - db - db / 2;
    plant.theta_q16 = (pos as i64) << 16;
    run_plant(&mut k, &sh, &mut plant, 2_000);
    assert!(k.medium.life == phases::Life::Held, "precondition: held");
    assert_eq!(
        k.medium.near_ticks,
        phases::loops::position::HOLD_SETTLE_TICKS
    );
    write_config(&sh, |t| t.calib.motor.fric_fc_counts = 1000);
    run_plant(&mut k, &sh, &mut plant, 2 * DECIM_MED as u32);
    for n in 0..20_000 {
        let f = plant.step(k.fast.duty_q15);
        k.on_scan(f, &sh);
        assert!(
            k.medium.life == phases::Life::Parked,
            "not parked at tick {n}"
        );
        assert_eq!(plant.pos(), pos, "moved at tick {n}");
    }
    assert_eq!(k.faults.mask(), 0);
}

/// A held shaft inside the band whose load falls inside the friction level
/// (the level raised past the held current) before the dwell has passed
/// parks at once, where it is: the band is a free shaft's park region from
/// the first period.
#[test]
fn a_held_shaft_that_loses_its_load_inside_the_band_parks_at_once() {
    let sh = Shared::new();
    seed(&sh);
    let (mut k, mut plant) = stuck_loaded_hold(&sh);
    let db = sh
        .table
        .with(|t| t.config.loop_position.pos_deadband_counts) as i32;
    let pos = plant.pos();
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = pos + db / 2);
    // the new goal's profile restarts the dwell
    for _ in 0..1_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.medium.near_ticks == 0 {
            break;
        }
    }
    assert_eq!(k.medium.near_ticks, 0, "precondition: a profile ran");
    for _ in 0..1_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.medium.life == phases::Life::Held {
            break;
        }
    }
    assert!(k.medium.life == phases::Life::Held, "precondition: held");
    assert!(k.medium.near_ticks < phases::loops::position::HOLD_SETTLE_TICKS / 2);
    assert_eq!(plant.pos(), pos, "precondition: still");
    write_config(&sh, |t| t.calib.motor.fric_fc_counts = 1000);
    run_plant(&mut k, &sh, &mut plant, 2 * DECIM_MED as u32);
    assert!(k.medium.life == phases::Life::Parked, "not parked");
    assert!(k.medium.near_ticks < phases::loops::position::HOLD_SETTLE_TICKS);
    for n in 0..20_000 {
        let f = plant.step(k.fast.duty_q15);
        k.on_scan(f, &sh);
        assert!(k.medium.life == phases::Life::Parked, "woke at tick {n}");
        assert_eq!(plant.pos(), pos, "moved at tick {n}");
    }
    assert_eq!(k.faults.mask(), 0);
}

/// A load inside the friction level is a free shaft's: torque on at rest
/// parks at once and friction holds the shaft there.
#[test]
fn a_load_inside_the_friction_level_parks() {
    let sh = Shared::new();
    seed(&sh);
    let (mut k, mut plant) = loaded_hold(&sh, -(PLANT_COULOMB_CPS as i32) / 2);
    for n in 0..20_000 {
        let f = plant.step(k.fast.duty_q15);
        k.on_scan(f, &sh);
        assert!(k.medium.life == phases::Life::Parked, "woke at tick {n}");
    }
    assert_eq!(plant.pos(), 2000);
    assert_eq!(k.faults.mask(), 0);
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
    let mut moving = false;
    for _ in 0..20_000u32 {
        // the write takes the drive kind the last tick's TRAJECTORY left
        let was_moving = moving;
        run_plant(&mut k, &sh, &mut plant, 1);
        moving = k.medium.traj.omega_star_q16() != 0;
        saw_move |= moving;
        if was_moving {
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

/// A still pot at `raw` under a goal of `goal`: whether the loop parks once
/// the profile has landed, judged before the settle dwell can widen the
/// gate.
fn parks(sh: &Shared, raw: u16, goal: i32) -> bool {
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = goal;
        t.config.loop_position.pos_deadband_counts = 12;
    });
    let mut k = kernel();
    let mut n = 0;
    while !(k.medium.traj.theta_star_q16() == goal << 16 && k.medium.traj.omega_star_q16() == 0) {
        k.on_scan(frame(raw, BIAS), sh);
        n += 1;
        assert!(n < 10_000, "the profile never landed");
    }
    for _ in 0..SETTLE_SCANS - SETTLE_SLACK {
        k.on_scan(frame(raw, BIAS), sh);
    }
    assert_eq!(k.faults.mask(), 0);
    k.medium.life == phases::Life::Parked
}

/// Local gain 26/16 over raw 1600..2240: raw 1924 linearizes to 2126.5,
/// where 12 raw counts are 19.5 linearized ones.
#[test]
fn hold_band_is_judged_in_raw_counts() {
    let sh = Shared::new();
    seed(&sh);
    // the pot alone places the observer, so the band's edge is judged to
    // the count: this rig's shaft never moves, and with a current gain the
    // observer sits a fraction of a count off it while the loop pushes
    // (under the window floor it is told the reference)
    sh.table.with_mut(|t| t.calib.motor.b_i_q313 = 0);
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

/// A non-trivial live table: local gain 26/16 over raw 1600..2240.
fn ramp_lut() -> [i16; pos_lut::POINTS] {
    let mut k = [0i16; pos_lut::POINTS];
    for (n, c) in k.iter_mut().enumerate().take(141).skip(100) {
        *c = 10 * (n as i16 - 100);
    }
    k
}

fn write_wide_map(sh: &Shared, map: [u8; adc::ADC_WIDE_SLOTS]) {
    sh.table.with_mut(|t| t.calib.adc.adc_wide_mid_q4 = map);
}

/// The all-zero map is the reading without one, on the identity and on a
/// live table, for every code the ADC can land.
#[test]
fn zero_wide_map_leaves_pos_q4_unchanged() {
    let sh = Shared::new();
    let k = ramp_lut();
    sh.with_pos_lut_mut(|a| *a = k);
    for raw in 0..=4095u16 {
        assert_eq!(phases::pos_q4(&sh, false, raw), pos_lut::identity_q4(raw));
        assert_eq!(
            phases::pos_q4(&sh, true, raw),
            pos_lut::interp_q4(raw, &k),
            "raw {raw}"
        );
    }
}

/// A set slot adds at exactly its stuck code and nowhere else: its bare
/// value on the identity, scaled by the interval's gain on a live table
/// (2047 sits in the ramp's 26/16 interval).
#[test]
fn wide_slot_adds_at_its_stuck_code_only() {
    let sh = Shared::new();
    let k = ramp_lut();
    sh.with_pos_lut_mut(|a| *a = k);
    let mut map = [0u8; adc::ADC_WIDE_SLOTS];
    map[3] = 88;
    map[7] = 80;
    map[15] = 255;
    write_wide_map(&sh, map);
    for raw in 0..=4095u16 {
        let add = match raw {
            1023 => 88,
            2047 => 80,
            4095 => 255,
            _ => 0,
        };
        let i = pos_lut::index(raw);
        let gain = pos_lut::band_gain_q4(k[i], k[i + 1]) as u32;
        let scaled = ((add as u32 * gain) >> pos_lut::GRID_SHIFT) as u16;
        assert_eq!(
            phases::pos_q4(&sh, false, raw),
            pos_lut::identity_q4(raw).wrapping_add(add),
            "raw {raw}"
        );
        assert_eq!(
            phases::pos_q4(&sh, true, raw),
            pos_lut::interp_q4(raw, &k).wrapping_add(scaled),
            "raw {raw}"
        );
    }
    assert_eq!(
        phases::pos_q4(&sh, true, 2047) - pos_lut::interp_q4(2047, &k),
        80 * 26 / 16
    );
}

/// A table with gain `16 + rise` (Q4) through raw 960..1120, across the
/// stuck code 1023 and the interval above it.
fn sloped_lut(rise: i16) -> [i16; pos_lut::POINTS] {
    let mut k = [0i16; pos_lut::POINTS];
    for (n, c) in k.iter_mut().enumerate().take(71).skip(60) {
        *c = rise * (n as i16 - 60);
    }
    k
}

/// The wide-code step is raw counts ahead of the table: where the code's
/// interval and the one above share a gain, on a live table it is the slot
/// times that gain, truncated, and lands within one Q4 count of the table
/// read at the corrected raw, steeper and shallower than unity alike.
#[test]
fn wide_step_scales_by_the_table_gain() {
    let sh = Shared::new();
    for (rise, gain) in [(8i16, 24u16), (-4, 12)] {
        let k = sloped_lut(rise);
        sh.with_pos_lut_mut(|a| *a = k);
        let i = pos_lut::index(1023);
        assert_eq!(pos_lut::band_gain_q4(k[i], k[i + 1]), gain);
        assert_eq!(pos_lut::band_gain_q4(k[i + 1], k[i + 2]), gain);
        for slot in [16u8, 80, 88, 255] {
            let mut map = [0u8; adc::ADC_WIDE_SLOTS];
            map[3] = slot;
            write_wide_map(&sh, map);
            let got = phases::pos_q4(&sh, true, 1023);
            let base = pos_lut::interp_q4(1023, &k);
            assert_eq!(
                got - base,
                (slot as u16 * gain) >> pos_lut::GRID_SHIFT,
                "gain {gain} slot {slot}"
            );
            // the table at raw 1023 + slot / 16, interpolated inside the
            // interval above
            let at = (1023u32 << pos_lut::GRID_SHIFT) + slot as u32;
            let r = (at >> pos_lut::GRID_SHIFT) as u16;
            let f = (at & 15) as f64 / 16.0;
            let (lo, hi) = (
                pos_lut::interp_q4(r, &k) as f64,
                pos_lut::interp_q4(r + 1, &k) as f64,
            );
            let want = lo + (hi - lo) * f;
            assert!(
                (got as f64 - want).abs() <= 1.0,
                "gain {gain} slot {slot}: {got} vs {want}"
            );
            assert_eq!(phases::pos_q4(&sh, false, 1023), (1023 << 4) + slot as u16);
        }
    }
}

/// The table as a continuous function of raw counts, Q4, the identity
/// past its last point.
fn table_at_q4(k: &[i16; pos_lut::POINTS], raw: f64) -> f64 {
    let j = ((raw / 16.0).floor() as usize).min(pos_lut::INTERVALS);
    let c0 = k[j] as f64;
    let c1 = k.get(j + 1).map_or(0.0, |&c| c as f64);
    16.0 * (raw + c0 + (c1 - c0) * (raw - 16.0 * j as f64) / 16.0)
}

/// A stuck code is the last of its interval, so a slot of a count or more
/// puts the band's middle in the interval above: the reading is the table
/// at raw + slot / 16, floored to a Q4 count, whatever the two intervals'
/// gains. The cases: a steeper interval above (2047, gains 20 and 23),
/// a shallower one (3071, 24 and 12), an own gain under the hold band's
/// clamp (1023, gain 4 below 28) and the top code, past the last point.
#[test]
fn wide_step_reads_the_table_where_the_band_lies() {
    let sh = Shared::new();
    let mut k = [0i16; pos_lut::POINTS];
    for (n, c) in [
        (128, 4),
        (129, 11),
        (192, 8),
        (193, 4),
        (64, -12),
        (255, -3),
    ] {
        k[n] = c;
    }
    sh.with_pos_lut_mut(|a| *a = k);
    for raw in [2047u16, 3071, 1023, 4095] {
        for slot in [88u8, 1, 8, 15, 16, 17, 40, 99, 160, 255] {
            let mut map = [0u8; adc::ADC_WIDE_SLOTS];
            map[adc::wide_slot(raw)] = slot;
            write_wide_map(&sh, map);
            let want = table_at_q4(&k, raw as f64 + slot as f64 / 16.0).floor() as u32 as u16;
            assert_eq!(
                phases::pos_q4(&sh, true, raw),
                want,
                "raw {raw} slot {slot}"
            );
            assert_eq!(
                phases::pos_q4(&sh, false, raw),
                pos_lut::identity_q4(raw).wrapping_add(slot as u16),
                "raw {raw} slot {slot}"
            );
        }
    }
}

/// A COMMIT of all-zero points over a LIVE table puts the kernel on the
/// identity: the flag, every code's reading and the hold band's gain are
/// what a boot on the stored points gives, under a full wide map.
#[test]
fn zero_table_commit_reads_the_pot_as_a_boot_does() {
    use crate::regions::control::addr::pos_lut::{POS_LUT_CMD, POS_LUT_PAGE};
    let sh = Shared::new();
    sh.with_pos_lut_mut(|a| *a = ramp_lut());
    let mut map = [0u8; adc::ADC_WIDE_SLOTS];
    for (s, m) in map.iter_mut().enumerate() {
        *m = 1 + 16 * s as u8;
    }
    write_wide_map(&sh, map);
    sh.table
        .with_mut(|t| t.control.pos_lut.pos_lut_state = pos_lut::state::LIVE);
    assert!(phases::Control::read(&sh).lut_live);

    for page in 0..pos_lut::PAGES as u8 {
        sh.table.with_mut(|t| {
            t.control.pos_lut.pos_lut_page = page;
            t.control.pos_lut.pos_lut_cmd = pos_lut::cmd::STORE;
            t.control.pos_lut.pos_lut_points = [0; pos_lut::PAGE_POINTS];
        });
        sh.pos_lut_after_commit(POS_LUT_PAGE, 2 + 2 * pos_lut::PAGE_POINTS as u16);
    }
    sh.table
        .with_mut(|t| t.control.pos_lut.pos_lut_cmd = pos_lut::cmd::COMMIT);
    sh.pos_lut_after_commit(POS_LUT_CMD, 1);
    assert!(sh.data_job_service());

    let booted = crate::persist::rebooted(&sh);
    let (ctl, boot) = (phases::Control::read(&sh), phases::Control::read(&booted));
    assert!(!boot.lut_live);
    assert_eq!(ctl.lut_live, boot.lut_live);
    for raw in 0..=4095u16 {
        assert_eq!(
            phases::pos_q4_band_gain(&sh, ctl.lut_live, raw),
            phases::pos_q4_band_gain(&booted, boot.lut_live, raw),
            "raw {raw}"
        );
    }
}

/// The observer innovates on the same measurement `pos_q4` gives: seeded
/// at `pos_q4(raw)` with no model, one observer step on `raw` leaves the
/// estimate where it was, for every code, the table live and a full map.
#[test]
fn observer_measures_what_pos_q4_reads() {
    let sh = Shared::new();
    let lut = ramp_lut();
    sh.with_pos_lut_mut(|a| *a = lut);
    let mut map = [0u8; adc::ADC_WIDE_SLOTS];
    for (s, m) in map.iter_mut().enumerate() {
        *m = 1 + 16 * s as u8;
    }
    write_wide_map(&sh, map);
    sh.table.with_mut(|t| {
        t.config.fusion.l1_q016 = 1 << 14;
        t.config.fusion.l2_q88 = 1 << 8;
        t.control.pos_lut.pos_lut_state = pos_lut::state::LIVE;
    });
    let mut k = kernel();
    for _ in 0..4 * DECIM_MED as u32 {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    for live in [true, false] {
        if !live {
            sh.table
                .with_mut(|t| t.control.pos_lut.pos_lut_state = pos_lut::state::IDENTITY);
            for _ in 0..4 * DECIM_MED as u32 {
                k.on_scan(frame(2000, BIAS), &sh);
            }
        }
        for raw in 0..=4095u16 {
            let pos = phases::pos_q4(&sh, live, raw);
            k.medium.fusion.seed(pos);
            k.medium.observe(raw, &k.cfg, &sh, &k.fast);
            assert_eq!(
                k.medium.fusion.theta_q16(),
                FusionObs::seed_theta_q16(pos),
                "live {live} raw {raw}"
            );
            assert_eq!(k.medium.fusion.omega_q16(), 0, "live {live} raw {raw}");
        }
    }
}

/// What a held OpenLoop drive publishes from `f` under `map`: the held
/// current, the differential, the rail and the NTC temperature. The raw
/// fields of the tick block are `f`'s own codes.
fn wide_drive_reads(map: [u8; adc::ADC_WIDE_SLOTS], f: SensorFrame) -> (i16, i16, u16, i16) {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    write_config(&sh, |t| t.calib.thermal.th_rise_q016 = 0);
    write_wide_map(&sh, map);
    let mut k = kernel();
    settle(&mut k, &sh, f);
    ticks(&mut k, &sh, f, BASE_FAST_TICKS);
    assert_eq!(k.faults.mask(), 0);
    let t = tick_block(&sh);
    let both = tick_flag::I_VALID | tick_flag::V_VALID;
    assert_eq!(t.tick_flags & both, both);
    assert_eq!(
        (t.current_raw, t.vmotor_a, t.vmotor_b, t.vbus_raw, t.ntc_raw),
        (f.current, f.vmotor_a, f.vmotor_b, f.vbus_raw, f.ntc_raw)
    );
    (
        t.i_tick_counts,
        t.vdiff_counts,
        sh.table.with(|t| t.telemetry.estimates.vbus_counts),
        therm(&sh).3,
    )
}

/// Every channel reads a wide code as the middle of its band: the crest
/// shunt, each tap, the rail and the NTC, each stuck on a wide code under
/// its slot, publish what the code that many counts up publishes without
/// a map. The raw fields stay raw.
#[test]
fn a_wide_code_reads_mid_band_on_every_channel() {
    const MID: u16 = 10;
    const MID_LOW: u16 = 6;
    let stuck = SensorFrame {
        current: 0x8ff,
        vmotor_a: 0xbff,
        vmotor_b: 0x0ff,
        vbus_raw: 0xaff,
        ntc_raw: 0x7ff,
        ..frame(2000, BIAS)
    };
    let mid = SensorFrame {
        current: stuck.current + MID,
        vmotor_a: stuck.vmotor_a + MID,
        vmotor_b: stuck.vmotor_b + MID_LOW,
        vbus_raw: stuck.vbus_raw + MID,
        ntc_raw: stuck.ntc_raw + MID,
        ..stuck
    };
    let none = [0u8; adc::ADC_WIDE_SLOTS];
    let want = wide_drive_reads(none, mid);
    // the bare stuck codes read apart on every channel: the pin can fail
    let bare = wide_drive_reads(none, stuck);
    assert!(bare.0 != want.0 && bare.1 != want.1 && bare.2 != want.2 && bare.3 != want.3);
    let mut map = [(MID << 4) as u8; adc::ADC_WIDE_SLOTS];
    map[adc::wide_slot(stuck.vmotor_b)] = (MID_LOW << 4) as u8;
    assert_eq!(wide_drive_reads(map, stuck), want);
}

/// The shunt zero stuck on a wide code is tracked at the middle of its
/// band.
#[test]
fn a_wide_zero_is_tracked_mid_band() {
    let sh = Shared::new();
    seed(&sh);
    let mut map = [0u8; adc::ADC_WIDE_SLOTS];
    map[7] = 160;
    write_wide_map(&sh, map);
    let mut k = kernel();
    // 12 time constants at either feed rate, a tick or a period
    ticks(&mut k, &sh, rest(0x7ff), 200_000);
    assert_eq!(published_bias(&sh), 0x7ff + 10);
}

/// What a torque-off servo publishes with the trough shunt, the rail and
/// the NTC all on `code` under `map`, after `n` ticks: the zero in use,
/// the rail and the NTC temperature. The zero boots at `code`, so only a
/// slot moves it. The raw fields of the tick block are the codes.
fn rest_reads(map: [u8; adc::ADC_WIDE_SLOTS], code: u16, n: u32) -> (u16, u16, i16) {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = false;
        t.telemetry.refs.current_bias_counts = code;
    });
    write_wide_map(&sh, map);
    let mut k = kernel();
    let f = SensorFrame {
        vbus_raw: code,
        ntc_raw: code,
        ..rest(code)
    };
    ticks(&mut k, &sh, f, n);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    let t = tick_block(&sh);
    assert_eq!(
        (t.current_trough, t.vbus_raw, t.ntc_raw),
        (code, code, code)
    );
    (
        published_bias(&sh),
        sh.table.with(|t| t.telemetry.estimates.vbus_counts),
        therm(&sh).3,
    )
}

/// The slower consumers on every wide code, torque off: the shunt zero,
/// the rail and the NTC. Under an all-zero map each reads its code raw,
/// the wide codes and their neighbors alike. Under a map each reads a
/// wide code as the code its slot's whole counts up reads without one,
/// and the neighbors of a wide code as before: the slot adds at its stuck
/// code and nowhere else.
#[test]
fn the_zero_the_rail_and_the_ntc_read_every_wide_code_mid_band() {
    const NONE: [u8; adc::ADC_WIDE_SLOTS] = [0; adc::ADC_WIDE_SLOTS];
    // a boot, a SLOW due and its finish: nothing left to move without a
    // slot
    const SHORT: u32 = 2 * SLOW;
    // 8 time constants of the zero carry it a slot of 15 counts to the
    // count
    const LONG: u32 = (8 << bias::ALPHA_SHIFT) * DECIM_MED as u32;
    // a slot of its own on every wide code, whole counts
    let counts = |k: usize| (k % 15 + 1) as u16;
    let map: [u8; adc::ADC_WIDE_SLOTS] = core::array::from_fn(|k| (counts(k) << 4) as u8);
    for k in 0..adc::ADC_WIDE_SLOTS {
        let code = ((k as u16) << adc::ADC_WIDE_SHIFT) | adc::ADC_WIDE_MASK;
        let up = code + counts(k);
        for near in [code - 1, code, code + 1, up] {
            let raw = rest_reads(NONE, near, SHORT);
            let rail = q_mul(near as i32, VBUS_SCALE_Q15 as i32, 15) as u16;
            assert_eq!((raw.0, raw.1), (near, rail), "no map, code {near}");
            if near != code {
                assert_eq!(rest_reads(map, near, SHORT), raw, "map, code {near}");
            }
        }
        let mid = rest_reads(map, code, LONG);
        assert_eq!(mid, rest_reads(NONE, up, SHORT), "map, wide code {code}");
        assert_eq!(mid.0, up, "map, wide code {code}");
        // the code alone reads apart on the zero and the rail: the pin can
        // fail
        let bare = rest_reads(NONE, code, SHORT);
        assert!(bare.0 != mid.0 && bare.1 != mid.1, "wide code {code}");
    }
}

/// The ten raw fields of a tick block, as the frame carries them.
fn raw_fields(t: &TelemetryTick) -> [u16; 10] {
    [
        t.pos,
        t.vref_raw,
        t.vbus_raw,
        t.ntc_raw,
        t.current_raw,
        t.vmotor_a,
        t.vmotor_b,
        t.current_trough,
        t.vmotor_a_trough,
        t.vmotor_b_trough,
    ]
}

/// Raw stays raw: with every channel parked on a wide code under a full
/// map, the raw fields of the tick block and of the row the TEL stream
/// takes are the converter's codes on every tick, as they are under no
/// map, through the fast ticks, every medium phase and both SLOW stages.
/// The derived fields beside them move with the map.
#[test]
fn a_written_map_leaves_every_raw_field_alone() {
    let f = SensorFrame {
        pos: 0x7ff,
        current_trough: 0x7ff,
        current: 0x8ff,
        vmotor_a: 0xbff,
        vmotor_b: 0x0ff,
        vbus_raw: 0xaff,
        ntc_raw: 0x7ff,
        vref_raw: 0x5ff,
        vmotor_a_trough: 0xbff,
        vmotor_b_trough: 0x0ff,
    };
    let codes = [
        f.pos,
        f.vref_raw,
        f.vbus_raw,
        f.ntc_raw,
        f.current,
        f.vmotor_a,
        f.vmotor_b,
        f.current_trough,
        f.vmotor_a_trough,
        f.vmotor_b_trough,
    ];
    assert!(
        codes
            .iter()
            .all(|c| c & adc::ADC_WIDE_MASK == adc::ADC_WIDE_MASK)
    );
    // ten counts on every wide code, six on the low tap's
    let mut full = [0xa0u8; adc::ADC_WIDE_SLOTS];
    full[adc::wide_slot(f.vmotor_b)] = 0x60;
    let derived = [[0u8; adc::ADC_WIDE_SLOTS], full].map(|map| {
        let sh = Shared::new();
        seed(&sh);
        seed_estimate(&sh);
        sh.table
            .with_mut(|t| t.telemetry.refs.current_bias_counts = f.current_trough);
        write_wide_map(&sh, map);
        let mut k = Kernel::with_tel(
            FakeIo {
                sensors: FakeSensors,
                motor: FakeMotor { last: None },
            },
            RecTel::new(&sh),
            TIMING,
        );
        settle(&mut k, &sh, f);
        for tick in 0..2 * SLOW {
            k.tel.rows.clear();
            k.on_scan(f, &sh);
            assert_eq!(raw_fields(&k.tel.rows[0]), codes, "row, tick {tick}");
            assert_eq!(raw_fields(&tick_block(&sh)), codes, "block, tick {tick}");
        }
        assert_eq!(k.faults.mask(), 0);
        let t = tick_block(&sh);
        (
            t.i_tick_counts,
            t.vdiff_counts,
            sh.table.with(|t| t.telemetry.estimates.vbus_counts),
            therm(&sh).3,
        )
    });
    let (none, full) = (derived[0], derived[1]);
    assert!(
        none.0 != full.0 && none.1 != full.1 && none.2 != full.2 && none.3 != full.3,
        "{none:?} {full:?}"
    );
}

/// The overcurrent trip compares the corrected current, either way. A
/// crest sample on a wide code under the trip without its slot and over
/// it with: no trip without the map, a trip with. A zero on a wide code
/// that reads the current over the trip without its slot and under it
/// with: a trip without the map, none with.
#[test]
fn the_trip_compares_the_corrected_current() {
    const TRIP: u16 = 1030;
    const SLOT: u16 = 10;
    // a held OpenLoop drive on a calm current until the zero has settled
    // on `zero` read under `map`, then `hot` on the crest; the fault mask
    // and the held current
    let run = |map: [u8; adc::ADC_WIDE_SLOTS], zero: u16, hot: u16| {
        let sh = Shared::new();
        seed(&sh);
        seed_estimate(&sh);
        write_config(&sh, |t| t.config.limits.oc_trip_counts = TRIP);
        sh.table
            .with_mut(|t| t.telemetry.refs.current_bias_counts = zero);
        write_wide_map(&sh, map);
        let mut k = kernel();
        let mut f = frame(2000, zero + 40);
        f.current_trough = zero;
        settle(&mut k, &sh, f);
        periods(&mut k, &sh, f, BIAS_SETTLE_PERIODS);
        assert_eq!(k.faults.mask(), 0);
        f.current = hot;
        ticks(&mut k, &sh, f, 50);
        (k.faults.mask(), tick_block(&sh).i_tick_counts)
    };
    let none = [0u8; adc::ADC_WIDE_SLOTS];
    let slot_at = |code: u16| {
        let mut map = none;
        map[adc::wide_slot(code)] = (SLOT << 4) as u8;
        map
    };
    // the crest on 0xbff over a zero of 2048: 1023 counts, 1033 with the
    // slot
    assert_eq!(run(none, BIAS, 0xbff), (0, TRIP as i16 - 7));
    let (mask, _) = run(slot_at(0xbff), BIAS, 0xbff);
    assert_eq!(mask, faults::BIT_OVER_CURRENT);
    // the zero on 0x7ff, the crest 1033 counts over it: 1023 with the
    // zero's slot
    let hot = 0x7ff + TRIP + 3;
    assert!(hot & adc::ADC_WIDE_MASK != adc::ADC_WIDE_MASK);
    let (mask, _) = run(none, 0x7ff, hot);
    assert_eq!(mask, faults::BIT_OVER_CURRENT);
    assert_eq!(run(slot_at(0x7ff), 0x7ff, hot), (0, TRIP as i16 - 7));
}

/// The settle dwell in fast ticks.
const SETTLE_SCANS: u32 = phases::loops::position::HOLD_SETTLE_TICKS as u32 * DECIM_MED as u32;

/// A still pot at 1924 under a goal of 1924 and a 12-count band, scanned
/// until the loop parks and then `after` more ticks.
fn parked_for(sh: &Shared, after: u32) -> Kernel<FakeIo> {
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 1924;
        t.config.loop_position.pos_deadband_counts = 12;
    });
    let mut k = kernel();
    let mut n = 0;
    while k.medium.life != phases::Life::Parked {
        k.on_scan(frame(1924, BIAS), sh);
        n += 1;
        assert!(n < 10_000, "never parked at zero error");
    }
    for _ in 0..after {
        k.on_scan(frame(1924, BIAS), sh);
        assert!(k.medium.life == phases::Life::Parked, "woke at zero error");
    }
    k
}

/// Whether the pot still at `raw` wakes the park within 2000 ticks.
fn wakes(k: &mut Kernel<FakeIo>, sh: &Shared, raw: u16) -> bool {
    for _ in 0..2_000 {
        k.on_scan(frame(raw, BIAS), sh);
        if k.medium.life != phases::Life::Parked {
            return true;
        }
    }
    false
}

#[test]
fn a_settled_hold_stays_parked_under_noise_past_the_band() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = parked_for(&sh, 2 * SETTLE_SCANS);
    // the pot then sits 1.5x the band off, +-1 count of noise
    let mut rng: u32 = 0xdead_beef;
    for n in 0..20_000 {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let raw = (1942 + ((rng >> 24) as i32 % 3) - 1) as u16;
        k.on_scan(frame(raw, BIAS), &sh);
        assert!(
            k.medium.life == phases::Life::Parked,
            "woke at tick {n}, pot {raw}"
        );
    }
    assert_eq!(k.faults.mask(), 0);
}

/// Torque on under a goal of 1924 and a 12-count band, the pot read by
/// `pot` on every scan, until the profile has landed.
fn landed(sh: &Shared, pot: &mut impl FnMut() -> u16) -> Kernel<FakeIo> {
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 1924;
        t.config.loop_position.pos_deadband_counts = 12;
    });
    let mut k = kernel();
    let mut n = 0;
    while !(k.medium.traj.theta_star_q16() == 1924 << 16 && k.medium.traj.omega_star_q16() == 0) {
        k.on_scan(frame(pot(), BIAS), sh);
        n += 1;
        assert!(n < 10_000, "the profile never landed");
    }
    k
}

/// Medium ticks of slack either side of the dwell for the phase the
/// landing falls on.
const SETTLE_SLACK: u32 = 2 * DECIM_MED as u32;

#[test]
fn a_still_pot_past_the_band_parks_once_the_dwell_has_passed() {
    let sh = Shared::new();
    seed(&sh);
    // 1.5x the band off the goal
    let mut k = landed(&sh, &mut || 1942);
    for n in 0..SETTLE_SCANS - SETTLE_SLACK {
        k.on_scan(frame(1942, BIAS), &sh);
        assert!(
            k.medium.life != phases::Life::Parked,
            "parked before the dwell, tick {n}"
        );
    }
    for _ in 0..2 * SETTLE_SLACK {
        k.on_scan(frame(1942, BIAS), &sh);
    }
    for n in 0..20_000 {
        k.on_scan(frame(1942, BIAS), &sh);
        assert!(
            k.medium.life == phases::Life::Parked,
            "woke after the dwell, tick {n}"
        );
    }
    assert_eq!(k.faults.mask(), 0);
}

#[test]
fn a_hold_on_the_band_edge_settles_through_its_wakes() {
    let sh = Shared::new();
    seed(&sh);
    // one count past the band, +-1 count of noise: it parks only on dips
    let mut rng: u32 = 0xdead_beef;
    let mut pot = || {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (1937 + ((rng >> 24) as i32 % 3) - 1) as u16
    };
    let mut k = landed(&sh, &mut pot);
    for _ in 0..SETTLE_SCANS + SETTLE_SLACK {
        k.on_scan(frame(pot(), BIAS), &sh);
    }
    for n in 0..20_000 {
        let raw = pot();
        k.on_scan(frame(raw, BIAS), &sh);
        assert!(
            k.medium.life == phases::Life::Parked,
            "woke after the dwell, tick {n}, pot {raw}"
        );
    }
    assert_eq!(k.faults.mask(), 0);
}

#[test]
fn a_settled_hold_wakes_past_two_bands() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = parked_for(&sh, 2 * SETTLE_SCANS);
    assert!(wakes(&mut k, &sh, 1954), "parked at 2.5x the band");
    assert_eq!(k.faults.mask(), 0);
}

#[test]
fn a_goal_write_unsettles_the_hold() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = parked_for(&sh, 2 * SETTLE_SCANS);
    // a 6-count move lands with the still pot inside the band: a new park
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = 1930);
    let mut n = 0;
    while !(k.medium.life == phases::Life::Parked && k.medium.traj.theta_star_q16() == 1930 << 16) {
        k.on_scan(frame(1924, BIAS), &sh);
        n += 1;
        assert!(n < 10_000, "the move never landed in a park");
    }
    // right after the landing, 1.5x the band off the new goal
    assert!(wakes(&mut k, &sh, 1912), "the new park started settled");
    assert_eq!(k.faults.mask(), 0);
}

/// A servo at rest moves only when asked: torque on with the goal on the
/// present position parks at once. The host writes the goal as it reads
/// the position, the published estimate rounded to the count, so the goal
/// sits inside the band of the shaft whatever one pot sample says.
#[test]
fn torque_on_at_rest_on_the_goal_drives_nothing() {
    let sh = Shared::new();
    seed(&sh);
    sh.table
        .with_mut(|t| t.config.loop_position.pos_deadband_counts = 4);
    let mut k = kernel();
    // a still shaft under +-2 counts of pot noise, half the band
    let mut rng: u32 = 0xdead_beef;
    let mut pot = move || {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (2000 + ((rng >> 24) as i32 % 5) - 2) as u16
    };
    for enable in 0..32 {
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        for _ in 0..2_000 {
            k.on_scan(frame(pot(), BIAS), &sh);
        }
        let goal = sh
            .table
            .with(|t| (t.telemetry.estimates.theta_hat_q16 + (1 << 15)) >> 16);
        sh.table.with_mut(|t| {
            t.control.lifecycle.goal_position = goal;
            t.control.lifecycle.torque_enable = true;
        });
        for n in 0..8_000u32 {
            k.on_scan(frame(pot(), BIAS), &sh);
            let width = match last_cmd(&k) {
                MotorCmd::Drive { ticks, .. } => ticks,
                _ => 0,
            };
            assert_eq!(width, 0, "enable {enable}, goal {goal}: drove at tick {n}");
            assert_eq!(
                k.medium.i_ref_cc, 0,
                "enable {enable}, goal {goal}: current asked at tick {n}"
            );
            // the edge lands on the next CONTROL phase, the park on that
            // period's TRAJECTORY phase
            if n >= 2 * DECIM_MED as u32 {
                assert!(
                    k.medium.life == phases::Life::Parked,
                    "enable {enable}, goal {goal}: not parked at tick {n}"
                );
            }
        }
        assert_eq!(k.faults.mask(), 0);
    }
}

/// One pot sample is no measure of a shaft at rest: the edge tick's sample
/// sits 3 counts off the still shaft, either side, and torque on with the
/// goal on the estimate still parks at once without a drive.
#[test]
fn torque_on_at_rest_ignores_a_stray_edge_sample() {
    let sh = Shared::new();
    seed(&sh);
    sh.table
        .with_mut(|t| t.config.loop_position.pos_deadband_counts = 4);
    let mut k = kernel();
    let mut rng: u32 = 0xdead_beef;
    let mut pot = move || {
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (2000 + ((rng >> 24) as i32 % 5) - 2) as u16
    };
    for enable in 0..32 {
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        for _ in 0..2_000 {
            k.on_scan(frame(pot(), BIAS), &sh);
        }
        let goal = sh
            .table
            .with(|t| (t.telemetry.estimates.theta_hat_q16 + (1 << 15)) >> 16);
        sh.table.with_mut(|t| {
            t.control.lifecycle.goal_position = goal;
            t.control.lifecycle.torque_enable = true;
        });
        while k.phase != phase::CONTROL {
            k.on_scan(frame(pot(), BIAS), &sh);
        }
        let stray = if enable % 2 == 0 { 3 } else { -3 };
        k.on_scan(frame((pot() as i32 + stray) as u16, BIAS), &sh);
        assert_eq!(k.medium.traj.theta_star_q16(), goal << 16);
        for n in 0..8_000u32 {
            k.on_scan(frame(pot(), BIAS), &sh);
            let width = match last_cmd(&k) {
                MotorCmd::Drive { ticks, .. } => ticks,
                _ => 0,
            };
            assert_eq!(width, 0, "enable {enable}, goal {goal}: drove at tick {n}");
            assert_eq!(
                k.medium.i_ref_cc, 0,
                "enable {enable}, goal {goal}: current asked at tick {n}"
            );
            // the park lands on the edge period's TRAJECTORY phase
            if n >= DECIM_MED as u32 {
                assert!(
                    k.medium.life == phases::Life::Parked,
                    "enable {enable}, goal {goal}: not parked at tick {n}"
                );
            }
        }
        assert_eq!(k.faults.mask(), 0);
    }
}

/// A shaft moved by hand under torque off leaves the estimate behind the
/// pot, and a host that reads the position then writes that stale estimate
/// as the goal. The edge sample disagrees with the estimate by more than
/// the zone: the estimate is not kept, fusion seeds at the sample, its
/// railed disturbance goes with it, and nothing latches, STALL included.
#[test]
fn torque_on_after_a_hand_move_seeds_at_the_sample() {
    let sh = Shared::new();
    seed(&sh);
    sh.table
        .with_mut(|t| t.config.limits.stall_response = StallResponse::Fault);
    let mut k = kernel();
    for i in 0..4000u32 {
        let pos = 1000 + ((i / 4) % 2000) as u16;
        k.on_scan(frame(pos, BIAS), &sh);
    }
    for _ in 0..150 {
        k.on_scan(frame(1000, BIAS), &sh);
    }
    let goal = sh
        .table
        .with(|t| (t.telemetry.estimates.theta_hat_q16 + (1 << 15)) >> 16);
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_position = goal;
        t.control.lifecycle.torque_enable = true;
    });
    run_to(&mut k, &sh, frame(1000, BIAS), phase::CONTROL);
    let band = 8 << 16;
    let theta_pre = k.medium.fusion.theta_q16();
    assert!(
        ((goal << 16) - theta_pre).abs() <= band,
        "precondition: the goal is on the estimate"
    );
    assert!(
        (theta_pre - (1000 << 16)).abs() > 2 * band,
        "precondition: the estimate lags the pot past the zone, at {}",
        theta_pre >> 16
    );
    assert!(
        k.medium.fusion.tau_d_counts().unsigned_abs() > 200,
        "precondition: hand motion railed tau_d, got {}",
        k.medium.fusion.tau_d_counts()
    );
    k.on_scan(frame(1000, BIAS), &sh);
    assert_eq!(k.medium.fusion.theta_q16(), 1000 << 16);
    assert_eq!(k.medium.fusion.tau_d_counts(), 0);
    assert_eq!(k.medium.traj.theta_star_q16(), 1000 << 16);
    for _ in 0..4000 {
        k.on_scan(frame(1000, BIAS), &sh);
        assert_eq!(k.faults.mask(), 0);
    }
}

/// A goal past the band of the estimate is a move: the profile starts at
/// the fresh estimate, seeded at the edge sample, and runs to the goal.
#[test]
fn torque_on_with_the_goal_past_the_band_moves_from_the_sample() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..2000 {
        k.on_scan(frame(2000, BIAS), &sh);
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_position = 2100;
        t.control.lifecycle.torque_enable = true;
    });
    run_to(&mut k, &sh, frame(2000, BIAS), phase::CONTROL);
    assert_eq!(k.medium.fusion.theta_q16(), 2000 << 16);
    // a sample off the estimate, inside the zone
    k.on_scan(frame(2003, BIAS), &sh);
    assert_eq!(k.medium.fusion.theta_q16(), 2003 << 16);
    assert_eq!(k.medium.traj.theta_star_q16(), 2003 << 16);
    let mut prev = 2003 << 16;
    let mut moved = false;
    for _ in 0..4000 {
        k.on_scan(frame(2000, BIAS), &sh);
        let theta_star = k.medium.traj.theta_star_q16();
        assert!(theta_star >= prev, "the profile turned back");
        moved |= k.medium.traj.omega_star_q16() > 0;
        prev = theta_star;
    }
    assert!(moved);
    assert_eq!(prev, 2100 << 16);
    assert_eq!(k.medium.traj.omega_star_q16(), 0);
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
    assert!(k.medium.life == phases::Life::Parked, "never parked");
    let mut rng: u32 = 0xdead_beef;
    for _ in 0..20_000 {
        let mut f = plant.step(k.fast.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f.pos = (f.pos as i32 + ((rng >> 24) as i32 % 7) - 3).clamp(0, 4095) as u16;
        k.on_scan(f, &sh);
        if k.medium.life == phases::Life::Parked && k.phase == phase::VELOCITY + 1 {
            assert_eq!(k.medium.i_ref_cc, 0, "parked with a current command");
        }
    }
    // pushed out of the band: the first velocity step after the park is a
    // fresh loop's on the same inputs, so no charge survived the hold
    plant.theta_q16 -= 60i64 << 16;
    for _ in 0..2000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.phase == phase::VELOCITY + 1 && k.medium.life != phases::Life::Parked {
            let (lv, motor) = sh.table.with(|t| (t.config.loop_velocity, t.calib.motor));
            let fresh = VelocityLoop::new().step(
                k.medium.omega_ref_q16,
                k.medium.omega_hat,
                k.medium.traj.alpha_star_q16(),
                k.medium.traj.omega_star_q16(),
                k.medium.i_band,
                &lv,
                &motor,
                false,
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
        unfolded(t);
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
/// or finish) and on no other: never a configuration take's tick, never a
/// plain one. The stages are the budget probe's (`Events::between` on the
/// tags).
#[test]
fn slow_stage_ticks_hold_the_tel_lane() {
    let sh = Shared::new();
    seed(&sh);
    seed_estimate(&sh);
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
    // due, finish, configuration take
    let mut seen = [0u32; 3];
    for t in 0..BASE_FAST_TICKS {
        if t == BASE_FAST_TICKS / 2 {
            write_config(&sh, |_| {});
        }
        let before = k.probe_tags();
        k.tel.held = false;
        k.on_scan(hot, &sh);
        let ev = Events::between(before, k.probe_tags());
        let kind = [ev.slow, ev.finish, ev.refresh];
        for (n, &hit) in seen.iter_mut().zip(&kind) {
            *n += hit as u32;
        }
        let want = kind[..2].iter().any(|&h| h);
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
    seed_estimate(&sh);
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
    let mut events = [0u32; 6];
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
        crate::kernel::config::belt::refuses(c.belt, Mode::Position),
        c.motor.recip_ke_q == 0,
        "the belt reads the boxcar's Ke"
    );
    [
        ol_max(k) == 5000,
        c.limits.stall_time_ticks == 400,
        c.motor.recip_ke_q == 0,
    ]
}

/// While a limit is unset the kernel's copy holds the ceiling at 0 with the
/// move speed under it, and the cap at the overcurrent trip, the guard
/// OpenLoop's limiter holds under. The take lands at SPARE_A and the belt
/// refuses every closed mode at the next CONTROL, ahead of TRAJECTORY,
/// LIMITS and VELOCITY, the phases that read either: the ticks between,
/// and the CONTROL tick's write, still drive on the band and reference
/// built before the take, and none after drives at all, so no closed-loop
/// tick runs on an unset limit.
#[test]
fn unset_limits_never_reach_a_closed_tick() {
    const _: () = assert!(
        phase::CONTROL < phase::TRAJECTORY
            && phase::TRAJECTORY < phase::LIMITS
            && phase::LIMITS < phase::VELOCITY
            && phase::VELOCITY < phase::SPARE_A
    );
    let f = frame(2000, BIAS);
    for mode in [Mode::Current, Mode::Velocity, Mode::Position] {
        for cap in [true, false] {
            let sh = Shared::new();
            seed(&sh);
            sh.table.with_mut(|t| {
                let l = &mut t.control.lifecycle;
                l.mode = mode;
                l.goal_current = 150;
                l.goal_velocity = 800;
                l.goal_position = 3000;
                l.torque_enable = true;
            });
            let mut k = kernel();
            periods(&mut k, &sh, f, 4);
            let case = (mode, cap);
            assert!(
                matches!(k.cmd.drive, fast::Drive::Closed { .. }),
                "{case:?}"
            );
            assert_eq!((k.cfg.belt, k.cfg.unset_flag), (0, 0), "{case:?}");
            write_config(&sh, |t| {
                if cap {
                    t.config.limits.current_limit_counts = 0;
                } else {
                    t.config.loop_position.speed_ceiling_cps = 0;
                }
            });
            run_to(&mut k, &sh, f, phase::SPARE_A);
            k.on_scan(f, &sh);
            let (lp, lim) = (k.cfg.loop_position, k.cfg.limits);
            let held = if cap {
                (2400, 3000, 3000)
            } else {
                (1200, 0, 0)
            };
            assert_eq!(
                (
                    lim.current_limit_counts,
                    lp.speed_ceiling_cps,
                    lp.velocity_limit_cps
                ),
                held,
                "{case:?}: the copy"
            );
            assert_eq!(k.cfg.belt, config::belt::LIMITS_UNSET, "{case:?}");
            assert_eq!(k.cfg.unset_flag, limits::flag::UNSET, "{case:?}");
            assert!(
                !config::belt::refuses(k.cfg.belt, Mode::OpenLoop),
                "{case:?}"
            );
            while k.phase != phase::CONTROL {
                k.on_scan(f, &sh);
                assert!(
                    matches!(k.cmd.drive, fast::Drive::Closed { .. }),
                    "{case:?}: the band of the take's period"
                );
            }
            for n in 0..2 * DECIM_MED {
                k.on_scan(f, &sh);
                assert!(matches!(k.cmd.drive, fast::Drive::Off), "{case:?} tick {n}");
                // the CONTROL tick's write ran on the band built before
                // the take
                assert_eq!(
                    matches!(last_cmd(&k), MotorCmd::Disabled),
                    n != 0,
                    "{case:?} tick {n}"
                );
            }
            assert_eq!(k.faults.code(), faults::CODE_DATA, "{case:?}");
            let flags = sh.table.with(|t| t.telemetry.limits.limit_flags);
            assert_ne!(flags & limits::flag::UNSET, 0, "{case:?}");
        }
    }
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
    let mut words = [0u32; SPAN_BYTES / 4];
    sh.table.with(|t| {
        // SAFETY: the table is repr(C) bytes, the span word-aligned.
        let p = (t as *const ControlTable).cast::<u32>();
        for (i, w) in words.iter_mut().enumerate() {
            *w = unsafe { p.add(SPAN_START as usize / 4 + i).read() };
        }
    });
    for w in words.iter_mut() {
        // byte 0 of each word is a plain integer
        *w = w.wrapping_add(1);
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
fn a_take_moves_the_probe_tag_at_spare_a_alone() {
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

// --- Mode changes under torque ---------------------------------------------

/// Runs `plant` until the next tick runs the CONTROL phase, where a mode
/// write lands.
fn plant_to_control(k: &mut Kernel<FakeIo>, sh: &Shared, plant: &mut Plant) {
    while k.phase != phase::CONTROL {
        plant_tick(k, sh, plant);
    }
}

/// The velocity loop entered from a mode that does not run it starts
/// without charge, as at the enable edge and out of the park. A Velocity
/// cruise charges the integrator with the plant's drag; Current mode at
/// zero lets the shaft run down with the loop not stepping; back in
/// Velocity at zero, the first velocity step is a fresh loop's on the same
/// inputs. A loop that kept its charge pushes the old cruise's current
/// into a shaft at rest.
#[test]
fn a_velocity_loop_entered_from_current_mode_carries_no_earlier_charge() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 1000;
        t.config.pos_soft.pos_max_soft_counts = 1_000_000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(100);
    run_plant(&mut k, &sh, &mut plant, 4000);
    assert!(plant.omega_cps > 800, "cruise {}", plant.omega_cps);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 0;
    });
    run_plant(&mut k, &sh, &mut plant, 2000);
    assert_eq!(plant.omega_cps, 0, "the shaft ran down");
    plant_to_control(&mut k, &sh, &mut plant);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 0;
    });
    while plant_tick(&mut k, &sh, &mut plant) != phase::VELOCITY {}
    let (lv, motor) = sh.table.with(|t| (t.config.loop_velocity, t.calib.motor));
    let fresh = VelocityLoop::new().step(
        k.medium.omega_ref_q16,
        k.medium.omega_hat,
        k.medium.traj.alpha_star_q16(),
        k.medium.traj.omega_star_q16(),
        k.medium.i_band,
        &lv,
        &motor,
        false,
    );
    assert_eq!(
        k.medium.i_ref_cc, fresh,
        "the loop came back with the cruise's charge"
    );
}

/// What the drive applied the tick before a switch from OpenLoop to
/// Current mode, and the duty of each tick from the switch to the end of
/// the next period. OpenLoop at 5000 on a still shaft, then Current at the
/// current that drive held. `r_q12` is the kernel's resistance.
fn open_loop_to_current(sh: &Shared, r_q12: u16, from_current: bool) -> (i16, [i16; 20]) {
    seed(sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.calib.motor.r_q12 = r_q12;
        if from_current {
            // a blind command a few ticks wide: every tick's width is a
            // pulse the bridge drops
            t.control.lifecycle.mode = Mode::Current;
            t.control.lifecycle.goal_current = 5;
        } else {
            t.control.lifecycle.mode = Mode::OpenLoop;
            t.control.lifecycle.goal_duty = 5000;
        }
    });
    let mut k = kernel();
    let mut plant = held_plant();
    run_plant(&mut k, sh, &mut plant, 400);
    if from_current {
        let w = k.fast.ticks;
        assert!(w != 0 && w < TIMING.bridge_min_ticks, "width {w}");
        plant_to_control(&mut k, sh, &mut plant);
        sh.table.with_mut(|t| {
            t.control.lifecycle.mode = Mode::OpenLoop;
            t.control.lifecycle.goal_duty = 5000;
        });
        run_plant(&mut k, sh, &mut plant, 400);
    }
    assert_eq!(k.fast.duty_q15, 5000);
    // the winding's current under that drive: 5000 of 3000 vcounts over 2.0
    let held = k.fast.i_meas_last;
    assert_eq!(held, 228);
    plant_to_control(&mut k, sh, &mut plant);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = held;
    });
    let before = k.fast.duty_q15;
    let mut duties = [0; 20];
    for d in &mut duties {
        plant_tick(&mut k, sh, &mut plant);
        *d = k.fast.duty_q15;
    }
    (before, duties)
}

/// A switch from OpenLoop to a closed mode enters the current loop on the
/// winding as the drive left it: the reference is the current that flows
/// and the integral the volts the bridge applied, so the duty runs on
/// through the switch. Here the new goal is that same current, so every
/// tick from the switch on drives within a sixteenth of the OpenLoop duty.
/// A loop entered on a zero reference and an empty integral reverses the
/// drive at the switch on its proportional gain alone.
#[test]
fn a_switch_from_open_loop_to_current_mode_continues_the_drive() {
    let sh = Shared::new();
    let (before, duties) = open_loop_to_current(&sh, 8192, false);
    for (n, d) in duties.into_iter().enumerate() {
        assert!(
            (d as i32 - before as i32).abs() <= before as i32 / 16,
            "tick {n} from the switch: duty {d} after {before}"
        );
    }
}

/// The first closed-loop tick after an OpenLoop drive takes its window's
/// reading: whether the bridge dropped the pulse ahead of that window is
/// a fact of the widths the drive wrote, whatever drive wrote them. A
/// Current-mode run whose blind command is a few ticks wide ends on such
/// a pulse; OpenLoop at 5000 follows, then the switch back to Current.
/// The kernel's resistance is half the winding's, so a tick that took no
/// reading would command half the volts; taken, the duty runs on as in
/// `a_switch_from_open_loop_to_current_mode_continues_the_drive`.
#[test]
fn the_first_closed_tick_after_open_loop_takes_its_reading() {
    let sh = Shared::new();
    let (before, duties) = open_loop_to_current(&sh, 4096, true);
    assert!(
        (duties[0] as i32 - before as i32).abs() <= before as i32 / 16,
        "the switch's tick drove {} after {before}",
        duties[0]
    );
}

/// A switch between the two profile modes keeps the profile's speed: the
/// reference moves on from where the shaft is, at the speed it was asked
/// for, and the new mode's goal shapes it from there under the accel
/// limit. A Velocity cruise at the velocity limit switched to a far
/// Position goal, and that move switched back to Velocity at the same
/// speed: the shaft never drops a tenth under its speed at the switch.
/// A profile restarted at rest asks the loop to stop the shaft and brakes
/// it at each switch.
#[test]
fn a_switch_between_velocity_and_position_keeps_the_profiles_speed() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 1000;
        t.config.loop_position.velocity_limit_cps = 1000;
        t.config.pos_soft.pos_max_soft_counts = 1_000_000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(100);
    run_plant(&mut k, &sh, &mut plant, 2000);
    let writes: [fn(&mut ControlTable); 2] = [
        |t| {
            t.control.lifecycle.mode = Mode::Position;
            t.control.lifecycle.goal_position = 3500;
        },
        |t| t.control.lifecycle.mode = Mode::Velocity,
    ];
    for (leg, write) in writes.into_iter().enumerate() {
        plant_to_control(&mut k, &sh, &mut plant);
        let cruise = plant.omega_cps;
        assert!(
            cruise > 800,
            "leg {leg}: cruise {cruise} ahead of the switch"
        );
        sh.table.with_mut(write);
        let mut low = i32::MAX;
        for _ in 0..1000 {
            plant_tick(&mut k, &sh, &mut plant);
            low = low.min(plant.omega_cps);
        }
        assert!(
            low >= cruise - cruise / 10,
            "leg {leg}: the switch slowed the shaft from {cruise} to {low}"
        );
    }
    assert_eq!(k.faults.mask(), 0);
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
    /// Ticks run through the first CONTROL phase after the write: that
    /// tick's write still drives the command before it.
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
        boundary: ((DECIM_MED - phase) % DECIM_MED) as usize + 1,
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
            unfolded(t);
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

/// The SLOW block's stages land one per period: the due stage, then the
/// finish on the next period's SLOW phase, then nothing until the next
/// due period, DECIM_SLOW periods after the first.
#[test]
fn the_slow_stages_take_one_period_each() {
    let f = frame(2000, BIAS);
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    let mut stages: heapless::Vec<(u32, bool, bool), 4> = heapless::Vec::new();
    for t in 0..2 * SLOW {
        let before = k.probe_tags();
        k.on_scan(f, &sh);
        let ev = Events::between(before, k.probe_tags());
        if ev.slow || ev.finish {
            stages
                .push((t, ev.slow, ev.finish))
                .expect("four stages at most");
        }
    }
    let due = phase::SLOW as u32;
    let med = DECIM_MED as u32;
    assert_eq!(
        stages[..],
        [
            (due, true, false),
            (due + med, false, true),
            (due + SLOW, true, false),
            (due + SLOW + med, false, true),
        ]
    );
}

/// The position loop's output clamps at the speed ceiling, not at the move
/// speed: the profile cruises at `velocity_limit_cps`, and a shaft held
/// back behind it is asked for more, up to the ceiling, so it can catch
/// up. The seed's ceiling is 3000 c/s; the move speed here is 300. On main
/// the clamp is the move speed and the loop never asks over 300 c/s.
#[test]
fn a_held_shaft_is_asked_over_the_move_speed_up_to_the_ceiling() {
    const MOVE_CPS: i32 = 300;
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.config.loop_position.velocity_limit_cps = MOVE_CPS as u16;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.control.lifecycle.goal_position = 3000;
        t.control.lifecycle.torque_enable = true;
    });
    let mut k = kernel();
    let mut plant = Plant::new(500);
    run_plant(&mut k, &sh, &mut plant, 8_000);
    plant.frozen = true;
    run_plant(&mut k, &sh, &mut plant, 800);
    plant.frozen = false;
    let mut asked = i32::MIN;
    for _ in 0..400 {
        run_plant(&mut k, &sh, &mut plant, 1);
        asked = asked.max(k.medium.omega_ref_q16);
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(
        asked > (MOVE_CPS + 5) << 16,
        "the loop asks at most {:.1} c/s of a held shaft, the move speed {MOVE_CPS}",
        asked as f64 / 65536.0
    );
    assert!(asked <= 3000 << 16, "{asked}");
}

/// The chip side's reports around the motor write: a write past its load
/// point, a scan that never completed the frame, and ticks that stopped.
mod streaming {
    use super::*;

    struct ProbeSensors {
        lost: bool,
    }
    impl Sensors for ProbeSensors {
        fn land(&mut self, _: &Shared) {}
        fn complete(&mut self, _: &Shared, _: bool) -> bool {
            !self.lost
        }
    }

    struct ProbeMotor {
        late: bool,
        last: Option<MotorCmd>,
        writes: u32,
    }
    impl Motor for ProbeMotor {
        fn write(&mut self, cmd: MotorCmd) {
            self.last = Some(cmd);
            self.writes += 1;
        }
        fn late(&self) -> bool {
            self.late
        }
    }

    struct ProbeIo {
        sensors: ProbeSensors,
        motor: ProbeMotor,
    }
    impl ControlIo for ProbeIo {
        type Sensors = ProbeSensors;
        type Motor = ProbeMotor;
        fn parts(&mut self) -> (&mut ProbeSensors, &mut ProbeMotor) {
            (&mut self.sensors, &mut self.motor)
        }
    }

    /// OpenLoop at 8000 on a +200 shunt, past the slew: every window reads.
    fn driving(sh: &Shared) -> Kernel<ProbeIo> {
        drive_setup(sh);
        let mut k = Kernel::new(
            ProbeIo {
                sensors: ProbeSensors { lost: false },
                motor: ProbeMotor {
                    late: false,
                    last: None,
                    writes: 0,
                },
            },
            TIMING,
        );
        for _ in 0..200 {
            k.on_scan(frame(2000, BIAS + 200), sh);
        }
        assert_ne!(tick_block(sh).tick_flags & tick_flag::I_VALID, 0);
        k
    }

    fn stays_off(k: &mut Kernel<ProbeIo>, sh: &Shared) {
        for _ in 0..2 * DECIM_MED {
            k.on_scan(frame(2000, BIAS + 200), sh);
            assert!(matches!(k.io.motor.last, Some(MotorCmd::Disabled)));
        }
        assert_eq!(k.faults.mask(), faults::BIT_SENSOR);
        sh.table.with(|t| {
            assert_eq!(t.telemetry.mode.fault_code, faults::CODE_SENSOR);
            assert_eq!(t.telemetry.common.fault_flags, faults::BIT_SENSOR);
        });
    }

    /// A write that lands counting up loads at the next crest, mid-pulse:
    /// the window that pulse drives is no current reading, and the next
    /// write on time restores it.
    #[test]
    fn a_late_write_voids_the_next_windows_current() {
        let sh = Shared::new();
        let mut k = driving(&sh);
        k.io.motor.late = true;
        k.on_scan(frame(2000, BIAS + 200), &sh);
        assert!(k.write_late());
        assert_ne!(
            tick_block(&sh).tick_flags & tick_flag::I_VALID,
            0,
            "the late write's own tick measured an earlier window"
        );
        for _ in 0..3 {
            k.on_scan(frame(2000, BIAS + 200), &sh);
            assert_eq!(tick_block(&sh).tick_flags & tick_flag::I_VALID, 0);
        }
        k.io.motor.late = false;
        k.on_scan(frame(2000, BIAS + 200), &sh);
        assert!(!k.write_late());
        assert_eq!(tick_block(&sh).tick_flags & tick_flag::I_VALID, 0);
        k.on_scan(frame(2000, BIAS + 200), &sh);
        assert_ne!(tick_block(&sh).tick_flags & tick_flag::I_VALID, 0);
        assert_eq!(k.faults.mask(), 0);
    }

    /// A scan that never lands may be a converter that never ticks the
    /// kernel again: the sensor fault latches and publishes, and the drive
    /// goes off in the same tick, after the write it already made, and
    /// stays off.
    #[test]
    fn a_scan_that_never_lands_faults_and_writes_the_drive_off() {
        let sh = Shared::new();
        let mut k = driving(&sh);
        let before = k.io.motor.writes;
        k.io.sensors.lost = true;
        k.on_scan(frame(2000, BIAS + 200), &sh);
        assert_eq!(k.io.motor.writes, before + 2);
        assert!(matches!(k.io.motor.last, Some(MotorCmd::Disabled)));
        assert_eq!(k.faults.mask(), faults::BIT_SENSOR);
        k.io.sensors.lost = false;
        stays_off(&mut k, &sh);
    }
}
