//! OpenLoop duty against the current band (`kernel::duty_limit`): the
//! kernel drives the MG90-scale R-L plant rig, so every pin reads the
//! winding current the limiter never sees directly, only through the shunt
//! window one tick late.

use osc_integration::plant::{
    FakeIo, RL_R_Q12, RL_VBUS, RlPlant, TIMING, duty_of, kernel, last_cmd, seed, stamp,
};
use osc_servo_core::estimator::window::floor_duty;
use osc_servo_core::kernel::duty_limit::UP_Q15;
use osc_servo_core::kernel::faults::{BIT_STALL, CODE_STALL};
use osc_servo_core::{Kernel, Mode, MotorCmd, RegionStorage, Shared, StallResponse};

const LIM: u16 = 280;
/// 160 of ARR 1200: the window floor is 13.3% duty.
const I_FLOOR_TICKS: u16 = 160;
const GOAL_64: i16 = 20971;
const GOAL_FULL: i16 = i16::MAX;
const MID: u16 = 2050;
const STALL_MS: u16 = 500;
const YIELD: u16 = 168;
const RELEASE: u16 = 84;
/// FAST ticks per ms.
const MS: u32 = 20;
/// Bound on the observer's settle past `stall_time_ms` before omega reads
/// slow and the timer runs out.
const SETTLE_MS: u32 = 100;
/// Under the rig's stall current at the window floor (239 counts): only a
/// duty under the floor can hold it.
const BLIND_LIM: u16 = 200;

fn rig(lim: u16) -> Shared {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.calib.sense.i_window_min_ticks = I_FLOOR_TICKS;
        t.calib.sense.v_window_min_ticks = I_FLOOR_TICKS;
        t.calib.motor.r_q12 = RL_R_Q12;
        t.config.limits.current_limit_counts = lim;
        // the collision trip folds the limit on its own: not under test here
        t.config.limits.stall_tau_trip_counts = u16::MAX;
        t.config.limits.openloop_zero_brake = false;
        t.control.lifecycle.mode = Mode::OpenLoop;
        // the mg90-a observer and motor model: `seed`'s gains ring for over
        // a second after a current step, landing the stall trip near 2 s
        // instead of at stall_time_ms plus the observer's settle
        t.config.fusion.l1_q016 = 8640;
        t.config.fusion.l2_q88 = 3180;
        t.config.fusion.l3_q88 = 84;
        t.calib.motor.b_i_q313 = 2427;
        t.calib.motor.fric_fc_counts = 53;
        t.calib.motor.ke_vpc_q = 603;
        t.calib.motor.recip_ke_q = 6957;
    });
    stamp(&sh);
    sh
}

/// The bench stall settings: mg90-a's stall time, the yield and release
/// folded under the 280 limit.
fn stall_rig(response: StallResponse) -> Shared {
    let sh = rig(LIM);
    sh.table.with_mut(|t| {
        let l = &mut t.config.limits;
        l.stall_response = response;
        l.stall_time_ms = STALL_MS;
        l.stall_yield_counts = YIELD;
        l.stall_release_counts = RELEASE;
    });
    stamp(&sh);
    sh
}

fn floor() -> i16 {
    floor_duty(I_FLOOR_TICKS, TIMING.pwm_arr, TIMING.recip_arr_q24) as i16
}

/// Per tick: the plant steps under the kernel's last write, the kernel
/// ticks on the frame. Returns (applied duty, winding current) per tick.
fn run(k: &mut Kernel<FakeIo>, sh: &Shared, p: &mut RlPlant, ticks: u32) -> Vec<(i16, i32)> {
    (0..ticks)
        .map(|_| {
            let duty = k.io.motor.last.map_or(0, duty_of);
            k.on_tick(p.step(duty), sh);
            (duty_of(last_cmd(k)), p.current())
        })
        .collect()
}

/// Torque-off rest, then torque on at `goal`.
fn start(sh: &Shared, p: &mut RlPlant, goal: i16) -> Kernel<FakeIo> {
    let mut k = kernel();
    run(&mut k, sh, p, 400);
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_duty = goal;
        t.control.lifecycle.torque_enable = true;
    });
    k
}

fn peak(r: &[(i16, i32)]) -> i32 {
    r.iter().map(|&(_, i)| i.abs()).max().unwrap_or(0)
}

fn mean(r: &[(i16, i32)]) -> i32 {
    (r.iter().map(|&(_, i)| i.abs() as i64).sum::<i64>() / r.len() as i64) as i32
}

fn faults(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.common.fault_flags)
}

/// The rig's steady winding current at `duty` with the rotor held.
fn stall_counts(duty: i16) -> i32 {
    let v = (duty.unsigned_abs() as i32 * RL_VBUS as i32) >> 15;
    (v << 12) / RL_R_Q12 as i32
}

/// The blind band carried the hold: most ticks sat under the floor, every
/// one of them at a duty whose stall current is `lim` or just under it.
fn assert_blind_band_caps(r: &[(i16, i32)], lim: u16, what: &str) {
    let blind: Vec<i16> = r
        .iter()
        .map(|&(d, _)| d)
        .filter(|&d| d.abs() < floor())
        .collect();
    assert!(
        blind.len() > r.len() / 2,
        "{what}: {} blind ticks",
        blind.len()
    );
    for d in blind {
        let i = stall_counts(d);
        let lim = lim as i32;
        assert!(
            (lim * 95 / 100..=lim).contains(&i),
            "{what}: duty {d} stalls at {i}"
        );
    }
}

fn i_lim(sh: &Shared) -> u16 {
    sh.table.with(|t| t.telemetry.estimates.i_lim_counts)
}

/// Mid-travel locked rotor at `goal` until the stall timer `acted`, which
/// must hold off for the whole `stall_time_ms` and act inside the settle.
fn locked_stall(sh: &Shared, goal: i16, acted: fn(&Shared) -> bool) -> (Kernel<FakeIo>, RlPlant) {
    let mut p = RlPlant::new(MID);
    p.locked = true;
    let mut k = start(sh, &mut p, goal);
    let at = (0..(STALL_MS as u32 + SETTLE_MS) * MS)
        .find(|_| {
            run(&mut k, sh, &mut p, 1);
            acted(sh)
        })
        .unwrap_or_else(|| panic!("goal {goal}: the stall timer never acted"));
    assert!(
        at >= STALL_MS as u32 * MS,
        "goal {goal}: acted {} ms in",
        at / MS
    );
    (k, p)
}

/// 2 s against the rotor, 100 ms of edge skipped for the mean.
fn assert_holds_at_the_limit(sh: &Shared, r: &[(i16, i32)], what: &str) {
    let lim = LIM as i32;
    let (pk, mn) = (peak(r), mean(&r[2_000..]));
    assert!(pk <= lim * 11 / 10, "{what}: peak {pk}");
    assert!((lim * 85 / 100..=lim).contains(&mn), "{what}: mean {mn}");
    assert_eq!(faults(sh), 0, "{what}");
}

#[test]
fn openloop_locked_rotor_mid_travel_holds_at_i_lim() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = rig(LIM);
        let mut p = RlPlant::new(MID);
        p.locked = true;
        let mut k = start(&sh, &mut p, goal);
        let r = run(&mut k, &sh, &mut p, 40_000);
        assert_holds_at_the_limit(&sh, &r, &format!("goal {goal}"));
    }
}

#[test]
fn openloop_stall_holds_at_i_lim() {
    // seated on a hard stop inside the soft limits: nothing but the
    // limiter stands between the goal and the stall current
    for (goal, stop) in [(GOAL_64, 3000), (-GOAL_64, 1000)] {
        let sh = rig(LIM);
        let mut p = RlPlant::new(stop as u16);
        (p.stop_lo, p.stop_hi) = (1000, 3000);
        let mut k = start(&sh, &mut p, goal);
        let r = run(&mut k, &sh, &mut p, 40_000);
        assert_eq!(p.pos(), stop, "seated");
        assert_holds_at_the_limit(&sh, &r, &format!("stop {stop}"));
    }
}

#[test]
fn openloop_first_edge_stays_under_i_lim() {
    for goal in [GOAL_FULL, -GOAL_FULL] {
        let sh = rig(LIM);
        let mut p = RlPlant::new(MID);
        p.locked = true;
        let mut k = start(&sh, &mut p, goal);
        let r = run(&mut k, &sh, &mut p, 400);
        // the ceiling stops one measured tick late: the few counts over
        // that the hold shows too, never the step's stall current
        assert!(
            peak(&r) <= LIM as i32 * 105 / 100,
            "goal {goal}: peak {}",
            peak(&r)
        );
    }
}

#[test]
fn openloop_ceiling_releases_when_the_shaft_frees() {
    let goal = 13107;
    let sh = rig(LIM);
    let mut p = RlPlant::new(1000);
    p.locked = true;
    let mut k = start(&sh, &mut p, goal);
    let r = run(&mut k, &sh, &mut p, 4_000);
    assert!(r[2_000..].iter().all(|&(d, _)| d < goal), "governed");
    p.locked = false;
    let r = run(&mut k, &sh, &mut p, 4_000);
    let at = r
        .iter()
        .position(|&(d, _)| d == goal)
        .expect("the goal duty returns");
    assert!(at <= 2_000, "goal duty after {at} ticks");
    assert!(peak(&r) <= LIM as i32 * 11 / 10, "peak {}", peak(&r));
    assert!(r[at..].iter().all(|&(d, _)| d == goal), "stays at the goal");
}

#[test]
fn openloop_blind_goal_is_untouched() {
    let goal = 3932;
    assert!(goal < floor());
    let sh = rig(LIM);
    let mut p = RlPlant::new(MID);
    p.locked = true;
    let mut k = start(&sh, &mut p, goal);
    let r = run(&mut k, &sh, &mut p, 2_000);
    assert!(r.iter().all(|&(d, _)| d == goal));
}

#[test]
fn openloop_reversal_restarts_from_the_floor() {
    let sh = rig(LIM);
    let mut p = RlPlant::new(MID);
    p.locked = true;
    let mut k = start(&sh, &mut p, GOAL_64);
    run(&mut k, &sh, &mut p, 2_000);
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_duty = -GOAL_64);
    let r = run(&mut k, &sh, &mut p, 400);
    let first = -r[0].0;
    assert!(
        (floor()..=floor() + UP_Q15 as i16).contains(&first),
        "first reversed duty {first}"
    );
    for (n, &(d, _)) in r.iter().enumerate() {
        let slew = floor() as i32 + (n as i32 + 1) * UP_Q15 as i32;
        assert!(d < 0 && -(d as i32) <= slew, "tick {n}: {d}");
    }
    assert!(peak(&r) <= LIM as i32 * 11 / 10, "peak {}", peak(&r));
}

#[test]
fn openloop_wall_hit_at_speed_recovers_inside_1ms() {
    for (goal, stop) in [(GOAL_64, 2500), (-GOAL_64, 1500)] {
        let sh = rig(LIM);
        let mut p = RlPlant::new(MID);
        (p.stop_lo, p.stop_hi) = (1500, 2500);
        let mut k = start(&sh, &mut p, goal);
        let mut speed = 0;
        let mut hit = None;
        let mut r = Vec::new();
        for n in 0..20_000 {
            let w = p.omega_cps();
            r.extend(run(&mut k, &sh, &mut p, 1));
            if hit.is_none() && p.pos() == stop && p.omega_cps() == 0 {
                (hit, speed) = (Some(n), w);
            }
        }
        let hit = hit.expect("the shaft reaches the stop");
        let over: Vec<usize> = (hit..r.len())
            .filter(|&n| r[n].1.abs() > LIM as i32 * 11 / 10)
            .collect();
        assert!(speed.abs() > 1_000, "at speed: {speed} c/s");
        assert!(
            over.iter().all(|&n| n < hit + 20),
            "over after 1 ms: {over:?}"
        );
        assert_eq!(faults(&sh), 0);
    }
}

#[test]
fn openloop_stall_yields_like_closed_loop() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = stall_rig(StallResponse::Yield);
        let (mut k, mut p) = locked_stall(&sh, goal, |sh| i_lim(sh) != LIM);
        assert_eq!(i_lim(&sh), YIELD, "goal {goal}");
        let r = run(&mut k, &sh, &mut p, 20_000);
        assert_eq!(i_lim(&sh), YIELD, "goal {goal}: still folded");
        assert!(peak(&r) <= LIM as i32, "goal {goal}: peak {}", peak(&r));
        assert_eq!(faults(&sh), 0, "goal {goal}");
    }
}

#[test]
fn openloop_stall_faults_on_the_boot_response() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = stall_rig(StallResponse::Fault);
        let (k, _) = locked_stall(&sh, goal, |sh| faults(sh) != 0);
        assert_eq!(faults(&sh), BIT_STALL, "goal {goal}");
        assert_eq!(sh.table.with(|t| t.telemetry.mode.fault_code), CODE_STALL);
        assert!(matches!(last_cmd(&k), MotorCmd::Disabled), "goal {goal}");
    }
}

#[test]
fn openloop_slew_never_counts_as_a_stall() {
    // the shortest timer the table holds, two MEDIUM ticks: a blind start
    // from rest, then the slew from the floor to the goal at speed
    for (cruise, goal, from) in [(3932, GOAL_64, 300), (-3932, -GOAL_64, 3800)] {
        let sh = stall_rig(StallResponse::Fault);
        sh.table.with_mut(|t| t.config.limits.stall_time_ms = 1);
        stamp(&sh);
        let mut p = RlPlant::new(from);
        let mut k = start(&sh, &mut p, cruise);
        run(&mut k, &sh, &mut p, 2_000);
        sh.table.with_mut(|t| t.control.lifecycle.goal_duty = goal);
        let mut r = Vec::new();
        while (300..=3800).contains(&p.pos()) && r.len() < 40_000 {
            r.extend(run(&mut k, &sh, &mut p, 1));
        }
        assert!(r.iter().any(|&(d, _)| d == goal), "goal {goal}: reached");
        assert_eq!(faults(&sh), 0, "goal {goal}");
        assert_eq!(i_lim(&sh), LIM, "goal {goal}");
    }
}

#[test]
fn openloop_stall_under_permit_never_trips() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = stall_rig(StallResponse::Fault);
        sh.table
            .with_mut(|t| t.control.lifecycle.stall_permit = true);
        let mut p = RlPlant::new(MID);
        p.locked = true;
        let mut k = start(&sh, &mut p, goal);
        let r = run(&mut k, &sh, &mut p, 40_000);
        assert_holds_at_the_limit(&sh, &r, &format!("goal {goal}"));
        assert_eq!(i_lim(&sh), LIM, "goal {goal}");
    }
}

#[test]
fn blind_band_caps_at_i_lim_when_r_is_known() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = rig(BLIND_LIM);
        let mut p = RlPlant::new(MID);
        p.locked = true;
        let mut k = start(&sh, &mut p, goal);
        let r = run(&mut k, &sh, &mut p, 40_000);
        let what = format!("goal {goal}");
        assert_blind_band_caps(&r[2_000..], BLIND_LIM, &what);
        let mn = mean(&r[2_000..]);
        assert!(mn <= BLIND_LIM as i32 * 11 / 10, "{what}: mean {mn}");
        assert_eq!(faults(&sh), 0, "{what}");
    }
}

#[test]
fn yield_fold_reaches_the_blind_band() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = stall_rig(StallResponse::Yield);
        let (mut k, mut p) = locked_stall(&sh, goal, |sh| i_lim(sh) != LIM);
        let r = run(&mut k, &sh, &mut p, 20_000);
        let what = format!("goal {goal}");
        assert_eq!(i_lim(&sh), YIELD, "{what}");
        assert_blind_band_caps(&r, YIELD, &what);
        let mn = mean(&r);
        assert!(mn <= YIELD as i32 * 12 / 10, "{what}: mean {mn}");
    }
}

#[test]
fn virgin_blind_band_passes_to_the_window_floor() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = rig(BLIND_LIM);
        sh.table.with_mut(|t| t.calib.motor.r_q12 = 0);
        stamp(&sh);
        let mut p = RlPlant::new(MID);
        p.locked = true;
        let mut k = start(&sh, &mut p, goal);
        let r = run(&mut k, &sh, &mut p, 40_000);
        let what = format!("goal {goal}");
        assert!(r.iter().all(|&(d, _)| d.abs() >= floor()), "{what}");
        // over the limit at the floor, the ceiling stays pinned there
        assert!(
            r[2_000..].iter().all(|&(d, _)| d.abs() == floor()),
            "{what}"
        );
        assert!(peak(&r) <= stall_counts(floor()) + 1, "{what}");
    }
}
