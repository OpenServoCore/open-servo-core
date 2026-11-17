//! The stored limits and the one stored resistance: pins of the layout
//! change that moves CONFIG and CALIB once. `r_q12` is the winding at
//! 25.00 C and the kernel reads it at the winding estimate's temperature;
//! the speed ceiling bounds Position, Velocity and Current mode, the move
//! speed under it; a servo whose current cap or speed ceiling is unset
//! (0, the factory block's value) refuses every closed mode and still
//! runs OpenLoop. The rigs are the servo of record's (`mg90_d`), whose set
//! carries the ceiling, and the factory block as the chip seeds it.

use osc_integration::mg90::{self, Mg90};
use osc_integration::mg90_d;
use osc_integration::plant::{BIAS, FakeIo, RlPlant, TIMING, duty_of, kernel, last_cmd};
use osc_integration::sim::{DEV_V006_SENSE as SENSE, DEV_V006_SENSE_EXT as SENSE_EXT};
use osc_servo_core::estimator::thermal::{NtcCfg, ThermCfg, WindingTherm, ntc_cc};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::kernel::phases::loops::faults::{BIT_DATA, CODE_DATA};
use osc_servo_core::regions::config::{
    DEFAULT_RAIL_SOURCE_MOHM, DEFAULT_V_UNDERVOLT_MV, rail_fold_q8, vmotor_counts,
};
use osc_servo_core::{
    BaudRate, ConfigDefaults, CurrentDefaults, ImageState, Kernel, Mode, MotorCmd, RegionStorage,
    Shared,
};

/// The set of record's speed ceiling, c/s: the maker's 600 deg/s at
/// 4.8 V derated by 2/3, on the set's 20.13 counts per degree, 8050.65
/// stored as 8050.
const CEILING_CPS: f64 = 8050.0;
/// The scorecard's test speed, the move speed these pins run at.
const MOVE_CPS: u16 = 1500;
/// `limit_flags` bits the design assigns from the two retired ones.
const FLAG_SPEED: u8 = 1 << 4;
const FLAG_UNSET: u8 = 1 << 5;

fn faults(sh: &Shared) -> (u8, u8) {
    sh.table
        .with(|t| (t.telemetry.common.fault_flags, t.telemetry.mode.fault_code))
}

fn limit_flags(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.limits.limit_flags)
}

/// The record rig at rest at `lin`, torque off, `cfg` over the set.
fn record(lin: f64, cfg: impl FnOnce(&mut osc_servo_core::ControlTable)) -> (Shared, Mg90) {
    let sh = mg90_d::rig(|t| {
        t.control.lifecycle.torque_enable = false;
        cfg(t);
    });
    (sh, mg90_d::plant(lin, 1))
}

/// Ticks the plant under the kernel's last write, the kernel on the frame;
/// the rotor's speed and the command each tick.
fn run(k: &mut Kernel<FakeIo>, sh: &Shared, p: &mut Mg90, ticks: u32) -> Vec<(f64, MotorCmd)> {
    (0..ticks)
        .map(|_| {
            let f = p.step(k.io.motor.last);
            k.on_scan(f, sh);
            (p.omega, last_cmd(k))
        })
        .collect()
}

fn torque_on(sh: &Shared, mode: Mode, f: impl FnOnce(&mut osc_servo_core::ControlTable)) {
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = mode;
        f(t);
        t.control.lifecycle.torque_enable = true;
    });
}

// --- D1: one stored resistance, at 25.00 C ---

/// The bench board's NTC curve and the record's rotor gain.
const NTC: NtcCfg = NtcCfg {
    raw_ref: 2048,
    t_ref_cc: 2500,
    k1_q88: 563,
    k2_q24: 5800,
};
const CFG: ThermCfg = ThermCfg {
    rise_q016: 1044,
    rise2_q016: 0,
    ntc: NTC,
};
/// Copper's slope as the kernel carries it, per centi-C.
const ALPHA_PER_CC: f64 = 2637.0 / (1u64 << 26) as f64;

/// `r_q12` is the winding at 25.00 C, so the resistance the loops read is
/// it raised or lowered by copper's slope on the estimate's temperature
/// over 25 C: the board's own offset and the rise on the heat. A cold board
/// reads under `r_q12`, a warm one over, 25.00 C with no heat reads it, and
/// with no NTC curve or a failed NTC it is `r_q12` itself. On main the
/// board's offset is not read: a 35 C board with no heat reads `r_q12`.
#[test]
fn r_hat_reads_the_25_c_resistance_at_the_estimates_temperature() {
    const R25: u16 = 3725;
    // boards at about -2, 9, 17, 25, 34, 53 and 67 C on the curve
    for raw in [3700u16, 2906, 2445, 2048, 1667, 958, 500] {
        let t_ntc = ntc_cc(raw, &NTC).expect("in the band") as f64;
        for heat in [0u32, 277 * 277] {
            let mut th = WindingTherm::new();
            let t_w = th.update(raw, [heat, 0], &CFG) as f64;
            let want = R25 as f64 * (1.0 + ALPHA_PER_CC * (t_w - 2500.0));
            let got = th.r_hat_q12(R25);
            assert!(
                (got as f64 - want).abs() <= 2.0,
                "raw {raw} ({:.2} C board, {:.2} C winding): r_hat {got}, want {want:.1}",
                t_ntc / 100.0,
                t_w / 100.0
            );
        }
    }
    let mut th = WindingTherm::new();
    th.update(2048, [0; 2], &CFG);
    assert_eq!(th.r_hat_q12(R25), R25, "25.00 C, no heat");
    th.update(2048, [0; 2], &ThermCfg::default());
    assert_eq!(th.r_hat_q12(R25), R25, "no NTC curve");
    th.update(0, [0; 2], &CFG);
    assert_eq!(th.r_hat_q12(R25), R25, "a failed NTC");
    assert_eq!(th.r_hat_q12(0), 0, "an unidentified winding");
}

// --- D2: the speed ceiling and the move speed ---

/// Current mode at the cap from the low end of travel: the band folds as
/// the speed nears the ceiling, so the shaft never turns past it, yet runs
/// within a tenth of it, and `limit_flags` names the fold while it binds.
/// On main Current mode has no ceiling and the shaft runs to its free
/// speed at the cap.
#[test]
fn current_mode_folds_under_the_speed_ceiling() {
    let (sh, mut p) = record(400.0, |_| {});
    let mut k = mg90::kernel();
    run(&mut k, &sh, &mut p, 400);
    let cap = sh.table.with(|t| t.config.limits.current_limit_counts) as i16;
    torque_on(&sh, Mode::Current, |t| {
        t.control.lifecycle.goal_current = cap
    });
    let (mut peak, mut flagged) = (0.0f64, false);
    for _ in 0..12_000 {
        let (omega, _) = run(&mut k, &sh, &mut p, 1)[0];
        peak = peak.max(omega);
        flagged |= limit_flags(&sh) & FLAG_SPEED != 0;
        if p.out > 3300.0 {
            break;
        }
    }
    assert_eq!(faults(&sh), (0, 0));
    assert!(
        peak <= CEILING_CPS,
        "Current mode at the cap peaks at {peak:.0} c/s, over the {CEILING_CPS} ceiling"
    );
    assert!(
        peak >= 0.9 * CEILING_CPS,
        "the fold holds it at {peak:.0} c/s"
    );
    assert!(flagged, "limit_flags never named the speed fold");
}

/// Velocity mode asked for more than the ceiling cruises at the ceiling,
/// whatever the move speed: the move speed is the Position profile's
/// cruise. On main the goal clamps at the move speed.
#[test]
fn velocity_mode_clamps_its_goal_at_the_ceiling() {
    let (sh, mut p) = record(400.0, |t| {
        t.config.loop_position.velocity_limit_cps = MOVE_CPS;
    });
    let mut k = mg90::kernel();
    run(&mut k, &sh, &mut p, 400);
    torque_on(&sh, Mode::Velocity, |t| {
        t.control.lifecycle.goal_velocity = 12_000
    });
    let rows = run(&mut k, &sh, &mut p, 5_000);
    let cruise = rows[3_000..].iter().map(|&(o, _)| o).sum::<f64>() / 2000.0;
    assert_eq!(faults(&sh), (0, 0));
    assert!(
        (cruise - CEILING_CPS).abs() <= 0.03 * CEILING_CPS,
        "cruise {cruise:.0} c/s against the {CEILING_CPS} ceiling"
    );
}

/// A move speed of 0 is the ceiling (Dynamixel's Profile Velocity 0): the
/// profile cruises there, and the shaft follows it within the velocity
/// loop's overshoot. On main a move speed of 0 pins the profile at rest
/// and nothing moves.
#[test]
fn a_zero_move_speed_cruises_at_the_ceiling() {
    let (sh, mut p) = record(400.0, |t| t.config.loop_position.velocity_limit_cps = 0);
    let mut k = mg90::kernel();
    run(&mut k, &sh, &mut p, 400);
    torque_on(&sh, Mode::Position, |t| {
        t.control.lifecycle.goal_position = 3400
    });
    let peak = run(&mut k, &sh, &mut p, 6_000)
        .iter()
        .map(|&(o, _)| o)
        .fold(0.0f64, f64::max);
    assert_eq!(faults(&sh), (0, 0));
    assert!(
        (0.9 * CEILING_CPS..=1.05 * CEILING_CPS).contains(&peak),
        "a zero move speed peaks at {peak:.0} c/s"
    );
}

// --- D3: a limit unset refuses closed loop ---

fn drives(cmds: &[(f64, MotorCmd)]) -> bool {
    cmds.iter()
        .any(|(_, c)| matches!(c, MotorCmd::Drive { ticks, .. } if *ticks != 0))
}

/// The record's set with the current cap at 0, its unset value: every
/// closed mode is refused at the enable with the data fault, nothing
/// drives, and `limit_flags` says a limit is unset with torque off and on.
/// OpenLoop still runs, under the board's own trip. On main the closed
/// modes run on a zero band without a fault, and OpenLoop's band is shut.
#[test]
fn a_servo_with_its_cap_unset_refuses_closed_loop_and_runs_openloop() {
    for mode in [Mode::Current, Mode::Velocity, Mode::Position] {
        let (sh, mut p) = record(1500.0, |t| t.config.limits.current_limit_counts = 0);
        let mut k = mg90::kernel();
        run(&mut k, &sh, &mut p, 400);
        assert_ne!(limit_flags(&sh) & FLAG_UNSET, 0, "{mode:?}: torque off");
        torque_on(&sh, mode, |t| {
            let l = &mut t.control.lifecycle;
            l.goal_current = 150;
            l.goal_velocity = 800;
            l.goal_position = 1900;
        });
        let cmds = run(&mut k, &sh, &mut p, 2 * DECIM_MED as u32);
        assert_eq!(faults(&sh), (BIT_DATA, CODE_DATA), "{mode:?}");
        assert!(
            cmds[DECIM_MED as usize..]
                .iter()
                .all(|(_, c)| matches!(c, MotorCmd::Disabled)),
            "{mode:?}: {:?}",
            &cmds[DECIM_MED as usize..]
        );
        assert_ne!(limit_flags(&sh) & FLAG_UNSET, 0, "{mode:?}: refused");
    }
    let (sh, mut p) = record(1500.0, |t| t.config.limits.current_limit_counts = 0);
    let mut k = mg90::kernel();
    run(&mut k, &sh, &mut p, 400);
    torque_on(&sh, Mode::OpenLoop, |t| {
        t.control.lifecycle.goal_duty = 5000
    });
    let cmds = run(&mut k, &sh, &mut p, 4_000);
    assert_eq!(faults(&sh), (0, 0), "OpenLoop");
    assert!(drives(&cmds), "OpenLoop drives");
    assert!(p.out - 1500.0 > 100.0, "OpenLoop moved to {:.0}", p.out);
}

/// A live write of a zero cap under a running loop stops it: the take
/// carries the unset limit and the next CONTROL latches the data fault,
/// so within two medium periods of the commit nothing drives. On main the
/// loop runs on a zero band, braking, with no fault.
#[test]
fn a_live_zero_cap_write_stops_a_running_closed_loop() {
    for mode in [Mode::Current, Mode::Velocity, Mode::Position] {
        let (sh, mut p) = record(1500.0, |_| {});
        let mut k = mg90::kernel();
        run(&mut k, &sh, &mut p, 400);
        torque_on(&sh, mode, |t| {
            let l = &mut t.control.lifecycle;
            l.goal_current = 150;
            l.goal_velocity = 800;
            l.goal_position = 2500;
        });
        assert!(drives(&run(&mut k, &sh, &mut p, 2_000)), "{mode:?}: drives");
        sh.config_commit_begin();
        sh.table
            .with_mut(|t| t.config.limits.current_limit_counts = 0);
        sh.config_commit_end();
        let cmds = run(&mut k, &sh, &mut p, 3 * DECIM_MED as u32);
        assert_eq!(faults(&sh), (BIT_DATA, CODE_DATA), "{mode:?}");
        assert!(
            cmds[2 * DECIM_MED as usize..]
                .iter()
                .all(|(_, c)| matches!(c, MotorCmd::Disabled)),
            "{mode:?}: {:?}",
            &cmds[2 * DECIM_MED as usize..]
        );
    }
}

/// The factory block as the chip seeds it with both images erased: no
/// limit is compiled in, so Current mode, a closed loop, is refused, while
/// OpenLoop runs. On main the factory block carries a compiled 500 mA cap
/// and Current mode runs on it.
#[test]
fn the_factory_block_refuses_current_mode_and_runs_openloop() {
    let sh = Shared::new();
    sh.table.seed_config_defaults(
        &ConfigDefaults {
            pos_min_phys_counts: 0,
            pos_max_phys_counts: 4095,
            id: 1,
            baud: BaudRate::B1000000,
            response_deadline_us: 500,
        },
        &CurrentDefaults::from_sense(SENSE.shunt_r_mohm, SENSE.gain_milli, SENSE.vdd_mv),
        vmotor_counts(
            DEFAULT_V_UNDERVOLT_MV,
            SENSE.vmotor_div_top as u32,
            SENSE.vmotor_div_bot as u32,
            SENSE.vdd_mv,
        ),
        rail_fold_q8(
            DEFAULT_RAIL_SOURCE_MOHM,
            SENSE.shunt_r_mohm,
            SENSE.gain_milli,
            SENSE.vdd_mv,
            SENSE.vmotor_div_top as u32,
            SENSE.vmotor_div_bot as u32,
        ),
    );
    sh.table
        .seed_calib_sense(&SENSE, &SENSE_EXT, TIMING.pwm_arr);
    sh.table.seed_current_bias(BIAS);
    sh.publish_data_state(ImageState::Virgin, ImageState::Virgin);
    let mut k = kernel();
    let mut p = RlPlant::new(1500);
    let mut run = |sh: &Shared, ticks: u32| -> Vec<MotorCmd> {
        (0..ticks)
            .map(|_| {
                let duty = duty_of(&k);
                k.on_scan(p.step(duty), sh);
                last_cmd(&k)
            })
            .collect()
    };
    run(&sh, 400);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 5000;
        t.control.lifecycle.torque_enable = true;
    });
    let cmds = run(&sh, 1_000);
    assert_eq!(faults(&sh), (0, 0), "OpenLoop");
    assert!(
        cmds.iter()
            .any(|c| matches!(c, MotorCmd::Drive { ticks, .. } if *ticks != 0)),
        "OpenLoop drives"
    );
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = false;
        t.control.lifecycle.goal_duty = 0;
    });
    run(&sh, 2 * DECIM_MED as u32);
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 200;
        t.control.lifecycle.torque_enable = true;
    });
    let cmds = run(&sh, 1_000);
    assert_eq!(faults(&sh), (BIT_DATA, CODE_DATA), "Current mode");
    assert!(
        cmds[DECIM_MED as usize..]
            .iter()
            .all(|c| matches!(c, MotorCmd::Disabled)),
        "Current mode drove on the factory block"
    );
}
