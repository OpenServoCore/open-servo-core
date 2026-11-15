//! A virgin servo on the class defaults: board data and the boot seed only,
//! nothing identified, nothing saved. The kernel drives the MG90-scale R-L
//! plant rig on the osc-dev-v006 sense chain (60 mohm, G 14.884).

use osc_integration::plant::{BIAS, FakeIo, RlPlant, TIMING, duty_of, kernel};
use osc_integration::sim::{DEV_V006_SENSE as SENSE, DEV_V006_SENSE_EXT as SENSE_EXT};
use osc_servo_core::kernel::phases::loops::faults::{CODE_DATA, CODE_NONE};
use osc_servo_core::regions::config::{
    DEFAULT_RAIL_SOURCE_MOHM, DEFAULT_V_UNDERVOLT_MV, STALL_TAU_TRIP_OFF, current_counts,
    rail_fold_q8, vmotor_counts,
};
use osc_servo_core::{
    BaudRate, ConfigDefaults, CurrentDefaults, ImageState, Kernel, Mode, RegionStorage, Shared,
};

/// `limit_flags` bit 5: the cap or the speed ceiling is unset.
const FLAG_UNSET: u8 = 1 << 5;
const GOAL_64: i16 = 20971;
const MID: u16 = 2050;

/// Boot as the chip does with both images erased: defaults, board data,
/// the measured shunt bias, the virgin data state.
fn virgin() -> Shared {
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
    sh.table
        .with_mut(|t| t.control.lifecycle.mode = Mode::OpenLoop);
    sh
}

/// Per tick: the plant steps under the kernel's last write, the kernel
/// ticks on the frame. Returns (applied duty, winding current) per tick.
fn run(k: &mut Kernel<FakeIo>, sh: &Shared, p: &mut RlPlant, ticks: u32) -> Vec<(i16, i32)> {
    (0..ticks)
        .map(|_| {
            let duty = duty_of(k);
            k.on_scan(p.step(duty), sh);
            (duty_of(k), p.current())
        })
        .collect()
}

fn drive(sh: &Shared, goal: i16) {
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_duty = goal;
        t.control.lifecycle.torque_enable = true;
    });
}

fn faults(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.common.fault_flags)
}

fn limit_flags(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.limits.limit_flags)
}

#[test]
fn virgin_openloop_run_never_collision_trips() {
    let sh = virgin();
    assert_eq!(
        sh.table.with(|t| t.config.limits.stall_tau_trip_counts),
        STALL_TAU_TRIP_OFF
    );
    let mut k = kernel();
    let mut p = RlPlant::new(600);
    run(&mut k, &sh, &mut p, 400);
    for goal in [GOAL_64, -GOAL_64, 9830, 0] {
        let from = p.pos();
        drive(&sh, goal);
        let r = run(&mut k, &sh, &mut p, 3_000);
        assert_eq!(faults(&sh), 0, "goal {goal}");
        if goal != 0 {
            assert!(r.iter().any(|&(d, _)| d == goal), "goal {goal}: reached");
            assert!((p.pos() - from).abs() > 100, "goal {goal}: moved");
        }
        let tau_d = sh.table.with(|t| t.telemetry.estimates.tau_d_counts);
        assert_eq!(tau_d, 0, "goal {goal}");
    }
}

/// One thermal limit, 90.00 C on the winding estimate.
#[test]
fn virgin_servo_cuts_off_at_90_c() {
    let sh = virgin();
    assert_eq!(sh.table.with(|t| t.config.thermal.cutoff_cc), 9000);
}

/// No cap is compiled in: a virgin servo's OpenLoop limiter runs under the
/// board's overcurrent trip, so a locked rotor takes the goal's duty in
/// full and draws what it drives, under the trip, with no fault inside
/// `stall_time_ticks`; `limit_flags` says a limit is unset.
#[test]
fn virgin_servo_stalls_under_the_board_trip() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = virgin();
        let (cap, trip) = sh.table.with(|t| {
            let l = &t.config.limits;
            (l.current_limit_counts, l.oc_trip_counts as i32)
        });
        assert_eq!(cap, 0, "unset");
        let mut k = kernel();
        let mut p = RlPlant::new(MID);
        p.locked = true;
        run(&mut k, &sh, &mut p, 400);
        drive(&sh, goal);
        // inside stall_time_ticks: the boot stall response is not under test
        let r = run(&mut k, &sh, &mut p, 8_000);
        let peak = r.iter().map(|&(_, i)| i.abs()).max().unwrap_or(0);
        assert!(r.iter().any(|&(d, _)| d == goal), "goal {goal}: applied");
        assert!(peak <= trip, "goal {goal}: peak {peak}");
        let (i_lim, flags) = sh.table.with(|t| {
            (
                t.telemetry.estimates.i_lim_counts as i32,
                t.telemetry.limits.limit_flags,
            )
        });
        assert_eq!(i_lim, trip, "goal {goal}: the band is the trip");
        assert_ne!(flags & FLAG_UNSET, 0, "goal {goal}");
        assert_eq!(faults(&sh), 0, "goal {goal}");
    }
}

/// The factory block as a host meets it: `limit_flags` names the unset
/// limits with torque off, every closed mode is refused with the data
/// fault, and the cap alone does not lift it. Once calibration has written
/// both limits the flag clears and Current mode, which the virgin data
/// state leaves open, is admitted; Velocity stays refused on the virgin
/// data until a set is identified and saved.
#[test]
fn virgin_servo_admits_current_mode_once_both_limits_are_written() {
    let sh = virgin();
    let mut k = kernel();
    let mut p = RlPlant::new(MID);
    run(&mut k, &sh, &mut p, 400);
    assert_ne!(limit_flags(&sh) & FLAG_UNSET, 0, "torque off");
    let enable = |k: &mut Kernel<FakeIo>, p: &mut RlPlant, mode: Mode| {
        sh.table.with_mut(|t| {
            t.control.lifecycle.mode = mode;
            t.control.lifecycle.torque_enable = true;
        });
        run(k, &sh, p, 400);
        let code = sh.table.with(|t| t.telemetry.mode.fault_code);
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        run(k, &sh, p, 40);
        code
    };
    for mode in [Mode::Current, Mode::Velocity, Mode::Position] {
        assert_eq!(enable(&mut k, &mut p, mode), CODE_DATA, "{mode:?}");
    }
    let commit = |f: &dyn Fn(&mut osc_servo_core::ControlTable)| {
        sh.config_commit_begin();
        sh.table.with_mut(f);
        sh.config_commit_end();
    };
    let cap = current_counts(500, SENSE.shunt_r_mohm, SENSE.gain_milli, SENSE.vdd_mv);
    commit(&|t| t.config.limits.current_limit_counts = cap);
    run(&mut k, &sh, &mut p, 40);
    assert_ne!(limit_flags(&sh) & FLAG_UNSET, 0, "the cap alone");
    assert_eq!(
        enable(&mut k, &mut p, Mode::Current),
        CODE_DATA,
        "the cap alone"
    );
    commit(&|t| t.config.loop_position.speed_ceiling_cps = 1500);
    run(&mut k, &sh, &mut p, 40);
    assert_eq!(limit_flags(&sh) & FLAG_UNSET, 0, "both written");
    assert_eq!(enable(&mut k, &mut p, Mode::Current), CODE_NONE, "Current");
    assert_eq!(
        enable(&mut k, &mut p, Mode::Velocity),
        CODE_DATA,
        "Velocity"
    );
}
