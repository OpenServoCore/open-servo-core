//! A virgin servo on the class defaults: board data and the boot seed only,
//! nothing identified, nothing saved. The kernel drives the MG90-scale R-L
//! plant rig on the osc-dev-v006 sense chain (60 mohm, G 15.0).

use osc_integration::plant::{BIAS, FakeIo, RlPlant, duty_of, kernel, last_cmd};
use osc_servo_core::{
    BaudRate, CalibSense, CalibSenseExt, ConfigDefaults, CurrentDefaults, ImageState, Kernel, Mode,
    RegionStorage, Shared,
};

/// The osc-dev-v006 app's sense chain.
const SENSE: CalibSense = CalibSense {
    shunt_r_mohm: 60,
    gain_milli: 15_000,
    vmotor_div_top: 6_800,
    vmotor_div_bot: 3_300,
    vdd_mv: 3_300,
    tick_hz: 20_000,
    i_window_min_ticks: 160,
    v_window_min_ticks: 160,
};
const SENSE_EXT: CalibSenseExt = CalibSenseExt {
    vbus_div_top_ohm: 15_000,
    vbus_div_bot_ohm: 10_000,
    ntc_pullup_ohm: 10_000,
    ntc_r25_ohm: 10_000,
    ntc_beta: 3950,
    vmotor_bias_nom_counts: 779,
    rail_drop_mv: 250,
};
/// 300 mA on this chain.
const CLASS_LIM: i32 = 335;
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
    );
    sh.table.seed_calib_sense(&SENSE, &SENSE_EXT);
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
            let duty = k.io.motor.last.map_or(0, duty_of);
            k.on_tick(p.step(duty), sh);
            (duty_of(last_cmd(k)), p.current())
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

#[test]
fn virgin_openloop_run_never_collision_trips() {
    let sh = virgin();
    assert_eq!(
        sh.table.with(|t| t.config.limits.stall_tau_trip_counts),
        CLASS_LIM as u16
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

#[test]
fn virgin_servo_stalls_at_the_class_limit() {
    for goal in [GOAL_64, -GOAL_64] {
        let sh = virgin();
        assert_eq!(
            sh.table.with(|t| t.config.limits.current_limit_counts),
            CLASS_LIM as u16
        );
        let mut k = kernel();
        let mut p = RlPlant::new(MID);
        p.locked = true;
        run(&mut k, &sh, &mut p, 400);
        drive(&sh, goal);
        // inside stall_time_ms: the boot stall response is not under test
        let r = run(&mut k, &sh, &mut p, 8_000);
        let peak = r.iter().map(|&(_, i)| i.abs()).max().unwrap_or(0);
        let held = &r[2_000..];
        let mean =
            (held.iter().map(|&(_, i)| i.abs() as i64).sum::<i64>() / held.len() as i64) as i32;
        assert!(peak <= CLASS_LIM * 11 / 10, "goal {goal}: peak {peak}");
        assert!(
            (CLASS_LIM * 85 / 100..=CLASS_LIM).contains(&mean),
            "goal {goal}: mean {mean}"
        );
        assert_eq!(faults(&sh), 0, "goal {goal}");
    }
}
