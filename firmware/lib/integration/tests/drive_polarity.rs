//! A motor wired the other way round from `drive_polarity`, and a load on
//! a hold, on the bench MG90 plant with its saved set, the mg90-a position
//! table and the soft limits of its 2S pilot envelope. The kernel does not
//! read the wiring at run time: `drive_polarity` is a calibration fact
//! under the plant stamp (protocol sec 5.7). A wrong one inverts every
//! closed loop and the soft-limit walls with it, and what ends the run is
//! what ends any losing drive, the position error timer and the press. A
//! load the limit cannot hold is the press's case too.

mod support;

use osc_integration::mg90::{self, Mg90};
use osc_integration::plant::{FakeIo, kernel, lut_live};
use osc_servo_core::kernel::phases::loops::faults::{
    CODE_DATA, CODE_NONE, CODE_POSITION_ERROR, CODE_STALL,
};
use osc_servo_core::kernel::phases::loops::limits::flag;
use osc_servo_core::regions::config::addr::limits::DRIVE_POLARITY;
use osc_servo_core::{
    ControlLifecycle, ControlTable, Kernel, Mode, MotorCmd, RegionStorage, Shared, StallResponse,
};

const MID: f64 = 2050.0;
/// The bench MG90's soft limits on its 2S pilot envelope.
const SOFT_MIN: i32 = 432;
const SOFT_MAX: i32 = 3626;
/// Its hard stops.
const STOPS: (f64, f64) = (support::MG90_A_MIN as f64, support::MG90_A_MAX as f64);
/// Linearized counts per degree on the bench MG90: 4096 over 184.2 deg.
const PER_DEG: f64 = 22.2;
/// FAST ticks per ms.
const MS: u32 = 20;
const SEEDS: u64 = 16;
/// The current limit of the bench set, as a load on the output.
const LIMIT_LOAD: f64 = 280.0;
/// The rev-2A bench's current limit.
const BENCH_LIMIT: f64 = 554.0;

/// The rev-2A bench's current limit and profile limits on the same plant:
/// the shaft runs at its 3-6 counts/ms there.
fn bench_limits(t: &mut ControlTable) {
    t.config.limits.current_limit_counts = BENCH_LIMIT as u16;
    t.config.loop_position.velocity_limit_cps = 4000;
    t.config.loop_position.accel_limit_q88 = 10240;
}

/// The board's window floors (osc-dev-v006: the shunt 64 ticks, the
/// terminals 93), where the current loop reads every drive it holds a
/// limit with.
fn board_floors(t: &mut ControlTable) {
    t.calib.sense.i_window_min_ticks = 64;
    t.calib.sense.v_window_min_ticks = 93;
}

struct Servo {
    sh: Shared,
    k: Kernel<FakeIo>,
    p: Mg90,
    /// The hard stops are in: rigid, each stops a shaft that runs into it
    /// and holds it while the torque points into the stop.
    stops: bool,
}

impl Servo {
    /// At `at` in `mode` with `cfg`, torque off for 20 ms.
    fn new(seed: u64, at: f64, mode: Mode, cfg: impl FnOnce(&mut ControlTable)) -> Self {
        let lut = support::mg90_a();
        let sh = mg90::rig(&lut, |t| {
            t.config.pos_soft.pos_min_soft_counts = SOFT_MIN;
            t.config.pos_soft.pos_max_soft_counts = SOFT_MAX;
            t.control.lifecycle.mode = mode;
            t.control.lifecycle.goal_position = at as i32;
            cfg(t);
        });
        let mut s = Self {
            sh,
            k: kernel(),
            p: Mg90::new(at, lut, seed),
            stops: false,
        };
        s.run(20 * MS);
        s
    }

    fn tick(&mut self) {
        if self.stops {
            let (torque, omega) = (self.p.i + self.p.load, self.p.omega);
            self.p.locked = (self.p.out <= STOPS.0 && (omega < 0.0 || torque < 0.0))
                || (self.p.out >= STOPS.1 && (omega > 0.0 || torque > 0.0));
        }
        let f = self.p.step(self.k.io.motor.last);
        self.k.on_scan(f, &self.sh);
    }

    fn run(&mut self, ticks: u32) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    fn control(&self, f: impl FnOnce(&mut ControlLifecycle)) {
        self.sh.table.with_mut(|t| f(&mut t.control.lifecycle));
    }

    /// The pot as the kernel reads it, linearized counts.
    fn pos(&self) -> f64 {
        self.p.sensed()
    }

    fn code(&self) -> u8 {
        self.sh.table.with(|t| t.telemetry.mode.fault_code)
    }

    fn limit_flags(&self) -> u8 {
        self.sh.table.with(|t| t.telemetry.limits.limit_flags)
    }

    /// Ticks until `done`, at most `ticks`.
    fn until(&mut self, ticks: u32, mut done: impl FnMut(&Self) -> bool) -> Option<u32> {
        (0..ticks).find(|_| {
            self.tick();
            done(self)
        })
    }

    /// A committed `drive_polarity` write, as the dispatcher lands one: a
    /// covered write, so the stamp reads stale from here.
    fn write_polarity(&self, polarity: bool) {
        self.sh.config_commit_begin();
        self.sh
            .table
            .with_mut(|t| t.config.limits.drive_polarity = polarity);
        self.sh.config_commit_end();
        self.sh.data_state_after_commit(DRIVE_POLARITY, 1);
    }

    /// The stamp over the set as it stands, as `osc cal` leaves a flip.
    fn stamp(&self) {
        lut_live(&self.sh, &support::mg90_a());
    }
}

/// Torque on, holding at `MID`, then `goal` either way: the shaft goes to
/// it and nothing latches.
#[test]
fn a_position_move_runs_to_its_goal() {
    for goal in [MID as i32 + 200, MID as i32 - 200] {
        let mut s = Servo::new(1, MID, Mode::Position, |_| {});
        s.control(|c| c.torque_enable = true);
        s.run(100 * MS);
        s.control(|c| c.goal_position = goal);
        s.run(1000 * MS);
        assert_eq!(s.code(), CODE_NONE, "goal {goal}");
        assert!(
            (s.pos() - goal as f64).abs() <= 20.0,
            "goal {goal}: {}",
            s.pos()
        );
    }
}

/// A motor wired against `drive_polarity` under a closed loop at the bench
/// limits, on the board's window floors, goal and torque in one write.
/// Nothing reads the wiring, so the loop drives the shaft away from its
/// goal at the profile's speed, through the soft limit, whose wall closes
/// the side that would bring a reversed motor back, and onto the hard stop
/// 82 deg from mid travel, inside half a second. There it presses at the
/// current limit until a timer ends it: in Position the position error
/// timer, started when the run passed `pos_error_counts`, about 0.1 s
/// after the stop; in Velocity the stall timer, `stall_time_ticks` after
/// the observer reads the shaft still, about 0.6 s after it. The latch
/// switches the drive off. The winding current stays under twice the limit
/// through the run and the impact, and on the stop it is the limit within
/// a millisecond of the impact. 16 seeds each.
#[test]
fn a_reversed_motor_runs_to_its_stop_and_a_timer_latches() {
    for (mode, code, latch_ms) in [
        (Mode::Position, CODE_POSITION_ERROR, 0..=150),
        (Mode::Velocity, CODE_STALL, 500..=650),
    ] {
        for dir in [1i32, -1] {
            let stop = if dir > 0 { STOPS.0 } else { STOPS.1 };
            for seed in 1..=SEEDS {
                let case = format!("{mode:?} {dir} seed {seed}");
                let mut s = Servo::new(seed, MID, mode, |t| {
                    bench_limits(t);
                    board_floors(t);
                    t.config.limits.stall_response = StallResponse::Fault;
                    t.control.lifecycle.goal_position = MID as i32 + 200 * dir;
                    t.control.lifecycle.goal_velocity = 4000 * dir;
                });
                s.p.swapped = true;
                s.stops = true;
                s.control(|c| c.torque_enable = true);
                let mut peak: f64 = 0.0;
                let mut at_stop = None;
                let (mut press, mut press_peak, mut pressed) = (0.0, 0.0f64, 0);
                let latch = (0..2000 * MS)
                    .find(|&t| {
                        s.tick();
                        peak = peak.max(s.p.i.abs());
                        if s.p.locked {
                            let at = *at_stop.get_or_insert(t);
                            if t > at + MS {
                                // into the stop: the reversed motor's own way
                                press += s.p.i * -dir as f64;
                                press_peak = press_peak.max(s.p.i.abs());
                                pressed += 1;
                            }
                        }
                        s.code() != CODE_NONE
                    })
                    .unwrap_or_else(|| panic!("{case}: never latched"));
                let at_stop =
                    at_stop.unwrap_or_else(|| panic!("{case}: latched short of the stop"));
                assert_eq!(s.code(), code, "{case}");
                assert!(s.limit_flags() & flag::ENDSTOP != 0, "{case}: no wall");
                assert!((s.p.out - stop).abs() <= 2.0, "{case}: at {:.1}", s.p.out);
                assert!(at_stop <= 500 * MS, "{case}: {at_stop} ticks to the stop");
                let held = (latch - at_stop) / MS;
                assert!(latch_ms.contains(&held), "{case}: {held} ms on the stop");
                assert!(peak <= 2.0 * BENCH_LIMIT, "{case}: peak {peak:.0}");
                assert!(
                    press_peak <= 1.125 * BENCH_LIMIT,
                    "{case}: {press_peak:.0} on the stop"
                );
                let press = press / pressed as f64;
                assert!(
                    (0.98 * BENCH_LIMIT..=1.02 * BENCH_LIMIT).contains(&press),
                    "{case}: pressing at {press:.0}"
                );
                s.run(MS);
                assert!(
                    !matches!(s.k.io.motor.last, Some(MotorCmd::Drive { .. })),
                    "{case}: driving after the latch"
                );
            }
        }
    }
}

/// Gravity on a held shaft at 80% of the current limit, let go at the
/// enable and again after the hold has parked, at the bench inertia and
/// twice it, 16 seeds each: the shaft drops against the drive until the
/// loop stops it and pulls it back. The fall ends inside the hold's zone
/// of 30 counts plus the load's work over it, shed by the limit's margin
/// over the load and friction: 30 + 30 x 224 / (280 - 224 + 53), 92 counts.
/// Half a second on it is back inside 3 deg, and nothing latches.
#[test]
fn a_load_under_the_limit_sags_and_is_pulled_back() {
    for inertia in [1.0, 2.0] {
        for load in [0.8 * LIMIT_LOAD, -0.8 * LIMIT_LOAD] {
            for let_go_ms in [0, 100] {
                for seed in 1..=SEEDS {
                    let mut s = Servo::new(seed, MID, Mode::Position, |_| {});
                    s.p.inertia = inertia;
                    s.p.load = load;
                    s.p.locked = true;
                    s.control(|c| c.torque_enable = true);
                    s.run(let_go_ms * MS + 1);
                    s.p.locked = false;
                    let mut sag: f64 = 0.0;
                    for _ in 0..500 * MS {
                        s.tick();
                        sag = sag.max((s.p.out - MID) * load.signum());
                    }
                    let case = format!("J x{inertia} load {load} at {let_go_ms} ms seed {seed}");
                    assert!(sag > 12.0, "{case}: no sag ({sag:.1})");
                    assert!(sag <= 92.0, "{case}: sag {sag:.1}");
                    assert_eq!(s.code(), CODE_NONE, "{case}: sag {sag:.1}");
                    assert!((s.p.out - MID).abs() <= 3.0 * PER_DEG, "{case}: fell");
                }
            }
        }
    }
}

/// The same let-go at the limit, on the board's window floors, where the
/// loop reads the catch and holds the limit to the count, so friction
/// alone stops the fall. That is the current limit giving way, the press's
/// case: no latch, the shaft at rest inside 7 deg (the load's work over the
/// hold's zone of 30 counts, shed by friction, 30 x 280 / 53), and the hold
/// ends on the press's fold at `stall_time_ticks`, the code still clear.
#[test]
fn a_load_at_the_limit_gives_way_to_the_press() {
    for inertia in [1.0, 2.0] {
        for load in [LIMIT_LOAD, -LIMIT_LOAD] {
            for let_go_ms in [0, 100] {
                for seed in 1..=SEEDS {
                    let mut s = Servo::new(seed, MID, Mode::Position, board_floors);
                    s.p.inertia = inertia;
                    s.p.load = load;
                    s.p.locked = true;
                    s.control(|c| c.torque_enable = true);
                    s.run(let_go_ms * MS + 1);
                    s.p.locked = false;
                    let mut sag: f64 = 0.0;
                    for _ in 0..400 * MS {
                        s.tick();
                        sag = sag.max((s.p.out - MID) * load.signum());
                    }
                    let case = format!("J x{inertia} load {load} at {let_go_ms} ms seed {seed}");
                    assert_eq!(s.code(), CODE_NONE, "{case}: sag {sag:.1}");
                    assert!(s.p.omega == 0.0, "{case}: not at rest");
                    assert!(sag <= 7.0 * PER_DEG, "{case}: sag {sag:.1}");
                    let folded = |s: &Servo| s.limit_flags() & flag::YIELD != 0;
                    s.until(400 * MS, folded)
                        .unwrap_or_else(|| panic!("{case}: the press never folded"));
                    assert_eq!(s.code(), CODE_NONE, "{case}: at the fold");
                }
            }
        }
    }
}

/// A locked rotor under a drive either way, on either wiring, Position and
/// OpenLoop at a duty whose stall current is over the limit: nothing
/// moves, and the stall timer latches whichever way the leads go.
#[test]
fn a_locked_rotor_latches_stall_on_either_wiring() {
    for swapped in [false, true] {
        for mode in [Mode::Position, Mode::OpenLoop] {
            for dir in [1i32, -1] {
                let mut s = Servo::new(1, MID, mode, |t| {
                    t.config.limits.stall_response = StallResponse::Fault;
                });
                s.p.locked = true;
                s.p.swapped = swapped;
                s.control(|c| {
                    c.goal_position = MID as i32 + 200 * dir;
                    c.goal_duty = (8000 * dir) as i16;
                    c.torque_enable = true;
                });
                s.run(1500 * MS);
                assert_eq!(
                    s.code(),
                    CODE_STALL,
                    "{mode:?} {dir} swapped {swapped}: the stall timer"
                );
            }
        }
    }
}

/// A `drive_polarity` write is a covered write: it leaves the stamp stale,
/// and the next enable of a closed loop is refused on the data state, the
/// shaft where it was. With the stamp over the flip, as `osc cal` leaves
/// one, the loop runs under the new polarity and a motor wired the other
/// way round goes to its goal.
#[test]
fn a_polarity_write_refuses_closed_loop_until_the_stamp() {
    let mut s = Servo::new(1, MID, Mode::Position, |_| {});
    s.p.swapped = true;
    s.write_polarity(false);
    let at = s.pos();
    s.control(|c| {
        c.goal_position = MID as i32 + 200;
        c.torque_enable = true;
    });
    s.run(100 * MS);
    assert_eq!(s.code(), CODE_DATA);
    assert!((s.pos() - at).abs() <= 5.0, "{}", s.pos());
    s.control(|c| c.torque_enable = false);
    s.run(10 * MS);
    s.stamp();
    s.control(|c| c.torque_enable = true);
    s.run(1000 * MS);
    assert_eq!(s.code(), CODE_NONE);
    assert!((s.pos() - (MID + 200.0)).abs() <= 20.0, "{}", s.pos());
}

/// An OpenLoop drive overpowered by a push mid travel from the enable: the
/// shaft goes 300 counts against the duty, inside the walls, then resumes
/// once let go, and nothing latches.
#[test]
fn an_openloop_drive_pushed_back_mid_travel_resumes() {
    for dir in [1i32, -1] {
        let mut s = Servo::new(1, MID, Mode::OpenLoop, |_| {});
        s.p.load = -400.0 * dir as f64;
        s.control(|c| {
            c.goal_duty = (2500 * dir) as i16;
            c.torque_enable = true;
        });
        let from = s.pos();
        s.until(3000 * MS, |s| (from - s.pos()) * dir as f64 >= 300.0)
            .expect("the push wins");
        s.p.load = 0.0;
        s.until(3000 * MS, |s| (s.pos() - from) * dir as f64 >= 0.0)
            .expect("the drive resumes");
        assert_eq!(s.code(), CODE_NONE, "{dir}");
    }
}
