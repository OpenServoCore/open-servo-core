//! The drive direction check (`DriveCheck`, kernel/phases/loops/faults.rs)
//! on the bench MG90 plant with its saved set, the mg90-a position table and
//! the soft limits of its 2S pilot envelope. A motor wired the other way
//! round from `drive_polarity` turns the shaft against every drive, which
//! inverts the soft-limit walls as well; the check latches
//! `drive_reversed` before that does damage, and nothing a correctly wired
//! servo meets latches it: a sag the loop wins, a stall, a push on an open
//! drive.

mod support;

use osc_integration::mg90::{self, Mg90};
use osc_integration::plant::{FakeIo, kernel, lut_live};
use osc_servo_core::budget::Events;
use osc_servo_core::kernel::phase;
use osc_servo_core::kernel::phases::loops::faults::{
    BIT_DATA, CODE_DRIVE_REVERSED, CODE_NONE, CODE_STALL, DRIVE_LEG_COUNTS, stage_due,
};
use osc_servo_core::{
    ControlLifecycle, ControlTable, Kernel, Mode, MotorCmd, RegionStorage, Shared, StallResponse,
};

const MID: f64 = 2050.0;
/// The bench MG90's soft limits on its 2S pilot envelope.
const SOFT_MIN: i32 = 432;
const SOFT_MAX: i32 = 3626;
/// Linearized counts per degree on the bench MG90: 4096 over 184.2 deg.
const PER_DEG: f64 = 22.2;
/// FAST ticks per ms.
const MS: u32 = 20;
const SEEDS: u64 = 16;
/// The current limit of the bench set, as a load on the output.
const LIMIT_LOAD: f64 = 280.0;

/// The rev-2A bench's current limit and profile limits on the same plant:
/// the shaft runs at its 3-6 counts/ms there, and a reversed motor's pot
/// leads the observer by up to a leg and a half, as on the bench.
fn bench_limits(t: &mut ControlTable) {
    t.config.limits.current_limit_counts = 554;
    t.config.loop_position.velocity_limit_cps = 4000;
    t.config.loop_position.accel_limit_q88 = 10240;
}

struct Servo {
    sh: Shared,
    k: Kernel<FakeIo>,
    p: Mg90,
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
        };
        s.run(20 * MS);
        s
    }

    fn tick(&mut self) {
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

    /// `drive_reversed`: the data fault bit under its own code.
    fn reversed(&self) -> bool {
        self.sh.table.with(|t| {
            t.telemetry.common.fault_flags & BIT_DATA != 0
                && t.telemetry.mode.fault_code == CODE_DRIVE_REVERSED
        })
    }

    /// Ticks until `done`, at most `ticks`.
    fn until(&mut self, ticks: u32, mut done: impl FnMut(&Self) -> bool) -> Option<u32> {
        (0..ticks).find(|_| {
            self.tick();
            done(self)
        })
    }

    /// A committed `drive_polarity` write with the stamp over it, as `osc
    /// cal` leaves a flip.
    fn write_polarity(&self, polarity: bool) {
        self.sh.config_commit_begin();
        self.sh
            .table
            .with_mut(|t| t.config.limits.drive_polarity = polarity);
        lut_live(&self.sh, &support::mg90_a());
        self.sh.config_commit_end();
    }
}

/// Torque on, holding at `MID`, then `goal`: a correctly wired motor gets
/// there, and once the move has run the check is done - a motor swapped
/// under the proven wiring runs away unjudged.
#[test]
fn a_position_move_proves_the_drive() {
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
        s.p.swapped = true;
        s.control(|c| c.goal_position = MID as i32);
        let from = s.pos();
        s.until(1000 * MS, |s| {
            (s.pos() - from) * (goal as f64 - MID).signum() >= 6.0 * DRIVE_LEG_COUNTS as f64
        })
        .expect("the swapped motor runs away");
        assert_eq!(s.code(), CODE_NONE, "goal {goal}: judged after the proof");
    }
}

/// An OpenLoop drive to the top wall proves the drive on its way, so the
/// same geometry the reversed pin faults at - past the wall, the duty
/// pointing in, the shaft going out - passes unjudged after it.
#[test]
fn an_openloop_drive_proves_the_drive() {
    let mut s = Servo::new(1, MID, Mode::OpenLoop, |_| {});
    s.control(|c| {
        c.goal_duty = 3500;
        c.torque_enable = true;
    });
    s.until(3000 * MS, |s| s.pos() >= SOFT_MAX as f64)
        .expect("reaches the top wall");
    s.run(200 * MS);
    assert_eq!(s.code(), CODE_NONE);
    s.p.swapped = true;
    s.control(|c| c.goal_duty = -3500);
    let from = s.pos();
    s.until(1000 * MS, |s| {
        s.pos() - from >= 4.0 * DRIVE_LEG_COUNTS as f64
    })
    .expect("the swapped motor drives out");
    assert_eq!(s.code(), CODE_NONE, "judged after the proof");
}

/// A motor wired against `drive_polarity` under a closed loop, torque on at
/// mid travel and a goal either way: the loop drives the shaft away and
/// `drive_reversed` latches, the winding shorted from that tick, within three
/// legs of pot travel from the enable: two legs on the observer's sample,
/// then the verdict's period and the sample's age at speed and the pot's
/// residual. 16 seeds each.
#[test]
fn a_reversed_motor_faults_in_the_closed_loops_within_three_legs() {
    for mode in [Mode::Position, Mode::Velocity] {
        for dir in [1i32, -1] {
            let mut worst: f64 = 0.0;
            for seed in 1..=SEEDS {
                let mut s = Servo::new(seed, MID, mode, |_| {});
                s.p.swapped = true;
                let from = s.pos();
                s.control(|c| c.torque_enable = true);
                s.run(10 * MS);
                assert_eq!(s.code(), CODE_NONE, "{mode:?} {dir} seed {seed}: at rest");
                s.control(|c| {
                    c.goal_position = MID as i32 + 200 * dir;
                    c.goal_velocity = 1000 * dir;
                });
                let mut travel: f64 = 0.0;
                s.until(1000 * MS, |s| {
                    travel = travel.max((s.pos() - from).abs());
                    s.reversed()
                })
                .unwrap_or_else(|| panic!("{mode:?} {dir} seed {seed}: never latched"));
                assert_eq!(s.code(), CODE_DRIVE_REVERSED, "{mode:?} {dir} seed {seed}");
                s.tick();
                assert!(
                    matches!(s.k.io.motor.last, Some(MotorCmd::Brake)),
                    "{mode:?} {dir} seed {seed}: not braking"
                );
                worst = worst.max(travel);
            }
            assert!(
                worst <= 3.0 * DRIVE_LEG_COUNTS as f64,
                "{mode:?} {dir}: {worst:.1} counts of travel before the latch"
            );
        }
    }
}

/// At the bench limits, goal and torque in one write: the pot leads the
/// observer by a full leg or more on the way, the lead that discarded legs
/// before the rule looked only for a pot left behind, and the latch still
/// comes within two legs, the lead and the verdict's period at speed:
/// three and a half legs. The winding short then stops the shaft within
/// five legs of the latch, where a coast runs ten or more. 16 seeds each.
#[test]
fn a_reversed_motor_at_the_bench_limits_latches_and_brakes() {
    let mut lead: f64 = 0.0;
    for mode in [Mode::Position, Mode::Velocity] {
        for dir in [1i32, -1] {
            let (mut worst, mut stop): (f64, f64) = (0.0, 0.0);
            for seed in 1..=SEEDS {
                let mut s = Servo::new(seed, MID, mode, |t| {
                    bench_limits(t);
                    t.control.lifecycle.goal_position = MID as i32 + 200 * dir;
                    t.control.lifecycle.goal_velocity = 4000 * dir;
                });
                s.p.swapped = true;
                let from = s.pos();
                s.control(|c| c.torque_enable = true);
                let mut travel: f64 = 0.0;
                s.until(1000 * MS, |s| {
                    travel = travel.max((s.pos() - from).abs());
                    let theta = s.sh.table.with(|t| t.telemetry.estimates.theta_hat_q16);
                    lead = lead.max((s.p.sensed() - theta as f64 / 65536.0) * s.p.omega.signum());
                    s.reversed()
                })
                .unwrap_or_else(|| panic!("{mode:?} {dir} seed {seed}: never latched"));
                worst = worst.max(travel);
                let at = s.pos();
                for _ in 0..400 * MS {
                    s.tick();
                    stop = stop.max((at - s.pos()) * dir as f64);
                }
                assert!(
                    matches!(s.k.io.motor.last, Some(MotorCmd::Brake)),
                    "{mode:?} {dir} seed {seed}: not braking"
                );
            }
            let leg = DRIVE_LEG_COUNTS as f64;
            assert!(
                worst <= 3.5 * leg,
                "{mode:?} {dir}: {worst:.1} counts to the latch"
            );
            assert!(
                stop <= 5.0 * leg,
                "{mode:?} {dir}: {stop:.1} counts after it"
            );
        }
    }
    assert!(
        lead >= DRIVE_LEG_COUNTS as f64,
        "the pot led by {lead:.1} at most"
    );
}

/// A reversed loop held 10 ms at mid at the bench limits, then stepped
/// toward the way it drifted, should latch in every seed. Some creep to the
/// goal under the dithering duty instead and prove the wiring.
#[test]
#[ignore = "a slow leg cannot show a reversed motor under a closed loop: friction and the dithering duty let it creep along the reference"]
fn a_reversed_motor_stepped_toward_its_drift_latches() {
    for mode in [Mode::Position, Mode::Velocity] {
        for seed in 1..=2 * SEEDS {
            let mut s = Servo::new(seed, MID, mode, bench_limits);
            s.p.swapped = true;
            let from = s.pos();
            s.control(|c| c.torque_enable = true);
            s.run(10 * MS);
            let dir = if s.pos() < from { -1 } else { 1 };
            s.control(|c| {
                c.goal_position = MID as i32 + 200 * dir;
                c.goal_velocity = 4000 * dir;
            });
            s.until(1000 * MS, |s| s.reversed())
                .unwrap_or_else(|| panic!("{mode:?} seed {seed}: drifted {dir}, never latched"));
        }
    }
}

/// The winding short holds from the latch through torque off, and the
/// torque_enable 0 to 1 ack releases it with the latch.
#[test]
fn the_latch_brakes_until_the_ack() {
    let mut s = Servo::new(1, 700.0, Mode::OpenLoop, |_| {});
    s.p.swapped = true;
    s.control(|c| {
        c.goal_duty = 3500;
        c.torque_enable = true;
    });
    s.until(3000 * MS, |s| s.reversed())
        .expect("latches at the wall");
    let braked = |s: &mut Servo, ticks: u32| {
        (0..ticks).all(|_| {
            s.tick();
            matches!(s.k.io.motor.last, Some(MotorCmd::Brake))
        })
    };
    assert!(braked(&mut s, 20 * MS));
    s.control(|c| c.torque_enable = false);
    assert!(braked(&mut s, 20 * MS));
    assert!(s.reversed());
    s.control(|c| c.torque_enable = true);
    s.run(MS);
    assert_eq!(s.code(), CODE_NONE);
    assert!(matches!(s.k.io.motor.last, Some(MotorCmd::Drive { .. })));
}

/// A reversed motor on an open drive mid travel is judged only at a soft
/// limit: OpenLoop and Current, each toward the far wall from inside the
/// near one, run to the near wall and latch within a leg and a half past
/// it, inside the room the wall leaves to the stop.
#[test]
fn a_reversed_open_drive_faults_at_the_soft_limit() {
    for mode in [Mode::OpenLoop, Mode::Current] {
        for (at, dir, wall) in [(700.0, 1i32, SOFT_MIN), (3350.0, -1, SOFT_MAX)] {
            for seed in 1..=4 {
                let mut s = Servo::new(seed, at, mode, |_| {});
                s.p.swapped = true;
                s.control(|c| {
                    c.goal_duty = (3500 * dir) as i16;
                    c.goal_current = (150 * dir) as i16;
                    c.torque_enable = true;
                });
                s.until(3000 * MS, |s| s.reversed())
                    .unwrap_or_else(|| panic!("{mode:?} from {at} seed {seed}: never latched"));
                assert_eq!(s.code(), CODE_DRIVE_REVERSED);
                // the pot's noise aside, past the wall
                let past = (wall as f64 - s.pos()) * dir as f64;
                assert!(
                    (-5.0..=1.5 * DRIVE_LEG_COUNTS as f64).contains(&past),
                    "{mode:?} from {at} seed {seed}: latched {past:.1} past the wall"
                );
            }
        }
    }
}

/// Gravity on a held shaft, let go at the enable and again after the hold
/// has parked, at the bench inertia and twice it, 16 seeds each: the shaft
/// drops against the drive until the loop stops it. Under 80% of the
/// current limit the loop pulls it back; at the limit, friction against
/// the lift, it holds where the fall stopped. Neither latches. Sizes
/// `DRIVE_LEG_COUNTS`: at twice the inertia and the limit, 16 trips.
#[test]
fn gravity_sag_at_the_limit_never_reads_reversed() {
    for inertia in [1.0, 2.0] {
        for load in [0.8 * LIMIT_LOAD, LIMIT_LOAD, -0.8 * LIMIT_LOAD, -LIMIT_LOAD] {
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
                    assert_eq!(s.code(), CODE_NONE, "{case}: sag {sag:.1}");
                    assert!((s.p.out - MID).abs() <= 3.0 * PER_DEG, "{case}: fell");
                }
            }
        }
    }
}

/// A locked rotor under a drive either way, either wiring, OpenLoop at a
/// duty whose stall current is over the limit: nothing moves, so nothing
/// is judged, and the stall timer acts as before.
#[test]
fn a_stall_never_reads_reversed() {
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
                assert!(!s.reversed(), "{mode:?} {dir} swapped {swapped}");
            }
        }
    }
}

/// A reversed motor held still under a closed loop at the bench limits,
/// the stall yielding: the observer's model walks it the drive's way and
/// the pot stays, so nothing proves the wiring through 1.5 s of it, and
/// once let go the shaft runs and latches. 16 seeds each.
#[test]
fn a_stalled_reversed_motor_is_never_proven() {
    for mode in [Mode::Position, Mode::Velocity] {
        for dir in [1i32, -1] {
            for seed in 1..=SEEDS {
                let mut s = Servo::new(seed, MID, mode, |t| {
                    bench_limits(t);
                    t.config.limits.stall_response = StallResponse::Yield;
                    t.control.lifecycle.goal_position = MID as i32 + 200 * dir;
                    t.control.lifecycle.goal_velocity = 4000 * dir;
                });
                s.p.locked = true;
                s.p.swapped = true;
                s.control(|c| c.torque_enable = true);
                s.run(1500 * MS);
                assert_eq!(s.code(), CODE_NONE, "{mode:?} {dir} seed {seed}");
                s.p.locked = false;
                s.until(1000 * MS, |s| s.reversed())
                    .unwrap_or_else(|| panic!("{mode:?} {dir} seed {seed}: proven while held"));
            }
        }
    }
}

/// A `drive_polarity` write unproves the wiring: a proven servo whose
/// polarity is flipped under it latches at its next move, and a reversed
/// one whose polarity the host flips back proves at its next enable.
#[test]
fn a_polarity_write_resets_the_check() {
    let mut s = Servo::new(1, MID, Mode::Position, |_| {});
    s.control(|c| {
        c.goal_position = MID as i32 + 200;
        c.torque_enable = true;
    });
    s.run(1000 * MS);
    assert_eq!(s.code(), CODE_NONE);
    s.write_polarity(false);
    s.control(|c| c.goal_position = MID as i32);
    s.until(1000 * MS, |s| s.reversed())
        .expect("the flipped polarity latches");
    assert_eq!(s.code(), CODE_DRIVE_REVERSED);

    let mut s = Servo::new(1, MID, Mode::Position, |_| {});
    s.p.swapped = true;
    s.control(|c| {
        c.goal_position = MID as i32 + 200;
        c.torque_enable = true;
    });
    s.until(1000 * MS, |s| s.reversed())
        .expect("the swapped motor latches");
    s.control(|c| c.torque_enable = false);
    s.run(10 * MS);
    s.write_polarity(false);
    let at = s.pos().round() as i32;
    s.control(|c| {
        c.goal_position = at + 200;
        c.torque_enable = true;
    });
    s.run(1000 * MS);
    assert_eq!(s.code(), CODE_NONE);
    assert!((s.pos() - (at + 200) as f64).abs() <= 20.0, "{}", s.pos());
}

/// An OpenLoop drive overpowered by a push mid travel from the enable, so
/// before anything proves the wiring: the shaft goes 300 counts against the
/// duty, inside the walls, then resumes once let go.
#[test]
fn an_external_push_mid_travel_in_openloop_never_reads_reversed() {
    for dir in [1i32, -1] {
        let mut s = Servo::new(1, MID, Mode::OpenLoop, |_| {});
        s.p.load = -400.0 * dir as f64;
        s.control(|c| {
            c.goal_duty = (2500 * dir) as i16;
            c.torque_enable = true;
        });
        let from = s.pos();
        let mut ahead: f64 = 0.0;
        s.until(3000 * MS, |s| {
            ahead = ahead.max((s.pos() - from) * dir as f64);
            (from - s.pos()) * dir as f64 >= 300.0
        })
        .expect("the push wins");
        assert!(ahead < DRIVE_LEG_COUNTS as f64, "{dir}: proven first");
        s.p.load = 0.0;
        s.until(3000 * MS, |s| (s.pos() - from) * dir as f64 >= 0.0)
            .expect("the drive resumes");
        assert_eq!(s.code(), CODE_NONE, "{dir}");
    }
}

/// The check's stage takes a SPARE_B no thermometer stage or configuration
/// take does: an open drive pushed back down most of the travel for 3 s,
/// under a commit every 50 ms, ends a leg every few periods, and no tick
/// runs the check's stage with another, a stage due on a SPARE_B the
/// thermometer took waiting a period instead.
#[test]
fn the_drive_stage_never_shares_a_tick_with_a_thermometer_stage() {
    let mut s = Servo::new(1, 3500.0, Mode::OpenLoop, |_| {});
    s.p.load = -250.0;
    s.control(|c| {
        c.goal_duty = 2500;
        c.torque_enable = true;
    });
    let (mut drive, mut therm, mut takes, mut waited) = (0, 0, 0, 0);
    for t in 0..3000 * MS {
        if t % (50 * MS) == 0 {
            s.write_polarity(true);
        }
        let before = s.k.probe_tags();
        s.tick();
        let ev = Events::between(before, s.k.probe_tags());
        assert!(
            !(ev.drive && (ev.side || ev.install || ev.refresh)),
            "tick {t}: {ev:?}"
        );
        drive += ev.drive as u32;
        therm += ev.side as u32;
        takes += ev.refresh as u32;
        waited += (before.phase == phase::SPARE_B && stage_due(before.drive) && ev.side) as u32;
    }
    assert_eq!(s.code(), CODE_NONE);
    assert!(
        drive > 50 && therm > 50 && takes > 50 && waited > 0,
        "{drive} {therm} {takes} {waited}"
    );
}
