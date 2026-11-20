//! A Position hold under a load over the continuous rating on the servo of
//! record's rig (`mg90_d`): a spring on the output whose pull at the goal
//! sits between `i_cont_counts` and the current cap, and falls as the
//! shaft gives way toward its rest point, as a gravity arm's pull falls as
//! it swings toward hanging. The cap holds the goal until the press verdict
//! folds the limit to `i_cont_counts`, `stall_time_ticks` on. Under Yield the
//! shaft then gives way to where the spring meets the fold, past
//! `pos_error_counts`, and the position error timer does not run while the
//! fold is latched (protocol sec 5.7, sec 5.8).

use osc_integration::mg90::{self, Mg90};
use osc_integration::mg90_d;
use osc_integration::plant::FakeIo;
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::kernel::phases::loops::faults::{CODE_NONE, CODE_POSITION_ERROR, CODE_STALL};
use osc_servo_core::kernel::phases::loops::limits::flag;
use osc_servo_core::{ControlTable, Kernel, MotorCmd, RegionStorage, Shared, StallResponse};

const MS: u32 = 20;
/// The spring's rate, winding current counts per count of travel.
const RATE: f64 = 0.5;
/// Its pull at the goal, counts of winding current: about 0.40 A, between
/// the set's 284-count continuous rating and its 568-count cap.
const PULL: f64 = 450.0;
/// The pull at the goal once the load lightens: under the rating less
/// the drag, so the fold holds the goal.
const LIGHT: f64 = 200.0;
const SEEDS: u64 = 3;

struct Hold {
    sh: Shared,
    k: Kernel<FakeIo>,
    p: Mg90,
    goal: i32,
    /// The way the spring pulls, +1 toward higher counts.
    dir: f64,
    pull: f64,
}

impl Hold {
    /// At the soft range's middle, the goal there, `cfg` over the set of
    /// record, the spring on, then torque on.
    fn new(seed: u64, dir: f64, cfg: impl FnOnce(&mut ControlTable)) -> Self {
        let sh = mg90_d::rig(cfg);
        let goal = sh.table.with(|t| {
            let s = &t.config.pos_soft;
            s.pos_min_soft_counts + (s.pos_max_soft_counts - s.pos_min_soft_counts) / 2
        });
        sh.table
            .with_mut(|t| t.control.lifecycle.goal_position = goal);
        let mut h = Self {
            sh,
            k: mg90::kernel(),
            p: mg90_d::plant(goal as f64, seed),
            goal,
            dir,
            pull: PULL,
        };
        h.run(20 * MS);
        h.sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        h
    }

    fn tick(&mut self) {
        self.p.load = self.dir * self.pull - RATE * (self.p.out - self.goal as f64);
        let f = self.p.step(self.k.io.motor.last);
        self.k.on_scan(f, &self.sh);
    }

    fn run(&mut self, ticks: u32) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    /// Ticks until `done`, at most `ticks`.
    fn until(&mut self, ticks: u32, mut done: impl FnMut(&Self) -> bool) -> Option<u32> {
        (0..ticks).find(|_| {
            self.tick();
            done(self)
        })
    }

    fn code(&self) -> u8 {
        self.sh.table.with(|t| t.telemetry.mode.fault_code)
    }

    fn yielded(&self) -> bool {
        self.sh
            .table
            .with(|t| t.telemetry.limits.limit_flags & flag::YIELD != 0)
    }

    fn i_lim(&self) -> u16 {
        self.sh.table.with(|t| t.telemetry.estimates.i_lim_counts)
    }

    /// The pot's error from the goal as the kernel reads it, counts.
    fn err(&self) -> f64 {
        self.p.sensed() - self.goal as f64
    }

    /// The observer's lag behind the goal, counts.
    fn lag(&self) -> f64 {
        let theta = self
            .sh
            .table
            .with(|t| t.telemetry.estimates.theta_hat_q16 as f64 / 65536.0);
        (self.goal as f64 - theta).abs()
    }

    fn driving(&self) -> bool {
        matches!(self.k.io.motor.last, Some(MotorCmd::Drive { .. }))
    }
}

fn cfg<T>(sh: &Shared, f: impl FnOnce(&ControlTable) -> T) -> T {
    sh.table.with(f)
}

/// Under Yield, each way and over three pot-noise seeds: the cap holds the
/// goal until the fold, which lands with no fault. On the fold the shaft
/// gives way past `pos_error_counts` and rests there, torque on at
/// `i_cont_counts`, YIELD set and no fault for at least ten
/// `pos_error_time_ticks`. Once the load lightens under the rating the
/// shaft climbs back and rests within the zone of its goal, twice the
/// deadband, with no fault.
#[test]
fn a_yielded_hold_sags_past_the_error_with_torque_on_and_climbs_back() {
    for seed in 1..=SEEDS {
        for dir in [1.0, -1.0] {
            let case = format!("seed {seed} dir {dir}");
            let mut h = Hold::new(seed, dir, |_| {});
            let (stall_ticks, i_cont, error_counts, error_ticks, deadband) = cfg(&h.sh, |t| {
                (
                    t.config.limits.stall_time_ticks,
                    t.config.limits.i_cont_counts,
                    t.config.fault_cfg.pos_error_counts as f64,
                    t.config.fault_cfg.pos_error_time_ticks,
                    t.config.loop_position.pos_deadband_counts as f64,
                )
            });
            let fold = h
                .until(stall_ticks + 500 * MS, |h| {
                    h.yielded() || h.code() != CODE_NONE
                })
                .unwrap_or_else(|| panic!("{case}: the press never folded"));
            assert_eq!(h.code(), CODE_NONE, "{case}: at the fold");
            assert!(fold >= stall_ticks, "{case}: folded at {fold}");
            let (mut over, mut sag) = (0, 0.0f64);
            for _ in 0..3000 * MS {
                h.tick();
                let at = format!("{case}: err {:.0}", h.err());
                assert_eq!(h.code(), CODE_NONE, "{at}");
                assert!(h.yielded(), "{at}: the fold released");
                assert_eq!(h.i_lim(), i_cont, "{at}");
                assert!(h.driving(), "{at}: {:?}", h.k.io.motor.last);
                over += (h.err().abs() > error_counts) as u32;
                sag = sag.max(dir * h.err());
            }
            assert!(
                over >= 10 * error_ticks,
                "{case}: {over} ticks past the error, sag {sag:.0}"
            );
            h.pull = LIGHT;
            h.run(1000 * MS);
            let mut errs = Vec::new();
            for _ in 0..500 * MS {
                h.tick();
                assert_eq!(h.code(), CODE_NONE, "{case}: climbing back");
                errs.push(h.err());
            }
            errs.sort_by(f64::total_cmp);
            let rest = errs[errs.len() / 2];
            assert!(
                rest.abs() <= 2.0 * deadband,
                "{case}: rests {rest:.1} off the goal"
            );
        }
    }
}

/// The same hold under Fault: `stall` latches no sooner than
/// `stall_time_ticks` after the enable, the limit never folds, and the
/// drive goes off.
#[test]
fn a_hold_over_the_rating_latches_stall_under_fault() {
    for seed in 1..=SEEDS {
        for dir in [1.0, -1.0] {
            let case = format!("seed {seed} dir {dir}");
            let mut h = Hold::new(seed, dir, |t| {
                t.config.limits.stall_response = StallResponse::Fault;
            });
            let stall_ticks = cfg(&h.sh, |t| t.config.limits.stall_time_ticks);
            let latch = h
                .until(stall_ticks + 500 * MS, |h| {
                    assert!(!h.yielded(), "folded under Fault");
                    h.code() != CODE_NONE
                })
                .unwrap_or_else(|| panic!("{case}: never latched"));
            assert_eq!(h.code(), CODE_STALL, "{case}");
            assert!(latch >= stall_ticks, "{case}: latched at {latch}");
            h.run(MS);
            assert!(!h.driving(), "{case}: driving after the latch");
        }
    }
}

/// The same load with `goal_current` under `i_cont_counts`: the hold gives
/// way as under a fold, but the press never sees a current over the rating,
/// so nothing yields, and `position_error` latches `pos_error_time_ticks`
/// after the observer's lag first passed `pos_error_counts`, to two medium
/// periods (the timer counts its first sample as a whole period, and the
/// lag is read as published, once a period). The drive goes off.
#[test]
fn a_hold_capped_under_the_rating_latches_position_error() {
    for seed in 1..=SEEDS {
        for dir in [1.0, -1.0] {
            let case = format!("seed {seed} dir {dir}");
            let mut h = Hold::new(seed, dir, |t| {
                t.control.lifecycle.goal_current = LIGHT as i16;
            });
            let (error_counts, error_ticks) = cfg(&h.sh, |t| {
                let f = &t.config.fault_cfg;
                (f.pos_error_counts as f64, f.pos_error_time_ticks)
            });
            let mut over = None;
            let latch = (0..3000 * MS)
                .find(|&n| {
                    h.tick();
                    assert!(!h.yielded(), "{case}: folded under the rating");
                    if h.lag() > error_counts {
                        over.get_or_insert(n);
                    }
                    h.code() != CODE_NONE
                })
                .unwrap_or_else(|| panic!("{case}: never latched"));
            assert_eq!(h.code(), CODE_POSITION_ERROR, "{case}");
            let due = over.unwrap_or_else(|| panic!("{case}: never over")) + error_ticks;
            assert!(
                latch.abs_diff(due) <= 2 * DECIM_MED as u32,
                "{case}: latched at {latch}, due at {due}"
            );
            h.run(MS);
            assert!(!h.driving(), "{case}: driving after the latch");
        }
    }
}
