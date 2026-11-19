//! Position holds under a standing load on the servo of record's rig
//! (`mg90_d`): a constant torque on the output, inside the range the
//! bench's gravity arm loads a hold with. A park drops the current that
//! holds the shaft, so the hold parks only a shaft that carries no load
//! past the friction level (`position::unloaded`), and a loaded one rests
//! on the velocity loop's integral, frozen while it is held.

use osc_integration::mg90::{self, Mg90};
use osc_integration::mg90_d::{self, COUNTS_PER_DEG, CURRENT_COUNTS_PER_A};
use osc_servo_core::{MotorCmd, RegionStorage};

/// The load as a winding current, counts: 200 mA, under the set's
/// continuous rating, so the press verdict never folds the hold.
const LOAD: f64 = 0.2 * CURRENT_COUNTS_PER_A;
const TICKS_PER_MS: u32 = 20;
/// Per goal, as the bench's load captures hold each: 1.5 s, the last 1.0
/// s judged for drive and the last 0.5 s for the rest.
const HOLD_MS: u32 = 1500;
const DRIVEN_MS: u32 = 1000;
const REST_MS: u32 = 500;
/// The long hold, as the bench's past-the-rating cells hold each.
const LONG_MS: u32 = 3000;

#[derive(Debug)]
struct Hold {
    /// Over the last `REST_MS`: the median sensed error and the mean
    /// winding current, counts.
    err: f64,
    i_mean: f64,
    /// Ticks braked over the last `DRIVEN_MS`.
    braked: u32,
    faults: u8,
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// The rig at rest at its soft range's middle under `load`, torque on,
/// then a 30-degree step out and back each way, each goal held `hold_ms`.
fn holds(load: f64, seed: u64, hold_ms: u32) -> Vec<(i32, Hold)> {
    let sh = mg90_d::rig(|_| {});
    let center = sh.table.with(|t| {
        let s = &t.config.pos_soft;
        s.pos_min_soft_counts + (s.pos_max_soft_counts - s.pos_min_soft_counts) / 2
    });
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = center);
    let mut k = mg90::kernel();
    let mut p: Mg90 = mg90_d::plant(center as f64, seed);
    p.load = load;
    let mut step = |p: &mut Mg90| {
        let f = p.step(k.io.motor.last);
        k.on_scan(f, &sh);
        k.io.motor.last
    };
    for _ in 0..400 {
        step(&mut p);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..HOLD_MS * TICKS_PER_MS {
        step(&mut p);
    }
    let out = (30.0 * COUNTS_PER_DEG).round() as i32;
    [center + out, center, center - out, center]
        .into_iter()
        .map(|goal| {
            sh.table
                .with_mut(|t| t.control.lifecycle.goal_position = goal);
            let (mut errs, mut i) = (Vec::new(), Vec::new());
            let mut braked = 0;
            let ticks = hold_ms * TICKS_PER_MS;
            for n in 0..ticks {
                let cmd = step(&mut p);
                if n >= ticks - DRIVEN_MS * TICKS_PER_MS {
                    braked += matches!(cmd, Some(MotorCmd::Brake)) as u32;
                }
                if n >= ticks - REST_MS * TICKS_PER_MS {
                    errs.push(p.sensed() - goal as f64);
                    i.push(p.i);
                }
            }
            let h = Hold {
                err: median(&mut errs),
                i_mean: i.iter().sum::<f64>() / i.len() as f64,
                braked,
                faults: sh.table.with(|t| t.telemetry.common.fault_flags),
            };
            (goal, h)
        })
        .collect()
}

/// Every hold of `hold_ms`, each way round, over three pot-noise seeds:
/// it rests within the zone of its goal, twice the set's deadband on the
/// identity table, with the drive on through its last second and the
/// current holding the load within the friction's level. The zone is the
/// region a free hold parks in once settled, and a loaded one is held
/// there, so the noise on the estimate at the band's edge never wakes its
/// integral. Parked as a free shaft is, the shaft
/// rests 7 to 11 counts short on the load's side, as the bench's gravity
/// arm does.
fn rests_in_the_zone(hold_ms: u32) {
    let zone = 2.0
        * mg90_d::rig(|_| {})
            .table
            .with(|t| t.config.loop_position.pos_deadband_counts) as f64;
    for seed in 1..=3u64 {
        for load in [-LOAD, LOAD] {
            for (goal, h) in holds(load, seed, hold_ms) {
                let at = format!("seed {seed} load {load:.0} goal {goal}: {h:?}");
                assert_eq!(h.faults, 0, "{at}");
                assert_eq!(h.braked, 0, "parked: {at}");
                assert!(h.err.abs() <= zone, "rest outside the zone: {at}");
                assert!(
                    (h.i_mean + load).abs() <= mg90_d::DRAG_FC,
                    "the current is not the load's: {at}"
                );
            }
        }
    }
}

#[test]
fn a_loaded_hold_rests_in_the_zone_with_the_drive_on() {
    rests_in_the_zone(HOLD_MS);
}

#[test]
fn a_long_loaded_hold_rests_in_the_zone_with_the_drive_on() {
    rests_in_the_zone(LONG_MS);
}
