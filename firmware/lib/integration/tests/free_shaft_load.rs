//! The load estimate of a free shaft on the servo of record's rig
//! (`mg90_d`): nothing loads the output, so the observer's disturbance
//! reads none, in the move and when the shaft stops. The estimate is the
//! torque the motion model cannot explain. A model without the friction
//! line's viscous part reads that part as a load of its own size while
//! the shaft moves (about 50 ccounts at the set's move speed, over the
//! friction level of 43), and the estimate comes down the profile's ramp
//! only to about the friction level, where a still shaft lets it stand:
//! a free shaft arrives on the edge of the hold's park test
//! (`position::unloaded`).

use osc_integration::mg90::{self, Mg90};
use osc_integration::mg90_d::{self, COUNTS_PER_DEG};
use osc_servo_core::RegionStorage;

const TICKS_PER_MS: u32 = 20;
/// The bench's hold block: a 45 degree move, the goal held 2 s.
const MOVE_DEG: f64 = 45.0;
const HOLD_MS: u32 = 2000;
/// The cruise is judged where the rotor runs within this share of the
/// set's move speed.
const CRUISE_SHARE: f64 = 0.95;

#[derive(Debug)]
struct Move {
    /// The published estimate along the travel, ccounts: its median over
    /// the cruise and its value on the tick the rotor first stops.
    cruise: f64,
    stop: f64,
    faults: u8,
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// The rig at rest at its soft range's middle, free, torque on, then a
/// 45 degree move out and back each way.
fn moves(seed: u64) -> Vec<(i32, Move)> {
    let sh = mg90_d::rig(|_| {});
    let (center, speed) = sh.table.with(|t| {
        let s = &t.config.pos_soft;
        (
            s.pos_min_soft_counts + (s.pos_max_soft_counts - s.pos_min_soft_counts) / 2,
            t.config.loop_position.velocity_limit_cps as f64,
        )
    });
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = center);
    let mut k = mg90::kernel();
    let mut p: Mg90 = mg90_d::plant(center as f64, seed);
    let mut step = |p: &mut Mg90| {
        let f = p.step(k.io.motor.last);
        k.on_scan(f, &sh);
    };
    for _ in 0..400 {
        step(&mut p);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..1500 * TICKS_PER_MS {
        step(&mut p);
    }
    let out = (MOVE_DEG * COUNTS_PER_DEG).round() as i32;
    let mut from = center;
    [center + out, center, center - out, center]
        .into_iter()
        .map(|goal| {
            let way = (goal - from).signum() as f64;
            from = goal;
            sh.table
                .with_mut(|t| t.control.lifecycle.goal_position = goal);
            let (mut cruise, mut stop) = (Vec::new(), None);
            for _ in 0..HOLD_MS * TICKS_PER_MS {
                step(&mut p);
                let tau = way * sh.table.with(|t| t.telemetry.estimates.tau_d_counts) as f64;
                if p.omega * way >= CRUISE_SHARE * speed {
                    cruise.push(tau);
                } else if !cruise.is_empty() && stop.is_none() && p.omega == 0.0 {
                    stop = Some(tau);
                }
            }
            let m = Move {
                cruise: median(&mut cruise),
                stop: stop.expect("the rotor stops"),
                faults: sh.table.with(|t| t.telemetry.common.fault_flags),
            };
            (goal, m)
        })
        .collect()
}

/// Every 45 degree move of a free shaft, each way round, over three
/// pot-noise seeds: the estimate is within half the friction level of
/// zero in the cruise and on the tick the rotor first stops. The rig's
/// drag is the bench winding's own line, not the stored one, so the
/// estimate keeps what the two differ by, a few ccounts.
#[test]
fn a_free_shaft_reads_no_load_in_the_move_or_at_its_end() {
    let half = mg90_d::rig(|_| {})
        .table
        .with(|t| t.calib.motor.fric_fc_counts) as f64
        / 2.0;
    for seed in 1..=3u64 {
        for (goal, m) in moves(seed) {
            let at = format!("seed {seed} goal {goal}: {m:?}");
            assert_eq!(m.faults, 0, "{at}");
            assert!(m.cruise.abs() <= half, "a load in the cruise: {at}");
            assert!(m.stop.abs() <= half, "a load at the stop: {at}");
        }
    }
}
