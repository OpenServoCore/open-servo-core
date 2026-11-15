//! The velocity loop's speed on the bench MG90 plant when the back-EMF
//! boxcar's level is wrong. The plant's winding is exact for the boxcar's
//! arithmetic, so a stored `r_q12` under the plant's stands in for a
//! boxcar that reads over the shaft: it leaves part of the winding's drop
//! in its speed, more the more current the cruise takes. The loop's mean
//! speed is the pot observer's (`estimator::omega_blend`), so the shaft
//! cruises where it does with the plant's own resistance stored.

use osc_integration::mg90::{self, Mg90};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::pos_lut::POINTS;
use osc_servo_core::{Mode, RegionStorage};

/// The plant's own resistance as the rig stores it.
const R_Q12: u16 = 7270;
const MID: f64 = 2050.0;
/// The rig's velocity limit.
const ASKED: i32 = 1500;
/// A torque against the travel, as a winding current: it lifts the cruise
/// over the back-EMF floor on part of its periods and doubles the current
/// the boxcar's level error scales with.
const LOAD: f64 = 60.0;
/// The run-up and the standing part's forming on the periods that read,
/// then the judged stretch: 3000 counts of travel in all.
const SETTLE_TICKS: u32 = 24_000;
const CRUISE_TICKS: u32 = 16_000;

#[derive(Debug)]
struct Cruise {
    /// Means over the judged stretch, c/s: the rotor, and the boxcar over
    /// the periods that had one.
    rotor: f64,
    boxcar: f64,
    /// The share of the stretch's periods a boxcar fed the loop.
    fed: f64,
    faults: u8,
}

/// Velocity mode at `asked` across mid travel with `r_q12` stored.
fn cruise(asked: i32, r_q12: u16, seed: u64) -> Cruise {
    let dir = asked.signum() as f64;
    let lut = [0i16; POINTS];
    let sh = mg90::rig(&lut, |t| {
        t.calib.motor.r_q12 = r_q12;
        t.config.pos_soft.pos_min_soft_counts = -1_000_000;
        t.config.pos_soft.pos_max_soft_counts = 1_000_000;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = asked;
    });
    let mut k = mg90::kernel();
    let mut p = Mg90::new(MID - dir * ASKED as f64, lut, seed);
    p.load = -dir * LOAD;
    let mut tick = |p: &mut Mg90| {
        let f = p.step(k.io.motor.last);
        k.on_scan(f, &sh);
    };
    for _ in 0..400 {
        tick(&mut p);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    for _ in 0..SETTLE_TICKS {
        tick(&mut p);
    }
    let (mut rotor, mut boxcar) = (0.0, 0.0);
    let (mut periods, mut fed) = (0u32, 0u32);
    for n in 0..CRUISE_TICKS {
        tick(&mut p);
        rotor += p.omega;
        if n % DECIM_MED as u32 != 0 {
            continue;
        }
        periods += 1;
        sh.table.with(|t| {
            if t.telemetry.mode.omega_hat_src != 0 {
                fed += 1;
                boxcar += t.telemetry.estimates.omega_bemf_cps as f64;
            }
        });
    }
    Cruise {
        rotor: rotor / CRUISE_TICKS as f64,
        boxcar: boxcar / fed.max(1) as f64,
        fed: fed as f64 / periods as f64,
        faults: sh.table.with(|t| t.telemetry.common.fault_flags),
    }
}

/// A cruise holds the asked speed with a back-EMF that reads over. With
/// half the plant's resistance stored the boxcar reads more than 1000 c/s
/// over the rotor on the periods that have one, either way round, and the
/// rotor still cruises within 2 % of the asked speed, as it does with the
/// plant's own resistance stored. A loop fed the boxcar's level on those
/// periods cruises a sixth under.
#[test]
fn a_cruise_holds_the_asked_speed_with_a_back_emf_that_reads_over() {
    for seed in 1..=4u64 {
        for asked in [ASKED, -ASKED] {
            let dir = asked.signum() as f64;
            let own = cruise(asked, R_Q12, seed);
            let low = cruise(asked, R_Q12 / 2, seed);
            for c in [&own, &low] {
                assert_eq!(c.faults, 0, "seed {seed} asked {asked}: {c:?}");
                assert!(c.fed > 0.1, "seed {seed} asked {asked}: {c:?}");
            }
            let over = (low.boxcar - low.rotor) * dir;
            assert!(
                over > 1000.0,
                "seed {seed} asked {asked}: the boxcar reads {over:.0} c/s over: {low:?}"
            );
            for c in [&own, &low] {
                let off = (c.rotor - asked as f64) * dir;
                assert!(
                    off.abs() <= 0.02 * ASKED as f64,
                    "seed {seed} asked {asked}: the rotor cruises {off:.0} c/s off: {c:?}"
                );
            }
        }
    }
}
