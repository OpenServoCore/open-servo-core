//! The current every consumer reads is the winding's. An iron-core motor's
//! loss conductance across its inductance draws a current the shunt sees
//! only in the drive window: the crest sample reads the winding's
//! period-mean current plus that conductance times the window's volts
//! across the inductance, `(1 - D) x V_on` once the current has settled.
//! The rig is the servo of record's (`mg90_d`), its plant's shunt reading
//! the bench winding's conductance ([`mg90_d::WINDOW_G`]).

use osc_integration::mg90::{self, Mg90};
use osc_integration::mg90_d;
use osc_integration::plant::FakeIo;
use osc_servo_core::regions::telemetry::tick_flag;
use osc_servo_core::{Kernel, Mode, RegionStorage, Shared};

/// Counts the winding may sit off the goal and the published current off
/// the winding: the plant carries the whole-pulse fit's 3.81 mS and the set
/// that plus the sample's lateness, 3.93, a 3 % gap, 1.3 counts here at
/// most; the fit's spread over its choices, 6 % either way, adds about 1
/// more. With no window term taken off both sit 16 to 20 counts off.
const TOL: f64 = 5.0;

fn tick(k: &mut Kernel<FakeIo>, sh: &Shared, p: &mut Mg90) {
    let f = p.step(k.io.motor.last);
    k.on_scan(f, sh);
}

/// Current mode at `goal` on a held rotor mid travel, 300 ms: the
/// winding's current over the last 100 ms and the published current over
/// that span's read rows.
fn held(goal: i16) -> (f64, f64) {
    let sh = mg90_d::rig(|t| t.control.lifecycle.torque_enable = false);
    let mut p = mg90_d::plant(2048.0, 1);
    p.locked = true;
    let mut k = mg90::kernel();
    for _ in 0..400 {
        tick(&mut k, &sh, &mut p);
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = goal;
        t.control.lifecycle.torque_enable = true;
    });
    for _ in 0..4000 {
        tick(&mut k, &sh, &mut p);
    }
    let (mut winding, mut read, mut rows) = (0.0, 0.0, 0);
    for _ in 0..2000 {
        tick(&mut k, &sh, &mut p);
        winding += p.i;
        let (i, flags) = sh
            .table
            .with(|t| (t.telemetry.tick.i_tick_counts, t.telemetry.tick.tick_flags));
        if flags & tick_flag::I_VALID != 0 {
            read += i as f64;
            rows += 1;
        }
    }
    let faults = sh.table.with(|t| t.telemetry.common.fault_flags);
    assert_eq!(faults, 0, "goal {goal}");
    assert!(rows >= 1900, "goal {goal}: {rows} read rows of 2000");
    (winding / 2000.0, read / rows as f64)
}

/// Held at 150 and 400 counts either way (11 and 29 % duty on the 5 V
/// rail), the current loop puts the goal in the winding and the published
/// current is the winding's: the stored limits, the friction line, the
/// heat constants and the resistance are all winding currents. With no
/// window term taken off the loop holds the reading at the goal, so the
/// winding sits under it by the term, 20 counts at 150 and 16 at 400, and
/// the published current over the winding by the same.
#[test]
fn a_held_rotor_carries_the_goal_and_reads_the_winding_current() {
    for goal in [150i16, 400, -150, -400] {
        let (winding, read) = held(goal);
        assert!(
            (winding - goal as f64).abs() <= TOL,
            "goal {goal}: the winding carries {winding:.1} counts"
        );
        assert!(
            (read - winding).abs() <= TOL,
            "goal {goal}: the published current reads {read:.1} counts, the winding {winding:.1}"
        );
    }
}
