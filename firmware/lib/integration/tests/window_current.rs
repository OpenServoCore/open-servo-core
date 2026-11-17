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
use osc_servo_core::kernel::DECIM_MED;
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

/// Ohm per vcount over ccount on the set's scales.
const OHM: f64 = mg90_d::CURRENT_COUNTS_PER_A / mg90_d::TERMINAL_COUNTS_PER_V;

/// One medium period's sums at the cruise: the half's volts and the
/// current three ways, the firmware's published one, the shunt's less the
/// set's term exactly and the same less it in whole counts, each rebuilt
/// from the plant.
#[derive(Default)]
struct Half {
    v: f64,
    fw: f64,
    exact: f64,
    whole: f64,
    valid: bool,
}

/// Velocity at 2500 c/s from the low end on the record rig with a contact
/// of `slope` (vcount per ccount), the brush term off: the periods past
/// the first 400.
fn cruise(slope: f64) -> Vec<Half> {
    let sh = mg90_d::rig(|t| {
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 2500;
        t.calib.motor.r_brush_q12 = 0;
    });
    let (g, arr) = sh
        .table
        .with(|t| (t.calib.motor.g_window_q016 as i64, mg90::ARR as i64));
    let k_q16 = (g * mg90::TIMING.recip_arr_q24 as i64 * mg90_d::VBUS as i64) >> 24;
    let mut k = mg90::kernel();
    let mut p = mg90_d::plant(400.0, 1);
    p.contact = Some(mg90::Contact {
        slope,
        ..mg90_d::CONTACT
    });
    let m = mg90_d::motor();
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    let mut halves = vec![];
    for period in 0..1900 {
        let mut h = Half {
            valid: true,
            ..Half::default()
        };
        for _ in 0..DECIM_MED {
            tick(&mut k, &sh, &mut p);
            let (i, duty, vdiff, flags) = sh.table.with(|t| {
                let tk = &t.telemetry.tick;
                (
                    tk.i_tick_counts,
                    tk.duty_ticks,
                    tk.vdiff_counts,
                    tk.tick_flags,
                )
            });
            h.valid &= flags & (tick_flag::I_VALID | tick_flag::V_VALID)
                == tick_flag::I_VALID | tick_flag::V_VALID;
            let width = duty.unsigned_abs() as i64;
            h.v += width as f64 * vdiff as f64 / arr as f64;
            h.fw += i as f64;
            let x = p
                .contact
                .filter(|_| p.omega != 0.0)
                .map_or(0.0, |c| c.at(p.i).0);
            let shunt = p.i + p.window_g * (m.vbus - (m.r * p.i + m.ke * p.omega + x));
            let term = k_q16 * (arr - width);
            h.exact += shunt - term as f64 / 65536.0;
            h.whole += shunt - ((term + (1 << 15)) >> 16) as f64;
        }
        if period >= 400 {
            halves.push(h);
        }
    }
    assert!(
        p.out < 3400.0,
        "the cruise ran out of travel at {:.0}",
        p.out
    );
    halves
}

/// The learner's dither slope (the A3 form) on the halves' volts and a
/// current: the volts' and the current's 1/8 filters off their 1/64
/// trails, the volts less the stored winding on the current, fit on the
/// current's deviation once it has held under 440 counts for 300 halves.
/// The reading in ohm, the deviation's rms and the halves fit.
fn a3(halves: &[Half], i: impl Fn(&Half) -> f64, r: f64) -> (f64, f64, usize) {
    let (a, b) = (1.0 / 64.0, 1.0 / 8.0);
    let (mut vf, mut jf) = (halves[0].v, i(&halves[0]));
    let (mut va, mut ia) = (vf, jf);
    let (mut sxy, mut sxx, mut held, mut n) = (0.0, 0.0, 0, 0);
    for h in &halves[1..] {
        if !h.valid {
            held = 0;
            continue;
        }
        vf += (h.v - vf) * b;
        jf += (i(h) - jf) * b;
        let (dv, di) = (vf - va, jf - ia);
        va += (vf - va) * a;
        ia += (jf - ia) * a;
        held = if di.abs() > 440.0 { 0 } else { held + 1 };
        if held > 300 && (i(h) - ia).abs() <= 440.0 {
            sxy += (dv - r * di) * di;
            sxx += di * di;
            n += 1;
        }
    }
    (sxy / sxx * OHM, (sxx / n as f64).sqrt(), n)
}

/// The rested bench's contact, 1.48 ohm.
const RESTED: f64 = 1.48 / OHM;

/// At the cruise the term sits near 17 counts and the dither moves it
/// about 0.2 counts a tick: a whole-count term passes only the part the
/// duty's tick noise carries across a step, and the slope the dither
/// reads on the published current comes out high by the rest, 0.11 ohm
/// worked and 0.05 rested. The kernel carries the term's fraction from
/// reading to reading, and the slope reads the exact term's.
///
/// The bound: the published current differs from the exact one by the
/// shunt code's rounding, a uniform half count a tick, and the carry's
/// two ends, `(DECIM_MED + 2) / 12` counts squared a half, white. The 1/8
/// filter passes `sqrt(1/15)` of it to the deviation, over the halves its
/// memory leaves apart (one in 8): the slope's sd is `Z x sd / (rms x
/// sqrt(n / 8))` on the winding's whole slope `Z`, about 0.004 ohm, and the
/// noise in the deviation biases it low by `Z x (sd / rms)^2`. The pin
/// allows five sd and the bias; the whole-count term on the same run sits
/// outside it.
#[test]
fn the_dither_slope_reads_the_exact_window_term() {
    let r = mg90_d::motor().r;
    for (name, slope) in [("worked", mg90_d::CONTACT.slope), ("rested", RESTED)] {
        let halves = cruise(slope);
        let (exact, rms, n) = a3(&halves, |h| h.exact, r);
        let (fw, _, _) = a3(&halves, |h| h.fw, r);
        let (whole, _, _) = a3(&halves, |h| h.whole, r);
        assert!(n > 1000, "{name}: {n} halves fit");
        let z = exact + r * OHM;
        let sd = ((DECIM_MED as f64 + 2.0) / 12.0 / 15.0).sqrt();
        let tol = 5.0 * z * sd / (rms * (n as f64 / 8.0).sqrt()) + z * (sd / rms).powi(2);
        assert!(
            (fw - exact).abs() <= tol,
            "{name}: {fw:.3} ohm on the published current, {exact:.3} on the exact term, within {tol:.3}"
        );
        assert!(
            (whole - exact).abs() > tol,
            "{name}: the whole-count term reads {whole:.3}, {exact:.3} exact, within {tol:.3}"
        );
    }
}
