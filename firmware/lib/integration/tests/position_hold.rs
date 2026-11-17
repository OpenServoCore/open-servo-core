//! Position hold against an MG90-scale plant: the deadband park must end an
//! arrival, never relay it into a limit cycle. The kernel runs the bench
//! MG90's identified set in linearized counts (2S rail, 280-count limit,
//! deadband 12) against the physics that decides a hold: an R-L winding with
//! back-EMF (Brake shorts it, Coast opens it), Coulomb plus viscous friction,
//! 5.7 counts of gear lash between the rotor and the pot, and a pot with 1.2
//! raw counts of noise that wanders +-5 counts off the position table, the
//! residual linearization leaves.
//!
//! A fast arrival is a shaft that trails its profile. Once the profile has
//! landed the position loop asks kp x error, which the velocity limit clamps
//! until the error is inside the band (kp x 12 = 1884 counts/s at the band
//! edge, over the limit's 1500), so the shaft runs in on the limit and
//! enters the band at speed. Left alone the loop tracks the profile and
//! lands slower than that, so the fast arrivals here are held ones: the
//! rotor is held from the goal's write until the profile has landed, then
//! let go. Friction alone stops the shaft inside the band only below
//! `coast_stop_cps`.

use osc_integration::mg90::{self, B, FC, Mg90, Rng, SIGMA_RAW, TICK_HZ};
use osc_integration::plant::FakeIo;
use osc_servo_core::adc::{ADC_WIDE_SHIFT, ADC_WIDE_SLOTS};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::kernel::phases::loops::limits::flag;
use osc_servo_core::pos_lut::{GRID, POINTS};
use osc_servo_core::{ControlTable, Kernel, MotorCmd, RegionStorage, Shared, StallResponse};

const DEADBAND: u16 = 12;
const CENTER: i32 = 2050;
/// Per goal: the move, the settle, and the judged tail.
const HOLD_TICKS: u32 = 16_000;
const TAIL_TICKS: u32 = 6_000;
/// A held rotor is let go this long after its profile has landed.
const LET_GO_TICKS: u32 = 400;

/// A rest `err` off the goal lies in the band where a raw count spans `gain`
/// linearized counts. The kernel judges the band on a reading that carries
/// the pot's noise, which `err` leaves out.
fn rests_in_band(err: f64, gain: f64) -> bool {
    err.abs() <= (DEADBAND as f64 + SIGMA_RAW) * gain
}

/// Entering at one edge, the speed Coulomb friction alone sheds across the
/// band: v^2 = 2 x (B x FC) x 2 x deadband.
fn coast_stop_cps() -> f64 {
    (2.0 * B * TICK_HZ * FC * 2.0 * DEADBAND as f64).sqrt()
}

/// Local gain 1.25 over raw 1600..2592, easing back to the identity by 3600.
fn table() -> [i16; POINTS] {
    ramp(100, 162, 225, 4)
}

/// Identity to point `a`, local gain `(16 + rise) / 16` from `a` to `b`,
/// easing back to the identity by point `c`.
fn ramp(a: i32, b: i32, c: i32, rise: i32) -> [i16; POINTS] {
    let peak = rise * (b - a);
    let mut k = [0i16; POINTS];
    for (n, p) in (0i32..).zip(k.iter_mut()) {
        *p = if n <= a || n >= c {
            0
        } else if n <= b {
            (rise * (n - a)) as i16
        } else {
            (peak - peak * (n - b) / (c - b)) as i16
        };
    }
    k
}

/// Local gain 1.625 over raw 1600..2160, the whole travel: the bench
/// MG90's gain at its rest spot.
fn steep() -> [i16; POINTS] {
    ramp(100, 135, 179, 10)
}

/// Local gain 4 over raw 1696..1888, the whole travel: the most the hold
/// band scales by.
fn steepest() -> [i16; POINTS] {
    ramp(106, 118, 190, 48)
}

/// The bench MG90's saved set, then `cfg`. One departure: the profile
/// asks for 20 counts/s per medium tick where the bench saved 15, the
/// steeper ramp into the band of the two.
fn rig(lut: &[i16; POINTS], cfg: impl FnOnce(&mut ControlTable)) -> Shared {
    mg90::rig(lut, |t| {
        t.config.loop_position.accel_limit_q88 = 20 << 8;
        cfg(t);
    })
}

/// A start and `n` goals after it, alternating about the center: steps of
/// 300 to 450 counts, the first one up.
fn goals(seed: u64, n: usize) -> Vec<i32> {
    let mut rng = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
    (0..=n)
        .map(|k| {
            let half = 150 + (rng.next_u64() % 76) as i32;
            if k % 2 == 0 {
                CENTER - half
            } else {
                CENTER + half
            }
        })
        .collect()
}

#[derive(Debug, Clone, Copy)]
struct Hold {
    /// Shaft speed when the kernel first parks.
    entry_cps: f64,
    /// Parks left after the first, the whole hold: the loop re-engaging.
    redrives: u32,
    /// Over the judged tail: pot shaft travel, parks left, mean supply
    /// current, peak winding current, ticks braked.
    spread: f64,
    wakes: u32,
    supply: f64,
    i_max: f64,
    braked: u32,
    /// Sensed position less the goal, at the end.
    err: f64,
    faults: u8,
}

impl Hold {
    fn cycles(&self) -> bool {
        self.spread > 8.0
    }
}

struct Session {
    sh: Shared,
    k: Kernel<FakeIo>,
    p: Mg90,
    goal: i32,
}

impl Session {
    /// Torque on at rest at `start`, parked.
    fn new(
        seed_n: u64,
        start: i32,
        lut: [i16; POINTS],
        cfg: impl FnOnce(&mut ControlTable),
    ) -> Self {
        let sh = rig(&lut, cfg);
        let mut s = Self {
            sh,
            k: mg90::kernel(),
            p: Mg90::new(start as f64, lut, seed_n),
            goal: start,
        };
        s.run(400);
        s.sh.table.with_mut(|t| {
            t.control.lifecycle.goal_position = start;
            t.control.lifecycle.torque_enable = true;
        });
        s.run(4_000);
        s
    }

    fn tick(&mut self) -> Option<MotorCmd> {
        let applied = self.k.io.motor.last;
        let f = self.p.step(applied);
        self.k.on_scan(f, &self.sh);
        applied
    }

    fn run(&mut self, ticks: u32) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    /// FAST ticks the profile takes from the last goal to `goal` under the
    /// set's limits: the trapezoid's d / v + v / a, the triangle's
    /// 2 x sqrt(d / a) where it never reaches the limit.
    fn profile_ticks(&self, goal: i32) -> u32 {
        let (v, a) = self.sh.table.with(|t| {
            let lp = &t.config.loop_position;
            let per_s = TICK_HZ / DECIM_MED as f64;
            (
                lp.velocity_limit_cps as f64,
                lp.accel_limit_q88 as f64 / 256.0 * per_s,
            )
        });
        let d = (goal - self.goal).abs() as f64;
        let s = if d >= v * v / a {
            d / v + v / a
        } else {
            2.0 * (d / a).sqrt()
        };
        (s * TICK_HZ).ceil() as u32
    }

    /// `goal` for `ticks`. `held`: the rotor is held until `LET_GO_TICKS`
    /// after the profile has landed, and the hold runs that much longer.
    fn hold(&mut self, goal: i32, ticks: u32, held: bool) -> Hold {
        let let_go = if held {
            self.profile_ticks(goal) + LET_GO_TICKS
        } else {
            0
        };
        let ticks = ticks + let_go;
        self.goal = goal;
        self.sh
            .table
            .with_mut(|t| t.control.lifecycle.goal_position = goal);
        let mut h = Hold {
            entry_cps: f64::NAN,
            redrives: 0,
            spread: 0.0,
            wakes: 0,
            supply: 0.0,
            i_max: 0.0,
            braked: 0,
            err: 0.0,
            faults: 0,
        };
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        let mut was_parked = false;
        // the park of the previous goal stands until the goal lands; the
        // entry is the first park after the drive leaves it
        let mut left = false;
        for t in 0..ticks {
            self.p.locked = t < let_go;
            let applied = self.tick();
            let parked = matches!(
                self.k.io.motor.last,
                Some(MotorCmd::Coast | MotorCmd::Brake)
            );
            left |= !parked;
            if parked && left && h.entry_cps.is_nan() {
                h.entry_cps = self.p.omega.abs();
            }
            h.redrives += (was_parked && !parked && !h.entry_cps.is_nan()) as u32;
            if t >= ticks - TAIL_TICKS {
                lo = lo.min(self.p.out);
                hi = hi.max(self.p.out);
                h.wakes += (was_parked && !parked) as u32;
                h.supply += self.p.supply() / TAIL_TICKS as f64;
                h.i_max = h.i_max.max(self.p.i.abs());
                h.braked += matches!(applied, Some(MotorCmd::Brake)) as u32;
            }
            was_parked = parked;
        }
        h.spread = hi - lo;
        h.err = self.p.sensed() - goal as f64;
        h.faults = self.sh.table.with(|t| t.telemetry.common.fault_flags);
        h
    }
}

/// Which of a session's arrivals are held.
#[derive(Clone, Copy)]
enum Held {
    None,
    All,
    /// Every other one, starting with the session's parity, so a sweep of
    /// sessions holds steps up and steps down alike.
    Alternate,
}

fn holds_on(lut: [i16; POINTS], seed_n: u64, n: usize, held: Held) -> Vec<Hold> {
    let g = goals(seed_n, n);
    let mut s = Session::new(seed_n, g[0], lut, |_| {});
    (0u64..)
        .zip(&g[1..])
        .map(|(k, &g)| {
            let held = match held {
                Held::None => false,
                Held::All => true,
                Held::Alternate => (k + seed_n).is_multiple_of(2),
            };
            s.hold(g, HOLD_TICKS, held)
        })
        .collect()
}

/// Seed 20's first two steps, one each way, both held: the shaft enters
/// the band faster than friction can stop it there.
fn fast_arrivals() -> Vec<Hold> {
    holds_on(table(), 20, 2, Held::All)
}

#[test]
fn hold_rests_after_a_fast_arrival() {
    for h in fast_arrivals() {
        assert!(h.entry_cps > coast_stop_cps(), "not a fast arrival: {h:?}");
        assert!(h.spread <= 1.0 && h.wakes == 0, "limit cycle: {h:?}");
        assert!(rests_in_band(h.err, 1.25), "rest outside the band: {h:?}");
        assert_eq!(h.faults, 0, "{h:?}");
    }
}

/// Twelve sessions of eight alternating steps, 300 to 450 counts each way,
/// every other arrival held.
#[test]
fn hold_never_cycles_over_many_arrivals() {
    let all = sweep(table(), Held::Alternate);
    let fast = all
        .iter()
        .filter(|h| h.entry_cps > coast_stop_cps())
        .count();
    assert!(fast >= 5, "{fast} fast arrivals: the sweep misses the case");
    let cycling: Vec<&Hold> = all.iter().filter(|h| h.cycles()).collect();
    assert!(
        cycling.is_empty(),
        "{} limit cycles in {} holds: {cycling:?}",
        cycling.len(),
        all.len()
    );
    for h in &all {
        assert!(rests_in_band(h.err, 1.25), "rest outside the band: {h:?}");
        assert_eq!(h.faults, 0, "{h:?}");
    }
}

/// Parked at rest after that fast arrival, the winding is shorted and
/// still: the rail supplies nothing and no current circulates.
#[test]
fn hold_draws_no_supply_current_at_rest() {
    let h = fast_arrivals()[0];
    assert_eq!(h.wakes, 0, "{h:?}");
    assert_eq!(h.braked, TAIL_TICKS, "{h:?}");
    assert_eq!(h.supply, 0.0, "{h:?}");
    assert!(h.i_max < 0.5, "{h:?}");
}

/// Three seconds parked under pot noise, the stall timer armed to fault
/// and the collision trip at 300 mA on this sense chain, under the class
/// default: the parked loop never pins its current reference, so nothing
/// trips.
#[test]
fn hold_never_trips_the_stall_detector() {
    let g = goals(7, 1);
    let mut s = Session::new(7, g[0], table(), |t| {
        t.config.limits.stall_response = StallResponse::Fault;
        t.config.limits.stall_tau_trip_counts = 335;
    });
    let goal = g[1];
    s.hold(goal, HOLD_TICKS, false);
    for _ in 0..60_000 {
        s.tick();
        let (flags, faults) = s.sh.table.with(|t| {
            (
                t.telemetry.limits.limit_flags,
                t.telemetry.common.fault_flags,
            )
        });
        assert_eq!(faults, 0, "faulted in the hold");
        assert_eq!(flags & flag::CEILING, 0, "pinned in the hold");
    }
    assert!((s.p.sensed() - goal as f64).abs() <= DEADBAND as f64);
}

fn sweep(lut: [i16; POINTS], held: Held) -> Vec<Hold> {
    std::thread::scope(|s| {
        let runs: Vec<_> = (1..=12u64)
            .map(|n| s.spawn(move || holds_on(lut, n, 8, held)))
            .collect();
        runs.into_iter()
            .flat_map(|r| r.join().expect("session"))
            .collect()
    })
}

fn redriven(all: &[Hold]) -> Vec<&Hold> {
    all.iter().filter(|h| h.redrives != 0).collect()
}

/// Where a raw count spans `gain` linearized counts the band is still 12
/// raw counts, so pot noise wakes the park no more often than on the
/// identity, and every rest lies inside those 12 raw counts.
#[test]
fn hold_rests_on_a_steep_table() {
    let base = redriven(&sweep([0; POINTS], Held::None)).len();
    for (lut, gain) in [(steep(), 1.625), (steepest(), 4.0)] {
        let all = sweep(lut, Held::None);
        let again = redriven(&all);
        assert!(
            again.len() <= base,
            "gain {gain}: {} of {} holds drove again, {base} on the identity: {again:?}",
            again.len(),
            all.len()
        );
        for h in &all {
            assert!(
                rests_in_band(h.err, gain),
                "gain {gain}: rest outside the band: {h:?}"
            );
            assert_eq!(h.faults, 0, "gain {gain}: {h:?}");
        }
    }
}

/// The same sweep with every other arrival held: no hold on a steep table
/// relays into a limit cycle.
#[test]
fn hold_never_cycles_on_a_steep_table() {
    for (lut, gain) in [(steep(), 1.625), (steepest(), 4.0)] {
        let all = sweep(lut, Held::Alternate);
        let fast = all
            .iter()
            .filter(|h| h.entry_cps > coast_stop_cps())
            .count();
        assert!(fast >= 5, "gain {gain}: {fast} fast arrivals");
        let cycling: Vec<&Hold> = all.iter().filter(|h| h.cycles()).collect();
        assert!(
            cycling.is_empty(),
            "gain {gain}: {} limit cycles in {} holds: {cycling:?}",
            cycling.len(),
            all.len()
        );
    }
}

/// An ADC wide code: raw 1023 holds while the input rises through the 11
/// codes above it, so the pot reads 1023 anywhere from 1023 to 1034.
const WIDE: (u16, u16) = (1023, 11);
/// A goal mid band, on the identity table.
const WIDE_GOAL: i32 = 1029;

/// Holds at `WIDE_GOAL` under the bench's 4-count deadband with `map` in
/// CALIB, from `from`: the hold, then the drive ticks over one more tail.
fn wide_hold(map: [u8; ADC_WIDE_SLOTS], seed_n: u64, from: i32) -> (Hold, usize) {
    let mut s = Session::new(seed_n, from, [0; POINTS], |t| {
        t.config.loop_position.pos_deadband_counts = 4;
        t.calib.adc.adc_wide_mid_q4 = map;
    });
    s.p.wide = Some(WIDE);
    let h = s.hold(WIDE_GOAL, HOLD_TICKS, false);
    let driven = (0..TAIL_TICKS)
        .filter(|_| {
            s.tick();
            !matches!(s.k.io.motor.last, Some(MotorCmd::Coast | MotorCmd::Brake))
        })
        .count();
    (h, driven)
}

/// A goal inside a wide code's dead band: the stuck reading sits 5 to 6
/// counts off it on either side, past the 4-count band, and with the slot
/// zero the hold hunts across the band without end. With the code's slot
/// set the stuck reading lands mid band, and the hold rests there with the
/// drive off, from below and from above.
#[test]
fn hold_rests_inside_a_wide_code() {
    let mut map = [0u8; ADC_WIDE_SLOTS];
    map[(WIDE.0 >> ADC_WIDE_SHIFT) as usize] = (WIDE.1 * GRID as u16 / 2) as u8;
    for seed_n in 1..=4u64 {
        for from in [WIDE_GOAL - 300, WIDE_GOAL + 300] {
            let (h, driven) = wide_hold(map, seed_n, from);
            assert_eq!(driven, 0, "seed {seed_n} from {from}: drove: {h:?}");
            assert!(
                h.wakes == 0 && h.spread <= 1.0,
                "seed {seed_n} from {from}: hunts: {h:?}"
            );
            let lo = WIDE.0 as f64 - 0.5;
            let hi = (WIDE.0 + WIDE.1) as f64 + 0.5;
            let at = h.err + WIDE_GOAL as f64;
            assert!(
                (lo..hi).contains(&at),
                "seed {seed_n} from {from}: rest outside the band: {h:?}"
            );
            assert_eq!(h.faults, 0, "seed {seed_n} from {from}: {h:?}");
        }
    }
}
