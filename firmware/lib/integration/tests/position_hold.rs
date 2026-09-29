//! Position hold against an MG90-scale plant: the deadband park must end an
//! arrival, never relay it into a limit cycle. The kernel runs the bench
//! MG90's identified set in linearized counts (2S rail, 280-count limit,
//! deadband 12) against the physics that decides a hold: an R-L winding with
//! back-EMF (Brake shorts it, Coast opens it), Coulomb plus viscous friction,
//! 5.7 counts of gear lash between the rotor and the pot, and a pot with 1.2
//! raw counts of noise that wanders +-5 counts off the position table, the
//! residual linearization leaves.
//!
//! That residual is what makes arrivals fast: the velocity loop chases it,
//! cruises behind the profile, and a shaft still a deceleration ramp behind
//! when the ramp starts runs it on the velocity limit (the position loop
//! asks kp x 12 = 1884 counts/s at the band edge) and enters the band at
//! speed. Friction alone stops it inside the band only below
//! `coast_stop_cps`.

use osc_integration::plant::{BIAS, FakeIo, kernel, lut_live, seed, vbus_raw};
use osc_servo_core::kernel::limits::flag;
use osc_servo_core::pos_lut::{GRID, POINTS};
use osc_servo_core::{
    ControlTable, Kernel, MotorCmd, RegionStorage, SensorFrame, Shared, StallResponse,
};

/// 7.1 V in vmotor-tap counts.
const VBUS: f64 = 2880.0;
const R: f64 = 1.775;
const KE: f64 = 0.14162;
/// c/s per FAST tick per current count.
const B: f64 = 0.03083;
const FC: f64 = 53.19;
const FV: f64 = 0.005135;
/// L/R = 0.87 mH / 4.89 ohm, in FAST ticks.
const TAU_E: f64 = 3.56;
const TICK_HZ: f64 = 20_000.0;
const SIGMA_RAW: f64 = 1.2;
const LASH: f64 = 5.7;
/// Amplitude and period, linearized counts.
const RESIDUAL: (f64, f64) = (5.0, 60.0);
const DEADBAND: u16 = 12;
const CENTER: i32 = 2050;
/// Per goal: the move, the settle, and the judged tail.
const HOLD_TICKS: u32 = 16_000;
const TAIL_TICKS: u32 = 6_000;
const ARR: u32 = 1200;

/// Entering at one edge, the speed Coulomb friction alone sheds across the
/// band: v^2 = 2 x (B x FC) x 2 x deadband.
fn coast_stop_cps() -> f64 {
    (2.0 * B * TICK_HZ * FC * 2.0 * DEADBAND as f64).sqrt()
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn unit(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn gauss(&mut self) -> f64 {
        let (u, v) = (self.unit(), self.unit());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
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

/// The table's inverse: the raw count it maps onto `lin`.
fn raw_of(lut: &[i16; POINTS], lin: f64) -> f64 {
    let at = |k: usize| (k * GRID) as f64 + lut[k] as f64;
    let k = (0..POINTS - 1)
        .find(|&k| at(k + 1) > lin)
        .unwrap_or(POINTS - 2);
    (k * GRID) as f64 + GRID as f64 * (lin - at(k)) / (at(k + 1) - at(k))
}

/// Rotor and pot shaft in linearized counts, the pot riding the rotor
/// through the lash.
struct Mg90 {
    rotor: f64,
    out: f64,
    omega: f64,
    i: f64,
    rng: Rng,
    lut: [i16; POINTS],
}

impl Mg90 {
    fn new(lin: f64, lut: [i16; POINTS], seed: u64) -> Self {
        Self {
            rotor: lin,
            out: lin,
            omega: 0.0,
            i: 0.0,
            rng: Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1),
            lut,
        }
    }

    /// The linearized position the kernel reads, noise aside.
    fn sensed(&self) -> f64 {
        self.out - RESIDUAL.0 * (std::f64::consts::TAU * self.out / RESIDUAL.1).sin()
    }

    /// One FAST tick under `cmd`, with the chip's mapping: a sub-tick duty
    /// coasts, Brake shorts the winding, Coast and Disabled open it.
    fn step(&mut self, cmd: Option<MotorCmd>) -> SensorFrame {
        let volts = match cmd {
            Some(MotorCmd::Drive { duty, .. }) => {
                let ticks = (duty.0.unsigned_abs() as u32 * ARR + (1 << 14)) >> 15;
                (ticks != 0).then(|| ticks as f64 / ARR as f64 * VBUS * (duty.0 as f64).signum())
            }
            Some(MotorCmd::Brake) => Some(0.0),
            _ => None,
        };
        self.i = match volts {
            Some(v) => self.i + ((v - KE * self.omega) / R - self.i) * (1.0 - (-1.0 / TAU_E).exp()),
            None => 0.0,
        };
        if self.omega != 0.0 || self.i.abs() > FC {
            let dir = if self.omega != 0.0 {
                self.omega.signum()
            } else {
                self.i.signum()
            };
            let next = self.omega + B * (self.i - FC * dir - FV * self.omega);
            // friction stops a coasting rotor, it never reverses it
            self.omega = if self.omega != 0.0 && next.signum() != self.omega.signum() {
                0.0
            } else {
                next
            };
        }
        self.rotor += self.omega / TICK_HZ;
        self.out = self
            .out
            .clamp(self.rotor - LASH / 2.0, self.rotor + LASH / 2.0);
        let raw = raw_of(&self.lut, self.sensed()) + SIGMA_RAW * self.rng.gauss();
        let fwd = !matches!(cmd, Some(MotorCmd::Drive { duty, .. }) if duty.0 < 0);
        let (sample, va, vb) = if fwd {
            (self.i, VBUS as u16, 0)
        } else {
            (-self.i, 0, VBUS as u16)
        };
        SensorFrame {
            pos: raw.round().clamp(0.0, 4095.0) as u16,
            current: (BIAS as f64 + sample).round().clamp(0.0, 4095.0) as u16,
            current_trough: BIAS,
            vmotor_a: va,
            vmotor_a_trough: va,
            vmotor_b: vb,
            vmotor_b_trough: vb,
            vcal: 1200,
            vbus_raw: vbus_raw(VBUS as u16),
            ntc_raw: 2048,
            tick: 0,
        }
    }

    /// What the rail supplies: the winding current over the on fraction.
    fn supply(&self, cmd: Option<MotorCmd>) -> f64 {
        match cmd {
            Some(MotorCmd::Drive { duty, .. }) => (self.i * duty.0 as f64 / 32767.0).abs(),
            _ => 0.0,
        }
    }
}

/// The bench MG90's saved set, identified in linearized counts, then `cfg`.
fn rig(lut: &[i16; POINTS], cfg: impl FnOnce(&mut ControlTable)) -> Shared {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        let c = &mut t.config;
        c.loop_current.i_kp_q88 = 508;
        c.loop_current.i_ki_q412 = 2284;
        c.loop_current.i_kaw_q412 = 4568;
        c.loop_velocity.v_kp_q88 = 522;
        c.loop_velocity.v_ki_q412 = 1312;
        c.loop_velocity.v_kaw_q412 = 2624;
        c.loop_velocity.j_ff_q88 = 831;
        c.loop_position.p_kp_q88 = 40212;
        c.loop_position.pos_deadband_counts = DEADBAND;
        c.loop_position.velocity_limit_cps = 1500;
        c.loop_position.accel_limit_q88 = 3840;
        c.limits.current_limit_counts = 280;
        c.limits.stall_response = StallResponse::Yield;
        c.limits.stall_time_ms = 500;
        c.limits.stall_yield_counts = 168;
        c.limits.stall_release_counts = 84;
        c.limits.stall_tau_trip_counts = 2182;
        c.limits.oc_trip_counts = 1843;
        c.limits.oc_trip_ticks = 8;
        c.limits.openloop_zero_brake = false;
        c.thermal.v_undervolt_counts = 1200;
        c.fusion.l1_q016 = 8640;
        c.fusion.l2_q88 = 3180;
        c.fusion.l3_q88 = 81;
        c.fault_cfg.pos_error_counts = 400;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = 160;
        cal.sense.v_window_min_ticks = 160;
        cal.motor.r_q12 = 7270;
        cal.motor.ke_vpc_q = 580;
        cal.motor.recip_ke_q = 7228;
        cal.motor.b_i_q313 = 2524;
        cal.motor.fric_fc_counts = 53;
        cal.motor.fric_fv_q016 = 336;
        cal.winding.r0_q12 = 7270;
    });
    sh.table.with_mut(cfg);
    lut_live(&sh, lut);
    sh
}

/// A start and `n` goals after it, alternating about the center: steps of
/// 300 to 450 counts, the first one up.
fn goals(seed: u64, n: usize) -> Vec<i32> {
    let mut rng = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1);
    (0..=n)
        .map(|k| {
            let half = 150 + (rng.next() % 76) as i32;
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
            k: kernel(),
            p: Mg90::new(start as f64, lut, seed_n),
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
        self.k.on_tick(f, &self.sh);
        applied
    }

    fn run(&mut self, ticks: u32) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    fn hold(&mut self, goal: i32, ticks: u32) -> Hold {
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
        for t in 0..ticks {
            let applied = self.tick();
            let parked = matches!(
                self.k.io.motor.last,
                Some(MotorCmd::Coast | MotorCmd::Brake)
            );
            if parked && h.entry_cps.is_nan() {
                h.entry_cps = self.p.omega.abs();
            }
            h.redrives += (was_parked && !parked && !h.entry_cps.is_nan()) as u32;
            if t >= ticks - TAIL_TICKS {
                lo = lo.min(self.p.out);
                hi = hi.max(self.p.out);
                h.wakes += (was_parked && !parked) as u32;
                h.supply += self.p.supply(applied) / TAIL_TICKS as f64;
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

fn holds(seed_n: u64, n: usize) -> Vec<Hold> {
    holds_on(table(), seed_n, n)
}

fn holds_on(lut: [i16; POINTS], seed_n: u64, n: usize) -> Vec<Hold> {
    let g = goals(seed_n, n);
    let mut s = Session::new(seed_n, g[0], lut, |_| {});
    g[1..].iter().map(|&g| s.hold(g, HOLD_TICKS)).collect()
}

/// Seed 28's first step: the shaft trails the profile into the
/// deceleration and enters the band at about 1880 counts/s, faster than
/// friction can stop it there.
#[test]
fn hold_rests_after_a_fast_arrival() {
    let h = holds(28, 1)[0];
    assert!(h.entry_cps > coast_stop_cps(), "not a fast arrival: {h:?}");
    assert!(h.spread <= 1.0 && h.wakes == 0, "limit cycle: {h:?}");
    assert!(
        h.err.abs() <= DEADBAND as f64,
        "rest outside the band: {h:?}"
    );
    assert_eq!(h.faults, 0, "{h:?}");
}

/// Twelve sessions of eight alternating steps, 300 to 450 counts each way.
#[test]
fn hold_never_cycles_over_many_arrivals() {
    let all: Vec<Hold> = std::thread::scope(|s| {
        let runs: Vec<_> = (1..=12u64).map(|n| s.spawn(move || holds(n, 8))).collect();
        runs.into_iter()
            .flat_map(|r| r.join().expect("session"))
            .collect()
    });
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
        // 12 raw counts at the table's gain
        assert!(
            h.err.abs() <= DEADBAND as f64 * 1.25,
            "rest outside the band: {h:?}"
        );
        assert_eq!(h.faults, 0, "{h:?}");
    }
}

/// Parked at rest after that fast arrival, the winding is shorted and
/// still: the rail supplies nothing and no current circulates.
#[test]
fn hold_draws_no_supply_current_at_rest() {
    let h = holds(28, 1)[0];
    assert_eq!(h.wakes, 0, "{h:?}");
    assert_eq!(h.braked, TAIL_TICKS, "{h:?}");
    assert_eq!(h.supply, 0.0, "{h:?}");
    assert!(h.i_max < 0.5, "{h:?}");
}

/// Three seconds parked under pot noise, the stall timer armed to fault
/// and the collision trip at the 300 mA class default on this sense chain:
/// the parked loop never pins its current reference, so nothing trips.
#[test]
fn hold_never_trips_the_stall_detector() {
    let g = goals(7, 1);
    let mut s = Session::new(7, g[0], table(), |t| {
        t.config.limits.stall_response = StallResponse::Fault;
        t.config.limits.stall_tau_trip_counts = 335;
    });
    let goal = g[1];
    s.hold(goal, HOLD_TICKS);
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

fn sweep(lut: [i16; POINTS]) -> Vec<Hold> {
    std::thread::scope(|s| {
        let runs: Vec<_> = (1..=12u64)
            .map(|n| s.spawn(move || holds_on(lut, n, 8)))
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
    let base = redriven(&sweep([0; POINTS])).len();
    for (lut, gain) in [(steep(), 1.625), (steepest(), 4.0)] {
        let all = sweep(lut);
        let again = redriven(&all);
        assert!(
            again.len() <= base,
            "gain {gain}: {} of {} holds drove again, {base} on the identity: {again:?}",
            again.len(),
            all.len()
        );
        for h in &all {
            assert!(
                h.err.abs() <= DEADBAND as f64 * gain,
                "gain {gain}: rest outside the band: {h:?}"
            );
            assert_eq!(h.faults, 0, "gain {gain}: {h:?}");
        }
    }
}

/// The same sweep: no hold on a steep table relays into a limit cycle.
#[test]
fn hold_never_cycles_on_a_steep_table() {
    for (lut, gain) in [(steep(), 1.625), (steepest(), 4.0)] {
        let all = sweep(lut);
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
