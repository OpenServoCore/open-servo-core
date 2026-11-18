//! The loops' stability margins on the bench servo of record's rig
//! ([`mg90_d`]), read the way a host reads them on the servo. The loop-margin
//! stimulus rides a triangle on the velocity loop's reference in
//! Velocity mode while a TEL burst streams under the bus dead-man; one row
//! a medium period carries `theta_hat_q16`, `omega_hat_cps` and
//! `omega_ref_cps`, and at the
//! triangle's frequency, with U, W and TH those rows' fundamentals over whole
//! cycles (TH through the period difference):
//!
//!   speed loop     L_v = W / (U - W)        (the loop closes on U - W)
//!   position loop  L_p = kp x TH / U        (Position mode closes kp x (theta* - TH) onto U)
//!
//! A loop's phase margin is 180 + arg L where |L| crosses 1, its gain margin
//! -|L| dB where arg L reaches -180: on this rig the speed loop's phase gets
//! there at Nyquist, a triangle that turns every period. The cruise keeps
//! the rotor above the speed where the shunt's window opens through every
//! burst, so the current loop reads every window, friction holds one side
//! and the lash stays closed. The truth is the linear model of the plant
//! and the kernel's loops at their ticks ([`model`]).

use std::f64::consts::{PI, TAU};

use osc_integration::mg90::{self, Motor, TAU_E};
use osc_integration::mg90_d;
use osc_integration::plant::{FakeIo, FakeMotor, FakeSensors};
use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::{ControlTable, Kernel, Mode, RegionStorage, TelStream};

/// A burst streams for the whole run.
struct Burst;
impl TelStream for Burst {
    fn on_publish(&mut self, _: impl FnOnce() -> bool) {}
    fn streaming(&self) -> bool {
        true
    }
}

/// The cruise, c/s: high enough that the triangle and the loop's answer
/// to the pot's residual leave the rotor above [`window_open_cps`] through
/// every burst, which [`gains`] asserts.
const CRUISE: i32 = 742;
/// Where a burst up and a burst down start, linearized counts: 86 counts
/// inside the soft limits. The longest burst, 6400 periods at 1.25 x the
/// cruise, travels 2968 counts and stops 134 counts short of the far one.
const UP: f64 = 400.0;
const DOWN: f64 = 3474.0;
/// Periods between torque on and the first row: the run-up and the loop
/// settling on the triangle.
const SETTLE: usize = 400;
/// The triangle's slope, c/s per period, and the peak it may reach, c/s:
/// the slope bounds the current the triangle asks for, the peak the speed
/// it takes off the cruise.
const SLOPE_CPS: f64 = 15.0;
const PEAK_CPS: f64 = 150.0;

/// The rows of one burst, one per medium period: theta counts, omega c/s,
/// the reference c/s; and the plant's slowest rotor speed over them, c/s
/// in the cruise's direction.
struct Rows {
    theta: Vec<f64>,
    omega: Vec<f64>,
    u: Vec<f64>,
    slowest: f64,
}

#[derive(Copy, Clone)]
struct Plant {
    /// No pot noise, no table residual and the stored winding: the plant
    /// [`model`] describes.
    quiet: bool,
    seed: u64,
    /// The cruise's sign.
    up: bool,
    /// The cruise's speed, c/s.
    cruise: i32,
}

/// The slope for half-period `n`, Q8.8 c/s per period.
fn slope_q88(n: u16) -> u16 {
    (SLOPE_CPS.min(2.0 * PEAK_CPS / n as f64) * 256.0).floor() as u16
}

/// The rotor speed, c/s, above which the shunt window reads with the
/// rotor's own drag current alone: the window floor's volts on the rail,
/// less the plant's Coulomb drag across R, over Ke.
fn window_open_cps() -> f64 {
    mg90_d::rig(|_| {}).table.with(|t| {
        let m = &t.calib.motor;
        let floor =
            t.calib.sense.i_window_min_ticks as f64 / mg90::ARR as f64 * mg90_d::VBUS as f64;
        let r = m.r_q12 as f64 / 4096.0;
        (floor - mg90_d::DRAG_FC * r) / (m.ke_vpc_q as f64 / 4096.0)
    })
}

/// One burst of `periods` rows at half-period `n`, after the settle.
fn burst(plant: Plant, n: u16, periods: usize, cfg: impl FnOnce(&mut ControlTable)) -> Rows {
    let dir = if plant.up { 1 } else { -1 };
    let sh = mg90_d::rig(|t| {
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = dir * plant.cruise;
        t.control.lifecycle.bus_deadman_ticks = u16::MAX;
        t.control.inject.inject_accel_q88 = slope_q88(n);
        t.control.inject.inject_half_periods = n;
        cfg(t);
    });
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        Burst,
        mg90::TIMING,
    );
    let mut p = mg90_d::plant(if plant.up { UP } else { DOWN }, plant.seed);
    if plant.quiet {
        p.sigma_raw = 0.0;
        p.residual = &[];
        p.contact = None;
        p.tau_e = TAU_E;
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    let mut rows = Rows {
        theta: Vec::new(),
        omega: Vec::new(),
        u: Vec::new(),
        slowest: f64::MAX,
    };
    // the boot tick runs CONTROL: a period's rows are read after its
    // PUBLISH, on its last tick
    for period in 0..SETTLE + periods {
        for _ in 0..DECIM_MED {
            let f = p.step(k.io.motor.last);
            k.on_scan(f, &sh);
            if period >= SETTLE {
                rows.slowest = rows.slowest.min(p.omega * dir as f64);
            }
        }
        if period < SETTLE {
            continue;
        }
        sh.table.with(|t| {
            assert_eq!(t.telemetry.common.fault_flags, 0, "a fault under the burst");
            rows.theta
                .push(t.telemetry.estimates.theta_hat_q16 as f64 / 65536.0);
            rows.omega
                .push(t.telemetry.estimates.omega_hat_cps as f64 / 65536.0);
            rows.u
                .push(t.telemetry_ext.inject.omega_ref_cps as f64 / 65536.0);
        });
    }
    rows
}

#[derive(Copy, Clone, Debug)]
struct C(f64, f64);
impl C {
    fn mul(self, o: C) -> C {
        C(self.0 * o.0 - self.1 * o.1, self.0 * o.1 + self.1 * o.0)
    }
    fn div(self, o: C) -> C {
        let d = o.0 * o.0 + o.1 * o.1;
        C(
            (self.0 * o.0 + self.1 * o.1) / d,
            (self.1 * o.0 - self.0 * o.1) / d,
        )
    }
    fn sub(self, o: C) -> C {
        C(self.0 - o.0, self.1 - o.1)
    }
    fn conj(self) -> C {
        C(self.0, -self.1)
    }
    fn db(self) -> f64 {
        20.0 * self.0.hypot(self.1).log10()
    }
    /// Degrees in (-270, 90]: a loop's phase runs from -90 down.
    fn deg(self) -> f64 {
        let p = self.1.atan2(self.0).to_degrees();
        if p > 90.0 { p - 360.0 } else { p }
    }
}

/// The component of `x` at `k` cycles over the record.
fn dft(x: &[f64], k: f64) -> C {
    let m = x.len() as f64;
    x.iter().enumerate().fold(C(0.0, 0.0), |a, (n, v)| {
        let ph = TAU * k * n as f64 / m;
        C(a.0 + v * ph.cos(), a.1 - v * ph.sin())
    })
}

/// One burst's fundamentals of U, W and TH at its whole cycles.
fn fundamentals(r: &Rows, cycles: usize) -> (C, C, C) {
    let k = cycles as f64;
    let w = TAU * k / r.u.len() as f64;
    let mut d = vec![0.0; r.theta.len()];
    for j in 1..d.len() {
        d[j] = r.theta[j] - r.theta[j - 1];
    }
    d[0] = d[1];
    let th = dft(&d, k).div(C(1.0 - w.cos(), w.sin()));
    (dft(&r.u, k), dft(&r.omega, k), th)
}

/// The speed and position loop gains at half-period `n`, the bursts' cross
/// spectra summed: each burst a record of `periods`, one up and one down.
fn gains(plants: &[Plant], n: u16, periods: usize, kp: f64) -> (C, C) {
    gains_with(plants, n, periods, kp, |_| {})
}

/// [`gains`] with `cfg` over every burst's rig.
fn gains_with(
    plants: &[Plant],
    n: u16,
    periods: usize,
    kp: f64,
    cfg: impl Fn(&mut ControlTable),
) -> (C, C) {
    let cycles = periods / (2 * n as usize);
    let (mut uu, mut wu, mut eu, mut tu) = (0.0, C(0.0, 0.0), C(0.0, 0.0), C(0.0, 0.0));
    let open = window_open_cps();
    for &pl in plants {
        let r = burst(pl, n, cycles * 2 * n as usize, &cfg);
        assert!(
            r.slowest >= open,
            "half-period {n}: the rotor fell to {:.1} c/s, under the window's {open:.1}",
            r.slowest
        );
        let (u, w, th) = fundamentals(&r, cycles);
        uu += u.0 * u.0 + u.1 * u.1;
        let uc = u.conj();
        let wuc = w.mul(uc);
        wu = C(wu.0 + wuc.0, wu.1 + wuc.1);
        let e = u.sub(w).mul(uc);
        eu = C(eu.0 + e.0, eu.1 + e.1);
        let t = th.mul(uc);
        tu = C(tu.0 + t.0, tu.1 + t.1);
    }
    let lp = tu.div(C(uu, 0.0));
    (wu.div(eu), C(kp * lp.0, kp * lp.1))
}

/// The rig's loop, linear: the winding and the rotor on the FAST tick, the
/// current loop every tick on the tick's current ahead of the medium step,
/// its write driving half the period up to the next frame and the one
/// before it the other half, as the rig's does, the observer at tick 1 on
/// the shaft and that current, the velocity PI at tick 4. Friction is a
/// constant at a cruise, the Ke feed-forward rides the cruise: both leave
/// the small signal. Driven with `cos(w n)` on the velocity loop's error
/// (open) or on its reference (closed), returns (W / E, TH / U) at `w`
/// radians a period.
fn model(g: &Gains, w: f64) -> (C, C) {
    let a = 1.0 - (-1.0 / TAU_E).exp();
    let Motor { r, ke, b, fv, .. } = mg90_d::motor();
    let Gains {
        kpi,
        kii,
        kv,
        kvi,
        bi,
        l1,
        l2,
        l3,
        ..
    } = *g;
    let cycles = 40.0;
    let settle = 8000;
    let record = (cycles * TAU / w).round() as usize;
    let run = |closed: bool| {
        let (mut i, mut om, mut x, mut ic) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let (mut i_ref, mut iv, mut written) = (0.0f64, 0.0f64, 0.0f64);
        let (mut th, mut ow, mut td, mut w_hat) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let (mut us, mut ws, mut es, mut ths) = (vec![], vec![], vec![], vec![]);
        for n in 0..settle + record {
            for tick in 0..DECIM_MED {
                let i_meas = i;
                let e = i_ref - i_meas;
                let u = kpi * e + ic;
                let v = (std::mem::replace(&mut written, u) + u) / 2.0;
                ic += kii * e;
                if tick == 1 {
                    ow += bi * (i_meas - td);
                    th += ow / 2000.0;
                    let e = x - th;
                    th += l1 * e;
                    ow += l2 * e;
                    td -= l3 * e;
                    w_hat = ow;
                }
                if tick == 4 {
                    let u = (w * n as f64).cos();
                    let e = if closed { u - w_hat } else { u };
                    i_ref = kv * e + iv;
                    iv += kvi * e;
                    if n >= settle {
                        us.push(u);
                        ws.push(w_hat);
                        es.push(e);
                        ths.push(th);
                    }
                }
                i += (v / r - ke / r * om - i) * a;
                om += b * (i - fv * om);
                x += om / 20_000.0;
            }
        }
        let r = Rows {
            theta: ths,
            omega: ws,
            u: us,
            slowest: 0.0,
        };
        let (u, w, th) = fundamentals(&r, cycles as usize);
        (u, w, th, dft(&es, cycles))
    };
    let (_, w_o, _, e_o) = run(false);
    let (u_c, _, th_c, _) = run(true);
    (w_o.div(e_o), th_c.div(u_c))
}

/// (crossover Hz, phase margin deg) by log-frequency interpolation, the
/// points ascending in frequency.
fn crossover(pts: &[(f64, C)]) -> (f64, f64) {
    for p in pts.windows(2) {
        let ((fa, a), (fb, b)) = (p[0], p[1]);
        if a.db() >= 0.0 && b.db() < 0.0 {
            let s = a.db() / (a.db() - b.db());
            let f = (fa.ln() + s * (fb.ln() - fa.ln())).exp();
            return (f, 180.0 + a.deg() + s * (b.deg() - a.deg()));
        }
    }
    panic!("no crossover in the band");
}

/// Half-periods the speed loop's band reads, ascending frequency: 25 Hz to
/// 125 Hz around its crossover; the Nyquist point is its gain margin.
const SPEED: [u16; 8] = [40, 30, 25, 20, 15, 12, 10, 8];
/// The position loop's band: 2 Hz to 20 Hz around its crossover, from the
/// ceiling the kernel reads `inject_half_periods` at.
const POSITION: [u16; 8] = [512, 400, 300, 200, 150, 100, 70, 50];

struct Sweep {
    speed: Vec<(f64, C)>,
    position: Vec<(f64, C)>,
    /// -|L_v| at Nyquist, dB.
    gm_db: f64,
}

fn sweep(plants: &[Plant], speed_periods: usize, position_periods: usize, kp: f64) -> Sweep {
    let at = |n: u16, periods| (1000.0 / n as f64, gains(plants, n, periods, kp));
    Sweep {
        speed: SPEED
            .iter()
            .map(|&n| {
                let (f, (lv, _)) = at(n, speed_periods);
                (f, lv)
            })
            .collect(),
        position: POSITION
            .iter()
            .map(|&n| {
                let (f, (_, lp)) = at(n, position_periods);
                (f, lp)
            })
            .collect(),
        gm_db: -gains(plants, 1, speed_periods, kp).0.db(),
    }
}

/// The rig's gains in the model's units: per count, per c/s, per period or
/// per tick as the kernel applies them.
#[derive(Copy, Clone)]
struct Gains {
    kpi: f64,
    kii: f64,
    kv: f64,
    kvi: f64,
    bi: f64,
    l1: f64,
    l2: f64,
    l3: f64,
    kp: f64,
}

fn rig_gains() -> Gains {
    mg90_d::rig(|_| {}).table.with(|t| {
        let c = &t.config;
        Gains {
            kpi: c.loop_current.i_kp_q88 as f64 / 256.0,
            kii: c.loop_current.i_ki_q412 as f64 / 4096.0,
            kv: c.loop_velocity.v_kp_q88 as f64 / 256.0,
            kvi: c.loop_velocity.v_ki_q412 as f64 / 4096.0,
            bi: t.calib.motor.b_i_q313 as f64 / 8192.0,
            l1: c.fusion.l1_q016 as f64 / 65536.0,
            l2: c.fusion.l2_q88 as f64 / 256.0,
            l3: c.fusion.l3_q88 as f64 / 256.0,
            kp: c.loop_position.p_kp_q88 as f64 / 256.0,
        }
    })
}

/// On a quiet plant (no pot noise, no table residual, the stored winding)
/// the loop is the linear one, so the reading is the model's: every point
/// of both bands within 1 dB and 10 degrees, the margins within 5 degrees
/// and 1 dB, the crossovers within 10 %. On the rig's gains the model's
/// speed loop crosses near 57 Hz with about 43 degrees and 23 dB, its
/// position loop near 3.0 Hz with about 88 degrees.
#[test]
fn the_margins_read_on_a_quiet_mg90_are_the_loop_models() {
    let g = rig_gains();
    let kp = g.kp;
    let quiet = [Plant {
        quiet: true,
        seed: 1,
        up: true,
        cruise: CRUISE,
    }];
    let got = sweep(&quiet, 2000, 2000, kp);
    let model_at = |f: f64| model(&g, TAU * f / 2000.0);
    let speed: Vec<(f64, C)> = got.speed.iter().map(|&(f, _)| (f, model_at(f).0)).collect();
    let position: Vec<(f64, C)> = got
        .position
        .iter()
        .map(|&(f, _)| {
            let tt = model_at(f).1;
            (f, C(kp * tt.0, kp * tt.1))
        })
        .collect();
    for (band, want) in [(&got.speed, &speed), (&got.position, &position)] {
        for (&(f, g), &(_, m)) in band.iter().zip(want.iter()) {
            assert!(
                (g.db() - m.db()).abs() <= 1.0 && (g.deg() - m.deg()).abs() <= 10.0,
                "{f:.1} Hz: read {:.2} dB {:.1} deg, model {:.2} dB {:.1} deg",
                g.db(),
                g.deg(),
                m.db(),
                m.deg()
            );
        }
    }
    let gm_model = -model(&g, PI).0.db();
    for ((f, pm), (mf, mpm)) in [
        (crossover(&got.speed), crossover(&speed)),
        (crossover(&got.position), crossover(&position)),
    ] {
        assert!(
            (pm - mpm).abs() <= 5.0 && (f / mf - 1.0).abs() <= 0.1,
            "read {f:.1} Hz {pm:.1} deg, model {mf:.1} Hz {mpm:.1} deg"
        );
    }
    assert!(
        (got.gm_db - gm_model).abs() <= 1.0,
        "gain margin read {:.2} dB, model {gm_model:.2} dB",
        got.gm_db
    );
}

/// Under the bench's pot noise and table residual two sweeps, each a burst
/// up and a burst down per point, repeat within 1 dB and 10 degrees at every
/// point of both bands and their phase margins within 5 degrees: the bench
/// gate's repeatability, on the plant the sim knows. The gain margin decides
/// a 6 dB acceptance floor, so the two agree within 1 dB or both read at
/// least 12 dB, twice the floor, where their scatter cannot change the
/// verdict.
#[test]
fn two_sweeps_on_the_bench_mg90_repeat_within_the_bounds() {
    let kp = rig_gains().kp;
    let pair = |a: u64| bench_pair(a, CRUISE);
    let (one, two) = (
        sweep(&pair(1), 2000, 6000, kp),
        sweep(&pair(3), 2000, 6000, kp),
    );
    for (a, b) in [(&one.speed, &two.speed), (&one.position, &two.position)] {
        for (&(f, x), &(_, y)) in a.iter().zip(b.iter()) {
            assert!(
                (x.db() - y.db()).abs() <= 1.0 && (x.deg() - y.deg()).abs() <= 10.0,
                "{f:.1} Hz: {:.2} dB {:.1} deg against {:.2} dB {:.1} deg",
                x.db(),
                x.deg(),
                y.db(),
                y.deg()
            );
        }
    }
    for (x, y) in [
        (crossover(&one.speed), crossover(&two.speed)),
        (crossover(&one.position), crossover(&two.position)),
    ] {
        assert!(
            (x.1 - y.1).abs() <= 5.0,
            "phase margins {x:?} against {y:?}"
        );
    }
    assert!(
        (one.gm_db - two.gm_db).abs() <= 1.0 || one.gm_db.min(two.gm_db) >= 12.0,
        "gain margins {:.2} dB against {:.2} dB",
        one.gm_db,
        two.gm_db
    );
}

/// A burst up and a burst down at `cruise` under the bench's pot noise and
/// table residual, seeds `a` and `a + 1`.
fn bench_pair(a: u64, cruise: i32) -> [Plant; 2] {
    [true, false].map(|up| Plant {
        quiet: false,
        seed: a + !up as u64,
        up,
        cruise,
    })
}

/// The injection rides only a streaming Velocity loop: the reference is the
/// cruise plus the triangle, peak slope x half / 2, inside the speed
/// ceiling; with no dead-man loaded it is the cruise alone.
#[test]
fn the_triangle_rides_the_velocity_reference_under_the_deadman() {
    let n = 20;
    let plant = Plant {
        quiet: true,
        seed: 1,
        up: true,
        cruise: CRUISE,
    };
    let r = burst(plant, n, 400, |_| {});
    let peak = slope_q88(n) as f64 / 256.0 * (n / 2) as f64;
    let d: Vec<f64> = r.u.iter().map(|u| u - CRUISE as f64).collect();
    let hi = d.iter().cloned().fold(f64::MIN, f64::max);
    let lo = d.iter().cloned().fold(f64::MAX, f64::min);
    assert_eq!((hi, lo), (peak, -peak));
    for w in d.windows(2) {
        assert_eq!((w[1] - w[0]).abs(), slope_q88(n) as f64 / 256.0);
    }
    let off = burst(plant, n, 400, |t| t.control.lifecycle.bus_deadman_ticks = 0);
    assert!(off.u.iter().all(|&u| u == CRUISE as f64));
    let capped = burst(plant, n, 400, |t| {
        t.config.loop_position.speed_ceiling_cps = CRUISE as u16 + 50
    });
    assert!(capped.u.iter().all(|&u| u <= CRUISE as f64 + 50.0));
}

/// The slow cruise's floor, degrees: the rig's stock loop reads 61 there,
/// where the contact term carries no weight.
const SLOW_FLOOR_DEG: f64 = 58.0;
/// The share of the current limit a large move's braking stays under. The
/// rig's stock moves peak at 0.54 to 0.72 of it; the bench broke into a
/// swing at the limit where a raised resistance braked into it.
const BRAKE_SHARE: f64 = 0.8;
/// The bench's large moves: 120 degrees between these goals, 1200 ms a
/// goal, at the set's move speed.
const MOVE_GOALS: [i32; 2] = [3124, 708];
const MOVE_TICKS: usize = 24_000;
/// The acceleration a move's peak is read after, FAST ticks: 75 ms.
const MOVE_RUN_UP: usize = 1500;

/// The largest share of the current limit the winding reaches over each
/// move after its run-up, twelve moves from the soft range's middle.
fn move_peaks(seed: u64) -> Vec<f64> {
    let center = 1937;
    let sh = mg90_d::rig(|t| {
        t.control.lifecycle.mode = Mode::Position;
        t.control.lifecycle.goal_position = center;
    });
    let mut k = mg90::kernel();
    let mut p = mg90_d::plant(center as f64, seed);
    let limit = sh.table.with(|t| t.config.limits.current_limit_counts) as f64;
    let tick = |k: &mut Kernel<FakeIo>, p: &mut mg90::Mg90| {
        let f = p.step(k.io.motor.last);
        k.on_scan(f, &sh);
    };
    for _ in 0..SETTLE {
        tick(&mut k, &mut p);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    let mut peaks = Vec::new();
    for m in 0..12 {
        let goal = MOVE_GOALS[m % 2];
        sh.table
            .with_mut(|t| t.control.lifecycle.goal_position = goal);
        let mut peak = 0.0f64;
        for n in 0..MOVE_TICKS {
            tick(&mut k, &mut p);
            if n >= MOVE_RUN_UP {
                peak = peak.max(p.i.abs() / limit);
            }
        }
        peaks.push(peak);
    }
    assert_eq!(
        sh.table.with(|t| t.telemetry.common.fault_flags),
        0,
        "a fault over the moves"
    );
    peaks
}

/// On the set of record the speed loop at the slow cruise keeps
/// [`SLOW_FLOOR_DEG`] of phase margin, and no large move's braking takes
/// the winding past [`BRAKE_SHARE`] of the current limit. A resistance
/// raised by the identified contact slope takes the braking to the limit
/// (the bench swung there on every move into 3124).
#[test]
fn the_record_rig_keeps_its_slow_cruise_margin_and_its_moves_off_the_limit() {
    let kp = rig_gains().kp;
    let (f, pm) = crossover(&sweep(&bench_pair(1, CRUISE), 2000, 2000, kp).speed);
    assert!(
        pm >= SLOW_FLOOR_DEG,
        "slow cruise: {f:.1} Hz {pm:.1} deg, under {SLOW_FLOOR_DEG}"
    );
    for seed in [1, 2] {
        for (m, peak) in move_peaks(seed).into_iter().enumerate() {
            assert!(
                peak < BRAKE_SHARE,
                "seed {seed} move {m}: braking peaked at {peak:.2} of the limit"
            );
        }
    }
}
