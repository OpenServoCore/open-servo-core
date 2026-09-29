//! The travel a free-running rung needs, and where it must brake. A rung of
//! duty d starts from rest in the start band at one end of the travel guard
//! and runs toward the other: it climbs to speed on the current limit,
//! settles, collects its steady windows, and brakes early enough that its
//! stop lands inside the guard with a quarter of a stop to spare.
//!
//! ```text
//! need(d) = climb(d) + settle(d) + steady(d) + 1.25 x stop(d)   <=   room
//! room    = guard_hi - guard_lo - START_BAND
//! climb   = v^2 / 2a            settle + steady = v x the windows' time
//! ```
//!
//! Speed comes from a pilot envelope when it describes this servo on this
//! supply ([`Envelope::check`]); otherwise the run sizes itself from what it
//! measured: the line through the last two measured speeds (one point
//! scales in proportion to duty). The stop is always the run's own: the
//! last braked stop scaled by speed squared (in proportion below it) - an
//! envelope's coasts only stand in until the run has braked once. The climb
//! is the acceleration of the last governed climb. Positions are raw pot
//! counts, speeds counts/ms, accelerations counts/ms per s, times ms.
//! Nothing here reads a clock or a file: the envelope arrives as data.

use core::fmt;

use crate::exp::seek::STOP_TOL;

/// Width of the band at the start end a seek comes to rest in, counts: a
/// rung starts at most this far inside the guard.
pub const START_BAND: u16 = 75;

/// A rung brakes this many predicted stops before the guard.
pub const STOP_MARGIN: f64 = 1.25;

/// Governed-climb acceleration before any climb was measured, counts/ms per
/// s: a third of the bench MG90's at its 280-count limit. A heavier load or
/// a lower limit climbs slower, and a first rung sized long only costs room.
pub const ACCEL_PRIOR: f64 = 40.0;

/// The brake idiom: under slow decay a token duty against the motion, 3 PWM
/// ticks at ARR 1200, holds the idle leg high for the rest of each period,
/// so the winding shorts through the bridge and the rotor stops in a few
/// hundred counts. A duty of 0 coasts. Pointed away from the wall the drive
/// approached, the soft-limit clamp can never zero it.
pub const BRAKE_DUTY_Q15: i16 = 82;
/// The brake holds until two polls `BRAKE_POLL_MS` apart differ by less
/// than this, counts, or for `BRAKE_POLLS` polls.
pub const BRAKE_REST_EPS: u16 = 4;
pub const BRAKE_POLL_MS: u32 = 20;
pub const BRAKE_POLLS: u32 = 50;

/// What powers the servo. A pilot envelope holds for the supply it was
/// measured on.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Supply {
    Usb,
    TwoS,
}

impl Supply {
    /// The supply a rail of `rail_mv` comes from: USB is 4.0 to 5.5 V, a
    /// 2S pack 6.0 to 8.7 V. None for anything else.
    pub fn of_rail(rail_mv: f64) -> Option<Self> {
        if (4000.0..=5500.0).contains(&rail_mv) {
            Some(Supply::Usb)
        } else if (6000.0..=8700.0).contains(&rail_mv) {
            Some(Supply::TwoS)
        } else {
            None
        }
    }
}

impl fmt::Display for Supply {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Supply::Usb => "USB",
            Supply::TwoS => "2S",
        })
    }
}

/// Steady speed as a line in duty: counts/ms = slope x duty percent +
/// intercept.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Line {
    pub slope: f64,
    pub intercept: f64,
}

impl Line {
    pub fn at(&self, pct: f64) -> f64 {
        self.slope * pct + self.intercept
    }
}

/// What `osc capture pilot` measured on a servo, as the runway reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct Envelope {
    pub supply: Supply,
    /// The physical stops the pilot read, raw counts.
    pub phys: (u16, u16),
    /// Steady speed driving up and down the pot.
    pub fwd: Line,
    pub rev: Line,
    /// Every coast the pilot measured: (entry speed, coast distance).
    pub coast: Vec<(f64, f64)>,
}

/// Why an envelope does not describe the servo in front of the run.
#[derive(Clone, Debug, PartialEq)]
pub enum Stale {
    Supply {
        envelope: Supply,
        run: Option<Supply>,
    },
    Stops {
        envelope: (u16, u16),
        servo: (i32, i32),
    },
    NoCoast,
}

impl fmt::Display for Stale {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stale::Supply { envelope, run } => match run {
                Some(run) => write!(f, "it was measured on {envelope} and this run is on {run}"),
                None => write!(
                    f,
                    "it was measured on {envelope} and this rail is neither USB nor 2S"
                ),
            },
            Stale::Stops { envelope, servo } => write!(
                f,
                "its stops {}..{} are more than {STOP_TOL} counts from the servo's {}..{}",
                envelope.0, envelope.1, servo.0, servo.1
            ),
            Stale::NoCoast => f.write_str("it holds no coast to size a stop by"),
        }
    }
}

impl Envelope {
    /// The envelope describes a servo on `run`'s supply whose physical
    /// stops are `phys`: same supply, both stops within [`STOP_TOL`].
    pub fn check(&self, run: Option<Supply>, phys: (i32, i32)) -> Result<(), Stale> {
        if run != Some(self.supply) {
            return Err(Stale::Supply {
                envelope: self.supply,
                run,
            });
        }
        let near = |a: u16, b: i32| (a as i32 - b).unsigned_abs() <= STOP_TOL as u32;
        if !near(self.phys.0, phys.0) || !near(self.phys.1, phys.1) {
            return Err(Stale::Stops {
                envelope: self.phys,
                servo: phys,
            });
        }
        if self.coast.is_empty() {
            return Err(Stale::NoCoast);
        }
        Ok(())
    }

    /// Steady speed of a drive of sign `dir` at `duty`, a fraction of full
    /// scale.
    pub fn speed(&self, dir: i8, duty: f64) -> f64 {
        let line = if dir > 0 { self.fwd } else { self.rev };
        line.at(duty.abs() * 100.0).max(0.0)
    }

    /// Coast distance from `v`: the `a v + b v^2` law over every coast
    /// measured, else the measured coasts scaled by speed squared from the
    /// fastest one at or under `v` - distance over v^2 falls with speed, so
    /// that over-predicts.
    pub fn coast(&self, v: f64) -> f64 {
        if let Some((a, b)) = coast_fit(&self.coast) {
            return a * v + b * v * v;
        }
        let under = self
            .coast
            .iter()
            .filter(|p| p.0 > 0.0 && p.0 <= v)
            .max_by(|x, y| x.0.total_cmp(&y.0));
        match under {
            Some(&(v0, d0)) => d0 * (v / v0) * (v / v0),
            None => self
                .coast
                .iter()
                .min_by(|x, y| x.0.total_cmp(&y.0))
                .map_or(0.0, |p| p.1),
        }
    }
}

/// Least-squares `coast = a v + b v^2` over (entry, coast) points; None when
/// the points cannot separate the terms or either comes out negative, which
/// no friction law gives.
pub fn coast_fit(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    let (s2, s3, s4, t1, t2) = pts.iter().fold(
        (0.0, 0.0, 0.0, 0.0, 0.0),
        |(s2, s3, s4, t1, t2), &(v, d)| {
            let v2 = v * v;
            (s2 + v2, s3 + v2 * v, s4 + v2 * v2, t1 + v * d, t2 + v2 * d)
        },
    );
    let det = s2 * s4 - s3 * s3;
    if det <= 0.0 {
        return None;
    }
    let a = (t1 * s4 - t2 * s3) / det;
    let b = (s2 * t2 - s3 * t1) / det;
    (a >= 0.0 && b >= 0.0).then_some((a, b))
}

/// The travel one rung needs, counts, and its predicted climb time, ms.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Need {
    /// The steady speed it was sized for, counts/ms.
    pub v: f64,
    pub climb: f64,
    pub climb_ms: f64,
    /// Settle and steady windows at speed, and any stretch of that run
    /// whose windows the fit masks.
    pub run: f64,
    pub stop: f64,
}

impl Need {
    pub fn total(&self) -> f64 {
        self.climb + self.run + STOP_MARGIN * self.stop
    }
}

/// A rung reaching `v` on a climb of `accel`, then running `steady_ms` at
/// speed, then stopping in `stop`.
pub fn need(v: f64, accel: f64, steady_ms: f64, stop: f64) -> Need {
    let (climb, climb_ms) = if accel > 0.0 {
        (v * v / (2.0 * accel) * 1000.0, v / accel * 1000.0)
    } else {
        (f64::INFINITY, f64::INFINITY)
    };
    Need {
        v,
        climb,
        climb_ms,
        run: v * steady_ms,
        stop,
    }
}

pub fn fits(need: &Need, room: f64) -> bool {
    need.total() <= room
}

/// The runway of one run: the guard, the envelope when one holds, and what
/// the run measured so far.
#[derive(Clone, Debug)]
pub struct Runway {
    guard: (u16, u16),
    envelope: Option<Envelope>,
    accel: f64,
    /// (duty, steady speed) measured, oldest first.
    speeds: Vec<(f64, f64)>,
    /// The last braked stop: (speed at the brake, distance).
    stop: Option<(f64, f64)>,
}

impl Runway {
    pub fn new(guard: (u16, u16)) -> Self {
        Self {
            guard,
            envelope: None,
            accel: ACCEL_PRIOR,
            speeds: Vec::new(),
            stop: None,
        }
    }

    /// A known plant's governed climb in place of [`ACCEL_PRIOR`].
    pub fn with_accel(self, accel: f64) -> Self {
        Self { accel, ..self }
    }

    /// Size by `env` when it describes this servo: the run's supply and the
    /// servo's physical stops. A stale envelope is left out.
    pub fn size_by(
        &mut self,
        env: Envelope,
        run: Option<Supply>,
        phys: (i32, i32),
    ) -> Result<(), Stale> {
        env.check(run, phys)?;
        self.envelope = Some(env);
        Ok(())
    }

    pub fn envelope(&self) -> Option<&Envelope> {
        self.envelope.as_ref()
    }

    pub fn accel(&self) -> f64 {
        self.accel
    }

    pub fn guard(&self) -> (u16, u16) {
        self.guard
    }

    pub fn room(&self) -> f64 {
        self.guard
            .1
            .saturating_sub(self.guard.0)
            .saturating_sub(START_BAND) as f64
    }

    /// The start band's inner edge for a rung of sign `dir`: its seek has
    /// arrived once it reaches this.
    pub fn start(&self, dir: i8) -> u16 {
        if dir > 0 {
            self.guard.0.saturating_add(START_BAND)
        } else {
            self.guard.1.saturating_sub(START_BAND)
        }
    }

    /// Where a rung of sign `dir` stopping in `stop` counts starts to brake.
    pub fn brake_at(&self, dir: i8, stop: f64) -> f64 {
        if dir > 0 {
            self.guard.1 as f64 - STOP_MARGIN * stop
        } else {
            self.guard.0 as f64 + STOP_MARGIN * stop
        }
    }

    /// Steady speed of a rung of sign `dir` at `duty`: the envelope's, else
    /// the line through the last two measured duties - one scales in
    /// proportion - never under a speed measured at or under `duty`.
    pub fn speed(&self, dir: i8, duty: f64) -> Option<f64> {
        let duty = duty.abs();
        if let Some(env) = &self.envelope {
            return Some(env.speed(dir, duty));
        }
        let &(d1, v1) = self.speeds.last()?;
        let line = match self.speeds.iter().rev().find(|p| (p.0 - d1).abs() > 1e-9) {
            Some(&(d0, v0)) => v1 + (v1 - v0) / (d1 - d0) * (duty - d1),
            None => v1 * duty / d1,
        };
        let floor = self
            .speeds
            .iter()
            .filter(|p| p.0 <= duty + 1e-9)
            .fold(0.0, |m: f64, p| m.max(p.1));
        Some(line.max(floor))
    }

    /// The stop from `v`: the last braked stop scaled by speed squared - in
    /// proportion under the speed it was braked from. A braked stop grows
    /// faster than speed but slower than its square (the winding's brake is
    /// viscous, friction a constant), so each scaling over-predicts on its
    /// side. Before the run has braked, the envelope's coast: three to five
    /// times the braked stop, so it only brakes early.
    pub fn stop(&self, v: f64) -> Option<f64> {
        let Some((v0, d0)) = self.stop else {
            return self.envelope.as_ref().map(|env| env.coast(v));
        };
        let k = if v0 > 0.0 { v / v0 } else { 1.0 };
        Some(d0 * k * k.max(1.0))
    }

    /// The need of a rung of sign `dir` at `duty` whose settle and steady
    /// windows take `steady_ms`; None until a speed and a stop are known.
    pub fn plan(&self, dir: i8, duty: f64, steady_ms: f64) -> Option<Need> {
        let v = self.speed(dir, duty)?;
        Some(need(v, self.accel, steady_ms, self.stop(v)?))
    }

    /// A drive at `duty` settled at `v`.
    pub fn ran(&mut self, duty: f64, v: f64) {
        if v > 0.0 && duty != 0.0 {
            self.speeds.push((duty.abs(), v));
        }
    }

    /// A governed climb averaged `accel`.
    pub fn climbed(&mut self, accel: f64) {
        if accel.is_finite() && accel > 0.0 {
            self.accel = accel;
        }
    }

    /// A brake from `v` stopped in `dist`.
    pub fn stopped(&mut self, v: f64, dist: f64) {
        if v > 0.0 && dist.is_finite() {
            self.stop = Some((v, dist.max(0.0)));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench guard: soft limits 432..3626 inset 100.
    const GUARD: (u16, u16) = (532, 3526);

    /// The bench MG90 on 2S as the pilot measured it: its v_ss fits and the
    /// five spindown coasts, alike both ways.
    fn mg90_2s() -> Envelope {
        let coast = [
            (3.16, 130.0),
            (5.62, 316.0),
            (7.55, 536.0),
            (11.67, 979.0),
            (13.15, 1333.0),
        ];
        Envelope {
            supply: Supply::TwoS,
            phys: (232, 3849),
            fwd: Line {
                slope: 0.2047,
                intercept: -0.721,
            },
            rev: Line {
                slope: 0.2079,
                intercept: -0.831,
            },
            coast: coast.iter().chain(coast.iter()).copied().collect(),
        }
    }

    /// Settle 5 and 12 steady windows over the untrimmed 70%, 0.8 ms each.
    const STEADY_MS: f64 = (5.0 + 12.0 / 0.7) * 0.8;

    /// Governed climb of the known bench plant: b x (0.9 x 280 - fc).
    const BENCH_ACCEL: f64 = 118.0;

    #[test]
    fn room_is_the_guard_less_the_start_band() {
        let r = Runway::new(GUARD);
        assert_eq!(r.room(), 2919.0);
        assert_eq!((r.start(1), r.start(-1)), (607, 3451));
        assert_eq!(r.brake_at(1, 100.0), 3401.0);
        assert_eq!(r.brake_at(-1, 100.0), 657.0);
    }

    /// The bench servo on 2S, limit 280, 4.9 ohm: the default rungs to 64%
    /// fit, 100% does not. Need per rung (both ways within a few counts):
    /// 26% 457, 40% 992, 64% 2350, 80% 3564, 100% 5431 counts against 2919.
    #[test]
    fn bench_mg90_runway_fits_the_rungs_to_64_and_not_100() {
        let mut r = Runway::new(GUARD).with_accel(BENCH_ACCEL);
        r.size_by(mg90_2s(), Supply::of_rail(7900.0), (232, 3849))
            .unwrap();
        let room = r.room();
        for (pct, want, fit) in [
            (26, 457.0, true),
            (40, 992.0, true),
            (64, 2350.0, true),
            (80, 3564.0, false),
            (100, 5431.0, false),
        ] {
            let n = r.plan(-1, pct as f64 / 100.0, STEADY_MS).unwrap();
            assert!(
                (n.total() - want).abs() < 0.01 * want,
                "{pct}%: need {:.0} (climb {:.0}, run {:.0}, stop {:.0})",
                n.total(),
                n.climb,
                n.run,
                n.stop
            );
            assert_eq!(fits(&n, room), fit, "{pct}%");
            let up = r.plan(1, pct as f64 / 100.0, STEADY_MS).unwrap();
            assert_eq!(fits(&up, room), fit, "{pct}% up");
        }
    }

    #[test]
    fn stale_envelope_is_ignored() {
        let servo = (232, 3849);
        let mut r = Runway::new(GUARD);
        assert_eq!(
            r.size_by(mg90_2s(), Some(Supply::Usb), servo),
            Err(Stale::Supply {
                envelope: Supply::TwoS,
                run: Some(Supply::Usb)
            })
        );
        let err = r.size_by(mg90_2s(), None, servo).unwrap_err();
        assert_eq!(
            err.to_string(),
            "it was measured on 2S and this rail is neither USB nor 2S"
        );
        let moved = (232 + 151, 3849);
        let err = r.size_by(mg90_2s(), Some(Supply::TwoS), moved).unwrap_err();
        assert_eq!(
            err.to_string(),
            "its stops 232..3849 are more than 150 counts from the servo's 383..3849"
        );
        let bare = Envelope {
            coast: Vec::new(),
            ..mg90_2s()
        };
        assert_eq!(
            r.size_by(bare, Some(Supply::TwoS), servo),
            Err(Stale::NoCoast)
        );
        assert!(r.envelope().is_none(), "a stale envelope sizes nothing");
        assert_eq!(r.speed(1, 0.4), None);

        // within the tolerance it holds
        r.size_by(mg90_2s(), Some(Supply::TwoS), (232 + 150, 3849 - 150))
            .unwrap();
        assert!((r.speed(-1, 0.4).unwrap() - 7.485).abs() < 1e-9);
        assert_eq!(Supply::of_rail(4390.0), Some(Supply::Usb));
        assert_eq!(Supply::of_rail(7900.0), Some(Supply::TwoS));
        assert_eq!(Supply::of_rail(12600.0), None);
    }

    #[test]
    fn self_sizing_follows_the_measured_rungs() {
        let mut r = Runway::new(GUARD);
        assert_eq!(r.plan(1, 0.26, STEADY_MS), None, "nothing measured");
        // the seek: 2 counts/ms at 15%, braked in 40 counts
        r.ran(0.15, 2.0);
        r.stopped(2.0, 40.0);
        let v = r.speed(1, 0.30).unwrap();
        assert!((v - 4.0).abs() < 1e-9, "one point scales in proportion");
        assert!((r.stop(4.0).unwrap() - 160.0).abs() < 1e-9);
        assert!(
            (r.stop(1.0).unwrap() - 20.0).abs() < 1e-9,
            "under it: linear"
        );
        let n = r.plan(1, 0.30, STEADY_MS).unwrap();
        assert!((n.climb - 200.0).abs() < 1e-9, "v^2 / 2 x the prior");
        assert!((n.climb_ms - 100.0).abs() < 1e-9);
        // two duties: the line through them
        r.ran(0.30, 5.0);
        r.climbed(100.0);
        assert!((r.speed(-1, 0.45).unwrap() - 8.0).abs() < 1e-9);
        assert!((r.plan(1, 0.45, 0.0).unwrap().climb - 320.0).abs() < 1e-9);
        // a falling line never predicts under a speed already measured
        r.ran(0.40, 4.0);
        assert_eq!(r.speed(1, 0.45), Some(5.0));
        // junk measurements are ignored
        r.climbed(f64::NAN);
        r.ran(0.5, 0.0);
        assert_eq!(r.accel(), 100.0);
        assert_eq!(r.speeds.len(), 3);
    }

    #[test]
    fn coast_falls_back_to_the_measured_runs() {
        let one = Envelope {
            coast: vec![(4.0, 200.0)],
            ..mg90_2s()
        };
        assert_eq!(coast_fit(&one.coast), None);
        assert!((one.coast(8.0) - 800.0).abs() < 1e-9);
        assert_eq!(one.coast(2.0), 200.0, "under the slowest: its distance");
        let (a, b) = coast_fit(&mg90_2s().coast).unwrap();
        assert!(
            (a - 24.94).abs() < 0.01 && (b - 5.557).abs() < 0.001,
            "{a} {b}"
        );
    }

    #[test]
    fn need_adds_climb_run_and_a_margined_stop() {
        let n = need(10.0, 100.0, 20.0, 200.0);
        assert_eq!((n.climb, n.climb_ms, n.run), (500.0, 100.0, 200.0));
        assert_eq!(n.total(), 950.0);
        assert!(fits(&n, 950.0) && !fits(&n, 949.0));
        assert!(!fits(&need(10.0, 0.0, 20.0, 200.0), 1e9));
    }
}
