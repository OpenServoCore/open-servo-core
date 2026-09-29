//! The servo's own limits, as a drive plans against them: the current limit
//! and the stall settings from CONFIG, the travel from the soft limits, and
//! the winding R and the rail that turn a duty into a stall current. The
//! host reads the fields; everything decided from them lives here, so the
//! CLI and the GUI refuse the same drives in the same words.

use core::fmt;

use crate::regs::{Reg, control};

/// How often a pump rewrites a held stall permit, ms. The firmware grants
/// about a second per write, so a missed rewrite or two still leaves the
/// permit standing.
pub const PERMIT_REFRESH_MS: u32 = 250;

/// The longest TEL stream a held permit may ride, ms: nothing is rewritten
/// while a stream runs, and the refresh before it may already be due.
pub const PERMIT_STREAM_MAX_MS: u32 = 750;

/// Full scale of the 12-bit pot ADC; soft limits spanning it are the board
/// default, not a calibration.
pub const POT_MAX: i32 = 4095;

/// Soft-to-guard inset, counts. A sweep's seek band spans 75 counts either
/// side of a guard, so the guard sits 25 counts more than that inside soft,
/// or the band reaches the firmware's soft-limit clamp and the seek fights
/// it.
pub const GUARD_INSET: u16 = 100;

const Q15: f64 = 32767.0;

/// The lowest winding R the servo class carries, ohms. A stall at
/// `i_lim x CLASS_R_MIN / vbus` draws at most the limit on any servo of the
/// class, so a drive at that duty is safe before anything measured R - and,
/// for the class on 2S, it sits under the shunt's window floor, where the
/// firmware limiter is blind.
pub const CLASS_R_MIN: f64 = 3.0;

/// A duty fraction as q15, rounded down: a planned duty never stalls over
/// what it was planned for.
pub fn q15_floor(duty: f64) -> i16 {
    (duty.clamp(0.0, 1.0) * Q15 + 1e-6).floor() as i16
}

/// The same as whole percent, rounded down.
pub fn pct_floor(duty: f64) -> u8 {
    (duty.clamp(0.0, 1.0) * 100.0 + 1e-9).floor() as u8
}

/// The travel guard for a servo's soft limits: each end `GUARD_INSET`
/// counts inside.
pub fn guards(soft: (u16, u16)) -> (u16, u16) {
    (
        soft.0.saturating_add(GUARD_INSET),
        soft.1.saturating_sub(GUARD_INSET),
    )
}

/// Soft limits spanning the whole pot: the board default, or limits a
/// widened run never put back.
pub fn is_board_default(soft: (i32, i32)) -> bool {
    soft.1 - soft.0 >= POT_MAX
}

/// Duty, as a fraction of full scale, whose stall current is `i_counts`:
/// a stalled winding passes duty x vbus over R. `r_vpc` is vcounts per
/// ccount, `vbus` vcounts.
pub fn duty_for(i_counts: f64, r_vpc: f64, vbus: f64) -> f64 {
    if vbus <= 0.0 {
        return 0.0;
    }
    (i_counts * r_vpc / vbus).clamp(0.0, 1.0)
}

/// The current, counts, a duty fraction draws with the shaft stalled.
pub fn stall_counts(duty: f64, r_vpc: f64, vbus: f64) -> f64 {
    if r_vpc <= 0.0 {
        return f64::INFINITY;
    }
    duty.abs() * vbus / r_vpc
}

/// What the servo allows, read from its table.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ServoLimits {
    /// `current_limit_counts`.
    pub i_lim: u16,
    /// `stall_yield_counts`: the limit a stall folds back to.
    pub stall_yield: u16,
    /// `stall_tau_trip_counts`: the collision trip.
    pub tau_trip: u16,
    pub soft: (i32, i32),
    pub phys: (i32, i32),
    /// The pot stops, `raw_min` and `raw_max`.
    pub raw: (u16, u16),
    /// Winding R, vcounts per ccount in Q12; 0 until identified.
    pub r_q12: u16,
    /// The rail, vcounts.
    pub vbus: u16,
    /// `window_floor_q15`: the smallest duty whose drive window the shunt
    /// reads, as the servo publishes it; 0 when it publishes none.
    pub window_floor_q15: u16,
    /// Shunt scale; 0 when the sense constants are unset.
    pub amps_per_count: f64,
}

/// The travel guard and the current abort a drive runs inside.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Envelope {
    pub guard: (u16, u16),
    pub i_abort: i16,
}

/// Why a drive is refused before anything moves.
#[derive(Clone, Debug, PartialEq)]
pub enum Refusal {
    /// The soft limits are not a calibrated range.
    Uncalibrated { soft: (i32, i32) },
    /// An abort threshold above the default, a quarter over the limit.
    AbortOverLimit {
        i_abort: i16,
        max: i16,
        i_lim: u16,
        ma: Ma,
    },
    /// A deliberate stall that would draw more than the current limit.
    StallOverLimit {
        what: String,
        duty: f64,
        stall: f64,
        i_lim: u16,
        allowed: f64,
        ma: Ma,
    },
    /// A TEL stream too long to hold the stall permit through.
    StreamOverLease { ms: u32 },
    /// The servo publishes no window floor.
    NoWindowFloor,
    /// A ladder of stall dwells, `what`, finds too few readable rungs or
    /// too little current span between the window floor and the
    /// stall-safe cap.
    NoLadderRoom {
        what: &'static str,
        floor: f64,
        cap: f64,
    },
    /// The duty that first moved the shaft stalls over the current limit:
    /// `need` is that stall current, counts.
    BreakawayOverLimit { need: f64, i_lim: u16, ma: Ma },
}

/// Counts to milliamps for a message; says nothing when the scale is
/// unknown.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Ma(pub f64);

impl Ma {
    pub fn of(&self, counts: f64) -> String {
        if self.0 > 0.0 {
            format!("{counts:.0} counts ({:.0} mA)", counts * self.0 * 1000.0)
        } else {
            format!("{counts:.0} counts")
        }
    }
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Refusal::Uncalibrated { soft } => write!(
                f,
                "the servo's soft position limits {}..{} are not a calibrated range: run \
                 `osc cal` first",
                soft.0, soft.1
            ),
            Refusal::AbortOverLimit {
                i_abort,
                max,
                i_lim,
                ma,
            } => write!(
                f,
                "an abort threshold of {} is over {}, a quarter above this servo's current limit \
                 of {}: the run could draw more current than the servo is set to allow (leave \
                 --i-abort out to abort there)",
                ma.of(*i_abort as f64),
                ma.of(*max as f64),
                ma.of(*i_lim as f64)
            ),
            Refusal::StallOverLimit {
                what,
                duty,
                stall,
                i_lim,
                allowed,
                ma,
            } => write!(
                f,
                "{what} stalls the motor at {:.0}% duty, which draws about {}, over this servo's \
                 current limit of {}; on this supply the limit holds a stall only up to {:.0}% \
                 duty",
                duty * 100.0,
                ma.of(*stall),
                ma.of(*i_lim as f64),
                (allowed * 100.0).floor()
            ),
            Refusal::StreamOverLease { ms } => write!(
                f,
                "a {ms} ms capture with the stall permit held is longer than the \
                 {PERMIT_STREAM_MAX_MS} ms the permit can be held without a rewrite: shorten \
                 the capture"
            ),
            Refusal::NoWindowFloor => write!(
                f,
                "the servo did not report its current sensor floor, the smallest duty whose \
                 current it can read (window_floor_q15 reads 0): no drive is planned without \
                 it; update the servo's firmware"
            ),
            Refusal::NoLadderRoom { what, floor, cap } => write!(
                f,
                "{what} has no room on this supply: the current sensor reads from {:.1}% duty \
                 and the current limit allows {:.1}% at a stop; run it on a lower supply \
                 voltage (USB) or raise the current limit",
                floor * 100.0,
                cap * 100.0
            ),
            Refusal::BreakawayOverLimit { need, i_lim, ma } => {
                let say = |c: f64| {
                    if ma.0 > 0.0 {
                        format!("{:.0} mA", c * ma.0 * 1000.0)
                    } else {
                        format!("{c:.0} counts")
                    }
                };
                write!(
                    f,
                    "this servo needs about {} to start moving, over its current limit of {}: \
                     raise the limit or free the mechanism",
                    say(*need),
                    say(*i_lim as f64)
                )
            }
        }
    }
}

impl std::error::Error for Refusal {}

/// The host half of the stall permit lease. The firmware grants a permit
/// written true with torque on for about a second, a rewrite extends it,
/// and torque off drops it; firmware whose permit is a plain level takes
/// the rewrites as no-ops. A pump mirrors every write it sends through
/// [`PermitLease::wrote`] and rewrites the permit whenever it is `due`. The
/// clock is the caller's: pass milliseconds on any monotone timebase.
#[derive(Copy, Clone, Debug, Default)]
pub struct PermitLease {
    torque: bool,
    held: bool,
    last_ms: f64,
}

impl PermitLease {
    /// Mirror one write sent at `now_ms`.
    pub fn wrote(&mut self, reg: Reg, value: i32, now_ms: f64) {
        if reg == control::TORQUE_ENABLE {
            self.torque = value != 0;
            self.held &= self.torque;
        } else if reg == control::STALL_PERMIT {
            // written with torque off it grants nothing
            self.held = value != 0 && self.torque;
            self.last_ms = now_ms;
        }
    }

    pub fn held(&self) -> bool {
        self.held
    }

    /// A rewrite of the held permit is due.
    pub fn due(&self, now_ms: f64) -> bool {
        self.held && now_ms - self.last_ms >= PERMIT_REFRESH_MS as f64
    }

    /// The longest slice of a pause to sleep before looking again: a held
    /// permit is rewritten between slices.
    pub fn slice(&self, ms: u32) -> u32 {
        if self.held {
            ms.min(PERMIT_REFRESH_MS)
        } else {
            ms
        }
    }

    /// Refuse a stream of `ms` the held permit would lapse inside.
    pub fn check_stream(&self, ms: u32) -> Result<(), Refusal> {
        if self.held && ms > PERMIT_STREAM_MAX_MS {
            return Err(Refusal::StreamOverLease { ms });
        }
        Ok(())
    }
}

impl ServoLimits {
    pub fn ma(&self) -> Ma {
        Ma(self.amps_per_count)
    }

    /// Winding R, vcounts per ccount, once identified and with a rail read.
    pub fn r_vpc(&self) -> Option<f64> {
        (self.r_q12 != 0 && self.vbus != 0).then(|| self.r_q12 as f64 / 4096.0)
    }

    /// Soft limits inside the pot and in order: `osc cal` has run.
    pub fn calibrated(&self) -> bool {
        let (lo, hi) = self.soft;
        !is_board_default(self.soft) && lo >= 0 && hi <= POT_MAX && lo < hi
    }

    /// The guard inside the soft limits.
    pub fn guard(&self) -> Result<(u16, u16), Refusal> {
        if !self.calibrated() {
            return Err(Refusal::Uncalibrated { soft: self.soft });
        }
        Ok(guards((self.soft.0 as u16, self.soft.1 as u16)))
    }

    /// The abort threshold a run takes by default: a quarter over the
    /// limit. The firmware holds a stall AT the limit - window means 0.85
    /// to 1.0 of it, peaks to 1.1 - so an abort at the limit trips on the
    /// hold's own noise.
    pub fn abort_default(&self) -> i16 {
        let lim = self.i_lim as u32;
        (lim + lim / 4).min(i16::MAX as u32) as i16
    }

    /// The envelope for a drive: each end of `guard` and `i_abort` as given,
    /// else the servo's soft limits inset and [`Self::abort_default`].
    pub fn envelope(
        &self,
        guard: (Option<u16>, Option<u16>),
        i_abort: Option<i16>,
    ) -> Result<Envelope, Refusal> {
        let inset = self.guard()?;
        let max = self.abort_default();
        let i_abort = match i_abort {
            Some(i) if i.unsigned_abs() > max.unsigned_abs() => {
                return Err(Refusal::AbortOverLimit {
                    i_abort: i,
                    max,
                    i_lim: self.i_lim,
                    ma: self.ma(),
                });
            }
            Some(i) => i,
            None => max,
        };
        Ok(Envelope {
            guard: (guard.0.unwrap_or(inset.0), guard.1.unwrap_or(inset.1)),
            i_abort,
        })
    }

    /// Refuse a deliberate stall at `duty_q15` whose current the limit
    /// cannot hold. Passes when R is not identified yet: nothing to judge by.
    pub fn check_stall(&self, what: &str, duty_q15: i16) -> Result<(), Refusal> {
        match self.r_vpc() {
            Some(r) => self.check_stall_at(what, duty_q15, r),
            None => Ok(()),
        }
    }

    /// The same by a winding R the run measured, vcounts per ccount.
    pub fn check_stall_at(&self, what: &str, duty_q15: i16, r: f64) -> Result<(), Refusal> {
        let duty = duty_q15.unsigned_abs() as f64 / Q15;
        let vbus = self.vbus as f64;
        let stall = stall_counts(duty, r, vbus);
        if stall <= self.i_lim as f64 {
            return Ok(());
        }
        Err(Refusal::StallOverLimit {
            what: what.into(),
            duty,
            stall,
            i_lim: self.i_lim,
            allowed: duty_for(self.i_lim as f64, r, vbus),
            ma: self.ma(),
        })
    }

    /// Refuse a shaft whose breakaway, `moved`, a fraction of full scale,
    /// stalls over the limit by a winding R of `r`: nothing the limit
    /// allows starts it.
    pub fn check_breakaway(&self, moved: f64, r: f64) -> Result<(), Refusal> {
        let need = stall_counts(moved, r, self.vbus as f64);
        if need <= self.i_lim as f64 {
            return Ok(());
        }
        Err(Refusal::BreakawayOverLimit {
            need,
            i_lim: self.i_lim,
            ma: self.ma(),
        })
    }

    /// The window floor as a duty, q15, as the servo publishes it.
    pub fn window_floor(&self) -> i16 {
        self.window_floor_q15.min(i16::MAX as u16) as i16
    }

    /// Refuse a servo that publishes no window floor: no board constant
    /// stands in for it.
    pub fn check_floor(&self) -> Result<(), Refusal> {
        if self.window_floor_q15 == 0 {
            return Err(Refusal::NoWindowFloor);
        }
        Ok(())
    }

    /// The stall-safe plan for a drive at a stop before anything in this
    /// run measured R: from the R the servo carries, else from the class's
    /// lowest, `class_r_vpc`, which errs safe on any winding of the class.
    pub fn stall_plan(&self, class_r_vpc: f64, moved_at: Option<f64>) -> DutyPlan {
        DutyPlan::new(self, self.r_vpc().unwrap_or(class_r_vpc), moved_at)
    }

    /// The class-safe duty: a stall at it draws at most the limit on any
    /// winding of [`CLASS_R_MIN`] or more. `class_r_vpc` is that R in the
    /// servo's own units.
    pub fn bootstrap_duty(&self, class_r_vpc: f64) -> f64 {
        duty_for(self.i_lim as f64, class_r_vpc, self.vbus as f64)
    }

    /// Settings that leave the current limit the only protection a stall
    /// meets, in plain words.
    pub fn warnings(&self) -> Vec<String> {
        let ma = self.ma();
        let lim = ma.of(self.i_lim as f64);
        let mut w = Vec::new();
        if self.stall_yield >= self.i_lim {
            w.push(format!(
                "the stall yield current, {}, is not below the current limit of {lim}: a stall \
                 never backs off (set stall_yield_counts under the limit)",
                ma.of(self.stall_yield as f64)
            ));
        }
        if self.tau_trip as u32 > 2 * self.i_lim as u32 {
            w.push(format!(
                "the collision trip, {}, is over twice the current limit of {lim}: a collision \
                 never reaches it (set stall_tau_trip_counts near the limit)",
                ma.of(self.tau_trip as f64)
            ));
        }
        w
    }
}

/// The most the jam check applies while the shaft has not moved, mV: 25%
/// at 7.9 V, 45% at 4.39 V. Under the window floor a stall draws floor x
/// rail / R; from the floor up the firmware limiter holds the limit and
/// its stall timer ends a blocked check.
pub const NUDGE_MAX_MV: f64 = 2000.0;

/// The duty a rail of `rail_mv` turns `mv` into.
pub fn duty_of_mv(mv: f64, rail_mv: f64) -> f64 {
    if rail_mv <= 0.0 {
        return 0.0;
    }
    (mv / rail_mv).clamp(0.0, 1.0)
}

/// What a burst may do past the static limit. A burst drives the winding
/// for the capture's 0.52 ms and the kernel limiter does not tick through
/// it, so it is bounded instead by the volts it applies - a class bound,
/// not a multiple of the user's limit: the load it bounds is the rotor's
/// momentum, not a held torque. The rest of the allowance: the shaft was
/// proven free in this run, the burst arms inside the centre band from
/// rest or from a duty the run already drives, never seated, never with
/// the permit, at most [`BurstAllowance::MAX_ARMS`] a run, 100 ms apart.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BurstAllowance;

impl BurstAllowance {
    /// Applied volts, duty x rail: 40% at 7.9 V, 38% at 8.4 V.
    pub const MAX_MV: f64 = 3200.0;
    /// The low rung: the shortest ON window the fit reads a slope from.
    pub const LO: f64 = 0.25;
    pub const HI: f64 = 0.40;
    /// The fit's pair span: two rungs closer than this give no R.
    pub const PAIR_MIN_PCT: u8 = 10;
    pub const MAX_ARMS: u32 = 24;

    /// The two rungs on a rail of `rail_mv`, whole percent. None when the
    /// volts cap leaves them under the pair span apart: rails over 9.1 V.
    pub fn rungs(rail_mv: f64) -> Option<[f64; 2]> {
        let lo = pct_floor(Self::LO);
        let hi = pct_floor(Self::HI.min(duty_of_mv(Self::MAX_MV, rail_mv)));
        (hi >= lo + Self::PAIR_MIN_PCT).then(|| [lo as f64 / 100.0, hi as f64 / 100.0])
    }

    /// Repeats of a plan whose every repeat arms `arms` bursts, the asked
    /// count cut so the run stays at [`Self::MAX_ARMS`].
    pub fn repeats(asked: u32, arms: u32) -> u32 {
        asked.min(Self::MAX_ARMS / arms.max(1))
    }

    /// The settled current of the worst winding of the class at the volts
    /// cap, amps: what a burst's own planner may let a rung reach.
    pub fn i_max_a() -> f64 {
        Self::MAX_MV / 1000.0 / CLASS_R_MIN
    }
}

/// The stall-safe duties a run plans once the winding R is measured,
/// fractions of full scale: each stalls at or under the current limit on
/// the supply in use. They serve every drive that points at a stop and can
/// reach it, runs with the permit, or starts outside the guards. Free
/// rungs - the ladder, inertia - are not capped by stall current: the
/// firmware limiter, the stall timer, the travel guard and the runway
/// bound them.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DutyPlan {
    /// The lowest duty that moves the shaft: what moved it + 2%, never
    /// above `stop_cap`. Seeks, centring.
    pub seek: f64,
    /// Seated against a stop: half the limit.
    pub hold: f64,
    /// The highest stall-safe duty: its stall draws the limit. Stop seeks,
    /// the breakaway ramp, bursts against a stop.
    pub stop_cap: f64,
    pub r_vpc: f64,
    i_lim: f64,
    vbus: f64,
}

impl DutyPlan {
    /// Over the duty that moved the shaft, the margin a seek drives with.
    pub const SEEK_MARGIN: f64 = 0.02;

    /// `moved_at`: the duty the jam check first travelled at; without it
    /// seeks start at the cap.
    pub fn new(lim: &ServoLimits, r_vpc: f64, moved_at: Option<f64>) -> Self {
        let (i, v) = (lim.i_lim as f64, lim.vbus as f64);
        let stop_cap = duty_for(i, r_vpc, v);
        Self {
            seek: moved_at.map_or(stop_cap, |m| (m + Self::SEEK_MARGIN).min(stop_cap)),
            hold: duty_for(i / 2.0, r_vpc, v),
            stop_cap,
            r_vpc,
            i_lim: i,
            vbus: v,
        }
    }

    /// `breakaway`: the larger of the two directions' breakaway duties.
    pub fn with_breakaway(self, breakaway: f64) -> Self {
        Self {
            seek: (breakaway + Self::SEEK_MARGIN).min(self.stop_cap),
            ..self
        }
    }

    /// The largest step over a moving base drawing `i_run` counts whose
    /// first edge stays under the band the limiter holds, `lim - lim/8`.
    pub fn step_run(&self, i_run: f64) -> f64 {
        duty_for(self.i_lim - self.i_lim / 8.0 - i_run, self.r_vpc, self.vbus)
    }

    /// The stall-safe duties.
    pub fn duties(&self) -> [f64; 3] {
        [self.seek, self.hold, self.stop_cap]
    }

    /// The current, counts, `duty` draws stalled by this plan's R and rail.
    pub fn stall(&self, duty: f64) -> f64 {
        stall_counts(duty, self.r_vpc, self.vbus)
    }

    /// The dwells of a ladder of stalls, `what`: spread from the window floor,
    /// `floor_q15`, to the stall-safe cap, at most
    /// [`STALL_LADDER_RUNGS`] of them and [`STALL_LADDER_STEP_Q15`] or
    /// more apart. Under the floor the shunt reads nothing and the window
    /// holds the last current it read, so no dwell goes there; a band
    /// with fewer than three rungs, or whose stall currents span under
    /// [`STALL_LADDER_SPAN`] of the limit, leaves nothing to fit a slope
    /// to.
    pub fn stall_ladder(&self, what: &'static str, floor_q15: i16) -> Result<Vec<f64>, Refusal> {
        let (lo, hi) = (floor_q15 as i32, q15_floor(self.stop_cap) as i32);
        let room = Refusal::NoLadderRoom {
            what,
            floor: lo as f64 / Q15,
            cap: self.stop_cap,
        };
        if hi < lo {
            return Err(room);
        }
        let n = STALL_LADDER_RUNGS.min((hi - lo) / STALL_LADDER_STEP_Q15 + 1);
        let at = |q: i32| stall_counts(q as f64 / Q15, self.r_vpc, self.vbus);
        if n < 3 || at(hi) - at(lo) < STALL_LADDER_SPAN * self.i_lim {
            return Err(room);
        }
        Ok((0..n)
            .map(|k| (lo + k * (hi - lo) / (n - 1)) as f64 / Q15)
            .collect())
    }
}

/// The resistance stop ladder, as its refusal names it.
pub const STOP_LADDER: &str = "the resistance stop ladder";

/// The most dwells the resistance stop ladder takes.
pub const STALL_LADDER_RUNGS: i32 = 4;
/// The closest two dwells may sit, q15: 0.5% of full scale, rounded up.
pub const STALL_LADDER_STEP_Q15: i32 = 164;
/// The least stall-current span, a fraction of the limit, the dwells must
/// cover for a slope.
pub const STALL_LADDER_SPAN: f64 = 0.15;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::TelemetrySnapshot;
    use crate::regs::telemetry;

    /// The bench MG90 as SAVEd: limit 280 with the SG90-era stall settings,
    /// R 4.9 ohm, a 7.9 V rail, 60 mohm at G 15.
    fn mg90() -> ServoLimits {
        ServoLimits {
            i_lim: 280,
            stall_yield: 545,
            tau_trip: 2182,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 7270,
            vbus: 3204,
            window_floor_q15: 4356,
            amps_per_count: 3.3 / 4096.0 / (15.0 * 0.060),
        }
    }

    #[test]
    fn guards_come_from_the_soft_limits() {
        assert_eq!(guards((432, 3626)), (532, 3526));
        let env = mg90().envelope((None, None), None).unwrap();
        assert_eq!(env.guard, (532, 3526));
        let env = mg90().envelope((Some(600), None), None).unwrap();
        assert_eq!(env.guard, (600, 3526), "each end overrides alone");
    }

    #[test]
    fn i_abort_defaults_a_quarter_over_the_current_limit() {
        assert_eq!(mg90().envelope((None, None), None).unwrap().i_abort, 350);
        assert_eq!(
            mg90().envelope((None, None), Some(200)).unwrap().i_abort,
            200
        );
        assert_eq!(
            mg90().envelope((None, None), Some(350)).unwrap().i_abort,
            350
        );
        let err = mg90()
            .envelope((None, None), Some(351))
            .unwrap_err()
            .to_string();
        assert_eq!(
            err,
            "an abort threshold of 351 counts (314 mA) is over 350 counts (313 mA), a quarter \
             above this servo's current limit of 280 counts (251 mA): the run could draw more \
             current than the servo is set to allow (leave --i-abort out to abort there)"
        );
    }

    /// 3.0 ohm in the bench board's units: 1117 counts/A, 2.466 mV/vcount.
    fn class_r_vpc() -> f64 {
        let v_per_vcount = 3.3 / 4096.0 * (6_800.0 + 3_300.0) / 3_300.0;
        CLASS_R_MIN * mg90().amps_per_count / v_per_vcount
    }

    #[test]
    fn every_stall_safe_duty_stalls_under_the_limit() {
        let r = 7270.0 / 4096.0;
        for (i_lim, vbus) in [
            (280, 3204),
            (280, 1780),
            (335, 3204),
            (150, 2400),
            (600, 3400),
        ] {
            let lim = ServoLimits {
                i_lim,
                vbus,
                ..mg90()
            };
            for plan in [
                DutyPlan::new(&lim, r, None),
                DutyPlan::new(&lim, r, Some(0.25)),
                DutyPlan::new(&lim, r, Some(0.10)).with_breakaway(0.08),
                DutyPlan::new(&lim, r, None).with_breakaway(0.9),
            ] {
                for d in plan.duties() {
                    for q in [d, q15_floor(d) as f64 / Q15, pct_floor(d) as f64 / 100.0] {
                        let stall = stall_counts(q, r, vbus as f64);
                        assert!(
                            stall <= i_lim as f64 + 1e-9,
                            "{plan:?}: {q} stalls at {stall}"
                        );
                    }
                }
                assert!(plan.seek <= plan.stop_cap && plan.hold < plan.stop_cap);
            }
        }
        // the bench servo on 2S, a shaft the jam check moved at 12%
        let plan = DutyPlan::new(&mg90(), r, Some(0.12));
        assert!((plan.seek - 0.14).abs() < 1e-12);
        let plan = plan.with_breakaway(0.08);
        assert!((plan.seek - 0.10).abs() < 1e-12);
        assert!((plan.stop_cap - 0.1551).abs() < 1e-3);
        assert!((plan.hold - 0.0776).abs() < 1e-3);
        // a step over a base drawing 80 counts: 9.1%
        assert!((plan.step_run(80.0) - 0.0910).abs() < 1e-3);
    }

    /// A telemetry read as the servo serves it, from `fault_flags` through
    /// `window_floor_q15`, publishing `floor` and a 4.39 V USB rail.
    fn published(floor: u16) -> TelemetrySnapshot {
        let (base, f, v) = (
            telemetry::FAULT_FLAGS.addr,
            telemetry::WINDOW_FLOOR_Q15,
            telemetry::VBUS_COUNTS,
        );
        let mut region = vec![0u8; (f.addr + f.width as u16 - base) as usize];
        let mut put = |r: Reg, b: [u8; 2]| {
            let at = (r.addr - base) as usize;
            region[at..at + 2].copy_from_slice(&b);
        };
        put(f, floor.to_le_bytes());
        put(v, 1780u16.to_le_bytes());
        TelemetrySnapshot::parse(base, &region).unwrap()
    }

    /// Two boards' floors, 160 and 240 ticks of a 1200 period: the limits
    /// carry the number the servo publishes, and the stop ladder starts on
    /// it.
    #[test]
    fn servo_limits_take_the_floor_from_the_servo() {
        let r = 7270.0 / 4096.0;
        for floor in [4356, 6534] {
            let tel = published(floor);
            let lim = ServoLimits {
                vbus: tel.vbus_counts,
                window_floor_q15: tel.window_floor_q15,
                ..mg90()
            };
            assert_eq!(lim.check_floor(), Ok(()));
            assert_eq!(lim.window_floor(), floor as i16);
            let rungs = DutyPlan::new(&lim, r, None)
                .stall_ladder(STOP_LADDER, lim.window_floor())
                .unwrap();
            assert_eq!(q15_floor(rungs[0]), floor as i16, "{rungs:?}");
        }
    }

    #[test]
    fn a_servo_without_a_floor_is_refused_in_plain_words() {
        let tel = published(0);
        let lim = ServoLimits {
            window_floor_q15: tel.window_floor_q15,
            ..mg90()
        };
        let err = lim.check_floor().unwrap_err();
        assert_eq!(err, Refusal::NoWindowFloor);
        assert_eq!(
            err.to_string(),
            "the servo did not report its current sensor floor, the smallest duty whose current \
             it can read (window_floor_q15 reads 0): no drive is planned without it; update \
             the servo's firmware"
        );
    }

    /// Every dwell of the stop ladder sits at or over the window floor and
    /// at or under the stall-safe cap, however it is rounded, and so does
    /// the seek that takes it to the stop.
    #[test]
    fn stall_ladder_rungs_stay_under_the_limit() {
        let floor = mg90().window_floor();
        for r in [7270.0 / 4096.0, class_r_vpc(), 1.2] {
            for (i_lim, vbus) in [(280, 1780), (335, 1780), (600, 3400), (150, 1200)] {
                let lim = ServoLimits {
                    i_lim,
                    vbus,
                    ..mg90()
                };
                for moved in [None, Some(0.05), Some(0.30)] {
                    let plan = DutyPlan::new(&lim, r, moved);
                    let seek = q15_floor(plan.seek) as f64 / Q15;
                    assert!(stall_counts(seek, r, vbus as f64) <= i_lim as f64);
                    let Ok(rungs) = plan.stall_ladder(STOP_LADDER, floor) else {
                        continue;
                    };
                    assert!((3..=4).contains(&rungs.len()), "{rungs:?}");
                    assert!(rungs.windows(2).all(|w| w[1] - w[0] >= 0.005), "{rungs:?}");
                    for d in &rungs {
                        let q = q15_floor(*d);
                        assert!(q >= floor, "{d} under the floor");
                        let stall = stall_counts(q as f64 / Q15, r, vbus as f64);
                        assert!(stall <= i_lim as f64 + 1e-9, "{d}: {stall}");
                    }
                }
            }
        }
    }

    /// 4.9 ohm on osc-dev-v006, the bench MG90 at limit 280 and an SG90-like
    /// servo at 335 and 3.9 ohm. On 2S the band from the 13.3% floor to
    /// the cap spans 14% of the MG90's limit and 10% of the SG90's: no
    /// room. On USB both ladders run over most of the limit.
    #[test]
    fn the_stop_ladder_needs_room_over_the_window_floor() {
        let mg = 7270.0 / 4096.0;
        let sg = class_r_vpc() * 3.9 / CLASS_R_MIN;
        let ladder = |i_lim: u16, r: f64, vbus: u16| {
            let lim = ServoLimits {
                i_lim,
                vbus,
                ..mg90()
            };
            DutyPlan::new(&lim, r, None).stall_ladder(STOP_LADDER, lim.window_floor())
        };
        let err = ladder(280, mg, 3204).unwrap_err();
        assert_eq!(
            err.to_string(),
            "the resistance stop ladder has no room on this supply: the current sensor reads \
             from 13.3% duty and the current limit allows 15.5% at a stop; run it on a lower \
             supply voltage (USB) or raise the current limit"
        );
        assert!(matches!(
            ladder(335, sg, 3204),
            Err(Refusal::NoLadderRoom { .. })
        ));
        for (i_lim, r, want) in [
            (280, mg, [0.1329, 0.1817, 0.2304, 0.2792]),
            (335, sg, [0.1329, 0.1775, 0.2220, 0.2665]),
        ] {
            let rungs = ladder(i_lim, r, 1780).unwrap();
            for (got, want) in rungs.iter().zip(want) {
                assert!((got - want).abs() < 5e-4, "{i_lim}: {rungs:?}");
            }
        }
        // the class's lowest R, R unknown: the cap falls under the floor on
        // 2S, and on USB the band still holds four rungs
        assert!(ladder(280, class_r_vpc(), 3204).is_err());
        assert_eq!(ladder(280, class_r_vpc(), 1780).unwrap().len(), 4);
    }

    #[test]
    fn burst_rungs_fit_the_fit_and_the_allowance() {
        assert_eq!(BurstAllowance::rungs(7900.0), Some([0.25, 0.40]));
        assert_eq!(BurstAllowance::rungs(8400.0), Some([0.25, 0.38]));
        assert_eq!(BurstAllowance::rungs(4390.0), Some([0.25, 0.40]));
        assert_eq!(BurstAllowance::rungs(12600.0), None);
        // the last rail the pair still spans
        assert_eq!(BurstAllowance::rungs(9100.0), Some([0.25, 0.35]));
        assert_eq!(BurstAllowance::rungs(9200.0), None);
        for rail in [4390.0, 7900.0, 8400.0, 9100.0] {
            let [_, hi] = BurstAllowance::rungs(rail).unwrap();
            assert!(hi * rail <= BurstAllowance::MAX_MV + 1e-9);
        }
        assert!((BurstAllowance::i_max_a() - 1.0667).abs() < 1e-3);
    }

    #[test]
    fn burst_count_is_bounded() {
        // two rungs and the from-a-hold control, both signs: six a repeat
        assert_eq!(BurstAllowance::repeats(5, 6), 4);
        assert_eq!(BurstAllowance::repeats(2, 6), 2);
        assert_eq!(BurstAllowance::repeats(9, 4), 6);
        for asked in 0..20 {
            assert!(BurstAllowance::repeats(asked, 6) * 6 <= BurstAllowance::MAX_ARMS);
        }
    }

    #[test]
    fn bootstrap_duty_is_class_safe() {
        // the plan's figure: 300 mA (335 counts) on 2S is 11.4%, under the
        // 13.3% window floor where the limiter cannot see current
        let lim = ServoLimits {
            i_lim: 335,
            ..mg90()
        };
        let d = lim.bootstrap_duty(class_r_vpc());
        assert!((d - 0.114).abs() < 0.001, "{d}");
        assert!(d < 160.0 / 1200.0);
        // the bench limit: 9.5%, stalling at the limit on a 3 ohm winding
        // and under it on the real 4.9 ohm one
        let lim = mg90();
        let d = lim.bootstrap_duty(class_r_vpc());
        assert!((d - 0.095).abs() < 0.001, "{d}");
        let q = q15_floor(d) as f64 / Q15;
        assert!(stall_counts(q, class_r_vpc(), 3204.0) <= 280.0);
        assert!(stall_counts(q, 7270.0 / 4096.0, 3204.0) < 200.0);
        // on USB the same current takes more duty; the volts are the same
        let usb = ServoLimits {
            vbus: 1780,
            ..mg90()
        };
        let du = usb.bootstrap_duty(class_r_vpc());
        assert!((du * 1780.0 - d * 3204.0).abs() < 1e-6);
    }

    #[test]
    fn plan_over_the_limit_is_refused_in_plain_words() {
        let lim = mg90();
        // 12% stalls at 217 counts on 2S: under the limit
        assert_eq!(lim.check_stall("the hold", 3932), Ok(()));
        let err = lim.check_stall("resistance", 11468).unwrap_err();
        assert_eq!(
            err.to_string(),
            "resistance stalls the motor at 35% duty, which draws about 632 counts (566 mA), \
             over this servo's current limit of 280 counts (251 mA); on this supply the limit \
             holds a stall only up to 15% duty"
        );
        // with R not identified there is nothing to judge by
        let virgin = ServoLimits { r_q12: 0, ..lim };
        assert_eq!(virgin.check_stall("resistance", 11468), Ok(()));
    }

    #[test]
    fn board_default_limits_are_refused() {
        for soft in [(0, 4095), (-4096, 8191), (-10, 3000), (3000, 1000)] {
            let lim = ServoLimits { soft, ..mg90() };
            let err = lim.envelope((None, None), None).unwrap_err();
            assert_eq!(err, Refusal::Uncalibrated { soft });
            assert!(err.to_string().ends_with("run `osc cal` first"), "{err}");
        }
    }

    #[test]
    fn stall_math_round_trips() {
        let r = 7270.0 / 4096.0;
        let d = duty_for(280.0, r, 3204.0);
        assert!((d - 0.1551).abs() < 1e-3, "{d}");
        assert!((stall_counts(d, r, 3204.0) - 280.0).abs() < 1e-9);
        assert_eq!(stall_counts(0.5, 0.0, 3204.0), f64::INFINITY);
        assert_eq!(duty_for(280.0, r, 0.0), 0.0);
    }

    #[test]
    fn the_lease_is_held_only_behind_torque_on() {
        let mut l = PermitLease::default();
        l.wrote(control::STALL_PERMIT, 1, 0.0);
        assert!(!l.held(), "a permit written with torque off grants nothing");
        l.wrote(control::TORQUE_ENABLE, 1, 10.0);
        assert!(!l.held());
        l.wrote(control::STALL_PERMIT, 1, 20.0);
        assert!(l.held());
        assert!(!l.due(269.0));
        assert!(l.due(270.0));
        assert_eq!(l.slice(3000), PERMIT_REFRESH_MS);
        assert_eq!(l.check_stream(750), Ok(()));
        assert_eq!(
            l.check_stream(800).unwrap_err().to_string(),
            "a 800 ms capture with the stall permit held is longer than the 750 ms the permit \
             can be held without a rewrite: shorten the capture"
        );
        l.wrote(control::TORQUE_ENABLE, 0, 30.0);
        assert!(!l.held() && !l.due(1000.0), "torque off drops it");
        assert_eq!(l.slice(3000), 3000);
        assert_eq!(l.check_stream(3000), Ok(()));
    }

    /// The bench MG90's SG90-era yield and trip: both warned about.
    #[test]
    fn stall_settings_above_the_limit_are_warned() {
        let w = mg90().warnings();
        assert_eq!(w.len(), 2, "{w:?}");
        assert_eq!(
            w[0],
            "the stall yield current, 545 counts (488 mA), is not below the current limit of \
             280 counts (251 mA): a stall never backs off (set stall_yield_counts under the \
             limit)"
        );
        assert!(w[1].starts_with("the collision trip, 2182 counts (1953 mA), is over twice"));
        let tidy = ServoLimits {
            stall_yield: 168,
            tau_trip: 280,
            ..mg90()
        };
        assert!(tidy.warnings().is_empty());
    }
}
