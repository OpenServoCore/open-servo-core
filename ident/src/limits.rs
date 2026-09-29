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
    /// `i_window_min_ticks`: the shortest drive window the shunt reads.
    pub i_floor_ticks: u16,
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
    /// An abort threshold above the current limit.
    AbortOverLimit { i_abort: i16, i_lim: u16, ma: Ma },
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
            Refusal::AbortOverLimit { i_abort, i_lim, ma } => write!(
                f,
                "an abort threshold of {} is above this servo's current limit of {}: the run \
                 could draw more current than the servo is set to allow (leave --i-abort out to \
                 abort at the limit)",
                ma.of(*i_abort as f64),
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

    /// The envelope for a drive: each end of `guard` and `i_abort` as given,
    /// else the servo's soft limits inset and its current limit.
    pub fn envelope(
        &self,
        guard: (Option<u16>, Option<u16>),
        i_abort: Option<i16>,
    ) -> Result<Envelope, Refusal> {
        let inset = self.guard()?;
        let i_lim = self.i_lim.min(i16::MAX as u16) as i16;
        let i_abort = match i_abort {
            Some(i) if i.unsigned_abs() > self.i_lim => {
                return Err(Refusal::AbortOverLimit {
                    i_abort: i,
                    i_lim: self.i_lim,
                    ma: self.ma(),
                });
            }
            Some(i) => i,
            None => i_lim,
        };
        Ok(Envelope {
            guard: (guard.0.unwrap_or(inset.0), guard.1.unwrap_or(inset.1)),
            i_abort,
        })
    }

    /// Refuse a deliberate stall at `duty_q15` whose current the limit
    /// cannot hold. Passes when R is not identified yet: nothing to judge by.
    pub fn check_stall(&self, what: &str, duty_q15: i16) -> Result<(), Refusal> {
        let Some(r) = self.r_vpc() else {
            return Ok(());
        };
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

#[cfg(test)]
mod tests {
    use super::*;

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
            i_floor_ticks: 160,
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
    fn i_abort_defaults_to_the_current_limit() {
        assert_eq!(mg90().envelope((None, None), None).unwrap().i_abort, 280);
        assert_eq!(
            mg90().envelope((None, None), Some(200)).unwrap().i_abort,
            200
        );
        let err = mg90()
            .envelope((None, None), Some(1100))
            .unwrap_err()
            .to_string();
        assert_eq!(
            err,
            "an abort threshold of 1100 counts (985 mA) is above this servo's current limit of \
             280 counts (251 mA): the run could draw more current than the servo is set to allow \
             (leave --i-abort out to abort at the limit)"
        );
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
