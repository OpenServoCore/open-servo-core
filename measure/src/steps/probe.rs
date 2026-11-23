//! Step 2, first probe and polarity, free shaft, wherever the horn stands.
//! It runs on the rest step's result: its zeros, the pot's noise and the
//! frame ripple. Metal trains only, for now.
//!
//! Part A, the electrical probe: shunt bursts from rest at 20 % and 32 %
//! of the stated voltage, both signs, twice each, 0.52 ms of drive each
//! and at least 100 ms apart. Each is read by the period-pair rise fit
//! ([`crate::fit::rise`]) on the nominal scales and the board's edge lags,
//! and one fit with an intercept over the bursts that are not the first
//! after a sign change gives the free rotor's planning R and L. Before
//! any R a burst is priced blocked at the lower of the board's overcurrent
//! trip and V over the 0.5 ohm acceptance floor, or at the nameplate's
//! stated stall where that reads more. [`judge`] is part A's acceptance,
//! pure, and the board's drive check: a sign with no current ends the run
//! here.
//!
//! The planning R prices part B and nothing else, at [`PRICE_SHARE`] of
//! itself so its current is overstated: the stored R, and every stall
//! current a set holds, come from a seated read later. It is never a set
//! field nor a rules input.
//!
//! Part B is [`super::polarity`]. A sign found wrong against the stored
//! `drive_polarity` is corrected volatile and put back by the way out; the
//! corrected sign goes to the result for later steps.
//!
//! The way out leaves the shaft where it is: no closed loop is trusted
//! yet, so nothing centers it.

use std::time::Duration;

use osc_client::pipe::Pipe;
use serde_json::{Map, Value, json};

use super::polarity::{self, Comp, Ladder, LadderCfg, Outcome};
use super::{
    Check, Ended, Halt, Rig, Taken, Verdict, interrupted, keep_all, mean_u16, mode_value, read_u8,
    read_u16, rest, write,
};
use crate::Board;
use crate::capture::burst::{self, ArmSeen, CHANS_DIFF, Capture, CaptureCfg, Pre};
use crate::capture::meta;
use crate::capture::ripple::{FOLD, Pattern};
use crate::capture::rows::Layout;
use crate::capture::tel::{self, Line};
use crate::envelope::{Envelope, Facts, HeatModel, Kind, Resistance, Rung, hold_a};
use crate::fit::rise::{self, Lags, Period, Rise, Scales};
use crate::fit::{mean, median};
use crate::guard::{self, Ledger, Way};
use crate::inputs::ThermalInputs;
use crate::install::Refused;
use crate::nameplate::{GearClass, Nameplate};
use crate::ntc::ADC_COUNTS;
use crate::sink::Sink;
use crate::state::{self, State, field};

pub const STEP: &str = "probe";
pub const RESULT: &str = "results/probe.json";
const METHOD: &str = "probe-rise-and-polarity";
const REVISION: u32 = 1;
/// Part A's levels, shares of the stated voltage.
pub const LEVELS: [f64; 2] = [0.20, 0.32];
/// The drive a burst applies: the half of its 960 samples after the step.
pub const BURST_S: f64 = 0.000_52;
/// Between bursts: the firmware wants 100 ms.
pub const GAP_S: f64 = 0.12;
/// The acceptance floor of R, ohm: a blocked burst is priced at no less.
pub const R_FLOOR_OHM: f64 = 0.5;
pub const R_RANGE_OHM: (f64, f64) = (R_FLOOR_OHM, 50.0);
/// L / R in PWM periods.
pub const TAU_RANGE_PERIODS: (f64, f64) = (1.0, 20.0);
pub const SIGNS_SHARE: f64 = 0.10;
pub const TAPS_SHARE: f64 = 0.10;
/// A sign whose current is under this share of the other's carries none.
pub const DEAD_SHARE: f64 = 0.25;
/// Under this a level carries no current either way, A.
pub const NO_CURRENT_A: f64 = 0.02;
/// A period's volts over this many times duty x rail is a full-rail pulse.
pub const FULL_RAIL: f64 = 1.5;
/// The current back at zero after a burst, A of the rest zero.
pub const AFTER_MAX_A: f64 = 0.02;
/// The share of the free rotor's planning R part B is priced at. The
/// pooled fit with an intercept reads about a tenth over a seated rotor's
/// R, so the share prices at about the seated R; the margin is the
/// ladder's ceiling, half the stop-hold line.
pub const PRICE_SHARE: f64 = 0.9;
/// Periods at a burst's end its current is read over.
const LATE: usize = 3;
const AFTER_READS: u32 = 4;
const Q15: f64 = 32768.0;
const PRICE_WORDS: &str = "planning R (free rotor, x0.9 for pricing), not the stored R";

/// One burst of part A.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Shot {
    pub k: usize,
    /// The logical duty's sign.
    pub dir: i8,
    pub level: usize,
    pub volts: f64,
    pub duty_q15: i16,
    /// Not the first burst after a sign change: the first reads L high,
    /// and the first of the run follows a sign no one knows.
    pub in_fit: bool,
}

/// Part A's eight bursts at `stated_v` on `rail_v`: +low, +low, +high,
/// +high, -high, -high, -low, -low.
pub fn shots(stated_v: f64, rail_v: f64) -> Vec<Shot> {
    let order = [
        (1, 0),
        (1, 0),
        (1, 1),
        (1, 1),
        (-1, 1),
        (-1, 1),
        (-1, 0),
        (-1, 0),
    ];
    order
        .iter()
        .enumerate()
        .map(|(k, &(dir, level))| {
            let volts = (LEVELS[level] * stated_v).min(burst::MAX_MV / 1000.0);
            let q = (volts / rail_v * Q15).floor().min(i16::MAX as f64) as i16;
            Shot {
                k,
                dir,
                level,
                volts: q as f64 / Q15 * rail_v,
                duty_q15: q * dir as i16,
                in_fit: k != 0 && order[k - 1].0 == dir,
            }
        })
        .collect()
}

/// The R a burst at `volts` is priced blocked at before any R is
/// measured: the lowest of the board trip's and the acceptance floor's,
/// and the nameplate's stated stall where it states one, so the current
/// is the highest any of them allows.
pub fn first_r(volts: f64, trip_a: f64, stated: Option<(f64, f64)>) -> Resistance {
    let floor = R_FLOOR_OHM.max(volts / trip_a);
    match stated.filter(|(v, a)| *v > 0.0 && *a > 0.0 && v / a < floor) {
        Some((v, a)) => Resistance {
            ohm: v / a,
            source: format!(
                "before R is measured: the nameplate's {v} V over its stated {a} A stall, which reads more current than the board's {trip_a} A trip or the {R_FLOOR_OHM} ohm floor"
            ),
        },
        None => Resistance {
            ohm: floor,
            source: format!(
                "before R is measured: the lower of the board's {trip_a} A trip and V over the {R_FLOOR_OHM} ohm floor"
            ),
        },
    }
}

/// A burst taken, and the current read after it, counts.
#[derive(Clone, Debug, PartialEq)]
pub struct Read {
    pub shot: Shot,
    pub cap: Capture,
    pub after_counts: f64,
}

/// What part A's judge reads by.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AFacts {
    pub rail_v: f64,
    /// The rest step's awake current zero, counts.
    pub zero_counts: f64,
    pub polarity: bool,
    pub scales: Scales,
}

/// One burst's reads.
#[derive(Clone, Debug, PartialEq)]
pub struct ShotRead {
    pub shot: Shot,
    pub periods: Result<Vec<Period>, String>,
    pub fit: Option<Rise>,
    /// The window current over the last periods, A.
    pub i_late_a: f64,
    /// The window's tap difference in the bridge's sign, V, and its sign
    /// as the taps read it.
    pub v_on_v: f64,
    pub v_on_sign: f64,
    pub v_mean_v: f64,
    pub after_a: f64,
}

#[derive(Clone, Debug)]
pub struct PartA {
    pub reads: Vec<ShotRead>,
    pub pooled: Option<Rise>,
    pub checks: Vec<Check>,
    pub bridge_fault: bool,
    /// The PWM period the bursts ran at, s.
    pub period_s: f64,
}

impl PartA {
    pub fn accepted(&self) -> bool {
        self.checks.iter().all(|c| c.verdict != Verdict::Fail)
    }

    /// The fit part B is priced from: none unless every check passed, so a
    /// bridge fault or a winding this board cannot read ends the run here.
    pub fn r_plan_free(&self) -> Option<&Rise> {
        self.pooled.as_ref().filter(|_| self.accepted())
    }
}

fn pass(ok: bool) -> Verdict {
    if ok { Verdict::Pass } else { Verdict::Fail }
}

/// Part A's estimates and every acceptance check, pure.
pub fn judge(f: &AFacts, reads: &[Read]) -> PartA {
    let s = &f.scales;
    let period_s = reads
        .first()
        .map_or(f64::NAN, |r| rise::pwm_period_s(r.cap.meta.pwm_arr));
    let wiring = |dir: i8| dir as f64 * if f.polarity { 1.0 } else { -1.0 };
    let rs: Vec<ShotRead> = reads
        .iter()
        .map(|r| {
            let sign = wiring(r.shot.dir);
            let periods = rise::periods(&r.cap, sign, s, 0.0);
            let (fit, i_late, v_on, v_mean) = match &periods {
                Ok(t) => {
                    let late = &t[t.len().saturating_sub(LATE)..];
                    (
                        rise::fit(t, period_s, s.window_scale),
                        mean(&late.iter().map(|p| p.i_a).collect::<Vec<_>>()).unwrap_or(f64::NAN),
                        median(&t.iter().map(|p| p.v_on_v).collect::<Vec<_>>()).unwrap_or(f64::NAN),
                        mean(&t.iter().map(|p| p.v_v).collect::<Vec<_>>()).unwrap_or(f64::NAN),
                    )
                }
                Err(_) => (None, f64::NAN, f64::NAN, f64::NAN),
            };
            ShotRead {
                shot: r.shot,
                periods,
                fit,
                i_late_a: i_late,
                v_on_v: v_on,
                // read in the bridge's sign: as the taps read it, that
                // times the bridge's
                v_on_sign: (v_on * sign).signum(),
                v_mean_v: v_mean,
                after_a: (r.after_counts - f.zero_counts) / s.counts_per_a,
            }
        })
        .collect();
    let mut checks = Vec::new();
    let mut check = |name: &str, value: String, bound: String, verdict: Verdict| {
        checks.push(Check {
            name: name.into(),
            value,
            bound,
            verdict,
        })
    };
    let broken: Vec<String> = rs
        .iter()
        .filter_map(|r| {
            r.periods
                .as_ref()
                .err()
                .map(|e| format!("burst {}: {e}", r.shot.k))
        })
        .collect();
    check(
        "every burst reads its periods",
        if broken.is_empty() {
            format!("{} bursts", rs.len())
        } else {
            broken.join("; ")
        },
        "a window line and a tap pair every full period".into(),
        pass(broken.is_empty() && !rs.is_empty()),
    );
    let current = |level: usize, dir: i8| {
        let v: Vec<f64> = rs
            .iter()
            .filter(|r| r.shot.level == level && r.shot.dir == dir && r.shot.in_fit)
            .map(|r| r.i_late_a)
            .collect();
        median(&v).unwrap_or(f64::NAN)
    };
    let mut fault = Vec::new();
    let mut apart = Vec::new();
    let mut said = Vec::new();
    for (level, share) in LEVELS.iter().enumerate() {
        let (p, n) = (current(level, 1), current(level, -1));
        let (lo, hi) = (p.min(n), p.max(n));
        said.push(format!(
            "{:.0}%: + {:.1} mA, - {:.1} mA, {:.1}% apart",
            100.0 * share,
            1e3 * p,
            1e3 * n,
            100.0 * (hi - lo) / hi
        ));
        if hi.is_nan() || hi < NO_CURRENT_A {
            fault.push(format!(
                "no current either way at {:.0}%: an open winding or a dead bridge",
                100.0 * share
            ));
        } else if lo.is_nan() || lo < DEAD_SHARE * hi {
            fault.push(format!(
                "the {} sign carries {:.1} mA against {:.1} mA at {:.0}%: one half of the bridge drives nothing",
                if p < n { "+" } else { "-" },
                1e3 * lo,
                1e3 * hi,
                100.0 * share
            ));
        }
        apart.push((hi - lo) / hi);
    }
    for r in &rs {
        let d = r.shot.duty_q15.unsigned_abs() as f64 / Q15;
        if r.v_mean_v > FULL_RAIL * d * f.rail_v {
            fault.push(format!(
                "burst {} applies {:.2} V a period at {:.1}% duty of a {:.2} V rail: a full-rail pulse at partial duty",
                r.shot.k,
                r.v_mean_v,
                100.0 * d,
                f.rail_v
            ));
        }
    }
    let bridge_fault = !fault.is_empty();
    check(
        "the bridge drives both signs",
        if fault.is_empty() {
            "both signs carry current, no period over its duty's volts".into()
        } else {
            fault.join("; ")
        },
        format!(
            "each sign over {:.0}% of the other's current and {} mA, a period under {FULL_RAIL} x duty x rail",
            100.0 * DEAD_SHARE,
            1e3 * NO_CURRENT_A
        ),
        pass(!bridge_fault),
    );
    let worst = apart.iter().copied().fold(0.0, f64::max);
    check(
        "both signs read the same current",
        said.join("; "),
        format!("within {:.0}%", 100.0 * SIGNS_SHARE),
        pass(worst <= SIGNS_SHARE),
    );
    let taps: Vec<String> = rs
        .iter()
        .map(|r| {
            format!(
                "{} {:.2} V ({:.1}%){}",
                r.shot.k,
                r.v_on_v,
                100.0 * r.v_on_v / f.rail_v,
                if r.v_on_sign == wiring(r.shot.dir) {
                    ""
                } else {
                    " WRONG SIGN"
                }
            )
        })
        .collect();
    let taps_ok = !rs.is_empty()
        && rs.iter().all(|r| {
            (r.v_on_v / f.rail_v - 1.0).abs() <= TAPS_SHARE && r.v_on_sign == wiring(r.shot.dir)
        });
    check(
        "the tap difference is the rail during the pulse and flips with the sign",
        format!("burst {}", taps.join(", ")),
        format!(
            "within {:.0}% of the {:.2} V rail, the sign the bridge drives",
            100.0 * TAPS_SHARE,
            f.rail_v
        ),
        pass(taps_ok),
    );
    let after = rs.iter().map(|r| r.after_a.abs()).fold(0.0, f64::max);
    check(
        "the current is back at zero after",
        format!(
            "at most {:.1} mA off the rest zero after a burst",
            1e3 * after
        ),
        format!("within {} mA", 1e3 * AFTER_MAX_A),
        pass(after <= AFTER_MAX_A),
    );
    let tables: Vec<&[Period]> = rs
        .iter()
        .filter(|r| r.shot.in_fit)
        .filter_map(|r| r.periods.as_ref().ok().map(|t| &t[..]))
        .collect();
    let pooled = rise::pooled(&tables, period_s, s.window_scale);
    match &pooled {
        Some(p) => {
            check(
                "R is a motor's this board can read",
                format!(
                    "{:.3} ohm (free rotor) over {} pairs of {} bursts, intercept {:+.3} V",
                    p.r_ohm,
                    p.pairs,
                    tables.len(),
                    p.v0_v
                ),
                format!("{} to {} ohm", R_RANGE_OHM.0, R_RANGE_OHM.1),
                pass((R_RANGE_OHM.0..=R_RANGE_OHM.1).contains(&p.r_ohm)),
            );
            let periods = p.tau_s / period_s;
            check(
                "L / R is a motor's this board can read",
                format!(
                    "L {:.3} mH, L / R {:.1} us, {periods:.2} PWM periods",
                    1e3 * p.l_h,
                    1e6 * p.tau_s
                ),
                format!("{} to {} periods", TAU_RANGE_PERIODS.0, TAU_RANGE_PERIODS.1),
                pass((TAU_RANGE_PERIODS.0..=TAU_RANGE_PERIODS.1).contains(&periods)),
            );
        }
        None => check(
            "R is a motor's this board can read",
            "the bursts give no fit".into(),
            "a fit over the bursts".into(),
            Verdict::Fail,
        ),
    }
    for c in checks.iter_mut().filter(|c| {
        c.verdict == Verdict::Fail && (c.name.starts_with("R is") || c.name.starts_with("L / R"))
    }) {
        c.bound.push_str(": not a motor this board can read");
    }
    PartA {
        reads: rs,
        pooled,
        checks,
        bridge_fault,
        period_s,
    }
}

impl ShotRead {
    pub fn said(&self, rail_v: f64, period_s: f64) -> String {
        let s = &self.shot;
        let fit = self.fit.as_ref().map_or("no fit".into(), |f| {
            format!(
                "R {:.3} ohm, L {:.3} mH, L / R {:.2} periods",
                f.r_ohm,
                1e3 * f.l_h,
                f.tau_s / period_s
            )
        });
        format!(
            "burst {} ({}): {}{:.3} V ({:.2}% duty) on a {rail_v:.2} V rail: window current {:.1} mA at the end, tap difference {:.3} V in the window ({:.1}% of the rail, sign {}), {fit}; {:.1} mA off the zero after{}",
            s.k,
            if s.in_fit {
                "in the fit"
            } else {
                "first after a sign change, out of the fit"
            },
            if s.dir > 0 { "+" } else { "-" },
            s.volts,
            100.0 * s.duty_q15.unsigned_abs() as f64 / Q15,
            1e3 * self.i_late_a,
            self.v_on_v,
            100.0 * self.v_on_v / rail_v,
            if self.v_on_sign > 0.0 { "+" } else { "-" },
            1e3 * self.after_a,
            match &self.periods {
                Ok(t) => format!(", {} periods", t.len()),
                Err(e) => format!(", {e}"),
            }
        )
    }
}

/// What the probe takes from the rest step's result.
#[derive(Clone, Debug, PartialEq)]
pub struct FromRest {
    pub zero_counts: f64,
    pub pot_sd: f64,
    pub tap_offset: f64,
    pub pattern: Option<Pattern>,
    pub pos_gain: f64,
}

impl FromRest {
    pub fn of(rest: &Value) -> Result<Self, Refused> {
        if rest["verdict"] != "accepted" {
            return Err(Refused(format!(
                "the probe runs on an accepted rest step; the rest step was {}",
                rest["verdict"].as_str().unwrap_or("not run")
            )));
        }
        let est = |k: &str| {
            rest["estimates"][k]["value"]
                .as_f64()
                .ok_or_else(|| Refused(format!("the rest result holds no {k}")))
        };
        let pattern = rest["frame_ripple"]["vref_pattern_counts"]
            .as_array()
            .filter(|a| a.len() == FOLD)
            .map(|a| {
                let mut p = [0.0; FOLD];
                for (x, v) in p.iter_mut().zip(a) {
                    *x = v.as_f64().unwrap_or(0.0);
                }
                p
            });
        Ok(FromRest {
            zero_counts: est("current_zero_awake_counts")?,
            pot_sd: est("pot_rest_sd_counts")?,
            tap_offset: est("tap_a_counts")? - est("tap_b_counts")?,
            pattern,
            pos_gain: rest["channels"]["raw"]["pos"]["vdd_gain"]
                .as_f64()
                .unwrap_or(0.0),
        })
    }
}

/// What the step reads besides the servo.
pub struct ProbeInputs<'a> {
    pub nameplate: &'a Nameplate,
    pub board: &'a Board,
    pub thermal: Option<&'a ThermalInputs>,
    /// The rest step's `results/rest.json`.
    pub rest: &'a Value,
}

#[derive(Clone, Debug)]
pub struct Probed {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Probed {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
}

/// What the step knew before its first write.
struct Before {
    line: Line,
    servo: Map<String, Value>,
    pos: f64,
    rail_v: f64,
    deadman_ticks: u16,
    polarity: bool,
    rest: FromRest,
    scales: Scales,
    pwm_arr: f64,
    shots: Vec<Shot>,
    env: Envelope,
    facts: Facts,
    stated_v: f64,
    /// The nameplate's speed, deg/s, and the counts a degree is priced at.
    speed_deg_s: f64,
    counts_per_deg: f64,
    soft: [Vec<u8>; 2],
    bridge_min_ticks: f64,
    counts_per_a: f64,
    tick_hz: f64,
    mv_per_count: f64,
    duty_max_q15: i16,
}

fn num(state: &State, d: &osc_client::descriptor::Descriptor, name: &str) -> Result<f64, Refused> {
    let f = field(d, name)?;
    state
        .text(f)
        .as_f64()
        .ok_or_else(|| Refused(format!("the state read holds no {name}")))
}

pub const NOT_METAL: &str = "plastic or unknown gears are not supported by the drive steps yet";

async fn before<P: Pipe>(rig: &mut Rig<'_, P>, i: &ProbeInputs<'_>) -> Result<Before, Refused> {
    let np = i.nameplate;
    np.check_user().map_err(Refused)?;
    if np.gear_class != Some(GearClass::Metal) {
        return Err(Refused(format!(
            "{NOT_METAL}: the nameplate's gear class is {}",
            match np.gear_class {
                Some(GearClass::Plastic) => "plastic",
                _ => "unknown",
            }
        )));
    }
    let thermal = i.thermal.ok_or_else(|| {
        Refused("no thermal values entered (--thermal): the heat of a blocked shaft cannot be priced, so nothing drives".into())
    })?;
    let line = tel::line(rig.c, rig.id).await?;
    line.admit(Layout::Raw)?;
    let rest = FromRest::of(i.rest)?;
    let state = state::read(rig.c, rig.id, rig.d).await?;
    if state.torque {
        return Err(Refused(
            "torque_enable is on: this step starts only with torque off, the shaft at rest".into(),
        ));
    }
    let faults = {
        let f = field(rig.d, "fault_flags")?;
        rig.c
            .read(rig.id, f.addr, 1)
            .await?
            .first()
            .copied()
            .unwrap_or(0)
    };
    if faults != 0 {
        return Err(Refused(format!(
            "a fault is latched (fault_flags {faults:#04x}): `osc status` names it"
        )));
    }
    let b = i.board;
    if !(b.oc_trip_ma.is_finite() && b.oc_trip_ma > 0.0) {
        return Err(Refused(format!(
            "the board's description ({}) states no overcurrent trip: before R is measured a burst is priced blocked at the trip, so part A cannot be priced",
            b.name
        )));
    }
    let edges = b.burst_edges.ok_or_else(|| {
        Refused(format!(
            "the board's description ({}) holds no bridge edge lags (burst_edges): a burst cannot be read",
            b.name
        ))
    })?;
    let bridge_min_ticks = b.bridge_min_ticks.ok_or_else(|| {
        Refused(format!(
            "the board's description ({}) states no bridge_min_ticks: part B starts at the bridge's shortest pulse",
            b.name
        ))
    })?;
    // check_user holds a speed row and the travel
    let speed = np
        .speed_row()
        .ok_or_else(|| Refused("the nameplate has no usable no_load".into()))?;
    let travel_deg = np
        .travel_deg
        .ok_or_else(|| Refused("the nameplate has no usable travel_deg".into()))?;
    let sense = rest::Sense::of(&state, rig.d)?;
    let servo = meta::servo(&state, rig.d, &line);
    let pos = rest::pos_now(rig.c, rig.id, rig.d).await?;
    let rail_v = rest::rail_now(rig.c, rig.id, rig.d, &sense).await?;
    let pwm_arr = num(&state, rig.d, "pwm_period_ticks")?;
    let polarity = state
        .bytes(field(rig.d, "drive_polarity")?)
        .and_then(|b| b.first().copied())
        .ok_or_else(|| Refused("the state read holds no drive_polarity".into()))?
        != 0;
    let duty_max_q15 = num(&state, rig.d, "duty_max_q15")?.clamp(0.0, i16::MAX as f64) as i16;
    let ntc_raw = {
        let f = field(rig.d, "ntc_raw")?;
        let b = rig.c.read(rig.id, f.addr, 2).await?;
        u16::from_le_bytes([b[0], b[1]]) as f64
    };
    let scales = Scales {
        counts_per_a: sense.counts_per_a(),
        terminal_counts_per_v: sense.terminal_counts_per_v(),
        terminal_offset_counts: 0.0,
        window_scale: 1.0,
        lags: Lags {
            on: edges.on_ticks,
            tap_on: edges.tap_on_ticks,
            off: edges.off_ticks,
            settle: edges.settle_ticks,
        },
    };
    let stated_v = speed.at_v;
    let shots = shots(stated_v, rail_v);
    let trip_a = b.oc_trip_ma / 1000.0;
    let stated = np
        .stall_row()
        .and_then(|s| s.current_a.map(|a| (s.at_v, a)));
    let mut facts = Facts::new(np, rail_v);
    facts.ntc_c = Some(sense.ntc.c_of_raw(ntc_raw));
    // the heat a blocked amp makes rises with R: the highest R any burst
    // is priced at
    let r_heat = shots
        .iter()
        .map(|s| first_r(s.volts, trip_a, stated).ohm)
        .fold(R_FLOOR_OHM, f64::max);
    let heat = HeatModel::new(b, thermal, r_heat, sense.counts_per_a(), sense.tick_hz);
    let mut env = Envelope::new(facts.clone(), heat);
    for s in &shots {
        env.facts.r = Some(first_r(s.volts, trip_a, stated));
        env.admit(&Rung {
            name: format!("burst {}", s.k),
            duty: s.duty_q15 as f64 / Q15,
            time_s: BURST_S,
            permit: false,
            gap_s: GAP_S,
            start: pos,
            speed_cps: 0.0,
            stop_counts: 0.0,
            kind: Kind::Drive,
        })?;
    }
    let deadman = (rest::DEADMAN_S * sense.tick_hz).round();
    if !(1.0..=u16::MAX as f64).contains(&deadman) {
        return Err(Refused(format!(
            "a {} s dead-man is no u16 of ticks at {} Hz",
            rest::DEADMAN_S,
            sense.tick_hz
        )));
    }
    let soft = [
        state
            .bytes(field(rig.d, "pos_min_phys_counts")?)
            .map(<[u8]>::to_vec),
        state
            .bytes(field(rig.d, "pos_max_phys_counts")?)
            .map(<[u8]>::to_vec),
    ];
    let [Some(lo), Some(hi)] = soft else {
        return Err(Refused("the state read holds no physical range".into()));
    };
    Ok(Before {
        line,
        servo,
        pos,
        rail_v,
        deadman_ticks: deadman as u16,
        polarity,
        rest,
        scales,
        pwm_arr,
        shots,
        env,
        facts,
        stated_v,
        speed_deg_s: speed.per_deg / speed.seconds,
        counts_per_deg: ADC_COUNTS / travel_deg,
        soft: [lo, hi],
        bridge_min_ticks,
        counts_per_a: sense.counts_per_a(),
        tick_hz: sense.tick_hz,
        mv_per_count: sense.vdd_mv / ADC_COUNTS * (sense.vbus_div_top + sense.vbus_div_bot)
            / sense.vbus_div_bot,
        duty_max_q15,
    })
}

/// What the body found, as far as it got.
#[derive(Default)]
struct Found {
    reads: Vec<(Read, (f64, f64))>,
    part_a: Option<PartA>,
    r_price: Option<f64>,
    ladder: Option<Ladder>,
    ceiling: Option<String>,
    corrected: Option<bool>,
}

async fn shot<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    s: &Shot,
) -> Result<(Read, (f64, f64)), Halt> {
    let pre = Pre {
        pre_q15: 0,
        vbus_raw: read_u16(rig, "vbus_raw").await?,
        bias: read_u16(rig, "current_bias_counts").await?,
        vmotor_bias: read_u16(rig, "vmotor_bias_counts").await?,
        pos: read_u16(rig, "pos").await?,
        seated: false,
        drive_polarity: b.polarity,
    };
    let t0 = (rig.clock)();
    let cap = match burst::shunt(
        rig.c,
        rig.id,
        rig.d,
        s.duty_q15,
        CHANS_DIFF,
        pre,
        &CaptureCfg::default(),
    )
    .await
    {
        Ok(c) => c,
        Err(burst::Error::Rejected) => {
            let word = |v: &[u8]| i32::from_le_bytes(v[..4].try_into().unwrap_or([0; 4]));
            let seen = ArmSeen {
                vbus_counts: read_u16(rig, "vbus_counts").await?,
                mv_per_count: b.mv_per_count,
                pos: pre.pos,
                soft: (word(&b.soft[0]), word(&b.soft[1])),
                limit_flags: read_u8(rig, "limit_flags").await?,
                fault_flags: read_u8(rig, "fault_flags").await?,
            };
            return Err(Halt::Refused(format!(
                "burst {}: {}",
                s.k,
                burst::rejected(s.duty_q15, &seen)
            )));
        }
        Err(burst::Error::Io(e)) => return Err(Halt::from(e)),
        Err(e) => return Err(Halt::Error(format!("shunt burst {}: {e}", s.k))),
    };
    let t1 = (rig.clock)();
    let after_counts = mean_u16(rig, "current_raw", AFTER_READS).await?;
    let faults = read_u8(rig, "fault_flags").await?;
    if faults != 0 {
        let code = read_u8(rig, "fault_code").await.unwrap_or(0);
        return Err(Halt::Error(format!(
            "a fault latched during burst {}: fault_flags {faults:#04x}, fault_code {code}",
            s.k
        )));
    }
    Ok((
        Read {
            shot: *s,
            cap,
            after_counts,
        },
        (t0, t1),
    ))
}

/// The pricing R from part A's free-rotor fit, and the ladder it builds.
#[allow(clippy::too_many_arguments)]
pub fn ladder_cfg(
    r_plan_free: f64,
    rail_v: f64,
    stated_v: f64,
    gear: Option<GearClass>,
    speed_cps: f64,
    threshold: f64,
    polarity: bool,
    start_q15: i16,
    unwedge_q15: i16,
) -> LadderCfg {
    let r = PRICE_SHARE * r_plan_free;
    let q = |v: f64| (v / rail_v * Q15).floor().clamp(0.0, i16::MAX as f64) as i16;
    LadderCfg {
        rail_v,
        start_q15,
        step_q15: q(polarity::STEP_SHARE * stated_v).max(1),
        ceiling_q15: q(hold_a(gear) / 2.0 * r),
        speed_v: stated_v,
        rated_cps: speed_cps,
        threshold,
        emf_min: polarity::EMF_MIN_COUNTS,
        polarity,
        r_ohm: r,
        // the speed voltage binds a press too: the brushes are why
        unwedge_q15: (gear == Some(GearClass::Metal)).then_some(unwedge_q15.min(q(stated_v))),
    }
}

/// The stored sign found wrong: `drive_polarity` written the other way,
/// volatile, its stored value kept in the ledger so the way out puts it
/// back, and read back.
pub(crate) async fn correct<P: Pipe>(
    rig: &mut Rig<'_, P>,
    ledger: &mut Ledger,
    stored: bool,
    lines: &mut Vec<String>,
) -> Result<bool, Halt> {
    write(rig, "goal_duty", &[0, 0]).await?;
    write(rig, "torque_enable", &[0]).await?;
    ledger.keep(rig.c, rig.id, rig.d, "drive_polarity").await?;
    write(rig, "drive_polarity", &[(!stored) as u8]).await?;
    let back = read_u8(rig, "drive_polarity").await?;
    if back != (!stored) as u8 {
        return Err(Halt::Error(format!(
            "drive_polarity {} did not read back after the correction",
            !stored
        )));
    }
    lines.push(
        "the motor leads are reversed against the stored calibration; the sign is corrected".into(),
    );
    lines.push(format!(
        "drive_polarity {} volatile, read back; the way out puts the stored {stored} back, the corrected sign is in the result",
        !stored
    ));
    Ok(!stored)
}

#[allow(clippy::too_many_arguments)]
async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    i: &ProbeInputs<'_>,
    ledger: &mut Ledger,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    ledger.touched = true;
    write(rig, "torque_enable", &[0]).await?;
    let open_loop = mode_value(rig.d, "OpenLoop")?;
    for name in [
        "mode",
        "current_limit_counts",
        "pos_min_soft_counts",
        "pos_max_soft_counts",
    ] {
        ledger.keep(rig.c, rig.id, rig.d, name).await?;
    }
    write(rig, "current_limit_counts", &[0, 0]).await?;
    let [lo, hi] = b.soft.clone();
    write(rig, "pos_min_soft_counts", &lo).await?;
    write(rig, "pos_max_soft_counts", &hi).await?;
    write(rig, "mode", &[open_loop]).await?;
    write(rig, "goal_duty", &[0, 0]).await?;
    write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
    write(rig, "torque_enable", &[1]).await?;
    lines.push(format!(
        "volatile: the peak cap unset, the soft limits out to the physical range, OpenLoop; dead-man {} ticks ({} s), then torque on at zero duty",
        b.deadman_ticks,
        rest::DEADMAN_S
    ));
    lines.push(format!(
        "part A: {} shunt bursts at {:.0}% and {:.0}% of the stated {} V, both signs, {} s of drive each, {} s apart",
        b.shots.len(),
        100.0 * LEVELS[0],
        100.0 * LEVELS[1],
        b.stated_v,
        BURST_S,
        GAP_S
    ));
    for s in b.shots.clone() {
        interrupted(rig.stop)?;
        let r = shot(rig, b, &s).await?;
        found.reads.push(r);
        rig.c.pause(Duration::from_secs_f64(GAP_S)).await;
    }
    let f = AFacts {
        rail_v: b.rail_v,
        zero_counts: b.rest.zero_counts,
        polarity: b.polarity,
        scales: b.scales,
    };
    let reads: Vec<Read> = found.reads.iter().map(|(r, _)| r.clone()).collect();
    let a = judge(&f, &reads);
    for r in &a.reads {
        lines.push(format!("  {}", r.said(b.rail_v, a.period_s)));
    }
    for c in &a.checks {
        lines.push(format!("  {}", c.said()));
    }
    let planned = a.r_plan_free().cloned();
    let period_s = a.period_s;
    found.part_a = Some(a);
    let Some(p) = planned else {
        lines.push("part A refused: nothing more drives".into());
        return Ok(());
    };
    lines.push(format!(
        "part A: R {:.3} ohm, L {:.3} mH, L / R {:.1} us ({:.2} PWM periods), intercept {:+.3} V, the free rotor's: planning grade, never a set field",
        p.r_ohm,
        1e3 * p.l_h,
        1e6 * p.tau_s,
        p.tau_s / period_s,
        p.v0_v
    ));
    interrupted(rig.stop)?;

    let gear = i.nameplate.gear_class;
    let start_q15 = (b.bridge_min_ticks * Q15 / b.pwm_arr).ceil() as i16;
    let cfg = ladder_cfg(
        p.r_ohm,
        b.rail_v,
        b.stated_v,
        gear,
        b.speed_deg_s * b.counts_per_deg,
        polarity::MOVE_MIN_COUNTS.max(polarity::MOVE_SD * b.rest.pot_sd),
        b.polarity,
        start_q15,
        b.duty_max_q15,
    );
    found.r_price = Some(cfg.r_ohm);
    lines.push(format!(
        "{PRICE_WORDS}: {:.3} ohm, from part A's {:.3} ohm",
        cfg.r_ohm, p.r_ohm
    ));
    let hold = hold_a(gear);
    let ceiling = format!(
        "part B ceiling {:.3} V ({:.2}% duty): {:.3} A blocked at the {PRICE_WORDS} {:.3} ohm, half the {hold} A a metal train may hold a stop",
        cfg.volts(cfg.ceiling_q15),
        100.0 * cfg.ceiling_q15 as f64 / Q15,
        hold / 2.0,
        cfg.r_ohm,
    );
    lines.push(ceiling.clone());
    found.ceiling = Some(ceiling);
    if cfg.ceiling_q15 < cfg.start_q15 {
        return Err(Halt::Refused(format!(
            "the ceiling's {:.3} V is under the bridge's shortest pulse ({} ticks, {:.3} V): no pulse is stall-safe",
            cfg.volts(cfg.ceiling_q15),
            b.bridge_min_ticks,
            cfg.volts(cfg.start_q15)
        )));
    }
    lines.push(format!(
        "part B: pulses of {} s with {} s of coast from {:.3} V up {:.3} V a level to the ceiling, +duty then -duty, the first pair that moves repeated; moving is {:.0} counts (the larger of {} and {} x the rest sd {:.2}); travel priced at {:.0} counts/s at {} V (the nameplate's {:.0} deg/s, {:.2} counts/deg: the converter's {ADC_COUNTS} counts over the nameplate's {:.1} deg travel); back-EMF read over the coast's first {} rows, at least {} counts; a wedged train gets one press each way at {:.2}% duty ({:.3} V, the speed voltage or the servo's duty_max_q15) for at most {} s",
        polarity::PULSE_S,
        polarity::COAST_S,
        cfg.volts(cfg.start_q15),
        cfg.volts(cfg.step_q15),
        cfg.threshold,
        polarity::MOVE_MIN_COUNTS,
        polarity::MOVE_SD,
        b.rest.pot_sd,
        cfg.rated_cps,
        cfg.speed_v,
        b.speed_deg_s,
        b.counts_per_deg,
        ADC_COUNTS / b.counts_per_deg,
        polarity::EMF_ROWS,
        cfg.emf_min,
        100.0 * cfg.unwedge_q15.unwrap_or(0) as f64 / Q15,
        cfg.volts(cfg.unwedge_q15.unwrap_or(0)),
        crate::envelope::UNWEDGE_MAX_S
    ));
    let mut facts = b.facts.clone();
    facts.r = Some(Resistance {
        ohm: cfg.r_ohm,
        source: PRICE_WORDS.into(),
    });
    let heat = match i.thermal {
        Some(t) => HeatModel::new(i.board, t, cfg.r_ohm, b.counts_per_a, b.tick_hz),
        None => Err("no thermal values entered".into()),
    };
    let env = std::mem::replace(
        &mut b.env,
        Envelope::new(b.facts.clone(), Err(String::new())),
    );
    let mut env = env.rebased(facts, heat);
    let comp = Comp {
        pattern: b.rest.pattern,
        pos_gain: b.rest.pos_gain,
        tap_offset: b.rest.tap_offset,
        zero_counts: b.rest.zero_counts,
        counts_per_a: b.counts_per_a,
    };
    let mut ladder = Ladder::new(cfg);
    let ran = polarity::body(
        rig,
        &mut env,
        &mut ladder,
        &comp,
        b.tick_hz,
        &b.servo,
        taken,
        lines,
    )
    .await;
    let proven = match ladder.outcome() {
        Some(Outcome::Proven { stored_right, .. }) => Some(*stored_right),
        _ => None,
    };
    found.ladder = Some(ladder);
    ran?;
    if proven == Some(false) {
        found.corrected = Some(correct(rig, ledger, b.polarity, lines).await?);
    }
    Ok(())
}

/// Keep part A's bursts as osc-lab's `burst-K.csv`, each with its meta.
fn keep_bursts(
    sink: &mut dyn Sink,
    servo: &Map<String, Value>,
    reads: &[(Read, (f64, f64))],
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    for (r, times) in reads {
        let plan = json!({"rung": format!("burst {}", r.shot.k), "dir": r.shot.dir, "level_share": LEVELS[r.shot.level],
                          "volts": r.shot.volts, "duty_q15": r.shot.duty_q15, "in_fit": r.shot.in_fit,
                          "chans": CHANS_DIFF, "after_counts": r.after_counts});
        let m = meta::burst(servo, &r.cap, plan, *times);
        let (csv, _) = meta::keep_burst(sink, STEP, r.shot.k as u32, &r.cap, &m)?;
        lines.push(format!("kept {csv}"));
    }
    Ok(())
}

fn est(value: f64, unit: &str, source: &str) -> Value {
    json!({"value": value, "lo": null, "hi": null, "unit": unit, "source": source})
}

fn rise_json(r: &Rise) -> Value {
    json!({"r_ohm": r.r_ohm, "l_h": r.l_h, "tau_s": r.tau_s, "v0_v": r.v0_v, "pairs": r.pairs,
           "resid_rms_v": r.resid_rms_v(), "resid_v": r.resid_v})
}

/// Step 2 on the servo: part A, then part B, then the way out. Refused with
/// nothing written when it cannot start; once it has written, it ends
/// through the guard whatever happens, and keeps its captures and result.
pub async fn probe<P: Pipe>(rig: &mut Rig<'_, P>, i: &ProbeInputs<'_>) -> Result<Probed, Refused> {
    let mut b = before(rig, i).await?;
    let mut lines = vec![
        format!("{}; bus at {} baud", b.line.said(), b.line.baud.as_hz()),
        format!(
            "position {:.1} counts, rail {:.2} V, stored drive_polarity {}, before anything is written",
            b.pos, b.rail_v, b.polarity
        ),
    ];
    let mut ledger = Ledger::default();
    let mut taken = Vec::new();
    let mut found = Found::default();
    let ran = body(
        rig,
        &mut b,
        i,
        &mut ledger,
        &mut taken,
        &mut found,
        &mut lines,
    )
    .await;
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let after = rest::pos_now(rig.c, rig.id, rig.d).await.ok();
    let kept = keep_all(rig.sink, STEP, &taken, &mut lines)
        .map_err(Halt::from)
        .and_then(|()| keep_bursts(rig.sink, &b.servo, &found.reads, &mut lines));
    if let Some(l) = &found.ladder {
        lines.extend(l.path().iter().map(|p| format!("decision: {p}")));
    }
    lines.push(format!(
        "end position {} counts ({:.1} before); the shaft is left where it is",
        after.map_or("not read".into(), |p| format!("{p:.1}")),
        b.pos
    ));
    let mut checks: Vec<Check> = found
        .part_a
        .as_ref()
        .map_or(Vec::new(), |a| a.checks.clone());
    let mut aborted = None;
    if let Some(l) = &found.ladder {
        checks.push(match l.outcome() {
            Some(Outcome::Proven {
                stored_right,
                pressed,
                volts,
            }) => Check {
                name: "polarity".into(),
                value: format!(
                    "two repeats agree at {volts:.3} V: the stored drive_polarity is {}{}",
                    if *stored_right { "right" } else { "wrong" },
                    pressed.map_or(String::new(), |s| format!(
                        "; the start was at the {} stop",
                        s.word()
                    ))
                ),
                bound: "opposite duties move opposite ways, or one presses a stop".into(),
                verdict: Verdict::Pass,
            },
            Some(Outcome::Refused { kind, why }) => {
                if kind.aborts() {
                    aborted = Some(why.clone());
                }
                Check {
                    name: "polarity".into(),
                    value: why.clone(),
                    bound: "opposite duties move opposite ways, or one presses a stop".into(),
                    verdict: Verdict::Fail,
                }
            }
            None => Check {
                name: "polarity".into(),
                value: "the ladder did not finish".into(),
                bound: "a verdict".into(),
                verdict: Verdict::NotJudged,
            },
        });
        let over: Vec<String> = l
            .pulses()
            .iter()
            .filter(|p| p.seen.peak.abs() > p.planned.priced_counts)
            .map(|p| format!("pulse {}", p.planned.n))
            .collect();
        checks.push(Check {
            name: "every pulse inside its priced travel".into(),
            value: if over.is_empty() {
                format!("{} pulses", l.pulses().len())
            } else {
                over.join(", ")
            },
            bound: "peak travel at most the priced bound".into(),
            verdict: pass(over.is_empty()),
        });
    }
    if !exit.torque_off {
        checks.push(Check {
            name: "torque off at the end".into(),
            value: "torque_enable did not read back 0".into(),
            bound: "reads 0".into(),
            verdict: Verdict::Fail,
        });
    }
    let failed: Vec<String> = checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .map(|c| c.name.clone())
        .collect();
    let (mut way, mut why) = match (&ran, &kept) {
        (Err(h), _) => (h.way(), Some(h.why())),
        (Ok(()), Err(h)) => (Way::Error, Some(h.why())),
        (Ok(()), Ok(())) if aborted.is_some() => (Way::Error, aborted),
        (Ok(()), Ok(())) if !failed.is_empty() => (
            Way::Refused,
            Some(format!("refused by {}", failed.join(", "))),
        ),
        (Ok(()), Ok(())) if found.ladder.as_ref().and_then(Ladder::outcome).is_none() => {
            (Way::Refused, Some("part B did not run".into()))
        }
        _ => (Way::Done, None),
    };
    let result = result_json(&b, &found, &checks, way, &exit, after);
    let mut text = serde_json::to_vec_pretty(&result).unwrap_or_default();
    text.push(b'\n');
    match rig.sink.write(RESULT, &text) {
        Ok(()) => lines.push(format!("result: {}", rig.sink.shown(RESULT))),
        Err(e) => {
            way = Way::Error;
            why = Some(format!("the result was not kept: {e}"));
        }
    }
    Ok(Probed {
        ended: Ended {
            way,
            why,
            exit,
            lines,
        },
        checks,
        result: Some(result),
    })
}

/// The estimates a probe result names: the free rotor's planning values,
/// each named so nothing takes it for the stored one.
pub fn estimates(pooled: Option<&Rise>, r_price: Option<f64>) -> Map<String, Value> {
    let mut m = Map::new();
    if let Some(p) = pooled {
        let src = "part A's period-pair fit with an intercept over the bursts not first after a sign change, nominal scales, the rotor free: planning grade, never a set field nor a rules input";
        m.insert("r_plan_free_ohm".into(), est(p.r_ohm, "ohm", src));
        m.insert("l_plan_free_h".into(), est(p.l_h, "H", src));
        m.insert("tau_plan_free_s".into(), est(p.tau_s, "s", src));
    }
    if let Some(r) = r_price {
        m.insert("r_price_ohm".into(), est(r, "ohm", PRICE_WORDS));
    }
    m
}

fn result_json(
    b: &Before,
    found: &Found,
    checks: &[Check],
    way: Way,
    exit: &guard::Exit,
    after: Option<f64>,
) -> Value {
    let part_a = found.part_a.as_ref().map_or(Value::Null, |a| {
        json!({
            "bursts": a.reads.iter().map(|r| json!({
                "k": r.shot.k, "dir": r.shot.dir, "volts": r.shot.volts, "duty_q15": r.shot.duty_q15, "in_fit": r.shot.in_fit,
                "i_late_a": r.i_late_a, "v_on_v": r.v_on_v, "v_on_sign": r.v_on_sign, "v_mean_v": r.v_mean_v, "after_a": r.after_a,
                "fit": r.fit.as_ref().map_or(Value::Null, rise_json),
                "error": r.periods.as_ref().err(),
            })).collect::<Vec<_>>(),
            "pooled": a.pooled.as_ref().map_or(Value::Null, rise_json),
            "bridge_fault": a.bridge_fault,
            "pwm_period_s": a.period_s,
            "scales": {"counts_per_a": b.scales.counts_per_a, "terminal_counts_per_v": b.scales.terminal_counts_per_v,
                       "terminal_offset_counts": b.scales.terminal_offset_counts, "window_scale": b.scales.window_scale,
                       "lags_ticks": {"on": b.scales.lags.on, "tap_on": b.scales.lags.tap_on, "off": b.scales.lags.off, "settle": b.scales.lags.settle},
                       "source": "the nominal sense block read off the servo and the board's burst_edges; no edge term, no window scale"},
        })
    });
    let pooled = found.part_a.as_ref().and_then(|a| a.pooled.as_ref());
    let part_b = found.ladder.as_ref().map_or(Value::Null, |l| {
        let c = l.cfg();
        json!({
            "ceiling": found.ceiling,
            "ceiling_q15": c.ceiling_q15, "ceiling_v": c.volts(c.ceiling_q15), "start_q15": c.start_q15, "step_q15": c.step_q15,
            "r_price_ohm": c.r_ohm, "unwedge_q15": c.unwedge_q15,
            "threshold_counts": c.threshold, "emf_min_counts": c.emf_min, "rated_cps": c.rated_cps,
            "counts_per_deg": {"value": b.counts_per_deg, "source": "the converter's counts over the nameplate's travel_deg: more than the pot spans, so travel is priced high"},
            "pulses": l.pulses().iter().map(|p| json!({
                "n": p.planned.n, "level": p.planned.level, "repeat": p.planned.repeat, "unwedge": p.planned.unwedge, "dir": p.planned.dir,
                "duty_q15": p.planned.duty_q15, "volts": p.planned.volts, "time_s": p.planned.time_s, "priced_counts": p.planned.priced_counts,
                "start_counts": p.seen.start, "peak_counts": p.seen.peak, "end_counts": p.seen.end, "emf_counts": p.seen.emf,
                "current_peak_a": p.seen.current_peak_a, "emf_agrees": p.emf_agrees, "motion": p.motion.word(),
            })).collect::<Vec<_>>(),
            "path": l.path(),
        })
    });
    let polarity = match found.ladder.as_ref().and_then(Ladder::outcome) {
        Some(Outcome::Proven {
            stored_right,
            pressed,
            ..
        }) => json!({
            "stored": b.polarity,
            "found": if *stored_right { b.polarity } else { !b.polarity },
            "stored_right": stored_right,
            "corrected_volatile": found.corrected,
            "pressed_stop": pressed.map(|s| s.word()),
        }),
        _ => json!({"stored": b.polarity, "found": null}),
    };
    json!({
        "step": STEP,
        "method": METHOD,
        "revision": REVISION,
        "image": b.line.image().name(),
        "capability_flags": b.line.caps,
        "verdict": way.name(),
        "estimates": estimates(pooled, found.r_price),
        "drive_polarity": polarity,
        "part_a": part_a,
        "part_b": part_b,
        "checks": checks.iter().map(Check::json).collect::<Vec<_>>(),
        "rail_v": b.rail_v,
        "position_before_counts": b.pos,
        "position_after_counts": after,
        "exit": exit.lines,
        "torque_off": exit.torque_off,
    })
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use futures_lite::future::block_on;
    use osc_client::descriptor::Descriptor;
    use osc_client::fake::FakePipe;
    use osc_client::{BaudRate, Client, Id};

    use super::*;
    use crate::sink::MemSink;
    use crate::steps::polarity::{Next, Seen};

    /// Leads swapped against a stored sign that says right: the ladder
    /// proves it wrong, the step writes the other sign volatile and says
    /// so, and the way out puts the stored sign back.
    #[test]
    fn swapped_leads_are_reported_and_corrected_volatile() {
        let cfg = ladder_cfg(
            4.2,
            5.0,
            4.8,
            Some(GearClass::Metal),
            13_332.0,
            20.0,
            true,
            738,
            32767,
        );
        let mut l = Ladder::new(cfg.clone());
        while let Next::Pulse(p) = l.next() {
            // the leads swapped: a +bridge lowers the counts
            let bridge = cfg.wiring(p.dir);
            l.feed(Seen {
                start: 2000.0,
                peak: -30.0 * bridge,
                end: -30.0 * bridge,
                emf: 40.0 * bridge,
                current_peak_a: 0.1,
            });
        }
        assert!(
            matches!(
                l.outcome(),
                Some(Outcome::Proven {
                    stored_right: false,
                    ..
                })
            ),
            "{:?}",
            l.path()
        );
        assert!((cfg.r_ohm - 0.9 * 4.2).abs() < 1e-12, "priced at 0.9 x R");

        let mut pipe = FakePipe::new(BaudRate::B3000000, &[1]);
        pipe.seed_calibrated(0);
        let mut c = block_on(Client::connect(pipe)).expect("connect");
        let d = Descriptor::parse(
            &std::fs::read_to_string(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../descriptors/osc-servo/0.1.json"
            ))
            .expect("descriptor"),
        )
        .expect("parses");
        let id = Id::new(1);
        let addr = field(&d, "drive_polarity").expect("field").addr;
        let stored = block_on(c.read(id, addr, 1)).expect("read")[0] != 0;
        let (stop, clock) = (AtomicBool::new(false), || 0.0);
        let mut sink = MemSink::default();
        let mut ledger = Ledger {
            touched: true,
            ..Ledger::default()
        };
        let mut lines = Vec::new();
        let found = {
            let mut rig = Rig {
                c: &mut c,
                id,
                d: &d,
                sink: &mut sink,
                stop: &stop,
                clock: &clock,
            };
            block_on(correct(&mut rig, &mut ledger, stored, &mut lines)).expect("corrects")
        };
        assert_eq!(found, !stored);
        assert_eq!(
            lines[0],
            "the motor leads are reversed against the stored calibration; the sign is corrected"
        );
        assert_eq!(
            block_on(c.read(id, addr, 1)).expect("read"),
            [!stored as u8]
        );
        let exit = block_on(guard::exit(&mut c, id, &d, &ledger));
        assert!(
            exit.lines
                .iter()
                .any(|l| l == "drive_polarity put back, read back"),
            "{exit}"
        );
        assert_eq!(block_on(c.read(id, addr, 1)).expect("read"), [stored as u8]);
        assert!(exit.torque_off && !exit.rebooted);
    }

    /// Before any R a burst is priced at the most current the board's trip,
    /// the 0.5 ohm floor and a stated stall allow.
    #[test]
    fn the_first_bursts_are_priced_at_the_most_current() {
        let at = |v: f64, s: Option<(f64, f64)>| first_r(v, 2.2, s).ohm;
        assert_eq!(at(0.96, None), 0.5, "the floor under the trip");
        assert!((at(1.6, None) - 1.6 / 2.2).abs() < 1e-12, "the trip");
        assert_eq!(at(0.96, Some((4.8, 1.15))), 0.5, "the floor reads more");
        assert!(
            (at(1.6, Some((4.8, 12.0))) - 0.4).abs() < 1e-12,
            "a stated stall over both"
        );
    }

    /// A rest outside the reference's pos band keeps no pattern: the probe
    /// reads its pot rows as they come, never refused for it.
    #[test]
    fn a_rest_without_a_reference_pattern_compensates_nothing() {
        let est = |v: f64| serde_json::json!({"value": v});
        let rest = serde_json::json!({
            "verdict": "accepted",
            "estimates": {
                "current_zero_awake_counts": est(1206.0),
                "pot_rest_sd_counts": est(0.8),
                "tap_a_counts": est(773.0),
                "tap_b_counts": est(772.0),
            },
            "channels": {"raw": {"pos": {"vdd_gain": 0.0}}},
            "frame_ripple": {
                "vref_trusted": false,
                "vref_pattern_counts": null,
                "vref_mean_counts": null,
            },
        });
        let r = FromRest::of(&rest).expect("not refused");
        assert_eq!((r.pattern, r.pos_gain), (None, 0.0));
    }
}
