//! The envelope: every rung is priced before anything of it is written,
//! for three cases at once, against five budgets. A burst cannot be cut
//! short and holds the dead-man, so a rung's length is its only live
//! bound, and a rung that does not fit is refused in plain words.
//!
//! The cases: the shaft is free (it runs at most [`FREE_MARGIN`] x the
//! speed the plan predicts for the rung's time, then its stopping
//! distance); the shaft is blocked (the winding carries D x V_rail / R for
//! the rung's time); the sign is wrong, until polarity is proven (the free
//! case the other way).
//!
//! The budgets: current (a press: the gear class's stop-hold ceiling; a
//! free drive: the nameplate's stall current), duty (no drive above the
//! voltage the nameplate states its speed at), time (the burst counter,
//! and less while the permit is held), travel (inside a guard in from each
//! stop, but a light press with the permit held may end on the stop it
//! drives toward) and temperature (the board NTC, and the winding's rise as the
//! kernel's own estimator predicts it from the user's thermal values, fed
//! every rung's blocked-case heat and the gaps between rungs).

use osc_servo_core::estimator::thermal::{NtcCfg, ThermCfg, WindingTherm};
use osc_servo_core::kernel::DECIM_SLOW;
use osc_servo_core::kernel::phases::loops::limits::{LimitState, tau_shift};

use crate::Board;
use crate::inputs::{Inputs, Measured, Quantity, Source, ThermalInputs, Tuning};
use crate::install::Refused;
use crate::nameplate::{GearClass, Nameplate, Stall};
use crate::rules::units::FAST_TICKS_PER_MEDIUM;
use crate::rules::{Outcome, build};

/// The free case's speed over the plan's prediction.
pub const FREE_MARGIN: f64 = 1.25;
/// The longest burst, s: the `tel_count` field at 20 kHz, rounded down.
pub const BURST_MAX_S: f64 = 3.2;
/// The longest rung while the stall permit is held, s: inside its lease.
pub const PERMIT_MAX_S: f64 = 0.7;
/// The guard in from each stop, a share of the stops' span.
pub const GUARD_SHARE: f64 = 0.03;
pub const NTC_MAX_C: f64 = 60.0;
/// The winding's predicted rise over the run's start a rung may reach.
pub const RISE_MAX_C: f64 = 30.0;
/// The stop-hold ceilings: bench safety ceilings by gear class.
pub const HOLD_PLASTIC_A: f64 = 0.3;
pub const HOLD_METAL_A: f64 = 0.5;
/// The longest unwedge press, s: breaking a wedge takes force, not time,
/// and a stiff shaft freed at mid travel must not run into a stop. The
/// duty ramp takes about 13 ms of it to reach the press's duty.
pub const UNWEDGE_MAX_S: f64 = 0.030;
/// The bus dead-man a host-timed drive runs under, s: the servo cuts the
/// drive this long after the host's last instruction, so a host that
/// stalls after the drive's rows holds it no longer. Such a drive is
/// priced at its own time plus this.
pub const DRIVE_DEADMAN_S: f64 = 0.050;

/// The stop-hold ceiling of a gear class; unknown is plastic.
pub fn hold_a(gear: Option<GearClass>) -> f64 {
    match gear {
        Some(GearClass::Metal) => HOLD_METAL_A,
        _ => HOLD_PLASTIC_A,
    }
}

/// The winding's resistance the blocked case is priced at, and where it
/// came from.
#[derive(Clone, Debug, PartialEq)]
pub struct Resistance {
    pub ohm: f64,
    pub source: String,
}

/// What the envelope knows of this servo now.
#[derive(Clone, Debug)]
pub struct Facts {
    pub rail_v: f64,
    pub r: Option<Resistance>,
    pub stall: Option<Stall>,
    pub gear: Option<GearClass>,
    /// The voltage the nameplate states its no-load speed at.
    pub speed_v: Option<f64>,
    /// The stops, raw counts, once found.
    pub stops: Option<(f64, f64)>,
    pub polarity_proven: bool,
    /// The drive steps found the shaft wedged: neither direction moved at
    /// the stall-safe ceiling. Only then may an unwedge rung run.
    pub wedged: bool,
    pub ntc_c: Option<f64>,
}

impl Facts {
    /// From the nameplate alone: R is its stated stall voltage over its
    /// stated stall current, where it states one.
    pub fn new(np: &Nameplate, rail_v: f64) -> Self {
        let stall = np.stall_row();
        let r = stall.and_then(|s| {
            s.current_a.filter(|a| *a > 0.0).map(|a| Resistance {
                ohm: s.at_v / a,
                source: format!("the nameplate: {} V over its {} A stall", s.at_v, a),
            })
        });
        Facts {
            rail_v,
            r,
            stall,
            gear: np.gear_class,
            speed_v: np.speed_row().map(|r| r.at_v),
            stops: None,
            polarity_proven: false,
            wedged: false,
            ntc_c: None,
        }
    }

    /// The free drive's current bound: the stated stall current, else the
    /// stated stall voltage over R.
    pub fn stall_a(&self) -> Option<f64> {
        let s = self.stall?;
        s.current_a
            .or_else(|| self.r.as_ref().map(|r| s.at_v / r.ohm))
    }

    /// The band a run's position must stay in: the stops less the guard.
    pub fn band(&self) -> Option<(f64, f64)> {
        let (lo, hi) = self.stops?;
        let g = GUARD_SHARE * (hi - lo);
        Some((lo + g, hi - g))
    }
}

/// The kernel's winding estimate run on the host: the rules' encoding of
/// the user's thermal values, the leak shifts the kernel takes, and its
/// heat and estimate steps, a SLOW period at a time.
#[derive(Clone, Debug)]
pub struct HeatModel {
    pub words: [(&'static str, i64); 4],
    rise: [u16; 2],
    shifts: [u8; 2],
    /// The board's NTC curve: `raw_ref`, `t_ref_cc`, `k1_q88`, `k2_q24`.
    ntc: (u16, i16, i16, u16),
    ntc_raw: u16,
    medium_s: f64,
    counts_per_a: f64,
}

const THERMAL: [&str; 4] = [
    "thermal_tau_s",
    "thermal_tau2_s",
    "th_rise_q016",
    "th_rise2_q016",
];

impl HeatModel {
    /// The model for a winding of `r_ohm` on a board reading `counts_per_a`,
    /// or why the rules cannot encode the user's values.
    pub fn new(
        board: &Board,
        thermal: &ThermalInputs,
        r_ohm: f64,
        counts_per_a: f64,
        tick_hz: f64,
    ) -> Result<Self, String> {
        let q = |value: f64, unit: &str| Quantity {
            value,
            lo: None,
            hi: None,
            sd: None,
            n: None,
            unit: unit.into(),
            source: Source::default(),
        };
        let inputs = Inputs {
            board: board.clone(),
            measured: Measured {
                r_ohm: Some(q(r_ohm, "ohm")),
                current_counts_per_a: Some(q(counts_per_a, "counts/A")),
                ..Measured::default()
            },
            nameplate: Nameplate::default(),
            policy: Default::default(),
            thermal: thermal.clone(),
            tuning: Tuning::default(),
        };
        let set = build(&inputs);
        let mut words = [("", 0i64); 4];
        for (w, name) in words.iter_mut().zip(THERMAL) {
            *w = match set.get(name) {
                Some(Outcome::Bound(e)) => (name, e.value),
                Some(Outcome::Held(m)) => {
                    return Err(format!("{name} has no value: {} is not entered", m.input));
                }
                Some(Outcome::Refused(r)) => return Err(format!("{name}: {}", r.why)),
                None => return Err(format!("the rules bind no {name}")),
            };
        }
        let [(_, tau), (_, tau2), (_, rise), (_, rise2)] = words;
        let c = board.ntc_curve;
        Ok(HeatModel {
            words,
            rise: [rise as u16, if tau2 == 0 { 0 } else { rise2 as u16 }],
            shifts: [tau_shift(tau as u16), tau_shift(tau2 as u16)],
            ntc: (
                c.raw_ref as u16,
                c.t_ref_cc as i16,
                c.k1_q88 as i16,
                c.k2_q24 as u16,
            ),
            ntc_raw: c.raw_ref as u16,
            medium_s: FAST_TICKS_PER_MEDIUM / tick_hz,
            counts_per_a,
        })
    }

    /// The highest rise over the NTC the estimate reads from the start of
    /// segment `from` on, C: `segments` are (amps, seconds) from the run's
    /// start, each a blocked rung or a gap.
    fn peak_rise_c(&self, segments: &[(f64, f64)], from: usize) -> f64 {
        let cfg = ThermCfg {
            rise_q016: self.rise[0],
            rise2_q016: self.rise[1],
            ntc: NtcCfg {
                raw_ref: self.ntc.0,
                t_ref_cc: self.ntc.1,
                k1_q88: self.ntc.2,
                k2_q24: self.ntc.3,
            },
        };
        let mut lim = LimitState::new();
        let mut therm = WindingTherm::new();
        therm.update(self.ntc_raw, lim.heats(), &cfg);
        let (mut n, mut peak) = (0u32, 0i32);
        for (k, &(amps, secs)) in segments.iter().enumerate() {
            let counts = (amps * self.counts_per_a)
                .round()
                .clamp(0.0, u16::MAX as f64) as u16;
            for _ in 0..(secs / self.medium_s).round() as u64 {
                lim.heat_sample(counts);
                n += 1;
                if n == DECIM_SLOW as u32 {
                    n = 0;
                    let mean = lim.take_heat_mean();
                    lim.heat(mean, therm.warm_q16(), self.shifts);
                    therm.update(self.ntc_raw, lim.heats(), &cfg);
                    if k >= from {
                        peak = peak.max(therm.t_cc() as i32 - therm.t_ntc_cc() as i32);
                    }
                }
            }
        }
        peak as f64 / 100.0
    }
}

/// One planned drive: one burst at one duty.
#[derive(Clone, Debug, PartialEq)]
pub struct Rung {
    pub name: String,
    /// Signed share of full duty.
    pub duty: f64,
    pub time_s: f64,
    /// The rung presses a stop with the stall permit held.
    pub permit: bool,
    /// The rest planned after it, s.
    pub gap_s: f64,
    /// The position it starts from, raw counts.
    pub start: f64,
    /// The plan's free speed at this duty, counts/s, and the stopping
    /// distance from it, counts.
    pub speed_cps: f64,
    pub stop_counts: f64,
    pub kind: Kind,
}

/// What a rung is for.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Kind {
    Drive,
    /// A press at the speed voltage to free a wedged metal train: admitted
    /// only from the wedged verdict, at most [`UNWEDGE_MAX_S`] and its
    /// [`DRIVE_DEADMAN_S`] with the permit held, one a direction a run.
    /// The heat budget and the speed voltage price it; the press ceiling
    /// and the stall current do not, since it is meant to press harder
    /// than both.
    Unwedge,
}

impl Rung {
    /// A rung that drives nothing: zero duty for `time_s`.
    pub fn still(name: &str, time_s: f64, start: f64) -> Self {
        Rung {
            name: name.into(),
            duty: 0.0,
            time_s,
            permit: false,
            gap_s: 0.0,
            start,
            speed_cps: 0.0,
            stop_counts: 0.0,
            kind: Kind::Drive,
        }
    }
}

/// A rung's price in every budget.
#[derive(Clone, Debug, PartialEq)]
pub struct Price {
    pub volts: f64,
    /// The blocked case's current, A.
    pub current_a: f64,
    /// The free case's worst travel, counts, and where it ends; the wrong
    /// sign's end while polarity is unproven.
    pub travel: f64,
    pub free_end: f64,
    pub wrong_end: Option<f64>,
    /// The winding's predicted peak rise over the run's start, C.
    pub rise_c: f64,
}

impl Price {
    pub fn said(&self, r: &Rung) -> String {
        format!(
            "{}: {:.2} V for {:.2} s, worst-case travel {:.0} counts, blocked {:.3} A, winding rise {:.1} C",
            r.name, self.volts, r.time_s, self.travel, self.current_a, self.rise_c
        )
    }
}

pub struct Envelope {
    pub facts: Facts,
    pub heat: Result<HeatModel, String>,
    /// (amps, seconds) of every admitted rung and gap, the run's start on.
    history: Vec<(f64, f64)>,
    /// The duty signs of the unwedge rungs admitted.
    unwedges: Vec<f64>,
}

impl Envelope {
    pub fn new(facts: Facts, heat: Result<HeatModel, String>) -> Self {
        Envelope {
            facts,
            heat,
            history: Vec::new(),
            unwedges: Vec::new(),
        }
    }

    /// `r`'s price, or every budget it breaks, each under its case.
    pub fn price(&self, r: &Rung) -> Result<Price, Refused> {
        let f = &self.facts;
        let mut no = Vec::new();
        let d = r.duty.abs();
        let volts = d * f.rail_v;
        if !(r.time_s > 0.0 && r.time_s <= BURST_MAX_S) {
            no.push(format!(
                "time: {:.2} s, a burst runs over 0 and at most {BURST_MAX_S} s",
                r.time_s
            ));
        }
        if r.permit && r.time_s > PERMIT_MAX_S {
            no.push(format!(
                "time: {:.2} s with the stall permit held, over its {PERMIT_MAX_S} s",
                r.time_s
            ));
        }
        if let Some(t) = f.ntc_c.filter(|t| *t >= NTC_MAX_C) {
            no.push(format!(
                "temperature: the board NTC reads {t:.1} C, at or over {NTC_MAX_C} C"
            ));
        }
        let current_a = if d == 0.0 {
            0.0
        } else {
            match &f.r {
                Some(res) => volts / res.ohm,
                None => {
                    no.push("blocked shaft: no resistance to price it at: the nameplate states no stall current and R is not measured yet".into());
                    0.0
                }
            }
        };
        let unwedge = r.kind == Kind::Unwedge;
        if unwedge {
            if f.gear != Some(GearClass::Metal) {
                no.push(format!(
                    "unwedge: only a metal train is freed by a hard press, and this one is {}",
                    gear_word(f.gear)
                ));
            }
            if !f.wedged {
                no.push("unwedge: the shaft was not found wedged".into());
            }
            if !r.permit {
                no.push("unwedge: the stall permit is not held for it".into());
            }
            if r.time_s > UNWEDGE_MAX_S + DRIVE_DEADMAN_S {
                no.push(format!(
                    "unwedge: {:.3} s, over the {UNWEDGE_MAX_S} s a press to free the shaft may last and the {DRIVE_DEADMAN_S} s dead-man after it",
                    r.time_s
                ));
            }
            let dir = r.duty.signum();
            if self.unwedges.len() >= 2 || self.unwedges.contains(&dir) {
                no.push("unwedge: a run tries once, one press each way, and this one has".into());
            }
        }
        if d > 0.0 {
            match f.speed_v {
                Some(v) if volts > v => no.push(format!(
                    "duty: {:.1}% of the {:.2} V rail applies {volts:.2} V, over the {v} V the nameplate states its speed at",
                    100.0 * d,
                    f.rail_v
                )),
                Some(_) => {}
                None => no.push("duty: the nameplate states no voltage for its no-load speed".into()),
            }
        }
        if d > 0.0 && !unwedge {
            if r.permit {
                let hold = hold_a(f.gear);
                if current_a > hold {
                    no.push(format!(
                        "blocked shaft: {current_a:.3} A for {:.2} s, over the {hold} A a {} train may hold a stop",
                        r.time_s,
                        gear_word(f.gear)
                    ));
                }
            } else {
                match f.stall_a() {
                    Some(a) if current_a > a => no.push(format!(
                        "blocked shaft: {current_a:.3} A for {:.2} s, over the {a:.3} A stall current of the nameplate",
                        r.time_s
                    )),
                    Some(_) => {}
                    None if f.r.is_some() => no.push("blocked shaft: the nameplate states no stall row".into()),
                    None => {}
                }
            }
        }
        let travel = if d == 0.0 {
            0.0
        } else {
            FREE_MARGIN * r.speed_cps * r.time_s + r.stop_counts
        };
        let dir = r.duty.signum();
        let free_end = r.start + dir * travel;
        let wrong_end = (!f.polarity_proven).then_some(r.start - dir * travel);
        if travel > 0.0 {
            match f.band() {
                Some((lo, hi)) => {
                    let out = |end: f64| end < lo || end > hi;
                    let onto_stop = r.permit
                        && f.polarity_proven
                        && r.kind == Kind::Drive
                        && if dir > 0.0 {
                            free_end > hi
                        } else {
                            free_end < lo
                        };
                    let light = hold_a(f.gear) / 2.0;
                    if onto_stop && current_a > light {
                        no.push(format!(
                            "free shaft: from {:.0} the worst case runs {travel:.0} counts to {free_end:.0}, past the guard {lo:.0}..{hi:.0} onto the stop, with {current_a:.3} A blocked, over the {light} A (half the stop-hold ceiling) a rung may arrive at a stop with",
                            r.start
                        ));
                    } else if !onto_stop && out(free_end) {
                        no.push(format!(
                            "free shaft: from {:.0} the worst case runs {travel:.0} counts to {free_end:.0}, past the guard {lo:.0}..{hi:.0}",
                            r.start
                        ));
                    }
                    if let Some(w) = wrong_end.filter(|w| out(*w)) {
                        no.push(format!(
                            "wrong sign: polarity is not proven, and from {:.0} the worst case the other way runs to {w:.0}, past the guard {lo:.0}..{hi:.0}",
                            r.start
                        ));
                    }
                }
                // before the stops a rung may arrive at one: only a light
                // press, stall-safe by its duty alone, is that safe
                None if unwedge => {}
                None => {
                    let light = hold_a(f.gear) / 2.0;
                    if f.r.is_none() {
                        no.push(
                            "travel: the stops are not found yet, and this rung moves the shaft"
                                .into(),
                        );
                    } else if current_a > light {
                        no.push(format!(
                            "travel: the stops are not found yet, and this rung moves the shaft with {current_a:.3} A blocked, over the {light} A (half the stop-hold ceiling) a rung may arrive at a stop with"
                        ));
                    }
                }
            }
        }
        let mut rise_c = 0.0;
        if current_a > 0.0 {
            match &self.heat {
                Ok(h) => {
                    let mut segs = self.history.clone();
                    segs.push((current_a, r.time_s));
                    rise_c = h.peak_rise_c(&segs, segs.len() - 1);
                    if rise_c > RISE_MAX_C {
                        no.push(format!(
                            "blocked shaft: {current_a:.3} A for {:.2} s raises the winding {rise_c:.1} C over the run's start by the kernel's estimate, over {RISE_MAX_C} C",
                            r.time_s
                        ));
                    }
                }
                Err(e) => no.push(format!(
                    "temperature: the heat of a blocked shaft cannot be priced: {e}"
                )),
            }
        }
        if no.is_empty() {
            Ok(Price {
                volts,
                current_a,
                travel,
                free_end,
                wrong_end,
                rise_c,
            })
        } else {
            Err(Refused(format!(
                "rung {} is refused before any write: {}",
                r.name,
                no.join("; ")
            )))
        }
    }

    /// The same run's envelope on what is known now: the heat of every rung
    /// admitted so far is kept.
    pub fn rebased(self, facts: Facts, heat: Result<HeatModel, String>) -> Self {
        Envelope {
            facts,
            heat,
            history: self.history,
            unwedges: self.unwedges,
        }
    }

    /// Price `r` and, when it fits, keep its heat and the gap after it.
    pub fn admit(&mut self, r: &Rung) -> Result<Price, Refused> {
        let p = self.price(r)?;
        if r.kind == Kind::Unwedge {
            self.unwedges.push(r.duty.signum());
        }
        self.history.push((p.current_a, r.time_s));
        if r.gap_s > 0.0 {
            self.history.push((0.0, r.gap_s));
        }
        Ok(p)
    }

    /// The five budgets in words, with this servo's numbers.
    pub fn budgets(&self) -> Vec<String> {
        let f = &self.facts;
        let free = match (f.stall, f.stall_a()) {
            (Some(s), Some(a)) if s.current_a.is_some() => {
                format!("{a:.3} A, the nameplate's stall at {} V", s.at_v)
            }
            (Some(s), Some(a)) => format!("{a:.3} A, the nameplate's {} V over R", s.at_v),
            (Some(s), None) => format!(
                "the nameplate's {} V over R, once R is measured (the probe and the stops step); until then no drive",
                s.at_v
            ),
            (None, _) => "the nameplate states no stall row: no drive".into(),
        };
        let duty = match f.speed_v {
            Some(v) => format!(
                "no drive above {v} V, the voltage the nameplate states its speed at: {:.0}% of this {:.2} V rail",
                (100.0 * v / f.rail_v).min(100.0),
                f.rail_v
            ),
            None => "the nameplate states no speed voltage: no drive".into(),
        };
        let travel = match (f.stops, f.band()) {
            (Some((lo, hi)), Some((glo, ghi))) => format!(
                "start, worst case and stop inside {glo:.0}..{ghi:.0}, {:.0}% of the stops {lo:.0}..{hi:.0} in from each; with the stall permit held and the sign proven, a rung at most {} A blocked (half the stop-hold ceiling) may end on the stop it drives toward",
                100.0 * GUARD_SHARE,
                hold_a(f.gear) / 2.0
            ),
            _ => format!(
                "the stops are not found yet: only a rung that moves nothing, or one stall-safe by its duty alone (at most {} A blocked, half the stop-hold ceiling)",
                hold_a(f.gear) / 2.0
            ),
        };
        let r = f.r.as_ref().map_or("R unknown".into(), |r| {
            format!("R {:.3} ohm from {}", r.ohm, r.source)
        });
        let heat = match &self.heat {
            Ok(h) => format!(
                "the winding at most {RISE_MAX_C} C over the run's start by the kernel's estimator ({}; {r})",
                h.words
                    .iter()
                    .map(|(n, v)| format!("{n} {v}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
            Err(e) => format!("the winding's rise cannot be priced ({e}): no blocked heat"),
        };
        let ntc = f.ntc_c.map_or(String::new(), |t| format!(", now {t:.1} C"));
        vec![
            format!(
                "current: a press at most {} A ({} train); a free drive at most {free}",
                hold_a(f.gear),
                gear_word(f.gear)
            ),
            format!("duty: {duty}"),
            format!(
                "time: a rung at most {BURST_MAX_S} s (the burst counter), {PERMIT_MAX_S} s while the stall permit is held"
            ),
            format!("travel: {travel}"),
            format!("temperature: the board NTC under {NTC_MAX_C} C{ntc}; {heat}"),
        ]
    }
}

fn gear_word(g: Option<GearClass>) -> &'static str {
    match g {
        Some(GearClass::Metal) => "metal",
        Some(GearClass::Plastic) => "plastic",
        _ => "unknown, taken as plastic,",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nameplate::VoltRange;

    fn board() -> Board {
        serde_json::from_str(include_str!("../boards/osc-dev-v006.json")).expect("board")
    }

    fn thermal() -> ThermalInputs {
        serde_json::from_str(include_str!("../testdata/mg90-d/thermal.json")).expect("thermal")
    }

    fn nameplate() -> Nameplate {
        serde_json::from_str(include_str!("../testdata/mg90-d/nameplate.json")).expect("nameplate")
    }

    /// The bench servo's class: the mg90-d nameplate, R as measured, the
    /// stops found, a 5 V rail.
    fn envelope(polarity_proven: bool) -> Envelope {
        let mut f = Facts::new(&nameplate(), 5.0);
        f.r = Some(Resistance {
            ohm: 4.177,
            source: "a measurement".into(),
        });
        f.stops = Some((200.0, 3900.0));
        f.polarity_proven = polarity_proven;
        let heat = HeatModel::new(&board(), &thermal(), 4.177, 1108.5, 20_000.0);
        Envelope::new(f, heat)
    }

    fn rung(duty: f64, time_s: f64, start: f64, permit: bool, speed_cps: f64) -> Rung {
        Rung {
            name: "probe".into(),
            duty,
            time_s,
            permit,
            gap_s: 1.0,
            start,
            speed_cps,
            stop_counts: 50.0,
            kind: Kind::Drive,
        }
    }

    fn refused(e: &Envelope, r: &Rung) -> String {
        e.price(r).expect_err("refused").0
    }

    #[test]
    fn rung_pricing_free_blocked_and_wrong_sign() {
        let e = envelope(false);
        // fits all three: 0.24 A blocked, 675 counts either way from mid travel
        let p = e
            .price(&rung(0.2, 0.5, 2048.0, false, 1000.0))
            .expect("fits");
        assert_eq!(
            (p.travel, p.free_end, p.wrong_end),
            (675.0, 2723.0, Some(1373.0))
        );
        assert!(
            (p.current_a - 1.0 / 4.177).abs() < 1e-9 && p.rise_c < 2.0,
            "{p:?}"
        );

        // the free case alone: the run toward the top stop passes its guard
        let why = refused(&e, &rung(0.2, 0.5, 3500.0, false, 1000.0));
        assert!(
            why.contains("free shaft") && !why.contains("wrong sign") && !why.contains("blocked"),
            "{why}"
        );

        // the wrong sign alone: the run the other way passes the bottom guard
        let r = rung(0.2, 0.5, 800.0, false, 1000.0);
        let why = refused(&e, &r);
        assert!(
            why.contains("wrong sign") && !why.contains("free shaft") && !why.contains("blocked"),
            "{why}"
        );
        assert!(
            envelope(true).price(&r).is_ok(),
            "a proven sign has no wrong case"
        );

        // the blocked case alone: a press at 1.08 A, over the metal train's 0.5 A
        let why = refused(&e, &rung(0.9, 0.3, 2048.0, true, 100.0));
        assert!(
            why.contains("blocked shaft")
                && why.contains("0.5 A a metal train")
                && !why.contains("free")
                && !why.contains("wrong"),
            "{why}"
        );
    }

    #[test]
    fn the_heat_budget_refuses_a_rung_past_30_c() {
        let mut e = envelope(true);
        // 1.0 A for 3 s, under the 1.149 A of 4.8 V over R and inside every
        // other budget, heats the fast node past 30 C
        let hot = rung(0.835, 3.0, 2048.0, false, 10.0);
        let why = refused(&e, &hot);
        assert!(
            why.contains("raises the winding") && why.contains("over 30 C"),
            "{why}"
        );
        assert_eq!(why.matches("; ").count(), 0, "the heat alone: {why}");

        // a fifth of it fits once, then the slow node fills with gaps of 1 s
        let warm = rung(0.4, 3.0, 2048.0, false, 10.0);
        let mut admitted = 0;
        let last = loop {
            match e.admit(&warm) {
                Ok(p) => {
                    assert!(p.rise_c <= RISE_MAX_C);
                    admitted += 1;
                }
                Err(r) => break r.0,
            }
            assert!(admitted < 100, "never refused");
        };
        assert!(admitted >= 2, "{admitted}");
        assert!(last.contains("raises the winding"), "{last}");
    }

    /// Before the stops a moving rung is admitted only while its blocked
    /// current is at most half the stop-hold ceiling, whatever its travel.
    #[test]
    fn before_stops_only_stall_safe_rungs_move() {
        let mut e = envelope(false);
        e.facts.stops = None;
        // metal: 0.25 A blocked at most; 4.177 ohm on a 5 V rail
        let at = |d: f64| rung(d, 0.15, 2048.0, false, 5000.0);
        let light = 0.25 * 4.177 / 5.0;
        let p = e.price(&at(light * 0.999)).expect("stall-safe by duty");
        assert!(
            p.current_a <= 0.25 && (p.travel - (1.25 * 5000.0 * 0.15 + 50.0)).abs() < 1e-9,
            "{p:?}"
        );
        let why = refused(&e, &at(light * 1.01));
        assert!(
            why.contains("stops are not found") && why.contains("half the stop-hold ceiling"),
            "{why}"
        );
        assert_eq!(why.matches("; ").count(), 0, "the travel alone: {why}");
        e.facts.gear = Some(GearClass::Plastic);
        assert!(refused(&e, &at(light * 0.999)).contains("0.15 A"));
    }

    /// Once the stops are known, a rung with the permit held and the sign
    /// proven may end on the stop it drives toward at a light press; the
    /// same rung without the permit, or with the sign unproven, may not.
    #[test]
    fn a_permit_rung_may_end_on_a_stop_at_a_light_press() {
        let e = envelope(true);
        // 300 counts inside the high stop, driven up for the permit's whole time
        let r = rung(0.04, PERMIT_MAX_S, 3600.0, true, 900.0);
        assert!(e.price(&r).is_ok(), "{}", refused(&e, &r));
        let down = rung(-0.04, PERMIT_MAX_S, 500.0, true, 900.0);
        let p = e.price(&down).expect("onto the low stop");
        assert!(p.free_end < 200.0, "{p:?}");
        let no = refused(&e, &rung(0.04, PERMIT_MAX_S, 3600.0, false, 900.0));
        assert!(no.contains("past the guard"), "{no}");
        let no = refused(&envelope(false), &r);
        assert!(
            no.contains("free shaft: ") && no.contains("past the guard"),
            "{no}"
        );
    }

    #[test]
    fn a_permit_rung_over_half_the_hold_ceiling_may_not_end_on_a_stop() {
        let e = envelope(true);
        // the duty whose blocked current is the whole stop-hold ceiling
        let r_ohm = e.facts.r.as_ref().map(|r| r.ohm).unwrap_or(1.0);
        let duty = hold_a(e.facts.gear) * r_ohm / e.facts.rail_v;
        let no = refused(&e, &rung(duty, PERMIT_MAX_S, 3600.0, true, 900.0));
        assert!(no.contains("half the stop-hold ceiling"), "{no}");
    }

    #[test]
    fn a_rung_without_the_permit_may_not_end_past_a_guard() {
        let e = envelope(true);
        for (duty, start) in [(0.04, 3600.0), (-0.04, 500.0)] {
            let no = refused(&e, &rung(duty, PERMIT_MAX_S, start, false, 900.0));
            assert!(no.contains("past the guard"), "{no}");
            assert_eq!(no.matches("; ").count(), 0, "the travel alone: {no}");
        }
    }

    /// From under the low guard a short rung up ends still under it: the
    /// permit lets a rung onto the stop it drives toward, not the one
    /// behind it.
    #[test]
    fn a_permit_rung_may_not_end_past_the_guard_it_drives_away_from() {
        let e = envelope(true);
        let (lo, hi) = e.facts.band().expect("stops");
        for (duty, start) in [(0.04, lo - 60.0), (-0.04, hi + 60.0)] {
            let r = rung(duty, 0.05, start, true, 100.0);
            let no = refused(&e, &r);
            assert!(
                no.contains("past the guard") && !no.contains("onto the stop"),
                "{no}"
            );
        }
    }

    fn unwedge(duty: f64, time_s: f64) -> Rung {
        Rung {
            name: "unwedge".into(),
            permit: true,
            kind: Kind::Unwedge,
            ..rung(duty, time_s, 2048.0, true, 5000.0)
        }
    }

    /// An unwedge press is admitted only from the wedged verdict, on a metal
    /// train, at most 0.03 s and its 0.05 s dead-man with the permit held,
    /// at most the speed voltage, past the press ceiling and the stall
    /// current; its heat is priced.
    #[test]
    fn an_unwedge_press_is_admitted_only_from_the_wedged_verdict() {
        let mut e = envelope(false);
        e.facts.stops = None;
        e.facts.r = Some(Resistance {
            ohm: 0.9 * 4.177,
            source: "planning R".into(),
        });
        let why = refused(&e, &unwedge(0.96, 0.05));
        assert!(why.contains("not found wedged"), "{why}");
        assert_eq!(why.matches("; ").count(), 0, "the verdict alone: {why}");
        e.facts.wedged = true;
        let why = refused(&e, &unwedge(1.0, 0.05));
        assert!(why.contains("over the 4.8 V"), "full duty on 5 V: {why}");
        let p = e.admit(&unwedge(0.96, 0.05)).expect("a wedged metal train");
        assert!(
            p.current_a > 1.149 && (p.volts - 4.8).abs() < 1e-9,
            "past the hold and the stall: {p:?}"
        );
        assert!(p.rise_c > 0.0, "the heat is priced: {p:?}");
        e.price(&unwedge(-0.96, 0.08))
            .expect("the press and its dead-man");
        assert!(refused(&e, &unwedge(-0.96, 0.09)).contains("over the 0.03 s"));
        let mut held = unwedge(-0.96, 0.05);
        held.permit = false;
        assert!(refused(&e, &held).contains("permit is not held"));
        e.facts.gear = Some(GearClass::Plastic);
        assert!(refused(&e, &unwedge(-0.96, 0.05)).contains("only a metal train"));
    }

    /// One attempt a run: one press each way, then no more.
    #[test]
    fn a_second_unwedge_in_one_run_is_refused() {
        let mut e = envelope(false);
        e.facts.stops = None;
        e.facts.wedged = true;
        e.admit(&unwedge(0.96, 0.05)).expect("+");
        assert!(refused(&e, &unwedge(0.96, 0.05)).contains("tries once"));
        e.admit(&unwedge(-0.96, 0.05)).expect("-");
        for d in [1.0, -1.0] {
            let why = refused(&e, &unwedge(0.96 * d, 0.05));
            assert!(why.contains("a run tries once"), "{why}");
        }
    }

    #[test]
    fn r_comes_from_the_nameplate_before_it_is_measured() {
        let mut np = nameplate();
        let f = Facts::new(&np, 5.0);
        assert_eq!(
            (f.r.clone(), f.stall_a(), f.speed_v),
            (None, None, Some(4.8))
        );
        let e = Envelope::new(f, Err("no R".into()));
        assert!(
            e.price(&Rung::still("rest", 1.0, 2048.0)).is_ok(),
            "a rung that drives nothing needs no R"
        );
        let why = refused(&e, &rung(0.2, 0.5, 2048.0, false, 0.0));
        assert!(
            why.contains("no resistance") && why.contains("stops are not found"),
            "{why}"
        );

        np.stall = vec![crate::nameplate::Stall {
            at_v: 4.8,
            current_a: Some(1.2),
        }];
        np.operating_v = Some(VoltRange { min: 4.8, max: 6.6 });
        let f = Facts::new(&np, 5.0);
        assert_eq!(f.r.as_ref().map(|r| r.ohm), Some(4.0));
        assert_eq!(f.stall_a(), Some(1.2));
    }
}
