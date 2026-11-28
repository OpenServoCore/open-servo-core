//! The gather: the results of one run folder's steps to the rules' inputs,
//! `measured.json`, the position table and the wide-code slots, each value
//! naming the result file and key it came from with that file's sha1, and
//! a line for each value and for each input the run does not give. The
//! table and the slots are the table step's when the run has it accepted,
//! built and written together (ident plan sec 2.4); else a straight table
//! and the slots the servo holds. Beside them, the profile's cross-checks
//! (`cross-checks.json`): stated values next to the run's, never judged.
//! Pure: the results come in as bytes, the files go out as JSON.

use osc_client::pos_lut::{PAGE_POINTS, PAGES};
use osc_servo_core::adc::ADC_WIDE_SLOTS;
use serde_json::{Map, Value, json};

use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::ntc::ADC_COUNTS;
use crate::set::file_sha1;
use crate::steps::polarity::PROVING_PAIRS;
use crate::steps::{cruise, probe, rest, stops, table, verify};

/// The later steps the gather reads where a run has them.
pub const WINDOW_STEP: &str = "window";
pub const WINDOW_RESULT: &str = "results/window.json";
pub const SPINS_STEP: &str = "spins";
pub const SPINS_RESULT: &str = "results/spins.json";
/// The friction line's source where the run has it accepted, else the
/// spins'.
pub const CRUISE_RESULT: &str = cruise::RESULT;
/// The table and the slots where the run has the step accepted.
pub const TABLE_RESULT: &str = table::RESULT;
/// The worst 4-step pot run a driven step reads over its drive rows; the
/// gather takes the larger of the spins' and the table's.
pub const POT_STEP4: &str = "pot_step4_counts";
/// Why the cruise line is the one the loop needs over the spins'.
const SPIN_LINE_HIGH: &str = "the spin line reads high at low speed: its spins are short, still speeding up and at the low end of the travel, and its lowest point is a fast spin, so its offset is an extrapolation";

const STOPS_RUN: &str =
    "run `osc-ident step stops`, which runs the rest step and the probe first, into one run folder";
const TABLE_FORMAT: &str = "protocol sec 5.7: 256 i16 calibration points, point k at raw 16 k, point 256 fixed 0 and not written; 8 pages of 32";

/// One step's result file as read: its name in the run folder, the sha1
/// of its bytes, its document.
#[derive(Clone, Debug)]
pub struct StepResult {
    pub file: String,
    pub sha1: String,
    pub doc: Value,
}

impl StepResult {
    pub fn parse(file: &str, bytes: &[u8]) -> Result<Self, Refused> {
        let doc = serde_json::from_slice(bytes)
            .map_err(|e| Refused(format!("{file} is no JSON result: {e}")))?;
        Ok(StepResult {
            file: file.into(),
            sha1: file_sha1(bytes),
            doc,
        })
    }
}

/// A run folder's results, each where the run has it.
#[derive(Clone, Debug, Default)]
pub struct RunResults {
    pub rest: Option<StepResult>,
    pub probe: Option<StepResult>,
    pub stops: Option<StepResult>,
    pub window: Option<StepResult>,
    pub spins: Option<StepResult>,
    pub cruise: Option<StepResult>,
    pub table: Option<StepResult>,
}

/// What the gather gives: the rules' `measured.json`, the table file
/// (`pos_lut.json`), the slots file (`wide-codes.json`) where the table
/// step gives one, the profile's cross-checks (`cross-checks.json`), and
/// its lines.
#[derive(Clone, Debug)]
pub struct Gathered {
    pub measured: Value,
    pub table: Value,
    pub wide_codes: Option<Value>,
    pub cross_checks: Value,
    pub lines: Vec<String>,
}

/// A result the gather reads, with what to run to have it again.
struct Taken<'a> {
    r: &'a StepResult,
    run: &'static str,
}

fn at<'a>(doc: &'a Value, key: &str) -> Option<&'a Value> {
    key.split('.').try_fold(doc, |v, k| v.get(k))
}

impl<'a> Taken<'a> {
    /// The result, refused unless it is the named step's and accepted.
    fn of(
        r: Option<&'a StepResult>,
        step: &str,
        file: &str,
        run: &'static str,
    ) -> Result<Option<Self>, Refused> {
        let Some(r) = r else {
            return Ok(None);
        };
        if let Some(s) = r.doc.get("step").and_then(Value::as_str)
            && s != step
        {
            return Err(Refused(format!(
                "{} is the {s} step's result, not the {step} step's",
                r.file
            )));
        }
        let verdict = r.doc.get("verdict").and_then(Value::as_str).unwrap_or("");
        if verdict != "accepted" {
            return Err(Refused(format!(
                "{} ({file}) is {}, not accepted: {run}",
                r.file,
                if verdict.is_empty() {
                    "of no verdict"
                } else {
                    verdict
                }
            )));
        }
        Ok(Some(Taken { r, run }))
    }

    fn source(&self, key: &str) -> Value {
        json!({"file": self.r.file, "key": key, "sha1": self.r.sha1})
    }

    fn lacks(&self, key: &str) -> Refused {
        Refused(format!(
            "{} holds no usable {key}: {}",
            self.r.file, self.run
        ))
    }

    fn num(&self, key: &str) -> Result<f64, Refused> {
        at(&self.r.doc, key)
            .and_then(Value::as_f64)
            .filter(|v| v.is_finite())
            .ok_or_else(|| self.lacks(key))
    }

    /// A number the result may lack: a spread over too few repeats.
    fn opt(&self, key: &str) -> Option<f64> {
        at(&self.r.doc, key)
            .and_then(Value::as_f64)
            .filter(|v| v.is_finite())
    }

    fn flag(&self, key: &str) -> Result<bool, Refused> {
        at(&self.r.doc, key)
            .and_then(Value::as_bool)
            .ok_or_else(|| self.lacks(key))
    }

    /// A step's estimate as a quantity: its value, its band where it gives
    /// one, its sd and repeat count, its unit, `scale` applied to the
    /// numbers.
    fn estimate(&self, name: &str, scale: f64, unit: Option<&str>) -> Result<Value, Refused> {
        let key = format!("estimates.{name}");
        let e = at(&self.r.doc, &key).ok_or_else(|| self.lacks(&key))?;
        let value = e
            .get("value")
            .and_then(Value::as_f64)
            .filter(|v| v.is_finite())
            .ok_or_else(|| self.lacks(&format!("{key}.value")))?;
        let end = |k: &str| {
            e.get(k)
                .and_then(Value::as_f64)
                .filter(|v| v.is_finite())
                .map(|v| v * scale)
        };
        let unit = unit
            .map(str::to_string)
            .or_else(|| e.get("unit").and_then(Value::as_str).map(str::to_string))
            .ok_or_else(|| self.lacks(&format!("{key}.unit")))?;
        let key = if scale == 1.0 {
            key
        } else {
            format!("{key}, x {scale}")
        };
        let n = e.get("n").and_then(Value::as_u64);
        Ok(quantity(
            value * scale,
            end("lo").zip(end("hi")),
            Spread { sd: end("sd"), n },
            &unit,
            self.source(&key),
        ))
    }
}

#[derive(Default)]
struct Out {
    m: Map<String, Value>,
    lines: Vec<String>,
}

impl Out {
    fn put(&mut self, name: &str, q: Value) {
        self.lines.push(said(name, &q));
        self.m.insert(name.into(), q);
    }
}

/// One sd over `n` repeats; `n` none is a nominal value.
#[derive(Clone, Copy)]
struct Spread {
    sd: Option<f64>,
    n: Option<u64>,
}

const NOMINAL: Spread = Spread { sd: None, n: None };
/// The table step's scale and the travel through it: one pooled fit's
/// cycles from stop to stop.
const ONE_FIT: Spread = Spread {
    sd: None,
    n: Some(1),
};

fn quantity(
    value: f64,
    band: Option<(f64, f64)>,
    spread: Spread,
    unit: &str,
    source: Value,
) -> Value {
    let mut q = Map::new();
    q.insert("value".into(), value.into());
    if let Some((lo, hi)) = band {
        q.insert("lo".into(), lo.into());
        q.insert("hi".into(), hi.into());
    }
    q.insert("sd".into(), spread.sd.into());
    q.insert("n".into(), spread.n.into());
    q.insert("unit".into(), unit.into());
    q.insert("source".into(), source);
    Value::Object(q)
}

fn short(v: f64) -> String {
    if v != 0.0 && v.abs() < 0.01 {
        format!("{v:.4e}")
    } else {
        format!("{v:.4}")
    }
}

/// A quantity's spread as a line says it: `+/- sd, n N`, `n N` where
/// nothing repeated, or `nominal`.
fn spread_said(q: &Value) -> String {
    match (
        q.get("sd").and_then(Value::as_f64),
        q.get("n").and_then(Value::as_u64),
    ) {
        (Some(sd), Some(n)) => format!("+/- {}, n {n}", short(sd)),
        (_, Some(n)) => format!("n {n}"),
        (_, None) => "nominal".into(),
    }
}

/// A quantity's line: its value, unit and spread, where it came from.
fn said(name: &str, q: &Value) -> String {
    let s = &q["source"];
    format!(
        "{name} {} {} {}: {} {}",
        short(q.get("value").and_then(Value::as_f64).unwrap_or(f64::NAN)),
        q["unit"].as_str().unwrap_or(""),
        spread_said(q),
        s["file"].as_str().unwrap_or(""),
        s["key"].as_str().unwrap_or("")
    )
}

fn need<'a>(t: Option<Taken<'a>>, file: &str) -> Result<Taken<'a>, Refused> {
    t.ok_or_else(|| Refused(format!("the run holds no {file}: {STOPS_RUN}")))
}

fn median(v: &mut [f64]) -> Option<f64> {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    match n {
        0 => None,
        _ if n % 2 == 1 => Some(v[n / 2]),
        _ => Some((v[n / 2 - 1] + v[n / 2]) / 2.0),
    }
}

/// The sample sd, none under two values.
fn sd(v: &[f64]) -> Option<f64> {
    if v.len() < 2 {
        return None;
    }
    let m = v.iter().sum::<f64>() / v.len() as f64;
    Some((v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (v.len() - 1) as f64).sqrt())
}

fn same(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs())
}

/// The rules' inputs from a run's results and the servo's nameplate.
/// Refused, naming the step to run, when the rest step, the probe or the
/// stops step has no accepted result, or when any result there is is not
/// accepted, but the cruise step's: the friction line is the cruise
/// step's where it was accepted, else the spin step's, a line saying
/// which.
pub fn gather(run: &RunResults, nameplate: &Nameplate) -> Result<Gathered, Refused> {
    let rest = need(
        Taken::of(run.rest.as_ref(), rest::STEP, rest::RESULT, STOPS_RUN)?,
        rest::RESULT,
    )?;
    let probe = need(
        Taken::of(run.probe.as_ref(), probe::STEP, probe::RESULT, STOPS_RUN)?,
        probe::RESULT,
    )?;
    let stops = need(
        Taken::of(run.stops.as_ref(), stops::STEP, stops::RESULT, STOPS_RUN)?,
        stops::RESULT,
    )?;
    let window = Taken::of(
        run.window.as_ref(),
        WINDOW_STEP,
        WINDOW_RESULT,
        "run the window step again",
    )?;
    let spins = Taken::of(
        run.spins.as_ref(),
        SPINS_STEP,
        SPINS_RESULT,
        "run the spins step again",
    )?;
    let tabled = Taken::of(
        run.table.as_ref(),
        table::STEP,
        TABLE_RESULT,
        "run the table step again",
    )?
    .ok_or_else(|| {
        Refused(format!(
            "the run holds no {TABLE_RESULT}: the stops, the table and its scale are read by the motor's ripple there; run the table step"
        ))
    })?;

    let mut o = Out::default();

    let nominal = "the board's parts, nominal: the sense block the steps fit with";
    let cpa_key = "seated.scales.counts_per_a";
    let tcpv_key = "seated.scales.terminal_counts_per_v";
    let cpa = stops.num(cpa_key)?;
    let tcpv = stops.num(tcpv_key)?;
    let others = [
        (rest.num("counts_per_a_nominal")?, cpa),
        (probe.num("part_a.scales.counts_per_a")?, cpa),
        (probe.num("part_a.scales.terminal_counts_per_v")?, tcpv),
    ];
    if others.iter().any(|&(a, b)| !same(a, b)) {
        return Err(Refused(format!(
            "the rest step, the probe and the stops step read different sense blocks: {STOPS_RUN}"
        )));
    }
    o.put(
        "current_counts_per_a",
        quantity(
            cpa,
            None,
            NOMINAL,
            "counts/A",
            stops.source(&format!("{cpa_key} ({nominal})")),
        ),
    );
    o.put(
        "terminal_counts_per_v",
        quantity(
            tcpv,
            None,
            NOMINAL,
            "counts/V",
            stops.source(&format!("{tcpv_key} ({nominal})")),
        ),
    );
    let rail_key = "rail_counts_per_v_nominal";
    match at(&rest.r.doc, rail_key) {
        None => o.lines.push(format!(
            "rail_counts_per_v, rail_offset_counts: missing, {} holds no {rail_key} (a rest step older than the key)",
            rest.r.file
        )),
        Some(_) => {
            let parts = "the board's parts, nominal";
            o.put(
                "rail_counts_per_v",
                quantity(
                    rest.num(rail_key)?,
                    None,
                    NOMINAL,
                    "counts/V",
                    rest.source(&format!("{rail_key} ({parts})")),
                ),
            );
            o.put(
                "rail_offset_counts",
                quantity(
                    0.0,
                    None,
                    NOMINAL,
                    "counts",
                    rest.source(&format!("{rail_key} ({parts}: the divider has no offset)")),
                ),
            );
        }
    }

    let found = probe.flag("drive_polarity.found")?;
    let stored = probe.flag("drive_polarity.stored")?;
    if stops.flag("drive_polarity.driven")? != found {
        return Err(Refused(format!(
            "the stops step drove on another sign than the probe proved: {STOPS_RUN}"
        )));
    }
    o.m.insert(
        "drive_polarity".into(),
        json!({"value": found, "sd": null, "n": PROVING_PAIRS,
               "source": probe.source("drive_polarity.found")}),
    );
    o.lines.push(format!(
        "drive_polarity {} n {PROVING_PAIRS}: {} drive_polarity.found, {}",
        if found { "on" } else { "off" },
        probe.r.file,
        if found == stored {
            "the stored sign proved right"
        } else {
            "the stored sign proved wrong, flipped"
        }
    ));

    let travel = nameplate
        .rated_travel_deg
        .filter(|t| t.is_finite() && *t > 0.0)
        .ok_or_else(|| {
            Refused(
                "the nameplate states no rated_travel_deg: the soft window is the rated travel"
                    .into(),
            )
        })?;
    let m = MeasuredTable::of(&tabled)?;
    let (low, high) = (m.pins.0 as f64, m.pins.1 as f64);
    let n_stop = m.n.0.min(m.n.1);
    o.m.insert(
        "stops".into(),
        json!({"raw_min": m.pins.0, "raw_max": m.pins.1,
               "contact_counts": [m.contacts.0, m.contacts.1],
               "sd_cycles": [m.sd_cycles.0, m.sd_cycles.1], "n": n_stop,
               "source": tabled.source("stops: low.pin_raw, high.pin_raw, each first contact by the ripple moved outward to the grid; contact_counts each contact in the table's counts; sd over each stop's rungs")}),
    );
    o.lines.push(format!(
        "stops {low} to {high} counts, the first contacts by the ripple at {:.1} and {:.1} table counts, n {} / {}, sd {:.2} / {:.2} cycles: {} stops; the stops step's seats {} up, {} down pace the steps only",
        m.contacts.0,
        m.contacts.1,
        m.n.0,
        m.n.1,
        m.sd_cycles.0,
        m.sd_cycles.1,
        tabled.r.file,
        stops.num("stops.low.median_counts")?,
        stops.num("stops.high.median_counts")?
    ));
    o.m.insert(
        "table".into(),
        json!({"counts_per_deg": m.cpd, "sd": ONE_FIT.sd, "n": ONE_FIT.n,
               "source": tabled.source(&format!("scale.counts_per_deg ({})", m.scale_source))}),
    );
    o.lines.push(format!(
        "table from the table step, {:.4} counts/deg n 1 ({}): {} points_by_page, pinned at {low} and {high}",
        m.cpd, m.scale_source, tabled.r.file
    ));
    let travel_sd = (m.sd_cycles.0.powi(2) + m.sd_cycles.1.powi(2)).sqrt() * m.deg_per_cycle;
    let measured_travel = quantity(
        m.travel_deg,
        None,
        Spread {
            sd: Some(travel_sd),
            n: Some(n_stop),
        },
        "deg",
        tabled.source("stops.travel_deg (contact to contact by the ripple; sd the stops' sds over their rungs in quadrature)"),
    );
    o.m.insert("travel_deg".into(), measured_travel.clone());
    o.lines.push(format!(
        "travel {:.2} deg +/- {:.2} n {n_stop}: contact to contact by the ripple, {} stops.travel_deg",
        m.travel_deg, travel_sd, tabled.r.file
    ));

    let r = stops.num("estimates.r_seated_ohm.value")?;
    let r_spread = Spread {
        sd: stops.opt("estimates.r_seated_ohm.sd"),
        n: at(&stops.r.doc, "estimates.r_seated_ohm.n").and_then(Value::as_u64),
    };
    let sd_key = "seated.r_sd_within_groups";
    let (low_key, high_key) = (
        "seated.by_stop.low.r_ohm.median",
        "seated.by_stop.high.r_ohm.median",
    );
    let spread = stops.opt(sd_key).map(|sd| sd * r);
    let half = stops
        .opt(low_key)
        .zip(stops.opt(high_key))
        .map(|(lo, hi)| (lo - hi).abs() / 2.0);
    let h = match (spread, half) {
        (Some(a), Some(b)) => Some(a.max(b)),
        (a, b) => a.or(b),
    };
    let term = |x: Option<f64>| x.map_or("none".to_string(), |x| format!("{x:.4} ohm"));
    o.put(
        "r_ohm",
        quantity(
            r,
            h.map(|h| (r - h, r + h)),
            r_spread,
            "ohm",
            stops.source(&format!(
                "estimates.r_seated_ohm; band +/- the larger of {sd_key} x R ({}) and half of {low_key} less {high_key} ({}), no band when neither is read",
                term(spread),
                term(half)
            )),
        ),
    );
    let ntc_key = "seated.ntc_c_by_seat";
    let mut seats: Vec<f64> = at(&stops.r.doc, ntc_key)
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_f64).collect())
        .unwrap_or_default();
    let seats_spread = Spread {
        sd: sd(&seats),
        n: Some(seats.len() as u64),
    };
    let board_c = match median(&mut seats).filter(|c| c.is_finite()) {
        Some(c) => quantity(
            c,
            None,
            seats_spread,
            "C",
            stops.source(&format!(
                "{ntc_key}, the median: the board NTC read before each seat's bursts"
            )),
        ),
        None => rest.estimate("board_ntc_c", 1.0, None)?,
    };
    o.put("r_board_c", board_c);
    o.put("l_mh", stops.estimate("l_seated_h", 1000.0, Some("mH"))?);

    match &spins {
        Some(s) => {
            o.put("ke_v_s_per_rad", s.estimate("ke_v_s_per_rad", 1.0, None)?);
            o.put("j_kg_m2", s.estimate("j_kg_m2", 1.0, None)?);
        }
        None => o.lines.push(format!(
            "ke_v_s_per_rad, j_kg_m2: missing, the run holds no {SPINS_RESULT}; run the spins step"
        )),
    }
    match &window {
        Some(w) => o.put("g_window_ms", w.estimate("g_window_ms", 1.0, None)?),
        None => o.lines.push(format!(
            "g_window_ms: missing, the run holds no {WINDOW_RESULT}; run the window step"
        )),
    }
    o.put(
        "pot_rest_sd_counts",
        rest.estimate("pot_rest_sd_counts", 1.0, None)?,
    );
    let cruised = match run.cruise.as_ref() {
        None => Err(format!("the run holds no {CRUISE_RESULT}")),
        Some(r) => {
            if let Some(s) = r.doc.get("step").and_then(Value::as_str)
                && s != cruise::STEP
            {
                return Err(Refused(format!(
                    "{} is the {s} step's result, not the {} step's",
                    r.file,
                    cruise::STEP
                )));
            }
            match r.doc.get("verdict").and_then(Value::as_str).unwrap_or("") {
                "accepted" => Ok(Taken {
                    r,
                    run: "run the cruise step again",
                }),
                v => Err(format!(
                    "the cruise step's {} is {}",
                    r.file,
                    if v.is_empty() { "of no verdict" } else { v }
                )),
            }
        }
    };
    match (cruised, &spins) {
        (Ok(c), _) => {
            let key = "scales.counts_per_a";
            if !same(c.num(key)?, cpa) {
                return Err(Refused(format!(
                    "{} read another sense block than the stops step: run the cruise step on the servo the run measured",
                    c.r.file
                )));
            }
            o.put(
                "cruise_coulomb_ma",
                c.estimate("friction_coulomb_ma", 1.0, None)?,
            );
            o.put(
                "cruise_viscous_ma_s_per_rad",
                c.estimate("friction_viscous_ma_s_per_rad", 1.0, None)?,
            );
            o.lines.push(format!(
                "friction line: from the cruise step's closed-loop cruises at mid travel; the spin step's open-loop line stays in {SPINS_RESULT} as a cross-check, and {SPIN_LINE_HIGH}"
            ));
        }
        (Err(why), Some(s)) => {
            o.put(
                "cruise_coulomb_ma",
                s.estimate("friction_coulomb_ma", 1.0, None)?,
            );
            o.put(
                "cruise_viscous_ma_s_per_rad",
                s.estimate("friction_viscous_ma_s_per_rad", 1.0, None)?,
            );
            o.lines.push(format!(
                "friction line: from the spin step's open-loop spins, as {why}; {SPIN_LINE_HIGH}: the cruise step on a tuned set gives the line the loop needs"
            ));
        }
        (Err(why), None) => o.lines.push(format!(
            "cruise_coulomb_ma, cruise_viscous_ma_s_per_rad: missing, the run holds no {SPINS_RESULT} and {why}"
        )),
    }
    let step4 = [spins.as_ref(), Some(&tabled)]
        .into_iter()
        .flatten()
        .filter_map(|t| {
            let key = format!("estimates.{POT_STEP4}");
            at(&t.r.doc, &key)?;
            Some(
                t.estimate(POT_STEP4, 1.0, None)
                    .map(|q| (q["value"].as_f64(), q)),
            )
        });
    let mut worst: Option<(Option<f64>, Value)> = None;
    for got in step4 {
        let got = got?;
        if worst.as_ref().is_none_or(|w| got.0 > w.0) {
            worst = Some(got);
        }
    }
    match worst {
        Some((_, q)) => o.put(POT_STEP4, q),
        None => o.lines.push(format!(
            "{POT_STEP4}: missing, neither {SPINS_RESULT} nor {TABLE_RESULT} holds it; run the spins step"
        )),
    }
    o.lines.extend([
        "breakaway_ma: not gathered, no step measures it".to_string(),
        "still_speed_cps: not gathered, the tune's accepted check gives it".into(),
        "shelf: none, a margin record is no step of a run".into(),
        "speed_kp_verified: not gathered, `osc-ident tune` settles it".into(),
    ]);
    let table_scale = Ok(quantity(
        m.cpd,
        None,
        ONE_FIT,
        "counts/deg",
        tabled.source(&format!("scale.counts_per_deg ({})", m.scale_source)),
    ));
    let cross_checks = cross_checks(
        nameplate,
        travel,
        Ok(measured_travel),
        table_scale,
        &o.m,
        &mut o.lines,
    );
    o.lines.push(format!(
        "wide-code slots {:?}: from {} slots, written with its table",
        m.slots, tabled.r.file
    ));

    let mut inputs = Map::new();
    inputs.insert(
        "stops".into(),
        json!({"raw_min": m.pins.0, "raw_max": m.pins.1, "from": tabled.r.file, "sha1": tabled.r.sha1}),
    );
    inputs.insert(
        "table".into(),
        json!({"from": tabled.r.file, "sha1": tabled.r.sha1}),
    );
    let table = json!({
        "format": TABLE_FORMAT,
        "inputs": inputs,
        "target": {
            "counts_per_deg": m.cpd,
            "source": tabled.source(&format!("scale.counts_per_deg ({})", m.scale_source)),
            "line": "the table step's ripple table, pinned at each stop's first contact moved outward to the grid, both pins mapping to themselves",
        },
        "points_by_page": m.pages,
        "source": tabled.source("points_by_page"),
    });
    let wide_codes = Some(json!({"slots": m.slots, "source": tabled.source("slots")}));
    Ok(Gathered {
        measured: Value::Object(o.m),
        table,
        wide_codes,
        cross_checks,
        lines: o.lines,
    })
}

/// The largest observer speed over the still holds of an accepted
/// closed-loop check (`results/verify.json` of the tune's accepted try), as
/// a measured term naming that file; none where the check holds none, a
/// check written before it streamed the holds. Refused when the result is
/// not an accepted check.
pub fn still_speed(check: &StepResult) -> Result<Option<Value>, Refused> {
    check_estimate(check, verify::STILL_SPEED)
}

/// The worst 4-step pot run over an accepted closed-loop check's raw
/// bursts, its wide-code holds too, outside the run's pot holes
/// (`results/verify.json` of the tune's accepted try), as a measured term
/// naming that file; none where the check holds none. Refused when the
/// result is not an accepted check.
pub fn check_pot_step4(check: &StepResult) -> Result<Option<Value>, Refused> {
    check_estimate(check, POT_STEP4)
}

fn check_estimate(check: &StepResult, key: &str) -> Result<Option<Value>, Refused> {
    let t = Taken::of(
        Some(check),
        verify::STEP,
        verify::RESULT,
        "run the tune again",
    )?
    .expect("a result was given");
    if at(&t.r.doc, &format!("estimates.{key}")).is_none() {
        return Ok(None);
    }
    t.estimate(key, 1.0, None).map(Some)
}

/// The pot statistic over the whole population, the worst of the
/// gather's (the spins' and the table's) and the check's: the larger
/// term, the gather's on a tie; none where neither holds one.
pub fn worst_pot_step4(gathered: Option<&Value>, check: Option<&Value>) -> Option<Value> {
    let v = |q: &Value| q["value"].as_f64();
    match (gathered, check) {
        (Some(g), Some(c)) if v(c) > v(g) => Some(c.clone()),
        (Some(g), _) => Some(g.clone()),
        (None, c) => c.cloned(),
    }
}

/// `k / x` for the quantity `x`: its sd to first order, its band's ends
/// swapped, its source's key `key`.
fn over(k: f64, x: &Value, unit: &str, key: String) -> Option<Value> {
    let f = |name: &str| x.get(name).and_then(Value::as_f64);
    let v = f("value").filter(|v| *v > 0.0)?;
    let value = k / v;
    let band = f("lo")
        .zip(f("hi"))
        .filter(|(lo, _)| *lo > 0.0)
        .map(|(lo, hi)| (k / hi, k / lo));
    let spread = Spread {
        sd: f("sd").map(|sd| value * sd / v),
        n: x.get("n").and_then(Value::as_u64),
    };
    let mut source = x["source"].clone();
    source["key"] = key.into();
    Some(quantity(value, band, spread, unit, source))
}

/// One cross-check: the stated row's volts and whether the run voltage
/// binds it, where it is a row's; the profile's value (none where it
/// states none), the run's (none where the run does not give it), and the
/// run's over the profile's as a difference in %; its line.
fn check(
    what: &str,
    row: Option<(f64, bool)>,
    stated: Option<(f64, &str, String)>,
    run: Result<Value, String>,
    lines: &mut Vec<String>,
) -> Value {
    let run_v = run
        .as_ref()
        .ok()
        .and_then(|q| q.get("value").and_then(Value::as_f64));
    let diff = match (run_v, &stated) {
        (Some(r), Some((s, _, _))) if *s != 0.0 => Some(100.0 * (r - s) / s),
        _ => None,
    };
    let run_said = match &run {
        Ok(q) => format!(
            "run {} {} {}",
            short(run_v.unwrap_or(f64::NAN)),
            q["unit"].as_str().unwrap_or(""),
            spread_said(q)
        ),
        Err(why) => format!("run none, {why}"),
    };
    let stated_said = match &stated {
        Some((v, unit, _)) => format!("profile {} {unit}", short(*v)),
        None => "profile states none".into(),
    };
    lines.push(match diff {
        Some(d) => format!("cross-check {what}: {run_said}; {stated_said}; {d:+.2} %"),
        None => format!("cross-check {what}: {run_said}; {stated_said}"),
    });
    let mut c = Map::new();
    if let Some((at_v, binds)) = row {
        c.insert("at_v".into(), at_v.into());
        c.insert("binds".into(), binds.into());
    }
    c.insert(
        "stated".into(),
        stated.map_or(
            Value::Null,
            |(v, unit, from)| json!({"value": v, "unit": unit, "from": from}),
        ),
    );
    c.insert("run".into(), run.unwrap_or(Value::Null));
    c.insert("difference_pct".into(), diff.into());
    Value::Object(c)
}

/// The profile's cross-checks (profile-policy sec 3): the travel the
/// ripple measured against `rated_travel_deg`; the price the steps before
/// the table pace their moves at, the converter's counts over the rated
/// travel, against the table's scale; at each stated no-load voltage
/// the free speed V / Ke through the gear ratio against the row; at each
/// stated stall voltage V / seated R against the stated current. Each is
/// reported, none judged, and the rules read none of them.
fn cross_checks(
    np: &Nameplate,
    travel_deg: f64,
    travel: Result<Value, String>,
    table_scale: Result<Value, String>,
    measured: &Map<String, Value>,
    lines: &mut Vec<String>,
) -> Value {
    let travel = check(
        "travel",
        None,
        Some((travel_deg, "deg", "the profile's rated_travel_deg".into())),
        travel,
        lines,
    );
    let priced = ADC_COUNTS / travel_deg;
    let read = table_scale
        .as_ref()
        .ok()
        .and_then(|q| q.get("value").and_then(Value::as_f64));
    let pacing = check(
        "pacing",
        None,
        Some((
            priced,
            "counts/deg",
            format!(
                "the price the steps before the table pace their moves at: the converter's {ADC_COUNTS} counts over the profile's rated_travel_deg {travel_deg}"
            ),
        )),
        table_scale,
        lines,
    );
    lines.push(match read {
        Some(t) if priced >= t => {
            format!("pacing priced {priced:.2} counts/deg; the table reads {t:.2}: conservative")
        }
        Some(t) => format!(
            "pacing priced {priced:.2} counts/deg; the table reads {t:.2}: not conservative, every travel bound before the table was priced short"
        ),
        None => format!(
            "pacing priced {priced:.2} counts/deg; the run holds no {TABLE_RESULT} to read it against"
        ),
    });
    let said = |binds: bool| {
        if binds {
            " (the row the run voltage binds)"
        } else {
            ""
        }
    };
    let ke = measured.get("ke_v_s_per_rad");
    let gear = np.gear_ratio.filter(|g| g.is_finite() && *g > 0.0);
    let free_speed: Vec<Value> = np
        .no_load
        .iter()
        .map(|row| {
            let run = match (ke, gear) {
                (None, _) => Err(format!("the run holds no {SPINS_RESULT}")),
                (_, None) => Err("the nameplate states no gear_ratio".into()),
                (Some(ke), Some(g)) => over(
                    row.at_v * 180.0 / (std::f64::consts::PI * g),
                    ke,
                    "deg/s",
                    format!(
                        "{} V over estimates.ke_v_s_per_rad, motor rad/s over the nameplate's gear_ratio {g} in output deg/s; sd and band through Ke's",
                        row.at_v
                    ),
                )
                .ok_or_else(|| "Ke is not over 0".to_string()),
            };
            let binds = np.speed_row().is_some_and(|r| r.at_v == row.at_v);
            let stated = (row.seconds > 0.0).then(|| {
                (
                    row.per_deg / row.seconds,
                    "deg/s",
                    format!(
                        "the nameplate's no_load at {} V: {} deg in {} s",
                        row.at_v, row.per_deg, row.seconds
                    ),
                )
            });
            check(
                &format!("free speed at {} V, V / Ke{}", row.at_v, said(binds)),
                Some((row.at_v, binds)),
                stated,
                run,
                lines,
            )
        })
        .collect();
    let r = measured.get("r_ohm");
    let stall_current: Vec<Value> = np
        .stall
        .iter()
        .map(|row| {
            let run = r
                .and_then(|r| {
                    over(
                        row.at_v,
                        r,
                        "A",
                        format!(
                            "{} V over estimates.r_seated_ohm, the seated R; sd and band through R's",
                            row.at_v
                        ),
                    )
                })
                .ok_or_else(|| "no seated R over 0".to_string());
            let binds = np.stall_row().is_some_and(|r| r.at_v == row.at_v);
            let stated = row.current_a.map(|a| {
                (
                    a,
                    "A",
                    format!("the nameplate's stall current at {} V", row.at_v),
                )
            });
            check(
                &format!("stall current at {} V, V / R{}", row.at_v, said(binds)),
                Some((row.at_v, binds)),
                stated,
                run,
                lines,
            )
        })
        .collect();
    json!({
        "travel": travel,
        "pacing": pacing,
        "free_speed": free_speed,
        "stall_current": stall_current,
    })
}

/// What the gather takes from an accepted table step.
struct MeasuredTable {
    cpd: f64,
    scale_source: String,
    pages: Vec<Vec<i16>>,
    slots: Vec<u8>,
    /// The pins, each stop's first contact moved outward to the grid.
    pins: (u16, u16),
    /// Each first contact in the table's counts.
    contacts: (f64, f64),
    sd_cycles: (f64, f64),
    n: (u64, u64),
    travel_deg: f64,
    deg_per_cycle: f64,
}

impl MeasuredTable {
    /// The table step's scale, points, slots and stops by the ripple;
    /// refused for a result from before the stops were read so.
    fn of(t: &Taken) -> Result<Self, Refused> {
        let revision = at(&t.r.doc, "revision")
            .and_then(Value::as_u64)
            .unwrap_or(0);
        if revision < 2 {
            return Err(Refused(format!(
                "{} is the table step's revision {revision}, from before the stops were read by the motor's ripple: run the table step again",
                t.r.file
            )));
        }
        let cpd = t.num("scale.counts_per_deg")?;
        let scale_source = at(&t.r.doc, "scale.source")
            .and_then(Value::as_str)
            .ok_or_else(|| t.lacks("scale.source"))?
            .to_string();
        let pages: Vec<Vec<i16>> = at(&t.r.doc, "points_by_page")
            .and_then(Value::as_array)
            .filter(|p| p.len() == PAGES)
            .and_then(|p| {
                p.iter()
                    .map(|page| {
                        let page = page.as_array().filter(|v| v.len() == PAGE_POINTS)?;
                        page.iter()
                            .map(|v| v.as_i64().and_then(|v| i16::try_from(v).ok()))
                            .collect::<Option<Vec<i16>>>()
                    })
                    .collect::<Option<Vec<_>>>()
            })
            .ok_or_else(|| t.lacks("points_by_page"))?;
        let slots: Vec<u8> = at(&t.r.doc, "slots")
            .and_then(Value::as_array)
            .filter(|v| v.len() == ADC_WIDE_SLOTS)
            .and_then(|v| {
                v.iter()
                    .map(|s| s.as_u64().and_then(|s| u8::try_from(s).ok()))
                    .collect::<Option<Vec<u8>>>()
            })
            .ok_or_else(|| t.lacks("slots"))?;
        let pin = |side: &str| -> Result<u16, Refused> {
            let key = format!("stops.{side}.pin_raw");
            u16::try_from(t.num(&key)? as i64).map_err(|_| t.lacks(&key))
        };
        let pins = (pin("low")?, pin("high")?);
        if pins.1 <= pins.0 {
            return Err(Refused(format!(
                "{} pins its table at {} and {}: no travel; run the table step again",
                t.r.file, pins.0, pins.1
            )));
        }
        let n = |side: &str| {
            at(&t.r.doc, &format!("stops.{side}.n"))
                .and_then(Value::as_u64)
                .ok_or_else(|| t.lacks(&format!("stops.{side}.n")))
        };
        Ok(MeasuredTable {
            cpd,
            scale_source,
            pages,
            slots,
            pins,
            contacts: (
                t.num("stops.low.contact_counts")?,
                t.num("stops.high.contact_counts")?,
            ),
            sd_cycles: (
                t.num("stops.low.sd_cycles")?,
                t.num("stops.high.sd_cycles")?,
            ),
            n: (n("low")?, n("high")?),
            travel_deg: t.num("stops.travel_deg")?,
            deg_per_cycle: t.num("stops.deg_per_cycle")?,
        })
    }
}

/// A gathered file as the record's inputs folder writes one: a key a line.
pub fn render(doc: &Value) -> String {
    let Value::Object(m) = doc else {
        return format!("{doc}\n");
    };
    let body: Vec<String> = m
        .iter()
        .map(|(k, v)| format!("  {}: {v}", Value::String(k.clone())))
        .collect();
    format!("{{\n{}\n}}\n", body.join(",\n"))
}
