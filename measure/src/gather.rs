//! The gather: the results of one run folder's steps to the rules' inputs,
//! `measured.json` and a straight position table, each value naming the
//! result file and key it came from with that file's sha1, and a line for
//! each value and for each input the run does not give. Pure: the results
//! come in as bytes, the files go out as JSON.

use osc_client::pos_lut::{PAGE_POINTS, PAGES};
use serde_json::{Map, Value, json};

use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::set::file_sha1;
use crate::steps::{cruise, probe, rest, stops};

/// The later steps the gather reads where a run has them.
pub const WINDOW_STEP: &str = "window";
pub const WINDOW_RESULT: &str = "results/window.json";
pub const SPINS_STEP: &str = "spins";
pub const SPINS_RESULT: &str = "results/spins.json";
/// The friction line's source where the run has it accepted, else the
/// spins'.
pub const CRUISE_RESULT: &str = cruise::RESULT;
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
}

/// What the gather gives: the rules' `measured.json`, the table file
/// (`pos_lut.json`), and its lines.
#[derive(Clone, Debug)]
pub struct Gathered {
    pub measured: Value,
    pub table: Value,
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

    fn flag(&self, key: &str) -> Result<bool, Refused> {
        at(&self.r.doc, key)
            .and_then(Value::as_bool)
            .ok_or_else(|| self.lacks(key))
    }

    /// A step's estimate as a quantity: its value, its band where it gives
    /// one, its unit, `scale` applied to all three numbers.
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
        Ok(quantity(
            value * scale,
            end("lo").zip(end("hi")),
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

fn quantity(value: f64, band: Option<(f64, f64)>, unit: &str, source: Value) -> Value {
    let mut q = Map::new();
    q.insert("value".into(), value.into());
    if let Some((lo, hi)) = band {
        q.insert("lo".into(), lo.into());
        q.insert("hi".into(), hi.into());
    }
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

/// A quantity's line: its value, band and unit, where it came from.
fn said(name: &str, q: &Value) -> String {
    let n = |k: &str| q.get(k).and_then(Value::as_f64);
    let band = match (n("lo"), n("hi")) {
        (Some(lo), Some(hi)) => format!(" ({} to {})", short(lo), short(hi)),
        _ => String::new(),
    };
    let s = &q["source"];
    format!(
        "{name} {}{band} {}: {} {}",
        short(n("value").unwrap_or(f64::NAN)),
        q["unit"].as_str().unwrap_or(""),
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
            "counts/A",
            stops.source(&format!("{cpa_key} ({nominal})")),
        ),
    );
    o.put(
        "terminal_counts_per_v",
        quantity(
            tcpv,
            None,
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
                    "counts/V",
                    rest.source(&format!("{rail_key} ({parts})")),
                ),
            );
            o.put(
                "rail_offset_counts",
                quantity(
                    0.0,
                    None,
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
        json!({"value": found, "source": probe.source("drive_polarity.found")}),
    );
    o.lines.push(format!(
        "drive_polarity {}: {} drive_polarity.found, {}",
        if found { "on" } else { "off" },
        probe.r.file,
        if found == stored {
            "the stored sign proved right"
        } else {
            "the stored sign proved wrong, flipped"
        }
    ));

    let (lo_key, hi_key) = ("stops.low.median_counts", "stops.high.median_counts");
    let (low, high) = (stops.num(lo_key)?.ceil(), stops.num(hi_key)?.floor());
    if low >= high || low < 0.0 {
        return Err(Refused(format!(
            "{}: the stops read {low} and {high} counts, no travel: {STOPS_RUN}",
            stops.r.file
        )));
    }
    o.m.insert(
        "stops".into(),
        json!({"raw_min": low as i64, "raw_max": high as i64,
               "source": stops.source(&format!("{lo_key} up, {hi_key} down: rounded inward"))}),
    );
    o.lines.push(format!(
        "stops {low} to {high} counts: {} {lo_key} ({}) up, {hi_key} ({}) down",
        stops.r.file,
        stops.num(lo_key)?,
        stops.num(hi_key)?
    ));

    let travel = nameplate
        .travel_deg
        .filter(|t| t.is_finite() && *t > 0.0)
        .ok_or_else(|| {
            Refused(
                "the nameplate states no travel_deg: the table's scale is the stops' span over it"
                    .into(),
            )
        })?;
    let cpd = (high - low) / travel;
    o.m.insert(
        "table".into(),
        json!({"counts_per_deg": cpd, "source": stops.source(&format!(
            "the stops' span, {lo_key} up to {hi_key} down, over the nameplate's travel_deg {travel}"))}),
    );
    o.lines.push(format!(
        "table straight, {cpd:.4} counts/deg: the stops' {} count span over the nameplate's {travel} deg travel",
        high - low
    ));

    let r = stops.num("estimates.r_seated_ohm.value")?;
    let sd_key = "seated.r_sd_within_groups";
    let (low_key, high_key) = (
        "seated.by_stop.low.r_ohm.median",
        "seated.by_stop.high.r_ohm.median",
    );
    let spread = stops.num(sd_key)? * r;
    let half = (stops.num(low_key)? - stops.num(high_key)?).abs() / 2.0;
    let h = spread.max(half);
    o.put(
        "r_ohm",
        quantity(
            r,
            Some((r - h, r + h)),
            "ohm",
            stops.source(&format!(
                "estimates.r_seated_ohm; band +/- the larger of {sd_key} x R ({spread:.4} ohm) and half of {low_key} less {high_key} ({half:.4} ohm)"
            )),
        ),
    );
    let ntc_key = "seated.ntc_c_by_seat";
    let mut seats: Vec<f64> = at(&stops.r.doc, ntc_key)
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_f64).collect())
        .unwrap_or_default();
    let board_c = match median(&mut seats).filter(|c| c.is_finite()) {
        Some(c) => quantity(
            c,
            None,
            "C",
            stops.source(&format!(
                "{ntc_key}, the median: the board NTC read before each seat's bursts"
            )),
        ),
        None => quantity(
            rest.num("estimates.board_ntc_c.value")?,
            None,
            "C",
            rest.source("estimates.board_ntc_c"),
        ),
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
    o.lines.extend([
        "breakaway_ma: not gathered, no step measures it".to_string(),
        "shelf: none, a margin record is no step of a run".into(),
        "speed_kp_verified: not gathered, `osc-ident tune` settles it".into(),
        "wide-code slots: not gathered, the servo keeps the slots it holds".into(),
    ]);

    let mut inputs = Map::new();
    inputs.insert(
        "stops".into(),
        json!({"raw_min": low as i64, "raw_max": high as i64, "from": stops.r.file, "sha1": stops.r.sha1}),
    );
    inputs.insert(
        "travel_deg".into(),
        json!({"value": travel, "from": "the nameplate's travel_deg"}),
    );
    let table = json!({
        "format": TABLE_FORMAT,
        "inputs": inputs,
        "target": {
            "counts_per_deg": cpd,
            "line": "straight between the stops: every correction point 0, the raw count is the table's count",
        },
        "points_by_page": vec![vec![0i16; PAGE_POINTS]; PAGES],
    });
    Ok(Gathered {
        measured: Value::Object(o.m),
        table,
        lines: o.lines,
    })
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
