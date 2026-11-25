//! Step 1, rest and board check: nothing drives. Torque off, the rest row
//! for a second; then torque on at zero duty in OpenLoop, the bridge awake
//! and coasting, the dead-man written first, the raw row for a second.
//! The estimators are medians and spreads, no fit: each channel's mean and
//! standard deviation, the same folded by tick modulo 16 (the frame's own
//! pattern, the supply's read off the rail tap), the asleep and awake
//! current zeros, the rail from `vbus_raw` with the chip supply from the
//! reference's one boot reading (`vref_raw`, a READ) where the servo holds
//! one, the board temperature through the NTC's nominal beta, and the NTC
//! curve's four words designed from the board's parts by the stage 1 rule
//! ([`crate::ntc`]).

use osc_client::descriptor::Descriptor;
use osc_client::{Client, Error, Id, pipe::Pipe};
use serde_json::{Map, Value, json};

use super::{
    Check, Ended, Halt, Rig, Taken, Verdict, interrupted, keep_all, mode_value, take, write,
};
use crate::capture::meta;
use crate::capture::ripple::{self, Pattern};
use crate::capture::rows::{Layout, TelFrame, telemetry::VREF_RAW};
use crate::capture::tel::{self, Burst, Line, StreamCfg};
use crate::envelope::{Envelope, Facts, Rung};
use crate::guard::{self, Ledger, Way};
use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::ntc::{self, Parts, Words};
use crate::state::{self, State, field};

pub const STEP: &str = "rest";
pub const RESULT: &str = "results/rest.json";
/// Rows a capture: a second at 20 kHz.
pub const ROWS: u16 = 20_000;
/// The dead-man the torque-on second runs under.
pub const DEADMAN_S: f64 = 0.5;
const METHOD: &str = "rest-noise";
/// The result's revision: 2 reads the supply's frame pattern off a steady
/// channel (`frame_ripple.pattern_*`, each channel's `pattern_gain`).
pub const REVISION: u32 = 2;
/// The telemetry frame's batch: a pattern in it is the frame's, not the
/// board's.
const FOLD: u64 = 16;
const ADC_TOP: f64 = 4095.0;
/// A count this close to a converter rail is a lead off or a short.
const RAIL_COUNTS: f64 = 8.0;
/// The chip's internal reference, CH32V006 datasheet nominal.
const VREF_V: f64 = 1.2;
/// Reads a position, or the rail tap, is the mean of.
const POS_READS: u32 = 8;

const ZERO_SD_MAX: f64 = 2.0;
const TAP_SD_MAX: f64 = 1.5;
const POT_SD: (f64, f64) = (0.3, 3.0);
const TAPS_APART_MAX: f64 = 5.0;
const BIAS_SHARE: f64 = 0.03;
const VDD_SHARE: f64 = 0.03;
const NTC_C: (f64, f64) = (-10.0, 60.0);
const FOLD_MAX: f64 = 1.0;
/// The most the position may move between the two seconds, counts.
const STILL_MAX: f64 = 5.0;

/// The rest-row channels the supply's frame pattern is read from, first
/// usable first. The rail tap reads a share of VSYS against VDD, so it
/// carries VDD's ripple and not the pot's code; the NTC sits behind the
/// pot's twin RC filter, so over a frame its pin holds and it reads the
/// ripple too.
pub const PATTERN_SOURCES: [&str; 2] = ["vbus_raw", "ntc_raw"];

/// The board's sense chain as the servo's CALIB holds it.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Sense {
    pub shunt_r_mohm: f64,
    pub gain_milli: f64,
    pub vdd_mv: f64,
    pub vmotor_div_top: f64,
    pub vmotor_div_bot: f64,
    pub vbus_div_top: f64,
    pub vbus_div_bot: f64,
    pub rail_drop_mv: f64,
    pub bias_nom: f64,
    pub tick_hz: f64,
    pub ntc: Parts,
    /// The NTC words the servo holds now.
    pub words: Words,
}

impl Sense {
    pub fn of(state: &State, d: &Descriptor) -> Result<Self, Refused> {
        let n = |name: &str| -> Result<f64, Refused> {
            let f = field(d, name)?;
            state
                .text(f)
                .as_f64()
                .ok_or_else(|| Refused(format!("the state read holds no {name}")))
        };
        Ok(Sense {
            shunt_r_mohm: n("shunt_r_mohm")?,
            gain_milli: n("gain_milli")?,
            vdd_mv: n("vdd_mv")?,
            vmotor_div_top: n("vmotor_div_top")?,
            vmotor_div_bot: n("vmotor_div_bot")?,
            vbus_div_top: n("vbus_div_top_ohm")?,
            vbus_div_bot: n("vbus_div_bot_ohm")?,
            rail_drop_mv: n("rail_drop_mv")?,
            bias_nom: n("vmotor_bias_nom_counts")?,
            tick_hz: n("tick_hz")?,
            ntc: Parts {
                pullup_ohm: n("ntc_pullup_ohm")?,
                r25_ohm: n("ntc_r25_ohm")?,
                beta: n("ntc_beta")?,
            },
            words: Words {
                raw_ref: n("ntc_raw_ref")? as u16,
                t_ref_cc: n("ntc_t_ref_cc")? as i16,
                k1_q88: n("ntc_k1_q88")? as i16,
                k2_q24: n("ntc_k2_q24")? as u16,
            },
        })
    }

    /// Shunt counts per amp, the parts' nominal.
    pub fn counts_per_a(&self) -> f64 {
        self.gain_milli / 1000.0 * self.shunt_r_mohm
            / 1000.0
            / (self.vdd_mv / 1000.0 / ntc::ADC_COUNTS)
    }

    /// Counts of terminal tap difference a volt across the motor, the
    /// parts' nominal.
    pub fn terminal_counts_per_v(&self) -> f64 {
        ntc::ADC_COUNTS / (self.vdd_mv / 1000.0) * self.vmotor_div_bot
            / (self.vmotor_div_top + self.vmotor_div_bot)
    }

    /// The chip supply, V, from the internal reference's count.
    pub fn vdd_v(&self, vref_raw: f64) -> f64 {
        VREF_V * ntc::ADC_COUNTS / vref_raw
    }

    /// The rail behind the board's input drop, V, the measured supply in
    /// place of the nominal when a `vref_raw` reading is given.
    pub fn rail_v(&self, vbus_raw: f64, vref_raw: Option<f64>) -> f64 {
        let v = vbus_raw / self.rail_counts_per_v();
        vref_raw.map_or(v, |r| v * self.vdd_v(r) / (self.vdd_mv / 1000.0))
    }

    /// Counts of the rail tap a volt of rail, the parts' nominal: no offset.
    pub fn rail_counts_per_v(&self) -> f64 {
        ntc::ADC_COUNTS / (self.vdd_mv / 1000.0) * self.vbus_div_bot
            / (self.vbus_div_top + self.vbus_div_bot)
    }

    /// The supply at the connector: the rail and the input drop.
    pub fn supply_v(&self, rail_v: f64) -> f64 {
        rail_v + self.rail_drop_mv / 1000.0
    }
}

/// One READ of `vref_raw`, the chip's reference as the servo read it once
/// at boot: the count, or why there is none.
pub async fn boot_vref<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<u16, String> {
    match c.read(id, VREF_RAW.addr, VREF_RAW.width as u16).await {
        Ok(b) => match b.get(..2) {
            Some([0, 0]) => Err("vref_raw reads 0, the servo took no reading at boot".into()),
            Some(&[lo, hi]) => Ok(u16::from_le_bytes([lo, hi])),
            _ => Err(format!("vref_raw came back {} B", b.len())),
        },
        Err(e) => Err(format!("vref_raw was not read ({e})")),
    }
}

/// The rail now, V, from a few reads of the rail tap and the boot reading
/// of the reference, the nominal supply without one.
pub async fn rail_now<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    s: &Sense,
) -> Result<f64, Error> {
    let vref = boot_vref(c, id).await.ok();
    rail_at(c, id, d, s, vref).await
}

async fn rail_at<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    s: &Sense,
    vref: Option<u16>,
) -> Result<f64, Error> {
    let vbus = mean_of(c, id, d, "vbus_raw").await?;
    Ok(s.rail_v(vbus, vref.map(f64::from)))
}

pub(crate) async fn pos_now<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
) -> Result<f64, Error> {
    mean_of(c, id, d, "pos").await
}

/// The mean of [`POS_READS`] reads of the two-byte field `name`.
async fn mean_of<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    name: &str,
) -> Result<f64, Error> {
    let f = field(d, name)?;
    let mut sum = 0.0;
    for _ in 0..POS_READS {
        let b = c.read(id, f.addr, 2).await?;
        sum += u16::from_le_bytes([b[0], b[1]]) as f64;
    }
    Ok(sum / POS_READS as f64)
}

/// A channel's spread over one capture.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Stats {
    pub n: usize,
    pub mean: f64,
    /// Population standard deviation.
    pub sd: f64,
    pub min: f64,
    pub max: f64,
    /// The largest gap of a tick-modulo-16 mean from the mean.
    pub fold: f64,
}

pub fn stats(rows: &[TelFrame], name: &str) -> Option<Stats> {
    stats_of(&ripple::series(rows, name))
}

/// [`Stats`] of a `(tick, value)` series.
pub fn stats_of(v: &[(u64, f64)]) -> Option<Stats> {
    if v.is_empty() {
        return None;
    }
    let n = v.len() as f64;
    let mean = v.iter().map(|(_, x)| x).sum::<f64>() / n;
    let sd = (v.iter().map(|(_, x)| (x - mean).powi(2)).sum::<f64>() / n).sqrt();
    let mut sums = [(0.0, 0usize); FOLD as usize];
    for (t, x) in v {
        let b = &mut sums[(t % FOLD) as usize];
        b.0 += x;
        b.1 += 1;
    }
    let fold = sums
        .iter()
        .filter(|(_, k)| *k > 0)
        .map(|(s, k)| (s / *k as f64 - mean).abs())
        .fold(0.0, f64::max);
    Some(Stats {
        n: v.len(),
        mean,
        sd,
        min: v.iter().map(|(_, x)| *x).fold(f64::INFINITY, f64::min),
        max: v.iter().map(|(_, x)| *x).fold(f64::NEG_INFINITY, f64::max),
        fold,
    })
}

const NO_STATS: Stats = Stats {
    n: 0,
    mean: f64::NAN,
    sd: f64::NAN,
    min: f64::NAN,
    max: f64::NAN,
    fold: f64::NAN,
};

/// One channel of a capture: as received, with the VDD frame ripple taken
/// out, and its gain on the supply's pattern.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Channel {
    pub raw: Stats,
    pub comp: Stats,
    pub gain: f64,
}

const NO_CHANNEL: Channel = Channel {
    raw: NO_STATS,
    comp: NO_STATS,
    gain: 0.0,
};

/// The supply's frame pattern as a steady channel of the rest row reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct SupplyPattern {
    /// The rest-row channel it is read from, one of [`PATTERN_SOURCES`].
    pub source: &'static str,
    /// The channel's tick-modulo-16 means less their mean, in its counts.
    pub counts: Pattern,
    /// The channel's mean, counts.
    pub mean: f64,
}

/// The supply's pattern from the first usable channel of
/// [`PATTERN_SOURCES`], and why each channel before it was passed over:
/// absent, its mean within [`RAIL_COUNTS`] of a converter limit, or a
/// phase missing.
pub fn supply_pattern(rest: &[TelFrame]) -> (Option<SupplyPattern>, Vec<String>) {
    let mut passed = Vec::new();
    for source in PATTERN_SOURCES {
        let s = ripple::series(rest, source);
        let Some(st) = stats_of(&s) else {
            passed.push(format!("the rest row carries no {source}"));
            continue;
        };
        if st.mean <= RAIL_COUNTS || st.mean >= ADC_TOP - RAIL_COUNTS {
            passed.push(format!(
                "{source} reads {:.1}, within {RAIL_COUNTS:.0} counts of a converter limit",
                st.mean
            ));
            continue;
        }
        match ripple::fold(&s) {
            Some(counts) => {
                return (
                    Some(SupplyPattern {
                        source,
                        counts,
                        mean: st.mean,
                    }),
                    passed,
                );
            }
            None => passed.push(format!("{source} misses a tick-modulo-16 phase")),
        }
    }
    (None, passed)
}

/// Both captures' channels and the frame ripple they were compensated by.
#[derive(Clone, Debug)]
pub struct Channels {
    /// The torque-off capture's supply pattern; None when no source is
    /// usable, and then every gain is 0.
    pub pattern: Option<SupplyPattern>,
    /// Why each source before the pattern's, or every one, was passed over.
    pub passed_over: Vec<String>,
    pub rest: Vec<(&'static str, Channel)>,
    pub raw: Vec<(&'static str, Channel)>,
}

pub const REST_CHANNELS: [&str; 6] = [
    "pos",
    "vbus_raw",
    "ntc_raw",
    "current_raw",
    "vmotor_a",
    "vmotor_b",
];
pub const RAW_CHANNELS: [&str; 6] = [
    "pos",
    "current_raw",
    "vmotor_a",
    "vmotor_b",
    "duty_ticks",
    "current_trough",
];

impl Channels {
    /// Every channel of both captures, each compensated by the rest
    /// capture's supply pattern: the raw row carries none of the sources,
    /// and both bursts start a frame at their first row, so the phase is
    /// the same.
    pub fn of(rest: &[TelFrame], raw: &[TelFrame]) -> Self {
        let (pattern, passed_over) = supply_pattern(rest);
        let one = |rows: &[TelFrame], name: &'static str| {
            let s = ripple::series(rows, name);
            let (gain, comp) = match &pattern {
                Some(p) => ripple::fit(&s, &p.counts),
                None => (0.0, s.clone()),
            };
            (
                name,
                Channel {
                    raw: stats_of(&s).unwrap_or(NO_STATS),
                    comp: stats_of(&comp).unwrap_or(NO_STATS),
                    gain,
                },
            )
        };
        Channels {
            rest: REST_CHANNELS.iter().map(|n| one(rest, n)).collect(),
            raw: RAW_CHANNELS.iter().map(|n| one(raw, n)).collect(),
            pattern,
            passed_over,
        }
    }

    /// The step's line on where the pattern came from.
    pub fn said(&self) -> String {
        let passed = self.passed_over.join("; ");
        match &self.pattern {
            Some(p) if passed.is_empty() => {
                format!("the supply's frame pattern from {}", p.source)
            }
            Some(p) => format!("{passed}: the supply's frame pattern from {}", p.source),
            None => format!(
                "no supply frame pattern ({passed}): nothing taken out, every channel's gain 0"
            ),
        }
    }

    pub fn rest(&self, name: &str) -> Channel {
        self.rest
            .iter()
            .find(|(n, _)| *n == name)
            .map_or(NO_CHANNEL, |c| c.1)
    }

    pub fn raw(&self, name: &str) -> Channel {
        self.raw
            .iter()
            .find(|(n, _)| *n == name)
            .map_or(NO_CHANNEL, |c| c.1)
    }

    /// The frame's own pattern a channel keeps once VDD's is out: flat
    /// within a count, every channel, else warned; not judged without a
    /// supply pattern. Never a refusal: the supply's frame
    /// ripple does not keep one shape from capture to capture, so the
    /// torque-on second can keep over a count of it against the rest
    /// second's pattern on a healthy board. A channel no row carried (NaN)
    /// is no flatness.
    pub fn pattern_check(&self) -> Check {
        let (worst, fold) = self
            .rest
            .iter()
            .map(|(n, c)| (format!("rest {n}"), c.comp.fold))
            .chain(
                self.raw
                    .iter()
                    .map(|(n, c)| (format!("raw {n}"), c.comp.fold)),
            )
            .fold((String::new(), 0.0), |a, b| {
                if b.1.is_nan() || b.1 > a.1 { b } else { a }
            });
        let worst = if worst.is_empty() { "none" } else { &worst };
        let bound = format!("flat within {FOLD_MAX} count");
        let Some(p) = &self.pattern else {
            return Check {
                name: "the frame's own pattern".into(),
                value: format!(
                    "no supply frame pattern: tick-modulo-16 means at most {fold:.2} counts off the mean ({worst}), nothing taken out"
                ),
                bound,
                verdict: Verdict::NotJudged,
            };
        };
        Check {
            name: "the frame's own pattern".into(),
            value: format!(
                "tick-modulo-16 means at most {fold:.2} counts off the mean ({worst}) once the VDD frame ripple from {} is out",
                p.source
            ),
            bound,
            verdict: if fold <= FOLD_MAX {
                Verdict::Pass
            } else {
                Verdict::Warn
            },
        }
    }

    pub fn json(&self) -> Value {
        let block = |cs: &[(&'static str, Channel)]| -> Value {
            cs.iter()
                .map(|(n, c)| {
                    (
                        n.to_string(),
                        json!({
                            "n": c.raw.n, "mean": c.raw.mean, "min": c.raw.min, "max": c.raw.max,
                            "sd_raw": c.raw.sd, "sd": c.comp.sd,
                            "fold_16_raw": c.raw.fold, "fold_16": c.comp.fold,
                            "pattern_gain": c.gain,
                        }),
                    )
                })
                .collect::<Map<String, Value>>()
                .into()
        };
        json!({"rest": block(&self.rest), "raw": block(&self.raw)})
    }
}

/// Step 1's outcome once it had written: how it ended, its checks, and
/// the result it kept.
#[derive(Clone, Debug)]
pub struct Rested {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Rested {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
}

/// What the step knew before its first write.
struct Before {
    line: Line,
    sense: Sense,
    servo: Map<String, Value>,
    /// The boot reading of the reference, or why there is none.
    vref: Result<u16, String>,
    pos: f64,
    rail_v: f64,
    deadman_ticks: u16,
    stream: StreamCfg,
    rungs: [Rung; 2],
}

async fn before<P: Pipe>(rig: &mut Rig<'_, P>, np: &Nameplate) -> Result<Before, Refused> {
    let line = tel::line(rig.c, rig.id).await?;
    line.admit(Layout::Rest)?;
    line.admit(Layout::Raw)?;
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
            "a fault is latched (fault_flags {faults:#04x}): every burst would carry it; `osc status` names it"
        )));
    }
    let vref = boot_vref(rig.c, rig.id).await;
    let sense = Sense::of(&state, rig.d)?;
    let servo = meta::servo(&state, rig.d, &line);
    let pos = pos_now(rig.c, rig.id, rig.d).await?;
    let rail_v = rail_at(rig.c, rig.id, rig.d, &sense, vref.as_ref().ok().copied()).await?;
    let span = ROWS as f64 / sense.tick_hz;
    let rungs = [
        Rung::still("rest, torque off", span, pos),
        Rung::still("raw, torque on at zero duty", span, pos),
    ];
    let mut env = Envelope::new(Facts::new(np, rail_v), Err("nothing drives".into()));
    for r in &rungs {
        env.admit(r)?;
    }
    let deadman = (DEADMAN_S * sense.tick_hz).round();
    if !(1.0..=u16::MAX as f64).contains(&deadman) {
        return Err(Refused(format!(
            "a {DEADMAN_S} s dead-man is no u16 of ticks at {} Hz",
            sense.tick_hz
        )));
    }
    let stream = StreamCfg::read(rig.c, rig.id, rig.d).await?;
    Ok(Before {
        line,
        sense,
        servo,
        vref,
        pos,
        rail_v,
        deadman_ticks: deadman as u16,
        stream,
        rungs,
    })
}

fn plan(r: &Rung, layout: Layout) -> Value {
    json!({"rung": r.name, "row": layout.name(), "rows": ROWS, "duty": r.duty, "time_s": r.time_s, "torque": layout == Layout::Raw})
}

async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    ledger: &mut Ledger,
    taken: &mut Vec<Taken>,
    lines: &mut Vec<String>,
) -> Result<(Burst, Burst), Halt> {
    ledger.touched = true;
    write(rig, "torque_enable", &[0]).await?;
    lines.push("torque off, a second of the rest row".into());
    let rest = take(
        rig,
        &b.stream,
        taken,
        Layout::Rest,
        ROWS,
        0,
        &plan(&b.rungs[0], Layout::Rest),
        &b.servo,
        lines,
    )
    .await?;
    interrupted(rig.stop)?;
    let open_loop = mode_value(rig.d, "OpenLoop")?;
    ledger.keep(rig.c, rig.id, rig.d, "mode").await?;
    write(rig, "mode", &[open_loop]).await?;
    write(rig, "goal_duty", &[0, 0]).await?;
    write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
    write(rig, "torque_enable", &[1]).await?;
    lines.push(format!(
        "dead-man {} ticks ({DEADMAN_S} s), then torque on at zero duty in OpenLoop, a second of the raw row",
        b.deadman_ticks
    ));
    let raw = take(
        rig,
        &b.stream,
        taken,
        Layout::Raw,
        ROWS,
        0,
        &plan(&b.rungs[1], Layout::Raw),
        &b.servo,
        lines,
    )
    .await?;
    interrupted(rig.stop)?;
    Ok((rest, raw))
}

/// Step 1 on the servo. Refused with nothing written below 3 Mbps, on a
/// row the servo cannot stream, or with torque on; once it has written,
/// it ends through the guard whatever happens, and keeps its captures and
/// its result.
pub async fn rest<P: Pipe>(
    rig: &mut Rig<'_, P>,
    np: Option<&Nameplate>,
) -> Result<Rested, Refused> {
    let none = Nameplate::default();
    let np_or = np.unwrap_or(&none);
    let b = before(rig, np_or).await?;
    let mut lines = vec![
        format!("{}; bus at {} baud", b.line.said(), b.line.baud.as_hz()),
        format!(
            "position {:.1} counts, rail {:.2} V before anything is written",
            b.pos, b.rail_v
        ),
    ];
    let mut ledger = Ledger::default();
    let mut taken = Vec::new();
    let ran = body(rig, &b, &mut ledger, &mut taken, &mut lines).await;
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let after = pos_now(rig.c, rig.id, rig.d).await.ok();
    let kept = keep_all(rig.sink, STEP, &taken, &mut lines);
    let ran = match (ran, kept) {
        (Err(h), _) => Err(h),
        (Ok(_), Err(e)) => Err(Halt::from(e)),
        (Ok(caps), Ok(())) => Ok(caps),
    };
    let (rest, raw) = match ran {
        Ok(caps) => caps,
        Err(h) => {
            return Ok(Rested {
                ended: Ended {
                    way: h.way(),
                    why: Some(h.why()),
                    exit,
                    lines,
                },
                checks: Vec::new(),
                result: None,
            });
        }
    };
    let j = judge(&b, &rest, &raw, np);
    lines.extend(j.lines);
    let mut checks = j.checks;
    if !exit.torque_off {
        checks.push(Check {
            name: "torque off at the end".into(),
            value: "torque_enable did not read back 0".into(),
            bound: "reads 0".into(),
            verdict: Verdict::Fail,
        });
    }
    lines.push(format!(
        "position {:.1} counts before, {} after ({POS_READS} reads each)",
        b.pos,
        after.map_or("not read".into(), |p| format!("{p:.1}"))
    ));
    let failed: Vec<String> = checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .map(|c| c.name.clone())
        .collect();
    let way = if failed.is_empty() {
        Way::Done
    } else {
        Way::Refused
    };
    let mut result = j.result;
    if let Some(m) = result.as_object_mut() {
        m.insert("verdict".into(), way.name().into());
        m.insert("checks".into(), checks.iter().map(Check::json).collect());
        m.insert("position_before_counts".into(), b.pos.into());
        m.insert(
            "position_after_counts".into(),
            after.map_or(Value::Null, Value::from),
        );
        m.insert("exit".into(), json!(exit.lines));
        m.insert("torque_off".into(), exit.torque_off.into());
    }
    let mut text = serde_json::to_vec_pretty(&result).unwrap_or_default();
    text.push(b'\n');
    let (way, why) = match rig.sink.write(RESULT, &text) {
        Ok(()) => {
            lines.push(format!("result: {}", rig.sink.shown(RESULT)));
            (
                way,
                (!failed.is_empty()).then(|| format!("refused by {}", failed.join(", "))),
            )
        }
        Err(e) => (Way::Error, Some(format!("the result was not kept: {e}"))),
    };
    Ok(Rested {
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

struct Judged {
    checks: Vec<Check>,
    result: Value,
    lines: Vec<String>,
}

fn est(value: f64, unit: &str, source: &str) -> Value {
    json!({"value": value, "lo": null, "hi": null, "unit": unit, "source": source})
}

/// The estimators and every acceptance check of the rest step, pure.
fn judge(b: &Before, rest: &Burst, raw: &Burst, np: Option<&Nameplate>) -> Judged {
    let s = &b.sense;
    let mut checks = Vec::new();
    let mut check = |name: &str, value: String, bound: String, verdict: Verdict| {
        checks.push(Check {
            name: name.into(),
            value,
            bound,
            verdict,
        })
    };
    let pass = |ok: bool| if ok { Verdict::Pass } else { Verdict::Fail };
    let ch = Channels::of(&rest.rows, &raw.rows);
    let mut lines = vec![ch.said()];
    let (r_pos, r_vbus, r_ntc, r_zero, r_a) = (
        ch.rest("pos"),
        ch.rest("vbus_raw").raw,
        ch.rest("ntc_raw").raw,
        ch.rest("current_raw"),
        ch.rest("vmotor_a"),
    );
    let vref = b.vref.as_ref().ok().map(|&v| f64::from(v));
    if let Err(why) = &b.vref {
        lines.push(format!(
            "{why}: no chip supply figure, the rail read through the nominal vdd_mv"
        ));
    }
    let (w_pos, w_zero, w_a, w_b) = (
        ch.raw("pos"),
        ch.raw("current_raw"),
        ch.raw("vmotor_a"),
        ch.raw("vmotor_b"),
    );

    let sd = |x: Channel| {
        format!(
            "sd {:.2} counts ({:.2} before the VDD frame ripple is taken out)",
            x.comp.sd, x.raw.sd
        )
    };
    check(
        "current zero noise, torque off",
        sd(r_zero),
        format!("<= {ZERO_SD_MAX}"),
        pass(r_zero.comp.sd <= ZERO_SD_MAX),
    );
    check(
        "current zero noise, torque on",
        sd(w_zero),
        format!("<= {ZERO_SD_MAX}"),
        pass(w_zero.comp.sd <= ZERO_SD_MAX),
    );
    for (name, x) in [
        ("tap a noise, torque off", r_a),
        ("tap a noise, torque on", w_a),
        ("tap b noise, torque on", w_b),
    ] {
        check(
            name,
            sd(x),
            format!("<= {TAP_SD_MAX}"),
            pass(x.comp.sd <= TAP_SD_MAX),
        );
    }
    check(
        "pot noise at rest",
        sd(r_pos),
        format!("{} to {}", POT_SD.0, POT_SD.1),
        pass((POT_SD.0..=POT_SD.1).contains(&r_pos.comp.sd)),
    );
    let (r_pos, r_zero, r_a, w_pos, w_zero, w_a, w_b) = (
        r_pos.raw, r_zero.raw, r_a.raw, w_pos.raw, w_zero.raw, w_a.raw, w_b.raw,
    );
    let pot_lo = r_pos.min.min(w_pos.min);
    let pot_hi = r_pos.max.max(w_pos.max);
    check(
        "pot off the converter rails",
        format!("{pot_lo:.0} to {pot_hi:.0} counts"),
        format!(
            "over {RAIL_COUNTS:.0} and under {:.0}: a pot lead off reads full scale",
            ADC_TOP - RAIL_COUNTS
        ),
        pass(pot_lo > RAIL_COUNTS && pot_hi < ADC_TOP - RAIL_COUNTS),
    );
    check(
        "taps agree, torque on",
        format!(
            "a {:.1}, b {:.1} counts, {:.1} apart",
            w_a.mean,
            w_b.mean,
            (w_a.mean - w_b.mean).abs()
        ),
        format!("<= {TAPS_APART_MAX} apart"),
        pass((w_a.mean - w_b.mean).abs() <= TAPS_APART_MAX),
    );
    let worst_bias = [r_a.mean, w_a.mean, w_b.mean]
        .into_iter()
        .map(|m| (m - s.bias_nom).abs() / s.bias_nom)
        .fold(0.0, f64::max);
    check(
        "taps at the divider bias",
        format!(
            "a {:.1} / {:.1}, b {:.1} counts, at most {:.2}% off {}",
            r_a.mean,
            w_a.mean,
            w_b.mean,
            100.0 * worst_bias,
            s.bias_nom
        ),
        format!(
            "within {:.0}% of vmotor_bias_nom_counts",
            100.0 * BIAS_SHARE
        ),
        pass(worst_bias <= BIAS_SHARE),
    );
    let vdd_bound = format!("within {:.0}% of vdd_mv {}", 100.0 * VDD_SHARE, s.vdd_mv);
    let vdd = match &b.vref {
        Ok(v) => {
            let vdd = s.vdd_v(f64::from(*v));
            let vdd_off = (vdd * 1000.0 - s.vdd_mv).abs() / s.vdd_mv;
            check(
                "chip supply",
                format!(
                    "{vdd:.3} V from vref_raw {v}, the boot reading, {:.2}% off",
                    100.0 * vdd_off
                ),
                vdd_bound,
                pass(vdd_off <= VDD_SHARE),
            );
            vdd
        }
        Err(why) => {
            check(
                "chip supply",
                format!("{why}; the nominal vdd_mv {} is used", s.vdd_mv),
                vdd_bound,
                Verdict::NotJudged,
            );
            s.vdd_mv / 1000.0
        }
    };
    let ntc_c = s.ntc.c_of_raw(r_ntc.mean);
    let ntc_railed = r_ntc.min <= RAIL_COUNTS || r_ntc.max >= ADC_TOP - RAIL_COUNTS;
    check(
        "board NTC",
        format!(
            "{ntc_c:.1} C from ntc_raw {:.1} (the part's nominal beta)",
            r_ntc.mean
        ),
        format!("{} to {} C, off both rails", NTC_C.0, NTC_C.1),
        pass(!ntc_railed && (NTC_C.0..=NTC_C.1).contains(&ntc_c)),
    );
    let cpa = s.counts_per_a();
    let room_a = (ADC_TOP - w_zero.mean) / cpa;
    let rail = s.rail_v(r_vbus.mean, vref);
    let supply = s.supply_v(rail);
    let stall = np
        .map(|n| Facts::new(n, rail))
        .and_then(|f| f.stall.and_then(|s| s.current_a));
    match stall {
        Some(a) => check(
            "the zero leaves room for the stall current",
            format!(
                "{room_a:.3} A over the zero {:.1} before the chain saturates",
                w_zero.mean
            ),
            format!("the nameplate's {a} A stall"),
            if room_a >= a {
                Verdict::Pass
            } else {
                Verdict::Fail
            },
        ),
        None => check(
            "the zero leaves room for the stall current",
            format!(
                "{room_a:.3} A over the zero {:.1} before the chain saturates",
                w_zero.mean
            ),
            "the nameplate states no stall current".into(),
            Verdict::NotJudged,
        ),
    }
    if let Some(c) = checks.last_mut().filter(|c| c.verdict == Verdict::Fail) {
        c.bound
            .push_str(": this board cannot read this servo's stall current");
    }
    let range = np.and_then(|n| n.operating_v);
    let mut check = |name: &str, value: String, bound: String, verdict: Verdict| {
        checks.push(Check {
            name: name.into(),
            value,
            bound,
            verdict,
        })
    };
    match range {
        Some(op) => check(
            "supply inside the nameplate's range",
            format!(
                "{supply:.2} V at the connector (rail {rail:.2} V and the {} mV input drop)",
                s.rail_drop_mv
            ),
            format!("{} to {} V", op.min, op.max),
            if (op.min..=op.max).contains(&supply) {
                Verdict::Pass
            } else {
                Verdict::Warn
            },
        ),
        None => check(
            "supply inside the nameplate's range",
            format!("{supply:.2} V at the connector"),
            "no nameplate range".into(),
            Verdict::NotJudged,
        ),
    }
    check(
        "VDD frame ripple",
        "no mV figure: vref_raw is one boot reading, no row carries it".into(),
        "reported; every channel's share of it is taken out".into(),
        Verdict::NotJudged,
    );
    checks.push(ch.pattern_check());
    let mut check = |name: &str, value: String, bound: String, verdict: Verdict| {
        checks.push(Check {
            name: name.into(),
            value,
            bound,
            verdict,
        })
    };
    let moved = (w_pos.mean - r_pos.mean).abs();
    check(
        "nothing moved",
        format!(
            "pot mean {:.1} torque off, {:.1} torque on, {moved:.1} apart",
            r_pos.mean, w_pos.mean
        ),
        format!("<= {STILL_MAX} counts"),
        pass(moved <= STILL_MAX),
    );
    let designed = ntc::design(&s.ntc, &ntc::STAGE_1);
    let ntc_words = match &designed {
        Ok(w) => {
            let same = *w == s.words;
            let (under, over) = w.form_error(&s.ntc, ntc::STAGE_1.range_c);
            check(
                "NTC words",
                format!(
                    "designed {} {} {} {}, the servo holds {} {} {} {}",
                    w.raw_ref,
                    w.t_ref_cc,
                    w.k1_q88,
                    w.k2_q24,
                    s.words.raw_ref,
                    s.words.t_ref_cc,
                    s.words.k1_q88,
                    s.words.k2_q24
                ),
                "the servo's equal the design (a difference is the rules' to install)".into(),
                if same { Verdict::Pass } else { Verdict::Warn },
            );
            json!({
                "ntc_raw_ref": w.raw_ref, "ntc_t_ref_cc": w.t_ref_cc, "ntc_k1_q88": w.k1_q88, "ntc_k2_q24": w.k2_q24,
                "servo": {"ntc_raw_ref": s.words.raw_ref, "ntc_t_ref_cc": s.words.t_ref_cc, "ntc_k1_q88": s.words.k1_q88, "ntc_k2_q24": s.words.k2_q24},
                "equal": same,
                "form_error_c": {"under": under, "over": over, "range_c": [ntc::STAGE_1.range_c.0, ntc::STAGE_1.range_c.1]},
                "design": {"pin_c": ntc::STAGE_1.pin_c, "floor_c": ntc::STAGE_1.floor_c, "low_range_c": [ntc::STAGE_1.low_range_c.0, ntc::STAGE_1.low_range_c.1]},
            })
        }
        Err(e) => {
            check(
                "NTC words",
                e.clone(),
                "a curve the design meets".into(),
                Verdict::Fail,
            );
            Value::Null
        }
    };
    let parts = json!({"ntc_pullup_ohm": s.ntc.pullup_ohm, "ntc_r25_ohm": s.ntc.r25_ohm, "ntc_beta": s.ntc.beta});
    let result = json!({
        "step": STEP,
        "method": METHOD,
        "revision": REVISION,
        "image": b.line.image().name(),
        "capability_flags": b.line.caps,
        "estimates": {
            "current_zero_asleep_counts": est(r_zero.mean, "counts", "rest row current_raw, torque off"),
            "current_zero_awake_counts": est(w_zero.mean, "counts", "raw row current_raw, torque on at zero duty"),
            "current_zero_sd_counts": est(ch.rest("current_raw").comp.sd.max(ch.raw("current_raw").comp.sd), "counts", "the larger of the two rows, the VDD frame ripple out"),
            "tap_a_counts": est(w_a.mean, "counts", "raw row vmotor_a, torque on"),
            "tap_b_counts": est(w_b.mean, "counts", "raw row vmotor_b, torque on"),
            "pot_rest_sd_counts": est(ch.rest("pos").comp.sd, "counts", "rest row pos, torque off, the VDD frame ripple out"),
            "vdd_frame_ripple_mv_pp": Value::Null,
            "vdd_v": match vref {
                Some(_) => est(vdd, "V", "vref_raw, the 1.2 V reference read once at boot"),
                None => Value::Null,
            },
            "rail_v": est(rail, "V", if vref.is_some() {
                "rest row vbus_raw through the rail divider, the measured supply"
            } else {
                "rest row vbus_raw through the rail divider, the nominal supply"
            }),
            "supply_v": est(supply, "V", "the rail and the board's input drop"),
            "board_ntc_c": est(ntc_c, "C", "rest row ntc_raw through the part's nominal beta"),
            "stall_room_a": est(room_a, "A", "the awake zero to the converter's top, nominal counts per A"),
        },
        "counts_per_a_nominal": cpa,
        "rail_counts_per_v_nominal": s.rail_counts_per_v(),
        "ntc_parts": parts,
        "ntc_words": ntc_words,
        "channels": ch.json(),
        "frame_ripple": {
            "pattern_source": ch.pattern.as_ref().map(|p| p.source),
            "pattern_counts": ch.pattern.as_ref().map(|p| p.counts.to_vec()),
            "pattern_mean_counts": ch.pattern.as_ref().map(|p| p.mean),
            "pattern_passed_over": ch.passed_over,
            "vdd_ripple_mv_pp": Value::Null,
            "fold": ripple::FOLD,
            "phase": "tick modulo 16 from a burst's first row",
            "compensation": "each channel less its pattern_gain x pattern_counts[tick % 16]",
        },
    });
    Judged {
        checks,
        result,
        lines,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench's frame ripple on the reference, counts: about 3 pp.
    fn vdd(k: u64) -> f64 {
        let x = 2.0 * core::f64::consts::PI * (k % 16) as f64 / 16.0;
        -1.6 * x.cos() - 0.6 * (2.0 * x).sin()
    }

    fn noise(i: u64, k: u64) -> f64 {
        let h =
            (i.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ k.wrapping_mul(0xC2B2_AE3D_27D4_EB4F)) >> 13;
        (h % 7) as f64 - 3.0
    }

    /// Rows of both captures: every channel follows the reference's
    /// ripple at the bench's gains, and `extra` adds a frame pattern of
    /// its own to the rest row's pos.
    fn rows(extra: impl Fn(u64) -> f64) -> (Vec<TelFrame>, Vec<TelFrame>) {
        rows_at(2041.0, extra)
    }

    /// [`rows`] with the pot at `pos`.
    fn rows_at(pos: f64, extra: impl Fn(u64) -> f64) -> (Vec<TelFrame>, Vec<TelFrame>) {
        let n = |i: u64, k: u64, base: f64, g: f64| {
            (base + g * vdd(i) + 0.4 * noise(i, k)).round() as u16
        };
        let rest = (0..20_000u64)
            .map(|i| TelFrame {
                tick: i,
                pos: Some((pos + 1.15 * vdd(i) + extra(i) + 0.4 * noise(i, 2)).round() as u16),
                vbus_raw: Some(n(i, 3, 1241.0, 0.73)),
                ntc_raw: Some(n(i, 4, 2010.0, 1.59)),
                current_raw: Some(n(i, 5, 1206.0, 0.16)),
                vmotor_a: Some(n(i, 6, 773.0, 0.5)),
                vmotor_b: Some(n(i, 12, 772.0, 0.5)),
                ..TelFrame::default()
            })
            .collect();
        let raw = (0..20_000u64)
            .map(|i| TelFrame {
                tick: i,
                pos: Some(n(i, 7, pos, 1.15)),
                current_raw: Some(n(i, 8, 1206.0, 0.16)),
                vmotor_a: Some(n(i, 9, 773.0, 0.5)),
                vmotor_b: Some(n(i, 10, 772.0, 0.5)),
                duty_counts: Some(0),
                current_trough: Some(n(i, 11, 1206.0, 0.16)),
                ..TelFrame::default()
            })
            .collect();
        (rest, raw)
    }

    /// The gains are on the rail tap's pattern, in its counts: the rail
    /// reads the reference's ripple at 0.73, so a channel at 1.59 of the
    /// reference's is 1.59 / 0.73 of the rail's.
    #[test]
    fn the_vdd_frame_ripple_is_taken_out_and_the_frame_passes() {
        let (rest, raw) = rows(|_| 0.0);
        let ch = Channels::of(&rest, &raw);
        let p = ch.pattern.as_ref().expect("a pattern");
        assert_eq!((p.source, ch.passed_over.len()), ("vbus_raw", 0));
        assert!((p.mean - 1241.0).abs() < 0.5, "{}", p.mean);
        let pos = ch.rest("pos");
        assert!(pos.raw.fold > 1.0, "the ripple shows: {}", pos.raw.fold);
        assert!(pos.comp.fold < 0.3 && pos.comp.sd < pos.raw.sd, "{pos:?}");
        let ntc = ch.rest("ntc_raw").gain;
        assert!((ntc - 1.59 / 0.73).abs() < 0.05 * 1.59 / 0.73, "{ntc}");
        let raw_pos = ch.raw("pos").gain;
        assert!(
            (raw_pos - 1.15 / 0.73).abs() < 0.05 * 1.15 / 0.73,
            "the raw row by the rest's pattern: {raw_pos}"
        );
        assert_eq!(ch.rest("vbus_raw").gain, 1.0);
        let c = ch.pattern_check();
        assert_eq!(c.verdict, Verdict::Pass, "{}", c.value);
        assert!(c.value.contains("from vbus_raw"), "{}", c.value);
        assert_eq!(ch.said(), "the supply's frame pattern from vbus_raw");
    }

    /// The rail tap within a converter limit of its top, or absent: the
    /// pattern comes from the NTC, said; neither usable: no pattern, every
    /// gain 0, said.
    #[test]
    fn the_pattern_falls_back_from_the_rail_to_the_ntc_then_to_none() {
        let (mut rest, raw) = rows(|_| 0.0);
        for f in &mut rest {
            f.vbus_raw = Some(4090);
        }
        let ch = Channels::of(&rest, &raw);
        assert_eq!(ch.pattern.as_ref().map(|p| p.source), Some("ntc_raw"));
        assert!(
            ch.said()
                .starts_with("vbus_raw reads 4090.0, within 8 counts of a converter limit"),
            "{}",
            ch.said()
        );
        let pos = ch.rest("pos");
        assert!(pos.comp.fold < 0.3, "{pos:?}");
        let g = ch.raw("pos").gain;
        assert!((g - 1.15 / 1.59).abs() < 0.05 * 1.15 / 1.59, "{g}");

        for f in &mut rest {
            f.vbus_raw = None;
        }
        let ch = Channels::of(&rest, &raw);
        assert_eq!(ch.pattern.as_ref().map(|p| p.source), Some("ntc_raw"));
        assert!(
            ch.said().starts_with("the rest row carries no vbus_raw"),
            "{}",
            ch.said()
        );

        for f in &mut rest {
            f.ntc_raw = Some(3);
        }
        let ch = Channels::of(&rest, &raw);
        assert!(ch.pattern.is_none());
        assert!(
            ch.rest.iter().chain(&ch.raw).all(|(_, c)| c.gain == 0.0),
            "every gain 0"
        );
        assert_eq!(ch.rest("pos").comp, ch.rest("pos").raw, "nothing taken out");
        assert_eq!(
            ch.said(),
            "no supply frame pattern (the rest row carries no vbus_raw; ntc_raw reads 3.0, within 8 counts of a converter limit): nothing taken out, every channel's gain 0"
        );
        let c = ch.pattern_check();
        assert_eq!(c.verdict, Verdict::NotJudged, "{}", c.value);
        let j = judged_rows(2041.0, rest, raw, Ok(1490));
        assert!(
            j.lines
                .iter()
                .any(|l| l.starts_with("no supply frame pattern"))
        );
        let fr = &j.result["frame_ripple"];
        assert!(fr["pattern_counts"].is_null() && fr["pattern_source"].is_null());
        assert_eq!(fr["pattern_passed_over"].as_array().map(Vec::len), Some(2));
        assert_eq!(j.result["channels"]["raw"]["pos"]["pattern_gain"], 0.0);
    }

    /// A frame pattern that is not VDD's is said, not refused.
    #[test]
    fn a_frame_pattern_unlike_vdds_is_warned() {
        let own = |i: u64| 2.0 * (2.0 * core::f64::consts::PI * 3.0 * (i % 16) as f64 / 16.0).sin();
        let (rest, raw) = rows(own);
        let c = Channels::of(&rest, &raw).pattern_check();
        assert_eq!(c.verdict, Verdict::Warn, "{}", c.value);
        assert!(c.value.contains("(rest pos)"), "{}", c.value);
    }

    /// The rail's nominal scale is the one `rail_v` reads through, and on
    /// the bench servo's sense block the value the gather fixture's rest
    /// result carries (`testdata/gather/rest.json`).
    #[test]
    fn the_rail_scale_is_the_divider_rail_v_reads_through() {
        let s = sense();
        assert_eq!(s.counts_per_a(), 1108.4520727272727);
        assert_eq!(s.rail_counts_per_v(), 248.24242424242425);
        let v = s.rail_v(5.0 * s.rail_counts_per_v(), None);
        assert!((v - 5.0).abs() < 1e-12, "{v}");
    }

    /// The dev board's sense chain, its NTC words the stage 1 design.
    fn sense() -> Sense {
        let ntc = Parts {
            pullup_ohm: 10_000.0,
            r25_ohm: 10_000.0,
            beta: 3950.0,
        };
        Sense {
            shunt_r_mohm: 60.0,
            gain_milli: 14_884.0,
            vdd_mv: 3300.0,
            vmotor_div_top: 6400.0,
            vmotor_div_bot: 1600.0,
            vbus_div_top: 6400.0,
            vbus_div_bot: 1600.0,
            rail_drop_mv: 250.0,
            bias_nom: 773.0,
            tick_hz: 20_000.0,
            words: ntc::design(&ntc, &ntc::STAGE_1).expect("designs"),
            ntc,
        }
    }

    fn burst(layout: Layout, rows: Vec<TelFrame>) -> Burst {
        Burst {
            layout,
            armed: ROWS,
            frames: rows.len() / 16,
            rows,
            holes: 0,
            skipped: 0,
            garble: 0,
            laps: 0,
            timed_out: false,
            alert: false,
            dropped: 0,
            first_stamp: None,
        }
    }

    /// [`judge`] on the rows of [`rows_at`], the boot reading `vref`.
    fn judged(pos: f64, vref: Result<u16, String>) -> Judged {
        let (rest, raw) = rows_at(pos, |_| 0.0);
        judged_rows(pos, rest, raw, vref)
    }

    /// [`judge`] on `rest` and `raw`, the horn at `pos` before the step,
    /// the boot reading `vref`.
    fn judged_rows(
        pos: f64,
        rest: Vec<TelFrame>,
        raw: Vec<TelFrame>,
        vref: Result<u16, String>,
    ) -> Judged {
        let b = Before {
            line: Line {
                caps: 0,
                baud: osc_client::BaudRate::B3000000,
                row_bytes_max: 32,
            },
            sense: sense(),
            servo: Map::new(),
            vref,
            pos,
            rail_v: 5.0,
            deadman_ticks: 10_000,
            stream: StreamCfg {
                i_window_min_ticks: 0,
                tick_hz: 20_000,
            },
            rungs: [Rung::still("rest", 1.0, pos), Rung::still("raw", 1.0, pos)],
        };
        judge(
            &b,
            &burst(Layout::Rest, rest),
            &burst(Layout::Raw, raw),
            None,
        )
    }

    fn named<'a>(j: &'a Judged, name: &str) -> &'a Check {
        j.checks
            .iter()
            .find(|c| c.name == name)
            .unwrap_or_else(|| panic!("no {name} check"))
    }

    fn failed(j: &Judged) -> Vec<String> {
        j.checks
            .iter()
            .filter(|c| c.verdict == Verdict::Fail)
            .map(|c| format!("{}: {}", c.name, c.value))
            .collect()
    }

    /// The supply is the boot reading of the reference wherever the horn
    /// rests, the top of travel included, and the rail is read through it.
    #[test]
    fn the_chip_supply_is_the_boot_reading_wherever_the_horn_rests() {
        for pos in [2041.0, 3517.0] {
            let j = judged(pos, Ok(1490));
            assert!(failed(&j).is_empty(), "{pos}: {:?}", failed(&j));
            let vdd = named(&j, "chip supply");
            assert_eq!(vdd.verdict, Verdict::Pass, "{pos}: {}", vdd.value);
            assert!(
                vdd.value
                    .contains("3.299 V from vref_raw 1490, the boot reading"),
                "{}",
                vdd.value
            );
            assert_eq!(named(&j, "the frame's own pattern").verdict, Verdict::Pass);
            let est = &j.result["estimates"];
            assert_eq!(est["vdd_v"]["value"].as_f64(), Some(1.2 * 4096.0 / 1490.0));
            let vbus = j.result["channels"]["rest"]["vbus_raw"]["mean"]
                .as_f64()
                .expect("vbus");
            assert_eq!(
                est["rail_v"]["value"].as_f64(),
                Some(sense().rail_v(vbus, Some(1490.0)))
            );
            assert_ne!(
                sense().rail_v(vbus, Some(1490.0)),
                sense().rail_v(vbus, None)
            );
            let fr = &j.result["frame_ripple"];
            assert_eq!(fr["pattern_source"], "vbus_raw");
            assert_eq!(fr["pattern_counts"].as_array().map(Vec::len), Some(16));
            assert!(fr["vdd_ripple_mv_pp"].is_null() && est["vdd_frame_ripple_mv_pp"].is_null());
            assert_eq!(named(&j, "VDD frame ripple").verdict, Verdict::NotJudged);
            assert_eq!(j.result["channels"]["rest"]["vmotor_b"]["n"], 20_000);
            assert!(j.result["channels"]["rest"].get("vref_raw").is_none());
            assert_eq!(j.result["revision"], REVISION);
        }
    }

    /// A servo that holds no boot reading (`vref_raw` 0), or a READ that
    /// fails: no supply figure, said, the rail through the nominal supply,
    /// the pattern and its gains as ever, and the step not refused for it.
    #[test]
    fn a_zero_boot_reading_reports_no_supply_and_still_corrects() {
        for (vref, why) in [
            (
                Err("vref_raw reads 0, the servo took no reading at boot".to_string()),
                "vref_raw reads 0",
            ),
            (
                Err("vref_raw was not read (timeout)".to_string()),
                "not read",
            ),
        ] {
            let j = judged(2041.0, vref);
            assert!(failed(&j).is_empty(), "{:?}", failed(&j));
            let vdd = named(&j, "chip supply");
            assert_eq!(vdd.verdict, Verdict::NotJudged, "{}", vdd.value);
            assert!(
                vdd.value.contains(why) && vdd.value.contains("the nominal vdd_mv 3300 is used"),
                "{}",
                vdd.value
            );
            assert!(
                j.lines
                    .iter()
                    .any(|l| l.contains(why) && l.contains("no chip supply figure")),
                "{:?}",
                j.lines
            );
            assert_eq!(named(&j, "the frame's own pattern").verdict, Verdict::Pass);
            let est = &j.result["estimates"];
            assert!(est["vdd_v"].is_null() && est["vdd_frame_ripple_mv_pp"].is_null());
            let vbus = j.result["channels"]["rest"]["vbus_raw"]["mean"]
                .as_f64()
                .expect("vbus");
            assert_eq!(
                est["rail_v"]["value"].as_f64(),
                Some(sense().rail_v(vbus, None))
            );
            assert!(
                est["rail_v"]["source"]
                    .as_str()
                    .is_some_and(|s| s.contains("nominal")),
                "{}",
                est["rail_v"]
            );
            assert_eq!(j.result["frame_ripple"]["pattern_source"], "vbus_raw");
            let g = j.result["channels"]["raw"]["pos"]["pattern_gain"]
                .as_f64()
                .expect("gain");
            assert!((g - 1.15 / 0.73).abs() < 0.05 * 1.15 / 0.73, "{g}");
        }
    }

    /// A committed rest capture's first 4096 rows
    /// (`testdata/rest/sources.json`), by its header's names.
    fn recorded(file: &str) -> Vec<TelFrame> {
        let path = format!("{}/testdata/rest/{file}", env!("CARGO_MANIFEST_DIR"));
        let gz = std::fs::read(&path).expect("fixture");
        let text = String::from_utf8(meta::gunzip(&gz).expect("gzip")).expect("text");
        let mut lines = text.lines();
        let head: Vec<&str> = lines.next().expect("header").split(',').collect();
        let at = |name: &str| head.iter().position(|h| *h == name).expect(name);
        let col = |c: &[&str], name: &str| c[at(name)].parse::<u16>().ok();
        lines
            .map(|l| {
                let c: Vec<&str> = l.split(',').collect();
                TelFrame {
                    tick: c[at("tick")].parse().expect("tick"),
                    pos: col(&c, "pos"),
                    current_raw: col(&c, "current_raw"),
                    vmotor_a: col(&c, "vmotor_a"),
                    vbus_raw: col(&c, "vbus_raw"),
                    ntc_raw: col(&c, "ntc_raw"),
                    ..TelFrame::default()
                }
            })
            .collect()
    }

    /// The pos ripple left once `ch`'s pattern is out, peak to peak of
    /// the 16-phase fold, and before.
    fn pos_ripple(rows: &[TelFrame], ch: &Channels) -> (f64, f64) {
        let s = ripple::series(rows, "pos");
        let before = ripple::pp(&ripple::fold(&s).expect("phases"));
        let p = &ch.pattern.as_ref().expect("a pattern").counts;
        let after = ripple::compensate(&s, p, ch.rest("pos").gain);
        (before, ripple::pp(&ripple::fold(&after).expect("phases")))
    }

    /// On the bench's own rows, the horn mid travel and at the top of
    /// travel: the rail's pattern leaves under half a count of the frame's
    /// ripple on pos in both.
    #[test]
    fn the_rails_pattern_takes_the_ripple_off_pos_anywhere_on_travel() {
        for (file, top) in [("rest-mid.csv.gz", false), ("rest-end.csv.gz", true)] {
            let rows = recorded(file);
            let ch = Channels::of(&rows, &[]);
            assert_eq!(ch.pattern.as_ref().map(|p| p.source), Some("vbus_raw"));
            let pos = ch.rest("pos").raw.mean;
            assert_eq!(pos > 2750.0, top, "{file}: pos {pos}");
            let (before, after) = pos_ripple(&rows, &ch);
            assert!(before > 2.0, "{file}: the ripple shows, {before:.2}");
            assert!(after < 0.5, "{file}: {before:.2} -> {after:.2} counts pp");
        }
    }
}
