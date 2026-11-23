//! Step 1, rest and board check: nothing drives. Torque off, the rest row
//! for a second; then torque on at zero duty in OpenLoop, the bridge awake
//! and coasting, the dead-man written first, the raw row for a second.
//! The estimators are medians and spreads, no fit: each channel's mean and
//! standard deviation, the same folded by tick modulo 16 (the frame's own
//! pattern), the asleep and awake current zeros, the rail from `vbus_raw`
//! with the chip supply from `vref_raw`, the board temperature through the
//! NTC's nominal beta, and the NTC curve's four words designed from the
//! board's parts by the stage 1 rule ([`crate::ntc`]).

use osc_client::descriptor::Descriptor;
use osc_client::{Client, Error, Id, pipe::Pipe};
use serde_json::{Map, Value, json};

use super::{
    Check, Ended, Halt, Rig, Taken, Verdict, interrupted, keep_all, mode_value, take, write,
};
use crate::capture::meta;
use crate::capture::ripple::{self, Pattern};
use crate::capture::rows::{Layout, TelFrame};
use crate::capture::tel::{self, Burst, Line};
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
const REVISION: u32 = 1;
/// The telemetry frame's batch: a pattern in it is the frame's, not the
/// board's.
const FOLD: u64 = 16;
const ADC_TOP: f64 = 4095.0;
/// A count this close to a converter rail is a lead off or a short.
const RAIL_COUNTS: f64 = 8.0;
/// The chip's internal reference, CH32V006 datasheet nominal.
const VREF_V: f64 = 1.2;
/// Reads a position before and after the step is the mean of.
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
    /// place of the nominal when `vref_raw` is given.
    pub fn rail_v(&self, vbus_raw: f64, vref_raw: Option<f64>) -> f64 {
        let nominal = self.vdd_mv / 1000.0;
        let v = vbus_raw * nominal / ntc::ADC_COUNTS * (self.vbus_div_top + self.vbus_div_bot)
            / self.vbus_div_bot;
        vref_raw.map_or(v, |r| v * self.vdd_v(r) / nominal)
    }

    /// The supply at the connector: the rail and the input drop.
    pub fn supply_v(&self, rail_v: f64) -> f64 {
        rail_v + self.rail_drop_mv / 1000.0
    }
}

/// The rail now, V, from a few reads of the rail tap and the reference.
pub async fn rail_now<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    s: &Sense,
) -> Result<f64, Error> {
    let lo = field(d, "pos")?.addr;
    let (mut vbus, mut vref) = (0.0, 0.0);
    for _ in 0..POS_READS {
        let b = c.read(id, lo, 6).await?;
        let w = |k: usize| u16::from_le_bytes([b[k], b[k + 1]]) as f64;
        vref += w(2);
        vbus += w(4);
    }
    let n = POS_READS as f64;
    Ok(s.rail_v(vbus / n, (vref > 0.0).then_some(vref / n)))
}

pub(crate) async fn pos_now<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
) -> Result<f64, Error> {
    let f = field(d, "pos")?;
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
/// out, and its gain on the reference's pattern.
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

/// Both captures' channels and the frame ripple they were compensated by.
#[derive(Clone, Debug)]
pub struct Channels {
    /// The torque-off capture's `vref_raw` folded by tick modulo 16, its
    /// mean removed, and that mean; None without a whole pattern.
    pub vref: Option<(Pattern, f64)>,
    pub rest: Vec<(&'static str, Channel)>,
    pub raw: Vec<(&'static str, Channel)>,
}

pub const REST_CHANNELS: [&str; 6] = [
    "pos",
    "vref_raw",
    "vbus_raw",
    "ntc_raw",
    "current_raw",
    "vmotor_a",
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
    /// capture's reference pattern: the raw row carries no `vref_raw`, and
    /// both bursts start a frame at their first row, so the phase is the
    /// same.
    pub fn of(rest: &[TelFrame], raw: &[TelFrame]) -> Self {
        let vs = ripple::series(rest, "vref_raw");
        let vref = ripple::fold(&vs).zip(stats_of(&vs).map(|s| s.mean));
        let one = |rows: &[TelFrame], name: &'static str| {
            let s = ripple::series(rows, name);
            let (gain, comp) = match &vref {
                Some((p, _)) => ripple::fit(&s, p),
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
            vref,
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
    /// within a count, every channel, else refused. A channel no row
    /// carried (NaN) is no flatness.
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
        Check {
            name: "the frame's own pattern".into(),
            value: format!(
                "tick-modulo-16 means at most {fold:.2} counts off the mean ({}) once the VDD frame ripple is out",
                if worst.is_empty() { "none" } else { &worst }
            ),
            bound: format!("flat within {FOLD_MAX} count"),
            verdict: if fold <= FOLD_MAX {
                Verdict::Pass
            } else {
                Verdict::Fail
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
                            "vdd_gain": c.gain,
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
    pos: f64,
    rail_v: f64,
    deadman_ticks: u16,
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
    let sense = Sense::of(&state, rig.d)?;
    let servo = meta::servo(&state, rig.d, &line);
    let pos = pos_now(rig.c, rig.id, rig.d).await?;
    let rail_v = rail_now(rig.c, rig.id, rig.d, &sense).await?;
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
    Ok(Before {
        line,
        sense,
        servo,
        pos,
        rail_v,
        deadman_ticks: deadman as u16,
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
    let (r_pos, r_vref, r_vbus, r_ntc, r_zero, r_a) = (
        ch.rest("pos"),
        ch.rest("vref_raw").raw,
        ch.rest("vbus_raw").raw,
        ch.rest("ntc_raw").raw,
        ch.rest("current_raw"),
        ch.rest("vmotor_a"),
    );
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
    let vdd = s.vdd_v(r_vref.mean);
    let vdd_off = (vdd * 1000.0 - s.vdd_mv).abs() / s.vdd_mv;
    check(
        "chip supply",
        format!(
            "{vdd:.3} V from vref_raw {:.1}, {:.2}% off",
            r_vref.mean,
            100.0 * vdd_off
        ),
        format!("within {:.0}% of vdd_mv {}", 100.0 * VDD_SHARE, s.vdd_mv),
        pass(vdd_off <= VDD_SHARE),
    );
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
    let rail = s.rail_v(r_vbus.mean, Some(r_vref.mean));
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
    let ripple_mv = ch
        .vref
        .as_ref()
        .map(|(p, mean)| ripple::vdd_ripple_mv_pp(p, *mean, vdd));
    check(
        "VDD frame ripple",
        ripple_mv.map_or("no whole vref_raw pattern".into(), |mv| {
            format!("{mv:.2} mV pp once a 16-row frame, from vref_raw folded by tick modulo 16")
        }),
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
            "vdd_frame_ripple_mv_pp": ripple_mv.map_or(Value::Null, |mv| est(mv, "mV", "rest row vref_raw folded by tick modulo 16, reported, not judged")),
            "vdd_v": est(vdd, "V", "rest row vref_raw, the 1.2 V reference"),
            "rail_v": est(rail, "V", "rest row vbus_raw through the rail divider, the measured supply"),
            "supply_v": est(supply, "V", "the rail and the board's input drop"),
            "board_ntc_c": est(ntc_c, "C", "rest row ntc_raw through the part's nominal beta"),
            "stall_room_a": est(room_a, "A", "the awake zero to the converter's top, nominal counts per A"),
        },
        "counts_per_a_nominal": cpa,
        "ntc_parts": parts,
        "ntc_words": ntc_words,
        "channels": ch.json(),
        "frame_ripple": {
            "vref_pattern_counts": ch.vref.as_ref().map_or(Value::Null, |(p, _)| json!(p.to_vec())),
            "vref_mean_counts": ch.vref.as_ref().map_or(Value::Null, |(_, m)| json!(m)),
            "vdd_ripple_mv_pp": ripple_mv,
            "fold": ripple::FOLD,
            "phase": "tick modulo 16 from a burst's first row",
            "compensation": "each channel less its vdd_gain x vref_pattern_counts[tick % 16]",
        },
    });
    Judged { checks, result }
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
        let n = |i: u64, k: u64, base: f64, g: f64| {
            (base + g * vdd(i) + 0.4 * noise(i, k)).round() as u16
        };
        let rest = (0..20_000u64)
            .map(|i| TelFrame {
                tick: i,
                vref_raw: Some(n(i, 1, 1490.0, 1.0)),
                pos: Some((2041.0 + 1.15 * vdd(i) + extra(i) + 0.4 * noise(i, 2)).round() as u16),
                vbus_raw: Some(n(i, 3, 1241.0, 0.73)),
                ntc_raw: Some(n(i, 4, 2010.0, 1.59)),
                current_raw: Some(n(i, 5, 1206.0, 0.16)),
                vmotor_a: Some(n(i, 6, 773.0, 0.5)),
                ..TelFrame::default()
            })
            .collect();
        let raw = (0..20_000u64)
            .map(|i| TelFrame {
                tick: i,
                pos: Some(n(i, 7, 2041.0, 1.15)),
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

    #[test]
    fn the_vdd_frame_ripple_is_taken_out_and_the_frame_passes() {
        let (rest, raw) = rows(|_| 0.0);
        let ch = Channels::of(&rest, &raw);
        let pos = ch.rest("pos");
        assert!(pos.raw.fold > 1.0, "the ripple shows: {}", pos.raw.fold);
        assert!(pos.comp.fold < 0.3 && pos.comp.sd < pos.raw.sd, "{pos:?}");
        let ntc = ch.rest("ntc_raw").gain;
        assert!((ntc - 1.59).abs() < 0.05 * 1.59, "{ntc}");
        let raw_pos = ch.raw("pos").gain;
        assert!(
            (raw_pos - 1.15).abs() < 0.05 * 1.15,
            "the raw row by the rest's pattern: {raw_pos}"
        );
        let c = ch.pattern_check();
        assert_eq!(c.verdict, Verdict::Pass, "{}", c.value);
    }

    /// A frame pattern that is not VDD's is still refused.
    #[test]
    fn a_frame_pattern_unlike_vdds_is_still_refused() {
        let own = |i: u64| 2.0 * (2.0 * core::f64::consts::PI * 3.0 * (i % 16) as f64 / 16.0).sin();
        let (rest, raw) = rows(own);
        let c = Channels::of(&rest, &raw).pattern_check();
        assert_eq!(c.verdict, Verdict::Fail, "{}", c.value);
        assert!(c.value.contains("(rest pos)"), "{}", c.value);
    }
}
