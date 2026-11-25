//! Step 3, the stops and the seated R and L, on the probe's result: its
//! proven sign, its pricing R and its ceiling. Metal trains only, for now.
//!
//! The two stops are found by [`super::seek`]: two seats a stop, the low
//! stop and the high alternated, every drive open loop and host-polled
//! every [`seek::POLL_S`] under a [`POLL_DEADMAN_S`] bus dead-man, at
//! three duties from the probe: the approach (the level that proved the
//! sign and one ladder step, at most the ceiling), which every press
//! toward a stop runs at and never above; the seat (half the ceiling);
//! and the ceiling (the volts whose blocked current is half the gear's
//! stop-hold ceiling at the pricing R), which only a leave rises to. No
//! drive exceeds the ceiling.
//!
//! At each seat, once the stop is read, the duty goes to zero with torque
//! kept on and the train wound against the stop; with the current back at
//! the rest zero, shunt bursts from rest as the probe's part A arms them,
//! at 25 % and 40 % of the rail, the sign that presses into the stop
//! only, four a level. The stall permit is held around them only where the
//! seat lies at or past a soft limit, since the firmware arms a burst
//! there only under a granted permit (protocol sec 5.8). They are read by
//! [`crate::fit::seated`]: the first of each group out, the period-pair
//! fit with no intercept per burst, medians over both stops and both
//! levels.
//!
//! Every window of a drive is priced before it starts, at its duty for
//! its time and the dead-man: the envelope prices a rung up to a burst's
//! length, and a leg runs up to [`seek::LEG_MAX_S`]. A burst is priced as
//! part A prices one. The way out leaves the shaft near the middle of the
//! stops, where the last leg stopped it.

use std::time::Duration;

use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use osc_servo_core::kernel::phases::loops::faults::BIT_DEADMAN;
use serde_json::{Map, Value, json};

use super::polarity::{self, HANDS, STALLED, Stop};
use super::probe::{self, FromRest, NOT_METAL};
use super::seek::{self, Act, Seek, SeekCfg, WRONG_WAY_COUNTS, toward};
use super::shunt::{self, AFTER_READS, Arm, Shunted};
use super::{
    Check, Ended, Halt, Rig, Verdict, interrupted, mean_u16, mode_value, read_u8, read_u16, rest,
    write,
};
use crate::Board;
use crate::capture::burst::{self, CHANS_DIFF};
use crate::capture::meta;
use crate::capture::rows::Layout;
use crate::capture::tel::{self, Line};
use crate::envelope::{Envelope, Facts, HeatModel, Kind, Resistance, Rung};
use crate::fit::median;
use crate::fit::rise::{Lags, Scales};
use crate::fit::seated::{self, SeatedRead, Shot, Spread};
use crate::guard::{self, Ledger, Way};
use crate::inputs::ThermalInputs;
use crate::install::Refused;
use crate::nameplate::{GearClass, Nameplate};
use crate::ntc::{ADC_COUNTS, Parts};
use crate::sink::Sink;
use crate::state::{self, State, field};

pub const STEP: &str = "stops";
pub const RESULT: &str = "results/stops.json";
const METHOD: &str = "stops-and-seated-rl";
const REVISION: u32 = 2;
/// Every poll of every leg, a row each.
pub const POLLS: &str = "captures/stops/polls.csv";
/// The bus dead-man a polled drive runs under, s: five poll periods, so a
/// host that stops polling holds the duty no longer.
pub const POLL_DEADMAN_S: f64 = 0.100;
/// The longest the host goes between rewrites of a held permit, s: well
/// inside its one-second lease (protocol sec 5.8).
pub const PERMIT_REFRESH_S: f64 = 0.25;
/// The seated bursts' levels, shares of the rail.
pub const LEVELS: [f64; 2] = [0.25, 0.40];
pub const BURSTS_A_LEVEL: usize = 4;
/// The longest the current may take back to the rest zero after a seat,
/// s.
pub const ZERO_WAIT_S: f64 = 1.0;
/// A stop's two seats within this, counts.
pub const SEATS_APART_MAX: f64 = seek::WALK_MAX_COUNTS;
/// The stops at least a quarter of the converter's range apart.
pub const SPAN_MIN_COUNTS: f64 = ADC_COUNTS / 4.0;
/// The span over the nameplate's travel, counts a degree.
pub const COUNTS_PER_DEG: (f64, f64) = (10.0, 30.0);
/// A stop this close to a converter rail is flagged.
pub const RAIL_NEAR_COUNTS: f64 = 8.0;
pub const KEPT_MIN: usize = 6;
/// The two stops' R within this share of their mean: the winding's R
/// moves a few percent with the rotor's angle, and the two stops are two
/// angles.
pub const STOPS_R_SHARE: f64 = 0.10;
/// The burst-to-burst sd of R inside the groups, over the median.
pub const R_SD_MAX: f64 = 0.03;
/// The two stops' R are compared only when each keeps this many bursts.
pub const KEPT_A_STOP_MIN: usize = 2;
/// Over this share of the bursts in the fit set aside as bridged warns.
pub const BRIDGED_WARN_SHARE: f64 = 0.25;
/// Two reads of the pos this far apart hold a torn read: the reply copies
/// a two-byte field a byte at a time out of the tick block the ADC DMA
/// lands each scan, so a read that straddles a landing across a byte
/// boundary can be about 256 counts off.
pub const TORN_COUNTS: u16 = 64;
const ADC_TOP: f64 = 4095.0;
const Q15: f64 = 32768.0;
const PRICE_WORDS: &str = "the probe's pricing R (planning grade)";

/// What the stops step takes from the probe's result.
#[derive(Clone, Debug, PartialEq)]
pub struct FromProbe {
    /// The `drive_polarity` stored on the servo when the probe ran, and
    /// the one the probe proved.
    pub stored: bool,
    pub found: bool,
    /// The volts of the ladder level that proved the sign.
    pub proven_v: f64,
    pub ceiling_v: f64,
    pub r_price_ohm: f64,
}

impl FromProbe {
    pub fn of(probe: &Value) -> Result<Self, Refused> {
        if probe["verdict"] != "accepted" {
            return Err(Refused(format!(
                "the stops step runs on an accepted probe; the probe was {}",
                probe["verdict"].as_str().unwrap_or("not run")
            )));
        }
        let pol = &probe["drive_polarity"];
        let b = |k: &str| {
            pol[k]
                .as_bool()
                .ok_or_else(|| Refused(format!("the probe result holds no drive_polarity {k}")))
        };
        let n = |k: &str| {
            probe["part_b"][k]
                .as_f64()
                .filter(|v| v.is_finite() && *v > 0.0)
                .ok_or_else(|| Refused(format!("the probe result holds no part_b {k}")))
        };
        // the ladder decides on its repeat pair: its last pulse is at the
        // level that proved the sign
        let proven_v = probe["part_b"]["pulses"]
            .as_array()
            .and_then(|p| p.last())
            .filter(|p| p["repeat"] == true && p["unwedge"] == false)
            .and_then(|p| p["volts"].as_f64())
            .ok_or_else(|| {
                Refused("the probe result names no pulse level that proved the sign".into())
            })?;
        Ok(FromProbe {
            stored: b("stored")?,
            found: b("found")?,
            proven_v,
            ceiling_v: n("ceiling_v")?,
            r_price_ohm: n("r_price_ohm")?,
        })
    }
}

/// What the step reads besides the servo.
pub struct StopsInputs<'a> {
    pub nameplate: &'a Nameplate,
    pub board: &'a Board,
    pub thermal: Option<&'a ThermalInputs>,
    /// The same run's `results/rest.json` and `results/probe.json`.
    pub rest: &'a Value,
    pub probe: &'a Value,
}

#[derive(Clone, Debug)]
pub struct Stopped {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Stopped {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
}

/// The stall permit is needed to arm a burst at `pos`: it does not lie
/// inside the soft limits by more than `band` (protocol sec 5.8: the arm
/// wants the raw pos strictly inside them, or a granted permit).
pub fn needs_permit(pos: f64, soft: (f64, f64), band: f64) -> bool {
    !(pos > soft.0 + band && pos < soft.1 - band)
}

/// The least share of the ceiling an approach runs at: a sign proven at
/// the motor's breakaway leaves an approach one step over it, which the
/// higher friction near an end of the travel can stall short of the stop.
pub const APPROACH_MIN_SHARE: f64 = 0.6;

/// The step's three duties, Q15, from the probe's volts on `rail_v`: the
/// approach one ladder step over the volts that proved the sign and at
/// least [`APPROACH_MIN_SHARE`] of the ceiling, the seat half the ceiling
/// and never over the approach.
pub fn duties(p: &FromProbe, rail_v: f64, stated_v: f64, still_counts: f64) -> SeekCfg {
    let q = |v: f64| (v / rail_v * Q15).floor().clamp(0.0, i16::MAX as f64) as i16;
    let ceiling = q(p.ceiling_v);
    let approach = q(p.proven_v + polarity::STEP_SHARE * stated_v)
        .max(q(APPROACH_MIN_SHARE * p.ceiling_v))
        .min(ceiling);
    SeekCfg {
        rail_v,
        approach_q15: approach,
        step_q15: q(polarity::STEP_SHARE * stated_v).max(1),
        ceiling_q15: ceiling,
        seat_q15: q(p.ceiling_v / 2.0).min(approach),
        still_counts,
    }
}

/// What the step knew before its first write.
struct Before {
    line: Line,
    servo: Map<String, Value>,
    pos: f64,
    rail_v: f64,
    deadman_ticks: u16,
    poll_deadman_ticks: u16,
    stored: bool,
    probe: FromProbe,
    rest: FromRest,
    cfg: SeekCfg,
    pos_read: PosRead,
    env: Envelope,
    scales: Scales,
    stated_v: f64,
    rated_cps: f64,
    travel_deg: f64,
    soft: [Vec<u8>; 2],
    soft_counts: (f64, f64),
    mv_per_count: f64,
    counts_per_a: f64,
    ntc: Parts,
    trip_a: f64,
    stall: Option<(f64, f64)>,
}

fn num(state: &State, d: &osc_client::descriptor::Descriptor, name: &str) -> Result<f64, Refused> {
    let f = field(d, name)?;
    state
        .text(f)
        .as_f64()
        .ok_or_else(|| Refused(format!("the state read holds no {name}")))
}

/// A seated burst's rung, priced as part A prices one: blocked at the
/// most current the board's trip, the 0.5 ohm floor and a stated stall
/// allow.
fn burst_rung(name: String, q15: i16, pos: f64) -> Rung {
    Rung {
        name,
        duty: q15 as f64 / Q15,
        time_s: probe::BURST_S,
        permit: false,
        gap_s: probe::GAP_S,
        start: pos,
        speed_cps: 0.0,
        stop_counts: 0.0,
        kind: Kind::Drive,
    }
}

/// `env` with R set to the most current a burst at `volts` may draw, for
/// `f`; the pricing R back after.
fn at_burst_r<T>(
    env: &mut Envelope,
    volts: f64,
    trip_a: f64,
    stall: Option<(f64, f64)>,
    f: impl FnOnce(&mut Envelope) -> T,
) -> T {
    let r = env.facts.r.replace(probe::first_r(volts, trip_a, stall));
    let out = f(env);
    env.facts.r = r;
    out
}

/// The Q15 of a level on `rail_v` and, when the envelope refuses it, the
/// highest it admits with what it said: the share of the rail, at most
/// the 3.2 V a burst may apply.
#[allow(clippy::too_many_arguments)]
fn level_q15(
    env: &mut Envelope,
    share: f64,
    rail_v: f64,
    dir: i8,
    pos: f64,
    trip_a: f64,
    stall: Option<(f64, f64)>,
) -> Result<(i16, Option<String>), Refused> {
    let volts = (share * rail_v).min(burst::MAX_MV / 1000.0);
    let top = (volts / rail_v * Q15).floor().min(i16::MAX as f64) as i16;
    let priced = |env: &mut Envelope, q: i16| {
        let v = q as f64 / Q15 * rail_v;
        at_burst_r(env, v, trip_a, stall, |e| {
            e.price(&burst_rung("seated burst".into(), q * dir as i16, pos))
        })
    };
    let why = match priced(env, top) {
        Ok(_) => return Ok((top, None)),
        Err(e) => e.0,
    };
    let (mut lo, mut hi) = (0i16, top);
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if priced(env, mid).is_ok() {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    if lo == 0 {
        return Err(Refused(why));
    }
    Ok((
        lo,
        Some(format!(
            "the {:.0}% level lowered to {:.2}% ({:.3} V), the most the envelope admits: {why}",
            100.0 * share,
            100.0 * lo as f64 / Q15,
            lo as f64 / Q15 * rail_v
        )),
    ))
}

async fn before<P: Pipe>(rig: &mut Rig<'_, P>, i: &StopsInputs<'_>) -> Result<Before, Refused> {
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
        Refused("no thermal values entered (--thermal): the heat of a press cannot be priced, so nothing drives".into())
    })?;
    let line = tel::line(rig.c, rig.id).await?;
    line.admit(Layout::Raw)?;
    let rest = FromRest::of(i.rest)?;
    let probe = FromProbe::of(i.probe)?;
    let state = state::read(rig.c, rig.id, rig.d).await?;
    if state.torque {
        return Err(Refused(
            "torque_enable is on: this step starts only with torque off, the shaft at rest".into(),
        ));
    }
    let faults = read_u8_raw(rig, "fault_flags").await?;
    if faults != 0 {
        return Err(Refused(format!(
            "a fault is latched (fault_flags {faults:#04x}): `osc status` names it"
        )));
    }
    let stored = state
        .bytes(field(rig.d, "drive_polarity")?)
        .and_then(|b| b.first().copied())
        .ok_or_else(|| Refused("the state read holds no drive_polarity".into()))?
        != 0;
    if stored != probe.stored {
        return Err(Refused(format!(
            "the servo's stored drive_polarity is {stored}, and the probe ran on {}: something changed it since; run the probe again",
            probe.stored
        )));
    }
    let b = i.board;
    if !(b.oc_trip_ma.is_finite() && b.oc_trip_ma > 0.0) {
        return Err(Refused(format!(
            "the board's description ({}) states no overcurrent trip: a burst cannot be priced",
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
            "the board's description ({}) states no bridge_min_ticks: the seat duty cannot be checked against the bridge's shortest pulse",
            b.name
        ))
    })?;
    let speed = np
        .speed_row()
        .ok_or_else(|| Refused("the nameplate has no usable no_load".into()))?;
    let travel_deg = np
        .travel_deg
        .ok_or_else(|| Refused("the nameplate has no usable travel_deg".into()))?;
    let sense = rest::Sense::of(&state, rig.d)?;
    let servo = meta::servo(&state, rig.d, &line);
    let pos_read = read_pos(rig.c, rig.id, rig.d).await?;
    let pos = pos_read.pos;
    let rail_v = rest::rail_now(rig.c, rig.id, rig.d, &sense).await?;
    let pwm_arr = num(&state, rig.d, "pwm_period_ticks")?;
    let ntc_raw = read_u16_raw(rig, "ntc_raw").await? as f64;
    let stated_v = speed.at_v;
    let cfg = duties(&probe, rail_v, stated_v, seek::still_band(rest.pot_sd));
    let start_q15 = (bridge_min_ticks * Q15 / pwm_arr).ceil() as i16;
    if cfg.seat_q15 < start_q15 || cfg.approach_q15 < cfg.seat_q15 {
        return Err(Refused(format!(
            "the probe's ceiling {:.3} V leaves a seat duty of {:.3} V under the bridge's shortest pulse ({:.3} V) or over the approach ({:.3} V): run the probe again",
            probe.ceiling_v,
            cfg.volts(cfg.seat_q15),
            cfg.volts(start_q15),
            cfg.volts(cfg.approach_q15)
        )));
    }
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
    let trip_a = b.oc_trip_ma / 1000.0;
    let stall = np
        .stall_row()
        .and_then(|s| s.current_a.map(|a| (s.at_v, a)));
    let mut facts = Facts::new(np, rail_v);
    facts.ntc_c = Some(sense.ntc.c_of_raw(ntc_raw));
    facts.polarity_proven = true;
    facts.r = Some(Resistance {
        ohm: probe.r_price_ohm,
        source: PRICE_WORDS.into(),
    });
    let heat = HeatModel::new(
        b,
        thermal,
        probe.r_price_ohm,
        sense.counts_per_a(),
        sense.tick_hz,
    );
    let mut env = Envelope::new(facts, heat);
    let rated_cps = speed.per_deg / speed.seconds * ADC_COUNTS / travel_deg;
    // the first window, a hold and every level priced before any write
    for (name, q, s) in [
        (
            "the first approach window",
            cfg.approach_q15,
            seek::WINDOW_S,
        ),
        ("a seat hold", cfg.seat_q15, seek::HOLD_S),
    ] {
        env.price(&window_rung(
            name, q, s, true, pos, &cfg, stated_v, rated_cps,
        ))?;
    }
    env.price(&window_rung(
        "a leave window at the ceiling",
        cfg.ceiling_q15,
        seek::WINDOW_S,
        false,
        pos,
        &cfg,
        stated_v,
        rated_cps,
    ))?;
    for share in LEVELS {
        level_q15(&mut env, share, rail_v, -1, pos, trip_a, stall)?;
    }
    let ticks = |s: f64| {
        let t = (s * sense.tick_hz).round();
        if (1.0..=u16::MAX as f64).contains(&t) {
            Ok(t as u16)
        } else {
            Err(Refused(format!(
                "a {s} s dead-man is no u16 of ticks at {} Hz",
                sense.tick_hz
            )))
        }
    };
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
    let word = |v: &[u8]| i32::from_le_bytes(v[..4].try_into().unwrap_or([0; 4])) as f64;
    Ok(Before {
        line,
        servo,
        pos,
        rail_v,
        deadman_ticks: ticks(rest::DEADMAN_S)?,
        poll_deadman_ticks: ticks(POLL_DEADMAN_S)?,
        stored,
        soft_counts: (word(&lo), word(&hi)),
        soft: [lo, hi],
        probe,
        rest,
        cfg,
        pos_read,
        env,
        scales,
        stated_v,
        rated_cps,
        travel_deg,
        mv_per_count: sense.vdd_mv / ADC_COUNTS * (sense.vbus_div_top + sense.vbus_div_bot)
            / sense.vbus_div_bot,
        counts_per_a: sense.counts_per_a(),
        ntc: sense.ntc,
        trip_a,
        stall,
    })
}

async fn read_u8_raw<P: Pipe>(rig: &mut Rig<'_, P>, name: &str) -> Result<u8, Refused> {
    let f = field(rig.d, name)?;
    Ok(rig
        .c
        .read(rig.id, f.addr, 1)
        .await?
        .first()
        .copied()
        .unwrap_or(0))
}

async fn read_u16_raw<P: Pipe>(rig: &mut Rig<'_, P>, name: &str) -> Result<u16, Refused> {
    let f = field(rig.d, name)?;
    let b = rig.c.read(rig.id, f.addr, 2).await?;
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

/// A drive's window as the envelope prices it: its duty for its time and
/// the dead-man after.
#[allow(clippy::too_many_arguments)]
fn window_rung(
    name: &str,
    q15: i16,
    span_s: f64,
    permit: bool,
    pos: f64,
    cfg: &SeekCfg,
    stated_v: f64,
    rated_cps: f64,
) -> Rung {
    Rung {
        name: name.into(),
        duty: q15 as f64 / Q15,
        time_s: span_s + POLL_DEADMAN_S,
        permit,
        gap_s: 0.0,
        start: pos,
        speed_cps: cfg.volts(q15) / stated_v * rated_cps,
        stop_counts: 0.0,
        kind: Kind::Drive,
    }
}

/// A seated burst taken.
#[derive(Clone, Debug)]
pub struct SeatShot {
    pub seat: usize,
    pub stop: Stop,
    pub level: usize,
    pub ntc_c: f64,
    pub permit: bool,
    pub shunted: Shunted,
    pub after_a: f64,
}

/// A guarded read of the pos that held a torn read, and where.
#[derive(Clone, Debug, PartialEq)]
pub struct Torn {
    pub leg: usize,
    pub kind: &'static str,
    pub read: PosRead,
}

/// One poll as kept in [`POLLS`]; a poll that halted keeps what it read
/// and why.
#[derive(Clone, Debug, PartialEq)]
pub struct PollRow {
    pub t: f64,
    pub leg: usize,
    pub kind: &'static str,
    pub duty_q15: i16,
    pub reads: Vec<u16>,
    pub pos: Option<f64>,
    pub faults: Option<u8>,
    pub halt: Option<String>,
}

/// What the body found, as far as it got.
#[derive(Default)]
struct Found {
    seek: Option<Seek>,
    shots: Vec<SeatShot>,
    lowered: Vec<String>,
    corrected: bool,
    polls: Vec<PollRow>,
    torn: Vec<Torn>,
}

/// The duty, the dead-man and the permit as the host last wrote them.
#[derive(Default)]
struct OnServo {
    duty: Option<i16>,
    driving: bool,
    permit: bool,
    refreshed: f64,
}

/// The pos as one guarded read: two bus reads, and when they lie over
/// [`TORN_COUNTS`] apart a third.
#[derive(Clone, Debug, PartialEq)]
pub struct PosRead {
    pub reads: Vec<u16>,
    pub pos: f64,
}

impl PosRead {
    /// The first of two reads; the median of three.
    pub fn of(reads: &[u16]) -> Self {
        let pos = match reads {
            [a, b, c] => {
                let mut m = [*a, *b, *c];
                m.sort_unstable();
                m[1]
            }
            _ => reads.first().copied().unwrap_or(0),
        };
        PosRead {
            reads: reads.to_vec(),
            pos: pos as f64,
        }
    }

    pub fn torn(&self) -> bool {
        self.reads.len() > 2
    }

    fn said(&self) -> String {
        let r: Vec<String> = self.reads.iter().map(|x| x.to_string()).collect();
        format!("reads {}, {:.0} used", r.join(", "), self.pos)
    }
}

pub(crate) async fn read_pos<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
) -> Result<PosRead, osc_client::Error> {
    let addr = field(d, "pos")?.addr;
    let mut reads: Vec<u16> = Vec::with_capacity(3);
    while reads.len() < 2 || (reads.len() == 2 && reads[0].abs_diff(reads[1]) > TORN_COUNTS) {
        let b = c.read(id, addr, 2).await?;
        reads.push(u16::from_le_bytes([b[0], b[1]]));
    }
    Ok(PosRead::of(&reads))
}

/// One poll as read: the pos, guarded, then the fault flags.
struct Polled {
    pos: PosRead,
    faults: u8,
}

async fn polled<P: Pipe>(rig: &mut Rig<'_, P>) -> Result<Polled, Halt> {
    let pos = read_pos(rig.c, rig.id, rig.d).await?;
    let faults = read_u8(rig, "fault_flags").await?;
    Ok(Polled { pos, faults })
}

/// A dead-man latched refuses with [`STALLED`], any other fault aborts.
async fn faulted<P: Pipe>(rig: &mut Rig<'_, P>, faults: u8) -> Result<(), Halt> {
    if faults & BIT_DEADMAN != 0 {
        return Err(Halt::Refused(STALLED.into()));
    }
    if faults != 0 {
        let code = read_u8(rig, "fault_code").await.unwrap_or(0);
        return Err(Halt::Error(format!(
            "a fault latched during a drive: fault_flags {faults:#04x}, fault_code {code}"
        )));
    }
    Ok(())
}

/// The pot after a poll period, read guarded: a dead-man latched refuses
/// with [`STALLED`], any other fault aborts. The stops step's own drive
/// polls through [`logged_poll`], which keeps every reading.
pub(crate) async fn poll<P: Pipe>(rig: &mut Rig<'_, P>) -> Result<f64, Halt> {
    let p = polled(rig).await?;
    faulted(rig, p.faults).await?;
    Ok(p.pos.pos)
}

/// [`poll`] with its row kept and a torn read counted: the pos and the
/// host's time it was read at.
async fn logged_poll<P: Pipe>(
    rig: &mut Rig<'_, P>,
    seek: &Seek,
    duty_q15: i16,
    found: &mut Found,
) -> Result<(f64, f64), Halt> {
    let (leg, kind) = seek.leg_now();
    let p = polled(rig).await;
    let t = (rig.clock)();
    let mut row = PollRow {
        t,
        leg,
        kind,
        duty_q15,
        reads: Vec::new(),
        pos: None,
        faults: None,
        halt: None,
    };
    let out = match p {
        Ok(p) => {
            row.reads = p.pos.reads.clone();
            row.pos = Some(p.pos.pos);
            row.faults = Some(p.faults);
            if p.pos.torn() {
                found.torn.push(Torn {
                    leg,
                    kind,
                    read: p.pos.clone(),
                });
            }
            faulted(rig, p.faults).await.map(|()| (p.pos.pos, t))
        }
        Err(h) => Err(h),
    };
    if let Err(h) = &out {
        row.halt = Some(h.why());
    }
    found.polls.push(row);
    out
}

/// The poll rows as CSV, the reads a poll took in order.
pub fn polls_csv(rows: &[PollRow]) -> String {
    let mut out = String::from("t_s,leg,kind,duty_q15,pos,read_1,read_2,read_3,fault_flags,halt\n");
    for r in rows {
        let read = |k: usize| r.reads.get(k).map_or(String::new(), |x| x.to_string());
        out.push_str(&format!(
            "{:.4},{},{},{},{},{},{},{},{},{}\n",
            r.t,
            r.leg,
            r.kind,
            r.duty_q15,
            r.pos.map_or(String::new(), |p| format!("{p:.0}")),
            read(0),
            read(1),
            read(2),
            r.faults.map_or(String::new(), |f| format!("{f:#04x}")),
            r.halt
                .as_ref()
                .map_or(String::new(), |h| format!("\"{}\"", h.replace('"', "\"\""))),
        ));
    }
    out
}

/// The duty to zero, the step's dead-man back, the permit off.
async fn halt_drive<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    on: &mut OnServo,
) -> Result<(), Halt> {
    if on.driving || on.duty.is_some_and(|q| q != 0) {
        write(rig, "goal_duty", &[0, 0]).await?;
        write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
    }
    if on.permit {
        write(rig, "stall_permit", &[0]).await?;
    }
    *on = OnServo::default();
    Ok(())
}

/// The current back at the rest zero after a seat, the duty at zero, as
/// part A reads it after a burst, A off the zero.
async fn back_at_zero<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before) -> Result<f64, Halt> {
    let t0 = (rig.clock)();
    loop {
        let a = (mean_u16(rig, "current_raw", AFTER_READS).await? - b.rest.zero_counts)
            / b.counts_per_a;
        if a.abs() <= probe::AFTER_MAX_A {
            return Ok(a);
        }
        if (rig.clock)() - t0 > ZERO_WAIT_S {
            return Err(Halt::Error(format!(
                "the current reads {:.1} mA off the rest zero {ZERO_WAIT_S} s after the duty went to zero at a seat: the bridge does not let go; check the board",
                1e3 * a
            )));
        }
        rig.c.pause(Duration::from_secs_f64(seek::POLL_S)).await;
    }
}

/// The seated bursts at `seat`: both levels, the sign into the stop, four
/// a level, each priced before its arm; a line a level.
async fn seat_bursts<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    seat: &seek::Seat,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    let zero_a = back_at_zero(rig, b).await?;
    let ntc_c = b.ntc.c_of_raw(read_u16(rig, "ntc_raw").await? as f64);
    let permit = needs_permit(seat.pos, b.soft_counts, b.cfg.still_counts);
    let dir = toward(seat.stop);
    let arm = Arm {
        polarity: b.probe.found,
        seated: true,
        mv_per_count: b.mv_per_count,
        soft: b.soft.clone(),
    };
    lines.push(format!(
        "seat {}: the {} stop reads {:.0} counts (range {:.0} over the last {} s){}; at zero duty the current {:+.1} mA off the rest zero, the board NTC {ntc_c:.1} C; bursts {} into the stop{}",
        seat.n,
        seat.stop.word(),
        seat.pos,
        seat.range,
        seek::READ_S,
        seat.walk
            .map_or(String::new(), |w| format!(", {w:+.0} off its earlier seat")),
        1e3 * zero_a,
        if dir > 0 { "+duty" } else { "-duty" },
        if permit {
            format!(
                ", the permit held: the seat at {:.0} is not inside the soft limits {:.0}..{:.0}",
                seat.pos, b.soft_counts.0, b.soft_counts.1
            )
        } else {
            String::new()
        }
    ));
    for (level, share) in LEVELS.iter().enumerate() {
        let (q, lowered) = level_q15(
            &mut b.env, *share, b.rail_v, dir, seat.pos, b.trip_a, b.stall,
        )?;
        if let Some(w) = lowered {
            lines.push(format!("seat {}: {w}", seat.n));
            found.lowered.push(w);
        }
        let duty = q * dir as i16;
        let from = found.shots.len();
        for k in 0..BURSTS_A_LEVEL {
            interrupted(rig.stop)?;
            let label = format!("seat {} burst {}", seat.n, level * BURSTS_A_LEVEL + k + 1);
            let v = q as f64 / Q15 * b.rail_v;
            at_burst_r(&mut b.env, v, b.trip_a, b.stall, |e| {
                e.admit(&burst_rung(label.clone(), duty, seat.pos))
            })?;
            if permit {
                write(rig, "stall_permit", &[1]).await?;
            }
            let t = shunt::shot(rig, &arm, duty, &label).await?;
            let after_a = (t.after_counts - b.rest.zero_counts) / b.counts_per_a;
            found.shots.push(SeatShot {
                seat: seat.n,
                stop: seat.stop,
                level,
                ntc_c,
                permit,
                shunted: t,
                after_a,
            });
            rig.c.pause(Duration::from_secs_f64(probe::GAP_S)).await;
        }
        let group = &found.shots[from..];
        let r = seated::read(&shots_of(group), &b.scales);
        let rs: Vec<String> = r
            .bursts
            .iter()
            .map(|x| {
                format!(
                    "{}{}",
                    x.fit
                        .as_ref()
                        .map_or("no fit".into(), |f| format!("{:.3}", f.r_ohm)),
                    if x.kept() { "" } else { " (out)" }
                )
            })
            .collect();
        lines.push(format!(
            "seat {} at the {} stop, {:.0}%: {BURSTS_A_LEVEL} bursts at {:.3} V, R {} ohm",
            seat.n,
            seat.stop.word(),
            100.0 * q as f64 / Q15,
            q as f64 / Q15 * b.rail_v,
            rs.join(", ")
        ));
    }
    if permit {
        write(rig, "stall_permit", &[0]).await?;
    }
    Ok(())
}

fn stop_index(s: Stop) -> usize {
    match s {
        Stop::Low => 0,
        Stop::High => 1,
    }
}

/// The seated fit's shots: a group a level at a seat.
fn shots_of(taken: &[SeatShot]) -> Vec<Shot> {
    taken
        .iter()
        .map(|s| Shot {
            group: (s.seat - 1) * LEVELS.len() + s.level,
            stop: stop_index(s.stop),
            level: s.level,
            ntc_c: s.ntc_c,
            cap: s.shunted.cap.clone(),
        })
        .collect()
}

/// The finder on the servo: each act of [`Seek`] written and polled, the
/// seated bursts at each seat, a line a leg and a seat.
async fn drive<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    seek: &mut Seek,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    let mut on = OnServo::default();
    let mut said = 0;
    let first = read_pos(rig.c, rig.id, rig.d).await?;
    let t = (rig.clock)();
    let (leg, kind) = seek.leg_now();
    if first.torn() {
        found.torn.push(Torn {
            leg,
            kind,
            read: first.clone(),
        });
    }
    found.polls.push(PollRow {
        t,
        leg,
        kind,
        duty_q15: 0,
        reads: first.reads.clone(),
        pos: Some(first.pos),
        faults: None,
        halt: None,
    });
    let mut pos = first.pos;
    let mut act = seek.start(t, pos);
    loop {
        for l in &seek.legs()[said..] {
            lines.push(format!(
                "leg {}, {} toward the {} stop: {:.0} to {:.0} in {:.2} s, at most {:.3} V",
                l.n,
                l.kind.word(),
                l.toward.word(),
                l.from,
                l.to,
                l.secs,
                seek.cfg().volts(l.top_q15)
            ));
        }
        said = seek.legs().len();
        match act {
            Act::Drive { q15, permit, price } => {
                interrupted(rig.stop)?;
                if q15.unsigned_abs() > b.cfg.ceiling_q15 as u16 {
                    return Err(Halt::Error(format!(
                        "a duty of {q15} was asked for, over the ceiling's {}: nothing more drives",
                        b.cfg.ceiling_q15
                    )));
                }
                if let Some(span) = price {
                    let r = window_rung(
                        "a drive window",
                        q15,
                        span,
                        permit,
                        pos,
                        &b.cfg,
                        b.stated_v,
                        b.rated_cps,
                    );
                    b.env.admit(&r)?;
                }
                let now = (rig.clock)();
                if permit && !on.permit {
                    write(rig, "stall_permit", &[1]).await?;
                    on.permit = true;
                    on.refreshed = now;
                } else if !permit && on.permit {
                    write(rig, "stall_permit", &[0]).await?;
                    on.permit = false;
                }
                if !on.driving {
                    write(
                        rig,
                        "bus_deadman_ticks",
                        &b.poll_deadman_ticks.to_le_bytes(),
                    )
                    .await?;
                    on.driving = true;
                }
                if on.duty != Some(q15) {
                    write(rig, "goal_duty", &q15.to_le_bytes()).await?;
                    on.duty = Some(q15);
                }
                if on.permit && now - on.refreshed >= PERMIT_REFRESH_S {
                    write(rig, "stall_permit", &[1]).await?;
                    on.refreshed = now;
                }
                rig.c.pause(Duration::from_secs_f64(seek::POLL_S)).await;
                let (p, t) = logged_poll(rig, seek, q15, found).await?;
                pos = p;
                act = seek.feed(t, pos);
            }
            Act::Seated(seat) => {
                halt_drive(rig, b, &mut on).await?;
                interrupted(rig.stop)?;
                seat_bursts(rig, b, &seat, found, lines).await?;
                interrupted(rig.stop)?;
                let (p, t) = logged_poll(rig, seek, 0, found).await?;
                pos = p;
                act = seek.resume(t, pos);
            }
            Act::Done => return halt_drive(rig, b, &mut on).await,
            Act::Stopped { kind, why } => {
                halt_drive(rig, b, &mut on).await?;
                return Err(if kind.aborts() {
                    Halt::Error(why)
                } else {
                    Halt::Refused(why)
                });
            }
        }
    }
}

/// The probe's sign written volatile for this step's drives, its stored
/// value kept so the way out puts it back, and read back.
pub(crate) async fn correct<P: Pipe>(
    rig: &mut Rig<'_, P>,
    ledger: &mut Ledger,
    p: &FromProbe,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    ledger.keep(rig.c, rig.id, rig.d, "drive_polarity").await?;
    write(rig, "drive_polarity", &[p.found as u8]).await?;
    if read_u8(rig, "drive_polarity").await? != p.found as u8 {
        return Err(Halt::Error(format!(
            "drive_polarity {} did not read back after the write",
            p.found
        )));
    }
    lines.push(format!(
        "the probe found the stored drive_polarity {} wrong: {} written volatile for this step's drives and read back; nothing is saved, and the way out puts the stored {} back",
        p.stored, p.found, p.stored
    ));
    Ok(())
}

/// The open-loop setup a drive step on the stops runs under: torque off,
/// the fields it changes kept for the way out, the probe's sign volatile
/// where it found the stored one wrong, the peak cap unset, the soft limits out
/// to the physical range `soft`, OpenLoop, zero duty, the step's dead-man,
/// then torque on. Whether the sign was corrected.
pub(crate) async fn open_loop<P: Pipe>(
    rig: &mut Rig<'_, P>,
    ledger: &mut Ledger,
    probe: &FromProbe,
    soft: &[Vec<u8>; 2],
    deadman_ticks: u16,
    lines: &mut Vec<String>,
) -> Result<bool, Halt> {
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
    let corrected = probe.found != probe.stored;
    if corrected {
        correct(rig, ledger, probe, lines).await?;
    }
    write(rig, "current_limit_counts", &[0, 0]).await?;
    write(rig, "pos_min_soft_counts", &soft[0]).await?;
    write(rig, "pos_max_soft_counts", &soft[1]).await?;
    write(rig, "mode", &[open_loop]).await?;
    write(rig, "goal_duty", &[0, 0]).await?;
    write(rig, "bus_deadman_ticks", &deadman_ticks.to_le_bytes()).await?;
    write(rig, "torque_enable", &[1]).await?;
    Ok(corrected)
}

async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    ledger: &mut Ledger,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    found.corrected = open_loop(rig, ledger, &b.probe, &b.soft, b.deadman_ticks, lines).await?;
    let c = &b.cfg;
    lines.push(format!(
        "volatile: the peak cap unset, the soft limits out to the physical range {:.0}..{:.0}, OpenLoop; dead-man {} ticks ({} s), then torque on at zero duty",
        b.soft_counts.0,
        b.soft_counts.1,
        b.deadman_ticks,
        rest::DEADMAN_S
    ));
    lines.push(format!(
        "drives: approach {:.3} V (the probe's {:.3} V and one {:.3} V step), ceiling {:.3} V ({:.3} A blocked at {PRICE_WORDS}, {:.3} ohm), seat {:.3} V; a poll every {} s under a dead-man of {} ticks ({POLL_DEADMAN_S} s); still is every reading within {:.1} counts of a {} s window's median",
        c.volts(c.approach_q15),
        b.probe.proven_v,
        c.volts(c.step_q15),
        c.volts(c.ceiling_q15),
        c.volts(c.ceiling_q15) / b.probe.r_price_ohm,
        b.probe.r_price_ohm,
        c.volts(c.seat_q15),
        seek::POLL_S,
        b.poll_deadman_ticks,
        c.still_counts,
        seek::WINDOW_S
    ));
    let mut s = Seek::new(b.cfg.clone());
    let ran = drive(rig, b, &mut s, found, lines).await;
    found.seek = Some(s);
    ran
}

/// A stop as its seats read it.
#[derive(Clone, Debug, PartialEq)]
pub struct StopRead {
    pub stop: Stop,
    pub seats: Vec<f64>,
    pub median: f64,
    pub spread: f64,
}

fn stop_read(seek: &Seek, s: Stop) -> Option<StopRead> {
    let seats: Vec<f64> = seek
        .seats()
        .iter()
        .filter(|x| x.stop == s)
        .map(|x| x.pos)
        .collect();
    let m = median(&seats)?;
    let hi = seats.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let lo = seats.iter().copied().fold(f64::INFINITY, f64::min);
    Some(StopRead {
        stop: s,
        seats,
        median: m,
        spread: hi - lo,
    })
}

fn pass(ok: bool) -> Verdict {
    if ok { Verdict::Pass } else { Verdict::Fail }
}

fn check(name: &str, value: String, bound: String, verdict: Verdict) -> Check {
    Check {
        name: name.into(),
        value,
        bound,
        verdict,
    }
}

/// What to do when a check refuses the step.
pub fn advice(name: &str) -> &'static str {
    match name {
        "each stop's two seats agree" => {
            "a stopper that yields or a stall that was no stop: check the horn and the stops, then run the step again"
        }
        "the stops are a travel apart" | "the span fits the nameplate's travel" => {
            "a stall short of a stop, or a nameplate travel_deg that is not this servo's: check travel_deg and that the horn turns freely, then run the step again"
        }
        "the start lies between the stops" => HANDS,
        "kept bursts in the fit" | "burst-to-burst spread of R" => {
            "the bursts scattered: run the step again with the cable seated; if it is refused again, check the motor leads"
        }
        "the two stops read the same R" => {
            "a seat that was no stop, or the brushes bridging two commutator bars at one seat: run the step again; if it is refused again, check the horn and the stops"
        }
        "R is a motor's this board can read" | "L / R is a motor's this board can read" => {
            "not a motor this board can read: check the motor leads and the board"
        }
        "the current is back at zero after each burst" => {
            "the bridge does not let go after a burst: check the board"
        }
        _ => "check the servo, then run the step again",
    }
}

/// The stops' checks, pure: the seats, the span against the converter and
/// the nameplate's travel, the start between them, a stop by a rail.
pub fn judge_stops(seek: &Seek, travel_deg: f64) -> Vec<Check> {
    let (Some(lo), Some(hi)) = (stop_read(seek, Stop::Low), stop_read(seek, Stop::High)) else {
        return vec![check(
            "both stops found",
            format!("{} seats read", seek.seats().len()),
            "two seats at each stop".into(),
            Verdict::Fail,
        )];
    };
    let mut out = Vec::new();
    let said = |r: &StopRead| {
        format!(
            "{} {} ({:.0} apart)",
            r.stop.word(),
            r.seats
                .iter()
                .map(|p| format!("{p:.0}"))
                .collect::<Vec<_>>()
                .join(", "),
            r.spread
        )
    };
    out.push(check(
        "each stop's two seats agree",
        format!("{}; {}", said(&lo), said(&hi)),
        format!("two seats each, within {SEATS_APART_MAX} counts"),
        pass(
            lo.seats.len() == 2
                && hi.seats.len() == 2
                && lo.spread <= SEATS_APART_MAX
                && hi.spread <= SEATS_APART_MAX,
        ),
    ));
    let span = hi.median - lo.median;
    out.push(check(
        "the stops are a travel apart",
        format!(
            "low {:.0}, high {:.0}: {span:.0} counts",
            lo.median, hi.median
        ),
        format!("at least {SPAN_MIN_COUNTS:.0} counts, a quarter of the converter's range"),
        pass(span >= SPAN_MIN_COUNTS),
    ));
    let cpd = span / travel_deg;
    let (a, z) = COUNTS_PER_DEG;
    out.push(check(
        "the span fits the nameplate's travel",
        format!(
            "{span:.0} counts over the nameplate's {travel_deg:.1} deg: {cpd:.2} counts/deg{}",
            if cpd < a {
                ": too few, the stops closer than the travel needs"
            } else if cpd > z {
                ": too many, the stops farther apart than the travel allows"
            } else {
                ""
            }
        ),
        format!("{a} to {z} counts/deg: a {ADC_COUNTS:.0}-count pot over 60 to 360 deg"),
        pass((a..=z).contains(&cpd)),
    ));
    let start = seek.start_pos().unwrap_or(f64::NAN);
    out.push(check(
        "the start lies between the stops",
        format!("{start:.0} against {:.0}..{:.0}", lo.median, hi.median),
        "strictly between".into(),
        pass(lo.median < start && start < hi.median),
    ));
    let near: Vec<String> = [&lo, &hi]
        .iter()
        .filter(|r| r.median <= RAIL_NEAR_COUNTS || r.median >= ADC_TOP - RAIL_NEAR_COUNTS)
        .map(|r| format!("the {} stop at {:.0}", r.stop.word(), r.median))
        .collect();
    out.push(check(
        "no stop at a converter rail",
        if near.is_empty() {
            "both inside".into()
        } else {
            format!("{}: the pot may run out before the stop", near.join(", "))
        },
        format!(
            "more than {RAIL_NEAR_COUNTS} counts from 0 and {ADC_TOP:.0}; accepted and flagged"
        ),
        if near.is_empty() {
            Verdict::Pass
        } else {
            Verdict::Warn
        },
    ));
    out
}

/// The position reads' check, pure: the torn reads the guarded read
/// caught and the single polls over the wrong-way line, each named; a
/// warning, never a refusal.
pub fn judge_reads(seek: Option<&Seek>, torn: &[Torn]) -> Check {
    let mut said: Vec<String> = torn
        .iter()
        .map(|x| format!("leg {} ({}): a torn read, {}", x.leg, x.kind, x.read.said()))
        .collect();
    said.extend(seek.map_or(&[][..], Seek::odd_reads).iter().map(|o| {
        format!(
            "leg {} ({} toward the {} stop): one poll read {:.0} between {:.0} and {:.0}, set aside",
            o.leg,
            o.kind.word(),
            o.toward.word(),
            o.read,
            o.before,
            o.after
        )
    }));
    check(
        "position reads consistent",
        if said.is_empty() {
            "every poll's reads agree".into()
        } else {
            format!("{} odd: {}", said.len(), said.join("; "))
        },
        format!(
            "a poll's two reads within {TORN_COUNTS} counts, else a third and the median; one poll over the {WRONG_WAY_COUNTS:.0}-count wrong-way line set aside, two in a row refuse; a warning, never a refusal"
        ),
        if said.is_empty() {
            Verdict::Pass
        } else {
            Verdict::Warn
        },
    )
}

/// The seated read's checks, pure.
pub fn judge_r(r: &SeatedRead, shots: &[SeatShot]) -> Vec<Check> {
    let mut out = Vec::new();
    let at = |s: usize| move |b: &seated::BurstRead| b.stop == s;
    let kept = [0, 1].map(|s| r.kept().filter(|b| b.stop == s).count());
    let all = kept[0] + kept[1];
    out.push(check(
        "kept bursts in the fit",
        format!("{all} (low {}, high {})", kept[0], kept[1]),
        format!("at least {KEPT_MIN} in all"),
        pass(all >= KEPT_MIN),
    ));
    let (lo, hi) = (r.r(at(0)), r.r(at(1)));
    let enough = kept.map(|k| k >= KEPT_A_STOP_MIN);
    let one = match enough {
        [true, false] => lo.map(|s| (Stop::Low, s, Stop::High, kept[1])),
        [false, true] => hi.map(|s| (Stop::High, s, Stop::Low, kept[0])),
        _ => None,
    };
    match (lo.filter(|_| enough[0]), hi.filter(|_| enough[1]), one) {
        (_, _, Some((s, r, other, n))) => out.push(check(
            "the two stops read the same R",
            format!(
                "the R is the {} stop's alone, {:.4} ohm: the {} stop keeps {n}",
                s.word(),
                r.median,
                other.word()
            ),
            format!(
                "compared when each stop keeps {KEPT_A_STOP_MIN}; a warning when one keeps fewer"
            ),
            Verdict::Warn,
        )),
        (Some(a), Some(b), None) => {
            let apart = (a.median - b.median).abs() / ((a.median + b.median) / 2.0);
            out.push(check(
                "the two stops read the same R",
                format!(
                    "low {:.4} ohm, high {:.4} ohm: {:.2}% apart",
                    a.median,
                    b.median,
                    100.0 * apart
                ),
                format!(
                    "within {:.0}%: more is a bridged seat or a seat that was no stop",
                    100.0 * STOPS_R_SHARE
                ),
                pass(apart <= STOPS_R_SHARE),
            ));
        }
        _ => out.push(check(
            "the two stops read the same R",
            format!("low keeps {}, high {}", kept[0], kept[1]),
            format!("{KEPT_A_STOP_MIN} kept at a stop at least"),
            Verdict::Fail,
        )),
    }
    let bridged = [0, 1].map(|s| r.bridged().filter(|b| b.stop == s).count());
    let cut = bridged[0] + bridged[1];
    let in_fit = cut + all;
    let share = if in_fit == 0 {
        0.0
    } else {
        cut as f64 / in_fit as f64
    };
    out.push(check(
        "few bursts read a bridged brush",
        format!(
            "{cut} of {in_fit} bursts in the fit set aside as bridged (low {}, high {})",
            bridged[0], bridged[1]
        ),
        format!(
            "at most {:.0}% of them; a warning, never a refusal",
            100.0 * BRIDGED_WARN_SHARE
        ),
        if share > BRIDGED_WARN_SHARE {
            Verdict::Warn
        } else {
            Verdict::Pass
        },
    ));
    let sd = r.r_sd_within_groups();
    out.push(check(
        "burst-to-burst spread of R",
        sd.map_or("no group of two kept bursts".into(), |s| {
            format!(
                "{:.2}% (the median of each group's sd over the pooled median)",
                100.0 * s
            )
        }),
        format!("within {:.0}%", 100.0 * R_SD_MAX),
        pass(sd.is_some_and(|s| s <= R_SD_MAX)),
    ));
    let (r0, r1) = probe::R_RANGE_OHM;
    match r.r(|_| true) {
        Some(p) => out.push(check(
            "R is a motor's this board can read",
            format!("{:.4} ohm over {} bursts", p.median, p.n),
            format!("{r0} to {r1} ohm"),
            pass((r0..=r1).contains(&p.median)),
        )),
        None => out.push(check(
            "R is a motor's this board can read",
            "no kept burst".into(),
            "a median".into(),
            Verdict::Fail,
        )),
    }
    let (t0, t1) = probe::TAU_RANGE_PERIODS;
    match r.tau(|_| true) {
        Some(t) => {
            let periods = t.median / r.period_s;
            out.push(check(
                "L / R is a motor's this board can read",
                format!("{:.1} us, {periods:.2} PWM periods", 1e6 * t.median),
                format!("{t0} to {t1} periods"),
                pass((t0..=t1).contains(&periods)),
            ));
        }
        None => out.push(check(
            "L / R is a motor's this board can read",
            "no kept burst".into(),
            "a median".into(),
            Verdict::Fail,
        )),
    }
    let after = shots.iter().map(|s| s.after_a.abs()).fold(0.0, f64::max);
    out.push(check(
        "the current is back at zero after each burst",
        format!("at most {:.1} mA off the rest zero", 1e3 * after),
        format!("within {} mA", 1e3 * probe::AFTER_MAX_A),
        pass(after <= probe::AFTER_MAX_A),
    ));
    out
}

fn spread_json(s: Option<Spread>, scale: f64) -> Value {
    s.map_or(Value::Null, |s| {
        json!({"median": s.median * scale, "n": s.n, "sd_share": if s.sd_share.is_finite() { json!(s.sd_share) } else { Value::Null },
               "lo": s.lo * scale, "hi": s.hi * scale})
    })
}

fn est(value: f64, unit: &str, source: &str) -> Value {
    json!({"value": value, "lo": null, "hi": null, "unit": unit, "source": source})
}

/// Keep every poll in [`POLLS`].
fn keep_polls(sink: &mut dyn Sink, rows: &[PollRow], lines: &mut Vec<String>) -> Result<(), Halt> {
    sink.write(POLLS, polls_csv(rows).as_bytes())?;
    lines.push(format!("kept {} ({} polls)", sink.shown(POLLS), rows.len()));
    Ok(())
}

/// Keep the seated bursts as osc-lab's `burst-K.csv`, each with its meta.
fn keep_bursts(
    sink: &mut dyn Sink,
    servo: &Map<String, Value>,
    shots: &[SeatShot],
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    for (k, s) in shots.iter().enumerate() {
        let plan = json!({"rung": format!("seat {} level {}", s.seat, s.level), "stop": s.stop.word(), "level_share": LEVELS[s.level],
                          "duty_q15": s.shunted.cap.meta.step_q15, "chans": CHANS_DIFF, "after_counts": s.shunted.after_counts,
                          "ntc_c": s.ntc_c, "permit": s.permit});
        let m = meta::burst(servo, &s.shunted.cap, plan, s.shunted.times);
        let (csv, _) = meta::keep_burst(sink, STEP, k as u32, &s.shunted.cap, &m)?;
        lines.push(format!("kept {csv}"));
    }
    Ok(())
}

/// Step 3 on the servo: two seats a stop and their bursts, then the way
/// out. Refused with nothing written when it cannot start; once it has
/// written, it ends through the guard whatever happens, and keeps its
/// captures and result.
pub async fn stops<P: Pipe>(rig: &mut Rig<'_, P>, i: &StopsInputs<'_>) -> Result<Stopped, Refused> {
    let mut b = before(rig, i).await?;
    let mut lines = vec![
        format!("{}; bus at {} baud", b.line.said(), b.line.baud.as_hz()),
        format!(
            "position {:.1} counts, rail {:.2} V, stored drive_polarity {}, the probe's {}, before anything is written",
            b.pos, b.rail_v, b.stored, b.probe.found
        ),
    ];
    let mut ledger = Ledger::default();
    let mut found = Found::default();
    let ran = body(rig, &mut b, &mut ledger, &mut found, &mut lines).await;
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let end = read_pos(rig.c, rig.id, rig.d).await.ok();
    let after = end.as_ref().map(|p| p.pos);
    for (kind, p) in [("before", Some(&b.pos_read)), ("end", end.as_ref())] {
        if let Some(p) = p.filter(|p| p.torn()) {
            found.torn.push(Torn {
                leg: 0,
                kind,
                read: p.clone(),
            });
        }
    }
    let kept = keep_polls(rig.sink, &found.polls, &mut lines).and(keep_bursts(
        rig.sink,
        &b.servo,
        &found.shots,
        &mut lines,
    ));
    let mut read = seated::read(&shots_of(&found.shots), &b.scales);
    read.set_aside_bridged();
    for (x, s) in read.bursts.iter().zip(&found.shots) {
        if let Some(a @ seated::Aside::Bridged { .. }) = &x.aside {
            lines.push(format!(
                "seat {} at the {} stop, {:.0}%, burst {}: set aside, {}",
                s.seat,
                s.stop.word(),
                100.0 * LEVELS[s.level],
                x.k + 1,
                a.said()
            ));
        }
    }
    let mut checks = Vec::new();
    let finished = found.seek.as_ref().is_some_and(Seek::finished);
    if let Some(s) = &found.seek {
        lines.extend(s.path().iter().map(|p| format!("decision: {p}")));
        if finished {
            checks.extend(judge_stops(s, b.travel_deg));
            checks.extend(judge_r(&read, &found.shots));
        }
        checks.push(judge_reads(Some(s), &found.torn));
    }
    lines.push(format!(
        "end position {} counts ({:.1} before)",
        after.map_or("not read".into(), |p| format!("{p:.1}")),
        b.pos
    ));
    if !exit.torque_off {
        checks.push(check(
            "torque off at the end",
            "torque_enable did not read back 0".into(),
            "reads 0".into(),
            Verdict::Fail,
        ));
    }
    let failed: Vec<&Check> = checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .collect();
    let (mut way, mut why) = match (&ran, &kept) {
        (Err(h), _) => (h.way(), Some(h.why())),
        (Ok(()), Err(h)) => (Way::Error, Some(h.why())),
        (Ok(()), Ok(())) if !failed.is_empty() => {
            let mut what: Vec<&str> = failed.iter().map(|c| advice(&c.name)).collect();
            what.dedup();
            (
                Way::Refused,
                Some(format!(
                    "refused by {}: {}",
                    failed
                        .iter()
                        .map(|c| c.name.as_str())
                        .collect::<Vec<_>>()
                        .join(", "),
                    what.join("; ")
                )),
            )
        }
        (Ok(()), Ok(())) if !finished => (
            Way::Refused,
            Some("the stop finder did not finish: run the step again".into()),
        ),
        _ => (Way::Done, None),
    };
    if let Some(w) = why.as_mut() {
        w.push_str(&format!(
            "; the shaft ends at {} counts",
            after.map_or("a position not read".into(), |p| format!("{p:.0}"))
        ));
    }
    let result = result_json(&b, &found, &read, &checks, way, &exit, after);
    let mut text = serde_json::to_vec_pretty(&result).unwrap_or_default();
    text.push(b'\n');
    match rig.sink.write(RESULT, &text) {
        Ok(()) => lines.push(format!("result: {}", rig.sink.shown(RESULT))),
        Err(e) => {
            way = Way::Error;
            why = Some(format!("the result was not kept: {e}"));
        }
    }
    Ok(Stopped {
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

#[allow(clippy::too_many_arguments)]
fn result_json(
    b: &Before,
    found: &Found,
    read: &SeatedRead,
    checks: &[Check],
    way: Way,
    exit: &guard::Exit,
    after: Option<f64>,
) -> Value {
    let seek = found.seek.as_ref();
    let stops = seek.map_or(Value::Null, |s| {
        let one = |stop: Stop| {
            stop_read(s, stop).map_or(Value::Null, |r| {
                json!({"median_counts": r.median, "seats_counts": r.seats, "spread_counts": r.spread})
            })
        };
        let (lo, hi) = (stop_read(s, Stop::Low), stop_read(s, Stop::High));
        let span = lo.as_ref().zip(hi.as_ref()).map(|(l, h)| h.median - l.median);
        json!({
            "low": one(Stop::Low), "high": one(Stop::High),
            "travel_counts": span,
            "counts_per_deg": span.map(|x| x / b.travel_deg),
            "travel_deg": {"value": b.travel_deg, "source": "the nameplate's travel_deg"},
            "start_counts": s.start_pos(),
            "seats": s.seats().iter().map(|x| json!({"n": x.n, "stop": x.stop.word(), "pos_counts": x.pos, "range_counts": x.range, "walk_counts": x.walk})).collect::<Vec<_>>(),
            "legs": s.legs().iter().map(|l| json!({"n": l.n, "kind": l.kind.word(), "toward": l.toward.word(), "from_counts": l.from, "to_counts": l.to, "secs": l.secs, "top_q15": l.top_q15, "top_v": s.cfg().volts(l.top_q15)})).collect::<Vec<_>>(),
            "approaches": s.approaches(),
            "odd_reads": s.odd_reads().iter().map(|o| json!({"leg": o.leg, "kind": o.kind.word(), "toward": o.toward.word(), "before_counts": o.before, "read_counts": o.read, "after_counts": o.after})).collect::<Vec<_>>(),
            "leave_raises": s.leave_raises().iter().map(|x| json!({"at_counts": x.at, "from": x.from.word(), "to_q15": x.q15, "to_v": s.cfg().volts(x.q15)})).collect::<Vec<_>>(),
            "path": s.path(),
        })
    });
    let all = |_: &seated::BurstRead| true;
    let by = |f: &dyn Fn(&seated::BurstRead) -> bool| json!({"r_ohm": spread_json(read.r(f), 1.0), "l_h": spread_json(read.l(f), 1.0), "r_25c_ohm": spread_json(read.r25(f), 1.0)});
    let temps: Vec<f64> = {
        let mut t: Vec<(usize, f64)> = found.shots.iter().map(|s| (s.seat, s.ntc_c)).collect();
        t.dedup();
        t.into_iter().map(|(_, c)| c).collect()
    };
    let src = "seated bursts from rest at both stops, 25 % and 40 % of the rail into the stop, the period-pair fit with no intercept per burst on the nominal scales and the board's edge lags, the first of each group and any whose first window current is over 12 % off its group's others set aside, then any whose own R reads over 8 % under the median R of the run's kept bursts at its level, both stops together (two passes, the median taken again), the median over the kept";
    let mut estimates = Map::new();
    if let Some(r) = read.r(all) {
        estimates.insert("r_seated_ohm".into(), est(r.median, "ohm", src));
    }
    if let Some(r) = read.r25(all) {
        estimates.insert(
            "r_seated_25c_ohm".into(),
            est(r.median, "ohm", "the seated R per burst through copper's slope (the kernel's ALPHA_Q26) from the board NTC read before its seat's bursts, the median"),
        );
    }
    if let Some(l) = read.l(all) {
        estimates.insert("l_seated_h".into(), est(l.median, "H", src));
    }
    if let Some(t) = read.tau(all) {
        estimates.insert("tau_seated_s".into(), est(t.median, "s", src));
    }
    let c = &b.cfg;
    json!({
        "step": STEP,
        "method": METHOD,
        "revision": REVISION,
        "image": b.line.image().name(),
        "capability_flags": b.line.caps,
        "verdict": way.name(),
        "estimates": estimates,
        "stops": stops,
        "seated": {
            "all": by(&all),
            "by_stop": {"low": by(&|x| x.stop == 0), "high": by(&|x| x.stop == 1)},
            "by_level": {"25": by(&|x| x.level == 0), "40": by(&|x| x.level == 1)},
            "r_sd_within_groups": read.r_sd_within_groups(),
            "tau_s": spread_json(read.tau(all), 1.0),
            "pwm_period_s": read.period_s,
            "ntc_c_by_seat": temps,
            "lowered": found.lowered,
            "bridged": read.bridged().count(),
            "bridged_share": seated::BRIDGED_SHARE,
            "bursts": read.bursts.iter().zip(&found.shots).map(|(x, s)| json!({
                "seat": s.seat, "stop": s.stop.word(), "level_share": LEVELS[s.level], "k": x.k,
                "duty_q15": x.step_q15, "pos": x.pos, "ntc_c": x.ntc_c, "permit": s.permit, "after_a": s.after_a,
                "i_first_a": x.i_first_a, "move_share": if x.move_share.is_finite() { json!(x.move_share) } else { Value::Null },
                "r_ohm": x.fit.as_ref().map(|f| f.r_ohm), "l_h": x.fit.as_ref().map(|f| f.l_h), "tau_s": x.fit.as_ref().map(|f| f.tau_s),
                "resid_rms_v": x.fit.as_ref().map(|f| f.resid_rms_v()), "kept": x.kept(), "aside": x.aside.as_ref().map(|a| a.said()),
            })).collect::<Vec<_>>(),
            "scales": {"counts_per_a": b.scales.counts_per_a, "terminal_counts_per_v": b.scales.terminal_counts_per_v,
                       "terminal_offset_counts": b.scales.terminal_offset_counts, "window_scale": b.scales.window_scale,
                       "lags_ticks": {"on": b.scales.lags.on, "tap_on": b.scales.lags.tap_on, "off": b.scales.lags.off, "settle": b.scales.lags.settle},
                       "source": "the nominal sense block read off the servo and the board's burst_edges; no edge term, no window scale"},
        },
        "drives": {
            "approach_q15": c.approach_q15, "approach_v": c.volts(c.approach_q15),
            "ceiling_q15": c.ceiling_q15, "ceiling_v": c.volts(c.ceiling_q15),
            "seat_q15": c.seat_q15, "seat_v": c.volts(c.seat_q15),
            "step_q15": c.step_q15, "still_counts": c.still_counts,
            "r_price_ohm": b.probe.r_price_ohm, "proven_v": b.probe.proven_v,
            "poll_s": seek::POLL_S, "poll_deadman_s": POLL_DEADMAN_S, "window_s": seek::WINDOW_S,
            "hold_s": seek::HOLD_S, "read_s": seek::READ_S, "leave_counts": seek::LEAVE_COUNTS, "leg_max_s": seek::LEG_MAX_S,
        },
        "drive_polarity": {"stored": b.stored, "driven": b.probe.found, "corrected_volatile": found.corrected},
        "position_reads": {
            "polls_csv": POLLS, "polls": found.polls.len(), "torn_counts": TORN_COUNTS,
            "torn": found.torn.iter().map(|x| json!({"leg": x.leg, "kind": x.kind, "reads": x.read.reads, "pos_counts": x.read.pos})).collect::<Vec<_>>(),
        },
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
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;

    use futures_lite::future::block_on;
    use osc_client::descriptor::Descriptor;
    use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
    use osc_client::pipe::{Pipe, PipeError};
    use osc_client::{BaudRate, Client, Id, Inst, Opcode};

    use super::*;
    use crate::sink::MemSink;

    const OSC_EXCHANGE: u8 = 0x20;
    const ID: Id = Id::new(1);

    type Log = Rc<RefCell<Vec<(u16, Vec<u8>)>>>;
    type OnWrite = Box<dyn FnMut(u16, &[u8], &mut FakePipe)>;
    type OnRead = Box<dyn FnMut(u16, &mut FakePipe)>;

    /// The fake adapter, every write logged and handed to a hook, every
    /// read's address handed to another before the read is answered.
    struct Spy {
        inner: FakePipe,
        log: Log,
        on_write: OnWrite,
        on_read: OnRead,
    }

    impl Pipe for Spy {
        async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
            if let [
                _,
                _,
                OSC_EXCHANGE,
                _,
                _,
                _,
                _,
                inst,
                _,
                _,
                _,
                _,
                payload @ ..,
            ] = bytes
                && payload.len() >= 2
                && Inst(*inst).opcode() == Some(Opcode::Write)
            {
                let addr = u16::from_le_bytes([payload[0], payload[1]]);
                self.log.borrow_mut().push((addr, payload[2..].to_vec()));
                (self.on_write)(addr, &payload[2..], &mut self.inner);
            }
            if let [_, _, OSC_EXCHANGE, _, _, _, _, inst, _, _, _, _, a0, a1, ..] = bytes
                && Inst(*inst).opcode() == Some(Opcode::Read)
            {
                (self.on_read)(u16::from_le_bytes([*a0, *a1]), &mut self.inner);
            }
            self.inner.send(bytes).await
        }

        async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
            self.inner.recv().await
        }

        async fn pause(&mut self, d: Duration) {
            self.inner.pause(d).await
        }
    }

    fn descriptor() -> Descriptor {
        Descriptor::parse(
            &std::fs::read_to_string(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../descriptors/osc-servo/0.1.json"
            ))
            .expect("descriptor"),
        )
        .expect("parses")
    }

    fn addr(d: &Descriptor, name: &str) -> u16 {
        field(d, name).expect(name).addr
    }

    /// The shaft at rest at 2041 counts, the zero at 1206.
    fn still() -> Vec<TelSample> {
        still_at(2041)
    }

    /// The shaft at rest at `pos`.
    fn still_at(pos: u16) -> Vec<TelSample> {
        (0..4096u32)
            .map(|i| TelSample {
                tick: TelemetryTick {
                    _scan: [0; 9],
                    pos,
                    vbus_raw: 1241,
                    ntc_raw: 2010,
                    current_raw: 1206,
                    vmotor_a: 773,
                    vmotor_b: 772,
                    duty_ticks: 0,
                    current_trough: 1206,
                    i_tick_counts: 0,
                    vdiff_counts: 0,
                    vmotor_a_trough: 773,
                    vmotor_b_trough: 772,
                    tick_flags: 0,
                    sample_tick: i,
                },
                vbus_counts: 2000,
            })
            .collect()
    }

    fn servo(on_write: OnWrite, on_read: OnRead) -> (Client<Spy>, Log) {
        let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
        inner.seed_calibrated(0);
        inner.set_track(0, still());
        let log = Log::default();
        let c = block_on(Client::connect(Spy {
            inner,
            log: log.clone(),
            on_write,
            on_read,
        }))
        .expect("connect");
        (c, log)
    }

    fn rest_json() -> Value {
        let est = |v: f64| json!({"value": v});
        json!({
            "verdict": "accepted",
            "estimates": {
                "current_zero_awake_counts": est(1206.0),
                "pot_rest_sd_counts": est(0.8),
                "tap_a_counts": est(773.0),
                "tap_b_counts": est(772.0),
            },
            "channels": {"raw": {"pos": {"pattern_gain": 0.0}}},
            "frame_ripple": {"pattern_counts": null},
        })
    }

    /// An accepted probe that proved the sign at 0.48 V with its ceiling
    /// at 0.945 V, the stored sign `stored`, found `found`.
    fn probe_json(stored: bool, found: bool) -> Value {
        json!({
            "verdict": "accepted",
            "drive_polarity": {"stored": stored, "found": found, "stored_right": stored == found},
            "part_b": {
                "ceiling_v": 0.945,
                "r_price_ohm": 3.78,
                "pulses": [
                    {"repeat": false, "unwedge": false, "volts": 0.48},
                    {"repeat": true, "unwedge": false, "volts": 0.48},
                ],
            },
        })
    }

    struct Ran {
        stopped: Stopped,
        writes: Vec<(&'static str, Vec<u8>)>,
        c: Client<Spy>,
        stored: bool,
        files: std::collections::BTreeMap<String, Vec<u8>>,
    }

    /// The step on the simulated servo, which has no motor and runs no
    /// shunt burst: the shaft never moves, so the first approach seats
    /// where it starts, and the first burst never completes.
    fn run(found_flips: bool, on_write: impl Fn(&Descriptor) -> OnWrite, stop: &AtomicBool) -> Ran {
        run_read(found_flips, on_write, |_| Box::new(|_, _| {}), stop)
    }

    fn run_read(
        found_flips: bool,
        on_write: impl Fn(&Descriptor) -> OnWrite,
        on_read: impl Fn(&Descriptor) -> OnRead,
        stop: &AtomicBool,
    ) -> Ran {
        let d = descriptor();
        let (mut c, log) = servo(on_write(&d), on_read(&d));
        let stored = block_on(c.read(ID, addr(&d, "drive_polarity"), 1)).expect("read")[0] != 0;
        let np: Nameplate =
            serde_json::from_str(include_str!("../../testdata/mg90-d/nameplate.json"))
                .expect("nameplate");
        let board: Board =
            serde_json::from_str(include_str!("../../boards/osc-dev-v006.json")).expect("board");
        let thermal: ThermalInputs =
            serde_json::from_str(include_str!("../../testdata/mg90-d/thermal.json"))
                .expect("thermal");
        let (rest, probe) = (rest_json(), probe_json(stored, stored != found_flips));
        let t = Cell::new(0.0);
        let clock = || {
            t.set(t.get() + 0.011);
            t.get()
        };
        let mut sink = MemSink::default();
        let stopped = {
            let mut rig = Rig {
                c: &mut c,
                id: ID,
                d: &d,
                sink: &mut sink,
                stop,
                clock: &clock,
            };
            block_on(stops(
                &mut rig,
                &StopsInputs {
                    nameplate: &np,
                    board: &board,
                    thermal: Some(&thermal),
                    rest: &rest,
                    probe: &probe,
                },
            ))
            .expect("starts")
        };
        let names = [
            "bus_deadman_ticks",
            "goal_duty",
            "stall_permit",
            "drive_polarity",
        ];
        let writes = log
            .borrow()
            .iter()
            .filter_map(|(a, b)| {
                names
                    .iter()
                    .find(|n| addr(&d, n) == *a)
                    .map(|n| (*n, b.clone()))
            })
            .collect();
        Ran {
            stopped,
            writes,
            c,
            stored,
            files: sink.files,
        }
    }

    fn polls_of(r: &Ran) -> Vec<Vec<String>> {
        let text =
            String::from_utf8(r.files.get(POLLS).expect("the polls kept").clone()).expect("utf8");
        let mut rows = text.lines();
        assert_eq!(
            rows.next(),
            Some("t_s,leg,kind,duty_q15,pos,read_1,read_2,read_3,fault_flags,halt")
        );
        rows.map(|l| l.splitn(10, ',').map(str::to_string).collect())
            .collect()
    }

    /// The polls are kept at every way out, the result names them, and a
    /// poll that halts keeps why.
    #[test]
    fn every_poll_is_kept_at_every_way_out() {
        let latch = |d: &Descriptor| -> OnWrite {
            let duty = addr(d, "goal_duty");
            Box::new(move |a, b, pipe| {
                if a == duty && b != [0, 0] {
                    pipe.sim_mut()
                        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = BIT_DEADMAN);
                }
            })
        };
        let r = run(false, latch, &AtomicBool::new(false));
        assert_eq!(r.stopped.ended.way, Way::Refused);
        let rows = polls_of(&r);
        assert_eq!(rows.len(), 2, "{rows:?}");
        assert_eq!(&rows[0][1..6], ["1", "start", "0", "2041", "2041"]);
        assert_eq!(rows[0][8], "", "the start reads no faults");
        assert_eq!(&rows[1][1..3], ["1", "approach"]);
        assert!(rows[1][3].starts_with('-'), "{:?}", rows[1]);
        assert_eq!(rows[1][8], format!("{BIT_DEADMAN:#04x}"), "{:?}", rows[1]);
        assert!(
            rows[1][9].starts_with(&format!("\"{STALLED}")),
            "{:?}",
            rows[1]
        );
        let res = r.stopped.result.as_ref().expect("result");
        assert_eq!(res["position_reads"]["polls_csv"], POLLS);
        assert_eq!(res["position_reads"]["polls"], 2);

        let r = run(false, |_| Box::new(|_, _, _| {}), &AtomicBool::new(false));
        let rows = polls_of(&r);
        assert!(rows.len() > 10, "{rows:?}");
        assert!(
            rows.iter()
                .all(|x| x[5] == "2041" && x[6] == "2041" && x[7].is_empty())
        );
        let (v, said) = verdict(&r.stopped.checks, "position reads consistent");
        assert_eq!(v, Verdict::Pass, "{said}");
    }

    /// A leave that carries the shaft up through 2048 (0x0800) while one
    /// read of the pos is torn, the high byte of 2041 (0x07F9) and the low
    /// byte of 2048: 0x0700, 1792. The guarded read takes a third and uses
    /// 2048, the finder never sees 1792, and the check counts it.
    #[test]
    fn a_torn_read_on_a_leave_is_survived_with_the_true_position() {
        let duty = Rc::new(Cell::new(0i16));
        let on_write = {
            let duty = duty.clone();
            move |d: &Descriptor| -> OnWrite {
                let (at, duty) = (addr(d, "goal_duty"), duty.clone());
                Box::new(move |a, b, _| {
                    if a == at {
                        duty.set(i16::from_le_bytes([b[0], b[1]]));
                    }
                })
            }
        };
        let on_read = {
            let duty = duty.clone();
            move |d: &Descriptor| -> OnRead {
                let (pos_at, faults_at, duty) =
                    (addr(d, "pos"), addr(d, "fault_flags"), duty.clone());
                let (mut pos, mut torn) = (2041u16, 0u8);
                Box::new(move |a, pipe| {
                    // the leave from the start drives +duty; it moves the
                    // shaft 7 counts a poll for 200 counts, then holds it
                    if a == faults_at && duty.get() > 0 && pos < 2241 {
                        let next = pos + 7;
                        if next >> 8 != pos >> 8 {
                            pipe.set_track(0, still_at((pos & 0xff00) | (next & 0x00ff)));
                            torn = 2;
                        } else {
                            pipe.set_track(0, still_at(next));
                        }
                        pos = next;
                    } else if a == pos_at && torn > 0 {
                        torn -= 1;
                        if torn == 0 {
                            pipe.set_track(0, still_at(pos));
                        }
                    }
                })
            }
        };
        let r = run_read(false, on_write, on_read, &AtomicBool::new(false));
        let e = &r.stopped.ended;
        let why = e.why.as_deref().unwrap_or("");
        assert_eq!(e.way, Way::Refused, "{why}");
        assert!(why.contains("blocked between the stops"), "{why}");
        let rows = polls_of(&r);
        let torn: Vec<&Vec<String>> = rows.iter().filter(|x| !x[7].is_empty()).collect();
        assert_eq!(torn.len(), 1, "{rows:?}");
        assert_eq!(
            &torn[0][1..8],
            [
                "2",
                "leave",
                torn[0][3].as_str(),
                "2048",
                "1792",
                "2048",
                "2048"
            ]
        );
        assert!(
            rows.iter().all(|x| x[4] != "1792"),
            "the torn read is never used"
        );
        let (v, said) = verdict(&r.stopped.checks, "position reads consistent");
        assert_eq!(v, Verdict::Warn);
        assert!(
            said.contains("leg 2 (leave): a torn read, reads 1792, 2048, 2048, 2048 used"),
            "{said}"
        );
        let res = r.stopped.result.as_ref().expect("result");
        assert_eq!(
            res["position_reads"]["torn"][0]["reads"],
            json!([1792, 2048, 2048])
        );
        assert_eq!(res["stops"]["odd_reads"], json!([]));
    }

    fn q(v: &[u8]) -> i16 {
        i16::from_le_bytes([v[0], v[1]])
    }

    fn cfg_of(rail_v: f64) -> SeekCfg {
        let p = FromProbe::of(&probe_json(true, true)).expect("probe");
        duties(&p, rail_v, 4.8, seek::still_band(0.8))
    }

    /// One approach and seat: the permit, the drive's dead-man, the duty
    /// stepping up to the ceiling toward the low stop, the seat duty, the
    /// zero, the step's dead-man back, the permit off; no duty over the
    /// ceiling in any write; the way out ends torque off.
    #[test]
    fn an_approach_and_seat_write_in_order_under_the_ceiling() {
        let stop = AtomicBool::new(false);
        // the first duty carries the shaft 141 counts on, where it rests
        let arrive = |d: &Descriptor| -> OnWrite {
            let (duty, mut moved) = (addr(d, "goal_duty"), false);
            Box::new(move |a, b, pipe| {
                if a == duty && b != [0, 0] && !moved {
                    moved = true;
                    pipe.set_track(0, still_at(1900));
                }
            })
        };
        let r = run(false, arrive, &stop);
        let e = &r.stopped.ended;
        assert_eq!(e.way, Way::Error, "no shunt burst on the fake: {:?}", e.why);
        assert!(
            e.why.as_deref().unwrap_or("").contains("seat 1 burst 1"),
            "{:?}",
            e.why
        );
        let rail_v = r.stopped.result.as_ref().expect("result")["rail_v"]
            .as_f64()
            .expect("rail");
        let c = cfg_of(rail_v);
        let short = (POLL_DEADMAN_S * 20_000.0) as u16;
        let long = (rest::DEADMAN_S * 20_000.0) as u16;
        let at = r
            .writes
            .iter()
            .position(|w| *w == ("bus_deadman_ticks", short.to_le_bytes().to_vec()))
            .expect("the drive's dead-man");
        let end = at
            + r.writes[at..]
                .iter()
                .position(|w| *w == ("bus_deadman_ticks", long.to_le_bytes().to_vec()))
                .expect("the step's dead-man back");
        assert_eq!(
            r.writes[at - 1],
            ("stall_permit", vec![1]),
            "{:?}",
            r.writes
        );
        assert_eq!(r.writes[end - 1], ("goal_duty", vec![0, 0]));
        assert_eq!(r.writes[end + 1], ("stall_permit", vec![0]));
        let duties: Vec<i16> = r.writes[at..end - 1]
            .iter()
            .filter(|w| w.0 == "goal_duty")
            .map(|w| q(&w.1))
            .collect();
        assert_eq!(duties, [-c.approach_q15, -c.seat_q15], "never raised");
        assert!(
            r.writes[at + 1..end - 1]
                .iter()
                .all(|w| w.0 == "goal_duty" || *w == ("stall_permit", vec![1])),
            "only the duty and the permit's refresh inside the press: {:?}",
            &r.writes[at..end]
        );
        for w in r.writes.iter().filter(|w| w.0 == "goal_duty") {
            assert!(q(&w.1).unsigned_abs() <= c.ceiling_q15 as u16, "{w:?}");
        }
        assert!(e.exit.torque_off, "{}", e.exit);
        let mut c = r.c;
        let d = descriptor();
        assert_eq!(
            block_on(c.read(ID, addr(&d, "torque_enable"), 1)).expect("read"),
            [0]
        );
        assert_eq!(
            block_on(c.read(ID, addr(&d, "goal_duty"), 2)).expect("read"),
            [0, 0]
        );
        let lines = e.lines.join("\n");
        assert!(lines.contains("seat 1: the low stop reads 1900"), "{lines}");
        assert!(
            lines.contains("leg 1, approach toward the low stop"),
            "{lines}"
        );
    }

    /// A dead-man latched in a drive refuses with the stall sentence; an
    /// interrupt stops it; both end torque off through the guard.
    #[test]
    fn a_latched_dead_man_or_an_interrupt_ends_torque_off() {
        let latch = |d: &Descriptor| -> OnWrite {
            let duty = addr(d, "goal_duty");
            Box::new(move |a, b, pipe| {
                if a == duty && b != [0, 0] {
                    pipe.sim_mut()
                        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = BIT_DEADMAN);
                }
            })
        };
        let r = run(false, latch, &AtomicBool::new(false));
        let e = &r.stopped.ended;
        assert_eq!(e.way, Way::Refused);
        let why = e.why.as_deref().unwrap_or("");
        assert!(
            why.starts_with(STALLED) && why.ends_with("the shaft ends at 2041 counts"),
            "{why}"
        );
        assert!(e.exit.torque_off, "{}", e.exit);

        let stop = Rc::new(AtomicBool::new(false));
        let set = stop.clone();
        let hook = move |d: &Descriptor| -> OnWrite {
            let (duty, set) = (addr(d, "goal_duty"), set.clone());
            Box::new(move |a, b, _| {
                if a == duty && b != [0, 0] {
                    set.store(true, Ordering::SeqCst);
                }
            })
        };
        let r = run(false, hook, &stop);
        let e = &r.stopped.ended;
        assert_eq!(e.way, Way::Interrupted, "{:?}", e.why);
        assert!(e.exit.torque_off, "{}", e.exit);
        let duties = r
            .writes
            .iter()
            .filter(|w| w.0 == "goal_duty" && w.1 != [0, 0])
            .count();
        assert_eq!(duties, 1, "one duty, then the way out");
    }

    /// The probe found the stored sign wrong: this step writes the found
    /// sign volatile before any drive, says so without claiming the servo
    /// keeps it, and the way out puts the stored sign back.
    #[test]
    fn a_sign_the_probe_found_wrong_is_driven_volatile_and_put_back() {
        let r = run(true, |_| Box::new(|_, _, _| {}), &AtomicBool::new(false));
        let e = &r.stopped.ended;
        let first_duty = r
            .writes
            .iter()
            .position(|w| w.0 == "goal_duty" && w.1 != [0, 0]);
        let pol = r
            .writes
            .iter()
            .position(|w| *w == ("drive_polarity", vec![!r.stored as u8]))
            .expect("the found sign written");
        assert!(first_duty.is_some_and(|f| pol < f));
        assert!(
            e.exit
                .lines
                .iter()
                .any(|l| l == "drive_polarity put back, read back"),
            "{}",
            e.exit
        );
        let mut c = r.c;
        let d = descriptor();
        assert_eq!(
            block_on(c.read(ID, addr(&d, "drive_polarity"), 1)).expect("read"),
            [r.stored as u8]
        );
        let lines = e.lines.join("\n");
        assert!(
            lines.contains("written volatile") && lines.contains("puts the stored"),
            "{lines}"
        );
        assert!(!lines.contains("now holds"), "{lines}");
    }

    /// The envelope this step builds for the bench servo's class, priced
    /// at the probe's R.
    fn envelope(rail_v: f64) -> Envelope {
        let np: Nameplate =
            serde_json::from_str(include_str!("../../testdata/mg90-d/nameplate.json"))
                .expect("nameplate");
        let board: Board =
            serde_json::from_str(include_str!("../../boards/osc-dev-v006.json")).expect("board");
        let thermal: ThermalInputs =
            serde_json::from_str(include_str!("../../testdata/mg90-d/thermal.json"))
                .expect("thermal");
        let mut f = Facts::new(&np, rail_v);
        f.polarity_proven = true;
        f.ntc_c = Some(25.0);
        f.r = Some(Resistance {
            ohm: 3.78,
            source: PRICE_WORDS.into(),
        });
        Envelope::new(f, HeatModel::new(&board, &thermal, 3.78, 1108.45, 20_000.0))
    }

    /// Every window of a run that meets nothing but its stops, and every
    /// seat's eight bursts, fit the envelope; so does a leg that runs its
    /// whole time at the ceiling, the worst a leg may price.
    #[test]
    fn a_whole_run_on_the_bench_servos_class_is_admitted() {
        let c = cfg_of(5.0);
        let mut env = envelope(5.0);
        let rated = 600.0 / 1.0 * 4096.0 / 184.33;
        let (lo, hi) = (432.0, 3626.0);
        let mut pos = 2000.0f64;
        let mut m = Seek::new(c.clone());
        let mut t = 0.0;
        let mut act = m.start(t, pos);
        let mut rise = 0.0f64;
        for _ in 0..100_000 {
            match act {
                Act::Drive { q15, permit, price } => {
                    if let Some(span) = price {
                        let r = window_rung("w", q15, span, permit, pos, &c, 4.8, rated);
                        rise = rise.max(env.admit(&r).expect("admitted").rise_c);
                    }
                    let v = c.volts(q15) - 0.4;
                    if v > 0.0 {
                        pos =
                            (pos + (q15 as f64).signum() * 3000.0 * v * seek::POLL_S).clamp(lo, hi);
                    }
                    t += seek::POLL_S;
                    act = m.feed(t, pos);
                }
                Act::Seated(s) => {
                    let dir = toward(s.stop);
                    for share in LEVELS {
                        let (q, lowered) = level_q15(&mut env, share, 5.0, dir, s.pos, 2.2, None)
                            .expect("a level");
                        assert!(lowered.is_none(), "{lowered:?}");
                        for _ in 0..BURSTS_A_LEVEL {
                            let v = q as f64 / Q15 * 5.0;
                            at_burst_r(&mut env, v, 2.2, None, |e| {
                                e.admit(&burst_rung("b".into(), q * dir as i16, s.pos))
                            })
                            .expect("a burst");
                        }
                    }
                    t += 1.5;
                    act = m.resume(t, pos);
                }
                Act::Done => break,
                Act::Stopped { why, .. } => panic!("{why}"),
            }
        }
        assert!(m.finished(), "{:?}", m.path());
        println!("whole run: peak priced rise {rise:.2} C");
        let mut worst = envelope(5.0);
        let n = (seek::LEG_MAX_S / seek::WINDOW_S).ceil() as usize;
        for _ in 0..n {
            let r = window_rung(
                "w",
                -c.ceiling_q15,
                seek::WINDOW_S,
                true,
                2000.0,
                &c,
                4.8,
                rated,
            );
            rise = rise.max(worst.admit(&r).expect("a leg at the ceiling").rise_c);
        }
        println!(
            "a leg at the ceiling for {} s: peak priced rise {rise:.2} C",
            seek::LEG_MAX_S
        );
    }

    /// The machine run to its end on a shaft between `lo` and `hi` from
    /// `start`, a jam at `jam` stopping it going down at any duty.
    fn found(start: f64, lo: f64, hi: f64, jam: Option<f64>) -> Seek {
        let c = cfg_of(5.0);
        let mut pos = start;
        let mut m = Seek::new(c.clone());
        let mut t = 0.0;
        let mut act = m.start(t, pos);
        for _ in 0..100_000 {
            match act {
                Act::Drive { q15, .. } => {
                    let v = c.volts(q15) - 0.4;
                    if v > 0.0 {
                        let mut to = pos + (q15 as f64).signum() * 3000.0 * v * seek::POLL_S;
                        if let Some(j) = jam.filter(|j| q15 < 0 && pos >= *j) {
                            to = to.max(j);
                        }
                        pos = to.clamp(lo, hi);
                    }
                    t += seek::POLL_S;
                    act = m.feed(t, pos);
                }
                Act::Seated(_) => {
                    t += 1.5;
                    act = m.resume(t, pos);
                }
                Act::Done => return m,
                Act::Stopped { why, .. } => panic!("{why}"),
            }
        }
        panic!("never ends")
    }

    fn verdict(checks: &[Check], name: &str) -> (Verdict, String) {
        let c = checks
            .iter()
            .find(|c| c.name == name)
            .unwrap_or_else(|| panic!("{name}: {checks:?}"));
        (c.verdict, c.value.clone())
    }

    /// The stops of the bench servo pass; a jam short of the low stop that
    /// seated twice is refused by the span against the nameplate's travel;
    /// a stop by a converter rail is accepted and flagged.
    #[test]
    fn the_stops_are_judged_by_their_seats_span_and_start() {
        let travel = 184.33;
        let ok = judge_stops(&found(2000.0, 432.0, 3626.0, None), travel);
        assert!(ok.iter().all(|c| c.verdict == Verdict::Pass), "{ok:?}");

        let jam = judge_stops(&found(2000.0, 432.0, 3626.0, Some(1900.0)), travel);
        let (v, said) = verdict(&jam, "the span fits the nameplate's travel");
        assert_eq!(v, Verdict::Fail, "{said}");
        assert!(said.contains("too few"), "{said}");
        assert!(advice("the span fits the nameplate's travel").contains("travel_deg"));

        let short = judge_stops(&found(2000.0, 1500.0, 2400.0, None), travel);
        assert_eq!(
            verdict(&short, "the stops are a travel apart").0,
            Verdict::Fail
        );

        let rail = judge_stops(&found(2000.0, 4.0, 3626.0, None), travel);
        let (v, said) = verdict(&rail, "no stop at a converter rail");
        assert!(
            v == Verdict::Warn && said.contains("low stop at 4"),
            "{said}"
        );
        assert!(rail.iter().all(|c| c.verdict != Verdict::Fail), "{rail:?}");
    }

    /// Kept bursts of `r` ohm at a stop, three a group at both levels and
    /// both seats, with a 0.5 % wobble.
    fn burst_reads(stop: usize, r: f64, n: usize) -> Vec<seated::BurstRead> {
        (0..4 * n)
            .map(|k| {
                let r = r * (1.0 + 0.005 * if k % 2 == 0 { 1.0 } else { -1.0 });
                seated::BurstRead {
                    group: 4 * stop + k / n,
                    stop,
                    level: (k / n) % 2,
                    k: k % n + 1,
                    ntc_c: 25.0,
                    step_q15: -8191,
                    pos: 432,
                    i_first_a: 0.058,
                    move_share: 0.0,
                    fit: Some(crate::fit::rise::Rise {
                        r_ohm: r,
                        l_h: 0.82e-3,
                        tau_s: 0.82e-3 / r,
                        v0_v: 0.0,
                        pairs: 8,
                        resid_v: vec![],
                    }),
                    aside: None,
                }
            })
            .collect()
    }

    /// The two stops' R within 10 % pass; 11 % apart refuses and names both;
    /// under six kept bursts in all refuses; a stop that keeps under two
    /// leaves the R the other stop's alone, a warning.
    #[test]
    fn the_seated_r_is_judged_across_the_stops() {
        let read = |a: f64, b: f64, n: usize| SeatedRead {
            bursts: [burst_reads(0, a, n), burst_reads(1, b, n)].concat(),
            period_s: 50e-6,
        };
        // the two seats of a bench run: about 7 % apart, two rotor angles
        for (a, b) in [(4.20, 4.30), (4.40, 4.12)] {
            let ok = judge_r(&read(a, b, 3), &[]);
            assert!(ok.iter().all(|c| c.verdict == Verdict::Pass), "{ok:?}");
        }
        let apart = judge_r(&read(4.20, 4.70, 3), &[]);
        let (v, said) = verdict(&apart, "the two stops read the same R");
        assert_eq!(v, Verdict::Fail);
        assert!(
            said.contains("low 4.2") && said.contains("high 4.7"),
            "{said}"
        );
        let lone = |n: usize| SeatedRead {
            bursts: [
                burst_reads(0, 4.20, 3),
                burst_reads(1, 3.0, 1)[..n].to_vec(),
            ]
            .concat(),
            period_s: 50e-6,
        };
        let one = judge_r(&lone(1), &[]);
        assert!(one.iter().all(|c| c.verdict != Verdict::Fail), "{one:?}");
        let (v, said) = verdict(&one, "the two stops read the same R");
        assert_eq!(v, Verdict::Warn);
        assert!(
            said.contains("the low stop's alone") && said.contains("high stop keeps 1"),
            "{said}"
        );
        let few = SeatedRead {
            bursts: [
                burst_reads(0, 4.20, 1),
                burst_reads(1, 4.25, 1)[..1].to_vec(),
            ]
            .concat(),
            period_s: 50e-6,
        };
        let few = judge_r(&few, &[]);
        assert_eq!(verdict(&few, "kept bursts in the fit").0, Verdict::Fail);
    }

    /// Bursts reading low at one stop are set aside by their own R against
    /// the level's median, both stops together; over a quarter of the fit
    /// set aside warns, and the stops then agree.
    #[test]
    fn bridged_bursts_are_set_aside_by_their_own_r() {
        let mut low = burst_reads(1, 4.10, 3);
        for b in low.iter_mut().filter(|b| b.k > 1) {
            if let Some(f) = b.fit.as_mut() {
                f.r_ohm = 3.2;
            }
        }
        let mut r = SeatedRead {
            bursts: [burst_reads(0, 4.20, 3), low].concat(),
            period_s: 50e-6,
        };
        let before = judge_r(&r, &[]);
        assert_eq!(
            verdict(&before, "the two stops read the same R").0,
            Verdict::Fail
        );
        r.set_aside_bridged();
        assert_eq!(r.bridged().count(), 8, "{:?}", r.bursts);
        assert!(r.bridged().all(|b| b.stop == 1 && b.k > 1));
        let after = judge_r(&r, &[]);
        assert!(
            after.iter().all(|c| c.verdict != Verdict::Fail),
            "{after:?}"
        );
        let (v, said) = verdict(&after, "few bursts read a bridged brush");
        assert_eq!(v, Verdict::Warn, "8 of 24: {said}");
        assert!(said.contains("8 of 24"), "{said}");
        let (v, _) = verdict(&after, "the two stops read the same R");
        assert_eq!(v, Verdict::Pass);
    }

    #[test]
    fn the_duties_come_from_the_probes_volts() {
        let c = cfg_of(5.0);
        let v = |x: i16| c.volts(x);
        assert!((v(c.ceiling_q15) - 0.945).abs() < 2e-4);
        assert!((v(c.approach_q15) - (0.48 + 0.04 * 4.8)).abs() < 2e-4);
        assert!((v(c.seat_q15) - 0.945 / 2.0).abs() < 2e-4);
        assert!((v(c.step_q15) - 0.192).abs() < 2e-4);
        // the approach is capped at the ceiling
        let mut p = FromProbe::of(&probe_json(true, true)).expect("probe");
        p.proven_v = 0.9;
        let c = duties(&p, 5.0, 4.8, 3.0);
        assert_eq!(c.approach_q15, c.ceiling_q15);
        let mut j = probe_json(true, true);
        j["part_b"]["pulses"][1]["repeat"] = json!(false);
        assert!(FromProbe::of(&j).is_err(), "no level proved the sign");
        j["verdict"] = json!("refused");
        assert!(
            FromProbe::of(&j)
                .expect_err("refused")
                .0
                .contains("accepted probe")
        );
    }

    #[test]
    fn a_burst_at_a_seat_holds_the_permit_only_outside_the_soft_limits() {
        let soft = (82.0, 3792.0);
        assert!(!needs_permit(2000.0, soft, 3.0));
        assert!(
            needs_permit(84.0, soft, 3.0),
            "inside by less than the band"
        );
        assert!(needs_permit(80.0, soft, 3.0));
        assert!(needs_permit(3800.0, soft, 3.0));
    }
}
