//! Step 2 part B, polarity on a free shaft: open-loop pulses of 150 ms,
//! each followed by 150 ms of coast, from the bridge's shortest pulse up
//! by 4 % of the stated voltage a level, a +duty and a -duty pulse at each
//! level, until the pot moves past the larger of 20 counts and 12 x its
//! rest sd; then the same pair once more. The ceiling is the volts whose
//! blocked current is half the gear's stop-hold ceiling at the pricing R,
//! so arriving at a stop is a light press, not a strike.
//!
//! A metal train that moves neither way at the ceiling may be wedged on a
//! stop: one press at the speed voltage of at most 0.05 s each way, the
//! stall permit held for it alone, frees it, and the ladder starts again
//! from the shortest pulse. The press is short so a stiff shaft it frees
//! at mid travel cannot reach a stop at speed. A press proves no sign; the
//! ladder does. One attempt a run.
//!
//! [`Ladder`] is the decision, pure: fed what each pulse did, it gives the
//! next pulse or a verdict. [`seen`] reads a pulse's rows. [`body`] only
//! prices, drives, captures and feeds them.
//!
//! The kernel's sign rule: a logical +duty moves the counts up when
//! `drive_polarity` is right. The taps read the wiring's frame, so a
//! coast's back-EMF has the bridge's sign, `duty x (polarity ? 1 : -1)`,
//! whichever way the leads are wired: it says the motor turned, the pot
//! says which way the output went.

use osc_client::pipe::Pipe;
use serde_json::{Map, Value, json};

use super::{Halt, Rig, Taken, interrupted, keep_alive, rest, take_once, write};
use crate::capture::ripple::{self, Pattern};
use crate::capture::rows::{Layout, TelFrame};
use crate::envelope::{Envelope, FREE_MARGIN, Kind, Rung, UNWEDGE_MAX_S};
use crate::fit::mean;

pub const PULSE_S: f64 = 0.150;
pub const COAST_S: f64 = 0.150;
/// The host's turn around a pulse's rows, priced on top of them: the
/// reads before the arm, and after LAST the reads and the zero write, each
/// a USB round trip of about a millisecond.
pub const HOST_S: f64 = 0.020;
/// A level's rise, a share of the stated voltage.
pub const STEP_SHARE: f64 = 0.04;
pub const MOVE_MIN_COUNTS: f64 = 20.0;
pub const MOVE_SD: f64 = 12.0;
/// The coast rows the back-EMF is read over: the first 2 ms, before the
/// rotor has slowed.
pub const EMF_ROWS: usize = 40;
pub const EMF_MIN_COUNTS: f64 = 2.0;
/// The least time from one unwedge press to the next, s.
pub const UNWEDGE_GAP_S: f64 = 1.0;
const START_ROWS: usize = 10;
const BLOCK_ROWS: usize = 20;
const Q15: f64 = 32768.0;

/// What the ladder is built on, every number from a measurement, the
/// nameplate or the board.
#[derive(Clone, Debug, PartialEq)]
pub struct LadderCfg {
    pub rail_v: f64,
    /// Q15 duties: the bridge's shortest pulse, a level's rise, the
    /// ceiling.
    pub start_q15: i16,
    pub step_q15: i16,
    pub ceiling_q15: i16,
    /// The voltage the nameplate states its speed at, and that speed in
    /// counts/s.
    pub speed_v: f64,
    pub rated_cps: f64,
    /// Pot travel that counts as moving, counts.
    pub threshold: f64,
    pub emf_min: f64,
    /// The stored `drive_polarity`, in force through the pulses.
    pub polarity: bool,
    /// The R the ceiling and the drives are priced at, ohm.
    pub r_ohm: f64,
    /// The duty an unwedge press runs at; None when the train may not take
    /// one.
    pub unwedge_q15: Option<i16>,
}

impl LadderCfg {
    pub fn levels(&self) -> Vec<i16> {
        let mut out = Vec::new();
        let mut q = self.start_q15 as i32;
        while q < self.ceiling_q15 as i32 {
            out.push(q as i16);
            q += self.step_q15.max(1) as i32;
        }
        out.push(self.ceiling_q15);
        out
    }

    pub fn volts(&self, q15: i16) -> f64 {
        q15.unsigned_abs() as f64 / Q15 * self.rail_v
    }

    /// The free shaft's speed at `q15`, counts/s, as the nameplate rates
    /// it.
    pub fn speed_cps(&self, q15: i16) -> f64 {
        self.volts(q15) / self.speed_v * self.rated_cps
    }

    /// The worst-case travel of a pulse at `q15`: its volts over the
    /// stated voltage, times the rated speed, times the pulse and the
    /// host's turn, times the free margin.
    pub fn priced(&self, q15: i16) -> f64 {
        FREE_MARGIN * self.speed_cps(q15) * (PULSE_S + HOST_S)
    }

    /// An unwedge press's stopping distance: its free speed held over the
    /// whole coast the rows see.
    pub fn unwedge_stop(&self, q15: i16) -> f64 {
        FREE_MARGIN * self.speed_cps(q15) * COAST_S
    }

    /// The worst-case travel of an unwedge press: free at its duty for its
    /// time, then [`Self::unwedge_stop`].
    pub fn unwedge_priced(&self, q15: i16) -> f64 {
        FREE_MARGIN * self.speed_cps(q15) * UNWEDGE_MAX_S + self.unwedge_stop(q15)
    }

    /// The bridge's sign of a logical duty sign.
    pub fn wiring(&self, dir: i8) -> f64 {
        dir as f64 * if self.polarity { 1.0 } else { -1.0 }
    }
}

/// One pulse the ladder asks for.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Planned {
    pub n: usize,
    pub level: usize,
    pub repeat: bool,
    /// A press at the speed voltage to free a wedged train.
    pub unwedge: bool,
    /// The logical duty's sign.
    pub dir: i8,
    pub duty_q15: i16,
    pub volts: f64,
    /// The drive's time as priced, s.
    pub time_s: f64,
    pub priced_counts: f64,
}

impl Planned {
    pub fn said(&self) -> String {
        let what = if self.unwedge {
            "unwedge press".to_string()
        } else {
            format!(
                "level {}, {}",
                self.level,
                if self.repeat { "repeat" } else { "first" }
            )
        };
        format!(
            "pulse {} ({what}): {}{:.3} V ({:.2}% duty) for {} s",
            self.n,
            if self.dir > 0 { "+" } else { "-" },
            self.volts,
            100.0 * self.duty_q15.unsigned_abs() as f64 / Q15,
            if self.unwedge { UNWEDGE_MAX_S } else { PULSE_S }
        )
    }
}

/// What a pulse did, read off its rows.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Seen {
    /// The pot before the pulse, counts.
    pub start: f64,
    /// The pot's farthest move from the start over the pulse and its
    /// coast, signed, and where it ended, counts.
    pub peak: f64,
    pub end: f64,
    /// The coast's tap difference over the rest's, wiring frame, counts.
    pub emf: f64,
    /// The highest 1 ms mean of the drive's current over the rest zero, A.
    pub current_peak_a: f64,
}

/// What a pulse's rows are read by: the frame ripple the pot rows are
/// compensated by, the taps' difference and the current zero at rest.
#[derive(Clone, Debug, PartialEq)]
pub struct Comp {
    pub pattern: Option<Pattern>,
    pub pos_gain: f64,
    pub tap_offset: f64,
    pub zero_counts: f64,
    pub counts_per_a: f64,
}

/// A pulse's [`Seen`] from its drive rows and its coast rows; None when a
/// burst carries too few rows to read.
pub fn seen(drive: &[TelFrame], coast: &[TelFrame], c: &Comp) -> Option<Seen> {
    let pos = |rows: &[TelFrame]| -> Vec<f64> {
        let s = ripple::series(rows, "pos");
        match &c.pattern {
            Some(p) => ripple::compensate(&s, p, c.pos_gain),
            None => s,
        }
        .into_iter()
        .map(|(_, x)| x)
        .collect()
    };
    let d = pos(drive);
    let all: Vec<f64> = d.iter().copied().chain(pos(coast)).collect();
    if d.len() < START_ROWS || all.len() < BLOCK_ROWS {
        return None;
    }
    let start = mean(&d[..START_ROWS])?;
    let blocks: Vec<f64> = all.chunks(BLOCK_ROWS).filter_map(mean).collect();
    let peak = blocks
        .iter()
        .map(|b| b - start)
        .max_by(|a, b| a.abs().total_cmp(&b.abs()))?;
    let end = blocks.last()? - start;
    let taps: Vec<f64> = coast
        .iter()
        .take(EMF_ROWS)
        .filter_map(|f| Some(f.vmotor_a? as f64 - f.vmotor_b? as f64))
        .collect();
    let emf = mean(&taps)? - c.tap_offset;
    let amps: Vec<f64> = drive
        .iter()
        .filter_map(|f| f.current_raw.map(|v| v as f64 - c.zero_counts))
        .collect();
    let current_peak_a = amps
        .chunks(BLOCK_ROWS)
        .filter_map(mean)
        .map(f64::abs)
        .fold(0.0, f64::max)
        / c.counts_per_a;
    Some(Seen {
        start,
        peak,
        end,
        emf,
        current_peak_a,
    })
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Motion {
    /// The pot moved past the threshold, counts up or down.
    Up,
    Down,
    /// The pot stayed, the coast showed back-EMF of the drive's sign: the
    /// motor turned.
    Turned,
    /// Neither.
    Still,
}

impl Motion {
    fn moved(self) -> bool {
        matches!(self, Motion::Up | Motion::Down)
    }

    pub fn word(self) -> &'static str {
        match self {
            Motion::Up => "moved up",
            Motion::Down => "moved down",
            Motion::Turned => "the motor turned, the pot stayed",
            Motion::Still => "still",
        }
    }
}

/// What a +duty, -duty pair says: whether the stored sign is right, and
/// the logical duty that pressed a stop, if one did.
type Says = (bool, Option<i8>);

/// [`Says`] of a pair; None when opposite duties did not move opposite
/// ways.
fn implies(m: [Motion; 2]) -> Option<Says> {
    use Motion::*;
    Some(match m {
        [Up, Down] => (true, None),
        [Down, Up] => (false, None),
        [Up, Still] => (true, Some(-1)),
        [Still, Down] => (true, Some(1)),
        [Down, Still] => (false, Some(-1)),
        [Still, Up] => (false, Some(1)),
        _ => return None,
    })
}

/// One pulse driven and what it did.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pulse {
    pub planned: Planned,
    pub seen: Seen,
    pub motion: Motion,
    /// The coast's back-EMF has the drive's sign and clears the minimum.
    pub emf_agrees: bool,
}

impl Pulse {
    pub fn said(&self) -> String {
        let p = &self.planned;
        format!(
            "{}: peak travel {:+.1} counts, end {:+.1}, priced bound {:.0} counts ({:.0}% of it); current peak {:.3} A; coast back-EMF {:+.1} counts (drive's sign {}, at least {EMF_MIN_COUNTS}): {}",
            p.said(),
            self.seen.peak,
            self.seen.end,
            p.priced_counts,
            100.0 * self.seen.peak.abs() / p.priced_counts.max(f64::MIN_POSITIVE),
            self.seen.current_peak_a,
            self.seen.emf,
            if self.emf_agrees { "yes" } else { "no" },
            self.motion.word()
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Stop {
    Low,
    High,
}

impl Stop {
    pub fn word(self) -> &'static str {
        match self {
            Stop::Low => "low",
            Stop::High => "high",
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Refusal {
    Blocked,
    DeadPot,
    TooStiff,
    Disagree,
    OverTravel,
    NoEmf,
}

impl Refusal {
    /// The evidence contradicts itself or the pricing: the step aborts
    /// rather than refuses.
    pub fn aborts(self) -> bool {
        matches!(self, Refusal::Disagree | Refusal::OverTravel)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// The sign the pulses proved: whether the stored `drive_polarity` is
    /// right, and the stop one duty pressed when the start was at one.
    Proven {
        stored_right: bool,
        pressed: Option<Stop>,
        volts: f64,
    },
    Refused {
        kind: Refusal,
        why: String,
    },
}

pub enum Next {
    Pulse(Planned),
    Done(Outcome),
}

pub const HANDS: &str = "turn the horn to mid travel by hand, then start again";

/// The pulse ladder's decision, pure.
#[derive(Clone, Debug)]
pub struct Ladder {
    cfg: LadderCfg,
    levels: Vec<i16>,
    level: usize,
    pair: Vec<Pulse>,
    /// The first decisive pair and what it says.
    first: Option<([Motion; 2], Says)>,
    pulses: Vec<Pulse>,
    done: Option<Outcome>,
    path: Vec<String>,
    /// The press due next, by its duty's sign.
    unwedging: Option<i8>,
    unwedge_used: bool,
}

impl Ladder {
    pub fn new(cfg: LadderCfg) -> Self {
        let levels = cfg.levels();
        Ladder {
            cfg,
            levels,
            level: 0,
            pair: Vec::new(),
            first: None,
            pulses: Vec::new(),
            done: None,
            path: Vec::new(),
            unwedging: None,
            unwedge_used: false,
        }
    }

    pub fn cfg(&self) -> &LadderCfg {
        &self.cfg
    }

    pub fn pulses(&self) -> &[Pulse] {
        &self.pulses
    }

    /// The decisions taken, in order, a line each.
    pub fn path(&self) -> &[String] {
        &self.path
    }

    pub fn outcome(&self) -> Option<&Outcome> {
        self.done.as_ref()
    }

    /// The ladder found the shaft wedged and spent its one unwedge.
    pub fn wedged(&self) -> bool {
        self.unwedge_used
    }

    pub fn next(&self) -> Next {
        if let Some(o) = &self.done {
            return Next::Done(o.clone());
        }
        if let (Some(dir), Some(q)) = (self.unwedging, self.cfg.unwedge_q15) {
            return Next::Pulse(Planned {
                n: self.pulses.len() + 1,
                level: self.level,
                repeat: false,
                unwedge: true,
                dir,
                duty_q15: q * dir as i16,
                volts: self.cfg.volts(q),
                time_s: UNWEDGE_MAX_S,
                priced_counts: self.cfg.unwedge_priced(q),
            });
        }
        let q = self.levels[self.level];
        let dir: i8 = if self.pair.is_empty() { 1 } else { -1 };
        Next::Pulse(Planned {
            n: self.pulses.len() + 1,
            level: self.level,
            repeat: self.first.is_some(),
            unwedge: false,
            dir,
            duty_q15: q * dir as i16,
            volts: self.cfg.volts(q),
            time_s: PULSE_S + HOST_S,
            priced_counts: self.cfg.priced(q),
        })
    }

    fn refuse(&mut self, kind: Refusal, why: String) {
        self.path.push(format!("refused: {why}"));
        self.done = Some(Outcome::Refused { kind, why });
    }

    /// What the pulse [`Self::next`] planned did.
    pub fn feed(&mut self, seen: Seen) {
        let Next::Pulse(planned) = self.next() else {
            return;
        };
        let c = &self.cfg;
        let wiring = c.wiring(planned.dir);
        let emf_strong = seen.emf.abs() >= c.emf_min;
        let emf_agrees = emf_strong && seen.emf.signum() == wiring;
        let motion = if seen.peak.abs() >= c.threshold {
            if seen.peak > 0.0 {
                Motion::Up
            } else {
                Motion::Down
            }
        } else if emf_agrees {
            Motion::Turned
        } else {
            Motion::Still
        };
        let pulse = Pulse {
            planned,
            seen,
            motion,
            emf_agrees,
        };
        self.pulses.push(pulse);
        if seen.peak.abs() > planned.priced_counts {
            return self.refuse(
                Refusal::OverTravel,
                format!(
                    "pulse {} travelled {:.0} counts, over the {:.0} it was priced at: the shaft runs faster than the nameplate's speed allows",
                    planned.n,
                    seen.peak.abs(),
                    planned.priced_counts
                ),
            );
        }
        if motion.moved() && emf_strong && !emf_agrees {
            return self.refuse(
                Refusal::Disagree,
                format!(
                    "pulse {}: the pot moved {:+.0} counts and the coast's back-EMF has the other sign: the reading and the motor disagree",
                    planned.n, seen.peak
                ),
            );
        }
        if planned.unwedge {
            return self.after_unwedge(&planned, motion, seen.peak);
        }
        self.pair.push(pulse);
        if self.pair.len() < 2 {
            return;
        }
        let m = [self.pair[0].motion, self.pair[1].motion];
        let volts = self.pair[0].planned.volts;
        let last = self.level + 1 == self.levels.len();
        self.pair.clear();
        let decisive = m.iter().any(|x| x.moved()) && m.iter().all(|x| *x != Motion::Turned);
        let said = format!("+duty {}, -duty {}", m[0].word(), m[1].word());
        match self.first {
            None if decisive => match implies(m) {
                Some(i) => {
                    self.path.push(format!(
                        "{volts:.3} V: {said}: the pair is repeated at the same volts"
                    ));
                    self.first = Some((m, i));
                }
                None => self.refuse(
                    Refusal::Disagree,
                    format!("at {volts:.3} V {said}: opposite duties must move opposite ways"),
                ),
            },
            None if last => self.at_ceiling(m, volts),
            None => {
                self.path
                    .push(format!("{volts:.3} V: {said}: raised a level"));
                self.level += 1;
            }
            // A pair started at a stop moves off it and back: the repeat
            // agrees when it says the same sign, whichever duty pressed.
            Some((fm, fi)) => match implies(m).filter(|_| decisive) {
                Some(i) if i.0 == fi.0 => {
                    self.path.push(format!(
                        "{volts:.3} V repeated: {said}: two repeats agree on the sign"
                    ));
                    self.decide(fi.0, fi.1.or(i.1), volts);
                }
                _ => self.refuse(
                    Refusal::Disagree,
                    format!(
                        "two repeats at {volts:.3} V disagree: +duty {} then {}, -duty {} then {}",
                        fm[0].word(),
                        m[0].word(),
                        fm[1].word(),
                        m[1].word()
                    ),
                ),
            },
        }
    }

    fn after_unwedge(&mut self, planned: &Planned, motion: Motion, peak: f64) {
        let sign = if planned.dir > 0 { "+" } else { "-" };
        if motion.moved() {
            self.path.push(format!(
                "the {sign} press moved the shaft {peak:+.0} counts: it is free; the ladder starts again from the shortest pulse (a press proves no sign)"
            ));
            self.unwedging = None;
            self.level = 0;
            self.pair.clear();
            self.first = None;
        } else if planned.dir > 0 {
            self.path.push(format!(
                "the + press did not free it ({}): the - press next, at least {UNWEDGE_GAP_S} s on",
                motion.word()
            ));
            self.unwedging = Some(-1);
        } else {
            self.unwedging = None;
            self.refuse(
                Refusal::Blocked,
                format!(
                    "neither direction moves the shaft at the ceiling, nor after one press at the speed voltage each way ({}): a blocked shaft or a dead pot. Nothing more drives; {HANDS}",
                    motion.word()
                ),
            );
        }
    }

    fn at_ceiling(&mut self, m: [Motion; 2], volts: f64) {
        let ma = 1000.0 * volts / self.cfg.r_ohm;
        if m.contains(&Motion::Turned) {
            return self.refuse(
                Refusal::DeadPot,
                format!(
                    "at the ceiling's {volts:.3} V the motor turns (back-EMF in the coast) and the pot does not follow: a dead or slipped pot. Nothing more drives; {HANDS}; if it is refused again, check the pot's wiring"
                ),
            );
        }
        let creep = self
            .pulses
            .iter()
            .rev()
            .take(2)
            .map(|p| p.seen.peak.abs())
            .fold(0.0, f64::max);
        if creep >= self.cfg.threshold / 2.0 {
            return self.refuse(
                Refusal::TooStiff,
                format!(
                    "this servo needs about {ma:.0} mA or more to start moving (it crept {creep:.0} counts at the ceiling's {volts:.3} V), over what an end stop may take. Nothing more drives"
                ),
            );
        }
        if let Some(q) = self.cfg.unwedge_q15.filter(|_| !self.unwedge_used) {
            self.unwedge_used = true;
            self.unwedging = Some(1);
            self.path.push(format!(
                "{volts:.3} V, the ceiling: +duty {}, -duty {}: the shaft may be wedged; one press at {:.3} V of at most {UNWEDGE_MAX_S} s each way, + first, to free it",
                m[0].word(),
                m[1].word(),
                self.cfg.volts(q)
            ));
            return;
        }
        self.refuse(
            Refusal::Blocked,
            format!(
                "neither direction moves the shaft at the ceiling's {volts:.3} V and no coast shows back-EMF{}: a blocked shaft or a horn wedged at a stop. Nothing more drives; {HANDS}",
                if self.unwedge_used {
                    ", and the run's one unwedge is spent"
                } else {
                    ""
                }
            ),
        );
    }

    fn decide(&mut self, right: bool, pressed_dir: Option<i8>, volts: f64) {
        let tail = self.pulses.len().saturating_sub(4);
        if !self.pulses[tail..]
            .iter()
            .any(|p| p.motion.moved() && p.emf_agrees)
        {
            return self.refuse(
                Refusal::NoEmf,
                format!(
                    "at {volts:.3} V the pot moved but no coast showed back-EMF of the drive's sign: the reading moved, not shown to be the motor"
                ),
            );
        }
        let pressed = pressed_dir.map(|d| {
            let counts = if right { d } else { -d };
            if counts > 0 { Stop::High } else { Stop::Low }
        });
        if let Some(s) = pressed {
            self.path.push(format!(
                "the start was at the {} stop: the duty toward it pressed it (no motion, no back-EMF in its coast)",
                s.word()
            ));
        }
        self.path.push(if right {
            "the stored drive_polarity is right: +duty moves the counts up".into()
        } else {
            "the stored drive_polarity is wrong: +duty moves the counts down".into()
        });
        self.done = Some(Outcome::Proven {
            stored_right: right,
            pressed,
            volts,
        });
    }
}

/// The plan a pulse's bursts carry in their meta.
fn plan(p: &Planned, part: &str, rows: u16, duty_q15: i16) -> Value {
    json!({"rung": format!("pulse {} {part}", p.n), "row": Layout::Raw.name(), "rows": rows, "duty_q15": duty_q15,
           "level": p.level, "repeat": p.repeat, "unwedge": p.unwedge, "volts": p.volts,
           "time_s": p.time_s, "priced_counts": p.priced_counts})
}

fn rows_of(s: f64, tick_hz: f64) -> u16 {
    (s * tick_hz).round().clamp(1.0, u16::MAX as f64) as u16
}

/// The ladder on the servo: each pulse priced in `env` before its write,
/// driven, its rows captured to LAST, its coast captured, then fed. A
/// pulse whose bursts lost a row is driven once more, twice aborts; an
/// unwedge press is never driven twice, so a lost row in one aborts.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    env: &mut Envelope,
    ladder: &mut Ladder,
    comp: &Comp,
    tick_hz: f64,
    servo: &Map<String, Value>,
    taken: &mut Vec<Taken>,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    loop {
        interrupted(rig.stop)?;
        let p = match ladder.next() {
            Next::Done(_) => return Ok(()),
            Next::Pulse(p) => p,
        };
        if p.unwedge {
            env.facts.wedged = ladder.wedged();
        }
        let drive_rows = rows_of(p.time_s - HOST_S, tick_hz);
        let coast_rows = rows_of(COAST_S, tick_hz);
        let mut attempt = 0;
        let (drive, coast, on_s) = loop {
            attempt += 1;
            let start = rest::pos_now(rig.c, rig.id, rig.d).await?;
            let rung = Rung {
                name: format!("pulse {}", p.n),
                duty: p.duty_q15 as f64 / Q15,
                time_s: p.time_s,
                permit: p.unwedge,
                gap_s: if p.unwedge { UNWEDGE_GAP_S } else { COAST_S },
                start,
                speed_cps: ladder.cfg().speed_cps(p.duty_q15),
                stop_counts: if p.unwedge {
                    ladder.cfg().unwedge_stop(p.duty_q15)
                } else {
                    0.0
                },
                kind: if p.unwedge {
                    Kind::Unwedge
                } else {
                    Kind::Drive
                },
            };
            let price = env.admit(&rung)?;
            if p.unwedge {
                lines.push(format!("priced: {}", price.said(&rung)));
                write(rig, "stall_permit", &[1]).await?;
            }
            write(rig, "goal_duty", &p.duty_q15.to_le_bytes()).await?;
            let t_on = (rig.clock)();
            let (drive, lost_d) = take_once(
                rig,
                taken,
                Layout::Raw,
                drive_rows,
                p.duty_q15,
                &plan(&p, "drive", drive_rows, p.duty_q15),
                servo,
            )
            .await?;
            write(rig, "goal_duty", &[0, 0]).await?;
            let on_s = (rig.clock)() - t_on;
            if p.unwedge {
                write(rig, "stall_permit", &[0]).await?;
            }
            let (coast, lost_c) = take_once(
                rig,
                taken,
                Layout::Raw,
                coast_rows,
                0,
                &plan(&p, "coast", coast_rows, 0),
                servo,
            )
            .await?;
            let lost: Vec<String> = lost_d.into_iter().chain(lost_c).collect();
            if lost.is_empty() {
                break (drive, coast, on_s);
            }
            lines.push(format!(
                "pulse {}, attempt {attempt}: lost rows ({}), no evidence",
                p.n,
                lost.join(", ")
            ));
            if attempt == 2 || p.unwedge {
                return Err(Halt::Error(format!(
                    "pulse {}: lost rows ({}){}: the line is not clean enough; check the cable and its contacts",
                    p.n,
                    lost.join(", "),
                    if p.unwedge {
                        " in an unwedge press, which is never repeated"
                    } else {
                        " twice"
                    }
                )));
            }
            interrupted(rig.stop)?;
        };
        if on_s > p.time_s {
            return Err(Halt::Error(format!(
                "pulse {} ran {on_s:.3} s from its write to the zero write, over the {:.3} s it was priced at",
                p.n, p.time_s
            )));
        }
        let Some(s) = seen(&drive.rows, &coast.rows, comp) else {
            return Err(Halt::Error(format!(
                "pulse {}: the bursts carry too few rows to read",
                p.n
            )));
        };
        ladder.feed(s);
        if let Some(last) = ladder.pulses().last() {
            lines.push(format!(
                "{}; on for {on_s:.3} s by the host's clock",
                last.said()
            ));
        }
        if p.unwedge {
            keep_alive(rig, UNWEDGE_GAP_S - COAST_S).await?;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench servo's class on a 5 V rail: R 0.9 x 4.2 ohm, the ceiling
    /// at half the metal stop-hold ceiling, a 4 % rise a level.
    fn cfg(polarity: bool) -> LadderCfg {
        let rail_v = 5.0;
        let r_ohm = 0.9 * 4.2;
        let q = |v: f64| (v / rail_v * Q15).floor() as i16;
        LadderCfg {
            rail_v,
            start_q15: 738,
            step_q15: q(0.04 * 4.8),
            ceiling_q15: q(0.25 * r_ohm),
            speed_v: 4.8,
            rated_cps: 600.0 * 4096.0 / 184.33,
            threshold: 20.0,
            emf_min: EMF_MIN_COUNTS,
            polarity,
            r_ohm,
            unwedge_q15: Some(q(4.8)),
        }
    }

    /// The shaft at `pos` between stops at `lo` and `hi`: the motor's sign
    /// `wired` (+1: a +bridge raises the counts), breakaway at
    /// `breakaway_v`, free running at half the priced speed, its pot alive
    /// or dead, and `wedge` unwedge presses short of free.
    struct Shaft {
        pos: f64,
        lo: f64,
        hi: f64,
        wired: f64,
        breakaway_v: f64,
        pot: bool,
        wedge: u32,
    }

    impl Shaft {
        fn mid(wired: f64) -> Self {
            Shaft {
                pos: 2000.0,
                lo: 432.0,
                hi: 3626.0,
                wired,
                breakaway_v: 0.4,
                pot: true,
                wedge: 0,
            }
        }

        /// What `p` does to the shaft under the ladder's `c`.
        fn drive(&mut self, c: &LadderCfg, p: &Planned) -> Seen {
            let bridge = c.wiring(p.dir);
            let free = if p.unwedge {
                let held = self.wedge > 0;
                self.wedge = self.wedge.saturating_sub(1);
                !held
            } else {
                self.wedge == 0 && p.volts >= self.breakaway_v
            };
            let start = self.pos;
            let run = if free {
                0.5 * c.speed_cps(p.duty_q15) * p.time_s
            } else {
                0.0
            };
            let to = (start + bridge * self.wired * run).clamp(self.lo, self.hi);
            self.pos = to;
            let read = |x: f64| if self.pot { x } else { 2000.0 };
            Seen {
                start: read(start),
                peak: read(to) - read(start),
                end: read(to) - read(start),
                emf: if to != start { 40.0 * bridge } else { 0.0 },
                current_peak_a: p.volts / c.r_ohm,
            }
        }
    }

    /// The ladder run to its verdict on `shaft`.
    fn run(c: LadderCfg, shaft: &mut Shaft) -> Ladder {
        let mut l = Ladder::new(c.clone());
        for _ in 0..200 {
            match l.next() {
                Next::Done(_) => return l,
                Next::Pulse(p) => {
                    let s = shaft.drive(&c, &p);
                    l.feed(s);
                }
            }
        }
        panic!("the ladder never ends: {:?}", l.path());
    }

    fn proven(l: &Ladder) -> (bool, Option<Stop>) {
        match l.outcome() {
            Some(Outcome::Proven {
                stored_right,
                pressed,
                ..
            }) => (*stored_right, *pressed),
            o => panic!("{o:?}: {:?}", l.path()),
        }
    }

    fn refused(l: &Ladder) -> (Refusal, String) {
        match l.outcome() {
            Some(Outcome::Refused { kind, why }) => (*kind, why.clone()),
            o => panic!("{o:?}: {:?}", l.path()),
        }
    }

    /// Mid travel, at the low stop and at the high stop, the stored sign
    /// right and wrong, both wirings: the right sign every time. A start
    /// at a stop shows as a press when +duty drives toward it; when +duty
    /// drives away, -duty brings the shaft back and both move.
    #[test]
    fn the_sign_is_found_from_either_stored_value_and_three_start_positions() {
        let mut pressed_seen = 0;
        for wired in [1.0, -1.0] {
            for stored in [true, false] {
                for start in [2000.0, 432.0, 3626.0] {
                    let mut s = Shaft {
                        pos: start,
                        ..Shaft::mid(wired)
                    };
                    let l = run(cfg(stored), &mut s);
                    let right = (wired > 0.0) == stored;
                    // +duty's way in counts: up when the stored sign is right
                    let plus_up = right;
                    let pressed = match start {
                        432.0 if !plus_up => Some(Stop::Low),
                        3626.0 if plus_up => Some(Stop::High),
                        _ => None,
                    };
                    assert_eq!(
                        proven(&l),
                        (right, pressed),
                        "wired {wired}, stored {stored}, from {start}: {:?}",
                        l.path()
                    );
                    assert!(l.pulses().iter().all(|p| !p.planned.unwedge));
                    if pressed.is_some() {
                        pressed_seen += 1;
                        assert!(
                            l.path().iter().any(|p| p.contains("the start was at the")),
                            "{:?}",
                            l.path()
                        );
                    }
                }
            }
        }
        assert_eq!(pressed_seen, 4, "each stop pressed in half its cases");
    }

    #[test]
    fn no_rung_exceeds_its_priced_travel() {
        for (stored, start) in [(true, 2000.0), (false, 432.0), (true, 3626.0)] {
            let mut s = Shaft {
                pos: start,
                breakaway_v: 0.9,
                ..Shaft::mid(1.0)
            };
            let c = cfg(stored);
            let l = run(c.clone(), &mut s);
            assert!(matches!(l.outcome(), Some(Outcome::Proven { .. })));
            for p in l.pulses() {
                assert!(p.seen.peak.abs() <= p.planned.priced_counts, "{}", p.said());
                assert!(p.planned.volts <= c.volts(c.ceiling_q15) + 1e-12);
                let q = p.planned.duty_q15.unsigned_abs() as i16;
                assert_eq!(p.planned.priced_counts, c.priced(q));
            }
        }
        // a shaft faster than the nameplate allows aborts at its first move
        let mut l = Ladder::new(cfg(true));
        let Next::Pulse(p) = l.next() else {
            panic!("a pulse first")
        };
        l.feed(Seen {
            start: 2000.0,
            peak: p.priced_counts + 1.0,
            end: p.priced_counts,
            emf: 40.0,
            current_peak_a: 0.0,
        });
        let (kind, why) = refused(&l);
        assert!(kind == Refusal::OverTravel && kind.aborts(), "{why}");
        assert!(matches!(l.next(), Next::Done(_)));
    }

    #[test]
    fn a_blocked_shaft_is_refused_and_nothing_more_drives() {
        let mut c = cfg(true);
        c.unwedge_q15 = None;
        let mut s = Shaft {
            breakaway_v: 99.0,
            ..Shaft::mid(1.0)
        };
        let l = run(c.clone(), &mut s);
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::Blocked, "{why}");
        assert!(
            why.contains(HANDS) && why.contains("Nothing more drives"),
            "{why}"
        );
        assert!(matches!(l.next(), Next::Done(_)), "nothing more drives");
        let top = l
            .pulses()
            .iter()
            .map(|p| p.planned.volts)
            .fold(0.0, f64::max);
        assert!(
            (top - c.volts(c.ceiling_q15)).abs() < 1e-12,
            "up to the ceiling"
        );
        assert!(!kind.aborts());

        // with the unwedge: two presses that free nothing, then the same
        let l = run(
            cfg(true),
            &mut Shaft {
                breakaway_v: 99.0,
                wedge: 9,
                ..Shaft::mid(1.0)
            },
        );
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::Blocked, "{why}");
        assert!(
            why.contains("press at the speed voltage each way") && why.contains(HANDS),
            "{why}"
        );
        assert_eq!(l.pulses().iter().filter(|p| p.planned.unwedge).count(), 2);
        assert!(matches!(l.next(), Next::Done(_)));
    }

    #[test]
    fn a_dead_pot_is_refused() {
        let mut s = Shaft {
            pot: false,
            ..Shaft::mid(1.0)
        };
        let l = run(cfg(true), &mut s);
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::DeadPot, "{why}");
        assert!(why.contains("pot") && why.contains(HANDS), "{why}");
        assert!(
            l.pulses().iter().all(|p| !p.planned.unwedge),
            "a dead pot is not pressed free"
        );
        assert!(matches!(l.next(), Next::Done(_)));
    }

    #[test]
    fn a_stiff_shaft_over_the_ceiling_is_refused() {
        let c = cfg(true);
        let mut l = Ladder::new(c.clone());
        // it creeps 12 counts at every level, under the 20 that count as
        // moving, and never turns the motor enough to show back-EMF
        while let Next::Pulse(p) = l.next() {
            l.feed(Seen {
                start: 2000.0,
                peak: 12.0 * p.dir as f64,
                end: 12.0 * p.dir as f64,
                emf: 0.0,
                current_peak_a: 0.0,
            });
        }
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::TooStiff, "{why}");
        let ma = 1000.0 * c.volts(c.ceiling_q15) / c.r_ohm;
        assert!(why.contains(&format!("about {ma:.0} mA")), "{why}");
        assert!(why.contains("over what an end stop may take"), "{why}");
        assert!(l.pulses().iter().all(|p| !p.planned.unwedge));
    }

    #[test]
    fn two_repeats_that_disagree_abort() {
        let mut l = Ladder::new(cfg(true));
        let mut k = 0;
        while let Next::Pulse(p) = l.next() {
            // the first pair moves up then down, the repeat the other way
            let way = if k < 2 { 1.0 } else { -1.0 } * p.dir as f64;
            l.feed(Seen {
                start: 2000.0,
                peak: 30.0 * way,
                end: 30.0 * way,
                emf: 40.0 * p.dir as f64,
                current_peak_a: 0.0,
            });
            k += 1;
        }
        let (kind, why) = refused(&l);
        assert!(kind == Refusal::Disagree && kind.aborts(), "{why}");
        assert!(why.contains("two repeats"), "{why}");
        assert_eq!(k, 4);
    }

    /// A wedged metal train: the + press does not free it, the - press
    /// does, and the ladder then proves the sign from the shortest pulse.
    #[test]
    fn a_wedged_shaft_freed_by_the_second_press_is_then_proven() {
        let c = cfg(false);
        let mut s = Shaft {
            wedge: 1,
            ..Shaft::mid(-1.0)
        };
        let press = c.unwedge_q15.expect("metal") as u16;
        let l = run(c, &mut s);
        let presses: Vec<&Pulse> = l.pulses().iter().filter(|p| p.planned.unwedge).collect();
        assert_eq!(presses.len(), 2, "{:?}", l.path());
        assert_eq!((presses[0].planned.dir, presses[1].planned.dir), (1, -1));
        assert!(presses[0].motion == Motion::Still && presses[1].motion.moved());
        assert!(
            presses
                .iter()
                .all(|p| p.planned.duty_q15.unsigned_abs() == press && p.planned.time_s == 0.05)
        );
        assert!(
            l.path().iter().any(|p| p.contains("starts again")),
            "{:?}",
            l.path()
        );
        let after = l
            .pulses()
            .iter()
            .position(|p| p.planned.unwedge && p.planned.dir < 0)
            .expect("the - press");
        let next = &l.pulses()[after + 1].planned;
        assert!(!next.unwedge && next.level == 0 && !next.repeat, "{next:?}");
        assert!(proven(&l).0, "wired -1 and stored false: right");
    }

    /// One unwedge a run: wedged again after it, the ladder refuses.
    #[test]
    fn one_unwedge_a_run() {
        let mut l = Ladder::new(cfg(true));
        let mut presses = 0;
        while let Next::Pulse(p) = l.next() {
            presses += p.unwedge as usize;
            // the - press frees it a little, then nothing moves again
            let peak = if p.unwedge && p.dir < 0 { -30.0 } else { 0.0 };
            l.feed(Seen {
                start: 2000.0,
                peak,
                end: peak,
                emf: if peak != 0.0 { -40.0 } else { 0.0 },
                current_peak_a: 0.0,
            });
        }
        assert_eq!(presses, 2);
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::Blocked);
        assert!(why.contains("one unwedge is spent"), "{why}");
    }
}
