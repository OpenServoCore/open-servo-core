//! Step 2 part B, polarity on a free shaft: open-loop pulses of 150 ms,
//! each followed by 150 ms of coast, from the bridge's shortest pulse up
//! by 4 % of the stated voltage a level, a +duty and a -duty pulse at each
//! level, until the pot moves past the larger of 20 counts and 12 x its
//! rest sd; then the same pair once more. The ceiling is the volts whose
//! blocked current is half the gear's stop-hold ceiling at the pricing R,
//! so arriving at a stop is a light press, not a strike.
//!
//! A metal train that moves neither way at the ceiling may be wedged on a
//! stop: one press at the speed voltage of at most 0.03 s each way, the
//! stall permit held for it alone, frees it, and the ladder starts again
//! from the shortest pulse. The press is short so a stiff shaft it frees
//! at mid travel cannot reach a stop at speed. A press proves no sign; the
//! ladder does. One attempt a run.
//!
//! Every drive here is timed by the host: it runs from the duty write to
//! the zero write. Each runs under a short bus dead-man,
//! [`DRIVE_DEADMAN_S`], written just before the duty, the step's own
//! written back after the zero, so the servo cuts a drive the host stalls
//! in. Each drive is priced at its own time plus that dead-man.
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
use osc_servo_core::kernel::phases::loops::faults::BIT_DEADMAN;
use serde_json::{Map, Value, json};

use super::{Halt, Rig, Taken, faulted, interrupted, keep_alive, rest, take_read, write};
use crate::capture::ripple::{self, Pattern};
use crate::capture::rows::{Layout, TelFrame};
use crate::capture::tel::{Burst, StreamCfg};
use crate::envelope::{DRIVE_DEADMAN_S, Envelope, FREE_MARGIN, Kind, Rung, UNWEDGE_MAX_S};
use crate::fit::mean;

pub const PULSE_S: f64 = 0.150;
pub const COAST_S: f64 = 0.150;
/// A level's rise, a share of the stated voltage.
pub const STEP_SHARE: f64 = 0.04;
pub const MOVE_MIN_COUNTS: f64 = 20.0;
pub const MOVE_SD: f64 = 12.0;
/// The coast rows the back-EMF is read over: the coast burst's first 2 ms,
/// counted from its arm, which follows the zero write by the step's
/// dead-man written back (and a press's permit off) and one read, each a
/// USB round trip of about a millisecond.
pub const EMF_ROWS: usize = 40;
pub const EMF_MIN_COUNTS: f64 = 2.0;
/// A coast's back-EMF counts as the motor turning at this many times the
/// run's floor, the largest a level-0 pulse that left the pot still shows:
/// a still shaft's taps read a few counts of either sign.
pub const EMF_FLOOR_GAIN: f64 = 2.5;
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
    /// stated voltage, times the rated speed, times the pulse and its
    /// dead-man, times the free margin.
    pub fn priced(&self, q15: i16) -> f64 {
        FREE_MARGIN * self.speed_cps(q15) * (PULSE_S + DRIVE_DEADMAN_S)
    }

    /// An unwedge press's stopping distance: its free speed held over the
    /// whole coast the rows see.
    pub fn unwedge_stop(&self, q15: i16) -> f64 {
        FREE_MARGIN * self.speed_cps(q15) * COAST_S
    }

    /// The worst-case travel of an unwedge press: free at its duty for its
    /// time and its dead-man, then [`Self::unwedge_stop`].
    pub fn unwedge_priced(&self, q15: i16) -> f64 {
        FREE_MARGIN * self.speed_cps(q15) * (UNWEDGE_MAX_S + DRIVE_DEADMAN_S)
            + self.unwedge_stop(q15)
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
    /// The drive's time as priced, its dead-man in it, s.
    pub time_s: f64,
    pub priced_counts: f64,
}

impl Planned {
    /// The drive's own time, its rows', s.
    pub fn drive_s(&self) -> f64 {
        if self.unwedge { UNWEDGE_MAX_S } else { PULSE_S }
    }

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
            self.drive_s()
        )
    }
}

/// What a pulse did, read off its rows.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Seen {
    /// The pot over the drive's first rows, before the duty's ramp moves
    /// it, counts.
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
    // Not the pot read before the writes: that read sits a few counts off
    // the streamed rows, more the higher the pos, which would bias the
    // moving threshold by direction. The duty ramps over about 13 ms, so
    // these rows are before any motion.
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
/// the logical duty that stayed still, if one did.
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
    /// The coast's back-EMF has the drive's sign and clears the bound.
    pub emf_agrees: bool,
    /// The least back-EMF that counted as the motor turning, counts.
    pub emf_bound: f64,
}

impl Pulse {
    pub fn said(&self) -> String {
        let p = &self.planned;
        format!(
            "{}: peak travel {:+.1} counts, end {:+.1}, priced bound {:.0} counts ({:.0}% of it); current peak {:.3} A; coast back-EMF {:+.1} counts (drive's sign {}, at least {:.1}): {}",
            p.said(),
            self.seen.peak,
            self.seen.end,
            p.priced_counts,
            100.0 * self.seen.peak.abs() / p.priced_counts.max(f64::MIN_POSITIVE),
            self.seen.current_peak_a,
            self.seen.emf,
            if self.emf_agrees { "yes" } else { "no" },
            self.emf_bound,
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

/// The pulse pairs a proven sign rests on: a decisive pair and its repeat
/// at the same volts, agreeing.
pub const PROVING_PAIRS: u64 = 2;

#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// The sign the pulses proved: whether the stored `drive_polarity` is
    /// right, and the end a duty that stayed still pointed at. A stop
    /// there or the motor's static friction: the pulses do not tell.
    Proven {
        stored_right: bool,
        still_toward: Option<Stop>,
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

/// A drive's dead-man ran out: the servo latched it and cut the drive.
pub const STALLED: &str =
    "the computer stalled during a drive, and the servo's dead-man cut the drive itself";

/// A pot that read no number around pulse `n`.
fn unread(n: usize) -> String {
    format!(
        "pulse {n}: the pot reads no number: a dead or slipped pot. Nothing more drives; {HANDS}; if it is refused again, check the pot's wiring"
    )
}

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
    /// The largest |back-EMF| of a level-0 pulse that left the pot still.
    emf_floor: f64,
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
            emf_floor: 0.0,
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

    /// The least coast back-EMF the next pulse counts as the motor turning:
    /// the larger of the minimum and [`EMF_FLOOR_GAIN`] x the run's floor.
    pub fn emf_bound(&self) -> f64 {
        self.cfg.emf_min.max(EMF_FLOOR_GAIN * self.emf_floor)
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
                time_s: UNWEDGE_MAX_S + DRIVE_DEADMAN_S,
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
            time_s: PULSE_S + DRIVE_DEADMAN_S,
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
        if ![seen.start, seen.peak, seen.end]
            .iter()
            .all(|x| x.is_finite())
        {
            return self.refuse(Refusal::DeadPot, unread(planned.n));
        }
        let bound = self.emf_bound();
        let c = &self.cfg;
        let wiring = c.wiring(planned.dir);
        let emf_strong = seen.emf.abs() >= bound;
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
            emf_bound: bound,
        };
        self.pulses.push(pulse);
        if planned.level == 0 && !planned.unwedge && !motion.moved() {
            self.emf_floor = self.emf_floor.max(seen.emf.abs());
        }
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
            // A repeat at the breakaway may not break away again: that is
            // no evidence either way.
            Some(_) if !decisive && last => {
                self.first = None;
                self.at_ceiling(m, volts)
            }
            Some(_) if !decisive => {
                self.path.push(format!(
                    "{volts:.3} V repeated: {said}: the repeat did not move at these volts: raised a level"
                ));
                self.first = None;
                self.level += 1;
            }
            // The repeat agrees when it says the same sign, whichever duty
            // stayed still.
            Some((fm, fi)) => match implies(m) {
                Some(i) if i.0 == fi.0 => {
                    self.path.push(format!(
                        "{volts:.3} V repeated: {said}: two repeats agree on the sign"
                    ));
                    self.decide(fi.0, [fi.1, i.1], volts, last);
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

    /// `still`: the duty that stayed still in the first pair and in the
    /// repeat, if one did. Below the last level, evidence the back-EMF
    /// does not carry raises a level.
    fn decide(&mut self, right: bool, still: [Option<i8>; 2], volts: f64, last: bool) {
        let tail = self.pulses.len().saturating_sub(4);
        if !self.pulses[tail..]
            .iter()
            .any(|p| p.motion.moved() && p.emf_agrees)
        {
            if !last {
                self.path.push(format!(
                    "at {volts:.3} V the pot moved but no coast showed back-EMF of the drive's sign over the floor (at least {:.1} counts): raised a level",
                    self.emf_bound()
                ));
                self.first = None;
                self.level += 1;
                return;
            }
            return self.refuse(
                Refusal::NoEmf,
                format!(
                    "at {volts:.3} V the pot moved but no coast showed back-EMF of the drive's sign: the reading moved, not shown to be the motor"
                ),
            );
        }
        // A first pulse from rest can stick under the motor's static
        // friction at mid travel exactly as it does against a stop, and
        // the repeat then moves either way: the pulses name the end, no
        // stop is claimed.
        let still_toward = still[0].or(still[1]).map(|d| {
            let counts = if right { d } else { -d };
            if counts > 0 { Stop::High } else { Stop::Low }
        });
        if let Some(s) = still_toward {
            self.path.push(format!(
                "a duty toward the {} end stayed still at {volts:.3} V (no motion, no back-EMF in its coast): a stop there or the motor's static friction, the pulses do not tell which",
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
            still_toward,
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

/// What the ladder's drives run under on the servo: the burst's numbers,
/// read once a step, the step's own dead-man and a drive's, in ticks.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct OnServo {
    pub stream: StreamCfg,
    pub deadman_ticks: u16,
    pub drive_deadman_ticks: u16,
}

/// The pot read before pulse `n`'s duty went on, refused unless it is a
/// number: the envelope's travel check is read from it.
fn start_of(n: usize, pos: f64) -> Result<f64, Halt> {
    if pos.is_finite() {
        Ok(pos)
    } else {
        Err(Halt::Refused(unread(n)))
    }
}

/// One burst of a pulse read to LAST: a dead-man latched in it refuses
/// with [`STALLED`], any other fault aborts.
async fn take_drive<P: Pipe>(
    rig: &mut Rig<'_, P>,
    stream: &StreamCfg,
    taken: &mut Vec<Taken>,
    rows: u16,
    duty_q15: i16,
    plan: &Value,
    servo: &Map<String, Value>,
) -> Result<(Burst, Vec<String>), Halt> {
    let (b, faults) =
        take_read(rig, stream, taken, Layout::Raw, rows, duty_q15, plan, servo).await?;
    if faults & BIT_DEADMAN != 0 {
        return Err(Halt::Refused(STALLED.into()));
    }
    faulted(rig, taken, &b, faults).await?;
    let lost = b.lost();
    Ok((b, lost))
}

/// The ladder on the servo: each pulse priced in `env` before its write,
/// its start read, driven under the drive's dead-man, its rows captured to
/// LAST, the zero written and the step's dead-man put back, its coast
/// captured, then fed. A pulse whose bursts lost a row is driven once
/// more, twice aborts; an unwedge press is never driven twice, so a lost
/// row in one aborts.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    env: &mut Envelope,
    ladder: &mut Ladder,
    comp: &Comp,
    on: &OnServo,
    servo: &Map<String, Value>,
    taken: &mut Vec<Taken>,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    let tick_hz = on.stream.tick_hz as f64;
    loop {
        interrupted(rig.stop)?;
        let p = match ladder.next() {
            Next::Done(_) => return Ok(()),
            Next::Pulse(p) => p,
        };
        if p.unwedge {
            env.facts.wedged = ladder.wedged();
        }
        let drive_rows = rows_of(p.drive_s(), tick_hz);
        let coast_rows = rows_of(COAST_S, tick_hz);
        let mut attempt = 0;
        let (drive, coast, on_s) = loop {
            attempt += 1;
            let start = start_of(p.n, rest::pos_now(rig.c, rig.id, rig.d).await?)?;
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
            write(
                rig,
                "bus_deadman_ticks",
                &on.drive_deadman_ticks.to_le_bytes(),
            )
            .await?;
            write(rig, "goal_duty", &p.duty_q15.to_le_bytes()).await?;
            let t_on = (rig.clock)();
            let (drive, lost_d) = take_drive(
                rig,
                &on.stream,
                taken,
                drive_rows,
                p.duty_q15,
                &plan(&p, "drive", drive_rows, p.duty_q15),
                servo,
            )
            .await?;
            write(rig, "goal_duty", &[0, 0]).await?;
            let on_s = (rig.clock)() - t_on;
            write(rig, "bus_deadman_ticks", &on.deadman_ticks.to_le_bytes()).await?;
            if p.unwedge {
                write(rig, "stall_permit", &[0]).await?;
            }
            let (coast, lost_c) = take_drive(
                rig,
                &on.stream,
                taken,
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
                still_toward,
                ..
            }) => (*stored_right, *still_toward),
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
    /// at a stop leaves +duty still when it drives toward the stop; when
    /// +duty drives away, -duty brings the shaft back and both move. The
    /// sign rests on [`PROVING_PAIRS`] pairs at the volts it is proven at.
    #[test]
    fn the_sign_is_found_from_either_stored_value_and_three_start_positions() {
        let mut still_seen = 0;
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
                    let still = match start {
                        432.0 if !plus_up => Some(Stop::Low),
                        3626.0 if plus_up => Some(Stop::High),
                        _ => None,
                    };
                    assert_eq!(
                        proven(&l),
                        (right, still),
                        "wired {wired}, stored {stored}, from {start}: {:?}",
                        l.path()
                    );
                    assert!(l.pulses().iter().all(|p| !p.planned.unwedge));
                    let Some(Outcome::Proven { volts, .. }) = l.outcome() else {
                        unreachable!()
                    };
                    let pairs = l
                        .pulses()
                        .iter()
                        .filter(|p| p.planned.volts == *volts)
                        .count()
                        / 2;
                    assert_eq!(pairs as u64, PROVING_PAIRS, "{:?}", l.path());
                    if still.is_some() {
                        still_seen += 1;
                        assert!(
                            l.path().iter().any(|p| p.contains("stayed still")),
                            "{:?}",
                            l.path()
                        );
                    }
                }
            }
        }
        assert_eq!(still_seen, 4, "a duty still at each stop in half its cases");
    }

    /// The ladder on `c` fed `pulses` in order, each its peak and its
    /// coast's back-EMF.
    fn fed_seen(c: LadderCfg, pulses: &[(f64, f64)]) -> Ladder {
        let mut l = Ladder::new(c);
        for &(peak, emf) in pulses {
            let Next::Pulse(_) = l.next() else {
                panic!("done early: {:?}", l.path())
            };
            l.feed(Seen {
                start: 2000.0,
                peak,
                end: peak,
                emf,
                current_peak_a: 0.0,
            });
        }
        l
    }

    /// The ladder fed a first pair and its repeat at the shortest pulse,
    /// each pulse doing what its motion says.
    fn fed(pairs: [[Motion; 2]; 2]) -> Ladder {
        let pulses: Vec<(f64, f64)> = pairs
            .into_iter()
            .flatten()
            .zip([1.0, -1.0].into_iter().cycle())
            .map(|(m, dir)| match m {
                Motion::Up => (30.0, 40.0 * dir),
                Motion::Down => (-30.0, 40.0 * dir),
                _ => (0.0, 0.0),
            })
            .collect();
        fed_seen(cfg(true), &pulses)
    }

    /// A start at mid travel with a pulse at the motor's breakaway leaves
    /// a duty still exactly as a stop does, whatever the repeat does: the
    /// sign stands, the end is named, no stop is claimed.
    #[test]
    fn a_still_duty_names_its_end_and_claims_no_stop() {
        use Motion::*;
        for pairs in [
            [[Still, Up], [Down, Still]],
            [[Still, Up], [Down, Up]],
            [[Still, Up], [Still, Up]],
        ] {
            let l = fed(pairs);
            assert_eq!(proven(&l), (false, Some(Stop::Low)), "{pairs:?}");
            assert!(
                l.path().iter().any(|p| p.contains("do not tell which")),
                "{:?}",
                l.path()
            );
            assert!(
                !l.path().iter().any(|p| p.contains("the start was at the")),
                "{:?}",
                l.path()
            );
        }
        let l = fed([[Down, Up], [Down, Still]]);
        assert_eq!(proven(&l), (false, Some(Stop::High)));
        let l = fed([[Up, Down], [Up, Down]]);
        assert_eq!(proven(&l), (true, None));
    }

    /// Run 4's pulses on the bench, the stored sign wrong: a move at the
    /// breakaway whose back-EMF of the other sign is inside the still
    /// pulses' noise is no disagreement; a repeat with weak back-EMF raises
    /// a level, where solid moves prove the sign.
    #[test]
    fn weak_back_emf_of_the_other_sign_raises_a_level() {
        let l = fed_seen(
            cfg(false),
            &[
                (-1.0, -0.1),
                (-1.0, -2.5),
                (-1.8, 1.0),
                (22.6, -3.9),
                (-1.5, 1.2),
                (24.0, 3.0),
                (-95.0, -27.6),
                (80.0, 40.0),
                (-94.0, -27.6),
                (84.0, 40.0),
            ],
        );
        assert_eq!(proven(&l), (false, None), "{:?}", l.path());
        assert_eq!(l.pulses().len(), 10);
        assert_eq!(l.emf_bound(), 2.5 * 2.5);
        assert_eq!(l.pulses()[3].motion, Motion::Up);
        assert!(!l.pulses()[3].emf_agrees);
        let at = |what: &str| l.path().iter().position(|p| p.contains(what));
        let raised = at("over the floor (at least 6.2 counts): raised a level").expect("raised");
        assert!(at("two repeats agree").expect("agreed") < raised);
        assert!(l.pulses()[6..].iter().all(|p| p.planned.level == 2));
        assert!(
            !l.path().iter().any(|p| p.contains("disagree")),
            "{:?}",
            l.path()
        );
    }

    /// Run 5's pulses: two pairs that agree, their moves' back-EMF under
    /// the floor's bound: raised a level, proven at the next. At the last
    /// level the same evidence refuses.
    #[test]
    fn moves_without_back_emf_raise_a_level_below_the_last() {
        let weak = [
            (-6.4, -2.7),
            (-5.6, -0.1),
            (-9.0, -3.9),
            (-24.1, 0.8),
            (12.1, -3.4),
            (-24.3, 0.5),
        ];
        let solid = [
            (94.0, 22.3),
            (-107.1, -33.8),
            (103.8, 23.8),
            (-112.0, -33.6),
        ];
        let all: Vec<(f64, f64)> = weak.iter().chain(&solid).copied().collect();
        let l = fed_seen(cfg(true), &all);
        assert_eq!(proven(&l), (true, None), "{:?}", l.path());
        assert!(
            l.path()
                .iter()
                .any(|p| p.contains("0.305 V the pot moved") && p.ends_with("raised a level")),
            "{:?}",
            l.path()
        );
        assert!(
            l.pulses()[6..]
                .iter()
                .all(|p| p.planned.level == 2 && p.emf_agrees)
        );

        let mut c = cfg(true);
        c.ceiling_q15 = c.start_q15 + c.step_q15;
        assert_eq!(c.levels().len(), 2);
        let l = fed_seen(c, &weak);
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::NoEmf, "{why}");
        assert!(why.contains("no coast showed back-EMF"), "{why}");
        assert!(!kind.aborts());
    }

    /// A move whose back-EMF has the other sign well over the bound is a
    /// real disagreement.
    #[test]
    fn a_strong_back_emf_of_the_other_sign_aborts() {
        let l = fed_seen(cfg(true), &[(-1.0, 2.8), (-1.0, -2.8), (40.0, -30.0)]);
        let (kind, why) = refused(&l);
        assert!(kind == Refusal::Disagree && kind.aborts(), "{why}");
        assert!(
            why.contains("pulse 3") && why.contains("other sign"),
            "{why}"
        );
    }

    /// The floor is read from the level-0 pulses that left the pot still,
    /// never from a move or a higher level; with none, the bound is the
    /// minimum.
    #[test]
    fn the_back_emf_bound_comes_from_the_still_level_0_pulses() {
        let l = fed_seen(cfg(true), &[(-1.0, 2.8), (-1.0, -2.8)]);
        assert_eq!(l.emf_bound(), 2.5 * 2.8);
        let next = fed_seen(cfg(true), &[(-1.0, 2.8), (-1.0, -2.8), (-1.0, 1.0)]);
        assert!(
            next.pulses()[2].said().contains("at least 7.0"),
            "{}",
            next.pulses()[2].said()
        );
        assert_eq!(next.pulses()[0].emf_bound, EMF_MIN_COUNTS);

        let l = fed_seen(cfg(true), &[(30.0, 40.0), (-1.0, 2.0)]);
        assert_eq!(l.emf_bound(), 5.0, "the move at level 0 is not noise");
        let l = fed_seen(cfg(true), &[(30.0, 40.0), (-30.0, -40.0)]);
        assert_eq!(l.emf_bound(), EMF_MIN_COUNTS);
        assert_eq!(Ladder::new(cfg(true)).emf_bound(), EMF_MIN_COUNTS);
        let l = fed_seen(
            cfg(true),
            &[(-1.0, 0.4), (-1.0, 0.0), (-1.0, 3.9), (-1.0, 3.3)],
        );
        assert_eq!(l.pulses()[3].planned.level, 1);
        assert_eq!(l.emf_bound(), EMF_MIN_COUNTS, "level 1 is not the floor");
    }

    /// A pot still at every level and back-EMF of the drive's sign far
    /// over the bound at the ceiling: a dead pot.
    #[test]
    fn a_dead_pot_under_the_floor_is_refused() {
        let c = cfg(true);
        let n = c.levels().len();
        let mut pulses = vec![(-1.0, 2.8), (-1.0, -2.8)];
        pulses.extend(std::iter::repeat_n((-1.0, 1.0), 2 * (n - 2)));
        pulses.extend([(-1.0, 30.0), (-1.0, -30.0)]);
        let l = fed_seen(c, &pulses);
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::DeadPot, "{why}");
        let top = &l.pulses()[l.pulses().len() - 2..];
        assert!(
            top.iter()
                .all(|p| p.planned.level == n - 1 && p.motion == Motion::Turned)
        );
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

    /// The bench's pulses at the motor's breakaway: the first pair moves
    /// up and stays still, the repeat stays still both ways. That is no
    /// disagreement: raised a level, where solid moves prove the sign.
    #[test]
    fn a_repeat_that_did_not_move_raises_a_level() {
        let bench = [
            (0.5, -2.4),
            (0.8, -0.2),
            (30.3, 8.9),
            (-12.8, 0.6),
            (10.4, -3.4),
            (-6.0, 1.7),
        ];
        let solid = [(92.0, 30.0), (-95.0, -30.0), (90.0, 30.0), (-94.0, -30.0)];
        let all: Vec<(f64, f64)> = bench.iter().chain(&solid).copied().collect();
        let l = fed_seen(cfg(true), &all);
        assert_eq!(proven(&l), (true, None), "{:?}", l.path());
        let m: Vec<Motion> = l.pulses()[2..6].iter().map(|p| p.motion).collect();
        assert_eq!(m, [Motion::Up, Motion::Still, Motion::Still, Motion::Still]);
        assert!(
            l.path()
                .iter()
                .any(|p| p.contains("V repeated: +duty still, -duty still")
                    && p.ends_with("the repeat did not move at these volts: raised a level")),
            "{:?}",
            l.path()
        );
        assert!(l.pulses()[6..].iter().all(|p| p.planned.level == 2));
        assert!(!l.pulses()[6].planned.repeat, "a new first pair");
        assert!(
            !l.path().iter().any(|p| p.contains("disagree")),
            "{:?}",
            l.path()
        );
    }

    /// A decisive repeat of the other sign, or one whose duties moved the
    /// same way, still aborts.
    #[test]
    fn a_decisive_repeat_of_the_other_sign_aborts() {
        use Motion::*;
        for pairs in [[[Up, Still], [Down, Still]], [[Up, Still], [Up, Up]]] {
            let l = fed(pairs);
            let (kind, why) = refused(&l);
            assert!(
                kind == Refusal::Disagree && kind.aborts(),
                "{pairs:?}: {why}"
            );
            assert!(why.contains("two repeats"), "{why}");
        }
    }

    /// The same repeat at the last level goes the ceiling's way: a press to
    /// free a wedged train, then the blocked refusal when none is left.
    #[test]
    fn a_repeat_that_did_not_move_at_the_last_level_takes_the_ceiling() {
        let mut c = cfg(true);
        c.ceiling_q15 = c.start_q15 + c.step_q15;
        assert_eq!(c.levels().len(), 2);
        let pulses = [
            (0.5, -2.4),
            (0.8, -0.2),
            (30.3, 8.9),
            (-1.0, 0.6),
            (1.0, -3.4),
            (-1.0, 1.7),
        ];
        let l = fed_seen(c.clone(), &pulses);
        assert!(l.outcome().is_none(), "{:?}", l.path());
        assert!(
            l.path()
                .last()
                .is_some_and(|p| p.contains("the ceiling") && p.contains("may be wedged")),
            "{:?}",
            l.path()
        );
        let Next::Pulse(p) = l.next() else {
            panic!("a press next: {:?}", l.path())
        };
        assert!(p.unwedge && p.dir > 0);

        c.unwedge_q15 = None;
        let l = fed_seen(c, &pulses);
        let (kind, why) = refused(&l);
        assert_eq!(kind, Refusal::Blocked, "{why}");
        assert!(!l.path().iter().any(|p| p.contains("disagree")));
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
                .all(|p| p.planned.duty_q15.unsigned_abs() == press
                    && p.planned.time_s == UNWEDGE_MAX_S + DRIVE_DEADMAN_S)
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

    /// The ladder's drives on the client's simulated servo, which has a
    /// bus and a table but no motor.
    mod on_servo {
        use std::cell::RefCell;
        use std::rc::Rc;
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::time::Duration;

        use futures_lite::future::block_on;
        use osc_client::descriptor::Descriptor;
        use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
        use osc_client::pipe::{Pipe, PipeError};
        use osc_client::{BaudRate, Client, Id, Inst, Opcode};

        use super::*;
        use crate::envelope::{Facts, HeatModel, Resistance};
        use crate::guard::{self, Ledger};
        use crate::inputs::ThermalInputs;
        use crate::nameplate::Nameplate;
        use crate::sink::MemSink;
        use crate::state::field;
        use crate::{Board, steps};

        const OSC_EXCHANGE: u8 = 0x20;
        const ID: Id = Id::new(1);
        const LONG: u16 = 10_000;
        const SHORT: u16 = 1_000;

        #[derive(Clone, Debug, PartialEq, Eq)]
        enum Ev {
            Read(u16),
            Write(u16, Vec<u8>),
        }

        type Log = Rc<RefCell<Vec<Ev>>>;
        /// A hook on every write, its address and bytes: it may reach into
        /// the rig.
        type OnWrite = Box<dyn FnMut(u16, &[u8], &mut FakePipe)>;

        /// The fake adapter, every read and write logged.
        struct Spy {
            inner: FakePipe,
            log: Log,
            on_write: OnWrite,
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
                {
                    let addr = u16::from_le_bytes([payload[0], payload[1]]);
                    match Inst(*inst).opcode() {
                        Some(Opcode::Read) => self.log.borrow_mut().push(Ev::Read(addr)),
                        Some(Opcode::Write) => {
                            self.log
                                .borrow_mut()
                                .push(Ev::Write(addr, payload[2..].to_vec()));
                            (self.on_write)(addr, &payload[2..], &mut self.inner);
                        }
                        _ => {}
                    }
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

        /// The shaft at rest at 2041 counts, the zero at 1206, the taps at
        /// their bias.
        fn still() -> Vec<TelSample> {
            (0..4096u32)
                .map(|i| TelSample {
                    tick: TelemetryTick {
                        _scan: [0; 9],
                        pos: 2041,
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

        /// The servo with torque on in OpenLoop under the step's dead-man,
        /// as part B finds it, every write from here on logged.
        fn servo(d: &Descriptor, on_write: OnWrite) -> (Client<Spy>, Log) {
            let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
            inner.seed_calibrated(0);
            inner.set_track(0, still());
            let log = Log::default();
            let mut c = block_on(Client::connect(Spy {
                inner,
                log: log.clone(),
                on_write: Box::new(|_, _, _| {}),
            }))
            .expect("connect");
            let open_loop = steps::mode_value(d, "OpenLoop").expect("OpenLoop");
            for (name, bytes) in [
                ("mode", vec![open_loop]),
                ("goal_duty", vec![0, 0]),
                ("bus_deadman_ticks", LONG.to_le_bytes().to_vec()),
                ("torque_enable", vec![1]),
            ] {
                block_on(c.write(ID, addr(d, name), &bytes)).expect(name);
            }
            c.pipe_mut().on_write = on_write;
            log.borrow_mut().clear();
            (c, log)
        }

        /// The bench servo's class, priced at the ladder's R, the stops not
        /// found, the sign not proven.
        fn envelope(c: &LadderCfg) -> Envelope {
            let np: Nameplate =
                serde_json::from_str(include_str!("../../testdata/mg90-d/nameplate.json"))
                    .expect("nameplate");
            let board: Board = steps::testkit::board();
            let thermal: ThermalInputs = crate::steps::testkit::thermal();
            let mut f = Facts::new(&np, c.rail_v);
            f.r = Some(Resistance {
                ohm: c.r_ohm,
                source: "the pricing R".into(),
            });
            let heat = HeatModel::new(&board, &thermal, c.r_ohm, 1108.5, 20_000.0);
            Envelope::new(f, heat)
        }

        fn comp() -> Comp {
            Comp {
                pattern: None,
                pos_gain: 0.0,
                tap_offset: 1.0,
                zero_counts: 1206.0,
                counts_per_a: 1108.5,
            }
        }

        /// What `body` did with `ladder` on `c`: its end, the bursts it
        /// took and its lines. The interrupt flag is set at the step's
        /// dead-man written back, so it stops after one drive.
        fn drive(
            c: &mut Client<Spy>,
            d: &Descriptor,
            ladder: &mut Ladder,
            stop: &AtomicBool,
        ) -> (Result<(), Halt>, Vec<Taken>, Vec<String>) {
            let stream = block_on(StreamCfg::read(c, ID, d)).expect("stream cfg");
            let on = OnServo {
                stream,
                deadman_ticks: LONG,
                drive_deadman_ticks: SHORT,
            };
            let mut env = envelope(ladder.cfg());
            let (mut taken, mut lines) = (Vec::new(), Vec::new());
            let clock = || 0.0;
            let mut sink = MemSink::default();
            let mut rig = Rig {
                c,
                id: ID,
                d,
                sink: &mut sink,
                stop,
                clock: &clock,
                tell: &crate::steps::untold,
            };
            let ran = block_on(body(
                &mut rig,
                &mut env,
                ladder,
                &comp(),
                &on,
                &serde_json::Map::new(),
                &mut taken,
                &mut lines,
            ));
            (ran, taken, lines)
        }

        /// Interrupt after the first drive: the flag is set when the
        /// step's dead-man goes back.
        fn after_one(d: &Descriptor, stop: &Rc<AtomicBool>) -> OnWrite {
            let (dm, set) = (addr(d, "bus_deadman_ticks"), stop.clone());
            Box::new(move |a, b, _| {
                if a == dm && b == LONG.to_le_bytes() {
                    set.store(true, Ordering::SeqCst);
                }
            })
        }

        /// The writes to the drive's fields, in order.
        fn drive_writes(log: &Log, d: &Descriptor) -> Vec<(&'static str, Vec<u8>)> {
            let names = ["bus_deadman_ticks", "goal_duty", "stall_permit"];
            log.borrow()
                .iter()
                .filter_map(|e| match e {
                    Ev::Write(a, b) => names
                        .iter()
                        .find(|n| addr(d, n) == *a)
                        .map(|n| (*n, b.clone())),
                    _ => None,
                })
                .collect()
        }

        fn time_s(t: &Taken) -> f64 {
            t.meta["plan"]["time_s"].as_f64().expect("time_s")
        }

        /// A ladder pulse: its start read before any write, then the
        /// drive's dead-man, the duty, its rows, the zero and the step's
        /// dead-man back, then the coast. Inside the drive the host reads
        /// only what the burst cannot know before it: no burst constant.
        #[test]
        fn a_pulse_runs_under_the_drives_dead_man() {
            let d = descriptor();
            let stop = Rc::new(AtomicBool::new(false));
            let (mut c, log) = servo(&d, after_one(&d, &stop));
            let mut ladder = Ladder::new(cfg(true));
            let Next::Pulse(p) = ladder.next() else {
                panic!("a pulse first")
            };
            let (ran, taken, lines) = drive(&mut c, &d, &mut ladder, &stop);
            assert_eq!(ran, Err(Halt::Interrupted), "{lines:?}");
            assert_eq!(
                drive_writes(&log, &d),
                vec![
                    ("bus_deadman_ticks", SHORT.to_le_bytes().to_vec()),
                    ("goal_duty", p.duty_q15.to_le_bytes().to_vec()),
                    ("goal_duty", vec![0, 0]),
                    ("bus_deadman_ticks", LONG.to_le_bytes().to_vec()),
                ]
            );
            let ev = log.borrow();
            let at = |want: &Ev| ev.iter().position(|e| e == want).expect("logged");
            let short = at(&Ev::Write(
                addr(&d, "bus_deadman_ticks"),
                SHORT.to_le_bytes().to_vec(),
            ));
            let zero = at(&Ev::Write(addr(&d, "goal_duty"), vec![0, 0]));
            let pos = addr(&d, "pos");
            assert!(
                ev[..short].iter().filter(|e| **e == Ev::Read(pos)).count() >= 1,
                "the start is read before the dead-man and the duty: {ev:?}"
            );
            let fixed = [addr(&d, "i_window_min_ticks"), addr(&d, "tick_hz")];
            assert!(
                !ev[short..]
                    .iter()
                    .any(|e| matches!(e, Ev::Read(a) if fixed.contains(a))),
                "{ev:?}"
            );
            assert!(short < zero);
            drop(ev);

            assert_eq!(taken.len(), 2, "the drive and its coast");
            assert_eq!(taken[0].burst.rows.len(), 3000, "PULSE_S of rows");
            assert_eq!(taken[1].burst.rows.len(), 3000, "COAST_S of rows");
            assert_eq!(time_s(&taken[0]), PULSE_S + DRIVE_DEADMAN_S);
            let pulse = ladder.pulses()[0];
            assert_eq!(pulse.planned.time_s, PULSE_S + DRIVE_DEADMAN_S);
            assert_eq!(pulse.seen.start, 2041.0, "the drive's first rows");
            assert_eq!(pulse.seen.peak, 0.0);
        }

        /// The ladder at its unwedge press: the shaft did not move at any
        /// level.
        fn wedged() -> Ladder {
            let mut l = Ladder::new(cfg(true));
            while let Next::Pulse(p) = l.next() {
                if p.unwedge {
                    return l;
                }
                l.feed(Seen {
                    start: 2000.0,
                    peak: 0.0,
                    end: 0.0,
                    emf: 0.0,
                    current_peak_a: 0.0,
                });
            }
            panic!("no press: {:?}", l.path())
        }

        /// The unwedge press: the permit, the drive's dead-man, the duty,
        /// UNWEDGE_MAX_S of rows, the zero, the step's dead-man back, the
        /// permit off; priced at the press and its dead-man.
        #[test]
        fn the_unwedge_press_runs_under_the_drives_dead_man() {
            let d = descriptor();
            let stop = Rc::new(AtomicBool::new(false));
            let (mut c, log) = servo(&d, after_one(&d, &stop));
            let mut ladder = wedged();
            let Next::Pulse(p) = ladder.next() else {
                panic!("the press")
            };
            let (ran, taken, lines) = drive(&mut c, &d, &mut ladder, &stop);
            assert_eq!(ran, Err(Halt::Interrupted), "{lines:?}");
            assert_eq!(
                drive_writes(&log, &d),
                vec![
                    ("stall_permit", vec![1]),
                    ("bus_deadman_ticks", SHORT.to_le_bytes().to_vec()),
                    ("goal_duty", p.duty_q15.to_le_bytes().to_vec()),
                    ("goal_duty", vec![0, 0]),
                    ("bus_deadman_ticks", LONG.to_le_bytes().to_vec()),
                    ("stall_permit", vec![0]),
                ]
            );
            assert_eq!(taken[0].burst.rows.len(), 600, "UNWEDGE_MAX_S of rows");
            let priced = UNWEDGE_MAX_S + DRIVE_DEADMAN_S;
            assert_eq!(time_s(&taken[0]), priced);
            assert!(
                lines
                    .iter()
                    .any(|l| l.starts_with("priced: pulse") && l.contains("for 0.08 s")),
                "the envelope priced the press and its dead-man: {lines:?}"
            );
            let c = ladder.cfg();
            let q = p.duty_q15.unsigned_abs() as i16;
            assert_eq!(
                p.priced_counts,
                FREE_MARGIN * c.speed_cps(q) * priced + c.unwedge_stop(q)
            );
        }

        /// The dead-man latched during a pulse, seen at the fault read
        /// after either burst: the step refuses with the stall sentence,
        /// and the way out ends torque off.
        #[test]
        fn a_dead_man_latched_in_a_pulse_refuses_and_ends_torque_off() {
            let d = descriptor();
            let duty = addr(&d, "goal_duty");
            for (at_zero, writes) in [(false, 2), (true, 4)] {
                let latch: OnWrite = Box::new(move |a, b, pipe| {
                    if a == duty && (b == [0, 0]) == at_zero {
                        pipe.sim_mut()
                            .servo_table_mut(0, |t| t.telemetry.common.fault_flags = BIT_DEADMAN);
                    }
                });
                let (mut c, log) = servo(&d, latch);
                let mut ladder = Ladder::new(cfg(true));
                let stop = AtomicBool::new(false);
                let (ran, taken, lines) = drive(&mut c, &d, &mut ladder, &stop);
                assert_eq!(ran, Err(Halt::Refused(STALLED.into())), "{lines:?}");
                assert_eq!(drive_writes(&log, &d).len(), writes, "at_zero {at_zero}");
                assert_eq!(taken.len(), if at_zero { 2 } else { 1 });
                assert!(ladder.pulses().is_empty(), "nothing is fed");

                let ledger = Ledger {
                    touched: true,
                    ..Ledger::default()
                };
                let exit = block_on(guard::exit(&mut c, ID, &d, &ledger));
                assert!(exit.torque_off && !exit.rebooted, "{exit}");
                let read = |c: &mut Client<Spy>, name: &str, n: u16| {
                    block_on(c.read(ID, addr(&d, name), n)).expect(name)
                };
                assert_eq!(read(&mut c, "torque_enable", 1), [0]);
                assert_eq!(read(&mut c, "goal_duty", 2), [0, 0]);
                assert_eq!(read(&mut c, "bus_deadman_ticks", 2), [0, 0]);
            }
        }
    }

    /// The ladder's pulses and its press are priced at their drive time
    /// and the drive's dead-man, as time and as travel.
    #[test]
    fn a_drive_is_priced_with_its_dead_man() {
        let c = cfg(true);
        let l = Ladder::new(c.clone());
        let Next::Pulse(p) = l.next() else {
            panic!("a pulse first")
        };
        assert_eq!(p.time_s, PULSE_S + DRIVE_DEADMAN_S);
        assert_eq!(p.drive_s(), PULSE_S);
        assert_eq!(
            p.priced_counts,
            FREE_MARGIN * c.speed_cps(p.duty_q15) * (PULSE_S + DRIVE_DEADMAN_S)
        );
        let q = c.unwedge_q15.expect("metal");
        assert_eq!(
            c.unwedge_priced(q),
            FREE_MARGIN * c.speed_cps(q) * (UNWEDGE_MAX_S + DRIVE_DEADMAN_S) + c.unwedge_stop(q)
        );
    }

    /// A pot that reads no number refuses as a dead pot, before the pulse
    /// is driven and when a pulse's reading has none; a NaN never passes
    /// the over-travel check.
    #[test]
    fn a_non_finite_start_refuses() {
        let Err(Halt::Refused(why)) = start_of(3, f64::NAN) else {
            panic!("refused")
        };
        assert!(
            why.contains("pulse 3") && why.contains("dead or slipped pot") && why.contains(HANDS),
            "{why}"
        );
        assert_eq!(start_of(3, 2041.0), Ok(2041.0));
        assert!(start_of(3, f64::INFINITY).is_err());

        for (start, peak) in [(f64::NAN, 0.0), (2000.0, f64::NAN)] {
            let mut l = Ladder::new(cfg(true));
            l.feed(Seen {
                start,
                peak,
                end: peak,
                emf: 40.0,
                current_peak_a: 0.0,
            });
            let (kind, why) = refused(&l);
            assert_eq!(kind, Refusal::DeadPot, "{why}");
            assert!(why.contains("dead or slipped pot"), "{why}");
            assert!(matches!(l.next(), Next::Done(_)), "nothing more drives");
        }
    }
}
