//! The stop finder's decisions, pure: fed the pot at each poll, it gives
//! the duty for the next poll period, a seat to take the seated bursts
//! at, or its end. Four seats, the low stop, the high, the low and the
//! high again, then a drive to the middle of the two.
//!
//! An approach drives toward its stop at the approach duty with the stall
//! permit held. A window of [`WINDOW_S`] whose every reading lies within
//! the still band of its median is still: the duty rises one step and the
//! next window watches. If the shaft moves on it was a sticky spot: the
//! raised duty stays for the rest of the approach, never lowered and
//! never off. Still for a window at the ceiling is seated. The duty then
//! backs down to the seat duty toward the stop for [`HOLD_S`], and the
//! median of the last [`READ_S`] is the stop's reading; a stop read more
//! than [`WALK_MAX_COUNTS`] from its earlier seat ends all pressing.
//!
//! A leave drives away at the approach duty, without the permit, a step up
//! a still window to the ceiling, until the shaft is [`LEAVE_COUNTS`] off
//! the stop; the next approach carries on from there with no zero between.
//! A leg is at most [`LEG_MAX_S`].
//!
//! The kernel's sign rule with the sign proven: a logical +duty moves the
//! counts up, so the low stop is pressed by -duty.

use super::polarity::{HANDS, Stop};
use crate::fit::median;

/// The host's poll period while a duty is on, s.
pub const POLL_S: f64 = 0.020;
pub const WINDOW_S: f64 = 0.150;
pub const HOLD_S: f64 = 0.300;
pub const READ_S: f64 = 0.100;
/// The still band: this many rest sds of the pot, at least
/// [`STILL_MIN_COUNTS`].
pub const STILL_SD: f64 = 3.0;
pub const STILL_MIN_COUNTS: f64 = 3.0;
pub const LEAVE_COUNTS: f64 = 150.0;
pub const LEG_MAX_S: f64 = 30.0;
pub const WALK_MAX_COUNTS: f64 = 10.0;
/// A leg whose shaft falls back this far from the farthest it got toward
/// its end runs the wrong way.
pub const WRONG_WAY_COUNTS: f64 = LEAVE_COUNTS;
/// The seats in order.
pub const ORDER: [Stop; 4] = [Stop::Low, Stop::High, Stop::Low, Stop::High];
const Q15: f64 = 32768.0;

/// The logical duty's sign that drives toward `s`.
pub fn toward(s: Stop) -> i8 {
    match s {
        Stop::Low => -1,
        Stop::High => 1,
    }
}

fn other(s: Stop) -> Stop {
    match s {
        Stop::Low => Stop::High,
        Stop::High => Stop::Low,
    }
}

/// The duties, Q15 magnitudes, and the still band.
#[derive(Clone, Debug, PartialEq)]
pub struct SeekCfg {
    pub rail_v: f64,
    pub approach_q15: i16,
    pub step_q15: i16,
    pub ceiling_q15: i16,
    pub seat_q15: i16,
    pub still_counts: f64,
}

impl SeekCfg {
    pub fn volts(&self, q15: i16) -> f64 {
        q15.unsigned_abs() as f64 / Q15 * self.rail_v
    }

    fn raised(&self, q15: i16) -> i16 {
        (q15 as i32 + self.step_q15.max(1) as i32).min(self.ceiling_q15 as i32) as i16
    }
}

/// The still band of a pot whose rest sd is `pot_sd` counts.
pub fn still_band(pot_sd: f64) -> f64 {
    (STILL_SD * pot_sd).max(STILL_MIN_COUNTS)
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LegKind {
    Approach,
    Leave,
    /// The last leave, to the middle of the two stops.
    Center,
}

impl LegKind {
    pub fn word(self) -> &'static str {
        match self {
            LegKind::Approach => "approach",
            LegKind::Leave => "leave",
            LegKind::Center => "to the middle",
        }
    }
}

/// A leg driven, as it ended.
#[derive(Clone, Debug, PartialEq)]
pub struct Leg {
    pub n: usize,
    pub kind: LegKind,
    /// The stop it drove toward.
    pub toward: Stop,
    pub from: f64,
    pub to: f64,
    pub secs: f64,
    /// The highest duty it drove at, Q15.
    pub top_q15: i16,
}

/// A spot that held the shaft at one duty and let it go one step up.
#[derive(Clone, Debug, PartialEq)]
pub struct Sticky {
    pub at: f64,
    pub toward: Stop,
    pub kind: LegKind,
    /// The duty that moved it on, Q15.
    pub q15: i16,
}

/// A stop seated and read.
#[derive(Clone, Debug, PartialEq)]
pub struct Seat {
    /// 1 to 4, in [`ORDER`].
    pub n: usize,
    pub stop: Stop,
    /// The median of the hold's last [`READ_S`], counts.
    pub pos: f64,
    /// The reading's spread over the same rows, counts.
    pub range: f64,
    /// Off the same stop's earlier seat, counts.
    pub walk: Option<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Refusal {
    DeadPot,
    TimeOut,
    Walked,
    Blocked,
    WrongWay,
}

impl Refusal {
    /// The evidence contradicts the proven sign: the step aborts.
    pub fn aborts(self) -> bool {
        self == Refusal::WrongWay
    }
}

/// What the host does next.
#[derive(Clone, Debug, PartialEq)]
pub enum Act {
    /// The duty for the next poll period, logical, with the permit held or
    /// not; `price` is the time it is planned for when it starts a window
    /// or a hold, which the host prices before the write.
    Drive {
        q15: i16,
        permit: bool,
        price: Option<f64>,
    },
    /// Seated and read: the duty goes to zero, the seated bursts are
    /// taken, then [`Seek::resume`].
    Seated(Seat),
    /// The four seats read and the shaft at the middle: zero.
    Done,
    /// Refused: zero, nothing more drives.
    Stopped { kind: Refusal, why: String },
}

struct Window {
    t0: f64,
    rows: Vec<f64>,
}

impl Window {
    fn new(t0: f64, pos: f64) -> Self {
        Window {
            t0,
            rows: vec![pos],
        }
    }

    /// Whether the window is complete at `t`, and if so still.
    fn judge(&mut self, t: f64, pos: f64, band: f64) -> Option<bool> {
        self.rows.push(pos);
        if t - self.t0 < WINDOW_S - 1e-9 {
            return None;
        }
        let m = median(&self.rows)?;
        Some(self.rows.iter().all(|x| (x - m).abs() <= band))
    }
}

enum Phase {
    Idle,
    Approach {
        stop: Stop,
        q15: i16,
        /// Where a still window last raised the duty, until it moves on.
        held_at: Option<f64>,
    },
    Hold {
        stop: Stop,
        t0: f64,
        rows: Vec<(f64, f64)>,
    },
    AtSeat,
    Leave {
        from: Stop,
        seat: f64,
        q15: i16,
        /// The middle of the stops, on the last leave.
        center: Option<f64>,
    },
    Over,
}

/// The stop finder's decision.
pub struct Seek {
    cfg: SeekCfg,
    phase: Phase,
    window: Window,
    /// The leg's start: its time, its pos, and the farthest it got toward
    /// its end, counts along its drive.
    leg_t0: f64,
    leg_from: f64,
    leg_top: i16,
    best: f64,
    start: Option<f64>,
    seats: Vec<Seat>,
    legs: Vec<Leg>,
    sticky: Vec<Sticky>,
    path: Vec<String>,
    refused: Option<(Refusal, String)>,
}

impl Seek {
    pub fn new(cfg: SeekCfg) -> Self {
        Seek {
            cfg,
            phase: Phase::Idle,
            window: Window::new(0.0, 0.0),
            leg_t0: 0.0,
            leg_from: 0.0,
            leg_top: 0,
            best: 0.0,
            start: None,
            seats: Vec::new(),
            legs: Vec::new(),
            sticky: Vec::new(),
            path: Vec::new(),
            refused: None,
        }
    }

    pub fn cfg(&self) -> &SeekCfg {
        &self.cfg
    }

    pub fn start_pos(&self) -> Option<f64> {
        self.start
    }

    pub fn seats(&self) -> &[Seat] {
        &self.seats
    }

    pub fn legs(&self) -> &[Leg] {
        &self.legs
    }

    pub fn sticky(&self) -> &[Sticky] {
        &self.sticky
    }

    /// The decisions taken, in order, a line each.
    pub fn path(&self) -> &[String] {
        &self.path
    }

    pub fn refused(&self) -> Option<&(Refusal, String)> {
        self.refused.as_ref()
    }

    /// All four seats read and the shaft brought to the middle.
    pub fn finished(&self) -> bool {
        matches!(self.phase, Phase::Over) && self.refused.is_none()
    }

    fn v(&self, q15: i16) -> f64 {
        self.cfg.volts(q15)
    }

    fn stop(&mut self, kind: Refusal, why: String) -> Act {
        self.path.push(format!("refused: {why}"));
        self.refused = Some((kind, why.clone()));
        self.phase = Phase::Over;
        Act::Stopped { kind, why }
    }

    fn begin_leg(&mut self, t: f64, pos: f64, q15: i16) {
        self.leg_t0 = t;
        self.leg_from = pos;
        self.leg_top = q15;
        self.best = 0.0;
        self.window = Window::new(t, pos);
    }

    fn end_leg(&mut self, kind: LegKind, toward: Stop, t: f64, pos: f64) {
        self.legs.push(Leg {
            n: self.legs.len() + 1,
            kind,
            toward,
            from: self.leg_from,
            to: pos,
            secs: t - self.leg_t0,
            top_q15: self.leg_top,
        });
    }

    fn approach(&mut self, stop: Stop, t: f64, pos: f64) -> Act {
        let q15 = self.cfg.approach_q15;
        self.begin_leg(t, pos, q15);
        self.phase = Phase::Approach {
            stop,
            q15,
            held_at: None,
        };
        self.path.push(format!(
            "seat {}: approach the {} stop from {pos:.0} at {:.3} V, the permit held",
            self.seats.len() + 1,
            stop.word(),
            self.v(q15)
        ));
        Act::Drive {
            q15: q15 * toward(stop) as i16,
            permit: true,
            price: Some(WINDOW_S),
        }
    }

    /// The first approach, from the pot read before any duty.
    pub fn start(&mut self, t: f64, pos: f64) -> Act {
        if !pos.is_finite() {
            return self.dead(pos);
        }
        self.start = Some(pos);
        self.approach(ORDER[0], t, pos)
    }

    fn dead(&mut self, pos: f64) -> Act {
        self.stop(
            Refusal::DeadPot,
            format!(
                "the pot reads {pos}, no number: a dead or slipped pot. Nothing more drives; {HANDS}; if it is refused again, check the pot's wiring"
            ),
        )
    }

    /// The leg's checks common to every drive: the pot reads, the leg is
    /// inside its time, the shaft does not run back from where it got.
    fn leg_checks(&mut self, t: f64, pos: f64, dir: f64, what: &str) -> Option<Act> {
        if !pos.is_finite() {
            return Some(self.dead(pos));
        }
        if t - self.leg_t0 > LEG_MAX_S {
            return Some(self.stop(
                Refusal::TimeOut,
                format!(
                    "the {what} ran {LEG_MAX_S} s from {:.0} and ended at {pos:.0} without its end: the shaft runs too slowly or something holds it. Nothing more drives; {HANDS}",
                    self.leg_from
                ),
            ));
        }
        let along = (pos - self.leg_from) * dir;
        self.best = self.best.max(along);
        if self.best - along > WRONG_WAY_COUNTS {
            return Some(self.stop(
                Refusal::WrongWay,
                format!(
                    "the {what} drove {} counts and the shaft ran {:.0} counts the other way: the drive's sign is not the one the probe proved, or something turns the horn. Nothing more drives; {HANDS}",
                    if dir > 0.0 { "up" } else { "down" },
                    self.best - along
                ),
            ));
        }
        None
    }

    /// The pot read at `t` after a poll period at the last duty given.
    pub fn feed(&mut self, t: f64, pos: f64) -> Act {
        let band = self.cfg.still_counts;
        match std::mem::replace(&mut self.phase, Phase::Over) {
            Phase::Approach { stop, q15, held_at } => {
                let dir = toward(stop);
                let what = format!("approach to the {} stop", stop.word());
                if let Some(a) = self.leg_checks(t, pos, dir as f64, &what) {
                    return a;
                }
                let Some(still) = self.window.judge(t, pos, band) else {
                    self.phase = Phase::Approach { stop, q15, held_at };
                    return Act::Drive {
                        q15: q15 * dir as i16,
                        permit: true,
                        price: None,
                    };
                };
                self.window = Window::new(t, pos);
                if !still {
                    let held_at = match held_at {
                        Some(at) => {
                            self.sticky.push(Sticky {
                                at,
                                toward: stop,
                                kind: LegKind::Approach,
                                q15,
                            });
                            self.path.push(format!(
                                "the shaft moved on at {:.3} V: a sticky spot at {at:.0}; the raised duty stays for the rest of this approach",
                                self.v(q15)
                            ));
                            None
                        }
                        None => None,
                    };
                    self.phase = Phase::Approach { stop, q15, held_at };
                    return Act::Drive {
                        q15: q15 * dir as i16,
                        permit: true,
                        price: Some(WINDOW_S),
                    };
                }
                if q15 < self.cfg.ceiling_q15 {
                    let up = self.cfg.raised(q15);
                    self.leg_top = self.leg_top.max(up);
                    self.path.push(format!(
                        "still at {pos:.0} for {WINDOW_S} s at {:.3} V: raised to {:.3} V",
                        self.v(q15),
                        self.v(up)
                    ));
                    self.phase = Phase::Approach {
                        stop,
                        q15: up,
                        held_at: held_at.or(Some(pos)),
                    };
                    return Act::Drive {
                        q15: up * dir as i16,
                        permit: true,
                        price: Some(WINDOW_S),
                    };
                }
                self.end_leg(LegKind::Approach, stop, t, pos);
                self.path.push(format!(
                    "still at {pos:.0} for {WINDOW_S} s at the ceiling's {:.3} V: seated; backed down to {:.3} V for {HOLD_S} s",
                    self.v(q15),
                    self.v(self.cfg.seat_q15)
                ));
                self.phase = Phase::Hold {
                    stop,
                    t0: t,
                    rows: Vec::new(),
                };
                Act::Drive {
                    q15: self.cfg.seat_q15 * dir as i16,
                    permit: true,
                    price: Some(HOLD_S),
                }
            }
            Phase::Hold { stop, t0, mut rows } => {
                if !pos.is_finite() {
                    return self.dead(pos);
                }
                rows.push((t, pos));
                if t - t0 < HOLD_S - 1e-9 {
                    self.phase = Phase::Hold { stop, t0, rows };
                    return Act::Drive {
                        q15: self.cfg.seat_q15 * toward(stop) as i16,
                        permit: true,
                        price: None,
                    };
                }
                let tail: Vec<f64> = rows
                    .iter()
                    .filter(|(at, _)| *at >= t - READ_S - 1e-9)
                    .map(|(_, p)| *p)
                    .collect();
                let read = median(&tail).unwrap_or(pos);
                let range = tail.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                    - tail.iter().copied().fold(f64::INFINITY, f64::min);
                let n = self.seats.len() + 1;
                let walk = self
                    .seats
                    .iter()
                    .find(|s| s.stop == stop)
                    .map(|s| read - s.pos);
                self.seats.push(Seat {
                    n,
                    stop,
                    pos: read,
                    range,
                    walk,
                });
                if let Some(w) = walk.filter(|w| w.abs() > WALK_MAX_COUNTS) {
                    return self.stop(
                        Refusal::Walked,
                        format!(
                            "seat {n} reads the {} stop at {read:.0}, {w:+.0} counts off its earlier seat, over {WALK_MAX_COUNTS}: a stopper that yields, or a stall that was no stop. All pressing ends; {HANDS}, and check the horn and the stop for damage",
                            stop.word()
                        ),
                    );
                }
                self.path.push(format!(
                    "seat {n}: the {} stop reads {read:.0} (the median of the last {READ_S} s){}",
                    stop.word(),
                    walk.map_or(String::new(), |w| format!(
                        ", {w:+.0} counts off its earlier seat"
                    ))
                ));
                self.phase = Phase::AtSeat;
                Act::Seated(self.seats[n - 1].clone())
            }
            Phase::Leave {
                from,
                seat,
                q15,
                center,
            } => {
                let to = other(from);
                let dir = toward(to);
                let kind = if center.is_some() {
                    LegKind::Center
                } else {
                    LegKind::Leave
                };
                let what = format!("leave from the {} stop", from.word());
                if let Some(a) = self.leg_checks(t, pos, dir as f64, &what) {
                    return a;
                }
                let arrived = match center {
                    Some(m) => (pos - m) * dir as f64 >= 0.0,
                    None => (pos - seat).abs() >= LEAVE_COUNTS,
                };
                if arrived {
                    self.end_leg(kind, to, t, pos);
                    if center.is_some() {
                        self.path
                            .push(format!("at {pos:.0}, the middle of the stops: zero"));
                        self.phase = Phase::Over;
                        return Act::Done;
                    }
                    return self.approach(to, t, pos);
                }
                let mut q = q15;
                if let Some(still) = self.window.judge(t, pos, band) {
                    self.window = Window::new(t, pos);
                    if still {
                        if q15 >= self.cfg.ceiling_q15 {
                            return self.stop(
                                Refusal::Blocked,
                                format!(
                                    "the shaft does not leave the {} stop at the ceiling's {:.3} V (still at {pos:.0}): a shaft blocked there or a dead pot. Nothing more drives; {HANDS}",
                                    from.word(),
                                    self.v(q15)
                                ),
                            );
                        }
                        q = self.cfg.raised(q15);
                        self.leg_top = self.leg_top.max(q);
                        self.path.push(format!(
                            "leaving the {} stop: still at {pos:.0} at {:.3} V: raised to {:.3} V",
                            from.word(),
                            self.v(q15),
                            self.v(q)
                        ));
                    }
                    self.phase = Phase::Leave {
                        from,
                        seat,
                        q15: q,
                        center,
                    };
                    return Act::Drive {
                        q15: q * dir as i16,
                        permit: false,
                        price: Some(WINDOW_S),
                    };
                }
                self.phase = Phase::Leave {
                    from,
                    seat,
                    q15: q,
                    center,
                };
                Act::Drive {
                    q15: q * dir as i16,
                    permit: false,
                    price: None,
                }
            }
            other_phase => {
                self.phase = other_phase;
                self.after()
            }
        }
    }

    /// The seated bursts taken: leave the stop just read, at `t` from
    /// `pos`.
    pub fn resume(&mut self, t: f64, pos: f64) -> Act {
        if !matches!(self.phase, Phase::AtSeat) {
            return self.after();
        }
        if !pos.is_finite() {
            return self.dead(pos);
        }
        let Some(last) = self.seats.last().cloned() else {
            return self.after();
        };
        let center = (self.seats.len() == ORDER.len()).then(|| {
            let at = |s: Stop| {
                self.seats
                    .iter()
                    .filter(|x| x.stop == s)
                    .map(|x| x.pos)
                    .collect::<Vec<_>>()
            };
            (median(&at(Stop::Low)).unwrap_or(pos) + median(&at(Stop::High)).unwrap_or(pos)) / 2.0
        });
        let q15 = self.cfg.approach_q15;
        self.begin_leg(t, pos, q15);
        self.path.push(match center {
            Some(m) => format!(
                "leave the {} stop at {:.3} V, without the permit, to the middle at {m:.0}",
                last.stop.word(),
                self.v(q15)
            ),
            None => format!(
                "leave the {} stop at {:.3} V, without the permit, until {LEAVE_COUNTS} counts off it",
                last.stop.word(),
                self.v(q15)
            ),
        });
        self.phase = Phase::Leave {
            from: last.stop,
            seat: last.pos,
            q15,
            center,
        };
        Act::Drive {
            q15: q15 * toward(other(last.stop)) as i16,
            permit: false,
            price: Some(WINDOW_S),
        }
    }

    fn after(&self) -> Act {
        match &self.refused {
            Some((kind, why)) => Act::Stopped {
                kind: *kind,
                why: why.clone(),
            },
            None => Act::Done,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench servo's class on a 5 V rail: the probe proved the sign at
    /// 0.48 V, the ceiling 0.945 V (0.25 A at 3.78 ohm), a 4 % step of
    /// 4.8 V.
    fn cfg() -> SeekCfg {
        let rail_v = 5.0;
        let q = |v: f64| (v / rail_v * Q15).floor() as i16;
        let ceiling = q(0.945);
        SeekCfg {
            rail_v,
            approach_q15: q(0.48 + 0.192).min(ceiling),
            step_q15: q(0.192),
            ceiling_q15: ceiling,
            seat_q15: q(0.945 / 2.0),
            still_counts: still_band(0.8),
        }
    }

    /// A shaft between stops at `lo` and `hi`: it turns at `cps_per_v`
    /// counts/s a volt over `breakaway_v`; a sticky spot over `sticky`
    /// needs `sticky_v`; a jam at `jam` stops it going down at any duty;
    /// the low stop moves `walk` counts after its first seat; a dead pot
    /// reads `dead` whatever the shaft does; the pot reads with a +/-1
    /// count wobble.
    struct Shaft {
        pos: f64,
        lo: f64,
        hi: f64,
        cps_per_v: f64,
        breakaway_v: f64,
        sticky: Option<(f64, f64, f64)>,
        jam: Option<f64>,
        walk: f64,
        dead: Option<f64>,
        k: u64,
        seats_low: u32,
    }

    impl Shaft {
        fn bench() -> Self {
            Shaft {
                pos: 2000.0,
                lo: 432.0,
                hi: 3626.0,
                cps_per_v: 3000.0,
                breakaway_v: 0.4,
                sticky: None,
                jam: None,
                walk: 0.0,
                dead: None,
                k: 0,
                seats_low: 0,
            }
        }

        fn drive(&mut self, c: &SeekCfg, q15: i16, s: f64) {
            let v = c.volts(q15);
            let dir = (q15 as f64).signum();
            let need = match self.sticky {
                Some((a, b, sv)) if (a..=b).contains(&self.pos) => sv,
                _ => self.breakaway_v,
            };
            if v < need || q15 == 0 {
                return;
            }
            let mut to = self.pos + dir * self.cps_per_v * (v - need).max(0.05) * s;
            if let Some(j) = self.jam.filter(|j| dir < 0.0 && self.pos >= *j) {
                to = to.max(j);
            }
            self.pos = to.clamp(self.lo, self.hi);
        }

        fn read(&mut self) -> f64 {
            self.k += 1;
            self.dead
                .unwrap_or(self.pos + if self.k % 2 == 0 { 1.0 } else { -1.0 })
        }
    }

    /// The machine run on `shaft` to its end, polled every POLL_S; each
    /// seat's bursts take a second, then the stopper walks if it does.
    fn run(shaft: &mut Shaft) -> (Seek, Vec<i16>) {
        let c = cfg();
        let mut m = Seek::new(c.clone());
        let mut t = 0.0;
        let mut duties = Vec::new();
        let mut act = m.start(t, shaft.read());
        for _ in 0..20_000 {
            match act {
                Act::Drive { q15, .. } => {
                    duties.push(q15);
                    shaft.drive(&c, q15, POLL_S);
                    t += POLL_S;
                    act = m.feed(t, shaft.read());
                }
                Act::Seated(s) => {
                    duties.push(0);
                    if s.stop == Stop::Low {
                        shaft.seats_low += 1;
                        if shaft.seats_low == 1 {
                            shaft.lo += shaft.walk;
                        }
                    }
                    t += 1.0;
                    act = m.resume(t, shaft.read());
                }
                Act::Done | Act::Stopped { .. } => return (m, duties),
            }
        }
        panic!("never ends: {:?}", m.path());
    }

    fn refusal(m: &Seek) -> Refusal {
        m.refused()
            .map(|r| r.0)
            .unwrap_or_else(|| panic!("not refused: {:?}", m.path()))
    }

    #[test]
    fn four_seats_alternate_the_stops_and_end_at_the_middle() {
        let mut s = Shaft::bench();
        let (m, duties) = run(&mut s);
        assert!(m.finished(), "{:?}", m.path());
        let seats: Vec<(Stop, f64)> = m.seats().iter().map(|x| (x.stop, x.pos)).collect();
        assert_eq!(seats.len(), 4);
        for (k, (stop, pos)) in seats.iter().enumerate() {
            assert_eq!(*stop, ORDER[k]);
            let want = if *stop == Stop::Low { 432.0 } else { 3626.0 };
            assert!((pos - want).abs() <= 1.0, "seat {k} at {pos}");
        }
        assert!((s.pos - 2029.0).abs() < 100.0, "near the middle: {}", s.pos);
        let c = cfg();
        assert!(
            duties
                .iter()
                .all(|q| q.unsigned_abs() <= c.ceiling_q15 as u16)
        );
        assert!(m.sticky().is_empty());
        // an approach to the stop, the leaves and the last leg
        let kinds: Vec<LegKind> = m.legs().iter().map(|l| l.kind).collect();
        use LegKind::*;
        assert_eq!(
            kinds,
            [
                Approach, Leave, Approach, Leave, Approach, Leave, Approach, Center
            ]
        );
    }

    /// An approach that meets a spot it cannot pass at the approach duty:
    /// one step up moves it on, the raised duty stays to the stop and is
    /// never lowered, and the spot is noted.
    #[test]
    fn a_sticky_spot_at_mid_travel_rides_one_step_up() {
        let c = cfg();
        let mut s = Shaft {
            sticky: Some((1400.0, 1500.0, c.volts(c.approach_q15) + 0.05)),
            ..Shaft::bench()
        };
        let (m, duties) = run(&mut s);
        assert!(m.finished(), "{:?}", m.path());
        let first = &m.sticky()[0];
        assert!((1390.0..=1510.0).contains(&first.at), "{first:?}");
        assert_eq!(first.q15, c.raised(c.approach_q15));
        // the first approach's duties never fall until the seat
        let first_leg: Vec<i16> = duties
            .iter()
            .take_while(|q| q.unsigned_abs() != c.seat_q15 as u16)
            .map(|q| -q)
            .collect();
        assert!(first_leg.windows(2).all(|w| w[1] >= w[0]), "{first_leg:?}");
        assert!(
            m.seats()
                .iter()
                .all(|x| (x.pos - 432.0).abs() <= 1.0 || (x.pos - 3626.0).abs() <= 1.0)
        );
    }

    /// A jam short of the low stop that no duty frees seats there; the
    /// machine finds four seats, and the span check refuses it later.
    #[test]
    fn a_stall_short_of_the_stop_seats_there() {
        let mut s = Shaft {
            jam: Some(1900.0),
            ..Shaft::bench()
        };
        let (m, _) = run(&mut s);
        assert!(m.finished(), "{:?}", m.path());
        let low: Vec<f64> = m
            .seats()
            .iter()
            .filter(|x| x.stop == Stop::Low)
            .map(|x| x.pos)
            .collect();
        assert!(low.iter().all(|p| (p - 1900.0).abs() <= 1.0), "{low:?}");
    }

    #[test]
    fn a_stopper_that_walks_ends_all_pressing() {
        let mut s = Shaft {
            walk: -15.0,
            ..Shaft::bench()
        };
        let (m, _) = run(&mut s);
        assert_eq!(refusal(&m), Refusal::Walked);
        assert_eq!(m.seats().len(), 3);
        let why = &m.refused().expect("refused").1;
        assert!(
            why.contains("-15 counts") && why.contains("All pressing ends"),
            "{why}"
        );
    }

    #[test]
    fn a_leg_over_its_time_is_refused() {
        // 1568 counts at about 50 counts/s: over 30 s
        let c = cfg();
        let mut s = Shaft {
            cps_per_v: 50.0 / (c.volts(c.approach_q15) - 0.4),
            ..Shaft::bench()
        };
        let (m, _) = run(&mut s);
        assert_eq!(refusal(&m), Refusal::TimeOut);
        assert!(m.seats().is_empty());
        assert!(m.refused().expect("refused").1.contains(HANDS));
    }

    /// A pot stuck at one reading seats the first approach where it reads,
    /// then no duty leaves it; a pot that reads no number refuses at once.
    #[test]
    fn a_dead_pot_is_refused() {
        let mut s = Shaft {
            dead: Some(2000.0),
            ..Shaft::bench()
        };
        let (m, duties) = run(&mut s);
        assert_eq!(refusal(&m), Refusal::Blocked, "{:?}", m.path());
        assert_eq!(m.seats().len(), 1);
        let c = cfg();
        assert!(
            duties
                .iter()
                .all(|q| q.unsigned_abs() <= c.ceiling_q15 as u16)
        );
        assert!(m.refused().expect("refused").1.contains("dead pot"));

        let mut s = Shaft {
            dead: Some(f64::NAN),
            ..Shaft::bench()
        };
        let (m, duties) = run(&mut s);
        assert_eq!(refusal(&m), Refusal::DeadPot);
        assert!(duties.is_empty(), "nothing drives");
    }

    /// A shaft that runs the other way from the duty: the sign is not the
    /// proven one, and the step aborts.
    #[test]
    fn a_shaft_that_runs_the_wrong_way_aborts() {
        let c = cfg();
        let mut m = Seek::new(c.clone());
        let mut s = Shaft::bench();
        let mut t = 0.0;
        let mut act = m.start(t, s.read());
        while let Act::Drive { q15, .. } = act {
            s.drive(&c, -q15, POLL_S);
            t += POLL_S;
            act = m.feed(t, s.read());
        }
        assert!(
            matches!(
                act,
                Act::Stopped {
                    kind: Refusal::WrongWay,
                    ..
                }
            ),
            "{act:?}"
        );
        assert!(Refusal::WrongWay.aborts());
    }

    #[test]
    fn the_still_band_is_three_rest_sds_at_least_three_counts() {
        assert_eq!(still_band(0.5), 3.0);
        assert!((still_band(2.0) - 6.0).abs() < 1e-12);
    }
}
