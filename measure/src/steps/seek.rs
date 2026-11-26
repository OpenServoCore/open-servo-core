//! The stop finder's decisions, pure: fed the pot at each poll, it gives
//! the duty for the next poll period, a seat to take the seated bursts
//! at, or its end. Approaches alternate between the stops, the low one
//! first, until each stop has three seats (plan sec 2.3); then a drive to
//! the middle of the two.
//!
//! An approach drives toward its stop at the approach duty with the stall
//! permit held, and that duty is never raised: a soft stopper gives under
//! a harder press, and on this servo's low stop it carries the shaft past
//! the pot's range. A window of [`WINDOW_S`] whose every reading lies
//! within the still band of its median is still. Still after at least
//! [`TRAVEL_MIN_COUNTS`] of the leg's travel is seated: the duty backs
//! down to the seat duty toward the stop for [`HOLD_S`], and the median of
//! the last [`READ_S`] is the stop's reading; a stop read more than
//! [`WALK_MAX_COUNTS`] from its earlier seat ends all pressing. Still
//! short of that travel on the first approach means the start was at or
//! by that stop: no seat, and the finder leaves for the other one. On a
//! later approach it is a shaft blocked between the stops.
//!
//! A leave drives away at the approach duty, without the permit, a step up
//! a still window to the ceiling, until the shaft is [`LEAVE_COUNTS`] off
//! the stop; the next approach carries on from there with no zero between.
//! A leg is at most [`LEG_MAX_S`].
//!
//! A leg that falls back more than [`WRONG_WAY_COUNTS`] from the farthest
//! it got runs the wrong way, but only on two polls in a row: a single
//! poll over the line is held out of every decision while the drive runs
//! on one more poll period at the duty in force, and is counted as an odd
//! read when the next poll is back inside.
//!
//! A refusal taken with the shaft at a seat it just read, the walk or a
//! blocked approach, first drives off that stop by a leave: to the middle
//! when both stops are known, else [`CLEAR_COUNTS`] off it. Every other
//! refusal stops at once.
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
/// Travel an approach must make before a still shaft is a seat, counts.
/// A few counts of elastic wind-up at a stop pass any stillness test, and
/// a finder that takes them for travel seats against the stop it started
/// on.
pub const TRAVEL_MIN_COUNTS: f64 = 100.0;
pub const LEAVE_COUNTS: f64 = 150.0;
/// How far a refusal's leave drives off a stop when the other is not
/// known, counts.
pub const CLEAR_COUNTS: f64 = 4.0 * LEAVE_COUNTS;
pub const LEG_MAX_S: f64 = 30.0;
pub const WALK_MAX_COUNTS: f64 = 10.0;
/// A leg whose shaft falls back this far from the farthest it got toward
/// its end runs the wrong way.
pub const WRONG_WAY_COUNTS: f64 = LEAVE_COUNTS;
pub const SEATS_A_STOP: usize = 3;
/// The most approaches a run makes: one that finds the start at a stop,
/// then [`SEATS_A_STOP`] seats a stop.
pub const APPROACHES_MAX: usize = 1 + 2 * SEATS_A_STOP;
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

/// A leave that went still and took a step up.
#[derive(Clone, Debug, PartialEq)]
pub struct LeaveRaise {
    pub at: f64,
    /// The stop it was leaving.
    pub from: Stop,
    /// The duty it rose to, Q15.
    pub q15: i16,
}

/// A poll over the wrong-way line that the next poll did not confirm.
#[derive(Clone, Debug, PartialEq)]
pub struct OddRead {
    pub leg: usize,
    pub kind: LegKind,
    pub toward: Stop,
    /// The polls before and after it, counts.
    pub before: f64,
    pub read: f64,
    pub after: f64,
}

/// What a leg's checks made of a poll.
enum Fed {
    Go,
    /// Over the wrong-way line once: held out until the next poll.
    Suspect,
    Stop(Refusal, String),
}

/// A stop seated and read.
#[derive(Clone, Debug, PartialEq)]
pub struct Seat {
    /// From 1, in the order taken.
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
    /// Every seat read and the shaft at the middle: zero.
    Done,
    /// Refused: zero, nothing more drives.
    Stopped { kind: Refusal, why: String },
}

pub(crate) struct Window {
    t0: f64,
    rows: Vec<f64>,
}

impl Window {
    pub(crate) fn new(t0: f64, pos: f64) -> Self {
        Window {
            t0,
            rows: vec![pos],
        }
    }

    /// Whether the window is complete at `t`, and if so still.
    pub(crate) fn judge(&mut self, t: f64, pos: f64, band: f64) -> Option<bool> {
        self.rows.push(pos);
        if t - self.t0 < WINDOW_S - 1e-9 {
            return None;
        }
        let m = median(&self.rows)?;
        Some(self.rows.iter().all(|x| (x - m).abs() <= band))
    }

    /// The readings' median.
    pub(crate) fn median(&self) -> Option<f64> {
        median(&self.rows)
    }
}

/// Where a leave ends.
#[derive(Copy, Clone, Debug, PartialEq)]
enum End {
    /// [`LEAVE_COUNTS`] off the stop, then the next approach.
    Approach,
    /// Past the middle of the stops.
    Center(f64),
    /// This far off the stop.
    Off(f64),
}

enum Phase {
    Idle,
    Approach {
        stop: Stop,
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
        end: End,
        /// The refusal this leave drives off the stop ahead of.
        then: Option<(Refusal, String)>,
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
    /// The leg's last poll believed, and one over the wrong-way line
    /// waiting on the next: its reading and the one before it.
    last: f64,
    suspect: Option<(f64, f64)>,
    odd: Vec<OddRead>,
    start: Option<f64>,
    approaches: usize,
    seats: Vec<Seat>,
    legs: Vec<Leg>,
    raises: Vec<LeaveRaise>,
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
            last: 0.0,
            suspect: None,
            odd: Vec::new(),
            start: None,
            approaches: 0,
            seats: Vec::new(),
            legs: Vec::new(),
            raises: Vec::new(),
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

    /// The approaches driven.
    pub fn approaches(&self) -> usize {
        self.approaches
    }

    pub fn leave_raises(&self) -> &[LeaveRaise] {
        &self.raises
    }

    /// The decisions taken, in order, a line each.
    pub fn path(&self) -> &[String] {
        &self.path
    }

    /// The single polls over the wrong-way line the next poll put back.
    pub fn odd_reads(&self) -> &[OddRead] {
        &self.odd
    }

    /// The leg the next poll fed belongs to, its number and word: the
    /// hold and the read after a seat's bursts name the approach that
    /// seated.
    pub fn leg_now(&self) -> (usize, &'static str) {
        let n = self.legs.len();
        match &self.phase {
            Phase::Idle => (1, "start"),
            Phase::Approach { .. } => (n + 1, LegKind::Approach.word()),
            Phase::Hold { .. } => (n, "hold"),
            Phase::AtSeat => (n, "seated"),
            Phase::Leave {
                end: End::Center(_),
                ..
            } => (n + 1, LegKind::Center.word()),
            Phase::Leave { .. } => (n + 1, LegKind::Leave.word()),
            Phase::Over => (n, "over"),
        }
    }

    pub fn refused(&self) -> Option<&(Refusal, String)> {
        self.refused.as_ref()
    }

    /// Every seat read and the shaft brought to the middle.
    pub fn finished(&self) -> bool {
        matches!(self.phase, Phase::Over) && self.refused.is_none()
    }

    fn v(&self, q15: i16) -> f64 {
        self.cfg.volts(q15)
    }

    fn seats_at(&self, s: Stop) -> Vec<f64> {
        self.seats
            .iter()
            .filter(|x| x.stop == s)
            .map(|x| x.pos)
            .collect()
    }

    /// The middle of the stops, once each has a seat.
    fn middle(&self) -> Option<f64> {
        Some((median(&self.seats_at(Stop::Low))? + median(&self.seats_at(Stop::High))?) / 2.0)
    }

    fn stop(&mut self, kind: Refusal, why: String) -> Act {
        self.path.push(format!("refused: {why}"));
        self.refused = Some((kind, why.clone()));
        self.phase = Phase::Over;
        Act::Stopped { kind, why }
    }

    /// Refuse, first driving off the stop of the last seat when the shaft
    /// is still at it.
    fn refuse_off(&mut self, kind: Refusal, why: String, t: f64, pos: f64) -> Act {
        let Some(last) = self
            .seats
            .last()
            .filter(|s| (pos - s.pos).abs() <= LEAVE_COUNTS)
            .cloned()
        else {
            return self.stop(kind, why);
        };
        let end = match self.middle() {
            Some(m) => End::Center(m),
            None => End::Off(CLEAR_COUNTS),
        };
        self.path.push(format!(
            "refusing: {why}; first a leave off the {} stop, {}",
            last.stop.word(),
            match end {
                End::Center(m) => format!("to the middle at {m:.0}"),
                _ => format!("{CLEAR_COUNTS} counts off it"),
            }
        ));
        self.leave(last.stop, last.pos, end, Some((kind, why)), t, pos)
    }

    fn begin_leg(&mut self, t: f64, pos: f64, q15: i16) {
        self.leg_t0 = t;
        self.leg_from = pos;
        self.leg_top = q15;
        self.best = 0.0;
        self.last = pos;
        self.suspect = None;
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
        self.approaches += 1;
        self.begin_leg(t, pos, q15);
        self.phase = Phase::Approach { stop };
        self.path.push(format!(
            "approach {}: toward the {} stop from {pos:.0} at {:.3} V, the permit held",
            self.approaches,
            stop.word(),
            self.v(q15)
        ));
        self.press(stop, q15, Some(WINDOW_S))
    }

    fn press(&self, stop: Stop, q15: i16, price: Option<f64>) -> Act {
        Act::Drive {
            q15: q15 * toward(stop) as i16,
            permit: true,
            price,
        }
    }

    fn leave(
        &mut self,
        from: Stop,
        seat: f64,
        end: End,
        then: Option<(Refusal, String)>,
        t: f64,
        pos: f64,
    ) -> Act {
        let q15 = self.cfg.approach_q15;
        self.begin_leg(t, pos, q15);
        self.phase = Phase::Leave {
            from,
            seat,
            q15,
            end,
            then,
        };
        Act::Drive {
            q15: q15 * toward(other(from)) as i16,
            permit: false,
            price: Some(WINDOW_S),
        }
    }

    /// The first approach, from the pot read before any duty.
    pub fn start(&mut self, t: f64, pos: f64) -> Act {
        if !pos.is_finite() {
            return self.stop(Refusal::DeadPot, dead(pos));
        }
        self.start = Some(pos);
        self.approach(Stop::Low, t, pos)
    }

    /// The leg's checks common to every drive: the pot reads, the leg is
    /// inside its time, the shaft does not run back from where it got on
    /// two polls in a row.
    fn leg_checks(&mut self, t: f64, pos: f64, kind: LegKind, to: Stop, what: &str) -> Fed {
        if !pos.is_finite() {
            return Fed::Stop(Refusal::DeadPot, dead(pos));
        }
        if t - self.leg_t0 > LEG_MAX_S {
            return Fed::Stop(
                Refusal::TimeOut,
                format!(
                    "the {what} ran {LEG_MAX_S} s from {:.0} and ended at {pos:.0} without its end: the shaft runs too slowly or something holds it. Nothing more drives; {HANDS}",
                    self.leg_from
                ),
            );
        }
        let dir = toward(to) as f64;
        let along = (pos - self.leg_from) * dir;
        if self.best - along > WRONG_WAY_COUNTS {
            let Some((first, _)) = self.suspect else {
                self.suspect = Some((pos, self.last));
                return Fed::Suspect;
            };
            return Fed::Stop(
                Refusal::WrongWay,
                format!(
                    "the {what} drove {} counts and the shaft ran {:.0} counts the other way on two polls in a row ({first:.0}, then {pos:.0}): the drive's sign is not the one the probe proved, or something turns the horn. Nothing more drives; {HANDS}",
                    if dir > 0.0 { "up" } else { "down" },
                    self.best - along
                ),
            );
        }
        if let Some((read, before)) = self.suspect.take() {
            self.path.push(format!(
                "the {what}: one poll read {read:.0} between {before:.0} and {pos:.0}, over the wrong-way line; set aside"
            ));
            self.odd.push(OddRead {
                leg: self.legs.len() + 1,
                kind,
                toward: to,
                before,
                read,
                after: pos,
            });
        }
        self.best = self.best.max(along);
        self.last = pos;
        Fed::Go
    }

    /// The pot read at `t` after a poll period at the last duty given.
    pub fn feed(&mut self, t: f64, pos: f64) -> Act {
        match std::mem::replace(&mut self.phase, Phase::Over) {
            Phase::Approach { stop } => self.on_approach(stop, t, pos),
            Phase::Hold { stop, t0, rows } => self.on_hold(stop, t0, rows, t, pos),
            Phase::Leave {
                from,
                seat,
                q15,
                end,
                then,
            } => self.on_leave(from, seat, q15, end, then, t, pos),
            other_phase => {
                self.phase = other_phase;
                self.after()
            }
        }
    }

    fn on_approach(&mut self, stop: Stop, t: f64, pos: f64) -> Act {
        let q15 = self.cfg.approach_q15;
        let what = format!("approach to the {} stop", stop.word());
        match self.leg_checks(t, pos, LegKind::Approach, stop, &what) {
            Fed::Go => {}
            Fed::Suspect => {
                self.phase = Phase::Approach { stop };
                return self.press(stop, q15, None);
            }
            Fed::Stop(kind, why) => return self.stop(kind, why),
        }
        let Some(still) = self.window.judge(t, pos, self.cfg.still_counts) else {
            self.phase = Phase::Approach { stop };
            return self.press(stop, q15, None);
        };
        self.window = Window::new(t, pos);
        if !still {
            self.phase = Phase::Approach { stop };
            return self.press(stop, q15, Some(WINDOW_S));
        }
        let travel = (pos - self.leg_from).abs();
        self.end_leg(LegKind::Approach, stop, t, pos);
        if travel < TRAVEL_MIN_COUNTS {
            if self.approaches == 1 {
                self.path.push(format!(
                    "still at {pos:.0} after {travel:.0} counts of travel, under {TRAVEL_MIN_COUNTS}: the start was at or by the {} stop; no seat there, the finder leaves for the {} stop",
                    stop.word(),
                    other(stop).word()
                ));
                return self.leave(stop, pos, End::Approach, None, t, pos);
            }
            return self.refuse_off(
                Refusal::Blocked,
                format!(
                    "the approach to the {} stop went still at {pos:.0} after {travel:.0} counts from {:.0}, under {TRAVEL_MIN_COUNTS}: the shaft is blocked between the stops. Nothing more presses; {HANDS}, and check that nothing holds the horn",
                    stop.word(),
                    self.leg_from
                ),
                t,
                pos,
            );
        }
        self.path.push(format!(
            "still at {pos:.0} for {WINDOW_S} s at {:.3} V after {travel:.0} counts: seated; backed down to {:.3} V for {HOLD_S} s",
            self.v(q15),
            self.v(self.cfg.seat_q15)
        ));
        self.phase = Phase::Hold {
            stop,
            t0: t,
            rows: Vec::new(),
        };
        self.press(stop, self.cfg.seat_q15, Some(HOLD_S))
    }

    fn on_hold(&mut self, stop: Stop, t0: f64, mut rows: Vec<(f64, f64)>, t: f64, pos: f64) -> Act {
        if !pos.is_finite() {
            return self.stop(Refusal::DeadPot, dead(pos));
        }
        rows.push((t, pos));
        if t - t0 < HOLD_S - 1e-9 {
            self.phase = Phase::Hold { stop, t0, rows };
            return self.press(stop, self.cfg.seat_q15, None);
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
            return self.refuse_off(
                Refusal::Walked,
                format!(
                    "seat {n} reads the {} stop at {read:.0}, {w:+.0} counts off its earlier seat, over {WALK_MAX_COUNTS}: a stopper that yields, or a stall that was no stop. All pressing ends; check the horn and the stop for damage, then {HANDS}",
                    stop.word()
                ),
                t,
                pos,
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

    /// A leave failed: a plain refusal, or, ahead of one, the refusal it
    /// was driving off the stop for, the horn left there.
    fn leave_failed(
        &mut self,
        from: Stop,
        then: Option<(Refusal, String)>,
        kind: Refusal,
        why: String,
    ) -> Act {
        match then {
            None => self.stop(kind, why),
            Some((k0, w0)) => self.stop(
                if kind.aborts() { kind } else { k0 },
                format!(
                    "{w0}. The leave off the stop failed ({why}): the horn was left at the {} stop",
                    from.word()
                ),
            ),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn on_leave(
        &mut self,
        from: Stop,
        seat: f64,
        q15: i16,
        end: End,
        then: Option<(Refusal, String)>,
        t: f64,
        pos: f64,
    ) -> Act {
        let to = other(from);
        let dir = toward(to);
        let kind = match end {
            End::Center(_) => LegKind::Center,
            _ => LegKind::Leave,
        };
        let what = format!("leave from the {} stop", from.word());
        match self.leg_checks(t, pos, kind, to, &what) {
            Fed::Go => {}
            Fed::Suspect => {
                self.phase = Phase::Leave {
                    from,
                    seat,
                    q15,
                    end,
                    then,
                };
                return Act::Drive {
                    q15: q15 * dir as i16,
                    permit: false,
                    price: None,
                };
            }
            Fed::Stop(k, why) => return self.leave_failed(from, then, k, why),
        }
        let arrived = match end {
            End::Approach => (pos - seat).abs() >= LEAVE_COUNTS,
            End::Center(m) => (pos - m) * dir as f64 >= 0.0,
            End::Off(c) => (pos - seat).abs() >= c,
        };
        if arrived {
            self.end_leg(kind, to, t, pos);
            if let Some((k0, w0)) = then {
                return self.stop(
                    k0,
                    format!(
                        "{w0}. The horn was driven off the {} stop first",
                        from.word()
                    ),
                );
            }
            if end == End::Approach {
                return self.approach(to, t, pos);
            }
            self.path
                .push(format!("at {pos:.0}, the middle of the stops: zero"));
            self.phase = Phase::Over;
            return Act::Done;
        }
        let mut q = q15;
        let price = match self.window.judge(t, pos, self.cfg.still_counts) {
            None => None,
            Some(still) => {
                self.window = Window::new(t, pos);
                if still {
                    if q15 >= self.cfg.ceiling_q15 {
                        let why = format!(
                            "the shaft does not leave the {} stop at the ceiling's {:.3} V (still at {pos:.0}): a shaft blocked there or a dead pot. Nothing more drives; {HANDS}",
                            from.word(),
                            self.v(q15)
                        );
                        return self.leave_failed(from, then, Refusal::Blocked, why);
                    }
                    q = self.cfg.raised(q15);
                    self.leg_top = self.leg_top.max(q);
                    self.raises.push(LeaveRaise {
                        at: pos,
                        from,
                        q15: q,
                    });
                    self.path.push(format!(
                        "leaving the {} stop: still at {pos:.0} at {:.3} V: raised to {:.3} V",
                        from.word(),
                        self.v(q15),
                        self.v(q)
                    ));
                }
                Some(WINDOW_S)
            }
        };
        self.phase = Phase::Leave {
            from,
            seat,
            q15: q,
            end,
            then,
        };
        Act::Drive {
            q15: q * dir as i16,
            permit: false,
            price,
        }
    }

    /// The seated bursts taken: leave the stop just read, at `t` from
    /// `pos`.
    pub fn resume(&mut self, t: f64, pos: f64) -> Act {
        if !matches!(self.phase, Phase::AtSeat) {
            return self.after();
        }
        if !pos.is_finite() {
            return self.stop(Refusal::DeadPot, dead(pos));
        }
        let Some(last) = self.seats.last().cloned() else {
            return self.after();
        };
        let done = [Stop::Low, Stop::High]
            .iter()
            .all(|s| self.seats_at(*s).len() >= SEATS_A_STOP);
        let end = match self.middle().filter(|_| done) {
            Some(m) => End::Center(m),
            None => End::Approach,
        };
        if end == End::Approach && self.approaches >= APPROACHES_MAX {
            return self.refuse_off(
                Refusal::Blocked,
                format!(
                    "{APPROACHES_MAX} approaches gave no {SEATS_A_STOP} seats at each stop. Nothing more presses; {HANDS}"
                ),
                t,
                pos,
            );
        }
        self.path.push(match end {
            End::Center(m) => format!(
                "leave the {} stop at {:.3} V, without the permit, to the middle at {m:.0}",
                last.stop.word(),
                self.v(self.cfg.approach_q15)
            ),
            _ => format!(
                "leave the {} stop at {:.3} V, without the permit, until {LEAVE_COUNTS} counts off it",
                last.stop.word(),
                self.v(self.cfg.approach_q15)
            ),
        });
        self.leave(last.stop, last.pos, end, None, t, pos)
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

/// A pot that read no number.
fn dead(pos: f64) -> String {
    format!(
        "the pot reads {pos}, no number: a dead or slipped pot. Nothing more drives; {HANDS}; if it is refused again, check the pot's wiring"
    )
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
    /// counts/s a volt over `breakaway_v`; a jam at `jam` stops it going
    /// down at any duty; a soft low stop `soft` (contact, blind, volts)
    /// holds it at its contact under the volts and carries it on to the
    /// blind end at or over them; the low stop moves `walk` counts after
    /// its first seat, and with `freeze` the shaft never moves once it
    /// meets the walked stop; a dead pot reads `dead` whatever the shaft
    /// does; once the shaft has met the high stop, the first read at or
    /// under `spike.0` reads `spike.1` off it, that one read alone; the pot
    /// reads with a +/-1 count wobble.
    struct Shaft {
        pos: f64,
        lo: f64,
        hi: f64,
        cps_per_v: f64,
        breakaway_v: f64,
        jam: Option<f64>,
        soft: Option<(f64, f64, f64)>,
        walk: f64,
        freeze: bool,
        dead: Option<f64>,
        spike: Option<(f64, f64)>,
        high_met: bool,
        spiked: bool,
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
                jam: None,
                soft: None,
                walk: 0.0,
                freeze: false,
                dead: None,
                spike: None,
                high_met: false,
                spiked: false,
                k: 0,
                seats_low: 0,
            }
        }

        fn drive(&mut self, c: &SeekCfg, q15: i16, s: f64) {
            let v = c.volts(q15);
            let dir = (q15 as f64).signum();
            if v < self.breakaway_v || q15 == 0 {
                return;
            }
            if self.freeze && self.seats_low >= 1 && self.pos <= self.lo + 0.5 {
                return;
            }
            let mut to = self.pos + dir * self.cps_per_v * (v - self.breakaway_v).max(0.05) * s;
            if let Some(j) = self.jam.filter(|j| dir < 0.0 && self.pos >= *j) {
                to = to.max(j);
            }
            if let Some((contact, _, push_v)) = self.soft
                && dir < 0.0
                && v < push_v
                && self.pos >= contact
            {
                to = to.max(contact);
            }
            self.pos = to.clamp(self.lo, self.hi);
            self.high_met |= self.pos >= self.hi;
        }

        fn read(&mut self) -> f64 {
            self.k += 1;
            let wobble = if self.k % 2 == 0 { 1.0 } else { -1.0 };
            if let Some((at, by)) = self.spike
                && self.high_met
                && !self.spiked
                && self.pos <= at
            {
                self.spiked = true;
                return self.pos + by + wobble;
            }
            self.dead.unwrap_or(self.pos + wobble)
        }
    }

    /// Every act of a run, the duty and the leg it drove in.
    struct Ran {
        m: Seek,
        duties: Vec<i16>,
        last: Act,
    }

    /// The machine run on `shaft` to its end, polled every POLL_S; each
    /// seat's bursts take a second, then the stopper walks if it does.
    fn run(shaft: &mut Shaft) -> Ran {
        let c = cfg();
        let mut m = Seek::new(c.clone());
        let mut t = 0.0;
        let mut duties = Vec::new();
        let mut act = m.start(t, shaft.read());
        for _ in 0..50_000 {
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
                Act::Done | Act::Stopped { .. } => {
                    return Ran {
                        m,
                        duties,
                        last: act,
                    };
                }
            }
        }
        panic!("never ends: {:?}", m.path());
    }

    fn refusal(m: &Seek) -> Refusal {
        m.refused()
            .map(|r| r.0)
            .unwrap_or_else(|| panic!("not refused: {:?}", m.path()))
    }

    fn seats(m: &Seek, s: Stop) -> Vec<f64> {
        m.seats()
            .iter()
            .filter(|x| x.stop == s)
            .map(|x| x.pos)
            .collect()
    }

    /// No approach duty is over the approach's: every press toward a stop
    /// is at the approach duty or the seat's.
    fn presses_at_most_the_approach(r: &Ran) {
        let c = cfg();
        for l in r.m.legs().iter().filter(|l| l.kind == LegKind::Approach) {
            assert_eq!(l.top_q15, c.approach_q15, "{l:?}");
        }
        assert!(
            r.duties
                .iter()
                .all(|q| q.unsigned_abs() <= c.ceiling_q15 as u16)
        );
    }

    #[test]
    fn six_seats_alternate_the_stops_and_end_at_the_middle() {
        let mut s = Shaft::bench();
        let r = run(&mut s);
        let m = &r.m;
        assert!(m.finished(), "{:?}", m.path());
        assert_eq!(m.approaches(), 6);
        let order: Vec<Stop> = m.seats().iter().map(|x| x.stop).collect();
        use Stop::{High, Low};
        assert_eq!(order, [Low, High, Low, High, Low, High]);
        for (stop, want) in [(Stop::Low, 432.0), (Stop::High, 3626.0)] {
            assert!(
                seats(m, stop).iter().all(|p| (p - want).abs() <= 1.0),
                "{:?}",
                m.seats()
            );
        }
        assert!((s.pos - 2029.0).abs() < 100.0, "near the middle: {}", s.pos);
        presses_at_most_the_approach(&r);
        assert!(m.leave_raises().is_empty());
        let kinds: Vec<LegKind> = m.legs().iter().map(|l| l.kind).collect();
        use LegKind::*;
        assert_eq!(
            kinds,
            [
                Approach, Leave, Approach, Leave, Approach, Leave, Approach, Leave, Approach,
                Leave, Approach, Center
            ]
        );
    }

    /// The bench servo's low stop: first contact at 108, and any press at
    /// the ceiling carries the shaft on to a blind 4. Never raised, the
    /// approach seats at 108 every time.
    #[test]
    fn a_soft_stop_is_seated_at_its_contact() {
        let c = cfg();
        let mut s = Shaft {
            lo: 4.0,
            soft: Some((108.0, 4.0, c.volts(c.ceiling_q15))),
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert!(r.m.finished(), "{:?}", r.m.path());
        let low = seats(&r.m, Stop::Low);
        assert_eq!(low.len(), SEATS_A_STOP);
        assert!(low.iter().all(|p| (p - 108.0).abs() <= 1.0), "{low:?}");
        presses_at_most_the_approach(&r);
        assert!(
            !r.m.path().iter().any(|p| p.contains("raised")),
            "{:?}",
            r.m.path()
        );
    }

    /// A start resting at the low stop: the first approach goes still with
    /// no travel, takes no seat, and leaves; three good seats a stop follow,
    /// the high stop first. A start 300 counts above the low stop travels
    /// enough and seats there first.
    #[test]
    fn a_start_at_a_stop_takes_no_seat_there() {
        let mut s = Shaft {
            pos: 432.0,
            ..Shaft::bench()
        };
        let r = run(&mut s);
        let m = &r.m;
        assert!(m.finished(), "{:?}", m.path());
        assert_eq!(m.approaches(), APPROACHES_MAX);
        let order: Vec<Stop> = m.seats().iter().map(|x| x.stop).collect();
        use Stop::{High, Low};
        assert_eq!(order, [High, Low, High, Low, High, Low]);
        assert!(seats(m, Stop::Low).iter().all(|p| (p - 432.0).abs() <= 1.0));
        assert!(
            m.path().iter().any(|p| p.contains("no seat there")),
            "{:?}",
            m.path()
        );
        assert_eq!(m.legs()[0].kind, LegKind::Approach);
        assert_eq!(m.legs()[1].kind, LegKind::Leave);
        presses_at_most_the_approach(&r);

        let mut s = Shaft {
            pos: 732.0,
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert!(r.m.finished(), "{:?}", r.m.path());
        assert_eq!(r.m.approaches(), 6);
        assert_eq!(r.m.seats()[0].stop, Stop::Low);
        assert!(
            seats(&r.m, Stop::Low)
                .iter()
                .all(|p| (p - 432.0).abs() <= 1.0)
        );
    }

    /// A later approach that goes still short of the travel a seat needs:
    /// the shaft is blocked between the stops, and the step refuses at
    /// once, the shaft off any seat.
    #[test]
    fn a_later_approach_blocked_short_refuses() {
        // the leave from the high stop ends at 3476; a jam at 3420 holds
        // the next approach down after 56 counts
        let mut s = Shaft {
            jam: Some(3420.0),
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert_eq!(refusal(&r.m), Refusal::Blocked, "{:?}", r.m.path());
        let why = &r.m.refused().expect("refused").1;
        assert!(
            why.contains("blocked between the stops") && why.contains(HANDS),
            "{why}"
        );
        assert!(!why.contains("driven off"), "no seat by it: {why}");
        assert_eq!(r.m.seats().len(), 2);
        assert!((s.pos - 3420.0).abs() <= 1.0);
    }

    /// A stall short of the stop after real travel seats there; the
    /// acceptance checks judge it.
    #[test]
    fn a_stall_short_of_the_stop_seats_there() {
        let mut s = Shaft {
            jam: Some(1900.0),
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert!(r.m.finished(), "{:?}", r.m.path());
        let low = seats(&r.m, Stop::Low);
        assert!(low.iter().all(|p| (p - 1900.0).abs() <= 1.0), "{low:?}");
    }

    /// A stopper that walks 15 counts: seat 3 refuses, all pressing ends,
    /// and the horn is first driven off the stop to the middle.
    #[test]
    fn a_stopper_that_walks_refuses_off_the_stop() {
        let mut s = Shaft {
            walk: -15.0,
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert_eq!(refusal(&r.m), Refusal::Walked);
        assert_eq!(r.m.seats().len(), 3);
        let why = &r.m.refused().expect("refused").1;
        assert!(
            why.contains("-15 counts")
                && why.contains("All pressing ends")
                && why.contains("driven off the low stop first"),
            "{why}"
        );
        let mid = (417.0 + 432.0) / 2.0 / 2.0 + 3626.0 / 2.0;
        assert!(
            (s.pos - mid).abs() < 50.0,
            "near the middle: {} against {mid}",
            s.pos
        );
        let last = r.m.legs().last().expect("legs");
        assert_eq!(last.kind, LegKind::Center);
        assert!(matches!(
            r.last,
            Act::Stopped {
                kind: Refusal::Walked,
                ..
            }
        ));
        presses_at_most_the_approach(&r);

        // the same, the shaft stuck at the stop: the leave off it fails,
        // and the refusal says the horn was left there
        let mut s = Shaft {
            walk: -15.0,
            freeze: true,
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert_eq!(refusal(&r.m), Refusal::Walked);
        let why = &r.m.refused().expect("refused").1;
        assert!(why.contains("the horn was left at the low stop"), "{why}");
        assert!((s.pos - 417.0).abs() <= 1.0);
    }

    #[test]
    fn a_leg_over_its_time_is_refused() {
        // 1568 counts at about 50 counts/s: over 30 s
        let c = cfg();
        let mut s = Shaft {
            cps_per_v: 50.0 / (c.volts(c.approach_q15) - 0.4),
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert_eq!(refusal(&r.m), Refusal::TimeOut);
        assert!(r.m.seats().is_empty());
        assert!(r.m.refused().expect("refused").1.contains(HANDS));
    }

    /// A pot stuck at one reading looks like a start at the low stop: no
    /// seat, then no duty leaves it; a pot that reads no number refuses at
    /// once.
    #[test]
    fn a_dead_pot_is_refused() {
        let mut s = Shaft {
            dead: Some(2000.0),
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert_eq!(refusal(&r.m), Refusal::Blocked, "{:?}", r.m.path());
        assert!(r.m.seats().is_empty());
        let c = cfg();
        assert!(
            r.duties
                .iter()
                .all(|q| q.unsigned_abs() <= c.ceiling_q15 as u16)
        );
        assert!(r.m.refused().expect("refused").1.contains("dead pot"));
        assert!(
            !r.m.leave_raises().is_empty(),
            "the leave rose to the ceiling"
        );

        let mut s = Shaft {
            dead: Some(f64::NAN),
            ..Shaft::bench()
        };
        let r = run(&mut s);
        assert_eq!(refusal(&r.m), Refusal::DeadPot);
        assert!(r.duties.is_empty(), "nothing drives");
    }

    /// One poll reading 400 counts the wrong way on the leave from the
    /// high stop, the next back inside: the run goes on, never ends the
    /// leave on that read, and counts it.
    #[test]
    fn a_single_wrong_way_read_on_a_leave_is_survived_and_counted() {
        let mut s = Shaft {
            spike: Some((3560.0, 400.0)),
            ..Shaft::bench()
        };
        let r = run(&mut s);
        let m = &r.m;
        assert!(s.spiked);
        assert!(m.finished(), "{:?}", m.path());
        assert_eq!(m.seats().len(), 2 * SEATS_A_STOP);
        let odd = m.odd_reads();
        assert_eq!(odd.len(), 1, "{odd:?}");
        let o = &odd[0];
        assert_eq!((o.leg, o.kind, o.toward), (4, LegKind::Leave, Stop::Low));
        assert!((o.read - o.before - 400.0).abs() < 30.0, "{o:?}");
        // two poll periods of travel at the approach duty, about 33 counts
        assert!((o.before - o.after - 33.0).abs() < 5.0, "{o:?}");
        let leave = &m.legs()[3];
        assert!(
            (leave.to - (3626.0 - LEAVE_COUNTS)).abs() < 30.0,
            "the leave ends on a believed read: {leave:?}"
        );
        assert!(
            m.path().iter().any(|p| p.contains("set aside")),
            "{:?}",
            m.path()
        );
    }

    /// A shaft that runs the other way from the duty: the sign is not the
    /// proven one, and the step aborts on the second poll over the line.
    #[test]
    fn a_shaft_that_runs_the_wrong_way_aborts() {
        let c = cfg();
        let mut m = Seek::new(c.clone());
        let mut s = Shaft::bench();
        let mut t = 0.0;
        let from = s.read();
        let mut act = m.start(t, from);
        let (mut best, mut first_over, mut polls) = (0.0f64, None, 0);
        while let Act::Drive { q15, .. } = act {
            s.drive(&c, -q15, POLL_S);
            t += POLL_S;
            let pos = s.read();
            polls += 1;
            let along = from - pos;
            best = best.max(along);
            if first_over.is_none() && best - along > WRONG_WAY_COUNTS {
                first_over = Some(polls);
            }
            act = m.feed(t, pos);
        }
        assert_eq!(
            Some(polls - 1),
            first_over,
            "one poll after the first over the line"
        );
        assert!(m.odd_reads().is_empty());
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
