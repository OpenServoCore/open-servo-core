//! The jam check: the drive direction and whether the shaft moves at all,
//! from short pulses each way; then away from the nearer stop, toward mid
//! travel; then into the band there, and torque off.
//!
//! The probe drives pulses of [`PULSE_POLLS`] polls with a rest at zero
//! between them, alternately each way, from the shunt floor's volts up by
//! [`PROBE_STEP_MV`] a pair. A pulse that moves the pot
//! [`MOVED_COUNTS`] ends at once: which way it went against the duty is
//! the polarity, and the next pulse, the other way, confirms it by moving
//! the other way or stalling. A pulse the endstop would refuse is never
//! written, and one it refuses mid-way tells nothing. A pulse the pot sees
//! still is felt ([`sag::feel`]): the ladder ends stiff once the next rung
//! would stall over the probe's cap or press a stop past the class's
//! budget, at the motor when nothing flows by the backstop, and at the pot
//! when the motor turns and the pot does not.
//!
//! The polarity a pulse shows goes to the servo at once, in RAM: past a
//! soft limit the endstop judges a duty by its sign, and only the true
//! polarity lets the drive back in. A servo that latches drive_reversed is
//! resumed once ([`JamCheck::latched`]): at rest, flipped, acknowledged,
//! and its next moving pulse must go the duty's way.
//!
//! Toward mid travel the drive runs a rung over the one that moved the
//! shaft, raises a rung a still window until it has left the stop by
//! [`SEEK_TRAVEL_MIN`], up to the class's stop hold, and drops one while
//! faster than [`PROVEN_CPS_MAX`]. It ends in the band. Directions are the
//! pot's: up is +1.

use std::collections::VecDeque;

use super::centre::POT_MID;
use super::sag::{self, Felt};
use super::seek::{self, SEEK_TRAVEL_MIN};
use super::{
    AbortReason, Cmd, Experiment, Guarded, LIMIT_ENDSTOP, LIMIT_GOVERNING, RigParams, limit_holds,
};
use crate::exp::rl::Scales;
use crate::frame::TelemetrySnapshot;
use crate::limits::{GearClass, ServoLimits};
use crate::regs::{Reg, config, control};
use crate::window::Watch;
use osc_servo_core::kernel::phases::loops::faults;

/// Pot travel a pulse must make to count as moved: four times a poll's 3
/// counts of noise.
pub const MOVED_COUNTS: u16 = 12;
/// A pulse, and the least rest after one, in polls: 40 and 40 ms asked, a
/// pulse 55 ms on the bus with its reads, under the 60 ms a press may
/// last. Its first poll reads the early windows, its second the sag. After
/// a pulse the pot saw still nothing coasts, and one poll at zero will do.
pub const PULSE_POLLS: u32 = 2;
pub const REST_POLLS: u32 = 2;
pub const STILL_REST_POLLS: u32 = 1;
/// A shaft that moves at zero this many polls on is turned by a load.
pub const REST_POLLS_MAX: u32 = 10;
/// The fastest a drive may run before the direction is proven, and after,
/// counts/s: torque off coasts the shaft about 23 counts per count/ms, and
/// the bench MG90's soft limits sit 220 counts inside its stops.
pub const PROBE_CPS_MAX: f64 = 3000.0;
pub const PROVEN_CPS_MAX: f64 = 9000.0;

/// The probe's rung step and its backstop, mV: 3% of a 7.4 V rail a step,
/// 5.4% of 3.7 V. The backstop is reached only while the current reads low.
pub const PROBE_STEP_MV: f64 = 200.0;
pub const PROBE_MAX_MV: f64 = 2500.0;

/// The probe's current cap over the class's stop hold: 1.5x over the
/// breakaway of the families the classes cover (MG90 <= 245 mA, SG90 <=
/// 160 mA).
pub const PROBE_OF_HOLD: f64 = 0.75;

/// Under this a drive draws nothing, mA: an open winding reads only the
/// shunt's noise, a few mA, and the class's windings (3 to 7 ohm) draw
/// over 70 mA at the lowest rung of a 3.7 V rail.
pub const OPEN_MA: f64 = 30.0;

#[derive(Clone, Debug)]
pub struct JamCfg {
    pub class: GearClass,
    pub mv_per_vcount: f64,
    pub ma_per_ccount: f64,
    /// The rung step and the backstop, vcounts.
    pub step: f64,
    pub v_max: f64,
    /// The probe's cap and the stop hold, both at most the servo's limit,
    /// and the open-winding floor: ccounts.
    pub i_probe: f64,
    pub i_hold: f64,
    pub i_open: f64,
    /// Winding R, vcounts per ccount, when the servo carries one.
    pub r_vpc: Option<f64>,
    pub press_budget_ms: f64,
    pub poll_ms: u32,
    /// Half-width of the band around mid travel the check ends in, counts.
    pub margin: u16,
    /// Poll budget: 400 x 20 ms is 8 s.
    pub polls_max: u32,
}

impl JamCfg {
    /// `i_lim`: the servo's current limit, ccounts.
    pub fn new(
        class: GearClass,
        mv_per_vcount: f64,
        ma_per_ccount: f64,
        i_lim: u16,
        r_vpc: Option<f64>,
    ) -> Self {
        let ccounts = |ma: f64| (ma / ma_per_ccount).min(i_lim as f64);
        Self {
            class,
            mv_per_vcount,
            ma_per_ccount,
            step: PROBE_STEP_MV / mv_per_vcount,
            v_max: PROBE_MAX_MV / mv_per_vcount,
            i_probe: ccounts(PROBE_OF_HOLD * class.hold_ma()),
            i_hold: ccounts(class.hold_ma()),
            i_open: OPEN_MA / ma_per_ccount,
            r_vpc,
            press_budget_ms: class.press_budget_ms(),
            poll_ms: 20,
            margin: 300,
            polls_max: 400,
        }
    }

    pub fn for_servo(lim: &ServoLimits, sc: &Scales, class: GearClass) -> Self {
        Self::new(
            class,
            sc.v_term_per_count * 1000.0,
            sc.amps_per_count * 1000.0,
            lim.i_lim,
            lim.r_vpc(),
        )
    }

    pub fn ma(&self, ccounts: f64) -> f64 {
        ccounts * self.ma_per_ccount
    }

    pub fn mv(&self, vcounts: f64) -> f64 {
        vcounts * self.mv_per_vcount
    }
}

/// What one pulse showed.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Seen {
    /// The pot moved, this way.
    Moved(i8),
    Felt(Felt),
    /// The endstop zeroed it.
    Refused,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Next {
    Pulse(i8),
    Toward,
}

#[derive(Copy, Clone, Debug)]
enum Leg {
    /// Zero duty for `least` polls and until the shaft rests, then `then`.
    Rest {
        polls: u32,
        least: u32,
        last: u16,
        then: Next,
    },
    /// A pulse of `sign` from `from`; `at` is the last read's time and pos.
    Pulse {
        sign: i8,
        from: u16,
        polls: u32,
        at: (f64, u16),
    },
    Toward {
        from: u16,
        dir: i8,
        at: (f64, u16),
    },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Phase {
    Start,
    Look,
    Wait,
    Read,
    Eval,
    Finished,
}

pub struct JamCheck {
    cfg: JamCfg,
    params: RigParams,
    mid: u16,
    band: (u16, u16),
    phase: Phase,
    leg: Leg,
    writes: VecDeque<(Reg, i32)>,
    still: Option<seek::Watch>,
    polls: u32,
    /// `drive_polarity` as the servo runs it now.
    polarity: bool,
    flips: u32,
    /// The latch's flip waits for the next moving pulse to bear it out.
    after_latch: bool,
    /// The first rung and this one, vcounts, and what each sign's pulse
    /// at it showed.
    v0: f64,
    v: f64,
    rung: [Option<Seen>; 2],
    /// +1 when a positive duty raised the counts under `polarity`.
    along: Option<i8>,
    /// The volts the first moving pulse ran at.
    moved_v: Option<f64>,
    vbus: f64,
    /// The pulse's first and latest watch currents, counts.
    watches: u32,
    early: Option<f64>,
    late: Option<f64>,
    /// Over every still pulse: stall counts per vcount, the most current,
    /// how many were felt and how many drew nothing.
    ohm: Option<f64>,
    i_still: f64,
    felt: u32,
    dead: u32,
    turning: u32,
    /// Time pressed, by the longest still pulse of each rung at a stop (at
    /// any, the stops unknown), the longest of this one, and the longest
    /// pulse yet: ms, measured.
    press_ms: f64,
    rung_press: f64,
    pulse_ms: f64,
    /// When the pulse started, host ms.
    t0: f64,
    /// Where the pulse started, and the most a pulse moved the shaft by
    /// the end of its rest, counts.
    from: u16,
    travel_max: u16,
    arrived: bool,
    halt: Option<AbortReason>,
}

impl JamCheck {
    pub fn new(cfg: JamCfg, params: &RigParams) -> Self {
        let mid = params.pos_guard.map_or(POT_MID, |(lo, hi)| (lo + hi) / 2);
        let band = (
            mid.saturating_sub(cfg.margin),
            mid.saturating_add(cfg.margin),
        );
        Self {
            cfg,
            params: *params,
            mid,
            band,
            phase: Phase::Start,
            leg: Leg::Rest {
                polls: 0,
                least: REST_POLLS,
                last: 0,
                then: Next::Toward,
            },
            writes: VecDeque::new(),
            still: None,
            polls: 0,
            polarity: params.drive_polarity,
            flips: 0,
            after_latch: false,
            v0: 0.0,
            v: 0.0,
            rung: [None; 2],
            along: None,
            moved_v: None,
            vbus: 0.0,
            watches: 0,
            early: None,
            late: None,
            ohm: None,
            i_still: 0.0,
            felt: 0,
            dead: 0,
            turning: 0,
            press_ms: 0.0,
            rung_press: 0.0,
            pulse_ms: 0.0,
            t0: 0.0,
            from: 0,
            travel_max: 0,
            arrived: false,
            halt: None,
        }
    }

    /// The shaft ended in the band at mid travel.
    pub fn arrived(&self) -> bool {
        self.arrived
    }

    /// The `drive_polarity` the servo runs in RAM now.
    pub fn polarity(&self) -> bool {
        self.polarity
    }

    /// The duty the first moving pulse ran at, a fraction of full scale:
    /// the probe's estimate of the breakaway.
    pub fn moved_at(&self) -> Option<f64> {
        let vbus = self.vbus;
        self.moved_v
            .filter(|_| vbus > 0.0)
            .map(|v| (v / vbus).min(1.0))
    }

    pub fn band(&self) -> (u16, u16) {
        self.band
    }

    /// The servo latched drive_reversed and the check went on flipped.
    pub fn resumed(&self) -> bool {
        self.after_latch
    }

    /// The most one probe pulse moved the shaft, its stop included, counts.
    pub fn pulse_travel_max(&self) -> u16 {
        self.travel_max
    }

    /// The servo latched drive_reversed and the shaft came to rest at
    /// `pos`, torque off: go on, once, flipped and acknowledged. False once
    /// the polarity flipped before.
    pub fn latched(&mut self, pos: u16) -> bool {
        if self.flips > 0 {
            return false;
        }
        self.after_latch = true;
        self.travel_max = self.travel_max.max(pos.abs_diff(self.from));
        self.along = None;
        self.moved_v = self.moved_v.or(Some(self.v));
        self.rung = [None; 2];
        self.writes.clear();
        self.flip();
        self.writes.push_back((control::GOAL_DUTY, 0));
        self.writes.push_back((control::TORQUE_ENABLE, 1));
        let sign = self.toward_mid(pos);
        self.resting(pos, REST_POLLS, Next::Pulse(sign));
        true
    }

    fn in_band(&self, pos: u16) -> bool {
        (self.band.0..=self.band.1).contains(&pos)
    }

    fn toward_mid(&self, pos: u16) -> i8 {
        if pos < self.mid { 1 } else { -1 }
    }

    /// The endstop zeroes a drive of `sign` here.
    fn walled(&self, sign: i8, o: &TelemetrySnapshot) -> bool {
        o.limit_flags & LIMIT_ENDSTOP != 0 && (sign > 0) == (o.pos > self.mid)
    }

    /// Drive `sign` at the rung's volts; nothing on a rail that reads 0.
    fn goal(&mut self, sign: i8) {
        let q15 = if self.vbus > 0.0 {
            (self.v / self.vbus * 32767.0).round().min(32767.0)
        } else {
            0.0
        };
        self.writes
            .push_back((control::GOAL_DUTY, sign as i32 * q15 as i32));
    }

    fn flip(&mut self) {
        self.polarity = !self.polarity;
        self.flips += 1;
        self.writes
            .push_back((config::DRIVE_POLARITY, self.polarity as i32));
    }

    /// The volts a stall draws `i` at: by the stored R, else scaled from
    /// the still pulses felt so far.
    fn volts_for(&self, i: f64) -> f64 {
        match (self.cfg.r_vpc, self.ohm) {
            (Some(r), _) => i * r,
            (None, Some(g)) if g > 0.0 => i / g,
            _ => f64::INFINITY,
        }
    }

    fn wait(&mut self) -> Cmd {
        self.phase = Phase::Wait;
        self.step(None)
    }

    fn finish(&mut self, arrived: bool) -> Cmd {
        self.arrived = arrived;
        self.writes.push_back((control::GOAL_DUTY, 0));
        self.writes.push_back((control::TORQUE_ENABLE, 0));
        self.phase = Phase::Finished;
        self.step(None)
    }

    fn halt(&mut self, reason: AbortReason) -> Cmd {
        self.halt = Some(reason);
        self.finish(false)
    }

    fn rest(&mut self, pos: u16, least: u32, then: Next) -> Cmd {
        self.writes.push_back((control::GOAL_DUTY, 0));
        self.resting(pos, least, then);
        self.wait()
    }

    fn resting(&mut self, pos: u16, least: u32, then: Next) {
        self.leg = Leg::Rest {
            polls: 0,
            least,
            last: pos,
            then,
        };
        self.phase = Phase::Wait;
    }

    fn eval(&mut self, o: &TelemetrySnapshot) -> Cmd {
        self.polls += 1;
        if self.polls > self.cfg.polls_max {
            return self.finish(false);
        }
        self.vbus = o.vbus_counts as f64;
        match self.leg {
            Leg::Rest {
                polls,
                least,
                last,
                then,
            } => {
                let polls = polls + 1;
                let moving = o.pos.abs_diff(last) > self.params.stall_eps;
                if polls < least || (moving && polls < REST_POLLS_MAX) {
                    self.leg = Leg::Rest {
                        polls,
                        least,
                        last: o.pos,
                        then,
                    };
                    return self.wait();
                }
                if moving {
                    return self.halt(seek::blocked(last, o.pos));
                }
                self.travel_max = self.travel_max.max(o.pos.abs_diff(self.from));
                self.next(o, then)
            }
            Leg::Pulse {
                sign,
                from,
                polls,
                at,
            } => self.pulse(o, sign, from, polls + 1, at),
            Leg::Toward { from, dir, at } => self.toward(o, from, dir, at),
        }
    }

    /// At rest: the next pulse, or the drive toward mid travel.
    fn next(&mut self, o: &TelemetrySnapshot, then: Next) -> Cmd {
        let sign = match then {
            Next::Toward => return self.start_toward(o),
            Next::Pulse(sign) => sign,
        };
        if self.walled(sign, o) {
            return match self.after(sign, Seen::Refused, o.pos) {
                Ok(then) => self.next(o, then),
                Err(why) => self.halt(why),
            };
        }
        self.goal(sign);
        (self.from, self.t0) = (o.pos, o.host_ms);
        (self.watches, self.early, self.late) = (0, None, None);
        self.leg = Leg::Pulse {
            sign,
            from: o.pos,
            polls: 0,
            at: (o.host_ms, o.pos),
        };
        self.wait()
    }

    fn pulse(
        &mut self,
        o: &TelemetrySnapshot,
        sign: i8,
        from: u16,
        polls: u32,
        at: (f64, u16),
    ) -> Cmd {
        let moved = o.pos as i32 - from as i32;
        let seen = if moved.unsigned_abs() >= MOVED_COUNTS as u32 {
            Seen::Moved(moved.signum() as i8)
        } else if self.walled(sign, o) {
            Seen::Refused
        } else if polls >= PULSE_POLLS {
            Seen::Felt(self.feel(o))
        } else {
            self.leg = Leg::Pulse {
                sign,
                from,
                polls,
                at: (o.host_ms, o.pos),
            };
            return self.wait();
        };
        let ms = o.host_ms - self.t0;
        self.pulse_ms = self.pulse_ms.max(ms);
        let at_stop = self
            .params
            .stops
            .is_none_or(|(lo, hi)| o.pos.abs_diff(lo).min(o.pos.abs_diff(hi)) <= seek::STOP_TOL);
        if matches!(seen, Seen::Felt(_)) && at_stop {
            self.rung_press = self.rung_press.max(ms);
        }
        let then = match self.after(sign, seen, o.pos) {
            Ok(then) => then,
            Err(why) => return self.halt(why),
        };
        if speed(o, at) > PROBE_CPS_MAX {
            self.v = (self.v - self.cfg.step).max(self.v0);
        }
        let least = match seen {
            Seen::Felt(_) => STILL_REST_POLLS,
            _ => REST_POLLS,
        };
        self.rest(o.pos, least, then)
    }

    fn feel(&mut self, o: &TelemetrySnapshot) -> Felt {
        let late = self.late.unwrap_or(o.i_mean_counts.unsigned_abs() as f64);
        let ohmic = self.cfg.r_vpc.map(|r| self.v / r);
        let pinned = o.limit_flags & LIMIT_GOVERNING != 0;
        let felt = sag::feel(late, self.early, ohmic, pinned, self.cfg.i_open);
        self.felt += 1;
        self.i_still = self.i_still.max(late);
        match felt {
            Felt::Stalled => self.ohm = Some(self.ohm.unwrap_or(0.0).max(late / self.v)),
            Felt::Dead => self.dead += 1,
            Felt::Turning => self.turning += 1,
            Felt::Unclear => {}
        }
        felt
    }

    /// What a pulse of `sign` showing `seen` leads to.
    fn after(&mut self, sign: i8, seen: Seen, pos: u16) -> Result<Next, AbortReason> {
        self.rung[(sign > 0) as usize] = Some(seen);
        if let Seen::Moved(dir) = seen {
            let along = dir * sign;
            self.moved_v = self.moved_v.or(Some(self.v));
            return match self.along {
                _ if self.after_latch && along < 0 => Err(seek::blocked(pos, pos)),
                _ if self.after_latch => {
                    self.along = Some(1);
                    Ok(Next::Toward)
                }
                None => {
                    self.along = Some(along);
                    Ok(Next::Pulse(-sign))
                }
                Some(a) if a != along => Err(seek::blocked(pos, pos)),
                Some(_) => Ok(Next::Toward),
            };
        }
        if self.along.is_some() {
            return Ok(Next::Toward);
        }
        if self.turning >= 2 {
            return Err(AbortReason::SensorDead { pos });
        }
        if self.rung[(sign < 0) as usize].is_none() {
            return Ok(Next::Pulse(-sign));
        }
        self.raise(pos)
    }

    /// Both ways tried at this rung: the next one, or the ladder's end.
    fn raise(&mut self, pos: u16) -> Result<Next, AbortReason> {
        self.press_ms += self.rung_press;
        self.rung_press = 0.0;
        let top = self.volts_for(self.cfg.i_probe).min(self.cfg.v_max);
        let v = (self.v + self.cfg.step).min(top);
        if v <= self.v || self.press_ms + self.pulse_ms > self.cfg.press_budget_ms {
            if self.turning > 0 {
                return Err(AbortReason::SensorDead { pos });
            }
            if self.felt > 0 && self.dead == self.felt {
                return Err(AbortReason::MotorOpen { pos });
            }
            let ma = self.cfg.ma(self.i_still).round().min(u16::MAX as f64) as u16;
            return Err(AbortReason::Stiff { pos, ma });
        }
        self.v = v;
        self.rung = [None; 2];
        Ok(Next::Pulse(self.toward_mid(pos)))
    }

    fn start_toward(&mut self, o: &TelemetrySnapshot) -> Cmd {
        if self.along == Some(-1) {
            self.flip();
        }
        let dir = self.toward_mid(o.pos);
        self.v = self.moved_v.unwrap_or(self.v) + self.cfg.step;
        self.goal(dir);
        let (eps, polls) = (self.params.stall_eps, self.params.stall_polls);
        self.still = Some(seek::Watch::new(o.pos, eps, polls));
        self.leg = Leg::Toward {
            from: o.pos,
            dir,
            at: (o.host_ms, o.pos),
        };
        self.wait()
    }

    fn toward(&mut self, o: &TelemetrySnapshot, from: u16, dir: i8, at: (f64, u16)) -> Cmd {
        let went = (o.pos as i32 - from as i32) * dir as i32;
        if went <= -(MOVED_COUNTS as i32) {
            return self.halt(seek::blocked(from, o.pos));
        }
        let travelled = went >= SEEK_TRAVEL_MIN as i32;
        if travelled && self.in_band(o.pos) {
            return self.finish(true);
        }
        let still = self.still.as_mut().is_some_and(|w| w.still(o.pos));
        if limit_holds(o, still) || (still && travelled) {
            return self.halt(seek::blocked(from, o.pos));
        }
        let floor = self.moved_v.unwrap_or(self.v0);
        let v = if still {
            self.v + self.cfg.step
        } else if speed(o, at) > PROVEN_CPS_MAX {
            (self.v - self.cfg.step).max(floor)
        } else {
            self.v
        };
        if v > self.v {
            let late = self.late.unwrap_or(o.i_mean_counts.unsigned_abs() as f64);
            let stall = match self.cfg.r_vpc {
                Some(r) => v / r,
                None => late * v / self.v,
            };
            if stall > self.cfg.i_hold {
                return self.halt(seek::blocked(from, o.pos));
            }
        }
        if v != self.v {
            self.v = v;
            self.goal(dir);
        }
        self.leg = Leg::Toward {
            from,
            dir,
            at: (o.host_ms, o.pos),
        };
        self.wait()
    }
}

/// Pump `exp` to its end with `pump`, through one drive_reversed latch:
/// the check, and the abort that ended it. Run outside the travel guard:
/// the shaft may start outside it. After a latch only reads watch the shaft
/// until it rests, torque off: the envelope would end on the latch they
/// see.
pub fn drive<E>(
    mut exp: JamCheck,
    params: &RigParams,
    mut pump: impl FnMut(&mut dyn Experiment) -> Result<(), E>,
) -> Result<(JamCheck, Option<AbortReason>), E> {
    loop {
        let mut g = Guarded::new(exp, params.without_pos_guard());
        pump(&mut g)?;
        let abort = g.abort();
        exp = g.into_inner();
        let Some(AbortReason::Fault {
            code: faults::CODE_DRIVE_REVERSED,
            ..
        }) = abort
        else {
            return Ok((exp, abort));
        };
        let mut rest = Settle::new(params, exp.cfg.poll_ms, exp.cfg.polls_max);
        pump(&mut rest)?;
        match rest.at {
            Some(pos) if exp.latched(pos) => {}
            _ => return Ok((exp, abort)),
        }
    }
}

/// Reads alone, torque off, until the shaft rests: `at`, where it rests.
struct Settle {
    still: Option<seek::Watch>,
    eps: u16,
    window: u32,
    poll_ms: u32,
    polls: u32,
    at: Option<u16>,
}

impl Settle {
    fn new(params: &RigParams, poll_ms: u32, polls: u32) -> Self {
        Self {
            still: None,
            eps: params.stall_eps,
            window: params.stall_polls,
            poll_ms,
            polls,
            at: None,
        }
    }
}

impl Experiment for Settle {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        let Some(o) = obs else {
            return Cmd::Read;
        };
        let (eps, window) = (self.eps, self.window);
        let still = self
            .still
            .get_or_insert(seek::Watch::new(o.pos, eps, window));
        if still.still(o.pos) {
            self.at = Some(o.pos);
            return Cmd::Done;
        }
        self.polls = self.polls.saturating_sub(1);
        if self.polls == 0 {
            return Cmd::Done;
        }
        Cmd::Pause { ms: self.poll_ms }
    }
}

/// Counts/s between the read at `at` and `o`.
fn speed(o: &TelemetrySnapshot, at: (f64, u16)) -> f64 {
    let dt = o.host_ms - at.0;
    if dt <= 0.0 {
        return 0.0;
    }
    o.pos.abs_diff(at.1) as f64 * 1000.0 / dt
}

impl Experiment for JamCheck {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        if let Some((reg, value)) = self.writes.pop_front() {
            return Cmd::Write { reg, value };
        }
        match self.phase {
            Phase::Start => {
                self.phase = Phase::Look;
                Cmd::Read
            }
            Phase::Look => {
                let Some(o) = obs else {
                    self.phase = Phase::Finished;
                    return Cmd::Done;
                };
                self.vbus = o.vbus_counts as f64;
                let floor = match o.window_floor_q15 {
                    0 => self.params.window_floor_q15,
                    f => f,
                };
                self.v0 = (floor as f64 / 32767.0 * self.vbus).max(self.cfg.step);
                self.v = self.v0;
                self.from = o.pos;
                self.writes.push_back((control::MODE, 0));
                self.writes.push_back((control::GOAL_DUTY, 0));
                self.writes.push_back((control::TORQUE_ENABLE, 1));
                let sign = self.toward_mid(o.pos);
                self.resting(o.pos, REST_POLLS, Next::Pulse(sign));
                self.step(None)
            }
            Phase::Wait => {
                self.phase = Phase::Read;
                Cmd::Pause {
                    ms: self.cfg.poll_ms,
                }
            }
            Phase::Read => {
                self.phase = Phase::Eval;
                Cmd::Read
            }
            Phase::Eval => match obs {
                Some(o) => self.eval(o),
                None => self.finish(false),
            },
            Phase::Finished => Cmd::Done,
        }
    }

    fn push_watch(&mut self, w: &Watch) {
        if self.watches == 0 {
            self.early = sag::early(w);
        }
        self.watches += 1;
        self.late = sag::late(w).or(self.late);
    }

    fn halted(&self) -> Option<AbortReason> {
        self.halt
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{
        BENCH_MA_PER_CCOUNT, BENCH_MV_PER_VCOUNT, FakeServo, bench_mg90, pump,
    };
    use super::*;

    /// The bench MG90's scales, its 500 mA limit: plastic probes to 225 mA
    /// and holds 300, metal 375 and 500; a lower limit caps both; a rung is
    /// 200 mV on every rail.
    #[test]
    fn knobs_in_physical_units() {
        let lim = (500.0 / BENCH_MA_PER_CCOUNT) as u16;
        let cfg = |class| JamCfg::new(class, BENCH_MV_PER_VCOUNT, BENCH_MA_PER_CCOUNT, lim, None);
        let ma = |c: &JamCfg, i: f64| c.ma(i).round();
        let p = cfg(GearClass::Plastic);
        assert_eq!((ma(&p, p.i_probe), ma(&p, p.i_hold)), (225.0, 300.0));
        assert_eq!(p.press_budget_ms, 250.0);
        let u = cfg(GearClass::Unknown);
        assert_eq!((u.i_probe, u.i_hold), (p.i_probe, p.i_hold));
        let m = cfg(GearClass::Metal);
        assert_eq!((ma(&m, m.i_probe), ma(&m, m.i_hold)), (375.0, 500.0));
        assert_eq!(
            (m.mv(m.step).round(), m.mv(m.v_max).round()),
            (200.0, 2500.0)
        );
        let low = JamCfg::new(
            GearClass::Metal,
            BENCH_MV_PER_VCOUNT,
            BENCH_MA_PER_CCOUNT,
            280,
            None,
        );
        assert_eq!((low.i_probe, low.i_hold), (280.0, 280.0));
    }

    pub(super) mod sweep {
        use super::*;
        use crate::runway::BRAKE_DUTY_Q15;

        /// The bench MG90's stops and soft limits, and its 500 mA limit.
        pub const STOPS: (u16, u16) = (232, 3849);
        pub const SOFT: (f64, f64) = (432.0, 3626.0);
        pub const LIMIT_MA: f64 = 500.0;
        pub const LASH: f64 = 10.0;

        #[derive(Copy, Clone, Debug, PartialEq, Eq)]
        pub enum Fault {
            None,
            Jam,
            DeadPot,
            Open,
        }

        #[derive(Copy, Clone, Debug)]
        pub struct Case {
            pub start: u16,
            pub along: bool,
            pub vbus_v: f64,
            pub breakaway_ma: f64,
            pub class: GearClass,
            pub fault: Fault,
            pub r_stored: bool,
        }

        #[derive(Clone, Debug, PartialEq, Eq)]
        pub enum End {
            Arrived,
            Stiff,
            Blocked,
            Sensor,
            Open,
            Other(String),
        }

        #[derive(Clone, Debug)]
        pub struct Outcome {
            pub end: End,
            pub ms: f64,
            pub impact_cpms: f64,
            pub press_ma: f64,
            pub pressed_ms: f64,
            /// Goals the endstop zeroed as written that were no brake.
            pub refused: usize,
            pub polarity_ok: bool,
            pub torque_off: bool,
            pub travel: u16,
            pub latched: bool,
        }

        pub fn servo(c: &Case) -> FakeServo {
            let mut s = bench_mg90((c.vbus_v * 1000.0 / BENCH_MV_PER_VCOUNT).round() as u16);
            s.ends = (STOPS.0 as f64, STOPS.1 as f64);
            s.soft = Some(SOFT);
            s.pos = c.start as f64;
            s.drive_polarity = c.along;
            s.drive_check = true;
            s.breakaway_q15 = 0;
            s.breakaway_i = c.breakaway_ma / BENCH_MA_PER_CCOUNT;
            s.current_limit = Some((LIMIT_MA / BENCH_MA_PER_CCOUNT) as u16);
            s.step_over_read = 1.3;
            s.backlash = LASH;
            match c.fault {
                Fault::None => {}
                Fault::Jam => s.jam = Some(s.pos),
                Fault::DeadPot => s.pot_dead = Some(c.start),
                Fault::Open => s.open = true,
            }
            s
        }

        pub fn params() -> RigParams {
            let lim = (LIMIT_MA / BENCH_MA_PER_CCOUNT) as i16;
            RigParams::new(Some((532, 3526)), lim + lim / 4)
                .with_stops(STOPS)
                .with_floor(4356)
        }

        pub fn cfg(c: &Case, s: &FakeServo) -> JamCfg {
            let lim = s.current_limit.unwrap_or(u16::MAX);
            let r = c.r_stored.then_some(s.r);
            JamCfg::new(c.class, BENCH_MV_PER_VCOUNT, BENCH_MA_PER_CCOUNT, lim, r)
        }

        pub fn run(c: &Case) -> Outcome {
            run_logged(c).0
        }

        /// [`run`], and every command the check sent.
        pub fn run_logged(c: &Case) -> (Outcome, Vec<String>) {
            let mut s = servo(c);
            let mut log = Vec::new();
            let p = params();
            let t0 = s.t_ms;
            let exp = JamCheck::new(cfg(c, &s), &p);
            let mut rounds = 0;
            let pumped = drive(exp, &p, |e| {
                rounds += 1;
                log.extend(pump(e, &mut s, 100_000));
                Ok::<(), std::convert::Infallible>(())
            });
            let (exp, abort) = match pumped {
                Ok(r) => r,
                Err(never) => match never {},
            };
            let end = match abort {
                None if exp.arrived() => End::Arrived,
                None => End::Other("not arrived".into()),
                Some(AbortReason::Stiff { .. }) => End::Stiff,
                Some(AbortReason::Blocked { .. }) => End::Blocked,
                Some(AbortReason::SensorDead { .. }) => End::Sensor,
                Some(AbortReason::MotorOpen { .. }) => End::Open,
                Some(other) => End::Other(format!("{other:?}")),
            };
            let brake = |g: &i16| g.unsigned_abs() <= BRAKE_DUTY_Q15 as u16;
            let o = Outcome {
                end,
                ms: s.t_ms - t0,
                impact_cpms: s.impact_cps_max / 1000.0,
                press_ma: s.press_i_max * BENCH_MA_PER_CCOUNT,
                pressed_ms: s.pressed_ms,
                refused: s.refused_goals.iter().filter(|g| !brake(g)).count(),
                polarity_ok: exp.polarity() == c.along,
                torque_off: !s.torque,
                travel: exp.pulse_travel_max(),
                latched: rounds > 1,
            };
            (o, log)
        }

        /// What the brief holds every case to: a free shaft arrives with
        /// the polarity right, or ends stiff or blocked; a jam ends stiff
        /// in 2 s, a dead pot at the sensor, an open winding at the motor.
        /// Nothing presses past the class's budget or over its hold, arrives
        /// over 3 counts/ms, or writes a duty the endstop refuses.
        pub fn check(c: &Case, o: &Outcome) -> Result<(), String> {
            let fail = |why: &str| Err(format!("{why}: {c:?} -> {o:?}"));
            if !o.torque_off {
                return fail("torque left on");
            }
            if o.refused > 0 {
                return fail("refused duty");
            }
            if o.impact_cpms > PROBE_CPS_MAX / 1000.0 {
                return fail("impact");
            }
            if o.pressed_ms > c.class.press_budget_ms() || o.press_ma > c.class.hold_ma() {
                return fail("press");
            }
            let ok = match c.fault {
                Fault::None => match o.end {
                    End::Arrived => o.polarity_ok,
                    End::Stiff | End::Blocked => true,
                    _ => false,
                },
                Fault::Jam => o.end == End::Stiff && o.ms <= 2000.0,
                Fault::DeadPot => {
                    o.end == End::Sensor || (o.end == End::Stiff && o.pressed_ms > 0.0)
                }
                Fault::Open => o.end == End::Open,
            };
            if !ok {
                return fail("verdict");
            }
            Ok(())
        }

        pub const RAILS: [f64; 4] = [3.7, 5.0, 6.75, 8.4];
        pub const BREAKAWAYS: [f64; 3] = [120.0, 200.0, 300.0];
        pub const CLASSES: [GearClass; 2] = [GearClass::Plastic, GearClass::Metal];

        pub fn cases(step: usize, faults: usize, r_stored: bool) -> Vec<Case> {
            let mut v = Vec::new();
            for along in [true, false] {
                for vbus_v in RAILS {
                    for class in CLASSES {
                        let mut add = |start, breakaway_ma, fault| {
                            v.push(Case {
                                start,
                                along,
                                vbus_v,
                                breakaway_ma,
                                class,
                                fault,
                                r_stored,
                            })
                        };
                        for breakaway in BREAKAWAYS {
                            for start in (250..=3830).step_by(step) {
                                add(start, breakaway, Fault::None);
                            }
                        }
                        for fault in [Fault::Jam, Fault::DeadPot, Fault::Open] {
                            for start in (250..=3830).step_by(faults) {
                                add(start, 200.0, fault);
                            }
                        }
                    }
                }
            }
            v
        }
    }

    use sweep::*;

    fn case(start: u16, along: bool, fault: Fault) -> Case {
        Case {
            start,
            along,
            vbus_v: 7.4,
            breakaway_ma: 200.0,
            class: GearClass::Metal,
            fault,
            r_stored: false,
        }
    }

    fn polarity_writes(log: &[String]) -> Vec<&String> {
        log.iter()
            .filter(|l| l.starts_with("write drive_polarity"))
            .collect()
    }

    /// Wired reversed under the low soft limit: the first pulse, meant up,
    /// runs down. The pulse back would point out past the soft limit, so it
    /// is never written; the flip goes to the servo in RAM and the drive
    /// leaves toward mid travel without pressing the stop.
    #[test]
    fn a_reversal_past_a_soft_limit_is_flipped_in_ram() {
        let (o, log) = run_logged(&case(380, false, Fault::None));
        assert_eq!(o.end, End::Arrived, "{o:?}");
        assert!(o.polarity_ok && o.torque_off && !o.latched);
        assert_eq!((o.refused, o.pressed_ms), (0, 0.0));
        assert_eq!(polarity_writes(&log), ["write drive_polarity 0"]);
    }

    /// A reversed motor that runs two legs against the duty past the soft
    /// limit unseen - a dead pot - latches drive_reversed. The check waits
    /// for rest, flips once, acknowledges at duty 0, and goes on to the
    /// verdict the pot earns; it never flips twice.
    #[test]
    fn a_latch_is_acknowledged_at_rest_and_resumed_once() {
        let (o, log) = run_logged(&case(400, false, Fault::DeadPot));
        assert!(o.latched, "{o:?}");
        assert_eq!(o.end, End::Sensor);
        assert!(o.torque_off);
        assert_eq!(polarity_writes(&log), ["write drive_polarity 0"]);
        let ack = log
            .iter()
            .position(|l| l == "write drive_polarity 0")
            .unwrap();
        assert_eq!(
            log[ack + 1..ack + 3],
            ["write goal_duty 0", "write torque_enable 1"]
        );
    }

    /// A locked shaft ends stiff at the probe's cap, an open winding at the
    /// motor, a dead pot at the sensor; a free shaft's pulses move it under
    /// 80 counts.
    #[test]
    fn each_fault_ends_at_its_verdict() {
        assert_eq!(run(&case(2029, true, Fault::Jam)).end, End::Stiff);
        assert_eq!(run(&case(2029, true, Fault::Open)).end, End::Open);
        assert_eq!(run(&case(2029, true, Fault::DeadPot)).end, End::Sensor);
        let free = run(&case(2029, true, Fault::None));
        assert_eq!(free.end, End::Arrived);
        assert!(free.travel < 80, "{free:?}");
    }

    /// Reversed and resting within a dozen counts of a stop past its soft
    /// limit, the only pulse the endstop allows presses the stop and the pot
    /// cannot show which way the motor turns: the check ends stiff, inside
    /// the class's press budget and hold. From a dozen counts off it
    /// arrives.
    #[test]
    fn reversed_at_a_stop_ends_stiff_inside_the_budget() {
        for class in CLASSES {
            for vbus_v in [3.7, 8.4] {
                for start in (232..=256).step_by(3).chain((3826..=3849).step_by(3)) {
                    let c = Case {
                        vbus_v,
                        class,
                        ..case(start, false, Fault::None)
                    };
                    let o = run(&c);
                    check(&c, &o).unwrap();
                    let off = start.abs_diff(STOPS.0).min(start.abs_diff(STOPS.1));
                    if off >= 2 * MOVED_COUNTS {
                        assert_eq!(o.end, End::Arrived, "{c:?}");
                    }
                }
            }
        }
    }

    /// Every case on `threads` threads, in order.
    fn run_all(cases: &[Case]) -> Vec<Outcome> {
        let threads = std::thread::available_parallelism().map_or(1, |n| n.get().min(4));
        let chunk = cases.len().div_ceil(threads).max(1);
        std::thread::scope(|sc| {
            let parts: Vec<_> = cases
                .chunks(chunk)
                .map(|part| sc.spawn(|| part.iter().map(run).collect::<Vec<_>>()))
                .collect();
            parts
                .into_iter()
                .flat_map(|h| h.join().expect("sweep thread"))
                .collect()
        })
    }

    /// The table a sweep prints: one row a wiring, rail, class and
    /// breakaway or fault.
    fn table(cases: &[Case], outs: &[Outcome]) -> String {
        use std::collections::BTreeMap;
        use std::fmt::Write;
        #[derive(Default)]
        struct Row {
            n: u32,
            ends: BTreeMap<String, u32>,
            ms: f64,
            impact: f64,
            pressed: f64,
            press_ma: f64,
            travel: u16,
            latched: u32,
        }
        let mut rows: BTreeMap<String, Row> = BTreeMap::new();
        for (c, o) in cases.iter().zip(outs) {
            let what = match c.fault {
                Fault::None => format!("{:>3.0} mA", c.breakaway_ma),
                f => format!("{f:?}"),
            };
            let key = format!(
                "{} | {:>4.2} V | {:<7} | {what:<7}",
                if c.along { "normal  " } else { "reversed" },
                c.vbus_v,
                c.class.name()
            );
            let r = rows.entry(key).or_default();
            r.n += 1;
            *r.ends.entry(format!("{:?}", o.end)).or_default() += 1;
            r.ms = r.ms.max(o.ms);
            r.impact = r.impact.max(o.impact_cpms);
            r.pressed = r.pressed.max(o.pressed_ms);
            r.press_ma = r.press_ma.max(o.press_ma);
            r.travel = r.travel.max(o.travel);
            r.latched += o.latched as u32;
        }
        let mut t = String::from(
            "wiring | rail | class | case | n | ends | max ms | max impact c/ms | max press ms \
             | max press mA | max pulse travel | latches\n",
        );
        for (k, r) in rows {
            let ends: Vec<String> = r.ends.iter().map(|(e, n)| format!("{e} {n}")).collect();
            let _ = writeln!(
                t,
                "{k} | {} | {} | {:.0} | {:.2} | {:.0} | {:.0} | {} | {}",
                r.n,
                ends.join(", "),
                r.ms,
                r.impact,
                r.pressed,
                r.press_ma,
                r.travel,
                r.latched
            );
        }
        t
    }

    fn sweep(step: usize, faults: usize, r_stored: bool) {
        let cases = cases(step, faults, r_stored);
        let outs = run_all(&cases);
        println!("{}", table(&cases, &outs));
        let fails: Vec<String> = cases
            .iter()
            .zip(&outs)
            .filter_map(|(c, o)| check(c, o).err())
            .collect();
        assert!(
            fails.is_empty(),
            "{} of {}: {:#?}",
            fails.len(),
            cases.len(),
            &fails[..fails.len().min(5)]
        );
    }

    /// Starts 250..3830 every 10 counts, both wirings, rails 3.7 to 8.4 V,
    /// breakaways 120, 200 and 300 mA, plastic and metal, on a virgin servo
    /// (no R stored), and a jam, a dead pot and an open winding every 250
    /// counts: each arrives with the polarity right or ends stiff or
    /// blocked, faults end at their verdict, nothing presses past the
    /// class's budget or over its hold, arrives at a stop over 3 counts/ms,
    /// or writes a duty the endstop refuses.
    #[test]
    fn the_sweep_holds_on_a_virgin_servo() {
        sweep(10, 250, false);
    }

    /// The same with the winding R stored, the ohmic reference in place of
    /// the early windows: every 50 counts.
    #[test]
    fn the_sweep_holds_with_the_stored_r() {
        sweep(50, 250, true);
    }
}
