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
//! still is felt ([`sag::feel`]); one that crept over [`STILL_COUNTS`]
//! raises a rung. The ladder ends stiff once the next rung would stall
//! over the probe's cap or press a stop past the class's budget, at the
//! motor when nothing flows by the backstop, and at the pot after
//! [`DEAD_PULSES`] pulses the motor turned and the pot did not, each the
//! way the one before went: a reversal crosses the gear lash, so a turning
//! pulse after one is repeated before it counts. Near a stop past a soft
//! limit, where the endstop allows one way only, that way may press the
//! stop: the cap is [`STOP_PROBE_MA`] and a still shaft there ends at the
//! stop, the direction unproven. Pulses are timed on the servo's clock: a host stall
//! lengthens the drive itself, and only a press counts against the budget.
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

/// Pot travel a pulse may make and still count as still: a poll's noise.
pub const STILL_COUNTS: u16 = 3;
/// Pot travel a pulse must make to count as moved.
pub const MOVED_COUNTS: u16 = 4 * STILL_COUNTS;
/// Turning pulses with the pot still, each repeating the way the one
/// before went, that end the check at the sensor.
pub const DEAD_PULSES: u32 = 2;
/// A pulse, and the least rest after one, in polls: 40 and 40 ms asked, a
/// pulse 55 ms on the bus with its reads, under the 60 ms a press may
/// last. Its first poll reads the early windows, its second the sag. After
/// a pulse that did not turn the motor nothing coasts, and one poll at
/// zero will do; a turning rotor the pot cannot see coasts on, and the
/// next pulse's early windows must not start on it.
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

/// The probe's current cap near a stop past a soft limit, mA: the bench
/// MG90 seats its stops at about this, and its low stopper took a set at
/// 320.
pub const STOP_PROBE_MA: f64 = 250.0;

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
    /// The probe's cap, its cap near a stop past a soft limit and the stop
    /// hold, all at most the servo's limit, and the open-winding floor:
    /// ccounts.
    pub i_probe: f64,
    pub i_stop: f64,
    pub i_hold: f64,
    pub i_open: f64,
    /// Winding R, vcounts per ccount, when the servo carries one.
    pub r_vpc: Option<f64>,
    pub press_budget_ms: f64,
    /// The servo's fast tick rate, Hz: `sample_tick` is its clock.
    pub tick_hz: f64,
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
        tick_hz: f64,
    ) -> Self {
        let ccounts = |ma: f64| (ma / ma_per_ccount).min(i_lim as f64);
        let i_probe = ccounts(PROBE_OF_HOLD * class.hold_ma());
        Self {
            class,
            mv_per_vcount,
            ma_per_ccount,
            step: PROBE_STEP_MV / mv_per_vcount,
            v_max: PROBE_MAX_MV / mv_per_vcount,
            i_probe,
            i_stop: ccounts(STOP_PROBE_MA).min(i_probe),
            i_hold: ccounts(class.hold_ma()),
            i_open: OPEN_MA / ma_per_ccount,
            r_vpc,
            press_budget_ms: class.press_budget_ms(),
            tick_hz,
            poll_ms: 20,
            margin: 300,
            polls_max: 400,
        }
    }

    pub fn for_servo(lim: &ServoLimits, sc: &Scales, class: GearClass, tick_hz: f64) -> Self {
        Self::new(
            class,
            sc.v_term_per_count * 1000.0,
            sc.amps_per_count * 1000.0,
            lim.i_lim,
            lim.r_vpc(),
            tick_hz,
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
pub enum Seen {
    /// The pot moved, this way.
    Moved(i8),
    Felt(Felt),
    /// The endstop zeroed it.
    Refused,
}

impl Seen {
    pub fn name(self) -> &'static str {
        match self {
            Seen::Moved(d) if d > 0 => "moved up",
            Seen::Moved(_) => "moved down",
            Seen::Felt(Felt::Stalled) => "stalled",
            Seen::Felt(Felt::Turning) => "turning",
            Seen::Felt(Felt::Dead) => "no current",
            Seen::Felt(Felt::Unclear) => "unclear",
            Seen::Refused => "refused",
        }
    }
}

/// One probe pulse.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pulse {
    /// The rung, mV, and the duty written, q15: none when refused.
    pub mv: f64,
    pub duty_q15: i32,
    pub sign: i8,
    /// The pot where it started and where the verdict read it.
    pub from: u16,
    pub to: u16,
    /// The last watch's current, and the stall current by the stored R:
    /// mA.
    pub late_ma: f64,
    pub ohmic_ma: Option<f64>,
    pub seen: Seen,
    /// The pulse before it turned the motor this way: no lash to cross.
    pub repeat: bool,
    /// Its length on the servo's clock and on the host's, ms.
    pub ms: f64,
    pub host_ms: f64,
}

impl Pulse {
    /// Pot travel between the start and the verdict, counts.
    pub fn travel(&self) -> i32 {
        self.to as i32 - self.from as i32
    }
}

impl core::fmt::Display for Pulse {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let way = if self.sign > 0 { '+' } else { '-' };
        write!(f, "{:.0} mV {way}", self.mv)?;
        if self.seen == Seen::Refused {
            return write!(f, ", refused at pos {}", self.from);
        }
        write!(
            f,
            " ({:.1}%), pos {} -> {} ({:+}), late {:.0} mA",
            self.duty_q15.unsigned_abs() as f64 / 32767.0 * 100.0,
            self.from,
            self.to,
            self.travel(),
            self.late_ma
        )?;
        if let Some(i) = self.ohmic_ma {
            write!(f, " of {i:.0}")?;
        }
        write!(f, ", {}", self.seen.name())?;
        let crept = self.travel().unsigned_abs() > STILL_COUNTS as u32;
        if matches!(self.seen, Seen::Felt(_)) && crept {
            write!(f, ", crept")?;
        } else if self.seen == Seen::Felt(Felt::Turning) {
            let lash = if self.repeat {
                "counted"
            } else {
                "repeat next"
            };
            write!(f, ", {lash}")?;
        }
        write!(f, ", {:.0} ms ({:.0} on the host)", self.ms, self.host_ms)
    }
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
    /// A pulse of `sign` from `from`; `at` is the last read's
    /// `sample_tick` and pos.
    Pulse {
        sign: i8,
        from: u16,
        polls: u32,
        at: (u32, u16),
    },
    Toward {
        from: u16,
        dir: i8,
        at: (u32, u16),
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
    /// The way the last pulse written turned the motor, 0 when it may not
    /// have or after a flip, and whether this one goes the same way.
    closed: i8,
    repeat: bool,
    /// The last read was past a soft limit and near a stop: the endstop
    /// walls one way and the other may press the stop.
    at_wall: bool,
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
    /// how many were felt and how many drew nothing; and the turning
    /// pulses that count toward [`DEAD_PULSES`].
    ohm: Option<f64>,
    i_still: f64,
    felt: u32,
    dead: u32,
    turning: u32,
    /// Time pressed, by the longest still pulse of each rung at a stop (at
    /// any, the stops unknown), and the longest of this one: ms on the
    /// servo's clock.
    press_ms: f64,
    rung_press: f64,
    /// When the pulse started, `sample_tick` and host ms, and its duty.
    t0: u32,
    duty: i32,
    host_t0: f64,
    pulses: Vec<Pulse>,
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
            closed: 0,
            repeat: false,
            at_wall: false,
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
            t0: 0,
            duty: 0,
            host_t0: 0.0,
            pulses: Vec::new(),
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

    /// Every probe pulse, refused ones included, in order.
    pub fn pulses(&self) -> &[Pulse] {
        &self.pulses
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

    /// Within [`seek::STOP_TOL`] of a stop, or anywhere with them unknown.
    fn near_stop(&self, pos: u16) -> bool {
        self.params
            .stops
            .is_none_or(|(lo, hi)| pos.abs_diff(lo).min(pos.abs_diff(hi)) <= seek::STOP_TOL)
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
    /// The duty written, q15.
    fn goal(&mut self, sign: i8) -> i32 {
        let q15 = if self.vbus > 0.0 {
            (self.v / self.vbus * 32767.0).round().min(32767.0)
        } else {
            0.0
        };
        let duty = sign as i32 * q15 as i32;
        self.writes.push_back((control::GOAL_DUTY, duty));
        duty
    }

    fn flip(&mut self) {
        self.polarity = !self.polarity;
        self.flips += 1;
        self.closed = 0;
        self.writes
            .push_back((config::DRIVE_POLARITY, self.polarity as i32));
    }

    /// Ms on the servo's clock since `tick`.
    fn servo_ms(&self, tick: u32, o: &TelemetrySnapshot) -> f64 {
        o.sample_tick.wrapping_sub(tick) as f64 * 1000.0 / self.cfg.tick_hz
    }

    /// Counts/s between the read at `at` and `o`.
    fn speed(&self, o: &TelemetrySnapshot, at: (u32, u16)) -> f64 {
        let ms = self.servo_ms(at.0, o);
        if ms > 0.0 {
            o.pos.abs_diff(at.1) as f64 * 1000.0 / ms
        } else {
            0.0
        }
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
        self.at_wall = o.limit_flags & LIMIT_ENDSTOP != 0 && self.near_stop(o.pos);
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
            self.pulses.push(Pulse {
                mv: self.cfg.mv(self.v),
                duty_q15: 0,
                sign,
                from: o.pos,
                to: o.pos,
                late_ma: 0.0,
                ohmic_ma: None,
                seen: Seen::Refused,
                repeat: false,
                ms: 0.0,
                host_ms: 0.0,
            });
            return match self.after(sign, Seen::Refused, o.pos) {
                Ok(then) => self.next(o, then),
                Err(why) => self.halt(why),
            };
        }
        self.duty = self.goal(sign);
        self.repeat = self.closed == sign;
        (self.from, self.t0, self.host_t0) = (o.pos, o.sample_tick, o.host_ms);
        (self.watches, self.early, self.late) = (0, None, None);
        self.leg = Leg::Pulse {
            sign,
            from: o.pos,
            polls: 0,
            at: (o.sample_tick, o.pos),
        };
        self.wait()
    }

    fn pulse(
        &mut self,
        o: &TelemetrySnapshot,
        sign: i8,
        from: u16,
        polls: u32,
        at: (u32, u16),
    ) -> Cmd {
        let moved = o.pos as i32 - from as i32;
        let late = self.late.unwrap_or(o.i_mean_counts.unsigned_abs() as f64);
        let seen = if moved.unsigned_abs() >= MOVED_COUNTS as u32 {
            Seen::Moved(moved.signum() as i8)
        } else if self.walled(sign, o) {
            Seen::Refused
        } else if polls >= PULSE_POLLS {
            Seen::Felt(self.feel(o, late))
        } else {
            self.leg = Leg::Pulse {
                sign,
                from,
                polls,
                at: (o.sample_tick, o.pos),
            };
            return self.wait();
        };
        let ms = self.servo_ms(self.t0, o);
        let turned = matches!(seen, Seen::Moved(_) | Seen::Felt(Felt::Turning))
            || moved.unsigned_abs() > STILL_COUNTS as u32;
        self.closed = if turned { sign } else { 0 };
        self.pulses.push(Pulse {
            mv: self.cfg.mv(self.v),
            duty_q15: self.duty,
            sign,
            from,
            to: o.pos,
            late_ma: self.cfg.ma(late),
            ohmic_ma: self.cfg.r_vpc.map(|r| self.cfg.ma(self.v / r)),
            seen,
            repeat: self.repeat,
            ms,
            host_ms: o.host_ms - self.host_t0,
        });
        if matches!(seen, Seen::Felt(_)) && self.near_stop(o.pos) {
            self.rung_press = self.rung_press.max(ms);
        }
        let then = match self.after(sign, seen, o.pos) {
            Ok(then) => then,
            Err(why) => return self.halt(why),
        };
        if self.speed(o, at) > PROBE_CPS_MAX {
            self.v = (self.v - self.cfg.step).max(self.v0);
        }
        let least = if turned { REST_POLLS } else { STILL_REST_POLLS };
        self.rest(o.pos, least, then)
    }

    fn feel(&mut self, o: &TelemetrySnapshot, late: f64) -> Felt {
        let ohmic = self.cfg.r_vpc.map(|r| self.v / r);
        let pinned = o.limit_flags & LIMIT_GOVERNING != 0;
        let felt = sag::feel(late, self.early, ohmic, pinned, self.cfg.i_open);
        self.felt += 1;
        self.i_still = self.i_still.max(late);
        match felt {
            Felt::Stalled => self.ohm = Some(self.ohm.unwrap_or(0.0).max(late / self.v)),
            Felt::Dead => self.dead += 1,
            Felt::Turning | Felt::Unclear => {}
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
        if let Seen::Felt(felt) = seen {
            if pos.abs_diff(self.from) > STILL_COUNTS {
                return self.raise(pos);
            }
            if felt == Felt::Turning {
                if !self.repeat {
                    return Ok(Next::Pulse(sign));
                }
                self.turning += 1;
                if self.turning >= DEAD_PULSES {
                    return Err(AbortReason::SensorDead { pos });
                }
            }
        }
        if self.rung[(sign < 0) as usize].is_none() {
            return Ok(Next::Pulse(-sign));
        }
        self.raise(pos)
    }

    /// Both ways tried at this rung, or a creep: the next one, or the
    /// ladder's end. The next rung presses a stop about as long as this
    /// one did.
    fn raise(&mut self, pos: u16) -> Result<Next, AbortReason> {
        let pressed = std::mem::take(&mut self.rung_press);
        self.press_ms += pressed;
        let cap = if self.at_wall {
            self.cfg.i_stop
        } else {
            self.cfg.i_probe
        };
        let top = self.volts_for(cap).min(self.cfg.v_max);
        let v = (self.v + self.cfg.step).min(top);
        if v <= self.v || self.press_ms + pressed > self.cfg.press_budget_ms {
            if self.turning > 0 {
                return Err(AbortReason::SensorDead { pos });
            }
            if self.felt > 0 && self.dead == self.felt {
                return Err(AbortReason::MotorOpen { pos });
            }
            let ma = self.cfg.ma(self.i_still).round().min(u16::MAX as f64) as u16;
            return Err(if self.at_wall {
                AbortReason::AtStop { pos, ma }
            } else {
                AbortReason::Stiff { pos, ma }
            });
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
            at: (o.sample_tick, o.pos),
        };
        self.wait()
    }

    fn toward(&mut self, o: &TelemetrySnapshot, from: u16, dir: i8, at: (u32, u16)) -> Cmd {
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
        } else if self.speed(o, at) > PROVEN_CPS_MAX {
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
            at: (o.sample_tick, o.pos),
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
        BENCH_MA_PER_CCOUNT, BENCH_MV_PER_VCOUNT, Bus, FakeServo, TICK_HZ, bench_mg90, pump_on,
    };
    use super::*;

    /// The bench MG90's scales, its 500 mA limit: plastic probes to 225 mA
    /// and holds 300, metal 375 and 500; near a stop past a soft limit
    /// plastic keeps its 225 and metal probes to 250; a lower limit caps
    /// them all; a rung is 200 mV on every rail.
    #[test]
    fn knobs_in_physical_units() {
        let lim = (500.0 / BENCH_MA_PER_CCOUNT) as u16;
        let new = |class, lim| {
            JamCfg::new(
                class,
                BENCH_MV_PER_VCOUNT,
                BENCH_MA_PER_CCOUNT,
                lim,
                None,
                TICK_HZ,
            )
        };
        let ma = |c: &JamCfg, i: f64| c.ma(i).round();
        let p = new(GearClass::Plastic, lim);
        assert_eq!(
            (ma(&p, p.i_probe), ma(&p, p.i_stop), ma(&p, p.i_hold)),
            (225.0, 225.0, 300.0)
        );
        assert_eq!(p.press_budget_ms, 250.0);
        let u = new(GearClass::Unknown, lim);
        assert_eq!((u.i_probe, u.i_hold), (p.i_probe, p.i_hold));
        let m = new(GearClass::Metal, lim);
        assert_eq!(
            (ma(&m, m.i_probe), ma(&m, m.i_stop), ma(&m, m.i_hold)),
            (375.0, 250.0, 500.0)
        );
        assert_eq!(
            (m.mv(m.step).round(), m.mv(m.v_max).round()),
            (200.0, 2500.0)
        );
        let low = new(GearClass::Metal, 200);
        assert_eq!((low.i_probe, low.i_stop, low.i_hold), (200.0, 200.0, 200.0));
    }

    pub(super) mod sweep {
        use super::*;
        use crate::runway::BRAKE_DUTY_Q15;

        /// The bench MG90's stops and soft limits, and its 500 mA limit.
        pub const STOPS: (u16, u16) = (232, 3849);
        pub const SOFT: (f64, f64) = (432.0, 3626.0);
        pub const LIMIT_MA: f64 = 500.0;
        pub const LASH: f64 = 10.0;
        /// The stoppers: first contact this many counts out from the press
        /// depth, and how far each yields, counts an amp. The bench MG90's
        /// low stop yields about 95 counts an amp, its high one 48.
        pub const GIVE_COUNTS: (f64, f64) = (30.0, 20.0);
        pub const GIVE_PER_A: (f64, f64) = (90.0, 40.0);

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
            AtStop,
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
            /// The polarity the check ended with is the stored one.
            pub kept: bool,
            pub torque_off: bool,
            pub travel: u16,
            pub latched: bool,
            pub pulses: Vec<Pulse>,
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
            let per_ccount = |per_a: f64| per_a * BENCH_MA_PER_CCOUNT / 1000.0;
            s.give = [
                (GIVE_COUNTS.0, per_ccount(GIVE_PER_A.0)),
                (GIVE_COUNTS.1, per_ccount(GIVE_PER_A.1)),
            ];
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
            JamCfg::new(
                c.class,
                BENCH_MV_PER_VCOUNT,
                BENCH_MA_PER_CCOUNT,
                lim,
                r,
                s.tick_hz(),
            )
        }

        pub fn run(c: &Case) -> Outcome {
            run_logged(c).0
        }

        /// [`run`], and every command the check sent.
        pub fn run_logged(c: &Case) -> (Outcome, Vec<String>) {
            run_with(c, Bus::BENCH, |_, _| {})
        }

        /// [`run_logged`] over `bus`, the servo and the rig as `tweak`
        /// leaves them.
        pub fn run_with(
            c: &Case,
            bus: Bus,
            tweak: impl FnOnce(&mut FakeServo, &mut RigParams),
        ) -> (Outcome, Vec<String>) {
            let mut s = servo(c);
            let mut log = Vec::new();
            let mut p = params();
            tweak(&mut s, &mut p);
            let t0 = s.t_ms;
            let exp = JamCheck::new(cfg(c, &s), &p);
            let stored = p.drive_polarity;
            let mut rounds = 0;
            let pumped = drive(exp, &p, |e| {
                rounds += 1;
                log.extend(pump_on(e, &mut s, 100_000, bus));
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
                Some(AbortReason::AtStop { .. }) => End::AtStop,
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
                kept: exp.polarity() == stored,
                torque_off: !s.torque,
                travel: exp.pulse_travel_max(),
                latched: rounds > 1,
                pulses: exp.pulses().to_vec(),
            };
            (o, log)
        }

        /// A press projected from a creeping pulse, whose current reads its
        /// stall current low by the back-EMF, lands this much over the cap.
        pub const CREEP_OVER: f64 = 1.03;

        /// What the brief holds every case to: a free shaft arrives with
        /// the polarity right, or ends stiff, blocked or at a stop; a jam
        /// ends stiff or at a stop in 2 s, a dead pot at the sensor, or
        /// pressing, an open winding at the motor. Nothing presses past the
        /// class's budget or over the cap near a stop, arrives over 3
        /// counts/ms, or writes a duty the endstop refuses.
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
            if o.pressed_ms > c.class.press_budget_ms() || o.press_ma > STOP_PROBE_MA * CREEP_OVER {
                return fail("press");
            }
            let ok = match c.fault {
                Fault::None => match o.end {
                    End::Arrived => o.polarity_ok,
                    End::Stiff | End::Blocked | End::AtStop => true,
                    _ => false,
                },
                Fault::Jam => matches!(o.end, End::Stiff | End::AtStop) && o.ms <= 2000.0,
                Fault::DeadPot => {
                    o.end == End::Sensor
                        || (matches!(o.end, End::Stiff | End::AtStop) && o.pressed_ms > 0.0)
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

    /// Reversed and resting in a stopper's give, 2 to 15 counts in from
    /// first contact, the only pulse the endstop allows presses the stop.
    /// Where the give at the cap is under a dozen counts the pot cannot
    /// show the way: the check ends at the stop, the stored polarity kept.
    /// Nearer first contact the give may show it, and the check arrives
    /// with the polarity right. No press goes over the cap or past the
    /// class's budget. Two dozen counts out from first contact it arrives.
    #[test]
    fn reversed_in_a_stoppers_give_ends_at_the_stop() {
        let s = servo(&case(2029, true, Fault::None));
        let (lo, hi) = s.contact();
        let gives = |end: usize| s.give[end].1 * STOP_PROBE_MA / BENCH_MA_PER_CCOUNT;
        let unproven = |start: u16| {
            let into = if start < STOPS.1 / 2 {
                gives(0) - (lo - start as f64)
            } else {
                gives(1) - (start as f64 - hi)
            };
            into < MOVED_COUNTS as f64
        };
        let (lo, hi) = (lo as u16, hi as u16);
        for class in CLASSES {
            for vbus_v in RAILS {
                let at = |start| Case {
                    vbus_v,
                    class,
                    ..case(start, false, Fault::None)
                };
                for start in (2..=15).step_by(3).flat_map(|d| [lo - d, hi + d]) {
                    let (c, o) = (at(start), run(&at(start)));
                    check(&c, &o).unwrap();
                    let ok = match o.end {
                        End::AtStop => o.kept,
                        End::Arrived => o.polarity_ok && !unproven(start),
                        _ => false,
                    };
                    assert!(ok, "{c:?} -> {o:?}");
                }
                for start in [lo + 2 * MOVED_COUNTS, hi - 2 * MOVED_COUNTS] {
                    let (c, o) = (at(start), run(&at(start)));
                    check(&c, &o).unwrap();
                    assert_eq!(o.end, End::Arrived, "{c:?} -> {o:?}");
                }
            }
        }
    }

    /// Run 31 on the bench MG90 at 4.7 V: the floor at 5.3% puts the rungs
    /// at 0.25, 0.45 and 0.65 V, the shaft breaks away between the first
    /// two and runs on 45 mA, so a turning rotor sags under 0.6; the lash
    /// is 15 to 45 counts.
    const RUN31_LASH: [f64; 4] = [15.0, 25.0, 35.0, 45.0];

    fn run31(start: u16, fault: Fault) -> Case {
        Case {
            start,
            vbus_v: 4.7,
            breakaway_ma: 80.0,
            r_stored: true,
            ..case(start, true, fault)
        }
    }

    fn run31_plant(lash: f64) -> impl FnOnce(&mut FakeServo, &mut RigParams) {
        move |s, p| {
            s.floor_q15 = 1734;
            p.window_floor_q15 = 1734;
            s.backlash = lash;
            s.fc = 45.0 / BENCH_MA_PER_CCOUNT;
            s.fv = 0.0;
        }
    }

    /// A rung creeps under a dozen counts and the pulse back spends itself
    /// in the lash: neither is a dead pot, and the check arrives.
    #[test]
    fn a_creep_and_a_lash_crossing_are_not_a_dead_pot() {
        for lash in RUN31_LASH {
            for start in [1992, 2029, 2400] {
                let (o, _) = run_with(&run31(start, Fault::None), Bus::BENCH, run31_plant(lash));
                assert_eq!(o.end, End::Arrived, "lash {lash} {start}: {o:?}");
                assert!(o.polarity_ok, "{o:?}");
            }
        }
    }

    /// The same plant with its pot dead still ends at the sensor, on
    /// turning pulses that repeat the way before.
    #[test]
    fn a_dead_pot_by_the_lash_ends_at_the_sensor() {
        for lash in RUN31_LASH {
            let (o, _) = run_with(&run31(2029, Fault::DeadPot), Bus::BENCH, run31_plant(lash));
            assert_eq!(o.end, End::Sensor, "lash {lash}: {o:?}");
            let turning = Seen::Felt(Felt::Turning);
            let counted = o
                .pulses
                .iter()
                .filter(|p| p.repeat && p.seen == turning)
                .count();
            assert!(counted >= DEAD_PULSES as usize, "{:#?}", o.pulses);
        }
    }

    /// The host stalls 600 ms inside a still pulse at mid travel, metal and
    /// plastic, at 5 V where the shaft moves at the third rung: the drive
    /// ran that long, but pressed nothing, and the check goes on to mid
    /// travel.
    #[test]
    fn a_host_stall_mid_travel_is_no_verdict() {
        for class in CLASSES {
            for read in [4, 5, 8, 9] {
                let c = Case {
                    class,
                    vbus_v: 5.0,
                    ..case(2029, true, Fault::None)
                };
                let (o, _) = run_with(&c, Bus::BENCH.with_stall(read, 600.0), |_, _| {});
                assert_eq!(o.end, End::Arrived, "{class:?} read {read}: {o:?}");
                assert!(o.polarity_ok && o.pressed_ms == 0.0, "{o:?}");
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
