//! Sans-io experiment engine. An [`Experiment`] is a state machine: each
//! `step` consumes at most one observation (the reply to a previous
//! [`Cmd::Read`]) and emits the next command. The driver - CLI over USB
//! today, wasm GUI over Web Serial later - owns all IO and time, and hands
//! its clock in as data: every snapshot carries the host's time of the read
//! ([`TelemetrySnapshot::host_ms`]), the clock polled fits time the shaft on.
//!
//! ```ignore
//! let mut pending = None;
//! loop {
//!     match exp.step(pending.take().as_ref()) {
//!         Cmd::Write { reg, value } => client.write(reg, value),
//!         Cmd::Read => pending = Some(read_telemetry_region(now_ms())),
//!         Cmd::Pause { ms } => sleep_ms(ms),
//!         Cmd::Stream { samples, goal, chunks } => {
//!             exp.push_tel(&run_burst(samples, goal, chunks))
//!         }
//!         Cmd::Burst { duty_q15, pre_q15, chans, seated } => {
//!             exp.push_burst(&capture(duty_q15, pre_q15, chans, seated))
//!         }
//!         Cmd::Done => break,
//!     }
//! }
//! ```
//!
//! A drive watches while it waits: every `Pause` streams TEL rows of the
//! pot, current, duty and differential ([`crate::window`]), handed in through
//! [`Experiment::push_watch`] as each burst lands, and the next `Read`
//! carries the latest window.
//!
//! [`Guarded`] wraps any experiment with the safety envelope, and ends one
//! whose burst the servo dropped rows from: such a burst is not evidence.
//! Rig limits live in [`RigParams`], the one home for bench constants.
//! [`Permitted`] holds the stall permit for one that stalls on purpose, and
//! the driver keeps a held permit alive with [`crate::limits::PermitLease`].

pub mod anchor;
pub mod bias;
pub mod breakaway;
pub mod centre;
pub mod endstop;
pub mod held;
pub mod inductance;
pub mod inertia;
pub mod jam;
pub mod ladder;
pub mod resistance;
pub mod rl;
pub mod sag;
pub mod seek;
pub mod session;
pub mod sweep;
pub mod verify;
pub mod wavefit;
pub mod winding;

use osc_protocol::tel::Chunks;
use osc_servo_core::kernel::DECIM_MED;

use crate::burst::Capture;
use crate::frame::{SeqUnwrap, TelBurst, TelemetrySnapshot, counts_q15, duty_counts};
use crate::pot::Pot;
use crate::regs::{Reg, control};
use crate::window::{WINDOW_TICKS, Watch, Watched, Window};

/// One driver action. `Write` is a single-field wire write (the value is
/// truncated to the reg width by the driver); `Read` is one gread of the
/// telemetry region, whose snapshot, the latest watched window attached,
/// feeds the NEXT `step`. `Pause` is a watch: the whole windows that fit in
/// it stream, the window's fields and the pot when the line holds it, in
/// bursts of at most [`crate::window::WATCH_BURST_MS`], each handed in
/// through [`Experiment::push_watch`] as it lands; a pause too short for a
/// window sleeps. `Stream` arms a TEL burst of `samples` fast ticks of
/// `chunks` rows: one WRITE of the chunks and `tel_count` - with `goal`
/// Some the driver stages that write and the arm under HOLD and fires one
/// broadcast COMMIT, so the write applies in the same instant the capture
/// starts. The decoded burst, with the rows the servo dropped from it,
/// returns through [`Experiment::push_tel`] before the next `step`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Cmd {
    Write {
        reg: Reg,
        value: i32,
    },
    Read,
    Pause {
        ms: u32,
    },
    Stream {
        samples: u16,
        goal: Option<(Reg, i32)>,
        chunks: Chunks,
    },
    /// One high-rate shunt burst: the driver stages `duty_q15`, the `chans`
    /// mask and the arm under HOLD, fires one COMMIT, polls to Done and
    /// walks the readback pages ([`crate::burst`]). `pre_q15` is the duty
    /// already in force and `seated` says the rotor is held against a stop -
    /// bookkeeping for the capture's meta, not writes. The assembled capture
    /// returns through [`Experiment::push_burst`].
    Burst {
        duty_q15: i16,
        pre_q15: i16,
        chans: u8,
        seated: bool,
    },
    Done,
}

/// A pumpable experiment. `step` with `Some` only in reply to [`Cmd::Read`].
pub trait Experiment {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd;
    /// The last [`Cmd::Stream`] burst; experiments that never stream keep
    /// the drop default.
    fn push_tel(&mut self, _tel: &TelBurst) {}
    /// The capture from the last [`Cmd::Burst`].
    fn push_burst(&mut self, _cap: &Capture) {}
    /// One watch burst of a `Pause`, as it lands.
    fn push_watch(&mut self, _w: &Watch) {}
    /// Set once the experiment stopped on something that ends the whole
    /// run, not just itself: [`Guarded`] preempts it on the next step.
    fn halted(&self) -> Option<AbortReason> {
        None
    }
}

/// Rig constants - the single home. The travel guard, the current abort
/// and the window floor come from the servo's own limits
/// ([`crate::limits::ServoLimits`]); the rest are bench defaults.
/// Experiments take what they need; the CLI overrides via flags.
#[derive(Copy, Clone, Debug)]
pub struct RigParams {
    /// Soft travel guard; `None` disables (end-stop experiments stall at
    /// the physical ends on purpose - see [`RigParams::without_pos_guard`]).
    pub pos_guard: Option<(u16, u16)>,
    /// Abort threshold on the window's current mean, counts.
    pub i_abort: i16,
    /// The servo's window floor, q15. The abort judges the current mean
    /// only while `|duty_mean|` is at or over it: under it, and at torque
    /// off, the servo repeats the last current it measured. A drive under
    /// the floor is stall-safe anyway: the firmware's limiter is as blind
    /// there and holds the duty to the one a stall draws the limit at. 0, a
    /// servo that publishes none, judges every nonzero duty.
    pub window_floor_q15: u16,
    /// A stretch of travel left out of the motion fits and avoided as a
    /// dwell region, for a servo with a damaged spot in its train (consumed
    /// by the ladder/inertia experiments). None by default.
    pub slip: Option<(u16, u16)>,
    /// Windows discarded after every duty change (L transient +
    /// window-boundary smear).
    pub settle_windows: u32,
    /// End-stop stall detect: a seek read whose pos moved <= `stall_eps`
    /// counts from the prior one counts as still; `stall_polls` consecutive
    /// still reads declare the mechanical rail.
    pub stall_eps: u16,
    pub stall_polls: u32,
    /// The counts the kernel controls on: raw, or the servo's LIVE table.
    /// Fits against pot motion read through it; seeks, stall detects and
    /// the guard stay on the raw count (identity at and beyond the stops).
    pub pot: Pot,
    /// The pot stops, `raw_min` and `raw_max`: where a seek toward a stop
    /// must come to rest. None until `osc cal` found them.
    pub stops: Option<(u16, u16)>,
    /// `drive_polarity` in force: which terminal a burst step drives.
    pub drive_polarity: bool,
}

impl RigParams {
    pub fn new(pos_guard: Option<(u16, u16)>, i_abort: i16) -> Self {
        Self {
            pos_guard,
            i_abort,
            window_floor_q15: 0,
            slip: None,
            settle_windows: 5,
            stall_eps: seek::STALL_EPS,
            stall_polls: seek::STALL_POLLS,
            pot: Pot::RAW,
            stops: None,
            drive_polarity: true,
        }
    }

    pub fn with_polarity(self, drive_polarity: bool) -> Self {
        Self {
            drive_polarity,
            ..self
        }
    }

    pub fn with_stops(self, stops: (u16, u16)) -> Self {
        Self {
            stops: Some(stops),
            ..self
        }
    }

    pub fn with_floor(self, window_floor_q15: u16) -> Self {
        Self {
            window_floor_q15,
            ..self
        }
    }

    pub fn without_pos_guard(self) -> Self {
        Self {
            pos_guard: None,
            ..self
        }
    }

    /// Abort on the servo's soft limits, `soft`, rather than the guard: a
    /// free-running drive plans its brake points on the guard, and one that
    /// brakes a poll late lands between the two. The firmware brakes at
    /// the soft limits anyway.
    pub fn abort_at_soft(self, soft: (i32, i32)) -> Self {
        let clamp = |p: i32| p.clamp(0, u16::MAX as i32) as u16;
        Self {
            pos_guard: Some((clamp(soft.0), clamp(soft.1))),
            ..self
        }
    }

    pub fn in_slip(&self, pos: u16) -> bool {
        self.slip.is_some_and(|(lo, hi)| (lo..=hi).contains(&pos))
    }
}

/// Why the envelope stopped an experiment.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AbortReason {
    /// The servo latched a fault (flags + code as read).
    Fault { flags: u8, code: u8 },
    /// Position left the soft guard band.
    PosGuard { pos: u16 },
    /// The window's current mean exceeded the abort threshold mid-drive.
    Overcurrent { i_mean: i16 },
    /// A seek came to rest short of anywhere it could stop: jammed, or the
    /// pot is not reading. `moved` is the travel since the seek started.
    Blocked { pos: u16, moved: u16 },
    /// The jam check drove the shaft each way up to `ma` and it never
    /// moved: jammed, or stiffer than the gears' probe current.
    Stiff { pos: u16, ma: u16 },
    /// The jam check, near a stop past a soft limit where the endstop
    /// allows one way only, drove the shaft up to `ma` that way and it did
    /// not move: that way may press the stop, so the direction is
    /// unproven.
    AtStop { pos: u16, ma: u16 },
    /// The motor turned both ways and the pot did not move.
    SensorDead { pos: u16 },
    /// No current flows into the motor at the jam check's backstop.
    MotorOpen { pos: u16 },
    /// The servo dropped `rows` TEL rows from a burst.
    RowsDropped { rows: u16 },
}

impl core::fmt::Display for AbortReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AbortReason::Fault { flags, code } => {
                write!(
                    f,
                    "the servo latched a fault (flags {flags:#04x}, code {code})"
                )
            }
            AbortReason::PosGuard { pos } => {
                write!(f, "the shaft left the travel guard at pos {pos}")
            }
            AbortReason::Overcurrent { i_mean } => write!(
                f,
                "the current reached {} counts, over the abort threshold",
                i_mean.unsigned_abs()
            ),
            AbortReason::Blocked { pos, moved } => write!(
                f,
                "the shaft is blocked or the position sensor is not reading (pos {pos}, moved \
                 {moved} counts)"
            ),
            AbortReason::Stiff { pos, ma } => write!(
                f,
                "the shaft did not move either way at up to {ma} mA (pos {pos}): it is jammed, or \
                 stiffer than the probe current allows"
            ),
            AbortReason::AtStop { pos, ma } => write!(
                f,
                "the shaft rests against an end stop and did not move at up to {ma} mA (pos \
                 {pos}): the one way the endstop allows may press the stop, so which way the \
                 motor turns is unproven"
            ),
            AbortReason::SensorDead { pos } => write!(
                f,
                "the motor turns but the position sensor does not move (pos {pos}): the pot is not \
                 reading, or the output gear slips"
            ),
            AbortReason::MotorOpen { pos } => write!(
                f,
                "no current flows into the motor (pos {pos}): a motor lead or the winding is open"
            ),
            AbortReason::RowsDropped { rows } => write!(
                f,
                "the servo dropped {rows} TEL rows: both stream buffers were waiting for the wire"
            ),
        }
    }
}

impl std::error::Error for AbortReason {}

enum GuardState {
    Run,
    /// A watch flagged a fault: one read for its flags and code.
    FaultRead,
    DutyOff,
    TorqueOff,
    PermitOff,
    Finished,
}

/// Safety envelope: checks every observation, and on a violation preempts
/// the inner experiment with duty-0 + torque-off before reporting Done.
/// The inner experiment is left unstepped from that point on, so a stall
/// permit it granted is withdrawn here: the envelope watches the writes.
/// It judges every watch as it lands - each row's pot against the travel
/// guard, each window's current mean against the abort, the ALERT flag -
/// and every read; the driver ends a pause at the first trip.
pub struct Guarded<E> {
    exp: E,
    params: RigParams,
    state: GuardState,
    abort: Option<AbortReason>,
    permit: bool,
}

impl<E: Experiment> Guarded<E> {
    pub fn new(exp: E, params: RigParams) -> Self {
        Self {
            exp,
            params,
            state: GuardState::Run,
            abort: None,
            permit: false,
        }
    }

    /// `Some` once the envelope has tripped; the run's samples are partial.
    pub fn abort(&self) -> Option<AbortReason> {
        self.abort
    }

    pub fn into_inner(self) -> E {
        self.exp
    }

    fn violation(&self, o: &TelemetrySnapshot) -> Option<AbortReason> {
        if o.fault_flags != 0 {
            return Some(AbortReason::Fault {
                flags: o.fault_flags,
                code: o.fault_code,
            });
        }
        if let Some((lo, hi)) = self.params.pos_guard
            && !(lo..=hi).contains(&o.pos)
        {
            return Some(AbortReason::PosGuard { pos: o.pos });
        }
        self.over_current(o.i_mean_counts, o.duty_mean_counts, o.pwm_period_ticks)
    }

    /// The current mean measures only over the window floor: under it the
    /// servo repeats the last current it measured.
    fn over_current(&self, i_mean: i16, duty: i16, pwm: u16) -> Option<AbortReason> {
        let floor = duty_counts(self.params.window_floor_q15 as i16, pwm);
        (duty != 0
            && duty.unsigned_abs() >= floor.unsigned_abs()
            && i_mean.unsigned_abs() > self.params.i_abort.unsigned_abs())
        .then_some(AbortReason::Overcurrent { i_mean })
    }

    fn watch_violation(&self, w: &Watch) -> Option<AbortReason> {
        if w.alert {
            return Some(AbortReason::Fault { flags: 0, code: 0 });
        }
        if let Some((lo, hi)) = self.params.pos_guard
            && let Some((min, max)) = w.pos
        {
            if min < lo {
                return Some(AbortReason::PosGuard { pos: min });
            }
            if max > hi {
                return Some(AbortReason::PosGuard { pos: max });
            }
        }
        w.windows.iter().find_map(|x| {
            self.over_current(x.window.i_mean, x.window.duty_mean, w.pwm_period_ticks)
        })
    }
}

impl<E: Experiment> Experiment for Guarded<E> {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        if matches!(self.state, GuardState::Run)
            && let Some(reason) = self
                .abort
                .or_else(|| self.exp.halted())
                .or_else(|| obs.and_then(|o| self.violation(o)))
        {
            self.abort = Some(reason);
            self.state = if reason == (AbortReason::Fault { flags: 0, code: 0 }) {
                GuardState::FaultRead
            } else {
                GuardState::DutyOff
            };
        }
        if let (GuardState::FaultRead, Some(o)) = (&self.state, obs) {
            self.abort = Some(AbortReason::Fault {
                flags: o.fault_flags,
                code: o.fault_code,
            });
            self.state = GuardState::DutyOff;
        }
        match self.state {
            GuardState::Run => match self.exp.step(obs) {
                Cmd::Done => {
                    self.state = GuardState::Finished;
                    Cmd::Done
                }
                cmd => {
                    if let Cmd::Write { reg, value } = cmd
                        && reg == control::STALL_PERMIT
                    {
                        self.permit = value != 0;
                    }
                    cmd
                }
            },
            GuardState::FaultRead => Cmd::Read,
            GuardState::DutyOff => {
                self.state = GuardState::TorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            GuardState::TorqueOff => {
                self.state = if self.permit {
                    GuardState::PermitOff
                } else {
                    GuardState::Finished
                };
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            GuardState::PermitOff => {
                self.state = GuardState::Finished;
                self.permit = false;
                Cmd::Write {
                    reg: control::STALL_PERMIT,
                    value: 0,
                }
            }
            GuardState::Finished => Cmd::Done,
        }
    }

    /// A burst with dropped rows ends the run, its frames withheld.
    fn push_tel(&mut self, tel: &TelBurst) {
        match tel.rows_dropped {
            0 => self.exp.push_tel(tel),
            rows => self.abort = Some(AbortReason::RowsDropped { rows }),
        }
    }

    fn push_burst(&mut self, cap: &Capture) {
        self.exp.push_burst(cap);
    }

    fn push_watch(&mut self, w: &Watch) {
        if matches!(self.state, GuardState::Run) && self.abort.is_none() {
            self.abort = self.watch_violation(w);
        }
        self.exp.push_watch(w);
    }

    fn halted(&self) -> Option<AbortReason> {
        self.abort
    }
}

/// Holds the stall permit for an experiment that stalls on purpose: the
/// permit follows every torque enable the experiment writes (written with
/// torque off it grants nothing) and is withdrawn once the experiment is
/// done. Wrap it in [`Guarded`], which withdraws it on an abort.
pub struct Permitted<E> {
    exp: E,
    grant: bool,
    granted: bool,
    done: bool,
}

impl<E: Experiment> Permitted<E> {
    pub fn new(exp: E) -> Self {
        Self {
            exp,
            grant: false,
            granted: false,
            done: false,
        }
    }

    pub fn into_inner(self) -> E {
        self.exp
    }
}

impl<E: Experiment> Experiment for Permitted<E> {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        if self.grant {
            self.grant = false;
            self.granted = true;
            return Cmd::Write {
                reg: control::STALL_PERMIT,
                value: 1,
            };
        }
        if self.done {
            return Cmd::Done;
        }
        let cmd = self.exp.step(obs);
        match cmd {
            Cmd::Write { reg, value } if reg == control::TORQUE_ENABLE && value != 0 => {
                self.grant = true;
            }
            Cmd::Done if self.granted => {
                self.done = true;
                return Cmd::Write {
                    reg: control::STALL_PERMIT,
                    value: 0,
                };
            }
            _ => {}
        }
        cmd
    }

    fn push_tel(&mut self, tel: &TelBurst) {
        self.exp.push_tel(tel);
    }

    fn push_burst(&mut self, cap: &Capture) {
        self.exp.push_burst(cap);
    }

    fn push_watch(&mut self, w: &Watch) {
        self.exp.push_watch(w);
    }

    fn halted(&self) -> Option<AbortReason> {
        self.exp.halted()
    }
}

/// `limit_flags` bit 0: the current limit holds the applied duty under the
/// goal.
pub const LIMIT_GOVERNING: u8 = 1 << 0;

/// `limit_flags` bit 1: the stall timer folded the limit to the yield. A
/// drive that sees it is held against something it cannot move.
pub const LIMIT_YIELD_FOLDED: u8 = 1 << 1;

/// `limit_flags` bit 2: a soft limit closed one side of the band, the low
/// side under mid travel, whichever way the duty points.
pub const LIMIT_ENDSTOP: u8 = 1 << 2;

/// The firmware says the drive is held against something it cannot move:
/// the stall fold, or the current limit governing a shaft that a whole
/// watch window found `still`. The fold alone can go unseen - with a
/// release over what a held stall reads it shows for one MEDIUM tick per
/// trip - and a raised goal adds no current the limit holds back. A
/// governed shaft that moves is climbing, never held.
pub fn limit_holds(o: &TelemetrySnapshot, still: bool) -> bool {
    o.limit_flags & LIMIT_YIELD_FOLDED != 0 || (still && o.limit_flags & LIMIT_GOVERNING != 0)
}

/// What a rung or step with no clean window reports.
pub const GOVERNED: &str = "declined: the current limit governed this window";

/// How far the firmware's duty ceiling climbs per fast tick after a goal
/// change, q15.
pub const SLEW_Q15_PER_TICK: u32 = 128;

/// One window, ms, on a servo ticking at `tick_hz`.
pub fn window_ms(tick_hz: f64) -> f64 {
    WINDOW_TICKS as f64 * 1000.0 / tick_hz
}

/// Ticks the applied duty may take to reach `goal` from `start` before the
/// limiter, not the slew, is holding it back (protocol sec 5.8), counted
/// from the goal's commit: the slew, two ticks of sample alignment and
/// rounding, and one medium period, since a goal lands at the first medium
/// tick after its commit. `start` is the duty applied before the change;
/// pass 0 from rest or across a reversal, where the firmware starts from
/// the window floor: 0 only widens the allowance.
pub fn slew_ticks(goal: i16, start: i16) -> u32 {
    let start = if start.signum() == goal.signum() {
        start.unsigned_abs() as u32
    } else {
        0
    };
    (goal.unsigned_abs() as u32).saturating_sub(start) / SLEW_Q15_PER_TICK + 2 + DECIM_MED as u32
}

/// The least Q15 magnitude that drives `counts` at `pwm_period_ticks`
/// (`duty_counts`' inverse edge), sign kept: a start read low only widens
/// the slew allowance.
fn least_q15(counts: i16, pwm_period_ticks: u16) -> i16 {
    let c = counts.unsigned_abs() as u32;
    if c == 0 || pwm_period_ticks == 0 {
        return 0;
    }
    let need = (c << 15) - (1 << 14);
    let mag = need.div_ceil(pwm_period_ticks as u32).min(i16::MAX as u32) as i16;
    if counts < 0 { -mag } else { mag }
}

/// One sample's applied duty against the goal it was commanded to.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Applied {
    /// Still climbing inside the slew the firmware allows: trimmed.
    Slew,
    /// The goal, as commanded: fitted.
    Clean,
    /// Held under the goal by the current limit: past the slew without
    /// reaching it, or fallen under it after: never fitted.
    Governed,
}

/// Judge a TEL series after a goal change: each sample's tick counted from
/// the change and its applied duty, compare counts at `pwm_period_ticks`.
/// `goal` is the commanded Q15, `start` the counts applied before it.
pub fn judge(
    samples: impl IntoIterator<Item = (u64, i16)>,
    goal: i16,
    start: i16,
    pwm_period_ticks: u16,
) -> Vec<Applied> {
    let budget = slew_ticks(goal, least_q15(start, pwm_period_ticks)) as u64;
    let goal = duty_counts(goal, pwm_period_ticks);
    let mut reached = false;
    samples
        .into_iter()
        .map(|(tick, duty)| {
            reached |= duty == goal;
            match (duty == goal, reached) {
                (true, _) => Applied::Clean,
                (false, false) if tick <= budget => Applied::Slew,
                _ => Applied::Governed,
            }
        })
        .collect()
}

/// A judged series with no clean sample that the limiter governed.
pub fn declined(judged: &[Applied]) -> bool {
    !judged.contains(&Applied::Clean) && judged.contains(&Applied::Governed)
}

/// The goal a [`WindowStream`] judges windows against since its last mark.
#[derive(Copy, Clone, Debug)]
struct Goal {
    q15: i16,
    /// Windows from the first one seen that the slew may still cover; the
    /// first may have opened before the write.
    slew_windows: u64,
    first: Option<u64>,
    reached: bool,
    clean: usize,
    governed: usize,
}

/// One accepted window.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindowSample {
    /// When the host read it ([`TelemetrySnapshot::host_ms`]), never the
    /// window number: each read stalls the kernel for a few ticks, so
    /// under polling `agg_seq` runs slow of real time.
    pub t_ms: f64,
    /// Signed bias-subtracted window current mean, counts.
    pub i: f64,
    /// Drive-window va - vb mean, vcounts.
    pub vdiff: f64,
    /// Applied duty mean over the window, q15.
    pub duty_q15: f64,
}

/// Turns polled snapshots into deduplicated window samples: a sample is
/// accepted only when `agg_seq` advanced (a read that formed no window
/// yields None), and the first `settle` windows after every duty transition
/// are discarded. `agg_seq` numbers windows and judges the slew, both in
/// the servo's own ticks; the sample's time is the host's.
///
/// Marked with an OpenLoop goal ([`WindowStream::mark_goal`]) it also
/// accepts only windows whose applied duty is the goal's: the climb is
/// trimmed, and a window the limiter held under the goal is declined.
pub struct WindowStream {
    unwrap: SeqUnwrap,
    last: Option<u64>,
    settle: u32,
    settle_windows: u32,
    goal: Option<Goal>,
    /// The least Q15 that applies the last window's duty mean.
    applied: i16,
    /// Windows watched since the last [`WindowStream::drain`].
    pending: Vec<Watched>,
    pwm_period_ticks: u16,
}

impl WindowStream {
    pub fn new(params: &RigParams) -> Self {
        Self {
            unwrap: SeqUnwrap::default(),
            last: None,
            settle: 0,
            settle_windows: params.settle_windows,
            goal: None,
            applied: 0,
            pending: Vec::new(),
            pwm_period_ticks: 0,
        }
    }

    /// Queue a watch's windows for the next [`WindowStream::drain`].
    pub fn watch(&mut self, w: &Watch) {
        self.pending.extend_from_slice(&w.windows);
        self.pwm_period_ticks = w.pwm_period_ticks;
    }

    /// The windows watched since the last drain that the stream accepts,
    /// each with its last row's pot reading when the watch carries it.
    pub fn drain(&mut self) -> Vec<(WindowSample, Option<u16>)> {
        let pending = core::mem::take(&mut self.pending);
        let pwm = self.pwm_period_ticks;
        pending
            .iter()
            .filter_map(|x| Some((self.accept(x.seq, x.t_ms, &x.window, pwm)?, x.pos)))
            .collect()
    }

    /// A mark starts from the duty last watched; windows queued before it
    /// belong to the drive it ends.
    fn flush(&mut self) {
        if let Some(x) = self.pending.last() {
            self.applied = least_q15(x.window.duty_mean, self.pwm_period_ticks);
        }
        self.pending.clear();
    }

    /// Call on every duty change whose goal is not an OpenLoop duty; the
    /// next `settle_windows` accepted windows are dropped.
    pub fn mark_transition(&mut self) {
        self.flush();
        self.settle = self.settle_windows;
        self.goal = None;
    }

    /// Call on every OpenLoop goal change, with the goal written. A goal
    /// that follows another slews from the duty last seen applied;
    /// otherwise from rest.
    pub fn mark_goal(&mut self, goal_q15: i16) {
        self.flush();
        let start = if self.goal.is_some() { self.applied } else { 0 };
        let ticks = slew_ticks(goal_q15, start);
        self.settle = self.settle_windows;
        self.goal = Some(Goal {
            q15: goal_q15,
            slew_windows: ticks.div_ceil(WINDOW_TICKS as u32) as u64 + 1,
            first: None,
            reached: false,
            clean: 0,
            governed: 0,
        });
    }

    /// The goal since the last mark produced no clean window and the
    /// limiter governed it.
    pub fn declined(&self) -> bool {
        self.goal.is_some_and(|g| g.clean == 0 && g.governed > 0)
    }

    /// The window a snapshot carries, keyed by its `agg_seq`.
    pub fn push(&mut self, o: &TelemetrySnapshot) -> Option<WindowSample> {
        let seq = self.unwrap.push(o.agg_seq);
        let w = Window {
            i_mean: o.i_mean_counts,
            i_min: o.i_min_counts,
            i_max: o.i_max_counts,
            vdiff_mean: o.vdiff_mean,
            duty_mean: o.duty_mean_counts,
        };
        self.accept(seq, o.host_ms, &w, o.pwm_period_ticks)
    }

    fn accept(&mut self, seq: u64, t_ms: f64, w: &Window, pwm: u16) -> Option<WindowSample> {
        if self.last == Some(seq) {
            return None;
        }
        self.last = Some(seq);
        self.applied = least_q15(w.duty_mean, pwm);
        let judged = self.goal.as_mut().map(|g| {
            let first = *g.first.get_or_insert(seq);
            let clean = w.duty_mean == duty_counts(g.q15, pwm);
            g.reached |= clean;
            match (clean, g.reached) {
                (true, _) => Applied::Clean,
                (false, false) if seq - first < g.slew_windows => Applied::Slew,
                _ => Applied::Governed,
            }
        });
        if self.settle > 0 {
            self.settle -= 1;
            return None;
        }
        if let Some(g) = self.goal.as_mut() {
            match judged {
                Some(Applied::Clean) => g.clean += 1,
                Some(Applied::Governed) => {
                    g.governed += 1;
                    return None;
                }
                _ => return None,
            }
        }
        Some(WindowSample {
            t_ms,
            i: w.i_mean as f64,
            vdiff: w.vdiff_mean as f64,
            duty_q15: counts_q15(w.duty_mean, pwm),
        })
    }
}

#[cfg(any(test, feature = "testkit"))]
pub mod testkit;

#[cfg(test)]
mod tests {
    use super::bias::{Bias, BiasCfg};
    use super::centre::Centre;
    use super::endstop::Endstop;
    use super::ladder::{Declined, Ladder, LadderCfg};
    use super::testkit::{
        Bus, FakeServo, PWM_PERIOD_TICKS as ARR, TICK_HZ, bench_mg90, bench_mg90_saved, pump,
    };
    use super::*;
    use crate::runway::Runway;
    use crate::window::{WATCH_BURST_MS, Windows};

    #[test]
    fn repeated_agg_seq_no_duplicate_sample() {
        let params = crate::exp::testkit::rig();
        let mut ws = WindowStream::new(&params);
        let mut o = TelemetrySnapshot {
            agg_seq: 7,
            i_mean_counts: 42,
            ..Default::default()
        };
        assert!(ws.push(&o).is_some());
        assert!(ws.push(&o).is_none(), "repeated agg_seq must not resample");
        o.agg_seq = 8;
        assert!(ws.push(&o).is_some());
    }

    #[test]
    fn settle_windows_discarded_after_transition() {
        let params = crate::exp::testkit::rig();
        let mut ws = WindowStream::new(&params);
        ws.mark_transition();
        let mut accepted = 0;
        for seq in 0..8u16 {
            let o = TelemetrySnapshot {
                agg_seq: seq,
                ..Default::default()
            };
            if ws.push(&o).is_some() {
                accepted += 1;
            }
        }
        assert_eq!(accepted, 3, "5 settle windows dropped from 8");
    }

    /// Windows at seq 0.. carrying `duties`, through a stream marked with
    /// `goal`; what it accepted, by seq.
    fn stream(goal: i16, duties: &[i16]) -> (WindowStream, Vec<u16>) {
        let params = RigParams {
            settle_windows: 0,
            ..crate::exp::testkit::rig()
        };
        let mut ws = WindowStream::new(&params);
        ws.mark_goal(goal);
        let kept = duties
            .iter()
            .enumerate()
            .filter_map(|(seq, d)| {
                let o = TelemetrySnapshot {
                    agg_seq: seq as u16,
                    duty_mean_counts: duty_counts(*d, ARR),
                    pwm_period_ticks: ARR,
                    ..Default::default()
                };
                ws.push(&o).map(|_| seq as u16)
            })
            .collect();
        (ws, kept)
    }

    /// A window the limiter held under the goal after the goal was reached
    /// is never handed to a fit; the windows around it are.
    #[test]
    fn governed_window_is_declined_never_fitted() {
        let (ws, kept) = stream(8000, &[6000, 8000, 8000, 7400, 7900, 8000]);
        assert_eq!(kept, [1, 2, 5]);
        assert!(!ws.declined(), "clean windows remain");
        // -7970 applies one compare count under -8000; -7999 applies the same
        let (ws, kept) = stream(-8000, &[-6000, -8000, -7970, -7999]);
        assert_eq!(kept, [1, 3]);
        assert!(!ws.declined());
        let (ws, kept) = stream(-8000, &[-6000, -8000, -7970]);
        assert_eq!(kept, [1]);
        assert!(!ws.declined());
    }

    /// 64% from rest climbs for 6.5 ms, nine windows: the climb is
    /// trimmed, not counted against the goal.
    #[test]
    fn slew_samples_are_trimmed_not_declined() {
        let goal = 20971;
        assert_eq!(slew_ticks(goal, 0), 175);
        let climb: Vec<i16> = (1..=9).map(|k| (4369 + 2048 * k).min(20000)).collect();
        let mut duties = climb.clone();
        duties.extend([goal; 3]);
        let (ws, kept) = stream(goal, &duties);
        assert_eq!(kept, [9, 10, 11]);
        assert!(!ws.declined());
        // cut short inside the slew: nothing to fit, nothing declined
        let (ws, kept) = stream(goal, &climb[..5]);
        assert!(kept.is_empty() && !ws.declined());
        // the same rule per TEL sample
        let c = |q15| duty_counts(q15, ARR);
        let tel = judge(
            [
                (1, c(4497)),
                (100, c(17169)),
                (165, c(20000)),
                (166, c(goal)),
            ],
            goal,
            0,
            ARR,
        );
        assert_eq!(
            tel,
            [Applied::Slew, Applied::Slew, Applied::Slew, Applied::Clean]
        );
        // a goal cut applies at once, once it lands
        assert_eq!(slew_ticks(8000, 12000), 12);
        // a start read from counts is its least Q15: the allowance widens
        assert_eq!(least_q15(duty_counts(12000, ARR), ARR), 11974);
        assert_eq!(duty_counts(11974, ARR), duty_counts(12000, ARR));
        assert_eq!(duty_counts(11973, ARR), duty_counts(12000, ARR) - 1);
        assert_eq!(least_q15(-1, ARR), -14);
        assert_eq!((least_q15(0, ARR), least_q15(5, 0)), (0, 0));
        assert_eq!(slew_ticks(-8000, 12000), 74);
    }

    /// The limiter holds the climb at the current limit well past the
    /// slew, then the shaft's speed lets the duty reach the goal: the
    /// governed climb is trimmed and every settled window fits.
    #[test]
    fn governed_climb_is_trimmed_and_the_settled_windows_fit() {
        let goal = 20971;
        let mut duties = vec![9000; 20];
        duties.extend((0..10).map(|k| 9000 + 1100 * k));
        duties.extend([goal; 6]);
        let (ws, kept) = stream(goal, &duties);
        assert_eq!(kept, (30..36).collect::<Vec<u16>>());
        assert!(!ws.declined());
        let tel: Vec<(u64, i16)> = duties
            .iter()
            .enumerate()
            .map(|(k, d)| (k as u64 * 16, duty_counts(*d, ARR)))
            .collect();
        let judged = judge(tel, goal, 0, ARR);
        assert!(judged[12..30].iter().all(|a| *a == Applied::Governed));
        assert!(judged[30..].iter().all(|a| *a == Applied::Clean));
        assert!(!declined(&judged));

        // never reaching it: nothing fits, and the goal is declined
        let (ws, kept) = stream(goal, &[9000; 20]);
        assert!(kept.is_empty() && ws.declined());
        let judged = judge(
            (0..40).map(|k| (k * 16, duty_counts(9000, ARR))),
            goal,
            0,
            ARR,
        );
        assert!(declined(&judged));
    }

    /// The bench servo's 64% ladder rung from rest, polled every 2 ms: the
    /// windows handed on all carry the goal, and the climb the limiter held
    /// is trimmed off their front.
    #[test]
    fn a_free_running_governed_climb_fits_only_settled_windows() {
        let mut s = FakeServo::new(7270.0 / 4096.0);
        s.vbus = 3204.0;
        s.dynamic = true;
        s.ke = 0.1472;
        s.b = 0.296;
        s.fc = 80.0;
        s.fv = 0.01;
        s.pos = 600.0;
        s.ends = (209.0, 3849.0);
        s.current_limit = Some(280);
        s.transient_gain = 1.0;
        let params = crate::exp::testkit::rig();
        let mut ws = WindowStream::new(&params);
        let goal = 20971;
        s.write(control::TORQUE_ENABLE, 1);
        s.write(control::GOAL_DUTY, goal as i32);
        ws.mark_goal(goal);
        let mut windows = Windows::new(ARR);
        let (mut governed_reads, mut kept) = (0, Vec::new());
        for _ in 0..150 {
            s.advance(2);
            let o = s.read_window(&mut windows);
            governed_reads += (o.limit_flags & 1) as usize;
            if let Some(w) = ws.push(&o) {
                kept.push(w);
            }
        }
        assert!(governed_reads > 50, "the climb was never governed");
        assert!(kept.len() > 30, "{} windows", kept.len());
        let applied = counts_q15(duty_counts(goal, ARR), ARR);
        assert!(kept.iter().all(|w| w.duty_q15 == applied));
        assert!(!ws.declined());
    }

    #[test]
    fn abort_on_fault_emits_safety_commands() {
        // an experiment that would poll forever
        struct Forever(bool);
        impl Experiment for Forever {
            fn step(&mut self, _: Option<&TelemetrySnapshot>) -> Cmd {
                self.0 = !self.0;
                if self.0 {
                    Cmd::Read
                } else {
                    Cmd::Pause { ms: 5 }
                }
            }
        }
        let mut servo = FakeServo::new(3.37);
        let mut exp = Guarded::new(Forever(false), crate::exp::testkit::rig());
        servo.fault_at_ms = Some(50.0);
        let log = pump(&mut exp, &mut servo, 10_000);
        assert!(matches!(exp.abort(), Some(AbortReason::Fault { .. })));
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
    }

    /// A burst the servo dropped rows from is not evidence: the run ends on
    /// it in capture's words, torque off, and its frames never reach the
    /// fit. A whole burst runs to its result.
    #[test]
    fn a_burst_with_dropped_rows_ends_the_run() {
        let run = |rows_dropped| {
            let mut servo = FakeServo::new(3.37);
            servo.rows_dropped = rows_dropped;
            let params = crate::exp::testkit::rig();
            let mut exp = Guarded::new(Bias::new(BiasCfg::default(), &params), params);
            let log = pump(&mut exp, &mut servo, 10_000);
            assert!(!servo.torque);
            (exp.abort(), exp.into_inner().result(), log)
        };
        let (abort, r, _) = run(0);
        assert_eq!(abort, None);
        assert_eq!(r.expect("a result").tel_n, 10_000);

        let (abort, r, log) = run(3);
        assert_eq!(abort, Some(AbortReason::RowsDropped { rows: 3 }));
        assert_eq!(
            abort.unwrap().to_string(),
            "the servo dropped 3 TEL rows: both stream buffers were waiting for the wire"
        );
        assert!(r.is_none(), "the frames reached the fit");
        let at = log.iter().position(|l| l.starts_with("stream")).unwrap();
        assert_eq!(
            log[at + 1..],
            ["write goal_duty 0", "write torque_enable 0"]
        );
    }

    #[test]
    fn abort_on_pos_guard_breach() {
        struct Drive(u8);
        impl Experiment for Drive {
            fn step(&mut self, _: Option<&TelemetrySnapshot>) -> Cmd {
                self.0 += 1;
                match self.0 {
                    1 => Cmd::Write {
                        reg: control::TORQUE_ENABLE,
                        value: 1,
                    },
                    2 => Cmd::Write {
                        reg: control::GOAL_DUTY,
                        value: 12000,
                    },
                    _ => {
                        if self.0 % 2 == 1 {
                            Cmd::Read
                        } else {
                            Cmd::Pause { ms: 30 }
                        }
                    }
                }
            }
        }
        let mut servo = FakeServo::new(3.37);
        servo.pos = 3800.0;
        let mut exp = Guarded::new(Drive(0), crate::exp::testkit::rig());
        let log = pump(&mut exp, &mut servo, 10_000);
        assert!(matches!(exp.abort(), Some(AbortReason::PosGuard { pos } ) if pos > 3950));
        assert_eq!(
            log[log.len() - 2..],
            ["write goal_duty 0", "write torque_enable 0"]
        );
    }

    /// A scripted run: each entry is one command, a `Read` result kept.
    struct Script(Vec<Cmd>, Option<TelemetrySnapshot>);

    impl Experiment for Script {
        fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
            if let Some(o) = obs {
                self.1 = Some(*o);
            }
            if self.0.is_empty() {
                Cmd::Done
            } else {
                self.0.remove(0)
            }
        }
    }

    fn write(reg: Reg, value: i32) -> Cmd {
        Cmd::Write { reg, value }
    }

    /// Every read carries the window streamed just before it: at rest it
    /// reads no drive, after a goal the goal's compare count, numbered on
    /// past the first.
    #[test]
    fn every_read_carries_its_window() {
        let mut servo = FakeServo::new(3.37);
        let mut rest = Script(vec![Cmd::Read], None);
        pump(&mut rest, &mut servo, 100);
        let rest = rest.1.expect("the read");
        assert_eq!(
            (
                rest.duty_mean_counts,
                rest.i_mean_counts,
                rest.pwm_period_ticks
            ),
            (0, 0, ARR)
        );
        let mut exp = Guarded::new(
            Script(
                vec![
                    write(control::TORQUE_ENABLE, 1),
                    write(control::GOAL_DUTY, 6000),
                    Cmd::Pause { ms: 50 },
                    Cmd::Read,
                    write(control::GOAL_DUTY, 0),
                    write(control::TORQUE_ENABLE, 0),
                ],
                None,
            ),
            testkit::rig(),
        );
        pump(&mut exp, &mut servo, 100);
        let o = exp.into_inner().1.expect("the read");
        assert!(o.agg_seq > rest.agg_seq, "{o:?}");
        assert!(o.duty_is(6000) && o.i_mean_counts > 0, "{o:?}");
    }

    /// A long pause is watched in bursts, and the guard judges each as it
    /// lands: a shaft that crosses the travel guard mid-pause stops at the
    /// end of the burst it crossed in, never at the pause's end, and no
    /// read comes first.
    #[test]
    fn the_guard_stops_a_watched_drive_at_the_burst_it_crossed_in() {
        let mut servo = FakeServo::new(3.37);
        servo.free_speed = 1000.0;
        servo.pos = 3000.0;
        // 0.5 counts/ms: the guard at 3200 is crossed 400 ms in
        let duty = 16384;
        let mut exp = Guarded::new(
            Script(
                vec![
                    write(control::TORQUE_ENABLE, 1),
                    write(control::GOAL_DUTY, duty),
                    Cmd::Pause { ms: 3000 },
                    Cmd::Read,
                ],
                None,
            ),
            RigParams::new(Some((250, 3200)), i16::MAX),
        );
        let log = pump(&mut exp, &mut servo, 100);
        let reach = 0.5 * WATCH_BURST_MS as f64 * Bus::BENCH.pause_scale;
        assert!(
            matches!(exp.abort(), Some(AbortReason::PosGuard { pos }) if pos > 3200 && (pos as f64) <= 3200.0 + reach),
            "{:?}",
            exp.abort()
        );
        assert!(servo.t_ms < 1000.0, "the pause ran on to {}", servo.t_ms);
        assert_eq!(
            log[log.len() - 2..],
            ["write goal_duty 0", "write torque_enable 0"]
        );
        assert!(exp.into_inner().1.is_none(), "a read came first");
    }

    /// Every window of a watch is judged: a locked shaft over the abort
    /// stops at the first burst, before any read.
    #[test]
    fn the_guard_judges_every_watched_window() {
        let mut servo = FakeServo::new(3.37);
        servo.jam = Some(servo.pos);
        let mut exp = Guarded::new(
            Script(
                vec![
                    write(control::TORQUE_ENABLE, 1),
                    write(control::GOAL_DUTY, 12000),
                    Cmd::Pause { ms: 1000 },
                    Cmd::Read,
                ],
                None,
            ),
            RigParams::new(None, 150),
        );
        pump(&mut exp, &mut servo, 100);
        assert!(
            matches!(exp.abort(), Some(AbortReason::Overcurrent { i_mean }) if i_mean > 150),
            "{:?}",
            exp.abort()
        );
        assert!(servo.t_ms < 400.0, "{}", servo.t_ms);
        assert!(exp.into_inner().1.is_none());
    }

    /// A watch frame flagged a fault: the guard reads its flags and code,
    /// then stops the drive, the pause cut short.
    #[test]
    fn a_fault_mid_watch_reads_its_code_then_stops() {
        let mut servo = FakeServo::new(3.37);
        servo.fault_at_ms = Some(100.0);
        let mut exp = Guarded::new(
            Script(
                vec![
                    write(control::TORQUE_ENABLE, 1),
                    write(control::GOAL_DUTY, 6000),
                    Cmd::Pause { ms: 3000 },
                ],
                None,
            ),
            testkit::rig(),
        );
        let log = pump(&mut exp, &mut servo, 100);
        assert_eq!(exp.abort(), Some(AbortReason::Fault { flags: 32, code: 6 }));
        assert!(servo.t_ms < 1000.0, "{}", servo.t_ms);
        assert_eq!(
            log[log.len() - 2..],
            ["write goal_duty 0", "write torque_enable 0"]
        );
    }

    /// Pushed against the low soft limit at a stop: only a live permit
    /// lets the outbound duty through.
    fn at_the_low_limit() -> FakeServo {
        let mut s = FakeServo::new(3.37);
        s.ends = (230.0, 4000.0);
        s.soft = Some((230.0, 3970.0));
        s.pos = 230.0;
        s.lease_ms = Some(1008.0);
        s
    }

    #[test]
    fn permit_follows_torque_on() {
        let mut exp = Permitted::new(Script(
            vec![
                write(control::TORQUE_ENABLE, 1),
                write(control::GOAL_DUTY, -3000),
                write(control::TORQUE_ENABLE, 0),
                write(control::TORQUE_ENABLE, 1),
                Cmd::Read,
                write(control::TORQUE_ENABLE, 0),
            ],
            None,
        ));
        let mut servo = at_the_low_limit();
        let log = pump(&mut exp, &mut servo, 100);
        assert_eq!(
            log,
            [
                "write torque_enable 1",
                "write stall_permit 1",
                "write goal_duty -3000",
                "write torque_enable 0",
                "write torque_enable 1",
                "write stall_permit 1",
                "write torque_enable 0",
                "write stall_permit 0",
            ]
        );
        let o = exp.into_inner().1.expect("the read");
        assert_eq!(
            o.duty_ticks,
            duty_counts(-3000, ARR),
            "re-granted after the re-enable"
        );
    }

    #[test]
    fn long_pause_is_sliced_under_the_lease() {
        let mut exp = Permitted::new(Script(
            vec![
                write(control::TORQUE_ENABLE, 1),
                write(control::GOAL_DUTY, -3000),
                Cmd::Pause { ms: 3000 },
                Cmd::Read,
                write(control::GOAL_DUTY, 0),
                write(control::TORQUE_ENABLE, 0),
            ],
            None,
        ));
        let mut servo = at_the_low_limit();
        let log = pump(&mut exp, &mut servo, 100);
        let grants = log.iter().filter(|l| *l == "write stall_permit 1").count();
        assert_eq!(grants, 1 + 3000 / crate::limits::PERMIT_REFRESH_MS as usize);
        let o = exp.into_inner().1.expect("the read");
        assert_eq!(
            o.duty_ticks,
            duty_counts(-3000, ARR),
            "the lease lapsed inside the pause"
        );
        assert_eq!(log.last().map(String::as_str), Some("write stall_permit 0"));
    }

    /// A locked shaft asked for 64% for two seconds: the firmware limiter
    /// holds it at the limit, window peaks a tenth over. An abort at the
    /// limit trips on that; the default, a quarter over, never does.
    #[test]
    fn abort_default_clears_a_held_stall() {
        let lim = crate::limits::ServoLimits {
            i_lim: 280,
            i_cont: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 0,
            vbus: 1731,
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: 0.0,
            drive_polarity: true,
        };
        let hold = |i_abort: i16| {
            let mut s = FakeServo::new(3.37);
            s.current_limit = Some(lim.i_lim);
            s.hold_ripple = 0.10;
            s.transient_gain = 1.0;
            s.jam = Some(s.pos);
            let mut drive = vec![
                write(control::TORQUE_ENABLE, 1),
                write(control::GOAL_DUTY, 20971),
            ];
            for _ in 0..200 {
                drive.extend([Cmd::Pause { ms: 10 }, Cmd::Read]);
            }
            let mut exp = Guarded::new(Script(drive, None), RigParams::new(None, i_abort));
            pump(&mut exp, &mut s, 10_000);
            (exp.abort(), exp.into_inner().1)
        };
        let env = lim.envelope((None, None), None).unwrap();
        let (abort, last) = hold(env.i_abort);
        assert_eq!(abort, None);
        let o = last.expect("the hold was read");
        assert!(
            o.i_mean_counts > lim.i_lim as i16,
            "the hold reads over the limit"
        );
        assert!(o.duty_ticks < duty_counts(20971, ARR), "and is governed");
        assert!(matches!(
            hold(lim.i_lim as i16).0,
            Some(AbortReason::Overcurrent { .. })
        ));
    }

    /// The current mean measures only over the window floor: under it the
    /// servo repeats the last current it measured, which the guard leaves
    /// unjudged. A servo that publishes no floor has every duty the bridge
    /// applies judged; 13 in Q15 applies no compare count, 14 one.
    #[test]
    fn the_guard_judges_the_current_only_over_the_floor() {
        let judged = |floor: u16, duty: i16| {
            let o = TelemetrySnapshot {
                duty_mean_counts: duty_counts(duty, ARR),
                pwm_period_ticks: ARR,
                i_mean_counts: 351,
                ..Default::default()
            };
            Guarded::new(
                Script(Vec::new(), None),
                RigParams::new(None, 350).with_floor(floor),
            )
            .violation(&o)
        };
        let over = Some(AbortReason::Overcurrent { i_mean: 351 });
        for duty in [0, 3211, -3211, 4355] {
            assert_eq!(judged(4356, duty), None, "{duty}");
        }
        for duty in [4356, -4356, 20971] {
            assert_eq!(judged(4356, duty), over, "{duty}");
        }
        assert_eq!(judged(0, 0), None);
        assert_eq!(judged(0, 13), None);
        for duty in [14, 3211, -3211] {
            assert_eq!(judged(0, duty), over, "{duty}");
        }
    }

    /// A rung that ends over the abort leaves the servo's current reading
    /// there through a drive in the same direction under the window floor:
    /// a centring's first drive, 9.5%, repeats it. Judged over the floor,
    /// the centring frees the shaft and reaches mid travel; judged at
    /// every nonzero duty, it aborts the free shaft on the stale reading.
    #[test]
    fn a_stale_current_under_the_floor_is_no_abort() {
        let after_a_rung = || {
            let mut s = bench_mg90(3204);
            s.pos = 1400.0;
            s.current_limit = None;
            s.jam = Some(s.pos);
            let mut rung = Script(
                vec![
                    write(control::TORQUE_ENABLE, 1),
                    write(control::GOAL_DUTY, 6500),
                    Cmd::Pause { ms: 100 },
                    Cmd::Read,
                ],
                None,
            );
            pump(&mut rung, &mut s, 100);
            s.current_limit = Some(280);
            let mut under = Script(
                vec![
                    write(control::GOAL_DUTY, 3119),
                    Cmd::Pause { ms: 100 },
                    Cmd::Read,
                ],
                None,
            );
            pump(&mut under, &mut s, 100);
            s.jam = None;
            let o = under.1.expect("the read under the floor");
            assert!(o.duty_mean_counts != 0 && o.duty_mean_counts < duty_counts(s.floor_q15, ARR));
            assert!(o.i_mean_counts > 350, "held {}", o.i_mean_counts);
            s
        };

        let mut s = after_a_rung();
        let p = bench().with_floor(s.floor_q15 as u16);
        let run = seen(
            Centre::new(crate::run::centre_cfg(0.0952, 0.253), &p),
            p.without_pos_guard(),
            &mut s,
        );
        assert_eq!(run.exp.abort(), None);
        let floor = duty_counts(s.floor_q15, ARR);
        assert!(run.obs.iter().any(|o| o.duty_mean_counts != 0
            && o.duty_mean_counts < floor
            && o.i_mean_counts > 350));
        assert!(run.exp.into_inner().arrived());

        let run = centring(&mut after_a_rung());
        assert!(
            matches!(run.exp.abort(), Some(AbortReason::Overcurrent { i_mean }) if i_mean > 350),
            "{:?}",
            run.exp.abort()
        );
    }

    /// Every observation an experiment was stepped with.
    struct Seen<E> {
        exp: E,
        obs: Vec<TelemetrySnapshot>,
        log: Vec<String>,
    }

    impl<E: Experiment> Experiment for Seen<E> {
        fn push_watch(&mut self, w: &crate::window::Watch) {
            self.exp.push_watch(w);
        }

        fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
            self.obs.extend(obs.copied());
            self.exp.step(obs)
        }

        fn halted(&self) -> Option<AbortReason> {
            self.exp.halted()
        }
    }

    fn seen<E: Experiment>(exp: E, params: RigParams, servo: &mut FakeServo) -> Seen<Guarded<E>> {
        let mut s = Seen {
            exp: Guarded::new(exp, params),
            obs: Vec::new(),
            log: Vec::new(),
        };
        s.log = pump(&mut s, servo, 4_000_000);
        assert!(!s.log.contains(&"OVERRUN".to_string()));
        s
    }

    /// The bench servo's guard, soft limits, stops and abort.
    fn bench() -> RigParams {
        RigParams::new(Some((532, 3526)), 350).with_stops((209, 3849))
    }

    const SOFT: (i32, i32) = (432, 3626);

    /// A centring as the run starts one on the bench servo's 2S rail with
    /// nothing measured: from the bootstrap duty, capped at 2 V.
    fn centring(servo: &mut FakeServo) -> Seen<Guarded<Centre>> {
        let p = bench();
        seen(
            Centre::new(crate::run::centre_cfg(0.0952, 0.253), &p),
            p.without_pos_guard(),
            servo,
        )
    }

    fn bench_ladder(servo: &mut FakeServo) -> Seen<Guarded<Ladder>> {
        let p = bench();
        let cfg = LadderCfg {
            seek_duty_q15: 4915,
            ..LadderCfg::new(TICK_HZ)
        };
        seen(
            Ladder::new(cfg, &p, Runway::new((532, 3526))),
            p.abort_at_soft(SOFT),
            servo,
        )
    }

    /// The largest goal duty a log wrote.
    fn top_goal(log: &[String]) -> i32 {
        log.iter()
            .filter_map(|l| l.strip_prefix("write goal_duty ")?.parse::<i32>().ok())
            .map(i32::abs)
            .max()
            .unwrap()
    }

    /// Reads of a ladder's rungs, from its first read at a rung's goal.
    fn rung_reads(obs: &[TelemetrySnapshot]) -> &[TelemetrySnapshot] {
        let first = obs.iter().position(|o| o.duty_mean_q15() > 5000.0).unwrap();
        &obs[first..]
    }

    fn span_ms(obs: &[TelemetrySnapshot]) -> f64 {
        let (a, b) = (obs.first().unwrap(), obs.last().unwrap());
        b.host_ms - a.host_ms
    }

    /// The firmware's verdict without the fold: the current limit governing
    /// a shaft that a whole watch window found still. A centring, a ladder
    /// rung and the stop finder each take it as blocked at once, no
    /// raise tried and no fold ever shown; the limit governing a moving
    /// shaft, or a still shaft the limit does not govern, is no verdict.
    #[test]
    fn a_governing_limit_on_a_still_shaft_is_blocked() {
        let o = |flags| TelemetrySnapshot {
            limit_flags: flags,
            ..Default::default()
        };
        assert!(limit_holds(&o(LIMIT_GOVERNING), true));
        assert!(!limit_holds(&o(LIMIT_GOVERNING), false));
        assert!(!limit_holds(&o(0), true));
        assert!(limit_holds(&o(LIMIT_YIELD_FOLDED), false));

        let locked = |pos: f64, jam: f64| {
            let mut s = bench_mg90(3204);
            s.stall_ms = None;
            s.pos = pos;
            s.jam = Some(jam);
            s
        };
        // 9.5 and 12% sit under the window floor, 14.5% stalls at 262
        // counts inside the limit: all raised. 17% is the first the limit
        // governs, and the last; the cap is 25.3%.
        let mut s = locked(1400.0, 1400.0);
        let run = centring(&mut s);
        assert_eq!(
            run.exp.abort(),
            Some(AbortReason::Blocked {
                pos: 1400,
                moved: 0
            })
        );
        assert_eq!(top_goal(&run.log), 3119 + 3 * 819);
        assert!(
            run.obs
                .iter()
                .all(|o| o.limit_flags & LIMIT_YIELD_FOLDED == 0)
        );
        assert!(!s.torque);

        // a rung into a jam at 1000: blocked on its first still window,
        // well inside twice its predicted climb
        let mut s = locked(900.0, 1000.0);
        let run = bench_ladder(&mut s);
        assert!(matches!(
            run.exp.abort(),
            Some(AbortReason::Blocked { pos: 1000, .. })
        ));
        let climb_ms = run.exp.into_inner().sized()[0].1.climb_ms;
        let reads = rung_reads(&run.obs);
        assert!(
            span_ms(reads) < 2.0 * climb_ms,
            "{} of {climb_ms}",
            span_ms(reads)
        );
        assert_ne!(reads.last().unwrap().limit_flags & LIMIT_GOVERNING, 0);
        assert!(!s.torque);

        // the stop finder on a winding lower than planned, the shaft locked
        // where it stands: 12% sits under the floor and rises; 14.5% stalls
        // over the limit, which governs it, and the approach rises no
        // further
        let mut s = locked(2029.0, 2029.0);
        s.r = 1.5;
        let p = bench().without_pos_guard();
        let cfg = crate::run::endstop_cfg(0.12, 0.06, 0.20);
        let run = seen(Permitted::new(Endstop::new(cfg, &p)), p, &mut s);
        assert_eq!(
            run.exp.abort(),
            Some(AbortReason::Blocked {
                pos: 2029,
                moved: 0
            })
        );
        assert_eq!(top_goal(&run.log), 3932 + 819);
        assert!(!s.torque && !s.permit_live());
    }

    /// A governed climb moves. Every rung of the bench ladder climbs on the
    /// limit, dozens of reads governed, and none is taken for a blocked
    /// shaft, at either stall setting; nor is a centring whose shaft the
    /// limit governs as it moves, nor a climb a heavy load holds on the
    /// limit for good, which declines the ladder instead.
    #[test]
    fn a_governed_climb_is_never_blocked() {
        for mut s in [bench_mg90(3204), bench_mg90_saved(3204)] {
            s.pos = 2029.0;
            let run = bench_ladder(&mut s);
            assert_eq!(run.exp.abort(), None);
            let governed = run
                .obs
                .iter()
                .filter(|o| o.duty_mean_counts != 0 && o.limit_flags & LIMIT_GOVERNING != 0)
                .count();
            assert!(governed > 100, "{governed} governed reads");
            let exp = run.exp.into_inner();
            assert_eq!(exp.declined(), None);
            assert!(exp.sized().len() >= 10);
        }

        // a 15% breakaway and a load that holds the moving shaft on the
        // limit: 14.5% stays put, 17% moves half a count a ms, governed
        let mut s = bench_mg90(3204);
        s.pos = 1400.0;
        s.breakaway_q15 = 4915;
        s.fv = 0.3;
        let run = centring(&mut s);
        assert_eq!(run.exp.abort(), None);
        assert!(run.exp.into_inner().arrived());
        assert_eq!(top_goal(&run.log), 3119 + 3 * 819);
        assert!(run.obs.iter().any(|o| o.limit_flags & LIMIT_GOVERNING != 0));

        let mut s = bench_mg90(3204);
        s.pos = 2029.0;
        s.fv = 0.1;
        let run = bench_ladder(&mut s);
        assert_eq!(run.exp.abort(), None);
        assert!(matches!(
            run.exp.into_inner().declined(),
            Some(Declined::Heavy { .. })
        ));
    }

    /// The bench servo's saved stall settings fold nothing, and the verdict
    /// shows for a MEDIUM tick per trip, which no poll catches. A blocked
    /// drive still ends, on the limit governing a still shaft: a centring
    /// at the first duty the limit governs, a ladder rung on its first still
    /// window - never waiting for the fold.
    #[test]
    fn inert_stall_settings_still_end_a_blocked_drive() {
        let mut s = bench_mg90_saved(3204);
        s.pos = 1400.0;
        s.jam = Some(1400.0);
        let run = centring(&mut s);
        assert!(matches!(run.exp.abort(), Some(AbortReason::Blocked { .. })));
        assert_eq!(top_goal(&run.log), 3119 + 3 * 819);
        assert!(
            run.obs
                .iter()
                .all(|o| o.limit_flags & LIMIT_YIELD_FOLDED == 0)
        );
        assert!(!s.torque);

        let mut s = bench_mg90_saved(3204);
        s.pos = 900.0;
        s.jam = Some(1000.0);
        let run = bench_ladder(&mut s);
        assert!(matches!(
            run.exp.abort(),
            Some(AbortReason::Blocked { pos: 1000, .. })
        ));
        assert!(
            run.obs
                .iter()
                .all(|o| o.limit_flags & LIMIT_YIELD_FOLDED == 0)
        );
        let climb_ms = run.exp.into_inner().sized()[0].1.climb_ms;
        assert!(span_ms(rung_reads(&run.obs)) < 2.0 * climb_ms);
        assert!(!s.torque);
    }

    /// The governed rule compares the applied duty, never the current: a
    /// window whose duty mean is the goal is clean however far a tick of
    /// commutation ripple took its current over the limit, and whatever
    /// the limit flags say of it. Only a duty the limiter held under the
    /// goal declines a window.
    #[test]
    fn current_ripple_does_not_decline_a_clean_window() {
        let goal = 20971;
        let params = RigParams {
            settle_windows: 0,
            ..crate::exp::testkit::rig()
        };
        let mut ws = WindowStream::new(&params);
        ws.mark_goal(goal);
        let kept: Vec<WindowSample> = [(goal, 250, 0), (goal, 330, 1), (goal, 300, 1)]
            .iter()
            .enumerate()
            .filter_map(|(seq, &(duty, i, flags))| {
                ws.push(&TelemetrySnapshot {
                    agg_seq: seq as u16,
                    duty_mean_counts: duty_counts(duty, ARR),
                    pwm_period_ticks: ARR,
                    i_mean_counts: i,
                    limit_flags: flags,
                    ..Default::default()
                })
            })
            .collect();
        assert_eq!(kept.len(), 3);
        assert_eq!(kept[1].i, 330.0);
        assert!(!ws.declined());
        let held = ws.push(&TelemetrySnapshot {
            agg_seq: 3,
            duty_mean_counts: duty_counts(goal - 40, ARR),
            pwm_period_ticks: ARR,
            i_mean_counts: 270,
            ..Default::default()
        });
        assert!(held.is_none(), "a duty held under the goal is declined");
        assert!(!ws.declined(), "the clean windows remain");
        // the same rule per TEL sample
        assert_eq!(
            judge([(200, duty_counts(goal, ARR)); 2], goal, 0, ARR),
            [Applied::Clean, Applied::Clean]
        );
    }
}
