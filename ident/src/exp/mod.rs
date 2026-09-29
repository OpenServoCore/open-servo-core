//! Sans-io experiment engine. An [`Experiment`] is a state machine: each
//! `step` consumes at most one observation (the reply to a previous
//! [`Cmd::Read`]) and emits the next command. The driver - CLI over USB
//! today, wasm GUI over Web Serial later - owns all IO and time:
//!
//! ```ignore
//! let mut pending = None;
//! loop {
//!     match exp.step(pending.take().as_ref()) {
//!         Cmd::Write { reg, value } => client.write(reg, value),
//!         Cmd::Read => pending = Some(read_telemetry_region()),
//!         Cmd::Pause { ms } => sleep_ms(ms),
//!         Cmd::Stream { samples, goal } => exp.push_tel(&run_burst(samples, goal)),
//!         Cmd::Burst { duty_q15, pre_q15, chans, seated } => {
//!             exp.push_burst(&capture(duty_q15, pre_q15, chans, seated))
//!         }
//!         Cmd::Done => break,
//!     }
//! }
//! ```
//!
//! [`Guarded`] wraps any experiment with the safety envelope; rig limits
//! live in [`RigParams`], the one home for bench constants. [`Permitted`]
//! holds the stall permit for one that stalls on purpose, and the driver
//! keeps a held permit alive with [`crate::limits::PermitLease`].

pub mod bias;
pub mod breakaway;
pub mod centre;
pub mod endstop;
pub mod held;
pub mod inductance;
pub mod inertia;
pub mod ladder;
pub mod resistance;
pub mod rl;
pub mod seek;
pub mod sweep;
pub mod verify;
pub mod winding;

use crate::burst::Capture;
use crate::frame::{SeqUnwrap, TelFrame, TelemetrySnapshot};
use crate::pot::Pot;
use crate::regs::{Reg, control};

/// One driver action. `Write` is a single-field wire write (the value is
/// truncated to the reg width by the driver); `Read` is one gread of the
/// telemetry region whose parsed snapshot feeds the NEXT `step`; `Stream`
/// arms a TEL burst of `samples` fast ticks - with `goal` Some the driver
/// stages that write and the arm under HOLD and fires one broadcast COMMIT,
/// so the write applies in the same instant the capture starts. Decoded
/// frames return through [`Experiment::push_tel`] before the next `step`.
/// The mask is sticky: write TEL_MASK with an ordinary `Write` first.
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
    /// Decoded TEL frames from the last [`Cmd::Stream`] burst; experiments
    /// that never stream keep the drop default.
    fn push_tel(&mut self, _frames: &[TelFrame]) {}
    /// The capture from the last [`Cmd::Burst`].
    fn push_burst(&mut self, _cap: &Capture) {}
    /// Set once the experiment stopped on something that ends the whole
    /// run, not just itself: [`Guarded`] preempts it on the next step.
    fn halted(&self) -> Option<AbortReason> {
        None
    }
}

/// Rig constants - the single home. The travel guard and the current abort
/// come from the servo's own limits ([`crate::limits::ServoLimits`]); the
/// rest are bench defaults. Experiments take what they need; the CLI
/// overrides via flags.
#[derive(Copy, Clone, Debug)]
pub struct RigParams {
    /// Soft travel guard; `None` disables (end-stop experiments stall at
    /// the physical ends on purpose - see [`RigParams::without_pos_guard`]).
    pub pos_guard: Option<(u16, u16)>,
    /// Abort threshold on the ident-window current mean, counts. Checked
    /// only while duty_mean is nonzero: at torque-off the ident block
    /// holds its last driven value and would trip forever.
    pub i_abort: i16,
    /// Stripped-gear slip zone, masked from motion fits and avoided as a
    /// dwell region (consumed by the ladder/inertia experiments).
    pub slip: (u16, u16),
    /// Ident windows discarded after every duty change (L transient +
    /// window-boundary smear).
    pub settle_windows: u32,
    /// One ident aggregate window in ms: 16 fast ticks at ~20 kHz.
    pub agg_period_ms: f64,
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
}

impl RigParams {
    pub fn new(pos_guard: Option<(u16, u16)>, i_abort: i16) -> Self {
        Self {
            pos_guard,
            i_abort,
            slip: (1250, 1650),
            settle_windows: 5,
            agg_period_ms: 0.8,
            stall_eps: seek::STALL_EPS,
            stall_polls: seek::STALL_POLLS,
            pot: Pot::RAW,
            stops: None,
        }
    }

    pub fn with_stops(self, stops: (u16, u16)) -> Self {
        Self {
            stops: Some(stops),
            ..self
        }
    }

    pub fn without_pos_guard(self) -> Self {
        Self {
            pos_guard: None,
            ..self
        }
    }

    pub fn in_slip(&self, pos: u16) -> bool {
        (self.slip.0..=self.slip.1).contains(&pos)
    }
}

/// Why the envelope stopped an experiment.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AbortReason {
    /// The servo latched a fault (flags + code as read).
    Fault { flags: u8, code: u8 },
    /// Position left the soft guard band.
    PosGuard { pos: u16 },
    /// Ident-window current mean exceeded the abort threshold mid-drive.
    Overcurrent { i_mean: i16 },
    /// A seek came to rest short of anywhere it could stop: jammed, or the
    /// pot is not reading. `moved` is the travel since the seek started.
    Blocked { pos: u16, moved: u16 },
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
        }
    }
}

impl std::error::Error for AbortReason {}

enum GuardState {
    Run,
    DutyOff,
    TorqueOff,
    PermitOff,
    Finished,
}

/// Safety envelope: checks every observation, and on a violation preempts
/// the inner experiment with duty-0 + torque-off before reporting Done.
/// The inner experiment is left unstepped from that point on, so a stall
/// permit it granted is withdrawn here: the envelope watches the writes.
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
        if o.duty_mean_q15 != 0
            && o.i_mean_counts.unsigned_abs() > self.params.i_abort.unsigned_abs()
        {
            return Some(AbortReason::Overcurrent {
                i_mean: o.i_mean_counts,
            });
        }
        None
    }
}

impl<E: Experiment> Experiment for Guarded<E> {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        if matches!(self.state, GuardState::Run)
            && let Some(reason) = self
                .exp
                .halted()
                .or_else(|| obs.and_then(|o| self.violation(o)))
        {
            self.abort = Some(reason);
            self.state = GuardState::DutyOff;
        }
        match self.state {
            GuardState::Run => {
                let cmd = self.exp.step(obs);
                if let Cmd::Write { reg, value } = cmd
                    && reg == control::STALL_PERMIT
                {
                    self.permit = value != 0;
                }
                cmd
            }
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

    fn push_tel(&mut self, frames: &[TelFrame]) {
        self.exp.push_tel(frames);
    }

    fn push_burst(&mut self, cap: &Capture) {
        self.exp.push_burst(cap);
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

    fn push_tel(&mut self, frames: &[TelFrame]) {
        self.exp.push_tel(frames);
    }

    fn push_burst(&mut self, cap: &Capture) {
        self.exp.push_burst(cap);
    }

    fn halted(&self) -> Option<AbortReason> {
        self.exp.halted()
    }
}

/// One accepted ident aggregate window, timebased on the unwrapped
/// `agg_seq` (x 0.8 ms) - poll jitter does not touch the fit clock.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindowSample {
    pub t_ms: f64,
    /// Signed bias-subtracted window current mean, counts.
    pub i: f64,
    /// Drive-window va - vb mean, vcounts.
    pub vdiff: f64,
    /// Commanded duty mean over the window, q15.
    pub duty_q15: f64,
}

/// Turns polled snapshots into deduplicated window samples: a sample is
/// accepted only when `agg_seq` advanced (torn or repeated reads yield
/// None), and the first `settle` windows after every duty transition are
/// discarded. The driver's re-read/agg_seq-match dance guards torn
/// aggregates; this stream guards duplicates and settling.
pub struct WindowStream {
    unwrap: SeqUnwrap,
    last: Option<u64>,
    settle: u32,
    settle_windows: u32,
    agg_period_ms: f64,
}

impl WindowStream {
    pub fn new(params: &RigParams) -> Self {
        Self {
            unwrap: SeqUnwrap::default(),
            last: None,
            settle: 0,
            settle_windows: params.settle_windows,
            agg_period_ms: params.agg_period_ms,
        }
    }

    /// Call on every duty change; the next `settle_windows` accepted
    /// windows are dropped.
    pub fn mark_transition(&mut self) {
        self.settle = self.settle_windows;
    }

    pub fn push(&mut self, o: &TelemetrySnapshot) -> Option<WindowSample> {
        let seq = self.unwrap.push(o.agg_seq);
        if self.last == Some(seq) {
            return None;
        }
        self.last = Some(seq);
        if self.settle > 0 {
            self.settle -= 1;
            return None;
        }
        Some(WindowSample {
            t_ms: seq as f64 * self.agg_period_ms,
            i: o.i_mean_counts as f64,
            vdiff: o.vdiff_mean as f64,
            duty_q15: o.duty_mean_q15 as f64,
        })
    }
}

#[cfg(test)]
pub(crate) mod testkit;

#[cfg(test)]
mod tests {
    use super::testkit::{FakeServo, pump};
    use super::*;

    #[test]
    fn torn_agg_seq_no_duplicate_sample() {
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
        assert_eq!(*log.last().unwrap(), "write torque_enable 0");
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
        assert_eq!(o.duty_applied_q15, -3000, "re-granted after the re-enable");
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
            o.duty_applied_q15, -3000,
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
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 0,
            vbus: 1731,
            i_floor_ticks: 160,
            amps_per_count: 0.0,
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
        assert!(o.duty_applied_q15 < 20971, "and is governed");
        assert!(matches!(
            hold(lim.i_lim as i16).0,
            Some(AbortReason::Overcurrent { .. })
        ));
    }
}
