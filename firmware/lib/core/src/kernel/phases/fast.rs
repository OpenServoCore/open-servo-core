//! The per-tick half of the kernel: measure the window the previous
//! command drove, then drive what the medium half last commanded. Owns
//! every piece of state that moves at the PWM rate; reads nothing from the
//! control table.

use super::loops::current::{CurrentLoop, RailClamp};
use super::loops::duty_limit::DutyLimiter;
use super::loops::faults::{self, FaultLatch, OcDetector};
use super::loops::limits::IBand;
use crate::estimator::bias::Zero;
use crate::estimator::{BemfObs, BiasTracker, window};
use crate::kernel::KernelTiming;
use crate::kernel::config::KernelConfig;
use crate::tel::tick_flag;
use crate::traits::{DecayMode, MotorCmd};
use crate::{RegionStorageRaw, Shared};

/// What the drive does until the medium half commands otherwise.
#[derive(Copy, Clone, Default)]
pub enum Drive {
    #[default]
    Off,
    /// The position hold's park: the winding short.
    Brake,
    /// OpenLoop at `goal_q15`, already clamped to `duty_max`.
    OpenLoop { goal_q15: i32 },
    /// The current loop on `Command::i_ref_cc`, with the Ke feed-forward
    /// (`current::u_ff_vc`) riding the profile speed (0 in Current mode).
    Closed { u_ff_vc: i32 },
}

/// What the bridge carried through a period, for the zero-current feed.
#[derive(Copy, Clone, PartialEq, Eq)]
enum Bridge {
    /// `MotorCmd::Disabled`.
    Asleep,
    Brake,
    /// A zero duty, which the chip writes as a coast.
    Coast,
    Drive,
}

/// Ticks a bridge state without drive holds before its shunt sample counts
/// as zero current: a drive's winding current drains through the body
/// diodes into the rail in L x I / V_rail, about 0.25 ms on the MG90 at the
/// trip current.
pub(in crate::kernel) const ZERO_SETTLE_TICKS: u8 = 32;

/// Medium -> fast: everything the drive and the stream need between two
/// medium ticks.
#[derive(Copy, Clone, Default)]
pub struct Command {
    pub drive: Drive,
    pub i_ref_cc: i32,
    pub band: IBand,
    /// The duty OpenLoop applies while its ceiling sits under the window
    /// floor.
    pub ol_base_q15: u16,
    pub vbus_counts: u16,
    pub vbus_recip_q15: u32,
    /// The current loop's clamp on `vbus_counts`, prepared by `prepare_rail`.
    pub rail: RailClamp,
}

impl Command {
    /// The clamp on the undervolt-floored vbus: the same floor `VbusEst`
    /// applies to the reciprocal, so (clamp, recip) stay the contract pair
    /// even before the first seed. Runs at the first tick and at every RAIL
    /// phase, which is where a `duty_max_q15` write reaches it.
    pub fn prepare_rail(&mut self, cfg: &KernelConfig) {
        let vbus_eff = self.vbus_counts.max(cfg.thermal.v_undervolt_counts);
        self.rail = RailClamp::new(cfg.loop_current.duty_max_q15, vbus_eff);
    }
}

/// Fast -> medium, this tick's measurement.
#[derive(Copy, Clone)]
pub struct Measured {
    pub i_meas: Option<i32>,
    pub vdiff: Option<i32>,
    /// Drive width of the window measured.
    pub ticks: u32,
}

pub struct Fast {
    pwm_arr: u16,
    bias_brake_min_ticks: u16,
    bemf_min_ticks: u16,
    i_settle_gain: window::SettleGain,
    /// Shunt zero-current offset in use; the medium step publishes it to
    /// `current_bias_counts`.
    bias: BiasTracker,
    oc: OcDetector,
    bemf: BemfObs,
    cur: CurrentLoop,
    /// OpenLoop's duty ceiling against the band.
    ol: DutyLimiter,
    /// The duty actually commanded this tick, post-clamp post-gate: 0 while
    /// Brake/Disabled. Feeds next tick's window select: the gate can
    /// override the loop's output.
    pub(in crate::kernel) duty_q15: i16,
    /// The drive width the motor write programmed for `duty_q15`
    /// (`window::drive_ticks`), 0 while Brake/Disabled: next tick's window.
    pub(in crate::kernel) ticks: u16,
    decay: DecayMode,
    /// The last command's bridge state, and the ticks it has left before
    /// its shunt sample counts as zero current (`ZERO_SETTLE_TICKS` from
    /// a change, 0 once settled).
    bridge: Bridge,
    settle: u8,
    /// Last window-valid measurement, 0 from the first tick nothing drives:
    /// the current the observer and the tick block's `i_tick_counts` read.
    /// A window under the floor holds it, as the drive still pushes there;
    /// with no drive the winding current is gone within a period, and every
    /// window under a zero duty is invalid, so the 0 holds until the drive
    /// resumes.
    pub(in crate::kernel) i_meas_last: i16,
    /// Last v-valid drive-window differential (va - vb), the tick block's
    /// `vdiff_counts` - same hold-last-valid pattern as `i_meas_last`.
    pub(in crate::kernel) vdiff_last: i16,
}

impl Fast {
    pub fn new(timing: &KernelTiming) -> Self {
        Self {
            pwm_arr: timing.pwm_arr,
            bias_brake_min_ticks: timing.bias_brake_min_ticks,
            bemf_min_ticks: timing.bemf_min_ticks,
            i_settle_gain: timing.i_settle_gain,
            bias: BiasTracker::new(),
            oc: OcDetector::new(),
            bemf: BemfObs::new(),
            cur: CurrentLoop::new(),
            ol: DutyLimiter::new(),
            duty_q15: 0,
            ticks: 0,
            decay: DecayMode::Slow,
            bridge: Bridge::Asleep,
            settle: ZERO_SETTLE_TICKS,
            i_meas_last: 0,
            vdiff_last: 0,
        }
    }

    pub fn seed_bias(&mut self, counts: u16) {
        self.bias.seed(counts);
    }

    /// Fault ack: the overcurrent window re-arms.
    pub fn ack(&mut self) {
        self.oc.reset();
    }

    pub fn reset_current_loop(&mut self) {
        self.cur.reset();
    }

    pub fn reset_limiter(&mut self, floor_q15: u16) {
        self.ol.reset(floor_q15);
    }

    pub fn take_pinned(&mut self) -> bool {
        self.ol.take_pinned()
    }

    /// Medium-boundary close of the back-EMF boxcar half (`BemfObs`).
    pub fn close_bemf_half(
        &mut self,
        r_q12: u16,
        recip_ke_q: u16,
        recip_arr_q24: u32,
    ) -> Option<i32> {
        self.bemf.close_half(r_q12, recip_ke_q, recip_arr_q24)
    }

    pub fn duty_q15(&self) -> i16 {
        self.duty_q15
    }

    pub fn i_meas_last(&self) -> i16 {
        self.i_meas_last
    }

    pub fn bias_counts(&self) -> u16 {
        self.bias.counts()
    }

    /// One tick's measurement against the window the PREVIOUS tick's
    /// command drove: this frame's scan sampled the period that command
    /// drove, so terminal and sign attribution stay correct across sign
    /// flips (bang-bang bench test pins this).
    pub fn measure(
        &mut self,
        cfg: &KernelConfig,
        faults: &mut FaultLatch,
        shared: &Shared,
    ) -> Measured {
        let frame = shared.frame();
        let ticks = self.ticks as u32;
        let fwd = self.duty_q15 >= 0;
        let sel = window::select(
            self.decay,
            ticks,
            cfg.i_window_min_ticks,
            cfg.v_floor_ticks[self.decay as usize],
        );
        let zero = match self.bridge {
            Bridge::Drive => {
                window::trough_is_brake(self.decay, ticks, self.pwm_arr, self.bias_brake_min_ticks)
                    .then_some(Zero::Awake)
            }
            _ if self.settle != 0 => None,
            Bridge::Asleep => Some(Zero::Asleep),
            Bridge::Brake | Bridge::Coast => Some(Zero::Awake),
        };
        self.bias.update(zero, frame.current_trough());
        // gated here, not only inside: a window under the floor skips the
        // zero's rounding and the settle-gain band lookup on the tick path
        let i_meas = if sel.i_valid {
            window::i_from_frame(
                frame,
                sel,
                fwd,
                self.bias.counts(),
                self.i_settle_gain.q15_at(ticks),
            )
        } else {
            None
        };
        if let Some(i) = i_meas {
            // 12-bit shunt codes under a settle gain < 2.0: |i| <= 8190
            self.i_meas_last = i as i16;
        }
        let oc_over = i_meas.map(|i| i.unsigned_abs() > cfg.limits.oc_trip_counts as u32);
        if self.oc.sample(oc_over, cfg.limits.oc_trip_ticks) {
            faults.raise(faults::BIT_OVER_CURRENT, faults::CODE_OVER_CURRENT);
        }

        // the taps read physical va - vb; a reversed motor makes that the
        // negative of the logical drive direction every consumer expects.
        // m is 0 or -1: (v ^ m) - m negates without a branch
        let m = cfg.limits.drive_polarity as i32 - 1;
        let vdiff = window::vdiff_from_frame(frame, sel).map(|v| (v ^ m) - m);
        if let Some(vdiff) = vdiff {
            // 12-bit taps: |vdiff| <= 4095
            self.vdiff_last = vdiff as i16;
        }
        self.bemf.sample(
            ticks,
            vdiff.filter(|_| ticks >= self.bemf_min_ticks as u32),
            i_meas,
        );
        // The tick block, HERE before the medium step: `ticks` still holds
        // the command whose window this frame's samples measured. The
        // kernel takes the row from it next.
        let rev = (self.duty_q15 >> 15) as i32;
        let duty = ((ticks as i32 ^ rev) - rev) as i16;
        let flags = (tick_flag::I_VALID * i_meas.is_some() as u16)
            | (tick_flag::FAULT * (faults.mask() != 0) as u16)
            | (tick_flag::V_VALID * vdiff.is_some() as u16);
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // per-field stores, ordered ahead of the row's copy.
        unsafe {
            let t = &raw mut (*p).telemetry.tick;
            (&raw mut (*t).i_tick_counts).write_volatile(self.i_meas_last);
            (&raw mut (*t).vdiff_counts).write_volatile(self.vdiff_last);
            (&raw mut (*t).duty_ticks).write_volatile(duty);
            (&raw mut (*t).tick_flags).write_volatile(flags);
        }

        Measured {
            i_meas,
            vdiff,
            ticks,
        }
    }

    /// The motor command for this tick: the fault latch gates, then the
    /// medium half's command runs.
    pub fn drive(
        &mut self,
        m: &Measured,
        cfg: &KernelConfig,
        cmd: &Command,
        faults: &FaultLatch,
    ) -> MotorCmd {
        // a fault raised this tick, fast or medium, disables this tick's
        // drive; the loop-reset bookkeeping follows at the medium step
        let drive = if faults.mask() == 0 {
            cmd.drive
        } else {
            core::hint::cold_path();
            Drive::Off
        };
        // the current loop runs on the fall-through: the other kinds sit
        // off the tick-budget path
        let out = if let Drive::Closed { u_ff_vc } = drive {
            if cmd.i_ref_cc == 0 && m.i_meas.is_none() && (cmd.band.hi == 0 || cmd.band.lo == 0) {
                core::hint::cold_path();
                // endstop-clamped ref with no window: the honest-zero
                // feed below makes e = 0, freezing the PI at whatever
                // sub-floor duty it unwound to - stalled at an endstop
                // that grinds the gears forever (bench: 18% duty held
                // into the rail). Brake shorts the winding, passively
                // holding against whatever momentum remains; it must be
                // commanded explicitly - chip-side Drive{0, Slow} maps to
                // coast. Scoped to a collapsed band so a transient i_ref
                // zero crossing in normal travel can never reset the loop
                // mid-reversal.
                self.cur.reset();
                self.duty_q15 = 0;
                self.decay = DecayMode::Slow;
                MotorCmd::Brake
            } else {
                // An invalid window only happens below the sampling
                // floor, where the duty is too small to push real
                // current: 0 is the honest estimate THERE, and it lets
                // the PI lift off zero duty - a strict freeze would
                // deadlock (no duty -> no window -> e = 0 -> no duty). OC
                // and the estimators keep the strict validity view.
                let i_loop = Some(m.i_meas.unwrap_or(0));
                let duty = self.cur.step(
                    cmd.i_ref_cc,
                    i_loop,
                    u_ff_vc,
                    &cmd.rail,
                    cmd.vbus_recip_q15,
                    &cfg.loop_current,
                );
                self.duty_q15 = duty;
                // closed-loop decay is fixed Slow (spec: config enum
                // exists for OpenLoop identification only)
                self.drive_cmd(duty, DecayMode::Slow, cfg.limits.drive_polarity)
            }
        } else if let Drive::Brake = drive {
            // anti-hunt: park instead of dithering on friction; the frozen
            // loop resumes bumplessly on exit. The winding short brakes a
            // shaft that enters the band at speed (Ke x omega / R, zero at
            // rest, nothing from the rail). On friction alone a fast arrival
            // leaves the far edge, and the wake kicks it back through at the
            // current limit: a limit cycle. Not with the cold kinds below: a
            // hold at rest parks here every tick.
            self.duty_q15 = 0;
            MotorCmd::Brake
        } else {
            core::hint::cold_path();
            match drive {
                // ceilinged against the current band above the window floor, raw
                // at or under it; NO vbus comp - identification wants
                // unconfounded actuation
                Drive::OpenLoop { goal_q15: goal } => {
                    // a start from zero duty is a reversal too: it restarts from
                    // the floor like the run edge does. So is a zero goal, which
                    // drives nothing: the held current restarts from 0 with it
                    if goal.signum() != (self.duty_q15 as i32).signum() {
                        self.ol.reset(cfg.ol_floor_q15);
                        self.i_meas_last = 0;
                    }
                    let lim = if goal >= 0 { cmd.band.hi } else { cmd.band.lo };
                    let i_abs = match m.i_meas {
                        Some(i) => i.unsigned_abs(),
                        None => 0,
                    };
                    let mag = self.ol.step(
                        goal.unsigned_abs() as u16,
                        i_abs,
                        lim.unsigned_abs(),
                        cfg.ol_floor_q15,
                        cmd.ol_base_q15,
                    ) as i16;
                    let mut duty = if goal < 0 { -mag } else { mag };
                    // endstop: a collapsed band side forbids that sign of
                    // current, and duty of the same sign is what drives it - zero
                    // the outbound push, retreat passes (bench: an open-loop
                    // sweep crashed the horn into the rail)
                    let blocked = (cmd.band.hi == 0 && duty > 0) || (cmd.band.lo == 0 && duty < 0);
                    if blocked {
                        duty = 0;
                    }
                    self.duty_q15 = duty;
                    // a zeroed push still coasts on its momentum into the
                    // physical stop, so the wall brakes whatever the flag says
                    if blocked || (duty == 0 && cfg.limits.openloop_zero_brake) {
                        // chip-side Drive{0, Slow} maps to coast; a winding short
                        // must be commanded explicitly
                        self.decay = DecayMode::Slow;
                        MotorCmd::Brake
                    } else {
                        self.drive_cmd(duty, cfg.limits.openloop_decay, cfg.limits.drive_polarity)
                    }
                }
                // Off; Closed took the branch above
                _ => {
                    self.duty_q15 = 0;
                    MotorCmd::Disabled
                }
            }
        };
        let bridge = match out {
            MotorCmd::Disabled => Bridge::Asleep,
            MotorCmd::Brake => Bridge::Brake,
            MotorCmd::Drive { .. } if self.duty_q15 == 0 => Bridge::Coast,
            _ => Bridge::Drive,
        };
        if bridge == self.bridge {
            self.settle = self.settle.saturating_sub(1);
        } else {
            self.settle = ZERO_SETTLE_TICKS;
            self.bridge = bridge;
        }
        // Off and the brake drive nothing: the held current drops
        // (`i_meas_last`)
        match out {
            MotorCmd::Drive { ticks, .. } => self.ticks = ticks,
            _ => {
                self.ticks = 0;
                self.i_meas_last = 0;
            }
        }
        out
    }

    /// The drive of `duty` (logical: +duty moves counts up) in the wiring's
    /// frame; `duty_q15` and the published duty stay logical. A zero width
    /// coasts whatever its `reverse`.
    fn drive_cmd(&mut self, duty: i16, decay: DecayMode, drive_polarity: bool) -> MotorCmd {
        self.decay = decay;
        MotorCmd::Drive {
            ticks: window::drive_ticks(duty, self.pwm_arr) as u16,
            reverse: (duty < 0) == drive_polarity,
            decay,
        }
    }
}
