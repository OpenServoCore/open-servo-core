//! The per-tick half of the kernel: measure the window the previous
//! command drove, then drive what the medium half last commanded. Owns
//! every piece of state that moves at the PWM rate; reads nothing from the
//! control table.

use super::KernelTiming;
use super::config::FastConfig;
use super::current::CurrentLoop;
use super::duty_limit::DutyLimiter;
use super::faults::{self, FaultLatch, OcDetector};
use super::ident::IdentAgg;
use super::limits::IBand;
use crate::estimator::bias::Zero;
use crate::estimator::{BemfObs, BiasTracker, VcalLpf, window};
use crate::pos_lut;
use crate::tel::{TelSample, TelStream};
use crate::traits::{DecayMode, MotorCmd};
use crate::{RegionStorageRaw, SensorFrame, Shared};
use osc_units::Effort;

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
    /// riding the profile speed (0 in Current mode).
    Closed { omega_ff_q16: i32 },
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
pub(super) const ZERO_SETTLE_TICKS: u8 = 32;

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
    /// The position table applies (`pos_lut_state` LIVE), for the stream.
    pub lut_live: bool,
    /// CONTROL `ident_agg`: the identification aggregate folds.
    pub ident_agg: bool,
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
    v_trough_min_ticks: u16,
    bemf_min_ticks: u16,
    i_settle_gain: window::SettleGain,
    vcal_lpf: VcalLpf,
    /// Shunt zero-current offset in use; the medium step publishes it to
    /// `current_bias_counts`.
    bias: BiasTracker,
    oc: OcDetector,
    bemf: BemfObs,
    cur: CurrentLoop,
    /// OpenLoop's duty ceiling against the band.
    ol: DutyLimiter,
    /// Identification aggregator, own /16 fast-tick window.
    ident: IdentAgg,
    /// The duty actually commanded this tick, post-clamp post-gate: 0 while
    /// Brake/Disabled. Feeds next tick's window select AND the
    /// `duty_applied_q15` publish - `CurrentLoop::last_duty` is not used for
    /// telemetry because the gate can override the loop's output.
    pub(super) duty_q15: i16,
    decay: DecayMode,
    /// The last command's bridge state, and the ticks it has left before
    /// its shunt sample counts as zero current (`ZERO_SETTLE_TICKS` from
    /// a change, 0 once settled).
    bridge: Bridge,
    settle: u8,
    /// Last window-valid measurement, 0 from the first tick nothing drives:
    /// the current the observer, the stream, the ident aggregate and the
    /// `i_hat_counts` publish read. A window under the floor holds it, as
    /// the drive still pushes there; with no drive the winding current is
    /// gone within a period, and every window under a zero duty is
    /// invalid, so the 0 holds until the drive resumes.
    pub(super) i_meas_last: i16,
    /// Last v-valid drive-window differential (va - vb), for the ident
    /// accumulation - same hold-last-valid pattern as `i_meas_last`.
    pub(super) vdiff_last: i16,
}

impl Fast {
    pub fn new(timing: &KernelTiming) -> Self {
        Self {
            pwm_arr: timing.pwm_arr,
            bias_brake_min_ticks: timing.bias_brake_min_ticks,
            v_trough_min_ticks: timing.v_trough_min_ticks,
            bemf_min_ticks: timing.bemf_min_ticks,
            i_settle_gain: timing.i_settle_gain,
            vcal_lpf: VcalLpf::new(),
            bias: BiasTracker::new(),
            oc: OcDetector::new(),
            bemf: BemfObs::new(timing.pulse_short_ticks),
            cur: CurrentLoop::new(),
            ol: DutyLimiter::new(),
            ident: IdentAgg::new(),
            duty_q15: 0,
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

    pub fn restart_ident(&mut self) {
        self.ident.restart();
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

    pub fn vcal_lpf_counts(&self) -> u16 {
        self.vcal_lpf.counts()
    }

    /// One tick's measurement against the window the PREVIOUS tick's
    /// command drove: this frame's scan sampled the period that command
    /// drove, so terminal and sign attribution stay correct across sign
    /// flips (bang-bang bench test pins this).
    pub fn measure<T: TelStream>(
        &mut self,
        frame: &SensorFrame,
        fc: &FastConfig,
        cmd: &Command,
        faults: &mut FaultLatch,
        tel: &mut T,
        shared: &Shared,
    ) -> Measured {
        self.vcal_lpf.update(frame.vcal);
        let ticks = window::drive_ticks(self.duty_q15, self.pwm_arr);
        let fwd = self.duty_q15 >= 0;
        let sel = window::select(
            self.decay,
            ticks,
            fc.i_window_min_ticks,
            window::v_floor(self.decay, fc.v_window_min_ticks, self.v_trough_min_ticks),
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
        self.bias.update(zero, frame.current_trough);
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
            self.i_meas_last = i.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        }
        let oc_over = i_meas.map(|i| i.unsigned_abs() > fc.oc_trip_counts as u32);
        if self.oc.sample(oc_over, fc.oc_trip_ticks) {
            faults.raise(faults::BIT_OVER_CURRENT, faults::CODE_OVER_CURRENT);
        }

        // IDENT: per-tick sample aligned to the window the PREVIOUS command
        // drove - duty_q15 still holds that command here; i/vdiff hold
        // last-valid through invalid windows (ident module doc).
        // the taps read physical va - vb; a reversed motor makes that the
        // negative of the logical drive direction every consumer expects
        let vdiff =
            window::vdiff_from_frame(frame, sel).map(|v| if fc.drive_polarity { v } else { -v });
        if let Some(vdiff) = vdiff {
            self.vdiff_last = vdiff.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        }
        self.bemf.sample(
            ticks,
            vdiff.filter(|_| ticks >= self.bemf_min_ticks as u32),
            i_meas,
        );
        // TEL emits HERE, before the medium step: duty_q15 still holds the
        // command whose window this frame's samples measured (the same
        // previous-tick alignment the ident aggregate uses), and the sample
        // lands at a near-constant tick offset.
        if tel.active() {
            let pos_lin_q4 = if cmd.lut_live {
                shared.pos_lut_q4(frame.pos)
            } else {
                pos_lut::identity_q4(frame.pos)
            };
            let s = TelSample {
                pos: frame.pos,
                current: self.i_meas_last,
                current_trough: frame.current_trough,
                duty_q15: self.duty_q15,
                vdiff: self.vdiff_last,
                vbus: cmd.vbus_counts,
                current_raw: frame.current,
                vmotor_a: frame.vmotor_a,
                vmotor_b: frame.vmotor_b,
                vbus_raw: frame.vbus_raw,
                ntc_raw: frame.ntc_raw,
                pos_lin_q4,
                window_valid: i_meas.is_some(),
                fault: faults.mask() != 0,
            };
            tel.on_tick(&s);
        }

        if cmd.ident_agg
            && let Some(agg) = self
                .ident
                .sample(self.i_meas_last, self.vdiff_last, self.duty_q15)
        {
            let p = shared.table.region_ptr();
            // SAFETY: sole-telemetry-writer contract (`Kernel` doc);
            // volatile per-field stores at the ident window boundary.
            unsafe {
                let d = &raw mut (*p).telemetry.ident;
                (&raw mut (*d).i_mean_counts).write_volatile(agg.i_mean_counts);
                (&raw mut (*d).i_min_counts).write_volatile(agg.i_min_counts);
                (&raw mut (*d).i_max_counts).write_volatile(agg.i_max_counts);
                (&raw mut (*d).vdiff_mean).write_volatile(agg.vdiff_mean);
                (&raw mut (*d).duty_mean_q15).write_volatile(agg.duty_mean_q15);
                (&raw mut (*d).agg_seq).write_volatile(agg.agg_seq);
            }
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
        fc: &FastConfig,
        cmd: &Command,
        faults: &FaultLatch,
    ) -> MotorCmd {
        // a fault raised this tick, fast or medium, disables this tick's
        // drive; the loop-reset bookkeeping follows at the medium step
        let drive = if faults.mask() == 0 {
            cmd.drive
        } else {
            Drive::Off
        };
        let out = match drive {
            Drive::Off => {
                self.duty_q15 = 0;
                MotorCmd::Disabled
            }
            // anti-hunt: park instead of dithering on friction; the frozen
            // loop resumes bumplessly on exit. The winding short brakes a
            // shaft that enters the band at speed (Ke x omega / R, zero at
            // rest, nothing from the rail). On friction alone a fast arrival
            // leaves the far edge, and the wake kicks it back through at the
            // current limit: a limit cycle.
            Drive::Brake => {
                self.duty_q15 = 0;
                MotorCmd::Brake
            }
            // ceilinged against the current band above the window floor, raw
            // at or under it; NO vbus comp - identification wants
            // unconfounded actuation
            Drive::OpenLoop { goal_q15: goal } => {
                // a start from zero duty is a reversal too: it restarts from
                // the floor like the run edge does. So is a zero goal, which
                // drives nothing: the held current restarts from 0 with it
                if goal.signum() != (self.duty_q15 as i32).signum() {
                    self.ol.reset(fc.ol_floor_q15);
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
                    fc.ol_floor_q15,
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
                if blocked || (duty == 0 && fc.ol_zero_brake) {
                    // chip-side Drive{0, Slow} maps to coast; a winding short
                    // must be commanded explicitly
                    self.decay = DecayMode::Slow;
                    MotorCmd::Brake
                } else {
                    self.decay = fc.ol_decay;
                    MotorCmd::Drive {
                        duty: Effort(duty),
                        decay: fc.ol_decay,
                    }
                }
            }
            Drive::Closed { omega_ff_q16 } => {
                if cmd.i_ref_cc == 0 && m.i_meas.is_none() && (cmd.band.hi == 0 || cmd.band.lo == 0)
                {
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
                    // undervolt-floored vbus: the same floor `vbus.step`
                    // applied to the reciprocal, so (vbus, recip) stay the
                    // contract pair even before the first seed
                    let vbus_eff = cmd.vbus_counts.max(fc.v_undervolt_counts);
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
                        omega_ff_q16,
                        vbus_eff,
                        cmd.vbus_recip_q15,
                        &fc.current,
                    );
                    self.duty_q15 = duty;
                    // closed-loop decay is fixed Slow (spec: config enum
                    // exists for OpenLoop identification only)
                    self.decay = DecayMode::Slow;
                    MotorCmd::Drive {
                        duty: Effort(duty),
                        decay: DecayMode::Slow,
                    }
                }
            }
        };
        let bridge = match out {
            MotorCmd::Disabled => Bridge::Asleep,
            MotorCmd::Brake => Bridge::Brake,
            MotorCmd::Drive { duty, .. } if duty.0 == 0 => Bridge::Coast,
            _ => Bridge::Drive,
        };
        if bridge == self.bridge {
            self.settle = self.settle.saturating_sub(1);
        } else {
            self.settle = ZERO_SETTLE_TICKS;
            self.bridge = bridge;
        }
        // logical (+duty moves counts up) -> wiring, on the output only:
        // duty_q15 and the published duty stay logical. Off and the brake
        // drive nothing: the held current drops (`i_meas_last`)
        match out {
            MotorCmd::Drive { duty, decay } if !fc.drive_polarity => MotorCmd::Drive {
                duty: Effort(duty.0.saturating_neg()),
                decay,
            },
            MotorCmd::Drive { .. } => out,
            _ => {
                self.i_meas_last = 0;
                out
            }
        }
    }
}
