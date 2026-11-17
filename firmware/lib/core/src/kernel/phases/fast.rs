//! The per-tick half of the kernel: measure the window the previous
//! command drove, then drive what the medium half last commanded. Owns
//! every piece of state that moves at the PWM rate; reads nothing from the
//! control table. The experiment image (kernel sec 2.7) keeps the window
//! measurement and the overcurrent trip and drives the OpenLoop goal as
//! written: no current loop, no duty ceiling, no back-EMF sums.

#[cfg(not(feature = "experiment"))]
use super::loops::current::{CurrentLoop, RailClamp};
#[cfg(not(feature = "experiment"))]
use super::loops::duty_limit::DutyLimiter;
use super::loops::faults::{self, FaultLatch, OcDetector};
#[cfg(not(feature = "experiment"))]
use super::loops::limits::IBand;
#[cfg(not(feature = "experiment"))]
use crate::estimator::BemfObs;
use crate::estimator::bias::Zero;
use crate::estimator::{BiasTracker, window};
use crate::kernel::KernelTiming;
use crate::kernel::config::KernelConfig;
#[cfg(not(feature = "experiment"))]
use crate::math::sat_clamp;
use crate::regions::telemetry::tick_flag;
use crate::traits::MotorCmd;
use crate::{RegionStorageRaw, Shared};

/// What the drive does until the medium half commands otherwise.
#[derive(Copy, Clone, Default)]
pub enum Drive {
    #[default]
    Off,
    /// The position hold's park: the winding short.
    #[cfg(not(feature = "experiment"))]
    Brake,
    /// OpenLoop at `goal_q15`, already clamped to `duty_max`.
    OpenLoop { goal_q15: i32 },
    /// The current loop on `Command::i_ref_cc`, with the Ke feed-forward
    /// (`current::u_ff_vc`) riding the profile speed (0 in Current mode).
    #[cfg(not(feature = "experiment"))]
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
    #[cfg(not(feature = "experiment"))]
    pub i_ref_cc: i32,
    #[cfg(not(feature = "experiment"))]
    pub band: IBand,
    /// The duty OpenLoop applies while its ceiling sits under the window
    /// floor.
    #[cfg(not(feature = "experiment"))]
    pub ol_base_q15: u16,
    #[cfg(not(feature = "experiment"))]
    pub vbus_counts: u16,
    #[cfg(not(feature = "experiment"))]
    pub vbus_recip_q15: u32,
    /// The current loop's clamp on `vbus_counts`, prepared by `prepare_rail`.
    #[cfg(not(feature = "experiment"))]
    pub rail: RailClamp,
    /// The winding resistance the current loop commands through on a tick
    /// with no reading (`CurrentLoop::step`), Q.12 vcounts per ccount: the
    /// one at the estimate's temperature, or the take's stand-in for an
    /// unidentified winding (`Medium::r_hat`).
    #[cfg(not(feature = "experiment"))]
    pub r_loop_q12: u32,
}

#[cfg(not(feature = "experiment"))]
impl Command {
    /// The clamp on the undervolt-floored vbus: the same floor `VbusEst`
    /// applies to the reciprocal, so (clamp, recip) stay the contract pair
    /// even before the first seed. Runs at the first tick and at every RAIL
    /// phase, which is where a `duty_max_q15` write reaches it. Returns the
    /// floored vbus, the window term's rail (`Fast::prepare_window`).
    pub fn prepare_rail(&mut self, cfg: &KernelConfig) -> u16 {
        let vbus_eff = self.vbus_counts.max(cfg.thermal.v_undervolt_counts);
        self.rail = RailClamp::new(cfg.loop_current.duty_max_q15, vbus_eff);
        vbus_eff
    }
}

/// The current side of this tick's measurement, for the drive and the
/// voltage side after the write.
#[derive(Copy, Clone)]
pub struct Measured {
    pub i_meas: Option<i32>,
    pub(in crate::kernel) sel: window::WindowSel,
}

/// The measured window's command and held current as the write found them:
/// the voltage side and the medium step run after the write and read the
/// window this frame sampled, not the command just written.
#[derive(Copy, Clone)]
struct Seen {
    duty_q15: i16,
    ticks: u16,
    bridge: Bridge,
    settle: u8,
    i_meas_last: i16,
}

pub struct Fast {
    pwm_arr: u16,
    bias_brake_min_ticks: u16,
    #[cfg(not(feature = "experiment"))]
    bemf_min_ticks: u16,
    i_settle_gain: window::SettleGain,
    /// Shunt zero-current offset in use, read every tick; the medium half
    /// feeds it once a period (`track_bias`) and publishes it to
    /// `current_bias_counts`.
    bias: BiasTracker,
    oc: OcDetector,
    #[cfg(not(feature = "experiment"))]
    pub(in crate::kernel) bemf: BemfObs,
    #[cfg(not(feature = "experiment"))]
    cur: CurrentLoop,
    /// OpenLoop's duty ceiling against the band.
    #[cfg(not(feature = "experiment"))]
    ol: DutyLimiter,
    /// The duty actually commanded this tick, post-clamp post-gate: 0 while
    /// Brake/Disabled. Feeds next tick's window select: the gate can
    /// override the loop's output.
    pub(in crate::kernel) duty_q15: i16,
    /// The drive width the motor write programmed for `duty_q15`
    /// (`window::drive_ticks`), 0 while Brake/Disabled: next tick's window.
    pub(in crate::kernel) ticks: u16,
    /// The last command's bridge state, and the ticks it has left before
    /// its shunt sample counts as zero current (`ZERO_SETTLE_TICKS` from
    /// a change, 0 once settled).
    bridge: Bridge,
    settle: u8,
    /// The way the last drive pushed while the winding may still carry its
    /// current: 1 forward, 2 reverse, 0 none, from boot and once a state
    /// without drive has settled.
    way: u8,
    /// Last window-valid measurement, 0 from the first tick nothing drives:
    /// the current the observer, the limits and the tick block's
    /// `i_tick_counts` read. A window under the floor holds it under
    /// OpenLoop, as the drive still pushes there, and under the current
    /// loop takes the reference, the current its command is built for
    /// (`drive`); with no drive the winding current is gone within a
    /// period, and every window under a zero duty is invalid, so the 0
    /// holds until the drive resumes.
    pub(in crate::kernel) i_meas_last: i16,
    /// Last v-valid drive-window differential (va - vb), the tick block's
    /// `vdiff_counts` - same hold-last-valid pattern as `i_meas_last`.
    pub(in crate::kernel) vdiff_last: i16,
    /// `KernelTiming::bridge_min_ticks` less one, 0 with the gate off: a
    /// commanded width of 1 up to it comes out as no pulse.
    #[cfg(not(feature = "experiment"))]
    dropped_max_ticks: u16,
    /// The width the next tick's window follows was such a command: the
    /// current loop does not take that window's reading.
    #[cfg(not(feature = "experiment"))]
    dropped: bool,
    seen: Seen,
    /// The last write missed its load point (`Motor::late`): the window it
    /// drives holds two commands, so its current is no reading.
    pub(in crate::kernel) late: bool,
    /// The window term every reading sheds (`window::WindowTerm`), on the
    /// rail the last RAIL phase read; zero on the experiment image, which
    /// keeps the raw reading. Its carry advances on each reading alone and
    /// holds across a take, a rail and a drive change: it is under a count
    /// of the magnitude, taken ahead of the sign.
    #[cfg(not(feature = "experiment"))]
    pub(in crate::kernel) win: window::WindowTerm,
}

impl Fast {
    pub fn new(timing: &KernelTiming) -> Self {
        Self {
            pwm_arr: timing.pwm_arr,
            bias_brake_min_ticks: timing.bias_brake_min_ticks,
            #[cfg(not(feature = "experiment"))]
            bemf_min_ticks: timing.bemf_min_ticks,
            i_settle_gain: timing.i_settle_gain,
            bias: BiasTracker::new(),
            oc: OcDetector::new(),
            #[cfg(not(feature = "experiment"))]
            bemf: BemfObs::new(timing.brush_dev_max),
            #[cfg(not(feature = "experiment"))]
            cur: CurrentLoop::new(),
            #[cfg(not(feature = "experiment"))]
            ol: DutyLimiter::new(),
            duty_q15: 0,
            ticks: 0,
            bridge: Bridge::Asleep,
            settle: ZERO_SETTLE_TICKS,
            way: 0,
            i_meas_last: 0,
            vdiff_last: 0,
            #[cfg(not(feature = "experiment"))]
            dropped_max_ticks: timing.bridge_min_ticks.saturating_sub(1),
            #[cfg(not(feature = "experiment"))]
            dropped: false,
            seen: Seen {
                duty_q15: 0,
                ticks: 0,
                bridge: Bridge::Asleep,
                settle: ZERO_SETTLE_TICKS,
                i_meas_last: 0,
            },
            late: false,
            #[cfg(not(feature = "experiment"))]
            win: window::WindowTerm::default(),
        }
    }

    pub fn seed_bias(&mut self, counts: u16) {
        self.bias.seed(counts);
    }

    /// Fault ack: the overcurrent window re-arms.
    pub fn ack(&mut self) {
        self.oc.reset();
    }

    #[cfg(not(feature = "experiment"))]
    pub fn reset_current_loop(&mut self) {
        self.cur.reset();
    }

    /// The current loop entered from OpenLoop: the integral at the volts
    /// the bridge applied, and the dropped-pulse gate's rule on the last
    /// width OpenLoop wrote, which nothing judged while it drove.
    #[cfg(not(feature = "experiment"))]
    pub fn enter_current_loop(&mut self, cmd: &Command) {
        let lim = cmd.rail.integ_lim_vq16 >> 16;
        let v = crate::math::q_mul(self.duty_q15 as i32, cmd.vbus_counts as i32, 15);
        self.cur.seed(v.max(-lim).min(lim));
        self.dropped = self.ticks.wrapping_sub(1) < self.dropped_max_ticks;
    }

    /// The window term on the take's conductance (`KernelConfig::g_arr`)
    /// and the rail `Command::prepare_rail` floored, beside it.
    #[cfg(not(feature = "experiment"))]
    pub fn prepare_window(&mut self, g_arr: u32, vbus_eff: u16) {
        self.win.set_rail(g_arr, vbus_eff, self.pwm_arr);
    }

    #[cfg(not(feature = "experiment"))]
    pub fn reset_limiter(&mut self, floor_q15: u16) {
        self.ol.reset(floor_q15);
    }

    #[cfg(not(feature = "experiment"))]
    pub fn take_pinned(&mut self) -> bool {
        self.ol.take_pinned()
    }

    /// Medium-boundary close of the back-EMF boxcar half (`BemfObs::close`)
    /// on the duty that drove the half's last sample. `xw_q12` is the brush
    /// term's size (`bemf::brush_xw_q12`).
    #[cfg(not(feature = "experiment"))]
    pub fn close_bemf_half(
        &mut self,
        r_q12: u16,
        xw_q12: i32,
        recip_ke_q: u16,
        recip_arr_q24: u32,
    ) -> Option<i32> {
        self.bemf
            .close(self.seen.duty_q15, r_q12, xw_q12, recip_ke_q, recip_arr_q24)
    }

    pub fn duty_q15(&self) -> i16 {
        self.duty_q15
    }

    /// The held current as this tick's measurement left it.
    #[cfg(not(feature = "experiment"))]
    pub fn i_meas_last(&self) -> i16 {
        self.seen.i_meas_last
    }

    pub fn bias_counts(&self) -> u16 {
        self.bias.counts()
    }

    /// The bias tracker's sample, once a period from the PUBLISH phase:
    /// the zero moves with board temperature, over minutes. On the bridge
    /// state of the command this frame's trough ran under.
    #[inline(always)]
    pub fn track_bias(&mut self, shared: &Shared) {
        let seen = self.seen;
        let zero = match seen.bridge {
            Bridge::Drive => {
                window::trough_is_brake(seen.ticks as u32, self.pwm_arr, self.bias_brake_min_ticks)
                    .then_some(Zero::Awake)
            }
            _ if seen.settle != 0 => None,
            Bridge::Asleep => Some(Zero::Asleep),
            Bridge::Brake | Bridge::Coast => Some(Zero::Awake),
        };
        // the sample loads, and reads a wide code as the middle of its
        // band, only on a period that feeds
        if let Some(zero) = zero {
            self.bias
                .update(zero, shared.adc_mid_q4(shared.frame().current_trough()));
        }
    }

    /// The bias tracker's aging step, once a SLOW period.
    #[inline(always)]
    pub fn age_bias(&mut self) {
        self.bias.age();
    }

    /// The current side of one tick's measurement against the window the
    /// PREVIOUS tick's command drove, the winding's current with the window
    /// term off (kernel sec 2.2), and the overcurrent trip: all the motor
    /// write waits on. This frame's scan sampled the period that
    /// command drove, so terminal and sign attribution stay correct across
    /// sign flips (bang-bang bench test pins this).
    pub fn measure(
        &mut self,
        cfg: &KernelConfig,
        faults: &mut FaultLatch,
        shared: &Shared,
    ) -> Measured {
        let frame = shared.frame();
        let ticks = self.ticks as u32;
        let fwd = self.duty_q15 >= 0;
        // The first window a drive reverses into pushes against the last
        // drive's current, which the winding still carries, and the
        // return-path shunt reads a current against the drive as about
        // zero: no reading for any consumer, as under the floor. A brake
        // carries that current on L/R, a coast or off drains it into the
        // rail, both within the zero's settle; past it the next drive is
        // a first one.
        let way = 2 - fwd as u8;
        let reversed = ticks != 0 && (self.way | way) == 3;
        if ticks != 0 {
            self.way = way;
        } else if self.settle == 0 && self.bridge != Bridge::Drive {
            self.way = 0;
        }
        let mut sel = window::select(
            if reversed { 0 } else { ticks },
            cfg.i_window_min_ticks,
            cfg.v_floor_ticks,
        );
        sel.i_valid &= !self.late;
        // gated here, not only inside: a window under the floor skips the
        // zero's rounding and the settle-gain band lookup on the tick path
        let along = if sel.i_valid {
            window::i_from_frame(
                shared,
                frame,
                sel,
                self.bias.counts(),
                self.i_settle_gain.q15_at(ticks),
            )
        } else {
            None
        };
        // the trip guards the bridge and the shunt, which carry the window
        // term: it reads the terminal current
        let oc_over = along.map(|i| i.unsigned_abs() > cfg.limits.oc_trip_counts as u32);
        let i_meas = along.map(|i| {
            #[cfg(not(feature = "experiment"))]
            let i = i - self.win.take(ticks);
            if fwd { i } else { -i }
        });
        if let Some(i) = i_meas {
            // 12-bit shunt codes under a settle gain < 2.0, less a term
            // of at most `window::WINDOW_V_MAX`: |i| <= 16381
            self.i_meas_last = i as i16;
        }
        if self.oc.sample(oc_over, cfg.limits.oc_trip_ticks) {
            faults.raise(faults::BIT_OVER_CURRENT, faults::CODE_OVER_CURRENT);
        }
        self.seen.duty_q15 = self.duty_q15;
        self.seen.i_meas_last = self.i_meas_last;
        Measured { i_meas, sel }
    }

    /// The voltage side of the measurement, after the write: the terminal
    /// differential, the back-EMF sample and the tick block, on the
    /// measured window.
    pub fn measure_voltage(
        &mut self,
        m: &Measured,
        cfg: &KernelConfig,
        faults: &FaultLatch,
        shared: &Shared,
    ) {
        let frame = shared.frame();
        let seen = self.seen;
        let ticks = seen.ticks as u32;
        let sel = m.sel;
        // the taps read physical va - vb; a reversed motor makes that the
        // negative of the logical drive direction every consumer expects.
        // neg is 0 or -1: (v ^ neg) - neg negates without a branch
        let neg = cfg.limits.drive_polarity as i32 - 1;
        let vdiff = window::vdiff_from_frame(shared, frame, sel).map(|v| (v ^ neg) - neg);
        if let Some(vdiff) = vdiff {
            // 12-bit taps: |vdiff| <= 4095
            self.vdiff_last = vdiff as i16;
        }
        #[cfg(not(feature = "experiment"))]
        self.bemf.sample(
            ticks,
            vdiff.filter(|_| ticks >= self.bemf_min_ticks as u32),
            m.i_meas,
        );
        // The tick block, before the TEL row's copy and the medium step, on
        // the command whose window this frame's samples measured.
        let rev = (seen.duty_q15 >> 15) as i32;
        let duty = ((ticks as i32 ^ rev) - rev) as i16;
        let flags = (tick_flag::I_VALID * m.i_meas.is_some() as u16)
            | (tick_flag::FAULT * (faults.mask() != 0) as u16)
            | (tick_flag::V_VALID * vdiff.is_some() as u16);
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // per-field stores, ordered ahead of the row's copy.
        unsafe {
            let t = &raw mut (*p).telemetry.tick;
            (&raw mut (*t).i_tick_counts).write_volatile(seen.i_meas_last);
            (&raw mut (*t).vdiff_counts).write_volatile(self.vdiff_last);
            (&raw mut (*t).duty_ticks).write_volatile(duty);
            (&raw mut (*t).tick_flags).write_volatile(flags);
        }
    }

    /// The motor command for this tick: the fault latch gates, then the
    /// medium half's command runs.
    #[cfg(not(feature = "experiment"))]
    pub fn drive(
        &mut self,
        m: &Measured,
        cfg: &KernelConfig,
        cmd: &Command,
        faults: &FaultLatch,
    ) -> MotorCmd {
        // a fault this tick's measurement or an earlier medium step raised
        // disables this drive; the loop-reset bookkeeping follows at the
        // medium step
        let drive = if faults.mask() == 0 {
            cmd.drive
        } else {
            core::hint::cold_path();
            Drive::Off
        };
        // the current loop runs on the fall-through: the other kinds sit
        // off the tick-budget path
        if let Drive::Closed { u_ff_vc } = drive {
            if cmd.i_ref_cc == 0 && m.i_meas.is_none() && (cmd.band.hi == 0 || cmd.band.lo == 0) {
                core::hint::cold_path();
                // endstop-clamped ref with no window: the loop below
                // would command the feed-forward alone, and a zero duty
                // holds nothing. Brake shorts the winding, passively
                // holding against whatever momentum remains; it must be
                // commanded explicitly - chip-side Drive{0} maps to coast.
                // Scoped to a collapsed band so a transient i_ref zero
                // crossing in normal travel can never reset the loop
                // mid-reversal.
                self.cur.reset();
                self.duty_q15 = 0;
                MotorCmd::Brake
            } else {
                // No window only happens under the sampling floor, where
                // the bridge is a voltage source and nothing reads the
                // winding: the loop commands the reference's volts
                // (`CurrentLoop::step`) and the held current is that
                // reference. Fed a zero instead, the integrator moves
                // `ki x i_ref` a tick whatever the winding carries and
                // walks the command across the band
                // (`the_command_never_integrates_under_the_shunt_floor`).
                // The overcurrent trip and the back-EMF sums keep the
                // strict validity view.
                // The first window read after a command the bridge drops
                // reads far over the winding current, and on the
                // proportional gain that one reading reverses the drive:
                // the loop takes it as no reading. `ticks` still holds
                // the width this frame measured, the one the next tick's
                // window follows; a width of 0 wraps out of the range.
                let taken = m.i_meas.filter(|_| !self.dropped);
                self.dropped = self.ticks.wrapping_sub(1) < self.dropped_max_ticks;
                if m.i_meas.is_none() {
                    self.i_meas_last =
                        sat_clamp(cmd.i_ref_cc, i16::MIN as i32, i16::MAX as i32) as i16;
                }
                let duty = self.cur.step(
                    cmd.i_ref_cc,
                    taken,
                    u_ff_vc,
                    &cmd.rail,
                    cmd.vbus_recip_q15,
                    &cfg.loop_current,
                    cmd.r_loop_q12,
                );
                self.duty_q15 = duty;
                self.drive_cmd(duty, cfg.limits.drive_polarity)
            }
        } else if let Drive::Brake = drive {
            // anti-hunt: park instead of dithering on friction. The first
            // closed tick after it has no reading (a zero width voids the
            // window), so the loop restarts at `r x i_ref`. The winding
            // short brakes a shaft that enters the band at speed
            // (Ke x omega / R, zero at rest, nothing from the rail). On
            // friction alone a fast arrival leaves the far edge, and the
            // wake kicks it back through at the current limit: a limit
            // cycle. Not with the cold kinds below: a hold at rest parks
            // here every tick.
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
                        // chip-side Drive{0} maps to coast; a winding short
                        // must be commanded explicitly
                        MotorCmd::Brake
                    } else {
                        self.drive_cmd(duty, cfg.limits.drive_polarity)
                    }
                }
                // Off; Closed took the branch above
                _ => {
                    self.duty_q15 = 0;
                    MotorCmd::Disabled
                }
            }
        }
    }

    /// The motor command for this tick on the experiment image: the fault
    /// latch gates, then the OpenLoop goal drives as written, no ceiling,
    /// no endstop, no base duty.
    #[cfg(feature = "experiment")]
    pub fn drive(
        &mut self,
        _: &Measured,
        cfg: &KernelConfig,
        cmd: &Command,
        faults: &FaultLatch,
    ) -> MotorCmd {
        // a fault this tick's measurement or an earlier medium step raised
        // disables this drive
        let drive = if faults.mask() == 0 {
            cmd.drive
        } else {
            core::hint::cold_path();
            Drive::Off
        };
        match drive {
            Drive::OpenLoop { goal_q15: goal } => {
                // a reversal or a zero goal: the held current restarts from 0
                if goal.signum() != (self.duty_q15 as i32).signum() {
                    self.i_meas_last = 0;
                }
                let duty = goal as i16;
                self.duty_q15 = duty;
                if duty == 0 && cfg.limits.openloop_zero_brake {
                    // chip-side Drive{0} maps to coast; a winding short
                    // must be commanded explicitly
                    MotorCmd::Brake
                } else {
                    self.drive_cmd(duty, cfg.limits.drive_polarity)
                }
            }
            Drive::Off => {
                self.duty_q15 = 0;
                MotorCmd::Disabled
            }
        }
    }

    /// After the write of `out`: the bridge state it leaves and its settle
    /// toward a zero-current sample; off and the brake drive nothing, so
    /// the held current drops (`i_meas_last`). The measured window's state
    /// is kept first, for the voltage side and the medium step.
    #[inline(always)]
    pub fn book(&mut self, out: MotorCmd) {
        self.seen.ticks = self.ticks;
        self.seen.bridge = self.bridge;
        self.seen.settle = self.settle;
        self.follow(out);
    }

    /// The drive written off outside the tick's own write: the measured
    /// window's state stays as the tick's write kept it.
    pub fn book_off(&mut self) {
        self.duty_q15 = 0;
        self.follow(MotorCmd::Disabled);
    }

    #[inline(always)]
    fn follow(&mut self, out: MotorCmd) {
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
        match out {
            MotorCmd::Drive { ticks, .. } => self.ticks = ticks,
            _ => {
                self.ticks = 0;
                self.i_meas_last = 0;
            }
        }
    }

    /// The drive of `duty` (logical: +duty moves counts up) in the wiring's
    /// frame; `duty_q15` and the published duty stay logical. A zero width
    /// coasts whatever its `reverse`.
    fn drive_cmd(&self, duty: i16, drive_polarity: bool) -> MotorCmd {
        MotorCmd::Drive {
            ticks: window::drive_ticks(duty, self.pwm_arr) as u16,
            reverse: (duty < 0) == drive_polarity,
        }
    }
}
