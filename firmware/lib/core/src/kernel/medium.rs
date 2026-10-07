//! The medium-rate half of the kernel: reads CONTROL, runs the edges, the
//! estimators, the outer loops, the limits, the detectors and the slow
//! block, publishes, and leaves the fast half its `Command`. The work is
//! split into phases (`phase`), one stage of the chain each; every value
//! is written by the phase that computes it and read by the later phases
//! and by the fast half.

use super::config::{FastConfig, KernelConfig};
use super::fast::{Command, Drive, Fast, Measured};
use super::faults::{self, Detectors, FaultLatch};
use super::limits::{self, IBand, LimitState};
use super::trajectory::TrajGen;
use super::velocity::VelocityLoop;
use super::{
    DECIM_MED, DECIM_SLOW, Elapsed, KernelTiming, LOST_MAX, PERMIT_LEASE_TICKS, TICK_SHARE_Q16,
    position,
};
use crate::estimator::{FusionObs, OmegaSwitch, VbusEst, WindingTherm, bemf};
use crate::math::{q_mul, q_mul_u};
use crate::pos_lut;
use crate::regions::control::{ControlLifecycle, Mode};
use crate::{RegionStorageRaw, SensorFrame, Shared};

/// The stages of the medium chain, in data-flow order.
pub mod phase {
    /// Configuration, CONTROL, the edges and the drive kind.
    pub const CONTROL: u8 = 0;
    /// Back-EMF close, observer step, speed source switch.
    pub const OBSERVER: u8 = 1;
    /// Trajectory and position loop; the hold updates the drive kind.
    pub const TRAJECTORY: u8 = 2;
    /// Limits fold, OpenLoop base duty, stall fault.
    pub const LIMITS: u8 = 3;
    /// Velocity loop or the current clamp; reference and band land.
    pub const VELOCITY: u8 = 4;
    /// Rail estimate, sensor screen, position error timer.
    pub const RAIL: u8 = 5;
    /// The slow block, every DECIM_SLOW periods.
    pub const SLOW: u8 = 6;
    /// Estimates, mode, limits and sensors publish.
    pub const PUBLISH: u8 = 7;
    /// Phases in use; the rest of the period is free.
    pub const COUNT: u8 = 8;
}

/// The SLOW period in kernel ticks: 16 ms at 20 kHz.
const SLOW_PERIOD_TICKS: u16 = DECIM_SLOW as u16 * DECIM_MED as u16;

/// One read of CONTROL.
#[derive(Copy, Clone, Default)]
pub struct Control {
    pub life: ControlLifecycle,
    /// `pos_lut_state` reads LIVE: the position table applies.
    pub lut_live: bool,
    pub ident_agg: bool,
}

impl Control {
    pub fn read(shared: &Shared) -> Self {
        let p = shared.table.region_ptr();
        // SAFETY: reads of transport-owned regions - raw-pointer volatile
        // block copies, no `&T` formed, aligned repr(C) blocks inside the
        // static table (single-writer contract on `Kernel`).
        let (life, pos_lut_state, ident_agg) = unsafe {
            (
                (&raw const (*p).control.lifecycle).read_volatile(),
                (&raw const (*p).control.pos_lut.pos_lut_state).read_volatile(),
                (&raw const (*p).control.ident.ident_agg).read_volatile(),
            )
        };
        Self {
            life,
            lut_live: pos_lut_state == pos_lut::state::LIVE,
            ident_agg,
        }
    }
}

/// The linearized pot (pos_lut module), Q4 counts: the one measurement
/// behind both seeds and the observer, so theta_hat and everything that
/// reads it (trajectory, position loop, soft limits, stall gates) are in
/// linearized counts. The raw pot stays for the sensors publish, TEL `pos`
/// and the glitch screen.
pub fn pos_q4(shared: &Shared, lut_live: bool, raw: u16) -> u16 {
    if lut_live {
        shared.pos_lut_q4(raw)
    } else {
        pos_lut::identity_q4(raw)
    }
}

pub struct Medium {
    /// `KernelTiming::dt_med_q32`.
    dt_med_q32: u32,
    /// `KernelTiming::dt_tick_q32`.
    dt_tick_q32: u32,
    /// `KernelTiming::recip_arr_q24`.
    recip_arr_q24: u32,
    /// Ticks reported lost since the CONTROL phase last ran.
    lost: u8,
    /// The time this period's steps integrate over.
    pub(super) elapsed: Elapsed,
    /// Kernel ticks elapsed toward the next SLOW run.
    pub(super) slow_ticks: u16,
    /// CONTROL as the CONTROL phase read it.
    ctl: Control,
    /// Torque on and no fault at the CONTROL phase: the loops run.
    run: bool,
    pub(super) traj: TrajGen,
    pub(super) fusion: FusionObs,
    /// The observer phase's picks, for the later phases.
    pub(super) omega_hat: i32,
    omega_bemf: Option<i32>,
    /// Linearized counts per raw count at the observer's sample.
    band_gain_q4: u16,
    vel: VelocityLoop,
    limits: LimitState,
    pub(super) i_band: IBand,
    limit_flags: u8,
    pub(super) vbus: VbusEst,
    thermal: WindingTherm,
    /// Velocity-loop feedback pick: back-EMF boxcar or the observer's omega.
    pub(super) omega_sw: OmegaSwitch,
    /// The duty OpenLoop applies while its ceiling sits under the window
    /// floor.
    ol_base_q15: u16,
    /// SLOW ticks left on the stall permit lease, renewed when HIGH moves
    /// `Shared::permit_gen` past `permit_gen`.
    permit_ticks: u8,
    permit_gen: u8,
    det: Detectors,
    te_prev: bool,
    run_prev: bool,
    mode_prev: Mode,
    /// The current command the fast half's loop tracks.
    pub(super) i_ref_cc: i32,
    /// Position loop output, held for the velocity step.
    pub(super) omega_ref_q16: i32,
    /// Anti-hunt hold from the position loop; the drive parks on it.
    pub(super) hold: bool,
}

impl Medium {
    pub fn new(timing: &KernelTiming) -> Self {
        Self {
            dt_med_q32: timing.dt_med_q32,
            dt_tick_q32: timing.dt_tick_q32,
            recip_arr_q24: timing.recip_arr_q24,
            lost: 0,
            elapsed: Elapsed {
                ticks: DECIM_MED as u32,
                dt_q32: timing.dt_med_q32,
                lost_q16: 0,
            },
            // primed so the FIRST period runs the slow block: the
            // thermometer and the derate exist before any consumer sees them
            slow_ticks: SLOW_PERIOD_TICKS - DECIM_MED as u16,
            ctl: Control::default(),
            run: false,
            traj: TrajGen::new(),
            fusion: FusionObs::new(),
            omega_hat: 0,
            omega_bemf: None,
            band_gain_q4: pos_lut::GRID as u16,
            vel: VelocityLoop::new(),
            limits: LimitState::new(),
            i_band: IBand { lo: 0, hi: 0 },
            limit_flags: 0,
            vbus: VbusEst::new(timing.vbus_scale_q15),
            thermal: WindingTherm::new(),
            omega_sw: OmegaSwitch::new(),
            ol_base_q15: 0,
            permit_ticks: 0,
            permit_gen: 0,
            det: Detectors::new(),
            te_prev: false,
            run_prev: false,
            mode_prev: Mode::OpenLoop,
            i_ref_cc: 0,
            omega_ref_q16: 0,
            hold: false,
        }
    }

    /// First tick: the observer starts at the measurement.
    pub fn seed(&mut self, pos_q4: u16) {
        self.fusion.seed(pos_q4);
    }

    /// Thermometer seed from the calib anchor.
    pub fn seed_thermal(&mut self, r0_q12: u16) {
        self.thermal.seed(r0_q12);
    }

    /// Lost ticks the chip reports (`Kernel::lost_ticks`); the next CONTROL
    /// phase folds the total into `elapsed` for the phases after it, so a
    /// loss lands in the period it is reported in or the next.
    pub fn lost_ticks(&mut self, lost: u16) {
        self.lost = self.lost.saturating_add(lost.min(LOST_MAX as u16) as u8);
    }

    /// Phase `k` of the medium chain (`phase`), on this tick's sample.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        k: u8,
        frame: &SensorFrame,
        m: &Measured,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
    ) {
        match k {
            phase::CONTROL => self.control(frame.pos, &cfg.fast, shared, faults, fast, cmd),
            phase::OBSERVER => self.observe(frame.pos, cfg, shared, fast),
            phase::TRAJECTORY => self.trajectory(cfg, faults, cmd),
            phase::LIMITS => self.limit(cfg, shared, faults, fast),
            phase::VELOCITY => self.velocity(cfg, cmd),
            phase::RAIL => self.rail(frame, cfg, faults, cmd),
            phase::SLOW => self.slow(m, cfg, faults),
            phase::PUBLISH => self.publish(frame, shared, faults, fast),
            _ => {}
        }
    }

    /// CONTROL: the period's time, the read, the edges, and the drive kind
    /// they imply.
    fn control(
        &mut self,
        raw_pos: u16,
        fc: &FastConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
    ) {
        let lost = self.lost.min(LOST_MAX) as u32;
        self.lost = 0;
        self.elapsed = Elapsed {
            ticks: DECIM_MED as u32 + lost,
            dt_q32: self.dt_med_q32 + lost * self.dt_tick_q32,
            lost_q16: lost * TICK_SHARE_Q16,
        };
        self.ctl = Control::read(shared);
        self.admit(fc, shared, faults, fast);
        self.edges(raw_pos, fc, shared, faults, fast);
        cmd.drive = self.drive(fc, faults);
        // the resets of an edge reach the fast half with the drive they
        // change
        cmd.i_ref_cc = self.i_ref_cc;
        cmd.lut_live = self.ctl.lut_live;
        if cmd.ident_agg && !self.ctl.ident_agg {
            fast.restart_ident();
        }
        cmd.ident_agg = self.ctl.ident_agg;
    }

    /// The torque enable edge, the data-state entry check and the Ke belt.
    fn admit(
        &mut self,
        fc: &FastConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
    ) {
        let life = &self.ctl.life;
        // torque_enable 0->1 is the fault ack: latch, detectors, and the
        // limits pend all clear; a still-present condition re-latches
        // through the normal detectors.
        let enable_edge = life.torque_enable && !self.te_prev;
        if enable_edge {
            faults.clear();
            self.det.reset();
            fast.ack();
            self.limits.ack();
        }
        self.te_prev = life.torque_enable;

        // Data-state entry check (data_state module): at the enable edge and
        // at a mode change under torque, a named reason refuses closed loop
        // for this run. A reason that appears mid-run (a live edit) waits
        // for the next entry; only the Ke belt below stops a running loop.
        if life.torque_enable && (enable_edge || life.mode != self.mode_prev) {
            let p = shared.table.region_ptr();
            // SAFETY: same volatile read contract as `Control::read`; boot
            // and the dispatcher own this byte, the kernel only reads it.
            let data = unsafe { (&raw const (*p).telemetry.mode.data_flags).read_volatile() };
            if !crate::data_state::allows(life.mode, data) {
                faults.raise(faults::BIT_DATA, faults::CODE_DATA);
            }
        }
        // Physics belt: a closed loop on a zero Ke runs open (the boxcar
        // yields nothing, the current loop decouples nothing), whatever the
        // flags say - a live write of 0 into a running loop stops it here.
        if life.torque_enable && matches!(life.mode, Mode::Velocity | Mode::Position) && fc.ke_unset
        {
            faults.raise(faults::BIT_DATA, faults::CODE_DATA);
        }
    }

    /// The run and mode edges.
    fn edges(
        &mut self,
        raw_pos: u16,
        fc: &FastConfig,
        shared: &Shared,
        faults: &FaultLatch,
        fast: &mut Fast,
    ) {
        let life = &self.ctl.life;
        let run = life.torque_enable && faults.mask() == 0;
        if run != self.run_prev {
            // both edges zero the loop chain; the enable edge additionally
            // reseeds fusion at the measurement (torque-off tau_d is built
            // on a zero current - a hand-moved shaft rails it and every
            // enable would re-latch STALL via the collision check) and the
            // profile at the fresh estimate - bumpless
            if run {
                self.fusion.seed(pos_q4(shared, self.ctl.lut_live, raw_pos));
                self.traj.reseed(self.fusion.theta_q16());
            }
            fast.reset_current_loop();
            self.vel.reset();
            fast.reset_limiter(fc.ol_floor_q15);
            self.i_ref_cc = 0;
            self.omega_ref_q16 = 0;
            self.hold = false;
            self.run_prev = run;
        }
        if run && life.mode != self.mode_prev {
            // mode change mid-run: reference = estimate, at rest
            self.traj.reseed(self.fusion.theta_q16());
            fast.reset_limiter(fc.ol_floor_q15);
            self.hold = false;
        }
        self.mode_prev = life.mode;
        self.run = run;
    }

    /// The drive kind the fast half runs: off, the hold's park, OpenLoop
    /// at its clamped goal, or the current loop.
    fn drive(&self, fc: &FastConfig, faults: &FaultLatch) -> Drive {
        let life = &self.ctl.life;
        if !life.torque_enable || faults.mask() != 0 {
            return Drive::Off;
        }
        match life.mode {
            Mode::OpenLoop => {
                let max = fc.ol_duty_max_q15 as i32;
                Drive::OpenLoop {
                    goal_q15: (life.goal_duty as i32).clamp(-max, max),
                }
            }
            _ if self.hold => Drive::Brake,
            // Ke decoupling rides the profile, not an estimate (current.rs
            // step doc); Current mode has no profile
            Mode::Velocity | Mode::Position => Drive::Closed {
                omega_ff_q16: self.traj.omega_star_q16(),
            },
            Mode::Current => Drive::Closed { omega_ff_q16: 0 },
        }
    }

    /// OBSERVER, on this tick's sample.
    fn observe(&mut self, raw_pos: u16, cfg: &KernelConfig, shared: &Shared, fast: &mut Fast) {
        let mc = &cfg.medium;
        let lut_live = self.ctl.lut_live;
        self.omega_bemf =
            fast.close_bemf_half(mc.r_q12, mc.l_tick_q412, mc.recip_ke_q, self.recip_arr_q24);
        self.fusion.step(
            fast.i_meas_last() as i32,
            pos_q4(shared, lut_live, raw_pos),
            &self.elapsed,
            &mc.fusion,
        );
        // The observer's omega keeps the rest-shaped consumers (stall
        // verdict, thermometer gate): it is always there and reads small at
        // rest, where the boxcar has no window at all.
        self.omega_hat = self.omega_sw.step(self.omega_bemf, self.fusion.omega_q16());
        // the interval this sample linearized in, for the hold band
        self.band_gain_q4 = if lut_live {
            shared.pos_lut_band_gain_q4(raw_pos)
        } else {
            pos_lut::GRID as u16
        };
    }

    /// TRAJECTORY: the profile and the position loop.
    fn trajectory(&mut self, cfg: &KernelConfig, faults: &FaultLatch, cmd: &mut Command) {
        if !self.run {
            return;
        }
        let mc = &cfg.medium;
        let life = &self.ctl.life;
        let theta_hat = self.fusion.theta_q16();
        match life.mode {
            Mode::Position => {
                self.traj
                    .step_position(life.goal_position, &self.elapsed, &mc.traj);
                let out = position::step(
                    self.traj.theta_star_q16(),
                    self.traj.omega_star_q16(),
                    theta_hat,
                    self.band_gain_q4,
                    &mc.position,
                );
                self.omega_ref_q16 = out.omega_ref_q16;
                self.hold = out.hold;
            }
            Mode::Velocity => {
                self.traj
                    .step_velocity(life.goal_velocity, theta_hat, &self.elapsed, &mc.traj);
                self.omega_ref_q16 = self.traj.omega_star_q16();
                self.hold = false;
            }
            Mode::Current | Mode::OpenLoop => self.hold = false,
        }
        cmd.drive = self.drive(&cfg.fast, faults);
    }

    /// LIMITS: the band, the OpenLoop base duty, the stall verdict.
    fn limit(
        &mut self,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
    ) {
        let (fc, mc) = (&cfg.fast, &cfg.medium);
        let life = &self.ctl.life;
        // pinned = last command sat at a nonzero ceiling, in OpenLoop the
        // duty ceiling held under the goal
        let prev_lim = self.limits.i_lim_counts();
        let pinned = if life.mode == Mode::OpenLoop {
            fast.take_pinned()
        } else {
            prev_lim != 0 && self.i_ref_cc.unsigned_abs() >= prev_lim as u32
        };
        let permit_gen = shared.permit_gen();
        if permit_gen != self.permit_gen {
            self.permit_gen = permit_gen;
            self.permit_ticks = PERMIT_LEASE_TICKS;
        }
        if !life.torque_enable {
            self.permit_ticks = 0;
        }
        let permit = life.stall_permit && self.permit_ticks != 0;
        let band = self.limits.fold(
            pinned,
            self.fusion.omega_q16().unsigned_abs() >> 16,
            self.fusion.tau_d_counts().unsigned_abs(),
            self.fusion.theta_q16() >> 16,
            permit,
            self.elapsed.ticks,
            &mc.limits,
        );
        self.i_band = band;
        // the stall-safe duty lim x R / vbus: winding R alone, so it errs low
        // by the bridge and shunt; unset R keeps it at the floor
        self.ol_base_q15 = if mc.r_q12 == 0 {
            fc.ol_floor_q15
        } else {
            let lim = if life.goal_duty >= 0 {
                band.hi
            } else {
                band.lo
            };
            let v = q_mul_u(lim.unsigned_abs(), mc.r_q12 as u32, 12);
            q_mul_u(v, self.vbus.recip_q15(), 15).min(fc.ol_floor_q15 as u32) as u16
        };
        if self.run && self.limits.stall_fault_pending() {
            faults.raise(faults::BIT_STALL, faults::CODE_STALL);
        }
        // one closed side makes the band asymmetric; a zero limit closes
        // both and is no endstop
        self.limit_flags = (pinned as u8 * limits::flag::CEILING)
            | (self.limits.stalled() as u8 * limits::flag::YIELD)
            | ((band.lo + band.hi != 0) as u8 * limits::flag::ENDSTOP)
            | (permit as u8 * limits::flag::PERMIT);
    }

    /// VELOCITY: the current reference, which lands in the command with
    /// the band and the base duty.
    fn velocity(&mut self, cfg: &KernelConfig, cmd: &mut Command) {
        if self.run {
            let life = &self.ctl.life;
            match life.mode {
                // Parked: nothing drives, so the velocity loop must stop too.
                // omega_hat is the pot observer at rest (no window,
                // +-hundreds c/s of noise); left running, the PI integrates
                // that phantom error until i_ref pins at the current limit,
                // and pinned + slow false-trips the stall detector (bench:
                // CODE_STALL a few seconds into a clean hold). Zero the
                // command and drain the integrator so it never winds and
                // resumes bumplessly.
                Mode::Position if self.hold => {
                    self.i_ref_cc = 0;
                    self.vel.reset();
                }
                Mode::Velocity | Mode::Position => {
                    self.i_ref_cc = self.vel.step(
                        self.omega_ref_q16,
                        self.omega_hat,
                        self.traj.alpha_star_q16(),
                        self.traj.omega_star_q16(),
                        self.i_band,
                        &cfg.medium.velocity,
                    );
                }
                // directional band: an endstop blocks only inward goals;
                // retreat clamps against the composed limit
                Mode::Current => self.i_ref_cc = self.i_band.clamp(life.goal_current as i32),
                Mode::OpenLoop => self.i_ref_cc = 0,
            }
        }
        cmd.i_ref_cc = self.i_ref_cc;
        cmd.band = self.i_band;
        cmd.ol_base_q15 = self.ol_base_q15;
    }

    /// RAIL: the rail estimate on this tick's tap, the sensor screen and
    /// the position error timer.
    fn rail(
        &mut self,
        frame: &SensorFrame,
        cfg: &KernelConfig,
        faults: &mut FaultLatch,
        cmd: &mut Command,
    ) {
        let (fc, mc) = (&cfg.fast, &cfg.medium);
        self.vbus.step(frame.vbus_raw, fc.v_undervolt_counts);
        cmd.vbus_counts = self.vbus.vbus_counts();
        cmd.vbus_recip_q15 = self.vbus.recip_q15();
        // raw-pot sanity screen runs in every mode, torque-off included
        if self
            .det
            .sensor_sample(frame.pos, mc.sensor_delta_max, mc.sensor_bad_count)
        {
            faults.raise(faults::BIT_SENSOR, faults::CODE_SENSOR);
        }
        // tracking-error persistence: only meaningful with a live profile
        let pos_err_over = self.run
            && self.ctl.life.mode == Mode::Position
            && self
                .traj
                .theta_star_q16()
                .saturating_sub(self.fusion.theta_q16())
                .unsigned_abs()
                > (mc.pos_error_counts as u32) << 16;
        if self
            .det
            .pos_err_sample(pos_err_over, self.elapsed.ticks, mc.pos_error_time_ticks)
        {
            faults.raise(faults::BIT_POSITION_ERROR, faults::CODE_POSITION_ERROR);
        }
    }

    /// SLOW, every SLOW_PERIOD_TICKS of elapsed time: the permit lease, the
    /// thermometer on this tick's window, the derate, overtemperature and
    /// undervoltage.
    fn slow(&mut self, m: &Measured, cfg: &KernelConfig, faults: &mut FaultLatch) {
        self.slow_ticks += self.elapsed.ticks as u16;
        if self.slow_ticks < SLOW_PERIOD_TICKS {
            return;
        }
        self.slow_ticks -= SLOW_PERIOD_TICKS;
        let (fc, mc) = (&cfg.fast, &cfg.medium);
        self.permit_ticks = self.permit_ticks.saturating_sub(1);
        // the LMS sample needs BOTH window paths valid (bemf RECIP_ARR
        // contract for v_mean)
        let (vm, therm_i) = match (m.vdiff, m.i_meas) {
            (Some(vdiff), Some(i)) => (
                q_mul(
                    m.ticks as i32 * vdiff,
                    self.recip_arr_q24 as i32,
                    bemf::RECIP_ARR_SHIFT,
                ),
                Some(i),
            ),
            _ => (0, None),
        };
        let t_cc = self.thermal.step(
            vm,
            therm_i,
            self.fusion.omega_q16().unsigned_abs() >> 16,
            &mc.therm_gates,
            &mc.therm_anchor,
        );
        self.limits.update_derate(t_cc, &mc.limits);
        if t_cc >= mc.limits.cutoff_cc {
            faults.raise(faults::BIT_OVER_TEMP, faults::CODE_OVER_TEMP);
        }
        // the rail tap samples drive or not, so a held sag never outlives the
        // sag itself: ack clears once the rail is back
        if self.vbus.vbus_counts() < fc.v_undervolt_counts {
            faults.raise(faults::BIT_UNDER_VOLT, faults::CODE_UNDER_VOLT);
        }
    }

    /// PUBLISH: sensors of this tick, the estimates, mode and limits.
    fn publish(&self, frame: &SensorFrame, shared: &Shared, faults: &FaultLatch, fast: &Fast) {
        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // per-field stores.
        unsafe {
            let s = &raw mut (*p).telemetry.sensors;
            (&raw mut (*s).pos).write_volatile(frame.pos);
            (&raw mut (*s).current).write_volatile(frame.current);
            (&raw mut (*s).vcal).write_volatile(frame.vcal);
            (&raw mut (*s).vcal_lpf).write_volatile(fast.vcal_lpf_counts());
            (&raw mut (*s).vmotor_a).write_volatile(frame.vmotor_a);
            (&raw mut (*s).vmotor_b).write_volatile(frame.vmotor_b);
            (&raw mut (*s).current_trough).write_volatile(frame.current_trough);
            (&raw mut (*s).vbus_raw).write_volatile(frame.vbus_raw);
            (&raw mut (*s).ntc_raw).write_volatile(frame.ntc_raw);
            (&raw mut (*s).current_bias_counts).write_volatile(fast.bias_counts());
            let e = &raw mut (*p).telemetry.estimates;
            (&raw mut (*e).theta_hat_q16).write_volatile(self.fusion.theta_q16());
            (&raw mut (*e).omega_hat_cps).write_volatile(self.omega_hat);
            (&raw mut (*e).tau_d_counts).write_volatile(self.fusion.tau_d_counts());
            (&raw mut (*e).i_lim_counts).write_volatile(self.limits.i_lim_counts());
            (&raw mut (*e).t_winding_cc).write_volatile(self.thermal.t_cc());
            (&raw mut (*e).vbus_counts).write_volatile(self.vbus.vbus_counts());
            (&raw mut (*e).duty_applied_q15).write_volatile(fast.duty_q15());
            (&raw mut (*e).omega_bemf_cps)
                .write_volatile(bemf::omega_cps_i16(self.omega_bemf.unwrap_or(0)));
            (&raw mut (*e).r_hat_q12).write_volatile(self.thermal.r_q12());
            (&raw mut (*e).i_hat_counts).write_volatile(fast.i_meas_last());
            let md = &raw mut (*p).telemetry.mode;
            (&raw mut (*md).mode_active).write_volatile(self.ctl.life.mode as u8);
            (&raw mut (*md).fault_code).write_volatile(faults.code());
            (&raw mut (*md).omega_hat_src).write_volatile(self.omega_sw.source() as u8);
            (&raw mut (*p).telemetry.common.fault_flags).write_volatile(faults.mask());
            (&raw mut (*p).telemetry.limits.limit_flags).write_volatile(self.limit_flags);
        }
    }
}
