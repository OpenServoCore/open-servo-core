//! The medium-rate half of the kernel: reads CONTROL, runs the edges, the
//! estimators, the outer loops, the limits, the detectors and the slow
//! block, publishes, and leaves the fast half its `Command`.

use super::config::{FastConfig, KernelConfig};
use super::fast::{Command, Drive, Fast, Measured};
use super::faults::{self, Detectors, FaultLatch};
use super::limits::{self, IBand, LimitState};
use super::trajectory::TrajGen;
use super::velocity::VelocityLoop;
use super::{DECIM_SLOW, KernelTiming, PERMIT_LEASE_TICKS, position};
use crate::estimator::{FusionObs, OmegaSwitch, VbusEst, WindingTherm, bemf};
use crate::math::{q_mul, q_mul_u};
use crate::pos_lut;
use crate::regions::control::{ControlLifecycle, Mode};
use crate::{RegionStorageRaw, SensorFrame, Shared};

/// One read of CONTROL.
pub struct Control {
    pub life: ControlLifecycle,
    /// `pos_lut_state` reads LIVE: the position table applies.
    pub lut_live: bool,
}

impl Control {
    pub fn read(shared: &Shared) -> Self {
        let p = shared.table.region_ptr();
        // SAFETY: reads of transport-owned regions - raw-pointer volatile
        // block copies, no `&T` formed, aligned repr(C) blocks inside the
        // static table (single-writer contract on `Kernel`).
        let (life, pos_lut_state) = unsafe {
            (
                (&raw const (*p).control.lifecycle).read_volatile(),
                (&raw const (*p).control.pos_lut.pos_lut_state).read_volatile(),
            )
        };
        Self {
            life,
            lut_live: pos_lut_state == pos_lut::state::LIVE,
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
    decim_slow: u8,
    pub(super) traj: TrajGen,
    pub(super) fusion: FusionObs,
    vel: VelocityLoop,
    limits: LimitState,
    pub(super) i_band: IBand,
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
    pub fn new(vbus_scale_q15: u32) -> Self {
        Self {
            // primed so the FIRST medium step runs the slow block: the
            // thermometer and the derate exist before any consumer sees them
            decim_slow: DECIM_SLOW - 1,
            traj: TrajGen::new(),
            fusion: FusionObs::new(),
            vel: VelocityLoop::new(),
            limits: LimitState::new(),
            i_band: IBand { lo: 0, hi: 0 },
            vbus: VbusEst::new(vbus_scale_q15),
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

    /// The torque enable edge, the data-state entry check and the Ke belt.
    pub fn admit(
        &mut self,
        life: &ControlLifecycle,
        fc: &FastConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
    ) {
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

    /// The run and mode edges; returns whether the loops run.
    pub fn edges(
        &mut self,
        ctl: &Control,
        raw_pos: u16,
        fc: &FastConfig,
        shared: &Shared,
        faults: &FaultLatch,
        fast: &mut Fast,
    ) -> bool {
        let life = &ctl.life;
        let run = life.torque_enable && faults.mask() == 0;
        if run != self.run_prev {
            // both edges zero the loop chain; the enable edge additionally
            // reseeds fusion at the measurement (torque-off tau_d is built
            // from i_use = 0 fiction - a hand-moved shaft rails it and every
            // enable would re-latch STALL via the collision check) and the
            // profile at the fresh estimate - bumpless
            if run {
                self.fusion.seed(pos_q4(shared, ctl.lut_live, raw_pos));
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
        run
    }

    /// The medium chain: estimators, outer loops, limits, detectors, the
    /// slow block and the estimates publish.
    #[allow(clippy::too_many_arguments)]
    pub fn chain(
        &mut self,
        frame: &SensorFrame,
        m: &Measured,
        run: bool,
        ctl: &Control,
        cfg: &KernelConfig,
        timing: &KernelTiming,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
    ) {
        let (fc, mc) = (&cfg.fast, &cfg.medium);
        let life = &ctl.life;
        let pos_q4 = pos_q4(shared, ctl.lut_live, frame.pos);
        // i_use: window-valid measurement, else the cached command - the
        // observer never sees the validity flag (fusion contract). While
        // disabled or in OpenLoop the cache is 0, so an invalid window
        // predicts torque-free.
        let i_use = m.i_meas.unwrap_or(self.i_ref_cc);
        let omega_bemf = fast.close_bemf_half(mc.r_q12, mc.recip_ke_q, timing.recip_arr_q24);
        self.fusion
            .step(i_use, pos_q4, timing.dt_med_q32, &mc.fusion);
        let theta_hat = self.fusion.theta_q16();
        // The observer's omega keeps the rest-shaped consumers (stall
        // verdict, thermometer gate): it is always there and reads small at
        // rest, where the boxcar has no window at all.
        let omega_pot = self.fusion.omega_q16();
        let omega_hat = self.omega_sw.step(omega_bemf, omega_pot);

        if run {
            match life.mode {
                Mode::Position => {
                    self.traj.step_position(life.goal_position, &mc.traj);
                    // the interval this tick linearized in: same raw sample,
                    // same torque-gated table as the observer's read
                    let band_gain_q4 = if ctl.lut_live {
                        shared.pos_lut_band_gain_q4(frame.pos)
                    } else {
                        pos_lut::GRID as u16
                    };
                    let out = position::step(
                        self.traj.theta_star_q16(),
                        self.traj.omega_star_q16(),
                        theta_hat,
                        band_gain_q4,
                        &mc.position,
                    );
                    self.omega_ref_q16 = out.omega_ref_q16;
                    self.hold = out.hold;
                }
                Mode::Velocity => {
                    self.traj
                        .step_velocity(life.goal_velocity, theta_hat, &mc.traj);
                    self.omega_ref_q16 = self.traj.omega_star_q16();
                    self.hold = false;
                }
                Mode::Current | Mode::OpenLoop => self.hold = false,
            }
        }

        // limits fold; pinned = last command sat at a nonzero ceiling, in
        // OpenLoop the duty ceiling held under the goal
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
        let omega_abs_cps = omega_pot.unsigned_abs() >> 16;
        let band = self.limits.fold(
            pinned,
            omega_abs_cps,
            self.fusion.tau_d_counts().unsigned_abs(),
            theta_hat >> 16,
            permit,
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
        if run && self.limits.stall_fault_pending() {
            faults.raise(faults::BIT_STALL, faults::CODE_STALL);
        }

        if run {
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
                        omega_hat,
                        self.traj.alpha_star_q16(),
                        self.traj.omega_star_q16(),
                        band,
                        &mc.velocity,
                    );
                }
                // clamped against the band by the command
                Mode::Current => {}
                Mode::OpenLoop => self.i_ref_cc = 0,
            }
        }

        self.vbus.step(frame.vbus_raw, fc.v_undervolt_counts);
        // this tick's v_mean for the thermometer at SLOW (bemf RECIP_ARR
        // contract)
        let v_mean = m.vdiff.map(|vdiff| {
            q_mul(
                m.ticks as i32 * vdiff,
                timing.recip_arr_q24 as i32,
                bemf::RECIP_ARR_SHIFT,
            )
        });

        // raw-pot sanity screen runs in every mode, torque-off included
        if self
            .det
            .sensor_sample(frame.pos, mc.sensor_delta_max, mc.sensor_bad_count)
        {
            faults.raise(faults::BIT_SENSOR, faults::CODE_SENSOR);
        }
        // tracking-error persistence: only meaningful with a live profile
        let pos_err_over = run
            && life.mode == Mode::Position
            && self
                .traj
                .theta_star_q16()
                .saturating_sub(theta_hat)
                .unsigned_abs()
                > (mc.pos_error_counts as u32) << 16;
        if self
            .det
            .pos_err_sample(pos_err_over, mc.pos_error_time_ticks)
        {
            faults.raise(faults::BIT_POSITION_ERROR, faults::CODE_POSITION_ERROR);
        }

        self.decim_slow += 1;
        if self.decim_slow >= DECIM_SLOW {
            self.decim_slow = 0;
            self.permit_ticks = self.permit_ticks.saturating_sub(1);
            // the LMS sample needs BOTH window paths valid
            let (vm, therm_i) = match (v_mean, m.i_meas) {
                (Some(v), Some(i)) => (v, Some(i)),
                _ => (0, None),
            };
            let t_cc = self.thermal.step(
                vm,
                therm_i,
                omega_abs_cps,
                &mc.therm_gates,
                &mc.therm_anchor,
            );
            self.limits.update_derate(t_cc, &mc.limits);
            if t_cc >= mc.limits.cutoff_cc {
                faults.raise(faults::BIT_OVER_TEMP, faults::CODE_OVER_TEMP);
            }
            // the rail tap samples drive or not, so a held sag never outlives
            // the sag itself: ack clears once the rail is back
            if self.vbus.vbus_counts() < fc.v_undervolt_counts {
                faults.raise(faults::BIT_UNDER_VOLT, faults::CODE_UNDER_VOLT);
            }
        }

        // one closed side makes the band asymmetric; a zero limit closes
        // both and is no endstop
        let limit_flags = (pinned as u8 * limits::flag::CEILING)
            | (self.limits.stalled() as u8 * limits::flag::YIELD)
            | ((band.lo + band.hi != 0) as u8 * limits::flag::ENDSTOP)
            | (permit as u8 * limits::flag::PERMIT);

        let p = shared.table.region_ptr();
        // SAFETY: sole-telemetry-writer contract (`Kernel` doc); volatile
        // per-field stores, medium-boundary publish.
        unsafe {
            let e = &raw mut (*p).telemetry.estimates;
            (&raw mut (*e).theta_hat_q16).write_volatile(theta_hat);
            (&raw mut (*e).omega_hat_cps).write_volatile(omega_hat);
            (&raw mut (*e).tau_d_counts).write_volatile(self.fusion.tau_d_counts());
            (&raw mut (*e).i_lim_counts).write_volatile(self.limits.i_lim_counts());
            (&raw mut (*e).t_winding_cc).write_volatile(self.thermal.t_cc());
            (&raw mut (*e).vbus_counts).write_volatile(self.vbus.vbus_counts());
            (&raw mut (*e).duty_applied_q15).write_volatile(fast.duty_q15());
            (&raw mut (*e).omega_bemf_cps)
                .write_volatile(bemf::omega_cps_i16(omega_bemf.unwrap_or(0)));
            (&raw mut (*e).r_hat_q12).write_volatile(self.thermal.r_q12());
            (&raw mut (*e).i_hat_counts).write_volatile(fast.i_meas_last());
            let md = &raw mut (*p).telemetry.mode;
            (&raw mut (*md).mode_active).write_volatile(life.mode as u8);
            (&raw mut (*md).fault_code).write_volatile(faults.code());
            (&raw mut (*md).omega_hat_src).write_volatile(self.omega_sw.source() as u8);
            (&raw mut (*p).telemetry.common.fault_flags).write_volatile(faults.mask());
            (&raw mut (*p).telemetry.limits.limit_flags).write_volatile(limit_flags);
        }
    }

    /// The command the fast half drives until the next one.
    pub fn command(&mut self, ctl: &Control, fc: &FastConfig, faults: &FaultLatch) -> Command {
        let life = &ctl.life;
        let drive = if !life.torque_enable || faults.mask() != 0 {
            Drive::Off
        } else {
            match life.mode {
                Mode::OpenLoop => {
                    let max = fc.ol_duty_max_q15 as i32;
                    Drive::OpenLoop {
                        goal_q15: (life.goal_duty as i32).clamp(-max, max),
                    }
                }
                mode => {
                    if mode == Mode::Current {
                        // directional band: an endstop blocks only inward
                        // goals; retreat clamps against the composed limit
                        self.i_ref_cc = self.i_band.clamp(life.goal_current as i32);
                    }
                    if self.hold {
                        Drive::Brake
                    } else {
                        // Ke decoupling rides the profile, not an estimate
                        // (current.rs step doc); Current mode has no profile
                        let omega_ff_q16 = match mode {
                            Mode::Velocity | Mode::Position => self.traj.omega_star_q16(),
                            Mode::Current | Mode::OpenLoop => 0,
                        };
                        Drive::Closed { omega_ff_q16 }
                    }
                }
            }
        };
        Command {
            drive,
            i_ref_cc: self.i_ref_cc,
            band: self.i_band,
            ol_base_q15: self.ol_base_q15,
            vbus_counts: self.vbus.vbus_counts(),
            vbus_recip_q15: self.vbus.recip_q15(),
            lut_live: ctl.lut_live,
        }
    }
}
