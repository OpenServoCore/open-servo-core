//! Kernel implementation.
//!
//! This implements two planes:
//! - Realtime plane: `update_frame()` and `tick()`
//! - Host plane: sideband commands (mode, fault ack, persist, reset) via HostOp
//!
//! Register I/O is handled exclusively via shadow table (ShadowKernel trait).
//!
//! Stage-0 policy choices (easy to change later):
//! - Mode switch allowed only when disengaged
//! - Fault gating returns `MotorCommand::safe()`
//! - Disengage resets the PID integrator (simple, deterministic)

use open_servo_hw::v2::io::{DriveMode, MotorCommand, MotorHints, SensorFrame};
use open_servo_kernel_api::faults::FaultSink;
use open_servo_kernel_api::faults::GateReason;
use open_servo_kernel_api::host_op::{HostError, HostOp, HostResp, HostResult};
use open_servo_kernel_api::kernel::{Kernel, KernelHost};
use open_servo_kernel_api::mode::{ModeError, ModeRequest, OperatingMode};
use open_servo_kernel_api::reset::ResetScope;
use open_servo_kernel_api::shadow::{CommitResult, KernelView, ShadowKernel};
use open_servo_kernel_api::telemetry::ids as tid;
use open_servo_kernel_api::TelemetrySink;
use open_servo_kernel_api::{TickCtx, TickDomain};
use open_servo_units::{Effort, MicroSecond};

use crate::shadow_fields::{ctrl, telem};
use crate::state::{KernelConfig, KernelState, PendingOps};

/// Concrete kernel.
pub struct ServoKernel {
    pub cfg: KernelConfig,
    pub st: KernelState,

    /// Pending mode request (deferred policy hook).
    pending_mode: Option<OperatingMode>,

    /// Pending host operations (deferred to safe boundary).
    pending: PendingOps,
}

impl ServoKernel {
    #[inline]
    pub fn new(cfg: KernelConfig) -> Self {
        Self {
            cfg,
            st: KernelState::default(),
            pending_mode: None,
            pending: PendingOps::default(),
        }
    }

    // =========================================================================
    // Realtime plane
    // =========================================================================

    /// Update the latest sensor frame.
    ///
    /// Must not advance time; safe to call 0..N times between ticks.
    #[inline]
    pub fn update_frame(&mut self, frame: SensorFrame) {
        self.st.update_frame(frame);
    }

    /// Main tick entrypoint.
    ///
    /// The board decides how often to call each domain; the kernel defines behavior.
    pub fn tick<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> MotorCommand
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        // Domain-specific work.
        match ctx.tick.domain {
            TickDomain::ControlFast => self.tick_fast(ctx),
            TickDomain::ControlMedium => {
                self.tick_medium(ctx);
                self.st.last_cmd
            }
            TickDomain::ControlSlow => {
                self.tick_slow(ctx);
                self.st.last_cmd
            }
            TickDomain::System => {
                self.tick_system(ctx);
                self.st.last_cmd
            }
        }
    }

    fn tick_fast<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> MotorCommand
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        let gate = self.compute_gate_reason();
        self.st.last_gate = gate;

        // Emit a few high-value telemetry signals in fast tick (optional).
        ctx.telem.sample_u32(tid::MODE, self.st.mode as u32);
        ctx.telem
            .sample_u32(tid::ENGAGED, if self.st.engaged { 1 } else { 0 });
        ctx.telem.sample_i32(tid::POS_CDEG32, self.st.pos.as_cdeg());
        ctx.telem
            .sample_i32(tid::EFFORT_RAW, self.st.last_cmd.effort.as_raw() as i32);

        if gate != GateReason::Ok {
            let cmd = MotorCommand::safe();
            self.st.last_cmd = cmd;
            return cmd;
        }

        // Select control law by operating mode.
        let effort = match self.st.mode {
            OperatingMode::OpenLoop => self.st.open_loop_effort,
            OperatingMode::Position => self.position_pid(ctx.tick.dt),
        };

        // Stage-0 compliance-ish behavior:
        // - if near setpoint, coast (reduce gear stress)
        // - otherwise drive (slow decay by default)
        let pos_err = self.st.pos_sp.as_cdeg() - self.st.pos.as_cdeg();
        let near = pos_err.abs() <= self.cfg.hold_deadband_cdeg;

        let cmd = if near {
            MotorCommand {
                driver_en: true,
                mode: DriveMode::Coast,
                effort: Effort::ZERO,
                hints: MotorHints::default(),
            }
        } else {
            MotorCommand {
                driver_en: true,
                mode: DriveMode::Drive,
                effort,
                hints: MotorHints::default(),
            }
        };

        self.st.last_cmd = cmd;
        cmd
    }

    fn tick_medium<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>)
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        // Stage-0: no windowing yet.
        // This is where WindowStats/accumulators will live later.
        ctx.telem.sample_u32(tid::SATURATION_COUNT, 0);
    }

    fn tick_slow<F, T>(&mut self, _ctx: &mut TickCtx<'_, F, T>)
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        // Stage-0: thermal/safety supervisors later.
    }

    fn tick_system<F, T>(&mut self, _ctx: &mut TickCtx<'_, F, T>)
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        // Apply deferred resets (safe boundary).
        self.apply_pending_reset();

        // Apply deferred mode requests (policy hook).
        if let Some(m) = self.pending_mode.take() {
            // Stage-0 policy: only switch if disengaged.
            if self.st.engaged {
                // Re-queue if you want deferred switching when disengaged.
                self.pending_mode = Some(m);
            } else {
                self.apply_mode(m);
            }
        }
    }

    /// Apply pending reset if queued.
    fn apply_pending_reset(&mut self) {
        if let Some(scope) = self.pending.reset_req.take() {
            match scope {
                ResetScope::Control => {
                    // Reset controller/filter integrators.
                    self.st.pos_i = 0;
                    self.st.last_pos_err = 0;
                }
                ResetScope::AllState => {
                    // Reset all feature states (control + monitors + thermal model).
                    self.st.pos_i = 0;
                    self.st.last_pos_err = 0;
                    // Future: reset thermal model, monitors, etc.
                }
            }
        }
    }

    /// Return the gating reason (Ok means control may run).
    #[inline]
    fn compute_gate_reason(&self) -> GateReason {
        if !self.st.engaged {
            return GateReason::Disengaged;
        }
        if !self.st.frame.driver_ok {
            return GateReason::DriverNotOk;
        }
        // Stage-0: fault gating is not wired yet (depends on your FaultLatch integration).
        // If you have kernel-owned FaultLatch, check it here and return Faulted.
        GateReason::Ok
    }

    /// Simple PID position controller (Stage-0).
    ///
    /// Replace with your controller crate once you’re ready.
    fn position_pid(&mut self, dt: MicroSecond) -> Effort {
        self.st.last_dt = dt;

        let err = self.st.pos_sp.as_cdeg() - self.st.pos.as_cdeg();

        // Integrator (very naive; you’ll replace this).
        self.st.pos_i = self.st.pos_i.saturating_add(err);

        // Derivative.
        let derr = err - self.st.last_pos_err;
        self.st.last_pos_err = err;

        // Compute u in i32 domain.
        let u = (err as i32) * (self.cfg.pos_pid.kp as i32)
            + (self.st.pos_i as i32) * (self.cfg.pos_pid.ki as i32)
            + (derr as i32) * (self.cfg.pos_pid.kd as i32);

        // Convert to normalized effort (clamp).
        let mut raw = (u / 1024).clamp(i16::MIN as i32, i16::MAX as i32) as i16;

        // Apply effort limit clamp.
        let lim = self.cfg.effort_limit_raw.abs();
        if raw > lim {
            raw = lim;
        } else if raw < -lim {
            raw = -lim;
        }

        Effort::from_raw(raw)
    }

    fn apply_mode(&mut self, mode: OperatingMode) {
        self.st.mode = mode;
        // Clear controller state on mode change.
        self.st.pos_i = 0;
        self.st.last_pos_err = 0;
    }

    // =========================================================================
    // Host plane (sideband commands)
    // =========================================================================

    pub fn request_mode(&mut self, req: ModeRequest) -> Result<(), ModeError> {
        // Stage-0 policy: only allow switching when disengaged.
        if self.st.engaged {
            return Err(ModeError::Busy);
        }
        self.pending_mode = Some(req.mode);
        Ok(())
    }
}

// =============================================================================
// Kernel trait implementation
// =============================================================================

impl Kernel for ServoKernel {
    type Frame = SensorFrame;
    type Command = MotorCommand;

    #[inline]
    fn update_frame(&mut self, frame: Self::Frame) {
        self.st.update_frame(frame);
    }

    fn tick<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> Self::Command
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        ServoKernel::tick(self, ctx)
    }
}

// =============================================================================
// KernelHost implementation
// =============================================================================

impl KernelHost for ServoKernel {
    fn apply_op(&mut self, op: HostOp) -> HostResult {
        match op {
            HostOp::ModeRequest(req) => {
                self.request_mode(req)?;
                Ok(HostResp::Ack)
            }

            HostOp::FaultAck { id: _ } => Err(HostError::UnsupportedOp), // Stage-0
            HostOp::FaultAckAll => Err(HostError::UnsupportedOp),        // Stage-0
            HostOp::PersistCommit => Err(HostError::UnsupportedOp),      // Stage-0

            HostOp::SoftReset(scope) => {
                // Deferred: set pending flag, actual reset on System tick.
                self.pending.reset_req = Some(scope);
                Ok(HostResp::Ack)
            }

            HostOp::Ping => Ok(HostResp::Pong),
        }
    }
}

// =============================================================================
// ShadowKernel implementation
// =============================================================================

impl ShadowKernel for ServoKernel {
    fn publish_telemetry(&self, view: &mut KernelView<'_>) {
        // Position (i32 LE).
        let _ = view.write_telem(telem::POS_CDEG32, &self.st.pos.as_cdeg().to_le_bytes());

        // Effort (i16 LE).
        let _ = view.write_telem(telem::EFFORT_RAW, &self.st.last_cmd.effort.as_raw().to_le_bytes());

        // Engaged (u8).
        let _ = view.write_telem(telem::ENGAGED, &[if self.st.engaged { 1 } else { 0 }]);

        // Mode (u8).
        let _ = view.write_telem(telem::MODE, &[self.st.mode as u8]);

        // Fault mask (u32 LE). Stage-0: no faults tracked yet.
        let _ = view.write_telem(telem::FAULT_MASK, &0u32.to_le_bytes());

        // Gate reason (u8).
        let _ = view.write_telem(telem::GATE_REASON, &[self.st.last_gate as u8]);
    }

    fn commit_shadow(&mut self, view: &mut KernelView<'_>) -> CommitResult {
        if !view.ctrl_dirty() {
            return CommitResult::NothingToCommit;
        }

        // Check ENGAGED field.
        if view.is_range_dirty(ctrl::ENGAGED.offset, ctrl::ENGAGED.len as u16) {
            let mut buf = [0u8; 1];
            if view.read_ctrl(ctrl::ENGAGED.offset, &mut buf).is_ok() {
                let new_engaged = buf[0] != 0;
                // Disengage side effects: clear controller state.
                if self.st.engaged && !new_engaged {
                    self.st.pos_i = 0;
                    self.st.last_pos_err = 0;
                }
                self.st.engaged = new_engaged;
                view.clear_range_dirty(ctrl::ENGAGED.offset, ctrl::ENGAGED.len as u16);
            }
        }

        // Check MODE field.
        if view.is_range_dirty(ctrl::MODE.offset, ctrl::MODE.len as u16) {
            let mut buf = [0u8; 1];
            if view.read_ctrl(ctrl::MODE.offset, &mut buf).is_ok() {
                match buf[0] {
                    0 => {
                        self.pending_mode = Some(OperatingMode::Position);
                        view.clear_range_dirty(ctrl::MODE.offset, ctrl::MODE.len as u16);
                    }
                    1 => {
                        self.pending_mode = Some(OperatingMode::OpenLoop);
                        view.clear_range_dirty(ctrl::MODE.offset, ctrl::MODE.len as u16);
                    }
                    _ => {
                        // Invalid value: return error, dirty bit stays set.
                        return CommitResult::ValidationError {
                            offset: ctrl::MODE.offset,
                        };
                    }
                }
            }
        }

        // Check GOAL_POS field (i32 LE).
        if view.is_range_dirty(ctrl::GOAL_POS.offset, ctrl::GOAL_POS.len as u16) {
            let mut buf = [0u8; 4];
            if view.read_ctrl(ctrl::GOAL_POS.offset, &mut buf).is_ok() {
                let val = i32::from_le_bytes(buf);
                self.st.pos_sp = open_servo_units::CentiDeg32::from_cdeg(val);
                view.clear_range_dirty(ctrl::GOAL_POS.offset, ctrl::GOAL_POS.len as u16);
            }
        }

        // Check OPEN_LOOP_EFFORT field (i16 LE).
        if view.is_range_dirty(ctrl::OPEN_LOOP_EFFORT.offset, ctrl::OPEN_LOOP_EFFORT.len as u16) {
            let mut buf = [0u8; 2];
            if view.read_ctrl(ctrl::OPEN_LOOP_EFFORT.offset, &mut buf).is_ok() {
                let val = i16::from_le_bytes(buf);
                self.st.open_loop_effort = Effort::from_raw(val);
                view.clear_range_dirty(ctrl::OPEN_LOOP_EFFORT.offset, ctrl::OPEN_LOOP_EFFORT.len as u16);
            }
        }

        CommitResult::Ok
    }
}
