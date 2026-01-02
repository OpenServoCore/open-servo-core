//! Kernel implementation.
//!
//! This implements two planes:
//! - Realtime plane: `update_frame()` and `tick()`
//! - Host plane: sideband commands (mode, fault ack, reset) via KernelOp
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
use open_servo_kernel_api::kernel::{Kernel, KernelHost};
use open_servo_kernel_api::mode::{ModeError, ModeRequest, OperatingMode};
use open_servo_kernel_api::ops::{FaultId, KernelOp, KernelResult};
use open_servo_kernel_api::reset::ResetScope;
use open_servo_kernel_api::shadow::{CommitResult, KernelView, ShadowKernel};
use open_servo_kernel_api::telemetry::ids as tid;
use open_servo_kernel_api::TelemetrySink;
use open_servo_kernel_api::{TickCtx, TickDomain};
use open_servo_units::{Effort, MicroSecond};

use crate::state::{KernelConfig, KernelState, PendingOps};
use open_servo_registry::vendor;

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
            st: KernelState::new(cfg.pos_pid, cfg.effort_limit_raw),
            cfg,
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
                    self.st.pos_pid.reset();
                }
                ResetScope::AllState => {
                    // Reset all feature states (control + monitors + thermal model).
                    self.st.pos_pid.reset();
                    // Future: reset thermal model, monitors, etc.
                }
            }
        }
    }

    /// Return the gating reason (Ok means control may run).
    ///
    /// Precedence is intentional: Disengaged → DriverNotOk → Faulted → Ok
    #[inline]
    fn compute_gate_reason(&self) -> GateReason {
        if !self.st.engaged {
            return GateReason::Disengaged;
        }
        if !self.st.frame.driver_ok {
            return GateReason::DriverNotOk;
        }
        if self.st.fault_mask != 0 {
            return GateReason::Faulted;
        }
        GateReason::Ok
    }

    /// Position PID controller using Q8.8 fixed-point gains.
    fn position_pid(&mut self, dt: MicroSecond) -> Effort {
        self.st.last_dt = dt;

        let sp = self.st.pos_sp.as_cdeg();
        let pv = self.st.pos.as_cdeg();

        // PidControllerI16 handles P/I/D, anti-windup, and output clamping
        let output = self.st.pos_pid.step(sp, pv);

        Effort::from_raw(output.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    fn apply_mode(&mut self, mode: OperatingMode) {
        self.st.mode = mode;
        // Clear controller state on mode change.
        self.st.pos_pid.reset();
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

/// Convert FaultId to bit position in fault_mask.
/// Returns None if id >= 32 (invalid).
#[inline]
fn fault_bit(id: FaultId) -> Option<u32> {
    let bit = id.0 as u32;
    if bit < 32 {
        Some(bit)
    } else {
        None
    }
}

impl KernelHost for ServoKernel {
    fn apply_op(&mut self, op: KernelOp) -> KernelResult {
        match op {
            KernelOp::ModeRequest(req) => match self.request_mode(req) {
                Ok(()) => KernelResult::Ok,
                Err(e) => match e {
                    ModeError::Unsupported => KernelResult::InvalidMode,
                    ModeError::Busy => KernelResult::Busy,
                    ModeError::Faulted => KernelResult::Faulted,
                },
            },

            KernelOp::FaultAck { id } => match fault_bit(id) {
                Some(bit) => {
                    self.st.fault_mask &= !(1u32 << bit); // idempotent clear
                    KernelResult::Ok
                }
                None => KernelResult::FaultAckRefused, // Invalid ID (>= 32)
            },

            KernelOp::FaultAckAll => {
                self.st.fault_mask = 0;
                KernelResult::Ok
            }

            KernelOp::CommitShadow => {
                // Shadow commit is automatic on medium tick; explicit request is a no-op.
                KernelResult::Ok
            }

            KernelOp::SoftReset(scope) => {
                // Deferred: set pending flag, actual reset on System tick.
                self.pending.reset_req = Some(scope);
                KernelResult::Ok
            }

            KernelOp::Ping => KernelResult::Ok,
        }
    }
}

// =============================================================================
// ShadowKernel implementation
// =============================================================================

impl ShadowKernel for ServoKernel {
    fn publish_telemetry(&self, view: &mut KernelView<'_>) {
        use open_servo_hw::v2::samples::{MotorCurrent, MotorVoltage, Sampled};

        // Position.
        let _ = vendor::reg::PRESENT_POS_CDEG.write(view, self.st.pos.as_cdeg());

        // Effort.
        let _ = vendor::reg::CONTROL_OUTPUT.write(view, self.st.last_cmd.effort.as_raw());

        // Engaged.
        let _ = vendor::reg::ENGAGED_MIRROR.write(view, self.st.engaged);

        // Mode.
        let _ = vendor::reg::MODE_MIRROR.write(view, self.st.mode as u8);

        // Fault mask.
        let _ = vendor::reg::FAULT_HISTORY.write(view, self.st.fault_mask);

        // Gate reason.
        let _ = vendor::reg::GATE_REASON.write(view, self.st.last_gate as u8);

        // Ambient temperature.
        let _ = vendor::reg::PRESENT_TEMP_CENTIC.write(view, self.st.frame.ambient_temp.as_centi_c());

        // MCU VDD.
        let _ = vendor::reg::PRESENT_VOLTAGE_MV.write(view, self.st.frame.mcu_vdd.as_mv());

        // Motor current.
        let current_ma = match self.st.frame.current {
            MotorCurrent::Bdc(ma) => ma.as_ma(),
            MotorCurrent::BldcPhases((a, _, _)) => a.as_ma(),
        };
        let _ = vendor::reg::PRESENT_CURRENT_MA.write(view, current_ma);

        // Motor temperature (optional).
        if let Sampled::Value(temp) = self.st.frame.motor_temp {
            let _ = vendor::reg::MOTOR_TEMP_CENTIC.write(view, temp.as_centi_c());
        }

        // Motor terminal voltages (optional).
        if let Sampled::Value(mv) = self.st.frame.motor_v {
            match mv {
                MotorVoltage::Bdc { a, b } => {
                    let _ = vendor::reg::MOTOR_VPLUS_MV.write(view, a.as_mv());
                    let _ = vendor::reg::MOTOR_VMINUS_MV.write(view, b.as_mv());
                }
                MotorVoltage::BldcPhases { a, b, .. } => {
                    let _ = vendor::reg::MOTOR_VPLUS_MV.write(view, a.as_mv());
                    let _ = vendor::reg::MOTOR_VMINUS_MV.write(view, b.as_mv());
                }
            }
        }
    }

    fn commit_shadow(&mut self, view: &mut KernelView<'_>) -> CommitResult {
        use open_servo_registry::reg::Reg;

        if !view.any_dirty() {
            return CommitResult::NothingToCommit;
        }

        // Type aliases for the control registers we're reading.
        type ModeReg = Reg<open_servo_registry::RW, u8>;
        type EngagedReg = Reg<open_servo_registry::RW, bool>;
        type GoalPosReg = Reg<open_servo_registry::RW, i32>;
        type EffortReg = Reg<open_servo_registry::RW, i16>;

        // Track which fields to clear dirty bits for after all processing.
        let mut clear_engaged = false;
        let mut clear_mode = false;
        let mut clear_goal_pos = false;
        let mut clear_effort = false;

        // Phase 1: Validate MODE (must happen before any clearing).
        let pending_mode = if view.is_range_dirty(
            vendor::reg::OPERATING_MODE.offset,
            ModeReg::len(),
        ) {
            if let Ok(mode_val) = vendor::reg::OPERATING_MODE.read(view) {
                match mode_val {
                    0 => {
                        clear_mode = true;
                        Some(OperatingMode::Position)
                    }
                    1 => {
                        clear_mode = true;
                        Some(OperatingMode::OpenLoop)
                    }
                    _ => {
                        return CommitResult::ValidationError {
                            offset: vendor::reg::OPERATING_MODE.offset,
                        };
                    }
                }
            } else {
                None
            }
        } else {
            None
        };

        // Phase 2: Read all dirty fields (before any clearing).
        let new_engaged = if view.is_range_dirty(
            vendor::reg::TORQUE_ENABLE.offset,
            EngagedReg::len(),
        ) {
            if let Ok(engaged) = vendor::reg::TORQUE_ENABLE.read(view) {
                clear_engaged = true;
                Some(engaged)
            } else {
                None
            }
        } else {
            None
        };

        let new_goal_pos = if view.is_range_dirty(
            vendor::reg::GOAL_POS_CDEG.offset,
            GoalPosReg::len(),
        ) {
            if let Ok(pos) = vendor::reg::GOAL_POS_CDEG.read(view) {
                clear_goal_pos = true;
                Some(pos)
            } else {
                None
            }
        } else {
            None
        };

        let new_effort = if view.is_range_dirty(
            vendor::reg::GOAL_PWM.offset,
            EffortReg::len(),
        ) {
            if let Ok(effort) = vendor::reg::GOAL_PWM.read(view) {
                clear_effort = true;
                Some(effort)
            } else {
                None
            }
        } else {
            None
        };

        // Phase 3: Apply all values (after reading, before clearing).
        if let Some(engaged) = new_engaged {
            if self.st.engaged && !engaged {
                self.st.pos_pid.reset();
            }
            self.st.engaged = engaged;
        }

        if let Some(mode) = pending_mode {
            self.pending_mode = Some(mode);
        }

        if let Some(pos) = new_goal_pos {
            self.st.pos_sp = open_servo_units::CentiDeg32::from_cdeg(pos);
        }

        if let Some(effort) = new_effort {
            self.st.open_loop_effort = Effort::from_raw(effort);
        }

        // Phase 4: Clear dirty bits for all processed fields.
        if clear_engaged {
            view.clear_range_dirty(vendor::reg::TORQUE_ENABLE.offset, EngagedReg::len());
        }
        if clear_mode {
            view.clear_range_dirty(vendor::reg::OPERATING_MODE.offset, ModeReg::len());
        }
        if clear_goal_pos {
            view.clear_range_dirty(vendor::reg::GOAL_POS_CDEG.offset, GoalPosReg::len());
        }
        if clear_effort {
            view.clear_range_dirty(vendor::reg::GOAL_PWM.offset, EffortReg::len());
        }

        CommitResult::Ok
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::KernelConfig;
    use open_servo_kernel_api::faults::GateReason;
    use open_servo_kernel_api::mode::OperatingMode;
    use open_servo_kernel_api::shadow::{CommitResult, HostView, KernelView, ShadowTable};
    use open_servo_registry::{ctrl, telem};
    use open_servo_units::{CentiDeg32, Effort};

    fn make_kernel() -> ServoKernel {
        ServoKernel::new(KernelConfig::default())
    }

    #[test]
    fn test_commit_invalid_mode_preserves_dirty() {
        let mut kernel = make_kernel();
        let mut table = ShadowTable::<1024>::new();
        // Host writes invalid MODE (0xFF)
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut host = HostView::new(bytes, dirty);
            host.write(ctrl::MODE.offset, &[0xFF]).unwrap();
        }
        // Kernel commits
        let (bytes, dirty) = table.as_mut_slices();
        let mut view = KernelView::new(bytes, dirty);
        let result = kernel.commit_shadow(&mut view);
        assert_eq!(
            result,
            CommitResult::ValidationError {
                offset: ctrl::MODE.offset
            }
        );
        // Dirty bit still set
        assert!(view.is_range_dirty(ctrl::MODE.offset, ctrl::MODE.len as u16));
    }

    #[test]
    fn test_commit_valid_clears_dirty_per_field() {
        let mut kernel = make_kernel();
        let mut table = ShadowTable::<1024>::new();
        // Host writes valid ENGAGED=1 and GOAL_POS=1000
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut host = HostView::new(bytes, dirty);
            host.write(ctrl::ENGAGED.offset, &[1]).unwrap();
            host.write(ctrl::GOAL_POS.offset, &1000i32.to_le_bytes())
                .unwrap();
        }
        // Kernel commits
        let (bytes, dirty) = table.as_mut_slices();
        let mut view = KernelView::new(bytes, dirty);
        let result = kernel.commit_shadow(&mut view);
        assert_eq!(result, CommitResult::Ok);
        assert!(kernel.st.engaged);
        assert_eq!(kernel.st.pos_sp.as_cdeg(), 1000);
        // Dirty bits cleared
        assert!(!view.is_range_dirty(ctrl::ENGAGED.offset, 1));
        assert!(!view.is_range_dirty(ctrl::GOAL_POS.offset, 4));
    }

    #[test]
    fn test_disengage_resets_controller() {
        let mut kernel = make_kernel();
        // Set engaged
        kernel.st.engaged = true;
        let mut table = ShadowTable::<1024>::new();
        // Host writes ENGAGED=0 (disengage)
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut host = HostView::new(bytes, dirty);
            host.write(ctrl::ENGAGED.offset, &[0]).unwrap();
        }
        let (bytes, dirty) = table.as_mut_slices();
        let mut view = KernelView::new(bytes, dirty);
        kernel.commit_shadow(&mut view);
        assert!(!kernel.st.engaged);
        // PID reset is tested in open-servo-math; here we just verify disengage succeeded
    }

    #[test]
    fn test_publish_telemetry_le_encoding() {
        let mut kernel = make_kernel();
        kernel.st.pos = CentiDeg32::from_cdeg(0x12345678);
        kernel.st.last_cmd.effort = Effort::from_raw(0x1234);
        kernel.st.engaged = true;
        kernel.st.mode = OperatingMode::OpenLoop;
        kernel.st.fault_mask = 0xDEADBEEF;
        kernel.st.last_gate = GateReason::Ok;
        let mut table = ShadowTable::<1024>::new();
        let (bytes, dirty) = table.as_mut_slices();
        let mut view = KernelView::new(bytes, dirty);
        kernel.publish_telemetry(&mut view);
        // Read back and verify LE encoding
        let mut buf4 = [0u8; 4];
        let mut buf2 = [0u8; 2];
        let mut buf1 = [0u8; 1];
        table.read(telem::POS_CDEG32, &mut buf4).unwrap();
        assert_eq!(buf4, 0x12345678i32.to_le_bytes());
        table.read(telem::EFFORT_RAW, &mut buf2).unwrap();
        assert_eq!(buf2, 0x1234i16.to_le_bytes());
        table.read(telem::ENGAGED, &mut buf1).unwrap();
        assert_eq!(buf1, [1]);
        table.read(telem::MODE, &mut buf1).unwrap();
        assert_eq!(buf1, [1]); // OpenLoop = 1
        table.read(telem::FAULT_MASK, &mut buf4).unwrap();
        assert_eq!(buf4, 0xDEADBEEFu32.to_le_bytes());
        table.read(telem::GATE_REASON, &mut buf1).unwrap();
        assert_eq!(buf1, [0]); // Ok = 0
    }
}
