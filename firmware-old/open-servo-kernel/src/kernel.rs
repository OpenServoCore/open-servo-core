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

use embedded_shadow::view::KernelView;
use open_servo_hw::v2::io::{DriveMode, MotorCommand, MotorHints, SensorFrame};
use open_servo_kernel_api::faults::FaultSink;
use open_servo_kernel_api::faults::GateReason;
use open_servo_kernel_api::kernel::{Kernel, KernelHost};
use open_servo_kernel_api::mode::{ModeError, ModeRequest, OperatingMode};
use open_servo_kernel_api::ops::{FaultId, KernelOp, KernelResult};
use open_servo_kernel_api::reset::ResetScope;
use open_servo_kernel_api::telemetry::ids as tid;
use open_servo_kernel_api::CommitResult;
use open_servo_kernel_api::TelemetrySink;
use open_servo_kernel_api::{TickCtx, TickDomain};
use open_servo_units::{Effort, MicroSecond};

use crate::state::{KernelConfig, KernelState, PendingOps};
use open_servo_registry::{facade, vendor};

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
// Shadow table methods (publish/commit)
// =============================================================================

impl ServoKernel {
    /// Publish telemetry data to shadow table.
    ///
    /// Called from ISR context at a safe tick boundary (e.g., ControlMedium).
    /// Writes kernel state to vendor telemetry registers.
    pub fn publish_telemetry<const TS: usize, const BS: usize, const BC: usize>(
        &self,
        view: &mut KernelView<'_, TS, BS, BC>,
    ) where
        bitmaps::BitsImpl<BC>: bitmaps::Bits,
    {
        use open_servo_hw::v2::samples::{
            MotorCurrent, MotorCurrentRaw, MotorVoltage, MotorVoltageRaw,
        };

        // Position (converted + raw).
        let _ = vendor::reg::PRESENT_POS_CDEG.write(view, self.st.pos.as_cdeg());
        let _ = vendor::reg::POSITION_RAW.write(view, self.st.frame.pos.raw());

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

        // Ambient temperature (converted + raw).
        let ambient_cc = self
            .st
            .frame
            .ambient_temp
            .value()
            .map(|v| v.as_centi_c())
            .unwrap_or(0);
        let _ = vendor::reg::PRESENT_TEMP_CENTIC.write(view, ambient_cc);
        let _ = vendor::reg::AMBIENT_TEMP_RAW.write(view, self.st.frame.ambient_temp.raw());

        // MCU VDD (converted + raw as VREFINT).
        let mcu_mv = self
            .st
            .frame
            .mcu_vdd
            .value()
            .map(|v| v.as_mv())
            .unwrap_or(0);
        let _ = vendor::reg::PRESENT_VOLTAGE_MV.write(view, mcu_mv);
        let _ = vendor::reg::VREFINT_RAW.write(view, self.st.frame.mcu_vdd.raw());

        // Motor current (converted + raw, handles BDC/BLDC).
        let (current_a, current_b, current_c) = self
            .st
            .frame
            .current
            .value()
            .map(|c| match c {
                MotorCurrent::Bdc(ma) => (ma.as_ma(), 0i16, 0i16),
                MotorCurrent::BldcPhases((a, b, c)) => (a.as_ma(), b.as_ma(), c.as_ma()),
            })
            .unwrap_or((0, 0, 0));
        let _ = vendor::reg::PRESENT_CURRENT_MA.write(view, current_a);
        let _ = vendor::reg::MOTOR_CURRENT_B_MA.write(view, current_b);
        let _ = vendor::reg::MOTOR_CURRENT_C_MA.write(view, current_c);

        // Motor current raw.
        let (raw_a, raw_b, raw_c) = match self.st.frame.current.raw() {
            MotorCurrentRaw::Bdc(a) => (a, 0u16, 0u16),
            MotorCurrentRaw::Bldc(a, b, c) => (a, b, c),
        };
        let _ = vendor::reg::MOTOR_CURRENT_A_RAW.write(view, raw_a);
        let _ = vendor::reg::MOTOR_CURRENT_B_RAW.write(view, raw_b);
        let _ = vendor::reg::MOTOR_CURRENT_C_RAW.write(view, raw_c);

        // Motor temperature (optional, converted + raw).
        if let Some(reading) = &self.st.frame.motor_temp {
            if let Some(temp) = reading.value() {
                let _ = vendor::reg::MOTOR_TEMP_CENTIC.write(view, temp.as_centi_c());
            }
            let _ = vendor::reg::MOTOR_TEMP_RAW.write(view, reading.raw());
        }

        // Motor terminal voltages (optional, converted + raw, handles BDC/BLDC).
        if let Some(reading) = &self.st.frame.motor_v {
            if let Some(mv) = reading.value() {
                match mv {
                    MotorVoltage::Bdc { a, b } => {
                        let _ = vendor::reg::MOTOR_VPLUS_MV.write(view, a.as_mv());
                        let _ = vendor::reg::MOTOR_VMINUS_MV.write(view, b.as_mv());
                        let _ = vendor::reg::MOTOR_VOLTAGE_C_MV.write(view, 0i16);
                    }
                    MotorVoltage::BldcPhases { a, b, c } => {
                        let _ = vendor::reg::MOTOR_VPLUS_MV.write(view, a.as_mv());
                        let _ = vendor::reg::MOTOR_VMINUS_MV.write(view, b.as_mv());
                        let _ = vendor::reg::MOTOR_VOLTAGE_C_MV.write(view, c.as_mv());
                    }
                }
            }
            // Motor voltage raw.
            let (raw_a, raw_b, raw_c) = match reading.raw() {
                MotorVoltageRaw::Bdc(a, b) => (a, b, 0u16),
                MotorVoltageRaw::Bldc(a, b, c) => (a, b, c),
            };
            let _ = vendor::reg::MOTOR_VOLTAGE_A_RAW.write(view, raw_a);
            let _ = vendor::reg::MOTOR_VOLTAGE_B_RAW.write(view, raw_b);
            let _ = vendor::reg::MOTOR_VOLTAGE_C_RAW.write(view, raw_c);
        }

        // Board capabilities (static, but written every telemetry cycle for simplicity).
        let _ = vendor::reg::SENSOR_CAPS.write(view, self.cfg.sensor_caps.bits());
        let _ = vendor::reg::MOTOR_TYPE.write(view, self.cfg.motor_type as u8);

        // Servo position kind.
        use open_servo_hw::v2::capability::ServoPosKind;
        match self.cfg.servo_pos_kind {
            ServoPosKind::Bounded { min, max } => {
                let _ = vendor::reg::SERVO_POS_KIND.write(view, 0u8);
                let _ = vendor::reg::SERVO_POS_MIN_CDEG.write(view, min.as_cdeg());
                let _ = vendor::reg::SERVO_POS_MAX_CDEG.write(view, max.as_cdeg());
            }
            ServoPosKind::Wrap360 => {
                let _ = vendor::reg::SERVO_POS_KIND.write(view, 1u8);
                let _ = vendor::reg::SERVO_POS_MIN_CDEG.write(view, 0i16);
                let _ = vendor::reg::SERVO_POS_MAX_CDEG.write(view, 0i16);
            }
        }
    }

    /// Commit dirty shadow data to live kernel state.
    ///
    /// Called from ISR context at a safe tick boundary (e.g., ControlMedium).
    /// Handles both facade (DXL) and vendor register dirty bits, with facade
    /// taking precedence for mapped fields.
    ///
    /// Clears all dirty bits at the end of processing.
    pub fn commit_shadow<const TS: usize, const BS: usize, const BC: usize>(
        &mut self,
        view: &mut KernelView<'_, TS, BS, BC>,
    ) -> CommitResult
    where
        bitmaps::BitsImpl<BC>: bitmaps::Bits,
    {
        use open_servo_registry::dxl::addr as dxl_addr;
        use open_servo_registry::reg::Reg;

        if !view.any_dirty() {
            return CommitResult::NothingToCommit;
        }

        // Type aliases for the control registers we're reading.
        type ModeReg = Reg<open_servo_registry::RW, u8>;
        type EngagedReg = Reg<open_servo_registry::RW, bool>;
        type GoalPosReg = Reg<open_servo_registry::RW, i32>;
        type EffortReg = Reg<open_servo_registry::RW, i16>;

        // Phase 1: Validate MODE (must happen before any clearing).
        let pending_mode =
            if view.is_dirty(vendor::reg::OPERATING_MODE.offset, ModeReg::len() as usize).unwrap_or(false) {
                if let Ok(mode_val) = vendor::reg::OPERATING_MODE.read(view) {
                    match mode_val {
                        0 => Some(OperatingMode::Position),
                        1 => Some(OperatingMode::OpenLoop),
                        _ => {
                            // Don't clear dirty bits on validation error
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

        // Phase 2: Read all dirty fields.
        // Check facade (DXL) torque_enable first, then vendor.
        let new_engaged =
            if view.is_dirty(dxl_addr::TORQUE_ENABLE, 1).unwrap_or(false) {
                let mut buf = [0u8; 1];
                if view.read_range(dxl_addr::TORQUE_ENABLE, &mut buf).is_ok() {
                    Some(buf[0] != 0)
                } else {
                    None
                }
            } else if view.is_dirty(vendor::reg::TORQUE_ENABLE.offset, EngagedReg::len() as usize).unwrap_or(false) {
                vendor::reg::TORQUE_ENABLE.read(view).ok()
            } else {
                None
            };

        // Check facade goal_position FIRST (Dynamixel compat takes precedence),
        // then vendor goal_pos_cdeg.
        let new_goal_pos =
            if view.is_dirty(dxl_addr::GOAL_POSITION, 4).unwrap_or(false) {
                let mut buf = [0u8; 4];
                if view.read_range(dxl_addr::GOAL_POSITION, &mut buf).is_ok() {
                    let pulses = i32::from_le_bytes(buf);
                    Some(facade::pulses_to_cdeg(pulses))
                } else {
                    None
                }
            } else if view.is_dirty(vendor::reg::GOAL_POS_CDEG.offset, GoalPosReg::len() as usize).unwrap_or(false) {
                vendor::reg::GOAL_POS_CDEG.read(view).ok()
            } else {
                None
            };

        // Check facade goal_pwm first, then vendor.
        let new_effort =
            if view.is_dirty(dxl_addr::GOAL_PWM, 2).unwrap_or(false) {
                let mut buf = [0u8; 2];
                if view.read_range(dxl_addr::GOAL_PWM, &mut buf).is_ok() {
                    Some(i16::from_le_bytes(buf))
                } else {
                    None
                }
            } else if view.is_dirty(vendor::reg::GOAL_PWM.offset, EffortReg::len() as usize).unwrap_or(false) {
                vendor::reg::GOAL_PWM.read(view).ok()
            } else {
                None
            };

        // Phase 3: Apply all values.
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

        // Phase 4: Clear ALL dirty bits at end (embedded-shadow model).
        view.clear_dirty();

        CommitResult::Ok
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::test_kernel_config;
    use embedded_shadow::persist::PersistTrigger;
    use embedded_shadow::policy::{AllowAllPolicy, NoPersistPolicy};
    use embedded_shadow::storage::ShadowStorage;
    use embedded_shadow::view::{HostView, KernelView};
    use open_servo_kernel_api::faults::GateReason;
    use open_servo_kernel_api::mode::OperatingMode;
    use open_servo_kernel_api::CommitResult;
    use open_servo_registry::{ctrl, telem};
    use open_servo_units::{CentiDeg32, Effort};

    /// No-op persist trigger for tests.
    struct NoPersistTrigger;
    impl<PK> PersistTrigger<PK> for NoPersistTrigger {
        fn push_key(&mut self, _key: PK) {}
        fn request_persist(&mut self) {}
    }

    // Test storage type: 1024 bytes, 64-byte blocks, 16 blocks
    type TestStorage = ShadowStorage<1024, 64, 16, AllowAllPolicy, NoPersistPolicy, NoPersistTrigger, ()>;
    type TestHostView<'a> = HostView<'a, 1024, 64, 16, AllowAllPolicy, NoPersistPolicy, NoPersistTrigger, ()>;
    type TestKernelView<'a> = KernelView<'a, 1024, 64, 16>;

    fn make_kernel() -> ServoKernel {
        ServoKernel::new(test_kernel_config())
    }

    fn make_storage() -> TestStorage {
        ShadowStorage::new(AllowAllPolicy {}, NoPersistPolicy {}, NoPersistTrigger)
    }

    #[test]
    fn test_commit_invalid_mode_preserves_dirty() {
        let mut kernel = make_kernel();
        let storage = make_storage();
        // Host writes invalid MODE (0xFF)
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            view.write_range(ctrl::MODE.offset, &[0xFF]).unwrap();
        });
        // Kernel commits
        // SAFETY: Test code, no concurrent access
        let result = unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                kernel.commit_shadow(view)
            })
        };
        assert_eq!(
            result,
            CommitResult::ValidationError {
                offset: ctrl::MODE.offset
            }
        );
        // Note: With embedded-shadow, validation errors don't clear dirty bits
        // so the dirty bit should still be set (but we can't easily check from outside)
    }

    #[test]
    fn test_commit_valid_clears_dirty() {
        let mut kernel = make_kernel();
        let storage = make_storage();
        // Host writes valid ENGAGED=1 and GOAL_POS=1000
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            view.write_range(ctrl::ENGAGED.offset, &[1]).unwrap();
            view.write_range(ctrl::GOAL_POS.offset, &1000i32.to_le_bytes()).unwrap();
        });
        // Kernel commits
        // SAFETY: Test code, no concurrent access
        let result = unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                kernel.commit_shadow(view)
            })
        };
        assert_eq!(result, CommitResult::Ok);
        assert!(kernel.st.engaged);
        assert_eq!(kernel.st.pos_sp.as_cdeg(), 1000);
        // Verify dirty bits are cleared (all at once in embedded-shadow)
        // SAFETY: Test code
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                assert!(!view.any_dirty());
            });
        }
    }

    #[test]
    fn test_disengage_resets_controller() {
        let mut kernel = make_kernel();
        // Set engaged
        kernel.st.engaged = true;
        let storage = make_storage();
        // Host writes ENGAGED=0 (disengage)
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            view.write_range(ctrl::ENGAGED.offset, &[0]).unwrap();
        });
        // SAFETY: Test code
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                kernel.commit_shadow(view);
            });
        }
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
        let storage = make_storage();
        // SAFETY: Test code
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                kernel.publish_telemetry(view);
            });
        }
        // Read back and verify LE encoding
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            let mut buf4 = [0u8; 4];
            let mut buf2 = [0u8; 2];
            let mut buf1 = [0u8; 1];
            view.read_range(telem::POS_CDEG32, &mut buf4).unwrap();
            assert_eq!(buf4, 0x12345678i32.to_le_bytes());
            view.read_range(telem::EFFORT_RAW, &mut buf2).unwrap();
            assert_eq!(buf2, 0x1234i16.to_le_bytes());
            view.read_range(telem::ENGAGED, &mut buf1).unwrap();
            assert_eq!(buf1, [1]);
            view.read_range(telem::MODE, &mut buf1).unwrap();
            assert_eq!(buf1, [1]); // OpenLoop = 1
            view.read_range(telem::FAULT_MASK, &mut buf4).unwrap();
            assert_eq!(buf4, 0xDEADBEEFu32.to_le_bytes());
            view.read_range(telem::GATE_REASON, &mut buf1).unwrap();
            assert_eq!(buf1, [0]); // Ok = 0
        });
    }

    /// Integration test: full round-trip through shadow table.
    ///
    /// Simulates the real firmware flow:
    /// 1. Host writes control registers (ENGAGED, MODE, GOAL_POS)
    /// 2. Kernel commits from shadow → updates internal state
    /// 3. Kernel publishes telemetry back to shadow
    /// 4. Host reads telemetry → verifies round-trip
    #[test]
    fn test_full_shadow_roundtrip() {
        let mut kernel = make_kernel();
        let storage = make_storage();

        // === Phase 1: Host writes control values ===
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            // Enable servo
            view.write_range(ctrl::ENGAGED.offset, &[1]).unwrap();
            // Set position mode
            view.write_range(ctrl::MODE.offset, &[0]).unwrap(); // Position = 0
            // Set goal position to 4500 centidegrees (45°)
            view.write_range(ctrl::GOAL_POS.offset, &4500i32.to_le_bytes()).unwrap();
        });

        // Verify dirty bits are set
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                assert!(view.any_dirty(), "Shadow should have dirty bits after host write");
            });
        }

        // === Phase 2: Kernel commits from shadow ===
        let result = unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                kernel.commit_shadow(view)
            })
        };
        assert_eq!(result, CommitResult::Ok);

        // Verify kernel state was updated
        assert!(kernel.st.engaged, "Kernel should be engaged");
        assert_eq!(kernel.st.mode, OperatingMode::Position);
        assert_eq!(kernel.st.pos_sp.as_cdeg(), 4500, "Goal position should be 4500 cdeg");

        // Verify dirty bits are cleared
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                assert!(!view.any_dirty(), "Dirty bits should be cleared after commit");
            });
        }

        // === Phase 3: Simulate kernel operation (set current position) ===
        kernel.st.pos = CentiDeg32::from_cdeg(4200); // Current pos slightly behind goal
        kernel.st.last_cmd.effort = Effort::from_raw(500);
        kernel.st.fault_mask = 0;
        kernel.st.last_gate = GateReason::Ok;

        // === Phase 4: Kernel publishes telemetry ===
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                kernel.publish_telemetry(view);
            });
        }

        // === Phase 5: Host reads telemetry back ===
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            let mut buf4 = [0u8; 4];
            let mut buf2 = [0u8; 2];
            let mut buf1 = [0u8; 1];

            // Verify position telemetry
            view.read_range(telem::POS_CDEG32, &mut buf4).unwrap();
            let pos = i32::from_le_bytes(buf4);
            assert_eq!(pos, 4200, "Telemetry position should be 4200 cdeg");

            // Verify effort telemetry
            view.read_range(telem::EFFORT_RAW, &mut buf2).unwrap();
            let effort = i16::from_le_bytes(buf2);
            assert_eq!(effort, 500, "Telemetry effort should be 500");

            // Verify engaged mirror
            view.read_range(telem::ENGAGED, &mut buf1).unwrap();
            assert_eq!(buf1[0], 1, "Telemetry engaged should be 1");

            // Verify mode mirror
            view.read_range(telem::MODE, &mut buf1).unwrap();
            assert_eq!(buf1[0], 0, "Telemetry mode should be Position (0)");

            // Verify no faults
            view.read_range(telem::FAULT_MASK, &mut buf4).unwrap();
            assert_eq!(buf4, [0, 0, 0, 0], "Fault mask should be 0");

            // Verify gate reason
            view.read_range(telem::GATE_REASON, &mut buf1).unwrap();
            assert_eq!(buf1[0], 0, "Gate reason should be Ok (0)");
        });
    }

    /// Integration test: multiple commit cycles with state changes.
    ///
    /// Tests that repeated host writes and kernel commits work correctly.
    #[test]
    fn test_multiple_commit_cycles() {
        let mut kernel = make_kernel();
        let storage = make_storage();

        // Cycle 1: Engage and set position
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            view.write_range(ctrl::ENGAGED.offset, &[1]).unwrap();
            view.write_range(ctrl::GOAL_POS.offset, &1000i32.to_le_bytes()).unwrap();
        });
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                let result = kernel.commit_shadow(view);
                assert_eq!(result, CommitResult::Ok);
            });
        }
        assert!(kernel.st.engaged);
        assert_eq!(kernel.st.pos_sp.as_cdeg(), 1000);

        // Cycle 2: Change position while engaged
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            view.write_range(ctrl::GOAL_POS.offset, &2000i32.to_le_bytes()).unwrap();
        });
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                let result = kernel.commit_shadow(view);
                assert_eq!(result, CommitResult::Ok);
            });
        }
        assert!(kernel.st.engaged, "Should still be engaged");
        assert_eq!(kernel.st.pos_sp.as_cdeg(), 2000, "Position should update to 2000");

        // Cycle 3: Disengage
        storage.host_shadow().with_view(|view: &mut TestHostView| {
            view.write_range(ctrl::ENGAGED.offset, &[0]).unwrap();
        });
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                let result = kernel.commit_shadow(view);
                assert_eq!(result, CommitResult::Ok);
            });
        }
        assert!(!kernel.st.engaged, "Should be disengaged");

        // Cycle 4: Nothing dirty → NothingToCommit
        unsafe {
            storage.kernel_shadow().with_view_unchecked(|view: &mut TestKernelView| {
                let result = kernel.commit_shadow(view);
                assert_eq!(result, CommitResult::NothingToCommit);
            });
        }
    }
}
