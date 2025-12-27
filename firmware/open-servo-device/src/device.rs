//! Device runner: ties together board + kernel + protocol services.
//!
//! Ownership:
//! - Device owns the concrete `board`, `kernel`, `comms` service, and sinks.
//!
//! Scheduling model:
//! - Real-time control ticks are called by the board/firmware scheduler (ISR).
//! - Comms polling runs in main loop or a low-rate system tick.
//!
//! Echo handling:
//! - If the CommsService requests `DisableRxDuringTx`, we disable RX while sending.
//! - Otherwise, RX stays enabled and the service must filter echoed bytes.

use open_servo_hw::v2::io::{MotorCommand, SensorFrame};
use open_servo_hw::v2::Board as HwBoard;

use open_servo_kernel_api::{
    kernel::{Kernel, KernelHost},
    tick::TickDomain,
    tick_ctx::TickCtx,
    ticks::KernelCtx,
    timebase::TimeStampUs,
    FaultSink, TelemetrySink,
};

use open_servo_units::MicroSecond;

use crate::comms_service::{CommsService, EchoPolicy, HostOp, HostResult};
use crate::uart_bus::UartBus;

/// Device glue type.
///
/// `B` must implement both:
/// - `open_servo_hw::Board` (sensor/motor boundary)
/// - `UartBus` (byte transport boundary for comms)
///
/// `K` is the kernel. We pin its IO shapes to the canonical hw types:
/// - `Frame = SensorFrame`
/// - `Command = MotorCommand`
///
/// `C` is the communications service (e.g. Dynamixel, CAN adapter).
///
/// This removes ambiguity and keeps the "brain" and "body" aligned.
pub struct Device<B, K, C, F, T> {
    pub board: B,
    pub kernel: K,
    pub comms: C,
    pub faults: F,
    pub telem: T,
    pub kctx: KernelCtx,
}

impl<B, K, C, F, T> Device<B, K, C, F, T>
where
    B: HwBoard + UartBus,
    K: Kernel<Frame = SensorFrame, Command = MotorCommand> + KernelHost,
    C: CommsService,
    F: FaultSink,
    T: TelemetrySink,
{
    /// Construct a new device runner.
    #[inline]
    pub fn new(board: B, kernel: K, comms: C, faults: F, telem: T) -> Self {
        Self {
            board,
            kernel,
            comms,
            faults,
            telem,
            kctx: KernelCtx::new(),
        }
    }

    // =========================================================================
    // Real-time control path
    // =========================================================================

    /// Run a single scheduled kernel tick for the given domain.
    ///
    /// The board/firmware decides scheduling (timers/divisors/ISRs).
    /// This method just executes the boundary calls in the correct order:
    ///
    /// 1) `board.read_sensors()` -> `kernel.update_frame(frame)`
    /// 2) build `Tick` using `KernelCtx` for `domain`
    /// 3) `kernel.tick(ctx)` -> `MotorCommand`
    /// 4) for `ControlFast`: `board.write_motor(cmd)`
    ///
    /// Returning `cmd` is useful for debug and testing, even if you only
    /// apply it on fast ticks.
    #[inline]
    pub fn run_tick(
        &mut self,
        domain: TickDomain,
        now: TimeStampUs,
        dt: MicroSecond,
    ) -> MotorCommand {
        // Update kernel with latest sensor frame (no time advance).
        let frame = self.board.read_sensors();
        self.kernel.update_frame(frame);

        // Produce a Tick for the requested domain (sequence counters tracked here).
        let tick = match domain {
            TickDomain::ControlFast => self.kctx.next_control_fast(dt),
            TickDomain::ControlMedium => self.kctx.next_control_medium(dt),
            TickDomain::ControlSlow => self.kctx.next_control_slow(dt),
            TickDomain::System => self.kctx.next_system(dt),
        };

        // Build TickCtx: faults/telemetry sinks are injected from the device.
        let mut ctx = TickCtx::new(tick, now, &mut self.faults, &mut self.telem);

        // Advance kernel.
        let cmd = self.kernel.tick(&mut ctx);

        // Apply actuator output on fast ticks by convention.
        if domain == TickDomain::ControlFast {
            self.board.write_motor(cmd);
        }

        cmd
    }

    // =========================================================================
    // Comms path (main loop / system tick)
    // =========================================================================

    /// Poll comms service and execute produced host operations against the kernel.
    ///
    /// Recommended call site:
    /// - main loop, or
    /// - low-rate System tick
    ///
    /// This function is intentionally cooperative: it does bounded work per call
    /// (bounded by available RX bytes / TX FIFO capacity).
    pub fn poll_comms(&mut self) {
        let echo_policy = self.comms.echo_policy();

        // 1) Drain RX into service.
        while let Some(b) = self.board.rx_pop() {
            self.comms.ingest_rx_byte(b);
        }

        // 2) Execute all requested ops.
        while let Some(op) = self.comms.next_op() {
            let result = self.exec_op(op);
            self.comms.push_result(result);
        }

        // 3) Start TX if pending and bus idle.
        if self.comms.tx_pending() && !self.board.tx_busy() {
            // Switch to transmit mode (half duplex).
            self.board.set_tx_enable(true);

            // Simplest echo handling: disable RX while TX is active (if supported).
            if echo_policy == EchoPolicy::DisableRxDuringTx {
                self.board.set_rx_enable(false);
            }

            // Fill TX FIFO/queue with as many bytes as possible.
            while let Some(b) = self.comms.tx_pop() {
                if !self.board.tx_push(b) {
                    break; // FIFO full; continue next poll
                }
            }

            self.board.tx_kick();
        }

        // 4) TX completion: release line and re-enable RX.
        //
        // Note: many HALs require checking TC (transmission complete) rather than TXE.
        if self.board.tx_busy() && self.board.tx_complete() {
            self.board.set_tx_enable(false);

            if echo_policy == EchoPolicy::DisableRxDuringTx {
                self.board.set_rx_enable(true);
            }

            self.comms.notify_tx_complete();
        }
    }

    /// Execute a host operation against the kernel.
    ///
    /// This delegates to [`KernelHost::apply_op`] and returns the result.
    /// The comms service is responsible for mapping the result to
    /// protocol-specific response packets.
    #[inline]
    fn exec_op(&mut self, op: HostOp) -> HostResult {
        self.kernel.apply_op(op)
    }
}
