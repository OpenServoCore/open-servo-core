//! Dynamixel request/response handling task.
//!
//! This embassy task receives [`Op`] values from the RX task and:
//! - For [`KernelOp`]: submits to RT queue, waits for [`KernelResult`]
//! - For [`ServiceOp`]: handles locally (persist, telemetry config)
//! - For [`ShadowRead`]: reads from shadow table directly
//!
//! ## Single-Outstanding KernelOp Invariant
//!
//! This task is the **sole producer** of [`KernelOp`] values. It waits for
//! the corresponding [`KernelResult`] before submitting the next op. This
//! eliminates the need for correlation IDs.
//!
//! ## Response Flow
//!
//! After processing an op, the task builds a Dynamixel response frame and
//! sends it via the UART (using `embedded-io-async::Write`).

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use open_servo_kernel_api::ops::{KernelResult, Op};

/// Signal type for KernelResult notification (ISR → async).
pub type KernelResultSignal = Signal<CriticalSectionRawMutex, KernelResult>;

/// DXL request handler task state.
///
/// This is a stub - the actual request handling logic will be added later.
pub struct DxlReqTask {
    /// Pending op waiting for kernel result (if any).
    pending_kernel_op: bool,
}

impl DxlReqTask {
    /// Create a new request handler task.
    pub const fn new() -> Self {
        Self {
            pending_kernel_op: false,
        }
    }

    /// Check if waiting for a kernel result.
    pub const fn has_pending_op(&self) -> bool {
        self.pending_kernel_op
    }

    /// Handle an Op from the RX task.
    ///
    /// Returns the response to send (stub - returns None for now).
    pub fn handle_op(&mut self, _op: Op) -> Option<Response> {
        // Stub implementation
        None
    }
}

impl Default for DxlReqTask {
    fn default() -> Self {
        Self::new()
    }
}

/// Response to send back to the host.
///
/// This is a stub - the actual response types will be defined later.
pub struct Response {
    /// Dynamixel packet ID (mirrors request).
    pub id: u8,
    /// Instruction (status packet = 0x55).
    pub instruction: u8,
    /// Error field.
    pub error: u8,
    /// Parameter data (variable length).
    pub params: heapless::Vec<u8, 256>,
}

// TODO: Add embassy task function.
// The task will:
// 1. Receive Op from dxl_rx channel
// 2. Match on Op variant:
//    - Kernel(op): enqueue to RT, await signal, build response
//    - Service(op): handle locally, build response
//    - ShadowRead: read from shadow table, build response
// 3. Send response via UART (embedded-io-async::Write)
// 4. Loop
