//! Dynamixel request/response handling task.
//!
//! This task receives parsed DXL requests and:
//! - For register reads/writes: uses `ServiceOps` methods directly
//! - For kernel ops (mode change, etc.): enqueues `KernelOp`, waits for result
//! - For persist/telemetry: signals async tasks directly
//!
//! ## Single-Outstanding KernelOp Invariant
//!
//! This task is the **sole producer** of [`KernelOp`] values. It waits for
//! the corresponding [`KernelResult`] before submitting the next op. This
//! eliminates the need for correlation IDs.
//!
//! ## Response Flow
//!
//! After processing a request, the task builds a Dynamixel response frame and
//! sends it via the UART (using `embedded-io-async::Write`).

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
// 1. Receive parsed DXL request from dxl_rx channel
// 2. Handle request:
//    - Read/Write: call ServiceOps.read_range() / write_range()
//    - Mode change: enqueue KernelOp, await result signal
//    - Persist: signal persist task
// 3. Build and send response via UART
// 4. Loop
