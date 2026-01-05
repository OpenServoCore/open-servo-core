//! Dynamixel RX parsing task.
//!
//! This embassy task reads from the UART RX DMA buffer, parses Dynamixel
//! Protocol 2.0 frames, and produces [`Op`] values for the request handler.
//!
//! ## Architecture
//!
//! ```text
//! UART RX DMA → dxl_rx_task → Channel<Op> → dxl_req_task
//! ```
//!
//! ## Frame Filtering
//!
//! The task only processes packets addressed to our servo ID (or broadcast).
//! Other packets are silently dropped.
//!
//! ## Error Handling
//!
//! - Checksum errors: packet dropped, error counter incremented
//! - Framing errors: re-sync to next header
//! - Buffer overflow: drop oldest bytes to make room

/// DXL RX task state.
///
/// This is a stub - the actual parser state machine will be added later.
pub struct DxlRxTask {
    /// Our servo ID for packet filtering.
    servo_id: u8,
}

impl DxlRxTask {
    /// Create a new RX task with the given servo ID.
    pub const fn new(servo_id: u8) -> Self {
        Self { servo_id }
    }

    /// Get the servo ID.
    pub const fn servo_id(&self) -> u8 {
        self.servo_id
    }
}

// TODO: Add embassy task function once RX DMA buffer access pattern is defined.
// The task will:
// 1. Read bytes from RX DMA ring buffer
// 2. Feed to Dynamixel frame parser
// 3. On valid frame for our ID: produce Op and send to channel
// 4. Loop
