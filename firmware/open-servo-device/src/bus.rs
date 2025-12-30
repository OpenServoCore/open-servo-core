//! RT bus engine abstraction for ISR-owned half-duplex bus arbitration.
//!
//! The [`BusEngine`] trait defines the RT-side interface for bus timing and TX.
//! It is owned by a timer ISR (e.g., TX_DUE timer) that handles:
//!
//! - Bus arbitration (half-duplex turn-around timing)
//! - TX frame submission with copy-in semantics
//! - DMA start/stop coordination
//!
//! ## Architecture
//!
//! ```text
//!                    ┌─────────────────────────────────┐
//!   async tasks      │  dxl_req_task (embassy)         │
//!   (soft-time)      │  - parses packets               │
//!                    │  - produces TxFrame             │
//!                    │  - waits for TxComplete signal  │
//!                    └─────────────┬───────────────────┘
//!                                  │ submit_tx()
//!                                  ▼
//!                    ┌─────────────────────────────────┐
//!   RT ISR           │  BusEngine (timer ISR owned)    │
//!   (hard-time)      │  - copies TxFrame to DMA buffer │
//!                    │  - manages turn-around timing   │
//!                    │  - starts/stops DMA             │
//!                    └─────────────────────────────────┘
//! ```
//!
//! ## TxFrame copy-in semantics
//!
//! The [`TxFrame`] uses a fixed 256-byte buffer with length field.
//! `submit_tx()` copies the frame data into the ISR's DMA buffer, allowing
//! the caller's stack to be reclaimed immediately. This is critical because
//! DMA operations may outlive the borrow.

/// Maximum frame size for TX operations.
///
/// 256 bytes covers Dynamixel Protocol 2.0 max packet size (255 + header).
pub const MAX_TX_FRAME: usize = 256;

/// TX frame with copy-in semantics.
///
/// This is a value type that can be copied into the BusEngine's DMA buffer.
/// The fixed size avoids const-generic complexity at trait boundaries.
#[derive(Clone)]
pub struct TxFrame {
    buf: [u8; MAX_TX_FRAME],
    len: u16,
}

impl TxFrame {
    /// Create a new TxFrame from a slice.
    ///
    /// Truncates to MAX_TX_FRAME if necessary.
    #[inline]
    pub fn new(data: &[u8]) -> Self {
        let len = data.len().min(MAX_TX_FRAME) as u16;
        let mut buf = [0u8; MAX_TX_FRAME];
        buf[..len as usize].copy_from_slice(&data[..len as usize]);
        Self { buf, len }
    }

    /// Create an empty TxFrame.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            buf: [0u8; MAX_TX_FRAME],
            len: 0,
        }
    }

    /// Get the frame data as a slice.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    /// Get the frame length.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Check if the frame is empty.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for TxFrame {
    #[inline]
    fn default() -> Self {
        Self::empty()
    }
}

/// TX submission result.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TxSubmitResult {
    /// Frame accepted; will be transmitted.
    Accepted,
    /// Bus is busy (previous TX in progress or turn-around delay active).
    Busy,
}

/// RT bus engine trait (ISR-owned).
///
/// Implementations of this trait are owned by the TX timer ISR and handle:
/// - Bus arbitration (turn-around timing after RX)
/// - TX frame buffering (copy-in from TxFrame)
/// - DMA coordination
///
/// ## Thread Safety
///
/// Methods on this trait are called from ISR context. The `submit_tx` method
/// may be called from the async context (via critical section or signal),
/// but the actual DMA start is deferred to the timer ISR.
pub trait BusEngine {
    /// Submit a TX frame for transmission.
    ///
    /// The frame data is copied into the engine's internal buffer.
    /// Returns `Busy` if a previous transmission is still in progress.
    ///
    /// The actual transmission starts when the bus is ready (turn-around
    /// timing satisfied). The async layer should wait for the `TxComplete`
    /// signal before submitting the next frame.
    fn submit_tx(&mut self, frame: &TxFrame) -> TxSubmitResult;

    /// Check if the bus is ready for a new TX frame.
    ///
    /// Returns `true` if `submit_tx` would accept a frame.
    fn tx_ready(&self) -> bool;

    /// Called by timer ISR to drive the bus state machine.
    ///
    /// This should be called at the TX_DUE timer rate to:
    /// - Check turn-around timing
    /// - Start pending TX DMA
    /// - Handle TX completion
    fn poll(&mut self);
}
