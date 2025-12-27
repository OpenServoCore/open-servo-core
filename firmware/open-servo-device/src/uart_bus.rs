//! UART bus abstraction for Dynamixel-style half-duplex UART.
//!
//! This trait is intentionally low-level and non-blocking.
//! It is designed to be implemented by a board/HAL driver that uses
//! DMA+ISR ring buffers or interrupt-driven FIFOs.
//!
//! ### Single-wire echo
//! Many single-wire half-duplex designs echo transmitted bytes back into RX.
//! Higher layers can handle this by either:
//! - disabling RX while TX is active, or
//! - filtering echoed bytes in software.
//!
//! This trait supports both approaches.

/// UART receive error (optional).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum UartError {
    Framing,
    Parity,
    Overrun,
    Other,
}

/// Minimal UART bus contract.
///
/// Implementations should be ISR-friendly:
/// - `rx_pop()` should drain from an ISR-fed buffer
/// - `tx_push()` should enqueue into a TX FIFO/buffer
pub trait UartBus {
    /// Pop one received byte if available.
    fn rx_pop(&mut self) -> Option<u8>;

    /// Optional: pop a UART error if tracked by the HAL.
    #[inline]
    fn rx_error_pop(&mut self) -> Option<UartError> {
        None
    }

    /// Push one byte for transmission.
    ///
    /// Returns `true` if accepted; `false` if the TX queue/FIFO is full.
    fn tx_push(&mut self, b: u8) -> bool;

    /// Kick/flush transmission of queued bytes.
    ///
    /// Some HALs auto-start; those can implement this as a no-op.
    fn tx_kick(&mut self);

    /// Returns `true` if TX is currently busy (actively sending or pending bytes).
    fn tx_busy(&self) -> bool;

    /// Returns `true` when the last transmission is fully complete (shift register empty).
    fn tx_complete(&self) -> bool;

    /// Control half-duplex direction buffer.
    ///
    /// - `true`: drive the line (transmit)
    /// - `false`: release / listen (receive)
    fn set_tx_enable(&mut self, en: bool);

    /// Optional: enable/disable RX path.
    ///
    /// If your hardware can disable RX during TX, you should implement this.
    /// Otherwise leave the default no-op and rely on software echo filtering.
    #[inline]
    fn set_rx_enable(&mut self, _en: bool) {}
}
