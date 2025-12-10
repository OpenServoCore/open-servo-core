//! UART communication traits.

use crate::types::UartPort;

/// UART communication.
///
/// Used for servo bus protocol and debug shell.
pub trait UartDriver {
    /// Write bytes to UART.
    fn uart_write(&mut self, port: UartPort, data: &[u8]);

    /// Read a byte from UART (non-blocking).
    fn uart_read_byte(&mut self, port: UartPort) -> Option<u8>;
}
