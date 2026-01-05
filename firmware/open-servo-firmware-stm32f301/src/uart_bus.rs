//! UART bus implementation for Device pattern.
//!
//! Provides polled TX with half-duplex awareness.

use stm32f3::stm32f301::USART1;

/// UART bus for Device<B, C> pattern.
pub struct UartBus {
    tx_active: bool,
}

impl UartBus {
    /// Create new UART bus.
    pub const fn new() -> Self {
        Self { tx_active: false }
    }

    /// Check if TX register is empty (ready to accept byte).
    #[inline]
    pub fn tx_ready(&self) -> bool {
        let usart = unsafe { &*USART1::ptr() };
        usart.isr.read().txe().bit_is_set()
    }

    /// Push a byte to TX.
    ///
    /// Only call when `tx_ready()` returns true.
    #[inline]
    pub fn tx_push(&mut self, byte: u8) {
        let usart = unsafe { &*USART1::ptr() };
        usart.tdr.write(|w| w.tdr().bits(byte as u16));
        self.tx_active = true;
    }

    /// Check if transmission is complete (TC flag).
    ///
    /// This indicates the shift register is empty, not just TDR.
    #[inline]
    pub fn tx_complete(&self) -> bool {
        let usart = unsafe { &*USART1::ptr() };
        usart.isr.read().tc().bit_is_set()
    }

    /// Check if TX is currently active.
    #[inline]
    pub fn is_tx_active(&self) -> bool {
        self.tx_active
    }

    /// Mark TX as complete.
    ///
    /// Called when TC flag is set and no more bytes to send.
    #[inline]
    pub fn tx_done(&mut self) {
        self.tx_active = false;
    }

    /// Set TX enable for half-duplex direction control.
    ///
    /// Stage-0: no-op (full-duplex assumed or handled by hardware).
    #[inline]
    pub fn set_tx_enable(&mut self, _enable: bool) {
        // Stage-0: no direction control needed
        // For half-duplex, this would toggle a GPIO or USART HDSEL
    }
}
