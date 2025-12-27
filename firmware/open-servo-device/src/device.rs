//! Device runner: ties together board + comms service.
//!
//! ## Architecture
//!
//! The [`Device`] type provides glue between board hardware and comms services.
//! It handles:
//!
//! - Half-duplex TX management via [`poll_tx`](Device::poll_tx)
//! - Board/comms wiring
//!
//! For kernel execution, use [`Executor`](crate::executor::Executor) which runs
//! in the ADC ISR and enforces single-writer semantics.
//!
//! ## Echo Handling
//!
//! - If the CommsService requests `DisableRxDuringTx`, we disable RX while sending.
//! - Otherwise, RX stays enabled and the service must filter echoed bytes.

use crate::comms_service::{CommsService, EchoPolicy};
use crate::uart_bus::UartBus;

/// Device glue type for board + comms.
///
/// `B` must implement:
/// - `UartBus` (byte transport boundary for comms)
///
/// `C` is the communications service (e.g. Dynamixel, CAN adapter).
///
/// For kernel execution, use [`Executor`](crate::executor::Executor) separately.
pub struct Device<B, C> {
    pub board: B,
    pub comms: C,
}

impl<B, C> Device<B, C>
where
    B: UartBus,
    C: CommsService,
{
    /// Construct a new device runner.
    #[inline]
    pub fn new(board: B, comms: C) -> Self {
        Self { board, comms }
    }

    /// Poll comms TX path (no kernel calls).
    ///
    /// This handles the TX side of half-duplex communication:
    /// - Start TX if pending and bus idle
    /// - Handle TX completion and re-enable RX
    ///
    /// For RX processing and HostOp execution, use the main loop helpers:
    /// - [`main_loop::parse_and_enqueue`](crate::main_loop::parse_and_enqueue)
    /// - [`main_loop::drain_and_respond`](crate::main_loop::drain_and_respond)
    ///
    /// With the [`Executor`](crate::executor::Executor) running in ADC ISR.
    pub fn poll_tx(&mut self) {
        let echo_policy = self.comms.echo_policy();

        // Start TX if pending and bus idle.
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

        // TX completion: release line and re-enable RX.
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

    /// Access the board.
    #[inline]
    pub fn board(&self) -> &B {
        &self.board
    }

    /// Access the board mutably.
    #[inline]
    pub fn board_mut(&mut self) -> &mut B {
        &mut self.board
    }

    /// Access the comms service.
    #[inline]
    pub fn comms(&self) -> &C {
        &self.comms
    }

    /// Access the comms service mutably.
    #[inline]
    pub fn comms_mut(&mut self) -> &mut C {
        &mut self.comms
    }
}
