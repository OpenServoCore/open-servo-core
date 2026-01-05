//! Async primitive traits for executor-agnostic services.
//!
//! These traits abstract over `embassy_sync` and `rtic_sync` primitives,
//! allowing services to be written once and work with any async executor.
//!
//! Firmware provides the concrete implementations (e.g., wrapping `embassy_sync::Signal`).

use core::future::Future;

use open_servo_units::{MicroSecond, TimeStampUs};

/// Async signal writer (sync side, callable from ISR).
///
/// Used to notify async tasks from interrupt context.
pub trait SignalWriter {
    /// Signal the reader. Non-blocking, ISR-safe.
    fn signal(&self);
}

/// Async signal reader (async side).
///
/// Used to wait for signals from ISR or other sync contexts.
pub trait SignalReader {
    /// Wait for a signal. Returns when signaled.
    fn wait(&self) -> impl Future<Output = ()>;
}

/// Channel sender (sync side).
///
/// Used to send values to async tasks.
pub trait Sender<T> {
    /// Try to send a value. Returns Err(val) if the channel is full.
    fn try_send(&self, val: T) -> Result<(), T>;
}

/// Channel receiver (async side).
///
/// Used to receive values from sync contexts.
pub trait Receiver<T> {
    /// Wait for and receive a value.
    fn recv(&self) -> impl Future<Output = T>;
}

/// Async timer for delays and timeouts.
///
/// Provides executor-agnostic timing using our own `MicroSecond` and `TimeStampUs` types.
/// Firmware implements this by wrapping `embassy_time::Timer` or similar.
pub trait AsyncTimer {
    /// Get current monotonic time.
    fn now(&self) -> TimeStampUs;

    /// Delay for the specified duration.
    fn delay(&self, duration: MicroSecond) -> impl Future<Output = ()>;
}
