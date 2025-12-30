//! Async RTT I/O adapter for debug shell.
//!
//! Provides an `embedded_io_async` compatible RTT stream for the debug shell.
//! Uses a poll-and-wait pattern driven by an external signal (e.g., SysTick).
//!
//! ## RTT Channel Layout
//!
//! - Up channel 0: defmt logging (NoBlockSkip)
//! - Up channel 1: debug_shell output (NoBlockTrim)
//! - Down channel 0: debug_shell input
//!
//! ## Usage
//!
//! ```ignore
//! // Board crate defines the wait signal
//! static DEBUG_TICK: Signal<CriticalSectionRawMutex, ()> = Signal::new();
//!
//! // SysTick ISR signals it
//! fn SysTick() { DEBUG_TICK.signal(()); }
//!
//! // Initialize RTT with signal reference
//! let rtt = RttAsyncIo::init(&DEBUG_TICK);
//! ```

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_io_async::{ErrorType, Read, Write};
use rtt_target::{rtt_init, ChannelMode, DownChannel, UpChannel};

/// Type alias for the debug tick signal.
pub type DebugTickSignal = Signal<CriticalSectionRawMutex, ()>;

/// Async RTT I/O for the debug shell.
///
/// Uses a reference to an Embassy Signal for async waiting.
pub struct RttAsyncIo {
    up: UpChannel,
    down: DownChannel,
    signal: &'static DebugTickSignal,
}

impl RttAsyncIo {
    /// Initialize RTT and create async I/O adapter.
    ///
    /// The `signal` is waited on when no input is available.
    /// Typically this is signaled by a periodic interrupt (e.g., SysTick).
    ///
    /// This initializes RTT with:
    /// - Up channel 0: "defmt" (512 bytes, NoBlockSkip)
    /// - Up channel 1: "debug_shell" (512 bytes, NoBlockTrim)
    /// - Down channel 0: "debug_shell" (128 bytes)
    pub fn init(signal: &'static DebugTickSignal) -> Self {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 512,
                    mode: ChannelMode::NoBlockSkip,
                    name: "defmt"
                }
                1: {
                    size: 512,
                    mode: ChannelMode::NoBlockTrim,
                    name: "debug_shell"
                }
            }
            down: {
                0: {
                    size: 128,
                    name: "debug_shell"
                }
            }
        };

        // Set channel 0 as defmt channel when defmt is enabled
        #[cfg(feature = "defmt")]
        rtt_target::set_defmt_channel(channels.up.0);

        Self {
            up: channels.up.1,
            down: channels.down.0,
            signal,
        }
    }
}

/// Error type for RTT I/O (infallible).
#[derive(Debug, Copy, Clone)]
pub struct RttError;

impl embedded_io_async::Error for RttError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

impl ErrorType for RttAsyncIo {
    type Error = RttError;
}

impl Read for RttAsyncIo {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            let n = self.down.read(buf);
            if n > 0 {
                return Ok(n);
            }
            // No data available, wait for signal then retry
            self.signal.wait().await;
        }
    }
}

impl Write for RttAsyncIo {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.up.write(buf);
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // RTT doesn't need explicit flushing
        Ok(())
    }
}
