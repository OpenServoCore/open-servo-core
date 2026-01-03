//! Async RTT I/O adapter for RPC service.
//!
//! Provides `embedded_io_async` compatible RTT streams for RPC communication.
//! Uses a poll-and-wait pattern driven by an external signal (e.g., SysTick).
//!
//! ## RTT Channel Layout
//!
//! - Up channel 0: defmt logging (NoBlockSkip)
//! - Up channel 1: RPC responses/telemetry (NoBlockTrim)
//! - Down channel 0: RPC requests
//!
//! ## Usage
//!
//! ```ignore
//! // Board crate defines the wait signal
//! static RPC_TICK: Signal<CriticalSectionRawMutex, ()> = Signal::new();
//!
//! // SysTick ISR signals it
//! fn SysTick() { RPC_TICK.signal(()); }
//!
//! // Initialize RTT with signal reference
//! let rtt = RttChannels::init(&RPC_TICK);
//! ```

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_io_async::{ErrorType, Read, Write};
use rtt_target::{rtt_init, ChannelMode, DownChannel, UpChannel};

/// Type alias for the RPC tick signal.
pub type RpcTickSignal = Signal<CriticalSectionRawMutex, ()>;

/// RTT channels for RPC communication.
pub struct RttChannels {
    pub rpc: RttRpcIo,
}

impl RttChannels {
    /// Initialize RTT and create async I/O adapter for RPC.
    ///
    /// The signal is used for async waiting on data availability.
    /// Typically signaled by a periodic interrupt (e.g., SysTick).
    ///
    /// This initializes RTT with:
    /// - Up channel 0: "defmt" (512 bytes, NoBlockSkip)
    /// - Up channel 1: "rpc" (1024 bytes, NoBlockTrim)
    /// - Down channel 0: "rpc" (256 bytes)
    pub fn init(rpc_signal: &'static RpcTickSignal) -> Self {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 512,
                    mode: ChannelMode::NoBlockSkip,
                    name: "defmt"
                }
                1: {
                    size: 1024,
                    mode: ChannelMode::NoBlockTrim,
                    name: "rpc"
                }
            }
            down: {
                0: {
                    size: 256,
                    name: "rpc"
                }
            }
        };

        // Set channel 0 as defmt channel when defmt is enabled
        #[cfg(feature = "defmt")]
        rtt_target::set_defmt_channel(channels.up.0);

        Self {
            rpc: RttRpcIo {
                up: channels.up.1,
                down: channels.down.0,
                signal: rpc_signal,
            },
        }
    }
}

/// Async RTT I/O for RPC communication.
pub struct RttRpcIo {
    up: UpChannel,
    down: DownChannel,
    signal: &'static RpcTickSignal,
}

/// Error type for RTT I/O (infallible).
#[derive(Debug, Copy, Clone)]
pub struct RttError;

impl embedded_io_async::Error for RttError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

impl ErrorType for RttRpcIo {
    type Error = RttError;
}

impl Read for RttRpcIo {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        #[cfg(feature = "defmt")]
        use core::sync::atomic::{AtomicU32, Ordering};
        #[cfg(feature = "defmt")]
        static POLL_COUNT: AtomicU32 = AtomicU32::new(0);

        loop {
            let n = self.down.read(buf);
            if n > 0 {
                #[cfg(feature = "defmt")]
                defmt::info!("RPC down read: {} bytes", n);
                return Ok(n);
            }
            // Log periodically to show loop is running
            #[cfg(feature = "defmt")]
            {
                let count = POLL_COUNT.fetch_add(1, Ordering::Relaxed);
                if count % 5000 == 0 {
                    defmt::info!("RPC poll #{}", count);
                }
            }
            // No data available, wait for signal then retry
            self.signal.wait().await;
        }
    }
}

impl Write for RttRpcIo {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.up.write(buf);
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
