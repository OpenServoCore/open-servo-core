//! RTT-based DebugIo implementation for STM32F301.
//!
//! Uses RTT channels for debug REPL I/O:
//! - Channel 0: defmt binary logs (up only)
//! - Channel 1: REPL interactive (up + down, paired for cargo-embed)

use open_servo_hw::DebugIo;
use rtt_target::{rtt_init, ChannelMode, DownChannel, UpChannel};

/// RTT-based debug I/O for the REPL.
///
/// Holds the RTT channels for REPL communication (separate from defmt).
pub struct RttDebugIo {
    up: UpChannel,
    down: DownChannel,
}

impl RttDebugIo {
    /// Initialize RTT with all channels and return the REPL I/O handle.
    ///
    /// This sets up:
    /// - Channel 0: defmt (up only)
    /// - Channel 1: REPL (up + down paired for interactive use)
    ///
    /// Must be called once at startup before any defmt logging or REPL use.
    pub fn init() -> Self {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 1024,
                    mode: ChannelMode::NoBlockSkip,
                    name: "defmt"
                }
                1: {
                    size: 512,
                    mode: ChannelMode::NoBlockSkip,
                    name: "repl"
                }
            }
            down: {
                0: {
                    size: 16,
                    name: "unused"
                }
                1: {
                    size: 128,
                    name: "repl"
                }
            }
        };

        // Configure defmt to use channel 0
        rtt_target::set_defmt_channel(channels.up.0);

        Self {
            up: channels.up.1,
            down: channels.down.1,
        }
    }
}

impl DebugIo for RttDebugIo {
    fn try_read(&mut self) -> Option<u8> {
        let mut buf = [0u8; 1];
        if self.down.read(&mut buf) == 1 {
            Some(buf[0])
        } else {
            None
        }
    }

    fn write(&mut self, data: &[u8]) {
        // Best-effort write; RTT is configured as NoBlockSkip so this won't block
        self.up.write(data);
    }

    fn flush(&mut self) {
        // RTT doesn't need explicit flushing
    }
}
