//! RTT-based DebugIo implementation for STM32F301.
//!
//! Uses two RTT channels:
//! - Channel 0: Logger output (write-only, global with critical section)
//! - Channel 1: REPL (read/write, owned by shell, lock-free)

use open_servo_hw::DebugIo;
use rtt_target::{rtt_init, ChannelMode, DownChannel, UpChannel};

/// RTT-based debug I/O for the REPL (channel 1).
pub struct RttDebugIo {
    up: UpChannel,
    down: DownChannel,
}

/// RTT-based logger output (channel 0).
/// Implements DebugIo for use with open-servo-log.
pub struct RttLoggerIo {
    up: UpChannel,
}

/// Initialize RTT channels and return both logger and REPL I/O handles.
///
/// Returns (logger_io, repl_io):
/// - logger_io: Channel 0 for logging (store in static for logger)
/// - repl_io: Channel 1 for REPL (give to DebugShell)
pub fn init_rtt() -> (RttLoggerIo, RttDebugIo) {
    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                mode: ChannelMode::NoBlockSkip,
                name: "log"
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

    let logger_io = RttLoggerIo {
        up: channels.up.0,
    };

    let repl_io = RttDebugIo {
        up: channels.up.1,
        down: channels.down.1,
    };

    (logger_io, repl_io)
}

impl DebugIo for RttLoggerIo {
    fn try_read(&mut self) -> Option<u8> {
        // Logger is write-only
        None
    }

    fn write(&mut self, data: &[u8]) {
        self.up.write(data);
    }

    fn flush(&mut self) {}
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
