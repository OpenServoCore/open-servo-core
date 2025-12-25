//! RTT-based DebugIo implementation for embedded targets.
//!
//! Uses RTT channel 0 for defmt (up only), channel 1 for REPL (bidirectional)
//!
//! This is a generic implementation that can be used by any Cortex-M board.

use open_servo_hw::DebugIo;
use rtt_target::{rtt_init, ChannelMode, DownChannel, UpChannel};

/// RTT-based debug I/O for the REPL.
pub struct RttDebugIo {
    up: UpChannel,
    down: DownChannel,
}

/// Initialize RTT channels and return REPL I/O handle.
///
/// Sets up:
/// - Channel 0 (up): defmt logging
/// - Channel 1 (up): REPL output  
/// - Channel 0 (down): REPL input
pub fn init_rtt() -> RttDebugIo {
    // Initialize all channels in one call
    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
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
                size: 128,
                name: "repl"
            }
        }
    };

    // Set channel 0 as defmt channel when defmt is enabled
    #[cfg(feature = "defmt")]
    rtt_target::set_defmt_channel(channels.up.0);

    RttDebugIo {
        up: channels.up.1,
        down: channels.down.0,
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
