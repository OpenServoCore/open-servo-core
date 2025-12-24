//! RTT-based DebugIo implementation for STM32F301.
//!
//! Uses RTT channel 0 for REPL (bidirectional), channel 1 for defmt (up only)

use open_servo_hw::DebugIo;
use rtt_target::{rtt_init, ChannelMode, DownChannel, UpChannel};
#[cfg(feature = "defmt")]
use rtt_target::set_defmt_channel;

/// RTT-based debug I/O for the REPL (channel 1).
pub struct RttDebugIo {
    up: UpChannel,
    down: DownChannel,
}

/// Initialize RTT channels and return REPL I/O handle.
///
/// Returns repl_io for channel 0 (channel 1 is for defmt when enabled)
pub fn init_rtt() -> RttDebugIo {
    // Initialize all channels in one call
    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                mode: ChannelMode::NoBlockSkip,
                name: "repl"
            }
            1: {
                size: 512,
                mode: ChannelMode::NoBlockSkip,
                name: "defmt"
            }
        }
        down: {
            0: {
                size: 128,
                name: "repl"
            }
        }
    };
    
    // Set channel 1 as defmt channel when defmt is enabled
    #[cfg(feature = "defmt")]
    rtt_target::set_defmt_channel(channels.up.1);
    
    RttDebugIo {
        up: channels.up.0,
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
