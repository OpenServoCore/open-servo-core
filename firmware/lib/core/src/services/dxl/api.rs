use dxl_protocol::prelude::{ParseError, parse_one};

use crate::traits::{DxlBus, ServiceEvents, ServicesIo};
use crate::{Shared, StagedWrites};

use super::dispatcher::Dispatcher;

pub struct Dxl {
    staged: StagedWrites,
}

impl Dxl {
    pub const fn new() -> Self {
        Self {
            staged: StagedWrites::new(),
        }
    }

    pub fn poll<I: ServicesIo>(&mut self, shared: &Shared, io: &mut I) {
        let (bus, events) = io.parts();
        let Some(window) = bus.rx_poll() else {
            return;
        };
        if window.is_empty() {
            return;
        }
        dispatch_window(window, shared, bus, events, &mut self.staged);
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}

/// Walk the IDLE-bounded window and dispatch the frame ending exactly at
/// the wire-end — earlier frames are pre-IDLE traffic the master has moved
/// on from, so they're parsed only to advance the cursor and dropped.
fn dispatch_window<B: DxlBus, E: ServiceEvents>(
    window: &[u8],
    shared: &Shared,
    bus: &mut B,
    events: &mut E,
    staged: &mut StagedWrites,
) {
    let n = window.len();
    let mut offset = 0;
    while offset < n {
        match parse_one(&window[offset..]) {
            Ok((packet, used)) => {
                if offset + used == n {
                    let mut d = Dispatcher::new(shared, bus, events, staged);
                    d.dispatch(packet);
                    return;
                }
                offset += used;
            }
            Err(ParseError::Incomplete) => return,
            Err(ParseError::Resync { skip })
            | Err(ParseError::BadCrc { skip })
            | Err(ParseError::BadInstruction { skip })
            | Err(ParseError::BadLength { skip }) => {
                offset = (offset + skip).min(n);
            }
        }
    }
}
