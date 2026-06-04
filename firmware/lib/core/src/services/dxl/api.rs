use dxl_protocol::prelude::{ParseError, parse_one};
use heapless::Vec;

use crate::traits::{DxlBus, ServiceEvents, ServicesIo};
use crate::{Shared, StagedWrites};

use super::dispatcher::Dispatcher;

/// Covers realistic incoming traffic: a maxed-out 128 B Write (140 B frame)
/// or a ~36-slave BulkRead (~190 B). Larger BulkReads silently drop and the
/// master retries — saves 64 B over a 256 B cap.
const DXL_STITCH_LEN: usize = 192;

pub struct Dxl {
    stitch: Vec<u8, DXL_STITCH_LEN>,
    staged: StagedWrites,
}

impl Dxl {
    pub const fn new() -> Self {
        Self {
            stitch: Vec::new(),
            staged: StagedWrites::new(),
        }
    }

    pub fn poll<I: ServicesIo>(&mut self, shared: &Shared, io: &mut I) {
        let (bus, events) = io.parts();
        {
            let Some((head, tail)) = bus.rx_poll() else {
                return;
            };
            if head.len() + tail.len() > DXL_STITCH_LEN {
                return;
            }
            self.stitch.clear();
            let _ = self.stitch.extend_from_slice(head);
            let _ = self.stitch.extend_from_slice(tail);
        }
        if self.stitch.is_empty() {
            return;
        }
        dispatch_window(&self.stitch, shared, bus, events, &mut self.staged);
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
