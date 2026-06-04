use dxl_protocol::prelude::{ParseError, parse_one};

use crate::traits::{DxlBus, ServicesIo};
use crate::{RingReader, Shared, StagedWrites};

use super::dispatcher::Dispatcher;

const DXL_SCRATCH_LEN: usize = 256;

pub struct Dxl {
    reader: RingReader<DXL_SCRATCH_LEN>,
    staged: StagedWrites,
}

impl Dxl {
    pub const fn new() -> Self {
        Self {
            reader: RingReader::new(),
            staged: StagedWrites::new(),
        }
    }

    pub fn poll<I: ServicesIo>(&mut self, shared: &Shared, io: &mut I) {
        let (bus, events) = io.parts();
        let Some(snap) = bus.rx_poll() else {
            return;
        };
        self.reader.ingest(snap.ring(), snap.write_pos());

        let window = self.reader.peek().len();
        if window == 0 {
            return;
        }
        let request_end_bytes = self.reader.consumed_total().wrapping_add(window as u32);

        // Only the frame ending exactly at the anchor gets dispatched —
        // earlier frames in the window are pre-IDLE traffic the master has
        // moved on from. Cursor advances past the whole window regardless.
        let mut offset = 0;
        while offset < window {
            match parse_one(&self.reader.peek()[offset..]) {
                Ok((packet, used)) => {
                    if offset + used == window {
                        let mut d = Dispatcher::new(shared, bus, events, &mut self.staged);
                        d.dispatch(packet, request_end_bytes);
                        break;
                    }
                    offset += used;
                }
                Err(ParseError::Incomplete) => break,
                Err(ParseError::Resync { skip })
                | Err(ParseError::BadCrc { skip })
                | Err(ParseError::BadInstruction { skip })
                | Err(ParseError::BadLength { skip }) => {
                    offset = (offset + skip).min(window);
                }
            }
        }

        self.reader.consume(window);
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}
