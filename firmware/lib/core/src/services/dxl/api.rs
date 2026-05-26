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
        let (bus, device) = io.parts();
        let snap = bus.received();
        self.reader.ingest(snap.ring(), snap.write_pos());

        let mut d = Dispatcher::new(shared, bus, device, &mut self.staged);
        loop {
            match parse_one(self.reader.peek()) {
                Ok((packet, used)) => {
                    let parsed_end = self.reader.consumed_total().wrapping_add(used as u32);
                    d.dispatch(packet, parsed_end);
                    self.reader.consume(used);
                }
                Err(ParseError::Incomplete) => break,
                Err(ParseError::Resync { skip })
                | Err(ParseError::BadCrc { skip })
                | Err(ParseError::BadInstruction { skip })
                | Err(ParseError::BadLength { skip }) => {
                    self.reader.consume(skip);
                }
            }
        }
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}
