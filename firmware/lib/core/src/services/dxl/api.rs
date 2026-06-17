use crate::traits::DxlBus;
use crate::{Shared, StagedWrites};

use super::dispatcher::{Dispatcher, Inflight};

/// Services-side DXL wrapper. Holds the staged-RegWrite buffer + the
/// in-flight instruction bookkeeping; the streaming parser lives in the
/// bus driver, so neither the CRC type nor any wire-byte buffer appears
/// in any services-layer type.
///
/// `inflight` lives here, not on `Dispatcher`, because a single packet's
/// Header / Payload / Crc events can straddle two `poll` wakes (the codec
/// fires HT on the 64-edge mark and IDLE on packet end — Header may land
/// on one, Crc on the next). If the dispatcher owned `inflight` it would
/// re-zero per poll and drop the reply.
pub struct Dxl {
    staged: StagedWrites,
    inflight: Option<Inflight>,
}

impl Dxl {
    pub fn new() -> Self {
        Self {
            staged: StagedWrites::new(),
            inflight: None,
        }
    }

    pub fn poll<B: DxlBus>(&mut self, shared: &Shared, bus: &mut B) {
        let mut dispatcher = Dispatcher::new(shared, &mut self.staged, &mut self.inflight);
        bus.poll(&mut dispatcher);
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}
