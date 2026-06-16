use crate::traits::DxlBus;
use crate::{Shared, StagedWrites};

use super::dispatcher::Dispatcher;

/// Services-side DXL wrapper. Holds only the staged-RegWrite buffer — the
/// streaming parser lives in the bus driver, so neither the CRC type nor any
/// wire-byte buffer appears in any services-layer type.
pub struct Dxl {
    staged: StagedWrites,
}

impl Dxl {
    pub fn new() -> Self {
        Self {
            staged: StagedWrites::new(),
        }
    }

    pub fn poll<B: DxlBus>(&mut self, shared: &Shared, bus: &mut B) {
        let mut dispatcher = Dispatcher::new(shared, &mut self.staged);
        bus.poll(&mut dispatcher);
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}
