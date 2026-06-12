use crate::traits::DxlBus;
use crate::{Shared, StagedWrites};

use super::dispatcher::Dispatcher;

/// Services-side DXL wrapper. Holds only the staged-RegWrite buffer — the
/// parser lives in the bus driver, so the CRC type doesn't appear in any
/// services-layer type. `poll` hands the bus a closure that wraps the
/// dispatcher; the bus invokes it with the parsed request and a reply
/// handle, both borrowed from disjoint pieces of bus-internal state.
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
        let staged = &mut self.staged;
        bus.poll(|packet, reply| {
            Dispatcher::new(shared, reply, staged).dispatch(packet);
        });
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}
