use crate::traits::DxlBus;
use crate::{Shared, StagedWrites};

use super::dispatch::Dispatch;

/// Services-side DXL wrapper. Holds only the staged-RegWrite buffer; the bus
/// driver hands the dispatcher one fully-decoded, addressing-resolved
/// [`DxlRequest`](crate::traits::DxlRequest) per poll, so no per-packet
/// reassembly state lives here (the framer owns any cross-poll frame state).
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
        let mut dispatch = Dispatch::new(shared, &mut self.staged);
        bus.poll(&mut dispatch);
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}
