use crate::traits::{DxlBus, ServicesIo};
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
        if let Some(packet) = bus.poll() {
            let mut d = Dispatcher::new(shared, bus, events, &mut self.staged);
            d.dispatch(packet);
        }
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}
