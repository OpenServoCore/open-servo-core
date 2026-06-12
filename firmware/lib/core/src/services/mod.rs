pub mod dxl;

use crate::Shared;
use crate::traits::DxlBus;
use dxl::Dxl;

pub struct Services<B: DxlBus> {
    pub bus: B,
    pub dxl: Dxl,
}

impl<B: DxlBus> Services<B> {
    pub fn new(bus: B) -> Self {
        Self {
            bus,
            dxl: Dxl::new(),
        }
    }

    pub fn poll(&mut self, shared: &Shared) {
        self.dxl.poll(shared, &mut self.bus);
    }
}
