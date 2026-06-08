pub mod dxl;

use crate::Shared;
use crate::traits::{DxlBus, ServicesIo};
use dxl::Dxl;

pub struct Services<I: ServicesIo> {
    pub io: I,
    pub dxl: Dxl<<I::Bus as DxlBus>::Crc>,
}

impl<I: ServicesIo> Services<I> {
    pub fn new(io: I) -> Self {
        Self {
            io,
            dxl: Dxl::new(),
        }
    }

    pub fn poll(&mut self, shared: &Shared) {
        self.dxl.poll(shared, &mut self.io);
    }
}
