pub mod dxl;

use crate::Shared;
use dxl::{Dxl, DxlIo};

pub struct ServicesIo<D: DxlIo> {
    pub dxl_io: D,
}

pub struct Services<D: DxlIo> {
    pub io: ServicesIo<D>,
    pub dxl: Dxl,
}

impl<D: DxlIo> Services<D> {
    pub const fn new(io: ServicesIo<D>) -> Self {
        Self {
            io,
            dxl: Dxl::new(),
        }
    }

    pub fn poll(&mut self, shared: &Shared) {
        self.dxl.poll(shared, &mut self.io.dxl_io);
    }
}
