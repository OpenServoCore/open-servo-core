pub mod dxl;

use crate::Shared;
use dxl::{Dxl, ServicesIo};

pub struct Services<I: ServicesIo> {
    pub io: I,
    pub dxl: Dxl,
}

impl<I: ServicesIo> Services<I> {
    pub const fn new(io: I) -> Self {
        Self {
            io,
            dxl: Dxl::new(),
        }
    }

    pub fn poll(&mut self, shared: &Shared) {
        self.dxl.poll(shared, &mut self.io);
    }
}
