use crate::{ControlTable, StreamCoord};

#[repr(C)]
pub struct Shared {
    pub table: ControlTable,
    pub stream: StreamCoord,
}

impl Shared {
    pub const fn const_new() -> Self {
        Self {
            table: ControlTable::const_new(),
            stream: StreamCoord::const_new(),
        }
    }
}
