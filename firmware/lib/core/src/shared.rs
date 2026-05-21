use crate::ControlTable;

#[repr(C)]
pub struct Shared {
    pub table: ControlTable,
}

impl Shared {
    pub const fn const_new() -> Self {
        Self {
            table: ControlTable::const_new(),
        }
    }
}
