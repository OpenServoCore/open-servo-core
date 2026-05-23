use crate::ControlTable;

#[repr(C)]
pub struct Shared {
    pub table: ControlTable,
}

#[allow(clippy::new_without_default)]
impl Shared {
    pub const fn new() -> Self {
        Self {
            table: ControlTable::new(),
        }
    }
}
