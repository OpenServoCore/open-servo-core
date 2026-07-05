use crate::ControlTableCell;

#[repr(C)]
pub struct Shared {
    pub table: ControlTableCell,
}

#[allow(clippy::new_without_default)]
impl Shared {
    pub const fn new() -> Self {
        Self {
            table: ControlTableCell::new(),
        }
    }
}
