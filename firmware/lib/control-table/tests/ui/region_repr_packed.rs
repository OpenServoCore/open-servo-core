use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Inner {
    x: u8,
}

#[repr(C, packed)]
#[derive(Region)]
#[ct_region(addr = 0, size = 16)]
struct Packed {
    pub inner: Inner,
}

fn main() {}
