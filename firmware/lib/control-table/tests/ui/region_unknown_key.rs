use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0, size = 8, nope = 1)]
struct UnknownKey {
    pub b: Blk,
}

fn main() {}
