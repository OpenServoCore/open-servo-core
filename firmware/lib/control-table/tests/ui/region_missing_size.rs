use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0)]
struct MissingSize {
    pub b: Blk,
}

fn main() {}
