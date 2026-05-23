use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(size = 8)]
struct MissingAddr {
    pub b: Blk,
}

fn main() {}
