use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0, size = 8)]
struct RegA {
    pub b: Blk,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 16, size = 8)]
struct RegB {
    pub b: Blk,
}

fn main() {}
