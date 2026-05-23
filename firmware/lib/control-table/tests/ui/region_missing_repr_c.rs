use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}

#[derive(Region)]
#[ct_region(addr = 0, size = 8)]
struct NoReprC {
    pub b: Blk,
}

fn main() {}
