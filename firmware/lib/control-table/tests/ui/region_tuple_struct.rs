use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0, size = 8)]
struct TupleRegion(Blk);

fn main() {}
