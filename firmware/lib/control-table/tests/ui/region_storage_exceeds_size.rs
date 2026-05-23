use control_table::{Block, Region};

#[repr(C)]
#[derive(Block)]
struct Big {
    a: u32,
    b: u32,
    c: u32,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0, size = 4)]
struct TooSmall {
    pub b: Big,
}

fn main() {}
