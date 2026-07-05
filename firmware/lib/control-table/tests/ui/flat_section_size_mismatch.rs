use control_table::{FlatBlock, Section};

#[repr(C)]
#[derive(FlatBlock)]
struct Blk {
    a: u16,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = 0, size = 4)]
struct Sec {
    blk: Blk,
}

fn main() {}
