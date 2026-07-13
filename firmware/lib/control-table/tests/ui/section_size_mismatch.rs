use control_table::{Block, Section};

#[repr(C)]
#[derive(Block)]
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
