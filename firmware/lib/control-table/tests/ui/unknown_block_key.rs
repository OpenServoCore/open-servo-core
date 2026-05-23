use control_table::Block;

#[repr(C)]
#[derive(Block)]
#[ct_block(nope = 1)]
struct UnknownBlockKey {
    x: u8,
}

fn main() {}
