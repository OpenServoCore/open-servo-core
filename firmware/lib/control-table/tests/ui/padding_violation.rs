use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct Padded {
    a: u8,
    b: u16,
}

fn main() {}
