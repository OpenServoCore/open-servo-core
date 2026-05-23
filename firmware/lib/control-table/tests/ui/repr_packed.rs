use control_table::Block;

#[repr(C, packed)]
#[derive(Block)]
struct Packed {
    x: u8,
    y: u16,
}

fn main() {}
