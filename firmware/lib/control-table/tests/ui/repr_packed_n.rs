use control_table::Block;

#[repr(C, packed(2))]
#[derive(Block)]
struct PackedN {
    x: u8,
    y: u16,
}

fn main() {}
