use control_table::Block;

#[repr(transparent)]
#[derive(Block)]
struct Transparent {
    x: u8,
}

fn main() {}
