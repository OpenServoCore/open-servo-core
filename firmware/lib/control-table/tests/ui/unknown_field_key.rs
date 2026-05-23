use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct UnknownKey {
    #[ct_field(nope = 1)]
    x: u8,
}

fn main() {}
