use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct WithinOne {
    #[ct_field(within = 5u8)]
    x: u8,
}

fn main() {}
