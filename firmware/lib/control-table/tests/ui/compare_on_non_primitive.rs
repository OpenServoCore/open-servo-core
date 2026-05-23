use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct BadCompare {
    #[ct_field(le = 0u8)]
    x: [u8; 4],
}

fn main() {}
