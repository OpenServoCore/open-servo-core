use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct CompareOnRo {
    #[ct_field(access = ro, le = 10u8)]
    x: u8,
}

fn main() {}
