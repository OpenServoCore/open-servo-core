use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct Dup {
    #[ct_field(le = 10u8, lt = 20u8)]
    x: u8,
}

fn main() {}
