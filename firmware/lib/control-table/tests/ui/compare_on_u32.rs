use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct WideCompare {
    #[ct_field(le = 10u32)]
    x: u32,
}

fn main() {}
