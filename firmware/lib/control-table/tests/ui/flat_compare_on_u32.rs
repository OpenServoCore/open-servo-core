use control_table::FlatBlock;

#[repr(C)]
#[derive(FlatBlock)]
struct WideCompare {
    #[ct_field(le = 10u32)]
    x: u32,
}

fn main() {}
