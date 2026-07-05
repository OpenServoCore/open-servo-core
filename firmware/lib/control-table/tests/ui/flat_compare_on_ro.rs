use control_table::FlatBlock;

#[repr(C)]
#[derive(FlatBlock)]
struct CompareOnRo {
    #[ct_field(access = ro, le = 10u8)]
    x: u8,
}

fn main() {}
