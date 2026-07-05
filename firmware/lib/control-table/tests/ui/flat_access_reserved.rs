use control_table::FlatBlock;

#[repr(C)]
#[derive(FlatBlock)]
struct BadReserved {
    #[ct_field(access = reserved)]
    x: u8,
}

fn main() {}
