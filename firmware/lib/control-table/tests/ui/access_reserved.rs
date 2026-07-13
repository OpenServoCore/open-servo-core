use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct BadReserved {
    #[ct_field(access = reserved)]
    x: u8,
}

fn main() {}
