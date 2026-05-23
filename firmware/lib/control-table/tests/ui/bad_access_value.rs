use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct BadAccess {
    #[ct_field(access = wat)]
    x: u8,
}

fn main() {}
