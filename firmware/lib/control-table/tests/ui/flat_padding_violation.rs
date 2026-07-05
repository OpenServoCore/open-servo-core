use control_table::FlatBlock;

#[repr(C)]
#[derive(FlatBlock)]
struct Padded {
    a: u8,
    b: u16,
}

fn main() {}
