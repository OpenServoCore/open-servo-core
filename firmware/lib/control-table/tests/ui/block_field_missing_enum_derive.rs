use control_table::Block;

#[repr(u8)]
#[derive(Copy, Clone)]
enum BareEnum {
    A = 0,
    B = 1,
}

#[repr(C)]
#[derive(Block)]
struct UsesBare {
    mode: BareEnum,
}

fn main() {}
