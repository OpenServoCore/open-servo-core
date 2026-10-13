use control_table::Block;

#[repr(C)]
#[derive(Block)]
struct WithinAbs {
    #[ct_field(within = (-5i8, 5i8), abs)]
    x: i8,
}

fn main() {}
