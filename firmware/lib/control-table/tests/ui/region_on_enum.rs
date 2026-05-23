use control_table::Region;

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0, size = 4)]
enum NotAStruct {
    A,
    B,
}

fn main() {}
