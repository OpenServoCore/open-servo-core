#![feature(sync_unsafe_cell)]

use control_table::{FlatBlock, FlatTable};

#[repr(C)]
#[derive(FlatBlock)]
struct Blk {
    a: u16,
}

mod a {
    use super::Blk;
    use control_table::Section;

    #[repr(C)]
    #[derive(Section)]
    #[ct_section(base = 0, size = 2)]
    pub struct SecA {
        pub blk: Blk,
    }
}

mod b {
    use super::Blk;
    use control_table::Section;

    // Declared base 0, but the field below sits at byte offset 2.
    #[repr(C)]
    #[derive(Section)]
    #[ct_section(base = 0, size = 2)]
    pub struct SecB {
        pub blk: Blk,
    }
}

#[repr(C)]
#[derive(FlatTable)]
#[ct_table(size = 4)]
struct Tbl {
    a: a::SecA,
    b: b::SecB,
}

fn main() {}
