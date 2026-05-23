#![feature(sync_unsafe_cell)]

use control_table::Table;
use core::cell::SyncUnsafeCell;

mod big {
    use control_table::{Block, Region};

    #[repr(C)]
    #[derive(Block)]
    pub struct Big {
        pub a: [u8; 256],
    }
    impl Big {
        pub const fn const_new() -> Self {
            Self { a: [0; 256] }
        }
    }

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0, size = 256)]
    pub struct BigRegion {
        pub b: Big,
    }
    impl BigRegion {
        pub const fn const_new() -> Self {
            Self { b: Big::const_new() }
        }
    }
}

#[repr(C)]
#[derive(Table)]
#[ct_table(max_sram = 64)]
struct TooBig {
    #[ct_region]
    pub big: SyncUnsafeCell<big::BigRegion>,
}

fn main() {}
