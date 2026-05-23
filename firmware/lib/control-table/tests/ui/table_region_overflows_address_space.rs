#![feature(sync_unsafe_cell)]

use control_table::Table;
use core::cell::SyncUnsafeCell;

mod tail {
    use control_table::{Block, Region};

    #[repr(C)]
    #[derive(Block)]
    pub struct Blk {
        pub x: [u8; 4],
    }
    impl Blk {
        pub const fn const_new() -> Self {
            Self { x: [0; 4] }
        }
    }

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0xFFFE, size = 4)]
    pub struct Tail {
        pub b: Blk,
    }
    impl Tail {
        pub const fn const_new() -> Self {
            Self { b: Blk::const_new() }
        }
    }
}

#[repr(C)]
#[derive(Table)]
#[ct_table(max_sram = 256)]
struct Overflowing {
    #[ct_region]
    pub tail: SyncUnsafeCell<tail::Tail>,
}

fn main() {}
