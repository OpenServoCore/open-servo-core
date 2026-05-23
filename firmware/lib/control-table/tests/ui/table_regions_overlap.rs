#![feature(sync_unsafe_cell)]

use control_table::Table;
use core::cell::SyncUnsafeCell;

mod a {
    use control_table::{Block, Region};

    #[repr(C)]
    #[derive(Block)]
    pub struct BlkA {
        pub x: [u8; 16],
    }
    impl BlkA {
        pub const fn const_new() -> Self {
            Self { x: [0; 16] }
        }
    }

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0x0000, size = 16)]
    pub struct RegA {
        pub b: BlkA,
    }
    impl RegA {
        pub const fn const_new() -> Self {
            Self { b: BlkA::const_new() }
        }
    }
}

mod b {
    use control_table::{Block, Region};

    #[repr(C)]
    #[derive(Block)]
    pub struct BlkB {
        pub x: [u8; 16],
    }
    impl BlkB {
        pub const fn const_new() -> Self {
            Self { x: [0; 16] }
        }
    }

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0x0008, size = 16)]
    pub struct RegB {
        pub b: BlkB,
    }
    impl RegB {
        pub const fn const_new() -> Self {
            Self { b: BlkB::const_new() }
        }
    }
}

#[repr(C)]
#[derive(Table)]
#[ct_table(max_sram = 1024)]
struct Overlap {
    #[ct_region]
    pub a: SyncUnsafeCell<a::RegA>,
    #[ct_region]
    pub b: SyncUnsafeCell<b::RegB>,
}

fn main() {}
