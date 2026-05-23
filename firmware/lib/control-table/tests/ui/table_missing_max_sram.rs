#![feature(sync_unsafe_cell)]

use control_table::{Block, Region, Table};
use core::cell::SyncUnsafeCell;

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}
impl Blk {
    const fn const_new() -> Self {
        Self { x: 0 }
    }
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0, size = 4)]
struct Reg {
    pub b: Blk,
}
impl Reg {
    const fn const_new() -> Self {
        Self { b: Blk::const_new() }
    }
}

#[repr(C)]
#[derive(Table)]
struct NoMaxSram {
    #[ct_region]
    pub r: SyncUnsafeCell<Reg>,
}

fn main() {}
