#![feature(sync_unsafe_cell)]

use control_table::{Block, Region, Table};
use core::cell::SyncUnsafeCell;

#[repr(C)]
#[derive(Block)]
struct Blk {
    x: u8,
}

#[repr(C)]
#[derive(Region)]
#[ct_region(addr = 0, size = 8)]
struct Regs {
    pub b: Blk,
}

#[repr(C, packed)]
#[derive(Table)]
#[ct_table(max_sram = 64)]
struct PackedTable {
    #[ct_region]
    pub r: SyncUnsafeCell<Regs>,
}

fn main() {}
