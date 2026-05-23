use control_table::{Access, Block, StagedView};
use core::mem::{offset_of, size_of};

#[repr(C)]
#[derive(Block)]
struct BlkA {
    a0: u8,
    a1: u16,
}

#[repr(C)]
#[derive(Block)]
struct BlkB {
    b0: u32,
    b1: bool,
}

mod basic {
    use super::{BlkA, BlkB};
    use control_table::Region;

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 100, size = 64)]
    pub struct TwoBlock {
        pub alpha: BlkA,
        pub beta: BlkB,
    }
}

use basic::TwoBlock;

#[test]
fn region_desc_carries_addr_and_size() {
    let d = TwoBlock::DESC;
    assert_eq!(d.addr, 100);
    assert_eq!(d.size, 64);
    assert_eq!(d.blocks.len(), 2);
}

#[test]
fn block_desc_addr_is_region_base_plus_struct_offset() {
    let blocks = TwoBlock::DESC.blocks;
    assert_eq!(blocks[0].addr, 100 + offset_of!(TwoBlock, alpha) as u16);
    assert_eq!(blocks[1].addr, 100 + offset_of!(TwoBlock, beta) as u16);
    assert_eq!(blocks[0].struct_offset, offset_of!(TwoBlock, alpha) as u16);
    assert_eq!(blocks[1].struct_offset, offset_of!(TwoBlock, beta) as u16);
    assert_eq!(blocks[0].size, BlkA::DESC.size);
    assert_eq!(blocks[1].size, BlkB::DESC.size);
}

#[test]
fn rebased_fields_are_absolute_addrs() {
    let blocks = TwoBlock::DESC.blocks;
    let alpha_base = 100 + offset_of!(TwoBlock, alpha) as u16;
    let beta_base = 100 + offset_of!(TwoBlock, beta) as u16;

    let alpha_fields = blocks[0].fields;
    assert_eq!(
        alpha_fields[0].addr,
        alpha_base + offset_of!(BlkA, a0) as u16
    );
    assert_eq!(
        alpha_fields[1].addr,
        alpha_base + offset_of!(BlkA, a1) as u16
    );

    let beta_fields = blocks[1].fields;
    assert_eq!(beta_fields[0].addr, beta_base + offset_of!(BlkB, b0) as u16);
    assert_eq!(beta_fields[1].addr, beta_base + offset_of!(BlkB, b1) as u16);

    // struct_offset stays block-relative
    assert_eq!(alpha_fields[0].struct_offset, offset_of!(BlkA, a0) as u16);
    assert_eq!(beta_fields[1].struct_offset, offset_of!(BlkB, b1) as u16);
}

#[test]
fn field_access_preserved_through_rebase() {
    let alpha_fields = TwoBlock::DESC.blocks[0].fields;
    assert_eq!(alpha_fields[0].access, Access::Rw);
}

#[test]
fn addr_hub_consts_resolve_to_absolute_field_addrs() {
    assert_eq!(
        basic::addr::alpha::A0,
        100 + offset_of!(TwoBlock, alpha) as u16 + offset_of!(BlkA, a0) as u16
    );
    assert_eq!(
        basic::addr::alpha::A1,
        100 + offset_of!(TwoBlock, alpha) as u16 + offset_of!(BlkA, a1) as u16
    );
    assert_eq!(
        basic::addr::beta::B0,
        100 + offset_of!(TwoBlock, beta) as u16 + offset_of!(BlkB, b0) as u16
    );
    assert_eq!(
        basic::addr::beta::B1,
        100 + offset_of!(TwoBlock, beta) as u16 + offset_of!(BlkB, b1) as u16
    );
}

fn ok_region(_v: &StagedView) -> Result<(), control_table::Error> {
    Ok(())
}

mod with_validator {
    use super::{BlkA, ok_region};
    use control_table::Region;

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0, size = 8, validators = [ok_region])]
    pub struct OneBlock {
        pub solo: BlkA,
    }
}

#[test]
fn ct_region_validators_list_populates_region_validators() {
    assert_eq!(with_validator::OneBlock::DESC.validators.len(), 1);
}

#[test]
fn region_storage_size_fits_declared_size_const_assert() {
    // If size_of::<TwoBlock>() > 64 this test file would fail to compile.
    // This assertion just keeps the invariant visible to humans reading the file.
    assert!(size_of::<TwoBlock>() <= 64);
}

#[repr(C)]
struct Trailer {
    pub _meta: [u8; 4],
}

mod with_skip {
    use super::{BlkA, Trailer};
    use control_table::Region;

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 200, size = 32)]
    pub struct Skipping {
        pub kept: BlkA,
        #[ct_region(skip)]
        pub _trailer: Trailer,
    }
}

#[test]
fn ct_region_skip_excludes_field_from_blocks_and_addr_hub() {
    let d = with_skip::Skipping::DESC;
    assert_eq!(d.blocks.len(), 1);
    assert_eq!(
        d.blocks[0].struct_offset,
        offset_of!(with_skip::Skipping, kept) as u16
    );
}
