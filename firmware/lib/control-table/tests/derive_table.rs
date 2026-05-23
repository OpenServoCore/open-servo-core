#![feature(sync_unsafe_cell)]

use control_table::{Router, Table};
use core::cell::SyncUnsafeCell;
use core::mem::size_of;

mod config {
    use control_table::{Block, Region};

    #[repr(C)]
    #[derive(Block)]
    pub struct CfgA {
        pub a0: u8,
        pub a1: u16,
    }

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0x0000, size = 16)]
    pub struct ConfigRegs {
        pub a: CfgA,
    }
}

mod telemetry {
    use control_table::{Block, Region};

    #[repr(C)]
    #[derive(Block)]
    pub struct TlmA {
        pub t0: u32,
    }

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0x0100, size = 8)]
    pub struct TelemetryRegs {
        pub a: TlmA,
    }
}

use config::ConfigRegs;
use telemetry::TelemetryRegs;

#[repr(C)]
#[derive(Table)]
#[ct_table(max_sram = 1024)]
struct ControlTable {
    #[ct_region]
    pub config: SyncUnsafeCell<ConfigRegs>,
    #[ct_region]
    pub telemetry: SyncUnsafeCell<TelemetryRegs>,
}

#[test]
fn regions_const_lists_each_region_desc_in_field_order() {
    let regions = ControlTable::REGIONS;
    assert_eq!(regions.len(), 2);
    assert_eq!(regions[0].addr, ConfigRegs::DESC.addr);
    assert_eq!(regions[1].addr, TelemetryRegs::DESC.addr);
}

#[test]
fn new_constructs_each_region_via_its_new() {
    static TBL: ControlTable = ControlTable::new();
    let cfg = unsafe { &*TBL.config.get() };
    assert_eq!(cfg.a.a0, 0);
    assert_eq!(cfg.a.a1, 0);
    let tlm = unsafe { &*TBL.telemetry.get() };
    assert_eq!(tlm.a.t0, 0);
}

#[test]
fn router_regions_returns_the_regions_const() {
    let t = ControlTable::new();
    let regions = t.regions();
    assert_eq!(regions.len(), 2);
    assert_eq!(regions[0].addr, ConfigRegs::DESC.addr);
}

#[test]
fn region_base_resolves_each_region_to_its_cell_pointer() {
    let t = ControlTable::new();
    let cfg_base = t.region_base(ConfigRegs::DESC).unwrap();
    let tlm_base = t.region_base(TelemetryRegs::DESC).unwrap();
    assert_eq!(cfg_base, t.config.get() as *mut u8);
    assert_eq!(tlm_base, t.telemetry.get() as *mut u8);
    assert_ne!(cfg_base, tlm_base);
}

#[test]
fn region_base_returns_none_for_descriptor_we_did_not_emit() {
    let t = ControlTable::new();
    let stranger = control_table::RegionDesc {
        addr: ConfigRegs::DESC.addr,
        size: ConfigRegs::DESC.size,
        blocks: &[],
        validators: &[],
    };
    assert!(t.region_base(&stranger).is_none());
}

#[test]
fn addr_hub_reexports_each_region_addr_module() {
    assert_eq!(
        addr::config::a::A0,
        ConfigRegs::DESC.addr
            + core::mem::offset_of!(ConfigRegs, a) as u16
            + core::mem::offset_of!(config::CfgA, a0) as u16,
    );
    assert_eq!(
        addr::telemetry::a::T0,
        TelemetryRegs::DESC.addr
            + core::mem::offset_of!(TelemetryRegs, a) as u16
            + core::mem::offset_of!(telemetry::TlmA, t0) as u16,
    );
}

#[test]
fn table_storage_size_fits_max_sram_const_assert() {
    assert!(size_of::<ControlTable>() <= 1024);
}

mod renamed {
    use control_table::{Block, Region};

    #[repr(C)]
    #[derive(Block)]
    pub struct Blk {
        pub x: u8,
    }

    #[repr(C)]
    #[derive(Region)]
    #[ct_region(addr = 0x0200, size = 8)]
    pub struct RenamedRegs {
        pub b: Blk,
    }
}

mod with_override {
    use super::renamed;
    use control_table::Table;
    use core::cell::SyncUnsafeCell;

    #[repr(C)]
    #[derive(Table)]
    #[ct_table(max_sram = 256)]
    pub struct OverrideTable {
        #[ct_region(addr_mod = super::super::renamed)]
        pub renamed_field: SyncUnsafeCell<renamed::RenamedRegs>,
    }
}

#[test]
fn addr_mod_override_re_exports_from_explicit_path_not_field_name() {
    static TBL: with_override::OverrideTable = with_override::OverrideTable::new();
    let r = unsafe { &*TBL.renamed_field.get() };
    assert_eq!(r.b.x, 0);

    assert_eq!(
        with_override::addr::renamed_field::b::X,
        renamed::RenamedRegs::DESC.addr
            + core::mem::offset_of!(renamed::RenamedRegs, b) as u16
            + core::mem::offset_of!(renamed::Blk, x) as u16,
    );
}
