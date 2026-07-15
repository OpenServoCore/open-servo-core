#![feature(sync_unsafe_cell)]

use control_table::descriptor::{EnumVariant, FieldKind};
use control_table::{Block, Enum, Error, RegisterFile, Table, ValidationKind};
use core::mem::{offset_of, size_of};

#[repr(u8)]
#[derive(Copy, Clone, Enum)]
enum Mode {
    Stop = 0,
    Go = 1,
}

// Field order keeps the struct dense (no repr(C) padding) so the derive's
// zero-padding const assert accepts it.
#[repr(C)]
#[derive(Block)]
struct Basic {
    b: u16,
    a: u8,
    c: bool,
    mode: Mode,
    d: u8,
}

#[test]
fn ct_size_tracks_struct_size() {
    assert_eq!(Basic::CT_SIZE, size_of::<Basic>() as u16);
    assert_eq!(Basic::CT_SIZE, 6);
}

#[test]
fn writable_covers_every_rw_field() {
    let w = Basic::CT_WRITABLE;
    assert_eq!(
        w,
        [
            (offset_of!(Basic, b) as u16, 2),
            (offset_of!(Basic, a) as u16, 1),
            (offset_of!(Basic, c) as u16, 1),
            (offset_of!(Basic, mode) as u16, 1),
            (offset_of!(Basic, d) as u16, 1),
        ]
    );
}

#[test]
fn ct_fields_carry_name_addr_width_writable_and_kind() {
    let f = Basic::CT_FIELDS;
    // Every kept field, in declaration order.
    assert_eq!(f.len(), 5);

    assert_eq!(f[0].name, "b");
    assert_eq!(f[0].addr, offset_of!(Basic, b) as u16);
    assert_eq!(f[0].width, 2);
    assert!(f[0].writable);
    assert_eq!(f[0].kind, FieldKind::UInt);

    assert_eq!(f[1].name, "a");
    assert_eq!(f[1].width, 1);
    assert_eq!(f[1].kind, FieldKind::UInt);

    assert_eq!(f[2].name, "c");
    assert_eq!(f[2].kind, FieldKind::Bool);

    assert_eq!(f[3].name, "mode");
    assert_eq!(f[3].addr, offset_of!(Basic, mode) as u16);
    assert_eq!(
        f[3].kind,
        FieldKind::Enum(&[
            EnumVariant {
                name: "Stop",
                value: 0
            },
            EnumVariant {
                name: "Go",
                value: 1
            },
        ])
    );

    assert_eq!(f[4].name, "d");
    // No bounds on any Basic field.
    assert!(f.iter().all(|d| d.min.is_none() && d.max.is_none()));
}

#[test]
fn ct_fields_int_kind_and_ro_flag() {
    // Access block: ro fields keep a descriptor but export writable = false.
    let f = Access::CT_FIELDS;
    let ro = f.iter().find(|d| d.name == "ro_field").unwrap();
    assert!(!ro.writable);
    assert_eq!(ro.kind, FieldKind::UInt);
    let rw = f.iter().find(|d| d.name == "rw_field").unwrap();
    assert!(rw.writable);
}

// Behavioral coverage of the generated `ct_check`: enum, an immediate compare, a
// signed abs compare, a cross-field register compare, and mask-driven ro
// rejection -- the write-outcome facts the old structural `CT_RULES` assertions
// encoded. `target < &checks::CEILING` is an in-table register reference (same
// shape the derive emits) so `read_fixed` stays in bounds.
#[repr(C)]
#[derive(Block)]
struct Checks {
    #[ct_field(le = 100u8)]
    ratio: u8,
    #[ct_field(ge = -5i8, le = 5i8, abs)]
    trim: i8,
    ceiling: u8,
    #[ct_field(lt = &bt::addr::checks::CEILING)]
    target: u8,
    flag: bool,
    #[ct_field(access = ro)]
    ro: u8,
}

mod bt {
    use super::Checks;
    use control_table::Section;

    #[repr(C)]
    #[derive(Section)]
    #[ct_section(base = 0, size = 6)]
    pub struct Sect {
        pub checks: Checks,
    }
}

#[repr(C)]
#[derive(Table)]
#[ct_table(size = 6)]
struct Bt {
    pub bt: bt::Sect,
}

#[test]
fn ct_check_write_outcomes() {
    use bt::addr::checks as a;

    let t = BtCell::new();
    // Seed the ceiling used by the register compare.
    t.write(a::CEILING, &[50]).unwrap();

    // enum: good then bad.
    assert!(t.write(a::FLAG, &[1]).is_ok());
    assert_eq!(
        t.write(a::FLAG, &[2]),
        Err(Error::ValidationError(ValidationKind::Enum))
    );

    // immediate compare: ratio <= 100.
    assert!(t.write(a::RATIO, &[100]).is_ok());
    assert_eq!(
        t.write(a::RATIO, &[101]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );

    // signed abs compare: |trim| <= 5. -5 (0xFB) passes, -6 (0xFA) fails.
    assert!(t.write(a::TRIM, &[0xFB]).is_ok());
    assert_eq!(
        t.write(a::TRIM, &[0xFA]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );

    // register compare: target < ceiling (50).
    assert!(t.write(a::TARGET, &[49]).is_ok());
    assert_eq!(
        t.write(a::TARGET, &[50]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );

    // ro field rejected by the writable mask.
    assert_eq!(t.write(a::RO, &[0]), Err(Error::AccessError));
}

#[test]
fn ct_fields_fold_immediate_bounds_only() {
    // ratio le=100 -> max Some(100); trim abs -> both None; target register
    // RHS -> both None.
    let f = Checks::CT_FIELDS;
    let ratio = f.iter().find(|d| d.name == "ratio").unwrap();
    assert_eq!(ratio.min, None);
    assert_eq!(ratio.max, Some(100));
    let trim = f.iter().find(|d| d.name == "trim").unwrap();
    assert_eq!((trim.min, trim.max), (None, None));
    let target = f.iter().find(|d| d.name == "target").unwrap();
    assert_eq!((target.min, target.max), (None, None));
}

#[repr(C)]
#[derive(Block)]
struct Exclusive {
    #[ct_field(gt = 3i16, lt = 10i16)]
    span: i16,
}

#[test]
fn ct_fields_exclusive_bounds_fold_inclusive() {
    // gt/lt shift by +/-1 into inclusive min/max.
    let f = Exclusive::CT_FIELDS;
    assert_eq!(f[0].name, "span");
    assert_eq!(f[0].kind, FieldKind::Int);
    assert_eq!(f[0].min, Some(4));
    assert_eq!(f[0].max, Some(9));
}

#[repr(C)]
#[derive(Block)]
struct Access {
    #[ct_field(access = ro)]
    ro_field: u16,
    rw_field: u16,
    #[ct_field(skip)]
    _rsvd: [u8; 2],
    #[ct_field(access = ro)]
    ro_bool: bool,
    tail: u8,
}

#[test]
fn ro_and_skip_excluded_from_writable_and_rules() {
    // ro (incl. ro bool) and skip fields never enter the writable mask.
    assert_eq!(
        Access::CT_WRITABLE,
        [
            (offset_of!(Access, rw_field) as u16, 2),
            (offset_of!(Access, tail) as u16, 1),
        ]
    );
    // skip field still counts toward layout size.
    assert_eq!(Access::CT_SIZE, size_of::<Access>() as u16);
    assert_eq!(Access::CT_SIZE, 8);
}

trait MyHooks {
    fn on_a(&mut self, v: u8);
}

struct HookRec {
    last: Option<u8>,
}

impl MyHooks for HookRec {
    fn on_a(&mut self, v: u8) {
        self.last = Some(v);
    }
}

#[repr(C)]
#[derive(Block)]
#[ct_block(hooks = MyHooks)]
struct Hooked {
    #[ct_field(hook = on_a)]
    pub a: u8,
    pub b: u8,
}

#[test]
fn hook_fires_only_when_write_window_overlaps_field() {
    let h = Hooked { a: 7, b: 0 };

    let mut hit = HookRec { last: None };
    h.dispatch_events(0, 1, 0, &mut hit);
    assert_eq!(hit.last, Some(7));

    // window on field `b` (offset 1) must not fire `a`'s hook.
    let mut miss = HookRec { last: None };
    h.dispatch_events(1, 1, 0, &mut miss);
    assert_eq!(miss.last, None);

    // block rebased by base: field `a` now lives at abs 10.
    let mut rebased = HookRec { last: None };
    h.dispatch_events(10, 1, 10, &mut rebased);
    assert_eq!(rebased.last, Some(7));
}

#[repr(C)]
#[derive(Block)]
struct Dense {
    w: u32,
    h: u16,
    a: u8,
    b: u8,
}

#[test]
fn zero_padding_assert_accepts_dense_struct() {
    // Compiling this struct at all proves the const assert passed.
    assert_eq!(Dense::CT_SIZE, size_of::<Dense>() as u16);
    assert_eq!(Dense::CT_SIZE, 8);
    let sum: u16 = Dense::CT_WRITABLE.iter().map(|(_, len)| len).sum();
    assert_eq!(sum, Dense::CT_SIZE);
}

// `new()` zero-initializes every field (enum/bool included) in const context.
const _: Basic = Basic::new();

#[test]
fn new_zero_initializes() {
    let v = Basic::new();
    assert_eq!(v.a, 0);
    assert_eq!(v.b, 0);
    assert!(!v.c);
    assert_eq!(v.d, 0);
}
