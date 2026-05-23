use control_table::{
    Access, BOOL_ALLOWED, Block, CompareOp, Enum, FieldValidator, RegmapError, Rhs, StagedView,
};
use core::mem::{offset_of, size_of};

#[repr(C)]
#[derive(Block)]
struct Basic {
    a: u8,
    b: u16,
    c: bool,
}

#[test]
fn at_zero_addrs_track_offset_of_each_field() {
    assert_eq!(Basic::SIZE, size_of::<Basic>() as u16);
    assert_eq!(Basic::FIELD_COUNT, 3);
    let f = Basic::FIELDS_AT_ZERO;
    assert_eq!(f[0].addr, offset_of!(Basic, a) as u16);
    assert_eq!(f[0].size, 1);
    assert_eq!(f[0].struct_offset, f[0].addr);
    assert_eq!(f[1].addr, offset_of!(Basic, b) as u16);
    assert_eq!(f[1].size, 2);
    assert_eq!(f[2].addr, offset_of!(Basic, c) as u16);
    assert_eq!(f[2].size, 1);
}

#[test]
fn bool_field_gets_default_enum_u8_bool_allowed() {
    let v = Basic::FIELDS_AT_ZERO[2].validators;
    assert_eq!(v.len(), 1);
    let FieldValidator::EnumU8 { allowed } = v[0] else {
        panic!("expected EnumU8")
    };
    assert_eq!(allowed, BOOL_ALLOWED);
}

#[test]
fn primitive_int_fields_get_no_default_validator() {
    assert!(Basic::FIELDS_AT_ZERO[0].validators.is_empty());
    assert!(Basic::FIELDS_AT_ZERO[1].validators.is_empty());
}

#[test]
fn default_block_validators_empty() {
    assert!(Basic::VALIDATORS.is_empty());
}

#[repr(C)]
#[derive(Block)]
struct WithPad {
    a: u8,
    #[ct_field(skip)]
    _pad: [u8; 3],
    b: u32,
}

#[test]
fn skip_excludes_field_but_preserves_struct_size() {
    assert_eq!(WithPad::FIELD_COUNT, 2);
    assert_eq!(WithPad::SIZE, size_of::<WithPad>() as u16);
    let f = WithPad::FIELDS_AT_ZERO;
    assert_eq!(f[0].addr, 0);
    assert_eq!(f[1].addr, 4);
    assert_eq!(f[1].size, 4);
}

#[repr(C)]
#[derive(Block)]
struct RoRw {
    #[ct_field(access = ro)]
    a: u8,
    b: u8,
}

#[test]
fn access_attr_overrides_default_rw() {
    assert_eq!(RoRw::FIELDS_AT_ZERO[0].access, Access::Ro);
    assert_eq!(RoRw::FIELDS_AT_ZERO[1].access, Access::Rw);
}

#[repr(C)]
#[derive(Block)]
struct CompLit {
    #[ct_field(le = 100u8)]
    x: u8,
}

#[test]
fn literal_rhs_dispatches_to_value_variant() {
    match CompLit::FIELDS_AT_ZERO[0].validators[0] {
        FieldValidator::CompareU8 {
            op: CompareOp::Le,
            abs: false,
            rhs: Rhs::Value(100),
        } => {}
        _ => panic!("wrong validator shape"),
    }
}

const OTHER_ADDR: u16 = 42;

#[repr(C)]
#[derive(Block)]
struct CompAddr {
    #[ct_field(lt = &OTHER_ADDR)]
    x: i32,
}

#[test]
fn reference_rhs_dispatches_to_addr_variant() {
    match CompAddr::FIELDS_AT_ZERO[0].validators[0] {
        FieldValidator::CompareI32 {
            op: CompareOp::Lt,
            abs: false,
            rhs: Rhs::Addr(42),
        } => {}
        _ => panic!("wrong validator shape"),
    }
}

#[repr(C)]
#[derive(Block)]
struct AbsMulti {
    #[ct_field(ge = 0i32, le = 100i32, abs)]
    x: i32,
}

#[test]
fn multiple_compare_ops_share_abs_and_field() {
    let v = AbsMulti::FIELDS_AT_ZERO[0].validators;
    assert_eq!(v.len(), 2);
    let mut ge_ok = false;
    let mut le_ok = false;
    for val in v {
        match *val {
            FieldValidator::CompareI32 {
                op: CompareOp::Ge,
                abs: true,
                rhs: Rhs::Value(0),
            } => ge_ok = true,
            FieldValidator::CompareI32 {
                op: CompareOp::Le,
                abs: true,
                rhs: Rhs::Value(100),
            } => le_ok = true,
            _ => panic!("unexpected variant"),
        }
    }
    assert!(ge_ok && le_ok);
}

fn my_custom(_v: &StagedView, _a: u16, _s: u16) -> Result<(), RegmapError> {
    Ok(())
}

#[repr(C)]
#[derive(Block)]
struct WithCustom {
    #[ct_field(custom = my_custom)]
    x: u8,
}

#[test]
fn custom_attr_emits_custom_validator() {
    let v = WithCustom::FIELDS_AT_ZERO[0].validators;
    assert_eq!(v.len(), 1);
    assert!(matches!(v[0], FieldValidator::Custom(_)));
}

const SMALL: &[u8] = &[7, 8];

#[repr(C)]
#[derive(Block)]
struct AllowOverride {
    #[ct_field(allowed = SMALL)]
    x: u8,
}

#[test]
fn allowed_overrides_default_with_provided_set() {
    let v = AllowOverride::FIELDS_AT_ZERO[0].validators;
    assert_eq!(v.len(), 1);
    let FieldValidator::EnumU8 { allowed } = v[0] else {
        panic!("expected EnumU8")
    };
    assert_eq!(allowed, SMALL);
}

#[repr(u8)]
#[derive(Copy, Clone, Enum)]
enum FakeMode {
    Stop = 0,
    Go = 1,
}

#[repr(C)]
#[derive(Block)]
struct WithMode {
    mode: FakeMode,
}

#[test]
fn user_type_field_gets_ty_allowed_default() {
    let v = WithMode::FIELDS_AT_ZERO[0].validators;
    assert_eq!(v.len(), 1);
    let FieldValidator::EnumU8 { allowed } = v[0] else {
        panic!("expected EnumU8")
    };
    assert_eq!(allowed, FakeMode::ALLOWED);
}

fn block_check(_v: &StagedView, _a: u16, _s: u16) -> Result<(), RegmapError> {
    Ok(())
}

#[repr(C)]
#[derive(Block)]
#[ct_block(validators = [block_check])]
struct WithBlockVal {
    x: u8,
}

#[test]
fn ct_block_validators_list_populates_validators_const() {
    assert_eq!(WithBlockVal::VALIDATORS.len(), 1);
}

#[test]
fn fields_at_zero_slice_points_to_arr_with_same_layout() {
    assert_eq!(Basic::FIELDS_AT_ZERO.len(), Basic::FIELDS_AT_ZERO_ARR.len());
    for (slice_f, arr_f) in Basic::FIELDS_AT_ZERO
        .iter()
        .zip(Basic::FIELDS_AT_ZERO_ARR.iter())
    {
        assert_eq!(slice_f.addr, arr_f.addr);
        assert_eq!(slice_f.size, arr_f.size);
    }
}

#[repr(C)]
#[derive(Block)]
struct MetaTarget {
    alpha: u8,
    #[ct_field(skip)]
    _skip: u8,
    bravo: u16,
}

mod hub_via_meta_macro {
    // Drives the Block-derive-emitted __ct_meta_MetaTarget macro the same way
    // Region derive will, producing an `addr::*` style submodule of consts.
    __ct_meta_MetaTarget!(@addr_consts base = 100u16, block_ty = super::MetaTarget);
}

#[test]
fn meta_macro_emits_named_consts_for_kept_fields() {
    assert_eq!(
        hub_via_meta_macro::ALPHA,
        100 + offset_of!(MetaTarget, alpha) as u16
    );
    assert_eq!(
        hub_via_meta_macro::BRAVO,
        100 + offset_of!(MetaTarget, bravo) as u16
    );
}
