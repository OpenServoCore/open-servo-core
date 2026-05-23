use control_table::{Block, Enum, FieldValidator, HasAllowed};

#[derive(Copy, Clone, Enum)]
#[repr(u8)]
enum Mode {
    OpenLoop = 0,
    PositionPid = 1,
}

#[derive(Copy, Clone, Enum)]
#[repr(u8)]
enum DefaultNumbered {
    A,
    B,
    C,
}

#[derive(Copy, Clone, Enum)]
#[repr(u8)]
enum Single {
    Only = 7,
}

#[test]
fn explicit_discriminants_produce_allowed_in_source_order() {
    assert_eq!(Mode::ALLOWED, &[0u8, 1]);
}

#[test]
fn default_numbered_variants_produce_sequential_allowed() {
    assert_eq!(DefaultNumbered::ALLOWED, &[0u8, 1, 2]);
}

#[test]
fn single_variant_is_a_one_element_slice() {
    assert_eq!(Single::ALLOWED, &[7u8]);
}

#[repr(C)]
#[derive(Block)]
struct UsesEnum {
    mode: Mode,
}

#[test]
fn block_field_typed_as_enum_picks_up_allowed_default() {
    let v = UsesEnum::FIELDS[0].validators;
    assert_eq!(v.len(), 1);
    let FieldValidator::EnumU8 { allowed } = v[0] else {
        panic!("expected EnumU8")
    };
    assert_eq!(allowed, Mode::ALLOWED);
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum SparseHandRolled {
    Low = 1,
    High = 9,
}

impl HasAllowed for SparseHandRolled {
    const ALLOWED: &'static [u8] = &[Self::Low as u8, Self::High as u8];
}

#[repr(C)]
#[derive(Block)]
struct UsesHandRolled {
    x: SparseHandRolled,
}

#[test]
fn manual_has_allowed_impl_satisfies_block_default() {
    let v = UsesHandRolled::FIELDS[0].validators;
    assert_eq!(v.len(), 1);
    let FieldValidator::EnumU8 { allowed } = v[0] else {
        panic!("expected EnumU8")
    };
    assert_eq!(allowed, &[1u8, 9]);
}
