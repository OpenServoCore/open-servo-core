use control_table::Enum;

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
