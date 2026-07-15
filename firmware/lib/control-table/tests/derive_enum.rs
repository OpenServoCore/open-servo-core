use control_table::Enum;
use control_table::HasAllowed;
use control_table::descriptor::EnumVariant;

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

#[test]
fn variants_pair_name_with_value() {
    assert_eq!(
        <Mode as HasAllowed>::VARIANTS,
        &[
            EnumVariant {
                name: "OpenLoop",
                value: 0
            },
            EnumVariant {
                name: "PositionPid",
                value: 1
            },
        ]
    );
    // Default-numbered discriminants pair with their implicit values.
    assert_eq!(
        <DefaultNumbered as HasAllowed>::VARIANTS,
        &[
            EnumVariant {
                name: "A",
                value: 0
            },
            EnumVariant {
                name: "B",
                value: 1
            },
            EnumVariant {
                name: "C",
                value: 2
            },
        ]
    );
}
