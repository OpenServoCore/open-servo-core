use control_table::rules::{CmpOp, Rhs, RuleKind};
use control_table::{BOOL_ALLOWED, Enum, FlatBlock};
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
#[derive(FlatBlock)]
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
fn auto_enum_rules_for_bool_and_enum_fields_only() {
    let r = Basic::CT_RULES;
    assert_eq!(r.len(), 2);

    let bool_rule = &r[0];
    assert_eq!(bool_rule.offset, offset_of!(Basic, c) as u16);
    assert_eq!(bool_rule.width, 1);
    match bool_rule.kind {
        RuleKind::Enum { allowed } => assert_eq!(allowed, BOOL_ALLOWED),
        _ => panic!("expected bool enum rule"),
    }

    let mode_rule = &r[1];
    assert_eq!(mode_rule.offset, offset_of!(Basic, mode) as u16);
    assert_eq!(mode_rule.width, 1);
    match mode_rule.kind {
        RuleKind::Enum { allowed } => assert_eq!(allowed, Mode::ALLOWED),
        _ => panic!("expected mode enum rule"),
    }
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

const LIMIT: u16 = 500;

#[repr(C)]
#[derive(FlatBlock)]
struct Comp {
    #[ct_field(lt = &LIMIT)]
    y: u16,
    #[ct_field(le = 100u8)]
    x: u8,
    #[ct_field(ge = -5i8, le = 5i8, abs)]
    z: i8,
}

#[test]
fn cmp_rules_infer_width_signed_and_dispatch_rhs() {
    let r = Comp::CT_RULES;
    assert_eq!(r.len(), 4);

    // y: unsigned u16, reference rhs -> Reg.
    assert_eq!(r[0].offset, offset_of!(Comp, y) as u16);
    assert_eq!(r[0].width, 2);
    match r[0].kind {
        RuleKind::Cmp {
            op: CmpOp::Lt,
            rhs: Rhs::Reg(v),
            signed: false,
            abs: false,
        } => assert_eq!(v, LIMIT),
        _ => panic!("wrong y rule"),
    }

    // x: unsigned u8, literal rhs -> Imm.
    assert_eq!(r[1].offset, offset_of!(Comp, x) as u16);
    assert_eq!(r[1].width, 1);
    match r[1].kind {
        RuleKind::Cmp {
            op: CmpOp::Le,
            rhs: Rhs::Imm(100),
            signed: false,
            abs: false,
        } => {}
        _ => panic!("wrong x rule"),
    }

    // z: signed i8, two compares sharing abs, negative literal.
    assert_eq!(r[2].offset, offset_of!(Comp, z) as u16);
    assert_eq!(r[2].width, 1);
    match r[2].kind {
        RuleKind::Cmp {
            op: CmpOp::Ge,
            rhs: Rhs::Imm(-5),
            signed: true,
            abs: true,
        } => {}
        _ => panic!("wrong z ge rule"),
    }
    match r[3].kind {
        RuleKind::Cmp {
            op: CmpOp::Le,
            rhs: Rhs::Imm(5),
            signed: true,
            abs: true,
        } => {}
        _ => panic!("wrong z le rule"),
    }
}

#[repr(C)]
#[derive(FlatBlock)]
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
    // ro (incl. ro bool) and skip fields emit no rules at all.
    assert_eq!(Access::CT_RULES.len(), 0);

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
#[derive(FlatBlock)]
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
#[derive(FlatBlock)]
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
