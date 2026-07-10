use super::*;
use crate::ValidationKind;
use core::cell::UnsafeCell;

// --- Basic fixture: checks + a read-only byte, no lock. Hand-written to mirror
// what the derive emits: enum [0,1]@0, i16 >= 0 @4, u16 < reg(8) @6, i8 |x| <= 5
// @10, i32 <= 100_000 @12 — each overlap-guarded against the pending range. ---

const BASIC_SIZE: usize = 16;

// The same five rules the old `basic_check` encoded, now as flash descriptors:
// enum [0,1]@0, i16 >= 0 @4, u16 < reg(8) @6, i8 |x| <= 5 @10, i32 <= 100_000 @12.
const BASIC_ALLOWED_RULES: &[crate::rules::AllowedRule] = &[crate::rules::AllowedRule {
    addr: 0,
    allowed: &[0, 1],
}];
const BASIC_CMP_RULES: &[crate::rules::CmpRule] = &[
    crate::rules::CmpRule {
        addr: 4,
        spec: crate::rules::spec(2, true, false, crate::rules::OP_GE),
        val: 0,
    },
    crate::rules::CmpRule {
        addr: 6,
        spec: crate::rules::spec(2, false, false, crate::rules::OP_LT) | crate::rules::SPEC_RHS_REG,
        val: 8,
    },
    crate::rules::CmpRule {
        addr: 10,
        spec: crate::rules::spec(1, true, true, crate::rules::OP_LE),
        val: 5,
    },
    crate::rules::CmpRule {
        addr: 12,
        spec: crate::rules::spec(4, true, false, crate::rules::OP_LE),
        val: 100_000,
    },
];

static BASIC_SECTIONS: &[SectionMeta] = &[SectionMeta {
    base: 0,
    size: BASIC_SIZE as u16,
    cmp_rules: (0, 4),
    allowed_rules: (0, 1),
}];

// Every byte writable except byte 3 (read-only, mid-range).
const BASIC_WRITABLE: &[u32] = &[!(1u32 << 3)];

struct Basic {
    store: UnsafeCell<[u8; BASIC_SIZE]>,
}

// SAFETY: tests are single-threaded.
unsafe impl Sync for Basic {}

impl Basic {
    fn new(init: [u8; BASIC_SIZE]) -> Self {
        Self {
            store: UnsafeCell::new(init),
        }
    }
}

impl RegisterMap for Basic {
    const SIZE: usize = BASIC_SIZE;
    const WRITABLE: &'static [u32] = BASIC_WRITABLE;
    const SECTIONS: &'static [SectionMeta] = BASIC_SECTIONS;
    const CMP_RULES: &'static [crate::rules::CmpRule] = BASIC_CMP_RULES;
    const ALLOWED_RULES: &'static [crate::rules::AllowedRule] = BASIC_ALLOWED_RULES;
    fn base(&self) -> *mut u8 {
        self.store.get() as *mut u8
    }
}

#[test]
fn read_bounds() {
    let m = Basic::new([0; BASIC_SIZE]);
    assert_eq!(m.read(14, 3), Err(Error::OutOfRange));
    assert_eq!(m.read(16, 1), Err(Error::OutOfRange));
    assert!(m.read(0, 16).is_ok());
    assert_eq!(m.read(4, 0), Ok(&[][..]));
}

#[test]
fn write_bounds() {
    let m = Basic::new([0; BASIC_SIZE]);
    assert_eq!(m.write(14, &[0, 0, 0]), Err(Error::OutOfRange));
    assert_eq!(m.write(16, &[0]), Err(Error::OutOfRange));
}

#[test]
fn read_returns_live_bytes() {
    let mut init = [0u8; BASIC_SIZE];
    init[3] = 0xAB;
    init[9] = 0xCD;
    let m = Basic::new(init);
    assert_eq!(m.read(3, 1), Ok(&[0xAB][..]));
    assert_eq!(m.read(9, 1), Ok(&[0xCD][..]));
}

#[test]
fn mask_rejects_readonly_byte() {
    let m = Basic::new([0; BASIC_SIZE]);
    // Byte 3 is RO: a write spanning it fails even if the rest is fine.
    assert_eq!(m.write(2, &[0, 0, 0]), Err(Error::AccessError));
    // A write that avoids byte 3 succeeds.
    assert!(m.write(4, &[0, 0]).is_ok());
}

#[test]
fn enum_rule() {
    let m = Basic::new([0; BASIC_SIZE]);
    assert!(m.write(0, &[1]).is_ok());
    assert_eq!(
        m.write(0, &[2]),
        Err(Error::ValidationError(ValidationKind::Enum))
    );
}

#[test]
fn cmp_imm_signed_i16() {
    let m = Basic::new([0; BASIC_SIZE]);
    // -1 as i16 LE = 0xFF 0xFF, must be >= 0 -> reject.
    assert_eq!(
        m.write(4, &[0xFF, 0xFF]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
    // +1 passes.
    assert!(m.write(4, &[1, 0]).is_ok());
}

#[test]
fn cmp_reg_cross_field_u16() {
    // field6 < field8 (both u16). Seed field8 = 100.
    let mut init = [0u8; BASIC_SIZE];
    init[8] = 100;
    let m = Basic::new(init);
    assert!(m.write(6, &[50, 0]).is_ok());
    assert_eq!(
        m.write(6, &[200, 0]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
}

#[test]
fn cmp_abs_i8() {
    let m = Basic::new([0; BASIC_SIZE]);
    // |x| <= 5. -5 (0xFB) passes, -6 (0xFA) fails.
    assert!(m.write(10, &[0xFB]).is_ok());
    assert_eq!(
        m.write(10, &[0xFA]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
}

#[test]
fn cmp_signed_i32_width4() {
    let m = Basic::new([0; BASIC_SIZE]);
    // <= 100_000. 50_000 passes.
    assert!(m.write(12, &50_000i32.to_le_bytes()).is_ok());
    // 200_000 fails.
    assert_eq!(
        m.write(12, &200_000i32.to_le_bytes()),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
    // A negative value passes (signed read).
    assert!(m.write(12, &(-1i32).to_le_bytes()).is_ok());
}

#[test]
fn partial_field_overwrite_revalidates_whole_field() {
    // Seed field4 (i16) = 0x0001 (valid, >= 0). Overwrite only the high byte
    // with 0xFF -> field becomes 0xFF01 = -255, which must re-validate and fail.
    let mut init = [0u8; BASIC_SIZE];
    init[4] = 0x01;
    init[5] = 0x00;
    let m = Basic::new(init);
    assert_eq!(
        m.write(5, &[0xFF]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
}

#[test]
fn straddle_low_pending_high_live() {
    // field12 (i32, <= 100_000): low bytes pending, high bytes live. Live high
    // byte 14 = 0x01 contributes 0x00010000 = 65536, so the rule must see both
    // the overlay low half and the live high half to reach the right verdict.
    let mut init = [0u8; BASIC_SIZE];
    init[14] = 0x01;
    // overlay low = 0x86A0 -> total 100_000, passes.
    let m = Basic::new(init);
    assert!(m.write(12, &[0xA0, 0x86]).is_ok());
    // overlay low = 0x86A1 -> total 100_001, fails (proves live-high seen too).
    let m = Basic::new(init);
    assert_eq!(
        m.write(12, &[0xA1, 0x86]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
}

#[test]
fn straddle_high_pending_low_live() {
    // field12 (i32, <= 100_000): high bytes pending, low bytes live. Live low
    // half = 0x86A0 = 34464.
    let mut init = [0u8; BASIC_SIZE];
    init[12] = 0xA0;
    init[13] = 0x86;
    // overlay high = 0x0001 -> total 100_000, passes.
    let m = Basic::new(init);
    assert!(m.write(14, &[0x01, 0x00]).is_ok());
    // overlay high = 0x0002 -> total 165_536, fails (proves live-low seen too).
    let m = Basic::new(init);
    assert_eq!(
        m.write(14, &[0x02, 0x00]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
}

#[test]
fn read_fixed_picks_per_path() {
    // Live 0..8 with the overlay covering 2..6: o1 at 2..4, o2 at 4..6. Spans
    // chosen to hit each read_fixed path: fully live (both sides), fully
    // inside o1, inside o2, and every straddle shape.
    let live: [u8; 8] = [10, 11, 12, 13, 14, 15, 16, 17];
    let o1 = [0xA0, 0xA1];
    let o2 = [0xB0, 0xB1];
    let v = View::new_split(live.as_ptr(), live.len(), 2, &o1, &o2);
    assert_eq!(v.read_fixed(0, 2), Ok([10, 11, 0, 0]));
    assert_eq!(v.read_fixed(6, 2), Ok([16, 17, 0, 0]));
    assert_eq!(v.read_fixed(2, 2), Ok([0xA0, 0xA1, 0, 0]));
    assert_eq!(v.read_fixed(4, 2), Ok([0xB0, 0xB1, 0, 0]));
    assert_eq!(v.read_fixed(1, 4), Ok([11, 0xA0, 0xA1, 0xB0]));
    assert_eq!(v.read_fixed(3, 2), Ok([0xA1, 0xB0, 0, 0]));
    assert_eq!(v.read_fixed(5, 3), Ok([0xB1, 16, 17, 0]));
    assert_eq!(v.read_fixed(0, 1), Ok([10, 0, 0, 0]));
    assert_eq!(v.read_fixed(0, 0), Ok([0, 0, 0, 0]));
    assert_eq!(v.read_fixed(6, 4), Err(Error::OutOfRange));
    // Empty overlay degenerates to all-live.
    let v = View::new_split(live.as_ptr(), live.len(), 3, &[], &[]);
    assert_eq!(v.read_fixed(2, 4), Ok([12, 13, 14, 15]));
}

#[test]
fn stage_validates_then_pushes() {
    let m = Basic::new([0; BASIC_SIZE]);
    let mut staged = StagedWrites::new();
    // Bad enum: rejected, nothing staged.
    assert_eq!(
        m.stage(0, &[2], &mut staged),
        Err(Error::ValidationError(ValidationKind::Enum))
    );
    assert!(staged.is_empty());
    // Good: staged.
    assert!(m.stage(0, &[1], &mut staged).is_ok());
    assert!(!staged.is_empty());
    // Not committed to live storage yet.
    assert_eq!(m.read(0, 1), Ok(&[0][..]));
}

#[test]
fn commit_staged_order_later_wins_no_revalidation() {
    let m = Basic::new([0; BASIC_SIZE]);
    let mut staged = StagedWrites::new();
    // Push directly (bypassing validate) an out-of-enum value: commit must not
    // re-validate it, and the later push at the same addr wins.
    staged.push(0, &[9]).unwrap();
    staged.push(0, &[1]).unwrap();
    staged.push(4, &[0xFF, 0xFF]).unwrap(); // would fail the i16 rule if checked
    m.commit_staged(&mut staged);
    assert_eq!(m.read(0, 1), Ok(&[1][..]));
    assert_eq!(m.read(4, 2), Ok(&[0xFF, 0xFF][..]));
    assert!(staged.is_empty());
}

// --- Mask fixture: 3 words (96 B), no rules, so only the writable-mask
// scan runs. Two read-only holes: byte 50 (mid word 1) and byte 64 (start of
// word 2, a word boundary). Ranges are positioned so a hole falls just inside
// vs just outside each edge shape. ---

const MASK_SIZE: usize = 96;

// word0 all writable; word1 hole at bit 18 (byte 50); word2 hole at bit 0 (byte 64).
const MASK_WRITABLE: &[u32] = &[0xFFFF_FFFF, !(1u32 << 18), !(1u32 << 0)];

static MASK_SECTIONS: &[SectionMeta] = &[SectionMeta {
    base: 0,
    size: MASK_SIZE as u16,
    cmp_rules: (0, 0),
    allowed_rules: (0, 0),
}];

struct MaskFix {
    store: UnsafeCell<[u8; MASK_SIZE]>,
}

// SAFETY: tests are single-threaded.
unsafe impl Sync for MaskFix {}

impl MaskFix {
    fn new() -> Self {
        Self {
            store: UnsafeCell::new([0; MASK_SIZE]),
        }
    }
}

impl RegisterMap for MaskFix {
    const SIZE: usize = MASK_SIZE;
    const WRITABLE: &'static [u32] = MASK_WRITABLE;
    const SECTIONS: &'static [SectionMeta] = MASK_SECTIONS;
    fn base(&self) -> *mut u8 {
        self.store.get() as *mut u8
    }
}

#[test]
fn mask_word_at_a_time_edges() {
    let z = [0u8; MASK_SIZE];
    let m = MaskFix::new();

    // start mid-word: [51,56) excludes hole@50 -> ok; [50,56) includes -> Access.
    assert!(m.write(51, &z[..5]).is_ok());
    assert_eq!(m.write(50, &z[..6]), Err(Error::AccessError));

    // end mid-word: [46,50) excludes hole@50 -> ok; [46,51) includes -> Access.
    assert!(m.write(46, &z[..4]).is_ok());
    assert_eq!(m.write(46, &z[..5]), Err(Error::AccessError));

    // span exactly one word: [0,32) no hole -> ok; [32,64) has hole@50 -> Access.
    assert!(m.write(0, &z[..32]).is_ok());
    assert_eq!(m.write(32, &z[..32]), Err(Error::AccessError));

    // span multiple words: [10,48) excludes hole@50 -> ok; [10,51) includes -> Access.
    assert!(m.write(10, &z[..38]).is_ok());
    assert_eq!(m.write(10, &z[..41]), Err(Error::AccessError));

    // 1-byte range at a word boundary: [63,64) excludes hole@64 -> ok;
    // [64,65) is the boundary hole -> Access.
    assert!(m.write(63, &z[..1]).is_ok());
    assert_eq!(m.write(64, &z[..1]), Err(Error::AccessError));
}

// --- Split (ring-seam) write/stage: the source arrives as head + tail, the
// split landing MID a multi-byte field. Behaviour must match the equivalent
// contiguous write. Field12 (i32 <= 100_000) spans bytes 12..16 on Basic. ---

#[test]
fn write_split_mid_field_accepts_valid_value() {
    let m = Basic::new([0; BASIC_SIZE]);
    // 50_000 across bytes 12..16, split after byte 13 (mid-field).
    let v = 50_000i32.to_le_bytes();
    assert!(m.write_split(12, &v[..2], &v[2..]).is_ok());
    assert_eq!(m.read(12, 4), Ok(&v[..]));
}

#[test]
fn write_split_mid_field_rejects_and_leaves_table_untouched() {
    let m = Basic::new([0; BASIC_SIZE]);
    // 200_000 fails the <= 100_000 rule; the split must not mutate the table.
    let v = 200_000i32.to_le_bytes();
    assert_eq!(
        m.write_split(12, &v[..2], &v[2..]),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
    assert_eq!(m.read(12, 4), Ok(&[0, 0, 0, 0][..]));
}

#[test]
fn write_split_matches_contiguous_at_every_split_point() {
    let v = 50_000i32.to_le_bytes();
    for k in 0..=v.len() {
        let m = Basic::new([0; BASIC_SIZE]);
        assert!(m.write_split(12, &v[..k], &v[k..]).is_ok());
        assert_eq!(m.read(12, 4), Ok(&v[..]));
    }
}

#[test]
fn stage_split_validates_then_pushes_contiguously() {
    let m = Basic::new([0; BASIC_SIZE]);
    let mut staged = StagedWrites::new();
    let v = 50_000i32.to_le_bytes();
    // Bad value rejected, nothing staged.
    let bad = 200_000i32.to_le_bytes();
    assert_eq!(
        m.stage_split(12, &bad[..2], &bad[2..], &mut staged),
        Err(Error::ValidationError(ValidationKind::Compare))
    );
    assert!(staged.is_empty());
    // Good split stages one contiguous 4-byte entry; commit lands it.
    assert!(m.stage_split(12, &v[..1], &v[1..], &mut staged).is_ok());
    let mut count = 0;
    for (a, d) in staged.iter_from(&crate::stage::Snapshot::ZERO) {
        assert_eq!(a, 12);
        assert_eq!(d, &v[..]);
        count += 1;
    }
    assert_eq!(count, 1);
    m.commit_staged(&mut staged);
    assert_eq!(m.read(12, 4), Ok(&v[..]));
}
