use super::*;
use crate::ValidationKind;
use crate::rules::{CmpOp, Rhs, Rule, RuleKind};
use core::cell::UnsafeCell;

// --- Basic fixture: rules + a read-only byte, no lock. ---

const BASIC_SIZE: usize = 16;

static BASIC_RULES: &[Rule] = &[
    Rule {
        offset: 0,
        width: 1,
        kind: RuleKind::Enum { allowed: &[0, 1] },
    },
    Rule {
        offset: 4,
        width: 2,
        kind: RuleKind::Cmp {
            op: CmpOp::Ge,
            rhs: Rhs::Imm(0),
            signed: true,
            abs: false,
        },
    },
    Rule {
        offset: 6,
        width: 2,
        kind: RuleKind::Cmp {
            op: CmpOp::Lt,
            rhs: Rhs::Reg(8),
            signed: false,
            abs: false,
        },
    },
    Rule {
        offset: 10,
        width: 1,
        kind: RuleKind::Cmp {
            op: CmpOp::Le,
            rhs: Rhs::Imm(5),
            signed: true,
            abs: true,
        },
    },
    Rule {
        offset: 12,
        width: 4,
        kind: RuleKind::Cmp {
            op: CmpOp::Le,
            rhs: Rhs::Imm(100_000),
            signed: true,
            abs: false,
        },
    },
];

static BASIC_SECTIONS: &[SectionMeta] = &[SectionMeta {
    base: 0,
    size: BASIC_SIZE as u16,
    rules: BASIC_RULES,
    write_lock: None,
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

// --- Lock fixture: two sections, section B guarded by a byte in section A. ---

const LOCK_SIZE: usize = 16;

static LOCK_B_RULES: &[Rule] = &[Rule {
    offset: 8,
    width: 1,
    kind: RuleKind::Enum { allowed: &[0, 1] },
}];

static LOCK_SECTIONS: &[SectionMeta] = &[
    SectionMeta {
        base: 0,
        size: 8,
        rules: &[],
        write_lock: None,
    },
    SectionMeta {
        base: 8,
        size: 8,
        rules: LOCK_B_RULES,
        write_lock: Some(0),
    },
];

const LOCK_WRITABLE: &[u32] = &[0xFFFF_FFFF];

struct Locked {
    store: UnsafeCell<[u8; LOCK_SIZE]>,
}

// SAFETY: tests are single-threaded.
unsafe impl Sync for Locked {}

impl Locked {
    fn new(init: [u8; LOCK_SIZE]) -> Self {
        Self {
            store: UnsafeCell::new(init),
        }
    }
}

impl RegisterMap for Locked {
    const SIZE: usize = LOCK_SIZE;
    const WRITABLE: &'static [u32] = LOCK_WRITABLE;
    const SECTIONS: &'static [SectionMeta] = LOCK_SECTIONS;
    fn base(&self) -> *mut u8 {
        self.store.get() as *mut u8
    }
}

#[test]
fn lock_permits_when_clear() {
    let m = Locked::new([0; LOCK_SIZE]);
    assert!(m.write(8, &[1]).is_ok());
}

#[test]
fn lock_blocks_when_set() {
    let mut init = [0u8; LOCK_SIZE];
    init[0] = 1;
    let m = Locked::new(init);
    assert_eq!(
        m.write(8, &[1]),
        Err(Error::ValidationError(ValidationKind::Locked))
    );
}

#[test]
fn lock_byte_carried_unlocks_in_same_write() {
    let mut init = [0u8; LOCK_SIZE];
    init[0] = 1; // live: locked
    let m = Locked::new(init);
    // Spanning write [0..9): sets byte0 = 0 (unlock) and byte8 = 1. The lock is
    // read through the overlay, so it sees 0 and permits.
    assert!(m.write(0, &[0, 0, 0, 0, 0, 0, 0, 0, 1]).is_ok());
}

#[test]
fn lock_byte_carried_locks_in_same_write() {
    let m = Locked::new([0; LOCK_SIZE]); // live: unlocked
    // Spanning write sets byte0 = 1 in the overlay -> section B write blocked.
    assert_eq!(
        m.write(0, &[1, 0, 0, 0, 0, 0, 0, 0, 1]),
        Err(Error::ValidationError(ValidationKind::Locked))
    );
}

#[test]
fn rules_run_before_lock() {
    let mut init = [0u8; LOCK_SIZE];
    init[0] = 1; // locked
    let m = Locked::new(init);
    // Bad enum into the locked section: reports Enum, not Locked.
    assert_eq!(
        m.write(8, &[5]),
        Err(Error::ValidationError(ValidationKind::Enum))
    );
}

// --- Mask fixture: 3 words (96 B), no rules/lock, so only the writable-mask
// scan runs. Two read-only holes: byte 50 (mid word 1) and byte 64 (start of
// word 2, a word boundary). Ranges are positioned so a hole falls just inside
// vs just outside each edge shape. ---

const MASK_SIZE: usize = 96;

// word0 all writable; word1 hole at bit 18 (byte 50); word2 hole at bit 0 (byte 64).
const MASK_WRITABLE: &[u32] = &[0xFFFF_FFFF, !(1u32 << 18), !(1u32 << 0)];

static MASK_SECTIONS: &[SectionMeta] = &[SectionMeta {
    base: 0,
    size: MASK_SIZE as u16,
    rules: &[],
    write_lock: None,
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
