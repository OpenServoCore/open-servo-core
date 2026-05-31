use crate::route::stage_write;
use crate::validate::run_field_validators;
use crate::*;
use core::cell::UnsafeCell;

const REGION_SIZE: u16 = 32;

fn always_reject(_view: &StagedView, _addr: u16, _size: u16) -> Result<(), Error> {
    Err(Error::ValidationError(ValidationKind::Custom))
}

fn block_reject(_view: &StagedView, _addr: u16, _size: u16) -> Result<(), Error> {
    Err(Error::ValidationError(ValidationKind::Locked))
}

const BLOCK_VAL_BLOCKS: &[BlockDesc] = &[BlockDesc {
    addr: 0,
    size: 2,
    struct_offset: 0,
    fields: &[FieldDesc {
        addr: 0,
        size: 2,
        struct_offset: 0,
        access: Access::Rw,
        validators: &[],
    }],
    validators: &[block_reject],
}];

static BLOCK_VAL_REGION: RegionDesc = RegionDesc {
    addr: 0,
    size: REGION_SIZE,
    blocks: BLOCK_VAL_BLOCKS,
    validators: &[],
};

static BLOCK_VAL_REGIONS: &[&RegionDesc] = &[&BLOCK_VAL_REGION];

struct BlockValRouter {
    storage: UnsafeCell<[u8; REGION_SIZE as usize]>,
}

// SAFETY: tests are single-threaded.
unsafe impl Sync for BlockValRouter {}

impl BlockValRouter {
    const fn new() -> Self {
        Self {
            storage: UnsafeCell::new([0; REGION_SIZE as usize]),
        }
    }
}

impl Router for BlockValRouter {
    fn regions(&self) -> &'static [&'static RegionDesc] {
        BLOCK_VAL_REGIONS
    }
    fn region_base(&self, _desc: &RegionDesc) -> Option<*mut u8> {
        Some(self.storage.get() as *mut u8)
    }
}

const REJECTING_BLOCKS: &[BlockDesc] = &[BlockDesc {
    addr: 0,
    size: 1,
    struct_offset: 0,
    fields: &[FieldDesc {
        addr: 0,
        size: 1,
        struct_offset: 0,
        access: Access::Rw,
        validators: &[FieldValidator::Custom(always_reject)],
    }],
    validators: &[],
}];

// Two blocks: RW block [0..4), gap [4..8), RW block [8..16) with first byte RO.
const ROUTING_BLOCKS: &[BlockDesc] = &[
    BlockDesc {
        addr: 0,
        size: 4,
        struct_offset: 0,
        fields: &[FieldDesc {
            addr: 0,
            size: 4,
            struct_offset: 0,
            access: Access::Rw,
            validators: &[],
        }],
        validators: &[],
    },
    BlockDesc {
        addr: 8,
        size: 8,
        struct_offset: 8,
        fields: &[
            FieldDesc {
                addr: 8,
                size: 1,
                struct_offset: 0,
                access: Access::Ro,
                validators: &[],
            },
            FieldDesc {
                addr: 9,
                size: 7,
                struct_offset: 1,
                access: Access::Rw,
                validators: &[],
            },
        ],
        validators: &[],
    },
];

static ROUTING_REGION: RegionDesc = RegionDesc {
    addr: 0,
    size: REGION_SIZE,
    blocks: ROUTING_BLOCKS,
    validators: &[],
};

static ROUTING_REGIONS: &[&RegionDesc] = &[&ROUTING_REGION];

struct StubRouter {
    storage: UnsafeCell<[u8; REGION_SIZE as usize]>,
}

// SAFETY: tests are single-threaded; the StubRouter is used as a fake
// single-writer region for routing/staging checks.
unsafe impl Sync for StubRouter {}

impl StubRouter {
    const fn new() -> Self {
        Self {
            storage: UnsafeCell::new([0; REGION_SIZE as usize]),
        }
    }
}

impl Router for StubRouter {
    fn regions(&self) -> &'static [&'static RegionDesc] {
        ROUTING_REGIONS
    }
    fn region_base(&self, _desc: &RegionDesc) -> Option<*mut u8> {
        Some(self.storage.get() as *mut u8)
    }
}

#[test]
fn walk_fields_rejects_writes_into_padding_gap() {
    let r = StubRouter::new();
    let mut staged = StagedWrites::new();
    let err = r.write_bytes(4, &[0xAA], &mut staged).unwrap_err();
    assert_eq!(err, Error::AccessError);
    assert!(staged.is_empty());
}

#[test]
fn walk_fields_rejects_writes_to_ro_field() {
    let r = StubRouter::new();
    let mut staged = StagedWrites::new();
    let err = r.write_bytes(8, &[0xAA], &mut staged).unwrap_err();
    assert_eq!(err, Error::AccessError);
}

#[test]
fn walk_fields_allows_reads_through_ro_and_rw() {
    let r = StubRouter::new();
    let mut buf = [0xFFu8; 8];
    r.read_bytes(8, &mut buf).unwrap();
    assert_eq!(buf, [0u8; 8]);
}

#[test]
fn stage_then_validator_reject_rewinds_buffer_and_does_not_commit() {
    let r = StubRouter::new();
    let mut staged = StagedWrites::new();
    let saved_data = staged.data.len();
    let saved_entries = staged.entries.len();
    stage_write(0, &[0xAA], REJECTING_BLOCKS, &mut staged).unwrap();
    let reject = run_field_validators(&r, &staged, saved_entries, 0, 1, REJECTING_BLOCKS);
    assert_eq!(reject, Err(Error::ValidationError(ValidationKind::Custom)),);
    staged.rewind(saved_data, saved_entries);
    assert!(staged.is_empty());
    let mut buf = [0xFFu8; 1];
    r.read_bytes(0, &mut buf).unwrap();
    assert_eq!(buf, [0u8]);
}

#[test]
fn block_validator_rejection_short_circuits_write_and_does_not_commit() {
    let r = BlockValRouter::new();
    let mut staged = StagedWrites::new();
    let err = r.write_bytes(0, &[0xAA, 0xBB], &mut staged).unwrap_err();
    assert_eq!(err, Error::ValidationError(ValidationKind::Locked));
    assert!(staged.is_empty());
    let mut buf = [0xFFu8; 2];
    r.read_bytes(0, &mut buf).unwrap();
    assert_eq!(buf, [0u8, 0u8]);
}

#[test]
fn staged_view_overlays_pending_bytes_on_live_data() {
    let r = StubRouter::new();
    let mut staged = StagedWrites::new();
    r.write_bytes(0, &[0x11, 0x22, 0x33, 0x44], &mut staged)
        .unwrap();
    staged.push_chunk(1, &[0xAA, 0xBB]).unwrap();
    let view = StagedView::new(&r, &staged, 0);
    let mut buf = [0u8; 4];
    view.read_bytes(0, &mut buf).unwrap();
    assert_eq!(buf, [0x11, 0xAA, 0xBB, 0x44]);
}

fn seed(r: &StubRouter, writes: &[(usize, &[u8])]) {
    let ptr = r.storage.get();
    for (off, bytes) in writes {
        // SAFETY: tests are single-threaded; StubRouter storage is only touched here.
        unsafe {
            core::ptr::copy_nonoverlapping(bytes.as_ptr(), (ptr as *mut u8).add(*off), bytes.len())
        };
    }
}

#[test]
fn compare_u16_value_rhs_checks_literal_bound() {
    let r = StubRouter::new();
    seed(&r, &[(0, &100u16.to_le_bytes())]);
    let staged = StagedWrites::new();
    let view = StagedView::new(&r, &staged, 0);
    let pass = FieldValidator::CompareU16 {
        op: CompareOp::Le,
        abs: false,
        rhs: Rhs::Value(200),
    };
    assert!(pass.run(&view, 0, 2).is_ok());
    let fail = FieldValidator::CompareU16 {
        op: CompareOp::Le,
        abs: false,
        rhs: Rhs::Value(50),
    };
    assert_eq!(
        fail.run(&view, 0, 2),
        Err(Error::ValidationError(ValidationKind::Compare)),
    );
}

#[test]
fn compare_i32_addr_rhs_reads_value_at_other_addr() {
    let r = StubRouter::new();
    seed(
        &r,
        &[(0, &(-10i32).to_le_bytes()), (12, &100i32.to_le_bytes())],
    );
    let staged = StagedWrites::new();
    let view = StagedView::new(&r, &staged, 0);
    let v = FieldValidator::CompareI32 {
        op: CompareOp::Lt,
        abs: false,
        rhs: Rhs::Addr(12),
    };
    assert!(v.run(&view, 0, 4).is_ok());
}

#[test]
fn compare_i32_abs_compares_magnitudes() {
    let r = StubRouter::new();
    seed(
        &r,
        &[(0, &(-150i32).to_le_bytes()), (12, &100i32.to_le_bytes())],
    );
    let staged = StagedWrites::new();
    let view = StagedView::new(&r, &staged, 0);
    let fail = FieldValidator::CompareI32 {
        op: CompareOp::Le,
        abs: true,
        rhs: Rhs::Addr(12),
    };
    assert_eq!(
        fail.run(&view, 0, 4),
        Err(Error::ValidationError(ValidationKind::Compare)),
    );
    let pass = FieldValidator::CompareI32 {
        op: CompareOp::Ge,
        abs: true,
        rhs: Rhs::Addr(12),
    };
    assert!(pass.run(&view, 0, 4).is_ok());
}

struct Payload {
    a: u8,
    b: u16,
}
impl Region for Payload {}

#[test]
fn sync_unsafe_cell_region_storage_with_and_with_mut_round_trip() {
    use core::cell::SyncUnsafeCell;
    let cell: SyncUnsafeCell<Payload> = SyncUnsafeCell::new(Payload { a: 0, b: 0 });
    cell.with_mut(|p| {
        p.a = 7;
        p.b = 0xBEEF;
    });
    let snap = cell.with(|p| (p.a, p.b));
    assert_eq!(snap, (7, 0xBEEF));
}

#[test]
fn sync_unsafe_cell_region_ptr_aliases_with_view() {
    use core::cell::SyncUnsafeCell;
    let cell: SyncUnsafeCell<Payload> = SyncUnsafeCell::new(Payload { a: 1, b: 2 });
    let p = RegionStorageRaw::region_ptr(&cell);
    // SAFETY: test owns the cell; no other accessor active.
    unsafe { (*p).a = 9 };
    assert_eq!(cell.with(|x| x.a), 9);
}

// [rw 0..2) | reserved 2..6) | rw 6..8) — covers reads spanning a reserved
// hole + writes hitting the hole edge.
const RESERVED_BLOCKS: &[BlockDesc] = &[BlockDesc {
    addr: 0,
    size: 8,
    struct_offset: 0,
    fields: &[
        FieldDesc {
            addr: 0,
            size: 2,
            struct_offset: 0,
            access: Access::Rw,
            validators: &[],
        },
        FieldDesc {
            addr: 2,
            size: 4,
            struct_offset: 2,
            access: Access::Reserved,
            validators: &[],
        },
        FieldDesc {
            addr: 6,
            size: 2,
            struct_offset: 6,
            access: Access::Rw,
            validators: &[],
        },
    ],
    validators: &[],
}];

static RESERVED_REGION: RegionDesc = RegionDesc {
    addr: 0,
    size: REGION_SIZE,
    blocks: RESERVED_BLOCKS,
    validators: &[],
};

static RESERVED_REGIONS: &[&RegionDesc] = &[&RESERVED_REGION];

struct ReservedRouter {
    storage: UnsafeCell<[u8; REGION_SIZE as usize]>,
}

// SAFETY: tests are single-threaded.
unsafe impl Sync for ReservedRouter {}

impl ReservedRouter {
    const fn new() -> Self {
        Self {
            storage: UnsafeCell::new([0; REGION_SIZE as usize]),
        }
    }
    fn seed(&self, off: usize, bytes: &[u8]) {
        // SAFETY: tests are single-threaded.
        unsafe {
            core::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                (self.storage.get() as *mut u8).add(off),
                bytes.len(),
            )
        }
    }
}

impl Router for ReservedRouter {
    fn regions(&self) -> &'static [&'static RegionDesc] {
        RESERVED_REGIONS
    }
    fn region_base(&self, _desc: &RegionDesc) -> Option<*mut u8> {
        Some(self.storage.get() as *mut u8)
    }
}

#[test]
fn reserved_field_read_zero_fills_regardless_of_storage() {
    let r = ReservedRouter::new();
    r.seed(0, &[0x11, 0x22, 0xAA, 0xBB, 0xCC, 0xDD, 0x33, 0x44]);
    let mut buf = [0xFFu8; 4];
    r.read_bytes(2, &mut buf).unwrap();
    assert_eq!(buf, [0, 0, 0, 0]);
}

#[test]
fn reserved_field_write_returns_access_error_and_does_not_commit() {
    let r = ReservedRouter::new();
    r.seed(2, &[0x55, 0x66, 0x77, 0x88]);
    let mut staged = StagedWrites::new();
    let err = r.write_bytes(2, &[0xAA], &mut staged).unwrap_err();
    assert_eq!(err, Error::AccessError);
    assert!(staged.is_empty());
    // Reserved reads zero-fill; storage is untouched but invisible to readers.
    let mut buf = [0xFFu8; 4];
    r.read_bytes(2, &mut buf).unwrap();
    assert_eq!(buf, [0, 0, 0, 0]);
}

#[test]
fn read_spanning_rw_reserved_rw_zero_fills_reserved_middle() {
    let r = ReservedRouter::new();
    r.seed(0, &[0x11, 0x22, 0xAA, 0xBB, 0xCC, 0xDD, 0x33, 0x44]);
    let mut buf = [0xFFu8; 8];
    r.read_bytes(0, &mut buf).unwrap();
    assert_eq!(buf, [0x11, 0x22, 0, 0, 0, 0, 0x33, 0x44]);
}

#[test]
fn write_straddling_reserved_field_returns_access_error() {
    let r = ReservedRouter::new();
    let mut staged = StagedWrites::new();
    // Spans rw [0..2) + reserved [2..6): walk_fields hits reserved with
    // require_rw=true → AccessError before any commit.
    let err = r.write_bytes(0, &[1, 2, 3, 4], &mut staged).unwrap_err();
    assert_eq!(err, Error::AccessError);
    assert!(staged.is_empty());
}

#[test]
fn compare_op_apply_covers_every_op() {
    assert!(CompareOp::Lt.apply(&1, &2));
    assert!(!CompareOp::Lt.apply(&2, &2));
    assert!(CompareOp::Le.apply(&2, &2));
    assert!(!CompareOp::Le.apply(&3, &2));
    assert!(CompareOp::Gt.apply(&3, &2));
    assert!(!CompareOp::Gt.apply(&2, &2));
    assert!(CompareOp::Ge.apply(&2, &2));
    assert!(!CompareOp::Ge.apply(&1, &2));
    assert!(CompareOp::Eq.apply(&2, &2));
    assert!(!CompareOp::Eq.apply(&1, &2));
    assert!(CompareOp::Ne.apply(&1, &2));
    assert!(!CompareOp::Ne.apply(&2, &2));
}
