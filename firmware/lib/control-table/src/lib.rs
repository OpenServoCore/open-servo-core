#![no_std]

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RegmapError {
    OutOfRange,
    AccessError,
    StagingFull,
    ValidationError(ValidationKind),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ValidationKind {
    Enum,
    Range,
    Compare,
    Locked,
    Custom,
}

/// `bool` is `u8` 0/1; any other byte yields UB on later access.
pub const BOOL_ALLOWED: &[u8] = &[0, 1];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Access {
    Ro,
    Rw,
}

pub const STAGE_DATA_CAP: usize = 128;
pub const STAGE_ENTRY_CAP: usize = 16;

#[derive(Copy, Clone)]
struct StagedEntry {
    addr: u16,
    len: u16,
    data_off: u16,
}

pub struct StagedWrites {
    data: heapless::Vec<u8, STAGE_DATA_CAP>,
    entries: heapless::Vec<StagedEntry, STAGE_ENTRY_CAP>,
}

impl StagedWrites {
    pub const fn new() -> Self {
        Self {
            data: heapless::Vec::new(),
            entries: heapless::Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.data.clear();
        self.entries.clear();
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    fn push_chunk(&mut self, addr: u16, src: &[u8]) -> Result<(), RegmapError> {
        let data_off = self.data.len();
        if data_off + src.len() > STAGE_DATA_CAP {
            return Err(RegmapError::StagingFull);
        }
        self.data
            .extend_from_slice(src)
            .map_err(|_| RegmapError::StagingFull)?;
        self.entries
            .push(StagedEntry {
                addr,
                len: src.len() as u16,
                data_off: data_off as u16,
            })
            .map_err(|_| {
                self.data.truncate(data_off);
                RegmapError::StagingFull
            })?;
        Ok(())
    }

    fn rewind(&mut self, data_len: usize, entry_count: usize) {
        self.data.truncate(data_len);
        self.entries.truncate(entry_count);
    }

    fn iter_from(&self, start_entry: usize) -> impl Iterator<Item = (u16, &[u8])> + '_ {
        self.entries[start_entry..].iter().map(|e| {
            let lo = e.data_off as usize;
            let hi = lo + e.len as usize;
            (e.addr, &self.data[lo..hi])
        })
    }
}

impl Default for StagedWrites {
    fn default() -> Self {
        Self::new()
    }
}

/// `struct_offset` is block-relative; the block's own offset is added during walks.
#[derive(Copy, Clone, Debug)]
pub struct FieldDesc {
    pub addr: u16,
    pub size: u16,
    pub struct_offset: u16,
    pub access: Access,
    pub validators: &'static [FieldValidator],
}

#[derive(Copy, Clone, Debug)]
pub struct BlockDesc {
    pub addr: u16,
    pub size: u16,
    pub struct_offset: u16,
    pub fields: &'static [FieldDesc],
    pub validators: &'static [BlockValidator],
}

#[derive(Copy, Clone, Debug)]
pub struct RegionDesc {
    pub addr: u16,
    pub size: u16,
    pub blocks: &'static [BlockDesc],
    pub validators: &'static [RegionValidator],
}

pub type BlockValidator = fn(&StagedView, u16, u16) -> Result<(), RegmapError>;
pub type RegionValidator = fn(&StagedView) -> Result<(), RegmapError>;

#[derive(Copy, Clone, Debug)]
pub enum FieldValidator {
    EnumU8 { allowed: &'static [u8] },
    RangeU8 { lo: u8, hi: u8 },
    RangeU16 { lo: u16, hi: u16 },
    RangeI32 { lo: i32, hi: i32 },
    Cross(CrossField),
    Custom(fn(&StagedView, u16, u16) -> Result<(), RegmapError>),
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CompareOp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

impl CompareOp {
    pub fn apply<T: PartialOrd>(self, a: &T, b: &T) -> bool {
        match self {
            CompareOp::Lt => a < b,
            CompareOp::Le => a <= b,
            CompareOp::Gt => a > b,
            CompareOp::Ge => a >= b,
            CompareOp::Eq => a == b,
            CompareOp::Ne => a != b,
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrossField {
    CompareI16 {
        op: CompareOp,
        other_addr: u16,
    },
    CompareI32 {
        op: CompareOp,
        other_addr: u16,
    },
    WithinI16 {
        lo_addr: u16,
        hi_addr: u16,
    },
    WithinI32 {
        lo_addr: u16,
        hi_addr: u16,
    },
    /// `saturating_abs` on both sides so `i*::MIN` doesn't overflow.
    MagBoundedI16 {
        bound_addr: u16,
    },
    MagBoundedI32 {
        bound_addr: u16,
    },
}

impl CrossField {
    fn run(&self, view: &StagedView, self_addr: u16) -> Result<(), RegmapError> {
        let ok = match *self {
            CrossField::CompareI16 { op, other_addr } => {
                let a = read_le(view, self_addr, i16::from_le_bytes)?;
                let b = read_le(view, other_addr, i16::from_le_bytes)?;
                op.apply(&a, &b)
            }
            CrossField::CompareI32 { op, other_addr } => {
                let a = read_le(view, self_addr, i32::from_le_bytes)?;
                let b = read_le(view, other_addr, i32::from_le_bytes)?;
                op.apply(&a, &b)
            }
            CrossField::WithinI16 { lo_addr, hi_addr } => {
                let v = read_le(view, self_addr, i16::from_le_bytes)?;
                let l = read_le(view, lo_addr, i16::from_le_bytes)?;
                let h = read_le(view, hi_addr, i16::from_le_bytes)?;
                (l..=h).contains(&v)
            }
            CrossField::WithinI32 { lo_addr, hi_addr } => {
                let v = read_le(view, self_addr, i32::from_le_bytes)?;
                let l = read_le(view, lo_addr, i32::from_le_bytes)?;
                let h = read_le(view, hi_addr, i32::from_le_bytes)?;
                (l..=h).contains(&v)
            }
            CrossField::MagBoundedI16 { bound_addr } => {
                let v = read_le(view, self_addr, i16::from_le_bytes)?;
                let b = read_le(view, bound_addr, i16::from_le_bytes)?;
                v.saturating_abs() <= b.saturating_abs()
            }
            CrossField::MagBoundedI32 { bound_addr } => {
                let v = read_le(view, self_addr, i32::from_le_bytes)?;
                let b = read_le(view, bound_addr, i32::from_le_bytes)?;
                v.saturating_abs() <= b.saturating_abs()
            }
        };
        if ok {
            Ok(())
        } else {
            Err(RegmapError::ValidationError(ValidationKind::Compare))
        }
    }
}

impl FieldValidator {
    pub fn run(&self, view: &StagedView, addr: u16, size: u16) -> Result<(), RegmapError> {
        match self {
            FieldValidator::EnumU8 { allowed } => {
                let b = read_le(view, addr, |b: [u8; 1]| b[0])?;
                if allowed.contains(&b) {
                    Ok(())
                } else {
                    Err(RegmapError::ValidationError(ValidationKind::Enum))
                }
            }
            FieldValidator::RangeU8 { lo, hi } => {
                check_range::<u8, 1>(view, addr, *lo, *hi, |b| b[0])
            }
            FieldValidator::RangeU16 { lo, hi } => {
                check_range(view, addr, *lo, *hi, u16::from_le_bytes)
            }
            FieldValidator::RangeI32 { lo, hi } => {
                check_range(view, addr, *lo, *hi, i32::from_le_bytes)
            }
            FieldValidator::Cross(cross) => cross.run(view, addr),
            FieldValidator::Custom(f) => f(view, addr, size),
        }
    }
}

fn read_le<T, const N: usize>(
    view: &StagedView,
    addr: u16,
    decode: fn([u8; N]) -> T,
) -> Result<T, RegmapError> {
    let mut b = [0u8; N];
    view.read_bytes(addr, &mut b)?;
    Ok(decode(b))
}

fn check_range<T: PartialOrd, const N: usize>(
    view: &StagedView,
    addr: u16,
    lo: T,
    hi: T,
    decode: fn([u8; N]) -> T,
) -> Result<(), RegmapError> {
    let v = read_le(view, addr, decode)?;
    if (lo..=hi).contains(&v) {
        Ok(())
    } else {
        Err(RegmapError::ValidationError(ValidationKind::Range))
    }
}

/// Overlays staged-but-uncommitted bytes onto live-table reads so validators
/// see the value about to be committed.
pub struct StagedView<'a> {
    router: &'a dyn Router,
    staged: &'a StagedWrites,
    start_entry: usize,
}

impl<'a> StagedView<'a> {
    pub fn new(router: &'a dyn Router, staged: &'a StagedWrites, start_entry: usize) -> Self {
        Self {
            router,
            staged,
            start_entry,
        }
    }

    pub fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), RegmapError> {
        router_read_bytes(self.router, addr, dst)?;
        if dst.is_empty() {
            return Ok(());
        }
        let req_lo = addr as usize;
        let req_hi = req_lo + dst.len();
        for (s_addr, s_data) in self.staged.iter_from(self.start_entry) {
            let s_lo = s_addr as usize;
            let s_hi = s_lo + s_data.len();
            let lo = req_lo.max(s_lo);
            let hi = req_hi.min(s_hi);
            if lo < hi {
                let dst_off = lo - req_lo;
                let src_off = lo - s_lo;
                dst[dst_off..dst_off + (hi - lo)]
                    .copy_from_slice(&s_data[src_off..src_off + (hi - lo)]);
            }
        }
        Ok(())
    }
}

struct RegionRef {
    base: *mut u8,
    def: &'static RegionDesc,
}

/// `write_bytes`/`commit_staged` form a transient `&mut` to the region via the
/// pointer from `region_base`; caller must hold its single-writer guarantee.
pub trait Router {
    fn regions(&self) -> &'static [&'static RegionDesc];
    fn region_base(&self, desc: &RegionDesc) -> *mut u8;

    fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), RegmapError>
    where
        Self: Sized,
    {
        router_read_bytes(self, addr, dst)
    }

    fn write_bytes(
        &self,
        addr: u16,
        src: &[u8],
        staged: &mut StagedWrites,
    ) -> Result<(), RegmapError>
    where
        Self: Sized,
    {
        let saved_data = staged.data.len();
        let saved_entries = staged.entries.len();
        router_stage_bytes(self, addr, src, staged)?;
        // SAFETY: caller upholds Router's single-writer contract.
        unsafe { commit_staged_range(self, staged, saved_entries) };
        staged.rewind(saved_data, saved_entries);
        Ok(())
    }

    fn stage_bytes(
        &self,
        addr: u16,
        src: &[u8],
        staged: &mut StagedWrites,
    ) -> Result<(), RegmapError>
    where
        Self: Sized,
    {
        router_stage_bytes(self, addr, src, staged)
    }

    fn commit_staged(&self, staged: &mut StagedWrites)
    where
        Self: Sized,
    {
        // SAFETY: caller upholds Router's single-writer contract.
        unsafe { commit_staged_range(self, staged, 0) };
        staged.clear();
    }
}

fn region_for(router: &dyn Router, addr: u16, end: usize) -> Option<RegionRef> {
    let def = router
        .regions()
        .iter()
        .copied()
        .find(|d| range_in(addr, end, d.addr, d.size as usize))?;
    Some(RegionRef {
        base: router.region_base(def),
        def,
    })
}

fn router_read_bytes(router: &dyn Router, addr: u16, dst: &mut [u8]) -> Result<(), RegmapError> {
    if dst.is_empty() {
        return Ok(());
    }
    let end = (addr as usize)
        .checked_add(dst.len())
        .ok_or(RegmapError::OutOfRange)?;
    let r = region_for(router, addr, end).ok_or(RegmapError::OutOfRange)?;
    // SAFETY: get() non-null; descriptor sizes verified at construction.
    unsafe { walk_read(r.base, addr, dst, r.def.blocks) }
}

fn router_stage_bytes(
    router: &dyn Router,
    addr: u16,
    src: &[u8],
    staged: &mut StagedWrites,
) -> Result<(), RegmapError> {
    if src.is_empty() {
        return Ok(());
    }
    let end = (addr as usize)
        .checked_add(src.len())
        .ok_or(RegmapError::OutOfRange)?;
    let r = region_for(router, addr, end).ok_or(RegmapError::OutOfRange)?;
    let saved_data = staged.data.len();
    let saved_entries = staged.entries.len();
    let result = stage_write(addr, src, r.def.blocks, staged)
        .and_then(|()| {
            run_field_validators(router, staged, saved_entries, addr, src.len(), r.def.blocks)
        })
        .and_then(|()| run_region_validators(router, staged, saved_entries, r.def.validators));
    if result.is_err() {
        staged.rewind(saved_data, saved_entries);
    }
    result
}

/// SAFETY: caller holds the region's single-writer guarantee.
unsafe fn commit_staged_range(router: &dyn Router, staged: &StagedWrites, start_entry: usize) {
    for (abs_addr, data) in staged.iter_from(start_entry) {
        let end = abs_addr as usize + data.len();
        let Some(r) = region_for(router, abs_addr, end) else {
            continue;
        };
        unsafe { commit_chunk(r.base, abs_addr, data, r.def.blocks) };
    }
}

fn range_in(addr: u16, end: usize, base: u16, size: usize) -> bool {
    let base = base as usize;
    (addr as usize) >= base && end <= base + size
}

/// Walk fields covering `[abs_start, +len)`. Calls `on_chunk(struct_off, buf_off, chunk_len)`
/// for each overlapping byte run (`struct_off` is region-relative). Returns `AccessError`
/// on a gap (between blocks or inside one), or on an RO field if `require_rw`.
/// Blocks and fields within each block must be sorted by `addr` and non-overlapping.
fn walk_fields(
    abs_start: u16,
    len: usize,
    blocks: &[BlockDesc],
    require_rw: bool,
    mut on_chunk: impl FnMut(usize, usize, usize),
) -> Result<(), RegmapError> {
    let req_hi = abs_start as usize + len;
    let mut req_lo = abs_start as usize;
    let mut buf_pos = 0usize;
    for block in blocks {
        if req_lo >= req_hi {
            break;
        }
        let b_lo = block.addr as usize;
        let b_hi = b_lo + block.size as usize;
        if b_hi <= req_lo {
            continue;
        }
        if b_lo > req_lo {
            return Err(RegmapError::AccessError);
        }
        let block_struct = block.struct_offset as usize;
        for field in block.fields {
            if req_lo >= req_hi.min(b_hi) {
                break;
            }
            let f_lo = field.addr as usize;
            let f_hi = f_lo + field.size as usize;
            if f_hi <= req_lo {
                continue;
            }
            if f_lo > req_lo || (require_rw && field.access != Access::Rw) {
                return Err(RegmapError::AccessError);
            }
            let chunk_hi = req_hi.min(f_hi);
            let chunk_len = chunk_hi - req_lo;
            let struct_off = block_struct + field.struct_offset as usize + (req_lo - f_lo);
            on_chunk(struct_off, buf_pos, chunk_len);
            buf_pos += chunk_len;
            req_lo = chunk_hi;
        }
    }
    if req_lo < req_hi {
        Err(RegmapError::AccessError)
    } else {
        Ok(())
    }
}

unsafe fn walk_read(
    region_base: *const u8,
    abs_start: u16,
    dst: &mut [u8],
    blocks: &[BlockDesc],
) -> Result<(), RegmapError> {
    let dst_ptr = dst.as_mut_ptr();
    walk_fields(
        abs_start,
        dst.len(),
        blocks,
        false,
        |struct_off, dst_pos, chunk_len| unsafe {
            core::ptr::copy_nonoverlapping(
                region_base.add(struct_off),
                dst_ptr.add(dst_pos),
                chunk_len,
            );
        },
    )
}

fn run_field_validators(
    router: &dyn Router,
    staged: &StagedWrites,
    start_entry: usize,
    abs_start: u16,
    len: usize,
    blocks: &[BlockDesc],
) -> Result<(), RegmapError> {
    let view = StagedView {
        router,
        staged,
        start_entry,
    };
    let req_lo = abs_start as usize;
    let req_hi = req_lo + len;
    for block in blocks {
        let b_lo = block.addr as usize;
        let b_hi = b_lo + block.size as usize;
        if b_hi <= req_lo {
            continue;
        }
        if b_lo >= req_hi {
            break;
        }
        for field in block.fields {
            let f_lo = field.addr as usize;
            let f_hi = f_lo + field.size as usize;
            if f_hi <= req_lo {
                continue;
            }
            if f_lo >= req_hi {
                break;
            }
            if field.validators.is_empty() {
                continue;
            }
            for v in field.validators {
                v.run(&view, field.addr, field.size)?;
            }
        }
    }
    Ok(())
}

fn run_region_validators(
    router: &dyn Router,
    staged: &StagedWrites,
    start_entry: usize,
    validators: &[RegionValidator],
) -> Result<(), RegmapError> {
    if validators.is_empty() {
        return Ok(());
    }
    let view = StagedView {
        router,
        staged,
        start_entry,
    };
    for v in validators {
        v(&view)?;
    }
    Ok(())
}

/// Validate that `src` lies entirely in RW fields with no gaps; stage as one entry.
fn stage_write(
    abs_addr: u16,
    src: &[u8],
    blocks: &[BlockDesc],
    staged: &mut StagedWrites,
) -> Result<(), RegmapError> {
    walk_fields(abs_addr, src.len(), blocks, true, |_, _, _| {})?;
    staged.push_chunk(abs_addr, src)
}

/// SAFETY: `region_base` points to the matching region struct; `[abs_addr, abs_addr+data.len())`
/// is contained in that region and covered by RW fields per `blocks`; caller has exclusive access.
unsafe fn commit_chunk(region_base: *mut u8, abs_addr: u16, data: &[u8], blocks: &[BlockDesc]) {
    let data_ptr = data.as_ptr();
    let _ = walk_fields(
        abs_addr,
        data.len(),
        blocks,
        false,
        |struct_off, data_pos, chunk_len| unsafe {
            core::ptr::copy_nonoverlapping(
                data_ptr.add(data_pos),
                region_base.add(struct_off),
                chunk_len,
            );
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::UnsafeCell;

    const REGION_SIZE: u16 = 32;

    fn always_reject(_view: &StagedView, _addr: u16, _size: u16) -> Result<(), RegmapError> {
        Err(RegmapError::ValidationError(ValidationKind::Custom))
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
        fn region_base(&self, _desc: &RegionDesc) -> *mut u8 {
            self.storage.get() as *mut u8
        }
    }

    #[test]
    fn walk_fields_rejects_writes_into_padding_gap() {
        let r = StubRouter::new();
        let mut staged = StagedWrites::new();
        let err = r.write_bytes(4, &[0xAA], &mut staged).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
        assert!(staged.is_empty());
    }

    #[test]
    fn walk_fields_rejects_writes_to_ro_field() {
        let r = StubRouter::new();
        let mut staged = StagedWrites::new();
        let err = r.write_bytes(8, &[0xAA], &mut staged).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
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
        assert_eq!(
            reject,
            Err(RegmapError::ValidationError(ValidationKind::Custom)),
        );
        staged.rewind(saved_data, saved_entries);
        assert!(staged.is_empty());
        // Live storage untouched (the StubRouter region was never written).
        let mut buf = [0xFFu8; 1];
        r.read_bytes(0, &mut buf).unwrap();
        assert_eq!(buf, [0u8]);
    }

    #[test]
    fn staged_view_overlays_pending_bytes_on_live_data() {
        let r = StubRouter::new();
        // Seed live storage at addr 0 with [0x11, 0x22, 0x33, 0x44].
        let mut staged = StagedWrites::new();
        r.write_bytes(0, &[0x11, 0x22, 0x33, 0x44], &mut staged)
            .unwrap();
        // Now stage an overlay at addr 1 covering 2 bytes.
        staged.push_chunk(1, &[0xAA, 0xBB]).unwrap();
        let view = StagedView::new(&r, &staged, 0);
        let mut buf = [0u8; 4];
        view.read_bytes(0, &mut buf).unwrap();
        assert_eq!(buf, [0x11, 0xAA, 0xBB, 0x44]);
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
}
