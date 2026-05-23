//! Byte-addressed access to the control table.
//!
//! Region → BlockDesc → FieldDesc descriptors map protocol slots to (struct offset, size, access):
//! * Outside any region → `OutOfRange`. Cross-region spans → `OutOfRange`.
//! * Block gap, padding gap, reserved slot, or RO byte on write → `AccessError`.
//! * Writes validate the full span before committing; failed write leaves the table untouched.
//!
//! Reads use raw-pointer copies, never form `&RegionT`. Writes form a transient
//! `&mut`-equivalent via `SyncUnsafeCell::get()`; caller must hold the region's
//! single-writer guarantee (CONFIG/CONTROL/CALIB from `services`, TELEMETRY from `kernel`).
//!
//! Struct fields whose name starts with `_rsvd_` are absent from `BlockDesc.fields` and remain
//! gaps. Hand-written today; layout is shaped to match a future `#[derive(RegmapBlock)]` macro.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RegmapError {
    /// Empty, wrapping, or outside every region.
    OutOfRange,
    /// Padding gap, reserved slot, or RO byte on write.
    AccessError,
    /// Staging buffer has no room for this chunk.
    StagingFull,
    /// All kinds map to wire `StatusError::DataRange`.
    ValidationError(ValidationKind),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ValidationKind {
    Enum,
    Range,
    Compare,
    Locked,
    Custom,
}

/// Allowed bytes for `bool` fields in the control table. `bool`'s representation
/// is `u8` with values `0` or `1`; any other byte yields UB on later access, so
/// every typed-`bool` field MUST attach `EnumU8 { allowed: BOOL_ALLOWED }`.
pub const BOOL_ALLOWED: &[u8] = &[0, 1];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
    /// Offset into `StagedWrites::data`.
    data_off: u16,
}

/// Per-poll staging buffer for writes pending commit.
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

/// `struct_offset` is **block-relative**; the block's own offset is added during walks.
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
    /// Region-relative byte offset of the block's storage struct.
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

/// Field-level value validator. Runs against a [`StagedView`] so it sees the
/// pending write, not the live byte. Width is implicit per variant; attaching a
/// width-mismatched variant to a field reads only the variant's bytes.
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
pub enum CompareOp {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
}

impl CompareOp {
    fn apply<T: PartialOrd>(self, a: &T, b: &T) -> bool {
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
    /// `|self| ≤ |bound|`; `saturating_abs` on both sides so `i*::MIN` doesn't overflow.
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
    fn run(&self, view: &StagedView, addr: u16, size: u16) -> Result<(), RegmapError> {
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
    // SAFETY: get() non-null; descriptor sizes verified in `region_structs_fit_regions`.
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
        // stage_write guarantees single-region containment.
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
    // stage_write already validated this range; walk_fields can't error here.
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
    use crate::ControlTable;
    use crate::regions::{
        CALIB_BASE_ADDR, CONFIG_BASE_ADDR, CONTROL_BASE_ADDR, TELEMETRY_BASE_ADDR,
    };

    fn fresh() -> ControlTable {
        ControlTable::const_new()
    }

    fn write(t: &ControlTable, addr: u16, src: &[u8]) -> Result<(), RegmapError> {
        let mut staged = StagedWrites::default();
        let result = t.write_bytes(addr, src, &mut staged);
        assert!(staged.is_empty(), "staged not emptied after sync write");
        result
    }

    #[test]
    fn read_identity_initially_zero() {
        let t = fresh();
        let mut buf = [0xAAu8; 12];
        t.read_bytes(CONFIG_BASE_ADDR, &mut buf).unwrap();
        assert_eq!(buf, [0u8; 12]);
    }

    #[test]
    fn write_then_read_comms() {
        use crate::regions::config::BaudRate;
        let t = fresh();
        // block 1 (offset 32) = comms, 3 B body (id u8 + baud_rate_idx u8 + return_delay_2us u8).
        let comms_addr = CONFIG_BASE_ADDR + 32;
        let payload = [0x07u8, 0x03, 75];
        write(&t, comms_addr, &payload).unwrap();
        let mut buf = [0u8; 3];
        t.read_bytes(comms_addr, &mut buf).unwrap();
        assert_eq!(buf, payload);
        let comms = unsafe { &*t.config.get() }.comms;
        assert_eq!(comms.id, 0x07);
        assert_eq!(comms.baud_rate_idx, BaudRate::B1000000);
        assert_eq!(comms.return_delay_2us, 75);
    }

    #[test]
    fn write_to_ro_identity_fails_and_does_not_mutate() {
        let t = fresh();
        let payload = [0xFFu8; 4];
        let err = write(&t, CONFIG_BASE_ADDR, &payload).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
        let identity = unsafe { &*t.config.get() }.identity;
        assert_eq!(identity.model_number, 0);
    }

    #[test]
    fn read_into_reserved_tail_fails() {
        // identity = 12 B; bytes 12..32 in block 0 are reserved.
        let t = fresh();
        let mut buf = [0u8; 1];
        let err = t.read_bytes(CONFIG_BASE_ADDR + 12, &mut buf).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn read_spanning_into_reserved_block_fails() {
        // Start in block 5 last 4 B, spill into reserved block 6.
        let t = fresh();
        let mut buf = [0u8; 8];
        let addr = CONFIG_BASE_ADDR + (5 * 32) + 20;
        let err = t.read_bytes(addr, &mut buf).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn cross_region_read_fails() {
        let t = fresh();
        let mut buf = [0u8; 8];
        // End of CONFIG → spill into TELEMETRY.
        let err = t.read_bytes(0x01FE, &mut buf).unwrap_err();
        assert_eq!(err, RegmapError::OutOfRange);
    }

    #[test]
    fn unknown_address_fails() {
        let t = fresh();
        let mut buf = [0u8; 1];
        let err = t.read_bytes(0xFFFF, &mut buf).unwrap_err();
        assert_eq!(err, RegmapError::OutOfRange);
    }

    #[test]
    fn write_control_lifecycle_full_block() {
        use crate::regions::config;
        use crate::regions::control::Mode;
        let t = fresh();
        // Seed soft limits + max_effort so goal_position/goal_effort have non-degenerate windows.
        let mut limits = [0u8; 16];
        limits[0..4].copy_from_slice(&(-100_000i32).to_le_bytes());
        limits[4..8].copy_from_slice(&100_000i32.to_le_bytes());
        limits[8..12].copy_from_slice(&(-50_000i32).to_le_bytes());
        limits[12..16].copy_from_slice(&50_000i32.to_le_bytes());
        write(&t, CONFIG_BASE_ADDR + 64, &limits).unwrap();
        write(&t, config::FIELD_MAX_EFFORT.addr, &500i16.to_le_bytes()).unwrap();
        // _rsvd_align gap at offset 2..4 forces a split write.
        write(&t, CONTROL_BASE_ADDR, &[1, 1]).unwrap();
        let mut tail = [0u8; 10];
        tail[0..4].copy_from_slice(&10000i32.to_le_bytes());
        tail[4..8].copy_from_slice(&2000i32.to_le_bytes());
        tail[8..10].copy_from_slice(&320i16.to_le_bytes());
        write(&t, CONTROL_BASE_ADDR + 4, &tail).unwrap();
        let lc = unsafe { &*t.control.get() }.lifecycle;
        assert!(lc.torque_enable);
        assert_eq!(lc.mode, Mode::PositionPid);
        assert_eq!(lc.goal_position, 10000);
        assert_eq!(lc.goal_velocity, 2000);
        assert_eq!(lc.goal_effort, 320);
    }

    #[test]
    fn telemetry_writes_fail() {
        let t = fresh();
        let payload = [0u8; 4];
        let err = write(&t, TELEMETRY_BASE_ADDR, &payload).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn telemetry_reads_succeed_for_active_fields() {
        // TelemetryConverted covers 14 B; trailing struct-alignment pad is a gap.
        let t = fresh();
        let mut buf = [0xAAu8; 14];
        t.read_bytes(TELEMETRY_BASE_ADDR, &mut buf).unwrap();
        assert_eq!(buf, [0u8; 14]);
    }

    #[test]
    fn calib_block_round_trip() {
        let t = fresh();
        // block 1 = bemf, 40 B.
        let bemf_addr = CALIB_BASE_ADDR + 256;
        let mut payload = [0u8; 8];
        payload[0..2].copy_from_slice(&1234u16.to_le_bytes());
        payload[2..4].copy_from_slice(&500u16.to_le_bytes());
        payload[4..6].copy_from_slice(&12000u16.to_le_bytes());
        payload[6..8].copy_from_slice(&100u16.to_le_bytes());
        write(&t, bemf_addr, &payload).unwrap();
        let mut readback = [0u8; 8];
        t.read_bytes(bemf_addr, &mut readback).unwrap();
        assert_eq!(readback, payload);
    }

    #[test]
    fn calib_reserved_block_fails() {
        let t = fresh();
        // block 2 (offset 512) reserved.
        let mut buf = [0u8; 1];
        let err = t.read_bytes(CALIB_BASE_ADDR + 512, &mut buf).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn write_config_pos_limits_round_trip() {
        let t = fresh();
        let addr = CONFIG_BASE_ADDR + 64;
        let mut payload = [0u8; 16];
        payload[0..4].copy_from_slice(&(-1_000_000i32).to_le_bytes());
        payload[4..8].copy_from_slice(&1_000_000i32.to_le_bytes());
        payload[8..12].copy_from_slice(&(-500_000i32).to_le_bytes());
        payload[12..16].copy_from_slice(&500_000i32.to_le_bytes());
        write(&t, addr, &payload).unwrap();
        let pos = unsafe { &*t.config.get() }.limits.pos;
        assert_eq!(pos.pos_min_phys_urad, -1_000_000);
        assert_eq!(pos.pos_max_phys_urad, 1_000_000);
        assert_eq!(pos.pos_min_soft_urad, -500_000);
        assert_eq!(pos.pos_max_soft_urad, 500_000);
    }

    #[test]
    fn write_config_pid_round_trip() {
        let t = fresh();
        let addr = CONFIG_BASE_ADDR + 5 * 32;
        // _rsvd_align gap at offset 6..8 forces a split write.
        let mut head = [0u8; 6];
        head[0..2].copy_from_slice(&0x0100u16.to_le_bytes());
        head[2..4].copy_from_slice(&0x0040u16.to_le_bytes());
        head[4..6].copy_from_slice(&0x0080u16.to_le_bytes());
        write(&t, addr, &head).unwrap();
        write(&t, addr + 8, &5000i32.to_le_bytes()).unwrap();
        let pid = unsafe { &*t.config.get() }.control.position;
        assert_eq!(pid.pid_kp_q88, 0x0100);
        assert_eq!(pid.pid_ki_q88, 0x0040);
        assert_eq!(pid.pid_kd_q88, 0x0080);
        assert_eq!(pid.pid_i_limit, 5000);
    }

    #[test]
    fn write_control_streaming_round_trip() {
        let t = fresh();
        let addr = CONTROL_BASE_ADDR + 32;
        let mut payload = [0u8; 8];
        payload[0] = 1;
        payload[1] = 4;
        payload[2..4].copy_from_slice(&500u16.to_le_bytes());
        payload[4..8].copy_from_slice(&0x0000_00ABu32.to_le_bytes());
        write(&t, addr, &payload).unwrap();
        let s = unsafe { &*t.control.get() }.streaming;
        assert!(s.stream_enable);
        assert_eq!(s.stream_decimation, 4);
        assert_eq!(s.stream_duration_ms, 500);
        assert_eq!(s.stream_field_mask, 0xAB);
    }

    #[test]
    fn write_to_stream_dropped_fails() {
        let t = fresh();
        let addr = CONTROL_BASE_ADDR + 32 + 8;
        let err = write(&t, addr, &[0u8; 4]).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn read_from_stream_dropped_succeeds() {
        let t = fresh();
        let addr = CONTROL_BASE_ADDR + 32 + 8;
        let mut buf = [0xAAu8; 4];
        t.read_bytes(addr, &mut buf).unwrap();
        assert_eq!(buf, [0u8; 4]);
    }

    #[test]
    fn write_calib_pot_lut_partial_does_not_smash_neighbours() {
        let t = fresh();
        let addr = CALIB_BASE_ADDR;
        let payload = [0xAA, 0xBB, 0xCC, 0xDD];
        write(&t, addr, &payload).unwrap();
        let pot = unsafe { &*t.calib.get() }.pot_lut;
        assert_eq!(pot.raw_min, 0xBBAA);
        assert_eq!(pot.raw_max, 0xDDCC);
        assert_eq!(pot.lut[0], 0);
    }

    #[test]
    fn write_spanning_block_body_into_reserved_tail_fails_atomically() {
        let t = fresh();
        let addr = CONFIG_BASE_ADDR + 3 * 32 + 8;
        let payload = [0xFFu8; 32];
        let err = write(&t, addr, &payload).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
        let stall = unsafe { &*t.config.get() }.safety.stall;
        assert_eq!(stall.stall_motion_threshold_urad, 0);
    }

    fn assert_region_sorted_and_bounded(region: &RegionDesc) {
        let region_end = region.addr + region.size;
        for win in region.blocks.windows(2) {
            let (a, b) = (&win[0], &win[1]);
            assert!(a.addr + a.size <= b.addr, "blocks overlap: {:?} {:?}", a, b);
        }
        for block in region.blocks {
            assert!(
                block.addr >= region.addr && block.addr + block.size <= region_end,
                "block out of region: {:?}",
                block,
            );
            for win in block.fields.windows(2) {
                let (a, b) = (&win[0], &win[1]);
                assert!(a.addr + a.size <= b.addr, "fields overlap: {:?} {:?}", a, b);
            }
            if let Some(last) = block.fields.last() {
                assert!(
                    last.addr + last.size <= block.addr + block.size,
                    "field runs past block: {:?}",
                    last,
                );
            }
        }
    }

    #[test]
    fn config_blocks_sorted_and_bounded() {
        assert_region_sorted_and_bounded(&crate::regions::config::CONFIG_REGION);
    }

    #[test]
    fn telemetry_blocks_sorted_and_bounded() {
        assert_region_sorted_and_bounded(&crate::regions::telemetry::TELEMETRY_REGION);
    }

    #[test]
    fn control_blocks_sorted_and_bounded() {
        assert_region_sorted_and_bounded(&crate::regions::control::CONTROL_REGION);
    }

    #[test]
    fn calib_blocks_sorted_and_bounded() {
        assert_region_sorted_and_bounded(&crate::regions::calib::CALIB_REGION);
    }

    #[test]
    fn write_to_padding_gap_fails() {
        // ControlLifecycle _rsvd_align gap at addr 2..4.
        let t = fresh();
        let err = write(&t, CONTROL_BASE_ADDR + 2, &[0u8, 0u8]).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn read_from_padding_gap_fails() {
        let t = fresh();
        let mut buf = [0u8; 2];
        let err = t.read_bytes(CONTROL_BASE_ADDR + 2, &mut buf).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn partial_write_preserves_unwritten_bytes() {
        use crate::regions::config::BaudRate;
        let t = fresh();
        let comms_addr = CONFIG_BASE_ADDR + 32;
        write(&t, comms_addr, &[0x07, 0x03, 75]).unwrap();
        write(&t, comms_addr, &[0x09, 0x05]).unwrap();
        let comms = unsafe { &*t.config.get() }.comms;
        assert_eq!(comms.id, 0x09);
        assert_eq!(comms.baud_rate_idx, BaudRate::B3000000);
        assert_eq!(comms.return_delay_2us, 75);
    }

    #[test]
    fn staged_view_overlays_pending_bytes_on_live_table() {
        let t = fresh();
        let comms_addr = CONFIG_BASE_ADDR + 32;
        write(&t, comms_addr, &[0x07, 0x03, 75]).unwrap();
        let mut staged = StagedWrites::default();
        staged.push_chunk(comms_addr + 1, &[0xAA, 0xBB]).unwrap();
        let view = StagedView {
            router: &t,
            staged: &staged,
            start_entry: 0,
        };
        let mut buf = [0u8; 3];
        view.read_bytes(comms_addr, &mut buf).unwrap();
        assert_eq!(buf, [0x07, 0xAA, 0xBB]);
    }

    #[test]
    fn staged_view_ignores_entries_before_start_entry() {
        let t = fresh();
        let comms_addr = CONFIG_BASE_ADDR + 32;
        write(&t, comms_addr, &[0x07, 0x03, 75]).unwrap();
        let mut staged = StagedWrites::default();
        staged.push_chunk(comms_addr, &[0xFF]).unwrap();
        let view = StagedView {
            router: &t,
            staged: &staged,
            start_entry: 1,
        };
        let mut buf = [0u8; 1];
        view.read_bytes(comms_addr, &mut buf).unwrap();
        assert_eq!(buf, [0x07]);
    }

    #[test]
    fn enum_u8_validator_accepts_listed_byte() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        staged.push_chunk(CONTROL_BASE_ADDR, &[1]).unwrap();
        let view = StagedView {
            router: &t,
            staged: &staged,
            start_entry: 0,
        };
        let v = FieldValidator::EnumU8 { allowed: &[0, 1] };
        assert_eq!(v.run(&view, CONTROL_BASE_ADDR, 1), Ok(()));
    }

    #[test]
    fn enum_u8_validator_rejects_unlisted_byte() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        staged.push_chunk(CONTROL_BASE_ADDR, &[2]).unwrap();
        let view = StagedView {
            router: &t,
            staged: &staged,
            start_entry: 0,
        };
        let v = FieldValidator::EnumU8 { allowed: &[0, 1] };
        assert_eq!(
            v.run(&view, CONTROL_BASE_ADDR, 1),
            Err(RegmapError::ValidationError(ValidationKind::Enum)),
        );
    }

    #[test]
    fn range_u8_validator_accepts_and_rejects_at_bounds() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        staged.push_chunk(CONTROL_BASE_ADDR, &[5]).unwrap();
        let view = StagedView {
            router: &t,
            staged: &staged,
            start_entry: 0,
        };
        let v = FieldValidator::RangeU8 { lo: 0, hi: 10 };
        assert_eq!(v.run(&view, CONTROL_BASE_ADDR, 1), Ok(()));
        let mut staged2 = StagedWrites::default();
        staged2.push_chunk(CONTROL_BASE_ADDR, &[11]).unwrap();
        let view2 = StagedView {
            router: &t,
            staged: &staged2,
            start_entry: 0,
        };
        assert_eq!(
            v.run(&view2, CONTROL_BASE_ADDR, 1),
            Err(RegmapError::ValidationError(ValidationKind::Range)),
        );
    }

    #[test]
    fn range_u16_validator_reads_le_bytes() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        staged
            .push_chunk(CONTROL_BASE_ADDR, &500u16.to_le_bytes())
            .unwrap();
        let view = StagedView {
            router: &t,
            staged: &staged,
            start_entry: 0,
        };
        let v = FieldValidator::RangeU16 { lo: 100, hi: 1000 };
        assert_eq!(v.run(&view, CONTROL_BASE_ADDR, 2), Ok(()));
        let mut staged2 = StagedWrites::default();
        staged2
            .push_chunk(CONTROL_BASE_ADDR, &50u16.to_le_bytes())
            .unwrap();
        let view2 = StagedView {
            router: &t,
            staged: &staged2,
            start_entry: 0,
        };
        assert_eq!(
            v.run(&view2, CONTROL_BASE_ADDR, 2),
            Err(RegmapError::ValidationError(ValidationKind::Range)),
        );
    }

    #[test]
    fn range_i32_validator_handles_negative_bounds() {
        // ControlLifecycle::goal_position lives at CONTROL_BASE_ADDR + 4 (after the
        // _rsvd_align gap at offset 2..4), so a 4-byte read lands cleanly on one field.
        let addr = CONTROL_BASE_ADDR + 4;
        let t = fresh();
        let mut staged = StagedWrites::default();
        staged.push_chunk(addr, &(-100i32).to_le_bytes()).unwrap();
        let view = StagedView {
            router: &t,
            staged: &staged,
            start_entry: 0,
        };
        let v = FieldValidator::RangeI32 {
            lo: -1000,
            hi: 1000,
        };
        assert_eq!(v.run(&view, addr, 4), Ok(()));
        let mut staged2 = StagedWrites::default();
        staged2.push_chunk(addr, &(-2000i32).to_le_bytes()).unwrap();
        let view2 = StagedView {
            router: &t,
            staged: &staged2,
            start_entry: 0,
        };
        assert_eq!(
            v.run(&view2, addr, 4),
            Err(RegmapError::ValidationError(ValidationKind::Range)),
        );
    }

    #[test]
    fn cross_compare_i16_dispatches_correctly() {
        use crate::regions::config::{FIELD_WINDING_CUTOFF_CC, FIELD_WINDING_RECOVER_CC};
        let t = fresh();
        // Defaults are 0; lift cutoff before lowering recover or both writes reject.
        write(&t, FIELD_WINDING_CUTOFF_CC.addr, &80i16.to_le_bytes()).unwrap();
        write(&t, FIELD_WINDING_RECOVER_CC.addr, &50i16.to_le_bytes()).unwrap();
        let err = write(&t, FIELD_WINDING_CUTOFF_CC.addr, &40i16.to_le_bytes()).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Compare));
        let mut buf = [0u8; 2];
        t.read_bytes(FIELD_WINDING_CUTOFF_CC.addr, &mut buf)
            .unwrap();
        assert_eq!(i16::from_le_bytes(buf), 80);
    }

    #[test]
    fn cross_compare_i32_dispatches_correctly() {
        use crate::regions::config::{FIELD_POS_MAX_PHYS_URAD, FIELD_POS_MIN_PHYS_URAD};
        let t = fresh();
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&(-100i32).to_le_bytes());
        payload[4..8].copy_from_slice(&100i32.to_le_bytes());
        write(&t, FIELD_POS_MIN_PHYS_URAD.addr, &payload).unwrap();
        let err = write(&t, FIELD_POS_MAX_PHYS_URAD.addr, &(-200i32).to_le_bytes()).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Compare));
    }

    #[test]
    fn cross_within_i32_dispatches_correctly() {
        use crate::regions::config::{
            FIELD_POS_MAX_SOFT_URAD, FIELD_POS_MIN_PHYS_URAD, FIELD_POS_MIN_SOFT_URAD,
        };
        let t = fresh();
        let mut payload = [0u8; 16];
        payload[0..4].copy_from_slice(&(-1000i32).to_le_bytes());
        payload[4..8].copy_from_slice(&1000i32.to_le_bytes());
        payload[8..12].copy_from_slice(&(-500i32).to_le_bytes());
        payload[12..16].copy_from_slice(&500i32.to_le_bytes());
        write(&t, FIELD_POS_MIN_PHYS_URAD.addr, &payload).unwrap();
        let err = write(&t, FIELD_POS_MAX_SOFT_URAD.addr, &2000i32.to_le_bytes()).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Compare));
        write(&t, FIELD_POS_MAX_SOFT_URAD.addr, &900i32.to_le_bytes()).unwrap();
        let mut buf = [0u8; 4];
        t.read_bytes(FIELD_POS_MIN_SOFT_URAD.addr, &mut buf)
            .unwrap();
        assert_eq!(i32::from_le_bytes(buf), -500);
    }

    #[test]
    fn cross_mag_bounded_i16_dispatches_correctly() {
        use crate::regions::config::FIELD_MAX_EFFORT;
        use crate::regions::control::FIELD_GOAL_EFFORT;
        let t = fresh();
        write(&t, FIELD_MAX_EFFORT.addr, &100i16.to_le_bytes()).unwrap();
        write(&t, FIELD_GOAL_EFFORT.addr, &(-50i16).to_le_bytes()).unwrap();
        write(&t, FIELD_GOAL_EFFORT.addr, &100i16.to_le_bytes()).unwrap();
        let err = write(&t, FIELD_GOAL_EFFORT.addr, &200i16.to_le_bytes()).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Compare));
        let err = write(&t, FIELD_GOAL_EFFORT.addr, &(-200i16).to_le_bytes()).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Compare));
    }

    #[test]
    fn cross_mag_bounded_saturating_abs_handles_i_min() {
        // Bare `-i16::MIN` would overflow; saturating_abs is what's under test.
        let t = fresh();
        let bound_addr = crate::regions::config::FIELD_MAX_EFFORT.addr;
        let goal_addr = crate::regions::control::FIELD_GOAL_EFFORT.addr;
        write(&t, bound_addr, &i16::MIN.to_le_bytes()).unwrap();
        write(&t, goal_addr, &i16::MAX.to_le_bytes()).unwrap();
        write(&t, goal_addr, &i16::MIN.to_le_bytes()).unwrap();
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

    #[test]
    fn stage_bytes_to_ro_address_fails_immediately() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        let err = t
            .stage_bytes(CONFIG_BASE_ADDR, &[0xAA, 0xBB], &mut staged)
            .unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
        assert!(staged.is_empty());
    }

    #[test]
    fn stage_then_commit_round_trip() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        t.stage_bytes(CONTROL_BASE_ADDR, &[1], &mut staged).unwrap();
        assert!(!unsafe { &*t.control.get() }.lifecycle.torque_enable);
        t.commit_staged(&mut staged);
        assert!(unsafe { &*t.control.get() }.lifecycle.torque_enable);
        assert!(staged.is_empty());
    }

    #[test]
    fn stage_invalid_value_rejected_and_buffer_rewound() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        let err = t
            .stage_bytes(CONTROL_BASE_ADDR, &[2], &mut staged)
            .unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Enum));
        assert!(staged.is_empty());
    }

    #[test]
    fn commit_staged_empty_buffer_is_noop() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        t.commit_staged(&mut staged);
        assert!(staged.is_empty());
    }

    #[test]
    fn commit_staged_applies_multiple_entries_in_order() {
        let t = fresh();
        let mut staged = StagedWrites::default();
        t.stage_bytes(CONTROL_BASE_ADDR, &[1], &mut staged).unwrap();
        t.stage_bytes(CONFIG_BASE_ADDR + 32, &[0x42], &mut staged)
            .unwrap();
        t.commit_staged(&mut staged);
        assert!(unsafe { &*t.control.get() }.lifecycle.torque_enable);
        assert_eq!(unsafe { &*t.config.get() }.comms.id, 0x42);
    }

    #[test]
    fn write_bytes_invalid_bool_rejected_rolls_back() {
        let t = fresh();
        write(&t, CONTROL_BASE_ADDR, &[1]).unwrap();
        assert!(unsafe { &*t.control.get() }.lifecycle.torque_enable);
        let err = write(&t, CONTROL_BASE_ADDR, &[2]).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Enum));
        // Live byte must still reflect the prior write, not the rejected 0x02.
        assert!(unsafe { &*t.control.get() }.lifecycle.torque_enable);
    }

    #[test]
    fn write_bytes_out_of_range_u8_rejected_rolls_back() {
        let t = fresh();
        // ConfigComms.id is at CONFIG_BASE_ADDR + 32, RangeU8 0..=252.
        let id_addr = CONFIG_BASE_ADDR + 32;
        write(&t, id_addr, &[0x42]).unwrap();
        assert_eq!(unsafe { &*t.config.get() }.comms.id, 0x42);
        let err = write(&t, id_addr, &[253]).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Range));
        assert_eq!(unsafe { &*t.config.get() }.comms.id, 0x42);
    }

    #[test]
    fn write_rejected_by_validator_leaves_live_table_unchanged() {
        // Synthesise a one-field table with a validator that always rejects, and
        // run it through the same stage→validate→commit path as `write_bytes`.
        fn always_reject(_view: &StagedView, _addr: u16, _size: u16) -> Result<(), RegmapError> {
            Err(RegmapError::ValidationError(ValidationKind::Custom))
        }
        const TEST_BLOCKS: &[BlockDesc] = &[BlockDesc {
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
        let t = fresh();
        let mut staged = StagedWrites::default();
        let saved_entries = staged.entries.len();
        let saved_data = staged.data.len();
        let stage = stage_write(0, &[0xAA], TEST_BLOCKS, &mut staged);
        assert!(stage.is_ok());
        let result = run_field_validators(&t, &staged, saved_entries, 0, 1, TEST_BLOCKS);
        assert_eq!(
            result,
            Err(RegmapError::ValidationError(ValidationKind::Custom)),
        );
        staged.rewind(saved_data, saved_entries);
        assert!(staged.is_empty());
    }

    #[test]
    fn config_write_rejected_when_torque_enabled() {
        use crate::regions::control::FIELD_TORQUE_ENABLE;
        let t = fresh();
        write(&t, FIELD_TORQUE_ENABLE.addr, &[1]).unwrap();
        let err = write(&t, CONFIG_BASE_ADDR + 32, &[0x42]).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Locked));
        assert_eq!(unsafe { &*t.config.get() }.comms.id, 0);
    }

    #[test]
    fn config_write_accepted_when_torque_disabled() {
        let t = fresh();
        write(&t, CONFIG_BASE_ADDR + 32, &[0x42]).unwrap();
        assert_eq!(unsafe { &*t.config.get() }.comms.id, 0x42);
    }

    #[test]
    fn calib_write_rejected_when_torque_enabled() {
        use crate::regions::control::FIELD_TORQUE_ENABLE;
        let t = fresh();
        write(&t, FIELD_TORQUE_ENABLE.addr, &[1]).unwrap();
        let err = write(&t, CALIB_BASE_ADDR, &[0xAA, 0xBB]).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Locked));
    }

    #[test]
    fn control_writes_ignore_torque_lock() {
        use crate::regions::control::FIELD_TORQUE_ENABLE;
        let t = fresh();
        write(&t, FIELD_TORQUE_ENABLE.addr, &[1]).unwrap();
        write(&t, FIELD_TORQUE_ENABLE.addr, &[0]).unwrap();
    }

    #[test]
    fn field_validation_surfaces_before_lock() {
        // Out-of-range CONFIG write under torque must report Range, not Locked.
        use crate::regions::control::FIELD_TORQUE_ENABLE;
        let t = fresh();
        write(&t, FIELD_TORQUE_ENABLE.addr, &[1]).unwrap();
        let err = write(&t, CONFIG_BASE_ADDR + 32, &[253]).unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Range));
    }

    #[test]
    fn locked_rejection_rewinds_staging() {
        use crate::regions::control::FIELD_TORQUE_ENABLE;
        let t = fresh();
        write(&t, FIELD_TORQUE_ENABLE.addr, &[1]).unwrap();
        let mut staged = StagedWrites::default();
        let err = t
            .stage_bytes(CONFIG_BASE_ADDR + 32, &[0x42], &mut staged)
            .unwrap_err();
        assert_eq!(err, RegmapError::ValidationError(ValidationKind::Locked));
        assert!(staged.is_empty());
    }
}
