use crate::desc::{Access, BlockDesc, Error, RegionDesc};
use crate::stage::{Snapshot, StagedWrites};
use crate::validate::{run_block_validators, run_field_validators, run_region_validators};

/// `write_bytes`/`commit_staged` form a transient `&mut` to the region via the
/// pointer from `region_base`; caller must hold its single-writer guarantee.
///
/// Default methods are gated `where Self: Sized` so they aren't callable via
/// `&dyn Router`. `router_*` free fns are generic `<R: Router + ?Sized>` so the
/// hot Read path (called with `&ControlTable`) monomorphizes and inlines the
/// `regions()` / `region_base()` calls, while off-path callers (`StagedView`'s
/// internal `&dyn Router`) keep the existing dynamic dispatch path.
pub trait Router {
    fn regions(&self) -> &'static [&'static RegionDesc];
    fn region_base(&self, desc: &RegionDesc) -> Option<*mut u8>;

    fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), Error>
    where
        Self: Sized,
    {
        router_read_bytes(self, addr, dst)
    }

    /// Iterator-shape read: yields one `ReadChunk` per descriptor segment of
    /// `[addr, addr+len)`. `Copy` chunks borrow directly from the region's
    /// backing storage; `Zero` chunks describe phantom address spans (gaps
    /// between fields/blocks/regions, `Reserved` fields, and ranges outside
    /// every region) that the caller must materialize as zero bytes. The
    /// concatenation of chunk payloads always equals `len` bytes.
    fn read_iter(&self, addr: u16, len: u16) -> ReadIter<'_, Self>
    where
        Self: Sized,
    {
        router_read_iter(self, addr, len)
    }

    fn write_bytes(&self, addr: u16, src: &[u8], staged: &mut StagedWrites) -> Result<(), Error>
    where
        Self: Sized,
    {
        let snap = staged.snapshot();
        staged.push(addr, src)?;
        self.write_bytes_iter(addr, staged, &snap)
    }

    /// Validate + commit a write to `[addr, addr+total)` where `total` is the
    /// summed length of staged chunks at indices >= `snap`. Zero-copy: the
    /// chunks live in `staged` from prior `push`es; no scratch buffer.
    /// On error: rewinds `staged` to `snap` and returns `Err`.
    /// On success: commits to the backing region and rewinds to `snap`.
    fn write_bytes_iter(
        &self,
        addr: u16,
        staged: &mut StagedWrites,
        snap: &Snapshot,
    ) -> Result<(), Error>
    where
        Self: Sized,
    {
        router_write_bytes_iter(self, addr, staged, snap)
    }

    fn stage_bytes(&self, addr: u16, src: &[u8], staged: &mut StagedWrites) -> Result<(), Error>
    where
        Self: Sized,
    {
        let snap = staged.snapshot();
        staged.push(addr, src)?;
        self.stage_bytes_iter(addr, staged, &snap)
    }

    /// Validate (without committing) a write to `[addr, addr+total)` where
    /// `total` is the summed length of staged chunks at indices >= `snap`.
    /// On error: rewinds `staged` to `snap`.
    /// On success: chunks remain staged for a later `commit_staged` (RegWrite
    /// flow). Zero-copy: chunks live in `staged` from prior `push`es.
    fn stage_bytes_iter(
        &self,
        addr: u16,
        staged: &mut StagedWrites,
        snap: &Snapshot,
    ) -> Result<(), Error>
    where
        Self: Sized,
    {
        router_stage_bytes_iter(self, addr, staged, snap)
    }

    fn commit_staged(&self, staged: &mut StagedWrites)
    where
        Self: Sized,
    {
        // SAFETY: caller upholds Router's single-writer contract.
        unsafe { commit_staged_range(self, staged, &Snapshot::ZERO) };
        staged.clear();
    }
}

struct RegionRef {
    base: *mut u8,
    def: &'static RegionDesc,
}

fn region_for<R: Router>(router: &R, addr: u16, end: usize) -> Option<RegionRef> {
    let def = router
        .regions()
        .iter()
        .copied()
        .find(|d| range_in(addr, end, d.addr, d.size as usize))?;
    Some(RegionRef {
        base: router.region_base(def)?,
        def,
    })
}

/// DXL 2.0 reads are memory-like: bytes inside any region's gap (between
/// blocks, inside a block, or past the last block) AND bytes outside every
/// region come back as 0. Only writes treat unmapped addresses as errors.
///
/// Implemented atop [`router_read_iter`]: each `Copy` chunk memcpys from
/// region storage into `dst`, each `Zero` chunk fills the corresponding
/// run with zeros. The iterator partitions `[addr, addr+dst.len())` so the
/// chunk payloads tile `dst` exactly once.
pub(crate) fn router_read_bytes<R: Router + ?Sized>(
    router: &R,
    addr: u16,
    dst: &mut [u8],
) -> Result<(), Error> {
    let mut pos = 0;
    for chunk in router_read_iter(router, addr, dst.len() as u16) {
        match chunk {
            ReadChunk::Copy(src) => {
                let n = src.len();
                dst[pos..pos + n].copy_from_slice(src);
                pos += n;
            }
            ReadChunk::Zero(n) => {
                let n = n as usize;
                dst[pos..pos + n].fill(0);
                pos += n;
            }
        }
    }
    Ok(())
}

/// One run of consecutive bytes from the descriptor walk: either a slice
/// borrowed from region storage (`Copy`), or a phantom span (`Zero`) the
/// caller emits as `n` zero bytes. See [`Router::read_iter`].
#[derive(Debug, PartialEq, Eq)]
pub enum ReadChunk<'a> {
    Copy(&'a [u8]),
    Zero(u16),
}

/// Yields the next descriptor segment of the request range. State holds
/// region/block/field cursors and the cached region base; advances
/// monotonically through `req_lo`. See [`Router::read_iter`].
pub struct ReadIter<'a, R: Router + ?Sized> {
    router: &'a R,
    req_lo: u32,
    req_hi: u32,
    region_idx: usize,
    block_idx: usize,
    field_idx: usize,
    /// Cached `region_base()` for `regions()[region_idx]`. `Some(None)` means
    /// the region descriptor has no backing storage (treated as all zeros);
    /// `None` means "not yet resolved for current region_idx".
    region_base: Option<Option<*const u8>>,
}

pub(crate) fn router_read_iter<R: Router + ?Sized>(
    router: &R,
    addr: u16,
    len: u16,
) -> ReadIter<'_, R> {
    let req_lo = addr as u32;
    let req_hi = req_lo + len as u32;
    ReadIter {
        router,
        req_lo,
        req_hi,
        region_idx: 0,
        block_idx: 0,
        field_idx: 0,
        region_base: None,
    }
}

impl<'a, R: Router + ?Sized> Iterator for ReadIter<'a, R> {
    type Item = ReadChunk<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let regions = self.router.regions();
        loop {
            if self.req_lo >= self.req_hi {
                return None;
            }
            // Past all regions: remaining range is phantom.
            let Some(region) = regions.get(self.region_idx).copied() else {
                let len = (self.req_hi - self.req_lo) as u16;
                self.req_lo = self.req_hi;
                return Some(ReadChunk::Zero(len));
            };
            let r_lo = region.addr as u32;
            let r_hi = r_lo + region.size as u32;
            if r_hi <= self.req_lo {
                self.advance_region();
                continue;
            }
            if r_lo > self.req_lo {
                // Gap before this region.
                let end = r_lo.min(self.req_hi);
                let len = (end - self.req_lo) as u16;
                self.req_lo = end;
                return Some(ReadChunk::Zero(len));
            }
            // req_lo ∈ [r_lo, r_hi). Resolve the region base lazily.
            let base = *self
                .region_base
                .get_or_insert_with(|| self.router.region_base(region).map(|p| p as *const u8));
            let Some(base) = base else {
                // Region has no backing storage; emit Zero for its overlap.
                let end = r_hi.min(self.req_hi);
                let len = (end - self.req_lo) as u16;
                self.req_lo = end;
                self.advance_region();
                return Some(ReadChunk::Zero(len));
            };
            // Walk blocks.
            let Some(block) = region.blocks.get(self.block_idx).copied() else {
                // Past last block: trailing gap inside this region.
                let end = r_hi.min(self.req_hi);
                let len = (end - self.req_lo) as u16;
                self.req_lo = end;
                self.advance_region();
                return Some(ReadChunk::Zero(len));
            };
            let b_lo = block.addr as u32;
            let b_hi = b_lo + block.size as u32;
            if b_hi <= self.req_lo {
                self.block_idx += 1;
                self.field_idx = 0;
                continue;
            }
            if b_lo > self.req_lo {
                // Inter-block gap.
                let end = b_lo.min(self.req_hi).min(r_hi);
                let len = (end - self.req_lo) as u16;
                self.req_lo = end;
                return Some(ReadChunk::Zero(len));
            }
            // Walk fields.
            let Some(field) = block.fields.get(self.field_idx).copied() else {
                // Past last field: trailing intra-block gap.
                let end = b_hi.min(self.req_hi);
                let len = (end - self.req_lo) as u16;
                self.req_lo = end;
                self.block_idx += 1;
                self.field_idx = 0;
                return Some(ReadChunk::Zero(len));
            };
            let f_lo = field.addr as u32;
            let f_hi = f_lo + field.size as u32;
            if f_hi <= self.req_lo {
                self.field_idx += 1;
                continue;
            }
            if f_lo > self.req_lo {
                // Inter-field gap inside block.
                let end = f_lo.min(self.req_hi).min(b_hi);
                let len = (end - self.req_lo) as u16;
                self.req_lo = end;
                return Some(ReadChunk::Zero(len));
            }
            // req_lo ∈ [f_lo, f_hi). Yield Copy (Ro/Rw) or Zero (Reserved).
            let end = f_hi.min(self.req_hi).min(b_hi);
            let chunk_len = (end - self.req_lo) as u16;
            self.req_lo = end;
            self.field_idx += if end >= f_hi { 1 } else { 0 };
            match field.access {
                Access::Reserved => return Some(ReadChunk::Zero(chunk_len)),
                Access::Ro | Access::Rw => {
                    let struct_off = block.struct_offset as usize
                        + field.struct_offset as usize
                        + ((end - chunk_len as u32) - f_lo) as usize;
                    // SAFETY: `struct_off + chunk_len` ≤ block.size ≤ region.size, which
                    // is `size_of` the backing region struct; descriptors are construction-
                    // verified. Read-side aliasing follows the existing Router contract.
                    let slice = unsafe {
                        core::slice::from_raw_parts(base.add(struct_off), chunk_len as usize)
                    };
                    return Some(ReadChunk::Copy(slice));
                }
            }
        }
    }
}

impl<R: Router + ?Sized> ReadIter<'_, R> {
    #[inline]
    fn advance_region(&mut self) {
        self.region_idx += 1;
        self.block_idx = 0;
        self.field_idx = 0;
        self.region_base = None;
    }
}

pub(crate) fn router_stage_bytes_iter<R: Router>(
    router: &R,
    addr: u16,
    staged: &mut StagedWrites,
    snap: &Snapshot,
) -> Result<(), Error> {
    let total: usize = staged.iter_from(snap).map(|(_, d)| d.len()).sum();
    if total == 0 {
        return Ok(());
    }
    let end = match (addr as usize).checked_add(total) {
        Some(e) => e,
        None => {
            staged.rewind_to(*snap);
            return Err(Error::OutOfRange);
        }
    };
    let r = match region_for(router, addr, end) {
        Some(r) => r,
        None => {
            staged.rewind_to(*snap);
            return Err(Error::OutOfRange);
        }
    };
    let result = walk_fields(addr, total, r.def.blocks, true, |_, _, _, _| {})
        .and_then(|()| run_field_validators(router, staged, *snap, addr, total, r.def.blocks))
        .and_then(|()| run_block_validators(router, staged, *snap, addr, total, r.def.blocks))
        .and_then(|()| run_region_validators(router, staged, *snap, r.def.validators));
    if result.is_err() {
        staged.rewind_to(*snap);
    }
    result
}

pub(crate) fn router_write_bytes_iter<R: Router>(
    router: &R,
    addr: u16,
    staged: &mut StagedWrites,
    snap: &Snapshot,
) -> Result<(), Error> {
    router_stage_bytes_iter(router, addr, staged, snap)?;
    // SAFETY: caller upholds Router's single-writer contract.
    unsafe { commit_staged_range(router, staged, snap) };
    staged.rewind_to(*snap);
    Ok(())
}

/// SAFETY: caller holds the region's single-writer guarantee.
pub(crate) unsafe fn commit_staged_range<R: Router>(
    router: &R,
    staged: &StagedWrites,
    snap: &Snapshot,
) {
    for (abs_addr, data) in staged.iter_from(snap) {
        let end = abs_addr as usize + data.len();
        let Some(r) = region_for(router, abs_addr, end) else {
            debug_assert!(false, "staged entry resolved to no region at commit time");
            continue;
        };
        unsafe { commit_chunk(r.base, abs_addr, data, r.def.blocks) };
    }
}

fn range_in(addr: u16, end: usize, base: u16, size: usize) -> bool {
    let base = base as usize;
    (addr as usize) >= base && end <= base + size
}

/// Walk fields covering `[abs_start, +len)`. Calls `on_chunk(access, struct_off, buf_off, chunk_len)`
/// for each overlapping byte run (`struct_off` is region-relative; `access` lets the read path
/// zero-fill `Reserved` chunks instead of memcpy'ing storage).
///
/// `require_rw=true` (write path): any gap (inter-block, intra-block, or past
/// the last block) and any non-RW field returns `AccessError`. `require_rw=false`
/// (read path): gaps are skipped without `on_chunk`; the caller pre-zeros `dst`
/// so the skipped bytes remain 0 per the DXL 2.0 memory-shaped read contract.
/// Blocks and fields within each block must be sorted by `addr` and non-overlapping.
fn walk_fields(
    abs_start: u16,
    len: usize,
    blocks: &[BlockDesc],
    require_rw: bool,
    mut on_chunk: impl FnMut(Access, usize, usize, usize),
) -> Result<(), Error> {
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
            if require_rw {
                return Err(Error::AccessError);
            }
            let new_lo = b_lo.min(req_hi);
            buf_pos += new_lo - req_lo;
            req_lo = new_lo;
            if req_lo >= req_hi {
                break;
            }
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
            if f_lo > req_lo {
                if require_rw {
                    return Err(Error::AccessError);
                }
                let new_lo = f_lo.min(req_hi).min(b_hi);
                buf_pos += new_lo - req_lo;
                req_lo = new_lo;
                if req_lo >= req_hi.min(b_hi) {
                    break;
                }
            }
            if require_rw && field.access != Access::Rw {
                return Err(Error::AccessError);
            }
            let chunk_hi = req_hi.min(f_hi);
            let chunk_len = chunk_hi - req_lo;
            let struct_off = block_struct + field.struct_offset as usize + (req_lo - f_lo);
            on_chunk(field.access, struct_off, buf_pos, chunk_len);
            buf_pos += chunk_len;
            req_lo = chunk_hi;
        }
        // Trailing gap inside this block (past the last field, before b_hi).
        let block_hi = req_hi.min(b_hi);
        if req_lo < block_hi {
            if require_rw {
                return Err(Error::AccessError);
            }
            buf_pos += block_hi - req_lo;
            req_lo = block_hi;
        }
    }
    if req_lo < req_hi && require_rw {
        return Err(Error::AccessError);
    }
    Ok(())
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
        |_access, struct_off, data_pos, chunk_len| unsafe {
            core::ptr::copy_nonoverlapping(
                data_ptr.add(data_pos),
                region_base.add(struct_off),
                chunk_len,
            );
        },
    );
}
