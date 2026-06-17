use crate::desc::{Access, BlockDesc, Error, RegionDesc};
use crate::stage::{Snapshot, StagedWrites};
use crate::validate::{run_block_validators, run_field_validators, run_region_validators};

/// `write_bytes`/`commit_staged` form a transient `&mut` to the region via the
/// pointer from `region_base`; caller must hold its single-writer guarantee.
///
/// Default methods are gated `where Self: Sized` so they aren't callable via
/// `&dyn Router` — `router_*` free fns take `&dyn Router` internally, and the
/// `Sized` bound prevents the trait-object recursion that would otherwise result.
/// Callers wanting to abstract over routers must take `&S: Router` generically,
/// not `&dyn Router`.
pub trait Router {
    fn regions(&self) -> &'static [&'static RegionDesc];
    fn region_base(&self, desc: &RegionDesc) -> Option<*mut u8>;

    fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), Error>
    where
        Self: Sized,
    {
        router_read_bytes(self, addr, dst)
    }

    fn write_bytes(&self, addr: u16, src: &[u8], staged: &mut StagedWrites) -> Result<(), Error>
    where
        Self: Sized,
    {
        let snap = staged.snapshot();
        router_stage_bytes(self, addr, src, staged)?;
        // SAFETY: caller upholds Router's single-writer contract.
        unsafe { commit_staged_range(self, staged, &snap) };
        staged.rewind_to(snap);
        Ok(())
    }

    fn stage_bytes(&self, addr: u16, src: &[u8], staged: &mut StagedWrites) -> Result<(), Error>
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
        unsafe { commit_staged_range(self, staged, &Snapshot::ZERO) };
        staged.clear();
    }
}

struct RegionRef {
    base: *mut u8,
    def: &'static RegionDesc,
}

fn region_for(router: &dyn Router, addr: u16, end: usize) -> Option<RegionRef> {
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
pub(crate) fn router_read_bytes(
    router: &dyn Router,
    addr: u16,
    dst: &mut [u8],
) -> Result<(), Error> {
    if dst.is_empty() {
        return Ok(());
    }
    dst.fill(0);
    let req_lo = addr as usize;
    let req_hi = req_lo + dst.len();
    for region in router.regions() {
        let r_lo = region.addr as usize;
        let r_hi = r_lo + region.size as usize;
        let lo = req_lo.max(r_lo);
        let hi = req_hi.min(r_hi);
        if lo >= hi {
            continue;
        }
        let Some(base) = router.region_base(region) else {
            continue;
        };
        let slice_off = lo - req_lo;
        let slice_len = hi - lo;
        // SAFETY: lo..hi ⊆ region.size, so base + (lo - r_lo) and base +
        // (hi - r_lo) are both in-bounds; descriptor sizes verified at
        // construction.
        unsafe {
            walk_read(
                base,
                lo as u16,
                &mut dst[slice_off..slice_off + slice_len],
                region.blocks,
            )?;
        }
    }
    Ok(())
}

pub(crate) fn router_stage_bytes(
    router: &dyn Router,
    addr: u16,
    src: &[u8],
    staged: &mut StagedWrites,
) -> Result<(), Error> {
    if src.is_empty() {
        return Ok(());
    }
    let end = (addr as usize)
        .checked_add(src.len())
        .ok_or(Error::OutOfRange)?;
    let r = region_for(router, addr, end).ok_or(Error::OutOfRange)?;
    let snap = staged.snapshot();
    let result = stage_write(addr, src, r.def.blocks, staged)
        .and_then(|()| run_field_validators(router, staged, snap, addr, src.len(), r.def.blocks))
        .and_then(|()| run_block_validators(router, staged, snap, addr, src.len(), r.def.blocks))
        .and_then(|()| run_region_validators(router, staged, snap, r.def.validators));
    if result.is_err() {
        staged.rewind_to(snap);
    }
    result
}

/// SAFETY: caller holds the region's single-writer guarantee.
pub(crate) unsafe fn commit_staged_range(
    router: &dyn Router,
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

unsafe fn walk_read(
    region_base: *const u8,
    abs_start: u16,
    dst: &mut [u8],
    blocks: &[BlockDesc],
) -> Result<(), Error> {
    dst.fill(0);
    let dst_ptr = dst.as_mut_ptr();
    walk_fields(
        abs_start,
        dst.len(),
        blocks,
        false,
        |access, struct_off, dst_pos, chunk_len| unsafe {
            match access {
                Access::Reserved => {
                    // Already zeroed by dst.fill(0) above.
                }
                Access::Ro | Access::Rw => {
                    core::ptr::copy_nonoverlapping(
                        region_base.add(struct_off),
                        dst_ptr.add(dst_pos),
                        chunk_len,
                    );
                }
            }
        },
    )
}

/// Validate that `src` lies entirely in RW fields with no gaps; stage as one entry.
pub(crate) fn stage_write(
    abs_addr: u16,
    src: &[u8],
    blocks: &[BlockDesc],
    staged: &mut StagedWrites,
) -> Result<(), Error> {
    walk_fields(abs_addr, src.len(), blocks, true, |_, _, _, _| {})?;
    staged.push(abs_addr, src)
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
