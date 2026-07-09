use crate::rules;
use crate::stage::{Snapshot, StagedWrites};
use crate::{Error, ValidationKind};

/// Compile-time description of a flat register map. Implemented by the Table
/// derive; hand-implemented in tests.
///
/// # Safety
/// `base` must return a valid, non-null, aligned pointer to `SIZE` bytes that
/// stays valid for the lifetime of `&self`; callers uphold a single-writer
/// contract like `RegionStorageRaw`.
pub trait RegisterMap {
    const SIZE: usize;
    /// Writable-byte bitmask, bit `i % 32` of word `i / 32` set = byte `i`
    /// host-writable. `len == SIZE.div_ceil(32)`.
    const WRITABLE: &'static [u32];
    /// Sorted by `base`, non-overlapping.
    const SECTIONS: &'static [SectionMeta];
    /// All sections' compare rules concatenated in section order (table-absolute
    /// addresses); each section owns the `cmp_rules` sub-slice of this array.
    const CMP_RULES: &'static [rules::CmpRule] = &[];
    /// All sections' enum/bool rules concatenated in section order; each section
    /// owns the `allowed_rules` sub-slice of this array.
    const ALLOWED_RULES: &'static [rules::AllowedRule] = &[];
    fn base(&self) -> *mut u8;
}

pub struct SectionMeta {
    pub base: u16,
    pub size: u16,
    /// `[start, end)` range into `RegisterMap::CMP_RULES` for this section.
    pub cmp_rules: (u16, u16),
    /// `[start, end)` range into `RegisterMap::ALLOWED_RULES` for this section.
    pub allowed_rules: (u16, u16),
    /// Byte offset of a lock register: when it reads nonzero (through the
    /// overlay), writes into this section fail with `ValidationError(Locked)`.
    pub write_lock: Option<u16>,
}

/// Live base pointer + size plus one pending overlay, so a check sees the value
/// about to be committed on top of live storage. The overlay is up to two
/// contiguous ranges — `o2` logically follows `o1` at `overlay_addr + o1.len()`
/// — because a ring-seam write arrives as head + tail (`o2` empty otherwise).
pub struct View<'a> {
    base: *const u8,
    size: usize,
    overlay_addr: u16,
    o1: &'a [u8],
    o2: &'a [u8],
}

impl<'a> View<'a> {
    /// Build a view over `[base, base+size)` with one pending overlay of
    /// `overlay` bytes starting at `overlay_addr`.
    ///
    /// # Safety
    /// `base` must point to `size` valid bytes that outlive the view; the caller
    /// upholds `RegisterMap`'s single-writer contract for shared reads.
    pub fn new(base: *const u8, size: usize, overlay_addr: u16, overlay: &'a [u8]) -> View<'a> {
        View::new_split(base, size, overlay_addr, overlay, &[])
    }

    /// As [`View::new`] but with the overlay split across the ring seam: `o1`
    /// starts at `overlay_addr`, `o2` continues at `overlay_addr + o1.len()`.
    ///
    /// # Safety
    /// Same contract as [`View::new`].
    pub fn new_split(
        base: *const u8,
        size: usize,
        overlay_addr: u16,
        o1: &'a [u8],
        o2: &'a [u8],
    ) -> View<'a> {
        View {
            base,
            size,
            overlay_addr,
            o1,
            o2,
        }
    }

    /// Load `[addr, addr+len)` (`len <= 4`) into a `[u8; 4]` as little-endian
    /// bytes, taking each byte from the pending overlay where it lands inside
    /// `[overlay_addr, overlay_addr+o1.len()+o2.len())` and from live storage
    /// otherwise. Returns `OutOfRange` when `[addr, addr+len)` leaves the map.
    /// Bounds-checked once up front; the per-byte picks then need no further
    /// checks. Bytes past `len` in the returned array are zero.
    ///
    /// Rule reads are almost always fully outside the overlay (a compare's
    /// RHS register, a lock byte) or fully inside `o1` (the field being
    /// written) — those take straight byte loads; the per-byte pick pays
    /// only on seam splits and overlay-edge straddles (bench: the pick chain
    /// ran ~6.8 µs per load on flash-resident code, the write path's
    /// hottest function). Outlined: inlining at the (deduped) rule-body
    /// sites measured zero gain for +146 B — the cost is the loads, not the
    /// call.
    #[inline(never)]
    pub fn read_fixed(&self, addr: u16, len: usize) -> Result<[u8; 4], Error> {
        let lo = addr as usize;
        let hi = match lo.checked_add(len) {
            Some(h) => h,
            None => return Err(Error::OutOfRange),
        };
        if hi > self.size {
            return Err(Error::OutOfRange);
        }
        let o_lo = self.overlay_addr as usize;
        let split = o_lo + self.o1.len();
        let o_hi = split + self.o2.len();
        if hi <= o_lo || lo >= o_hi {
            // SAFETY: `hi <= size` bytes of live storage; the caller upholds
            // RegisterMap's single-writer contract, so this shared read does
            // not race a commit.
            return Ok(unsafe { load_le4(self.base.add(lo), len) });
        }
        if lo >= o_lo && hi <= split {
            // SAFETY: `[lo, hi)` lies inside `[o_lo, split)`, so the offset
            // span is in `o1`.
            return Ok(unsafe { load_le4(self.o1.as_ptr().add(lo - o_lo), len) });
        }
        Ok(self.read_mixed(lo, len, o_lo, split, o_hi))
    }

    /// Overlay-edge straddles and seam splits: pick each byte from live
    /// storage, `o1`, or `o2`. Bounds pre-checked by [`Self::read_fixed`].
    #[cold]
    fn read_mixed(&self, lo: usize, len: usize, o_lo: usize, split: usize, o_hi: usize) -> [u8; 4] {
        let mut out = [0u8; 4];
        for (i, slot) in out.iter_mut().take(len).enumerate() {
            let a = lo + i;
            *slot = if a >= o_lo && a < split {
                // Bounds guaranteed by `a < split = o_lo + o1.len()`; get keeps
                // the never-panic contract regardless.
                self.o1.get(a - o_lo).copied().unwrap_or(0)
            } else if a >= split && a < o_hi {
                self.o2.get(a - split).copied().unwrap_or(0)
            } else {
                // SAFETY: `a < hi <= size` bytes of live storage; the caller
                // upholds RegisterMap's single-writer contract, so this shared
                // read does not race a commit.
                unsafe { *self.base.add(a) }
            };
        }
        out
    }
}

/// Load `len` bytes (capped at 4) from `p` into a zero-padded `[u8; 4]` with
/// unrolled byte loads — no per-byte range compares.
///
/// # Safety
/// `p` must be valid for `len.min(4)` byte reads.
unsafe fn load_le4(p: *const u8, len: usize) -> [u8; 4] {
    let mut out = [0u8; 4];
    // SAFETY: caller guarantees `len.min(4)` readable bytes at `p`.
    unsafe {
        match len {
            4.. => out = p.cast::<[u8; 4]>().read_unaligned(),
            2 => out[..2].copy_from_slice(&p.cast::<[u8; 2]>().read_unaligned()),
            1 => out[0] = p.read(),
            3 => {
                out[..2].copy_from_slice(&p.cast::<[u8; 2]>().read_unaligned());
                out[2] = p.add(2).read();
            }
            0 => {}
        }
    }
    out
}

/// Validate `[addr, addr+o1.len()+o2.len())` against the flat map: bounds →
/// writable mask → rules → lock, in that precedence. The view is live storage
/// plus this op's `o1`/`o2` overlay only (previously staged entries are not
/// visible). `o2` continues after `o1` at the ring seam; it is empty for a
/// contiguous write.
fn validate<M: RegisterMap + ?Sized>(m: &M, addr: u16, o1: &[u8], o2: &[u8]) -> Result<(), Error> {
    let total = o1.len() + o2.len();
    if total == 0 {
        return Ok(());
    }
    let lo = addr as usize;
    let hi = match lo.checked_add(total) {
        Some(h) => h,
        None => return Err(Error::OutOfRange),
    };
    if hi > M::SIZE {
        return Err(Error::OutOfRange);
    }
    // Writable-mask check a word at a time: interior words must equal `!0`; the
    // (up to two) edge words test only their covered bit-spans. `src` is non-
    // empty here, so `hi - 1` never underflows.
    let first = lo / 32;
    let last = (hi - 1) / 32;
    for wi in first..=last {
        let word_lo = wi * 32;
        let start_bit = lo.saturating_sub(word_lo);
        let end_bit = (hi - word_lo).min(32);
        // u32 shifts only — a runtime u64 shift is an __ashldi3 libcall on
        // RV32E. `start_bit < 32` always; branch on the one amount that would
        // overflow a u32 shift.
        let covered = if end_bit == 32 {
            !0u32
        } else {
            (1u32 << end_bit) - 1
        };
        let mask = covered & !((1u32 << start_bit) - 1);
        let w = M::WRITABLE.get(wi).copied().unwrap_or(0);
        if (w & mask) != mask {
            return Err(Error::AccessError);
        }
    }

    let view = View::new_split(m.base() as *const u8, M::SIZE, addr, o1, o2);

    // Checks across every overlapping section first, so a bad value in a locked
    // section reports its own kind (e.g. Enum), not Locked. Within a section
    // allowed rules now run before compare rules (previously interleaved in
    // field order — only observable when one write spans multiple invalid
    // fields of different kinds; not spec-pinned).
    for sec in M::SECTIONS {
        if !overlaps(sec.base as usize, sec.size as usize, lo, hi) {
            continue;
        }
        // Both stripes are address-ordered (blocks and fields emit in
        // declaration = address order), so the scan ends at the first rule
        // past the write.
        let (a0, a1) = sec.allowed_rules;
        if let Some(rs) = M::ALLOWED_RULES.get(a0 as usize..a1 as usize) {
            for r in rs {
                if r.addr as usize >= hi {
                    break;
                }
                if overlaps(r.addr as usize, 1, lo, hi) {
                    rules::check_allowed(&view, r.addr, r.allowed)?;
                }
            }
        }
        let (c0, c1) = sec.cmp_rules;
        if let Some(rs) = M::CMP_RULES.get(c0 as usize..c1 as usize) {
            for r in rs {
                if r.addr as usize >= hi {
                    break;
                }
                let width = (r.spec & 0xF) as usize;
                if overlaps(r.addr as usize, width, lo, hi) {
                    let rhs = if r.spec & rules::SPEC_RHS_REG != 0 {
                        rules::Rhs::Reg(r.val as u16)
                    } else {
                        rules::Rhs::Imm(r.val as i32)
                    };
                    rules::check_cmp(&view, r.addr, r.spec, rhs)?;
                }
            }
        }
    }
    for sec in M::SECTIONS {
        if !overlaps(sec.base as usize, sec.size as usize, lo, hi) {
            continue;
        }
        if let Some(lock) = sec.write_lock {
            let b = view.read_fixed(lock, 1)?;
            if b[0] != 0 {
                return Err(Error::ValidationError(ValidationKind::Locked));
            }
        }
    }
    Ok(())
}

/// `[a_lo, a_lo+a_len)` overlaps `[lo, hi)` at all.
fn overlaps(a_lo: usize, a_len: usize, lo: usize, hi: usize) -> bool {
    a_lo < hi && a_lo + a_len > lo
}

/// Copy every staged entry pushed since `snap` into live storage (out-of-bounds
/// entries skipped). The caller truncates the buffer afterward per its commit
/// semantics (`clear` for a full COMMIT, `revert_to(snap)` for a verdict).
fn apply_from<M: RegisterMap + ?Sized>(m: &M, staged: &StagedWrites, snap: &Snapshot) {
    let base = m.base();
    for (addr, data) in staged.iter_from(snap) {
        let end = addr as usize + data.len();
        if end > M::SIZE {
            continue;
        }
        // SAFETY: bounds guarded above; the caller upholds RegisterMap's
        // single-writer contract.
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(addr as usize), data.len());
        }
    }
}

/// Extension surface over a `RegisterMap`: memory-like `read`/`write` plus the
/// staged (RegWrite) flow. Blanket-implemented for every map.
pub trait RegisterFile: RegisterMap {
    fn read(&self, addr: u16, len: u16) -> Result<&[u8], Error> {
        let hi = addr as u32 + len as u32;
        if hi > Self::SIZE as u32 {
            return Err(Error::OutOfRange);
        }
        if len == 0 {
            return Ok(&[]);
        }
        // SAFETY: `[addr, addr+len)` lies in `SIZE` bytes; the caller upholds
        // RegisterMap's single-writer contract, so this borrow does not race a
        // commit for its lifetime.
        let slice =
            unsafe { core::slice::from_raw_parts(self.base().add(addr as usize), len as usize) };
        Ok(slice)
    }

    fn write(&self, addr: u16, src: &[u8]) -> Result<(), Error> {
        self.write_split(addr, src, &[])
    }

    /// As [`write`](RegisterFile::write) but with the source split across the
    /// ring seam: `head` at `addr`, `tail` continuing after it.
    fn write_split(&self, addr: u16, head: &[u8], tail: &[u8]) -> Result<(), Error> {
        validate(self, addr, head, tail)?;
        // SAFETY: validate confirmed `[addr, addr+head.len()+tail.len())` is in
        // bounds; the caller upholds RegisterMap's single-writer contract.
        unsafe {
            let base = self.base();
            core::ptr::copy_nonoverlapping(head.as_ptr(), base.add(addr as usize), head.len());
            core::ptr::copy_nonoverlapping(
                tail.as_ptr(),
                base.add(addr as usize + head.len()),
                tail.len(),
            );
        }
        Ok(())
    }

    fn stage(&self, addr: u16, src: &[u8], staged: &mut StagedWrites) -> Result<(), Error> {
        self.stage_split(addr, src, &[], staged)
    }

    /// As [`stage`](RegisterFile::stage) but with the source split across the
    /// ring seam.
    fn stage_split(
        &self,
        addr: u16,
        head: &[u8],
        tail: &[u8],
        staged: &mut StagedWrites,
    ) -> Result<(), Error> {
        validate(self, addr, head, tail)?;
        staged.push_split(addr, head, tail)
    }

    fn commit_staged(&self, staged: &mut StagedWrites) {
        apply_from(self, staged, &Snapshot::ZERO);
        staged.clear();
    }

    /// Apply only the entries pushed since `snap`, then truncate the buffer back
    /// to it — a pending write's verdict commit, leaving any pre-`snap` HOLD
    /// entries intact for a later real COMMIT.
    fn commit_from(&self, staged: &mut StagedWrites, snap: &Snapshot) {
        apply_from(self, staged, snap);
        staged.revert_to(snap);
    }
}

impl<M: RegisterMap> RegisterFile for M {}

#[cfg(test)]
mod tests;
