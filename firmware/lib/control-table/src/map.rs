use crate::rules::Rule;
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
    fn base(&self) -> *mut u8;
}

pub struct SectionMeta {
    pub base: u16,
    pub size: u16,
    /// Sorted by offset; offsets absolute and inside the section.
    pub rules: &'static [Rule],
    /// Byte offset of a lock register: when it reads nonzero (through the
    /// overlay), writes into this section fail with `ValidationError(Locked)`.
    pub write_lock: Option<u16>,
}

/// Live base pointer + size plus one pending `(addr, bytes)` overlay, so a
/// rule sees the value about to be committed on top of live storage.
pub struct View<'a> {
    base: *const u8,
    size: usize,
    overlay_addr: u16,
    overlay: &'a [u8],
}

impl View<'_> {
    /// Load `[addr, addr+len)` (`len <= 4`) into a `[u8; 4]`, taking each byte
    /// from the pending overlay where it lands inside `[overlay_addr,
    /// overlay_addr+overlay.len())` and from live storage otherwise. Bounds-
    /// checked once up front (same `OutOfRange` conditions as a slice read); the
    /// per-byte picks then need no further checks.
    #[inline(always)]
    pub(crate) fn read_fixed(&self, addr: u16, len: usize) -> Result<[u8; 4], Error> {
        let lo = addr as usize;
        let hi = match lo.checked_add(len) {
            Some(h) => h,
            None => return Err(Error::OutOfRange),
        };
        if hi > self.size {
            return Err(Error::OutOfRange);
        }
        let o_lo = self.overlay_addr as usize;
        let o_hi = o_lo + self.overlay.len();
        let mut out = [0u8; 4];
        for (i, slot) in out.iter_mut().take(len).enumerate() {
            let a = lo + i;
            *slot = if a >= o_lo && a < o_hi {
                // `a - o_lo < overlay.len()` by the bounds above; get keeps the
                // never-panic contract regardless.
                self.overlay.get(a - o_lo).copied().unwrap_or(0)
            } else {
                // SAFETY: `a < hi <= size` bytes of live storage; the caller
                // upholds RegisterMap's single-writer contract, so this shared
                // read does not race a commit.
                unsafe { *self.base.add(a) }
            };
        }
        Ok(out)
    }
}

/// Validate `[addr, addr+src.len())` against the flat map: bounds → writable
/// mask → rules → lock, in that precedence. The view is live storage plus this
/// op's `src` only (previously staged entries are not visible).
fn validate<M: RegisterMap + ?Sized>(m: &M, addr: u16, src: &[u8]) -> Result<(), Error> {
    if src.is_empty() {
        return Ok(());
    }
    let lo = addr as usize;
    let hi = match lo.checked_add(src.len()) {
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
        // Build in u64 so the shift amount never reaches 32 (Rust UB on u32).
        let mask = (((1u64 << end_bit) - 1) & !((1u64 << start_bit) - 1)) as u32;
        let w = M::WRITABLE.get(wi).copied().unwrap_or(0);
        if (w & mask) != mask {
            return Err(Error::AccessError);
        }
    }

    let view = View {
        base: m.base() as *const u8,
        size: M::SIZE,
        overlay_addr: addr,
        overlay: src,
    };

    // Rules across every overlapping section first, so a bad value in a locked
    // section reports its own kind (e.g. Enum), not Locked.
    for sec in M::SECTIONS {
        if !overlaps(sec.base as usize, sec.size as usize, lo, hi) {
            continue;
        }
        for rule in sec.rules {
            if overlaps(rule.offset as usize, rule.width as usize, lo, hi) {
                rule.eval(&view)?;
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
        validate(self, addr, src)?;
        // SAFETY: validate confirmed `[addr, addr+len)` is in bounds; the caller
        // upholds RegisterMap's single-writer contract.
        unsafe {
            core::ptr::copy_nonoverlapping(src.as_ptr(), self.base().add(addr as usize), src.len());
        }
        Ok(())
    }

    fn stage(&self, addr: u16, src: &[u8], staged: &mut StagedWrites) -> Result<(), Error> {
        validate(self, addr, src)?;
        staged.push(addr, src)
    }

    fn commit_staged(&self, staged: &mut StagedWrites) {
        let base = self.base();
        for (addr, data) in staged.iter_from(&Snapshot::ZERO) {
            let end = addr as usize + data.len();
            if end > Self::SIZE {
                continue;
            }
            // SAFETY: bounds guarded above; the caller upholds RegisterMap's
            // single-writer contract.
            unsafe {
                core::ptr::copy_nonoverlapping(data.as_ptr(), base.add(addr as usize), data.len());
            }
        }
        staged.clear();
    }
}

impl<M: RegisterMap> RegisterFile for M {}

#[cfg(test)]
mod tests;
