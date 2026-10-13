use crate::Error;
use crate::rules;
use crate::stage::StagedWrites;

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
    /// Every compare rule, address-ordered (table-absolute addresses).
    const CMP_RULES: &'static [rules::CmpRule] = &[];
    /// Every enum/bool rule, address-ordered.
    const ALLOWED_RULES: &'static [rules::AllowedRule] = &[];
    /// Per [`RULE_CHUNK`] bytes, the first `CMP_RULES` index a write starting
    /// there can overlap ([`cmp_from`]). Empty scans from the first rule.
    const CMP_FROM: &'static [u8] = &[];
    /// As `CMP_FROM`, into `ALLOWED_RULES` ([`allowed_from`]).
    const ALLOWED_FROM: &'static [u8] = &[];
    fn base(&self) -> *mut u8;
}

pub struct SectionMeta {
    pub base: u16,
    pub size: u16,
}

/// Bytes of map per `*_FROM` entry.
pub const RULE_CHUNK: usize = 32;

/// A map's `CMP_FROM`: entry `k` is the first rule that ends past byte
/// `k * RULE_CHUNK`, so no earlier rule can overlap a write starting in chunk
/// `k`. Const-evaluated by the Table derive.
pub const fn cmp_from<const N: usize>(rules: &[rules::CmpRule]) -> [u8; N] {
    assert!(rules.len() <= u8::MAX as usize, "rule index must fit a u8");
    let mut i = 1;
    while i < rules.len() {
        assert!(
            rules[i - 1].addr <= rules[i].addr,
            "rules must be address-ordered"
        );
        i += 1;
    }
    let mut out = [0u8; N];
    let mut k = 0;
    while k < N {
        let mut i = 0;
        while i < rules.len() && rules[i].end() <= k * RULE_CHUNK {
            i += 1;
        }
        out[k] = i as u8;
        k += 1;
    }
    out
}

/// [`cmp_from`] for the one-byte enum/bool rules.
pub const fn allowed_from<const N: usize>(rules: &[rules::AllowedRule]) -> [u8; N] {
    assert!(rules.len() <= u8::MAX as usize, "rule index must fit a u8");
    let mut i = 1;
    while i < rules.len() {
        assert!(
            rules[i - 1].addr <= rules[i].addr,
            "rules must be address-ordered"
        );
        i += 1;
    }
    let mut out = [0u8; N];
    let mut k = 0;
    while k < N {
        let mut i = 0;
        while i < rules.len() && (rules[i].addr as usize) < k * RULE_CHUNK {
            i += 1;
        }
        out[k] = i as u8;
        k += 1;
    }
    out
}

/// Run `allowed` on every enum/bool rule, then `cmp` on every compare rule,
/// that overlaps `[lo, hi)`, each in address order; the first error stops
/// the walk. Each walk starts at its `*_FROM` entry for `lo` and stops at
/// the first rule past `hi`.
#[inline(always)]
pub fn for_each_rule<M: RegisterMap + ?Sized>(
    lo: usize,
    hi: usize,
    mut allowed: impl FnMut(&rules::AllowedRule) -> Result<(), Error>,
    mut cmp: impl FnMut(&rules::CmpRule) -> Result<(), Error>,
) -> Result<(), Error> {
    let chunk = lo / RULE_CHUNK;
    let from = M::ALLOWED_FROM.get(chunk).map_or(0, |&i| i as usize);
    for r in M::ALLOWED_RULES.get(from..).unwrap_or(&[]) {
        if r.addr as usize >= hi {
            break;
        }
        if r.addr as usize >= lo {
            allowed(r)?;
        }
    }
    let from = M::CMP_FROM.get(chunk).map_or(0, |&i| i as usize);
    for r in M::CMP_RULES.get(from..).unwrap_or(&[]) {
        if r.addr as usize >= hi {
            break;
        }
        if r.end() > lo {
            cmp(r)?;
        }
    }
    Ok(())
}

/// Live base pointer + size plus one pending overlay, so a check sees the value
/// about to be committed on top of live storage. The overlay is up to two
/// contiguous ranges -- `o2` logically follows `o1` at `overlay_addr + o1.len()`
/// -- because a ring-seam write arrives as head + tail (`o2` empty otherwise).
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
    /// written) -- those take straight byte loads; the per-byte pick pays
    /// only on seam splits and overlay-edge straddles (bench: the pick chain
    /// ran ~6.8 us per load on flash-resident code, the write path's
    /// hottest function). Outlined: inlining at the (deduped) rule-body
    /// sites measured zero gain for +146 B -- the cost is the loads, not the
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
/// unrolled byte loads -- no per-byte range compares.
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

/// Validate `[addr, addr+o1.len()+o2.len())` against the flat map: bounds ->
/// writable mask -> rules, in that precedence. The view is live storage
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
        // u32 shifts only -- a runtime u64 shift is an __ashldi3 libcall on
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
    for_each_rule::<M>(
        lo,
        hi,
        |r| rules::check_allowed(&view, r.addr, r.allowed),
        |r| rules::check_cmp(&view, r.addr, r.spec, r.val),
    )
}

/// Copy `head` then `tail` to `dst`, one store per naturally aligned unit:
/// 4 bytes while `dst` is 4-aligned with 4 left, else 2 while 2-aligned with
/// 2 left, else 1. Every table field is naturally aligned, so a reader that
/// preempts the copy (the kernel above the bus) sees each field all-old or
/// all-new; two fields can still disagree, as when a commit lands between
/// two of the reader's own loads.
///
/// # Safety
/// `dst` must be valid for `head.len() + tail.len()` byte writes.
#[inline(never)]
unsafe fn store_units(dst: *mut u8, head: &[u8], tail: &[u8]) {
    if !tail.is_empty() {
        // SAFETY: forwarded.
        return unsafe { store_seam(dst, head, tail) };
    }
    copy_run(dst as usize, head, &mut |i, u, w| {
        // SAFETY: caller guarantees the span; `copy_run` picks `w` so the
        // unit is aligned for its width. Volatile keeps it one store.
        unsafe {
            let p = dst.add(i);
            match w {
                4 => p.cast::<u32>().write_volatile(u32::from_ne_bytes(u)),
                2 => p
                    .cast::<u16>()
                    .write_volatile(u16::from_ne_bytes([u[0], u[1]])),
                _ => p.write_volatile(u[0]),
            }
        }
    });
}

/// [`store_units`] split at the ring seam: the covered bytes of the aligned
/// word holding the seam are gathered on the stack, so a unit straddling the
/// seam still lands whole. Units never cross a 4-aligned address, so each of
/// the three runs splits into the units the whole copy would.
///
/// # Safety
/// As [`store_units`].
#[cold]
#[inline(never)]
unsafe fn store_seam(dst: *mut u8, head: &[u8], tail: &[u8]) {
    let mut word = [0u8; 4];
    let (pre, n, post) = seam_split(dst as usize, head, tail, &mut word);
    // SAFETY: `pre`, the `n` gathered bytes and `post` tile the copy in order.
    unsafe {
        store_units(dst, pre, &[]);
        store_units(dst.add(pre.len()), word.get(..n).unwrap_or(&[]), &[]);
        store_units(dst.add(pre.len() + n), post, &[]);
    }
}

/// The seam split of [`store_seam`]: `head` before the seam's aligned word,
/// the count of that word's covered bytes (gathered into `word`), and `tail`
/// after it.
#[inline(always)]
fn seam_split<'a>(
    dst: usize,
    head: &'a [u8],
    tail: &'a [u8],
    word: &mut [u8; 4],
) -> (&'a [u8], usize, &'a [u8]) {
    let off = (dst + head.len()) & 3;
    let (pre, lead) = head
        .split_at_checked(head.len() - off.min(head.len()))
        .unwrap_or((head, &[]));
    let (trail, post) = tail
        .split_at_checked((4 - off).min(tail.len()))
        .unwrap_or((tail, &[]));
    for (b, s) in word.iter_mut().zip(lead.iter().chain(trail)) {
        *b = *s;
    }
    (pre, lead.len() + trail.len(), post)
}

/// One contiguous run of [`store_units`]: hand each unit to
/// `store(offset, bytes, width)` in address order; `dst` is the run's first
/// address.
#[inline(always)]
fn copy_run(dst: usize, src: &[u8], store: &mut impl FnMut(usize, [u8; 4], usize)) {
    let mut rest = src;
    let mut i = 0;
    if dst & 1 != 0
        && let [a, r @ ..] = rest
    {
        store(i, [*a, 0, 0, 0], 1);
        rest = r;
        i += 1;
    }
    if (dst + i) & 2 != 0
        && let [a, b, r @ ..] = rest
    {
        store(i, [*a, *b, 0, 0], 2);
        rest = r;
        i += 2;
    }
    while let [a, b, c, d, r @ ..] = rest {
        store(i, [*a, *b, *c, *d], 4);
        rest = r;
        i += 4;
    }
    if let [a, b, r @ ..] = rest {
        store(i, [*a, *b, 0, 0], 2);
        rest = r;
        i += 2;
    }
    if let [a] = rest {
        store(i, [*a, 0, 0, 0], 1);
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
        self.check_split(addr, head, tail)?;
        self.store_split(addr, head, tail);
        Ok(())
    }

    /// The checks of [`write_split`](RegisterFile::write_split) alone: bounds,
    /// writable mask and rules, the source standing in for the live bytes.
    fn check_split(&self, addr: u16, head: &[u8], tail: &[u8]) -> Result<(), Error> {
        validate(self, addr, head, tail)
    }

    /// The store of [`write_split`](RegisterFile::write_split) alone, for
    /// bytes [`check_split`](RegisterFile::check_split) passed. A span past the
    /// map is dropped.
    fn store_split(&self, addr: u16, head: &[u8], tail: &[u8]) {
        if addr as usize + head.len() + tail.len() > Self::SIZE {
            return;
        }
        // SAFETY: span bounds-checked above; the caller upholds RegisterMap's
        // single-writer contract.
        unsafe { store_units(self.base().add(addr as usize), head, tail) }
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

    /// Store every staged entry, oldest first, then empty the buffer.
    fn commit_staged(&self, staged: &mut StagedWrites) {
        for (addr, data) in staged.iter_all() {
            self.store_split(addr, data, &[]);
        }
        staged.clear();
    }
}

impl<M: RegisterMap> RegisterFile for M {}

#[cfg(test)]
mod tests;
