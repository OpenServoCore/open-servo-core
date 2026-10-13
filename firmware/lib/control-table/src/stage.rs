use crate::Error;

/// Total staging buffer for HOLD write data, in bytes; held writes stay here
/// until a COMMIT, so the cap must fit the largest legal single write -
/// 250 B of data under the 252 B payload ceiling (osc-native sec 5.1: LEN is
/// the only size limit). Override with `OSC_STAGE_DATA_CAP`.
pub const STAGE_DATA_CAP: usize = env_usize(option_env!("OSC_STAGE_DATA_CAP"), 256);

/// Max number of pending RegWrite entries before Action commits. Override
/// with `OSC_STAGE_ENTRY_CAP`.
pub const STAGE_ENTRY_CAP: usize = env_usize(option_env!("OSC_STAGE_ENTRY_CAP"), 16);

const fn env_usize(value: Option<&'static str>, default: usize) -> usize {
    match value {
        Some(s) => parse_usize(s),
        None => default,
    }
}

const fn parse_usize(s: &str) -> usize {
    let bytes = s.as_bytes();
    let mut n: usize = 0;
    let mut i = 0;
    while i < bytes.len() {
        let b = bytes[i];
        assert!(
            b >= b'0' && b <= b'9',
            "OSC_STAGE_* must be a decimal integer",
        );
        n = n * 10 + (b - b'0') as usize;
        i += 1;
    }
    n
}

#[derive(Copy, Clone)]
pub(crate) struct StagedEntry {
    addr: u16,
    len: u16,
    data_off: u16,
}

/// Caller-held transaction handle. Hand to `revert_to` to discard pushes
/// since capture, or to `iter_from` to walk them. Nesting is free -- each
/// caller holds its own.
#[derive(Copy, Clone)]
pub struct Snapshot {
    pub(crate) data: u16,
    pub(crate) entries: u16,
}

impl Snapshot {
    /// Start-of-buffer snapshot: `iter_from(&ZERO)` walks every staged entry.
    /// Used by `commit_staged` to flush the full buffer through the same
    /// path as range commits.
    pub(crate) const ZERO: Snapshot = Snapshot {
        data: 0,
        entries: 0,
    };
}

pub struct StagedWrites {
    pub(crate) data: heapless::Vec<u8, STAGE_DATA_CAP>,
    pub(crate) entries: heapless::Vec<StagedEntry, STAGE_ENTRY_CAP>,
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

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            data: self.data.len() as u16,
            entries: self.entries.len() as u16,
        }
    }

    /// Discard everything pushed since `snap`. Infallible: truncation only.
    pub fn revert_to(&mut self, snap: &Snapshot) {
        self.data.truncate(snap.data as usize);
        self.entries.truncate(snap.entries as usize);
    }

    pub fn push(&mut self, addr: u16, src: &[u8]) -> Result<(), Error> {
        self.push_split(addr, src, &[])
    }

    /// As [`push`](Self::push) but with the source split across the ring seam:
    /// `head` then `tail` land contiguously in one staged entry.
    pub fn push_split(&mut self, addr: u16, head: &[u8], tail: &[u8]) -> Result<(), Error> {
        let data_off = self.data.len();
        let total = head.len() + tail.len();
        if data_off + total > STAGE_DATA_CAP || self.entries.is_full() {
            return Err(Error::StagingFull);
        }
        // SAFETY: the check above leaves room for `total` bytes past
        // `data_off` and for one entry; the copies write exactly those bytes
        // before the length covers them.
        unsafe {
            let dst = self.data.as_mut_ptr().add(data_off);
            copy_bytes(dst, head);
            copy_bytes(dst.add(head.len()), tail);
            self.data.set_len(data_off + total);
            self.entries.push_unchecked(StagedEntry {
                addr,
                len: total as u16,
                data_off: data_off as u16,
            });
        }
        Ok(())
    }

    /// Walk every staged entry, oldest first.
    pub fn iter_all(&self) -> impl Iterator<Item = (u16, &[u8])> + '_ {
        self.iter_from(&Snapshot::ZERO)
    }

    pub fn iter_from(&self, snap: &Snapshot) -> impl Iterator<Item = (u16, &[u8])> + '_ {
        self.entries[snap.entries as usize..].iter().map(|e| {
            let lo = e.data_off as usize;
            let hi = lo + e.len as usize;
            (e.addr, &self.data[lo..hi])
        })
    }
}

/// Volatile keeps it a loop: a plain copy loop compiles to a `memcpy` call.
///
/// # Safety
/// `dst` must be valid for `src.len()` byte writes.
#[inline(always)]
unsafe fn copy_bytes(dst: *mut u8, src: &[u8]) {
    for (i, &b) in src.iter().enumerate() {
        // SAFETY: forwarded.
        unsafe { dst.add(i).write_volatile(b) };
    }
}

impl Default for StagedWrites {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_split_lands_contiguously_and_rejects_whole() {
        let mut s = StagedWrites::new();
        s.push_split(7, &[1, 2, 3], &[4, 5]).unwrap();
        s.push(9, &[6]).unwrap();
        assert!(s.iter_all().eq([(7, &[1, 2, 3, 4, 5][..]), (9, &[6][..])]));

        let fill = [0u8; STAGE_DATA_CAP];
        let room = STAGE_DATA_CAP - 6;
        assert_eq!(
            s.push_split(1, &fill[..room], &[0]),
            Err(Error::StagingFull)
        );
        assert_eq!(s.snapshot().data, 6, "a rejected push stages no byte");
        s.push_split(1, &fill[..room - 1], &[0]).unwrap();
        assert_eq!(s.snapshot().data as usize, STAGE_DATA_CAP);
    }

    #[test]
    fn entry_cap_rejects_with_nothing_staged() {
        let mut s = StagedWrites::new();
        for a in 0..STAGE_ENTRY_CAP as u16 {
            s.push(a, &[1]).unwrap();
        }
        let before = s.snapshot();
        assert_eq!(s.push_split(0, &[2], &[3]), Err(Error::StagingFull));
        let after = s.snapshot();
        assert_eq!((after.data, after.entries), (before.data, before.entries));
    }
}
