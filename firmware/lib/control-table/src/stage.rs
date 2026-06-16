use crate::desc::Error;
use crate::route::{Router, router_read_bytes};

/// Total staging buffer for pending RegWrite data, in bytes. Must be at
/// least `MAX_CONTROL_RW` from the DXL services layer (asserted in
/// `osc_core::services::dxl::limits`). Override with `OSC_STAGE_DATA_CAP`.
/// Multiple RegWrites share this budget before Action commits.
pub const STAGE_DATA_CAP: usize = env_usize(option_env!("OSC_STAGE_DATA_CAP"), 128);

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

/// Caller-held transaction handle. Hand to `rewind_to` to discard pushes
/// since capture, or to `iter_from` to walk them. Nesting is free — each
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

    pub fn rewind_to(&mut self, snap: Snapshot) {
        self.data.truncate(snap.data as usize);
        self.entries.truncate(snap.entries as usize);
    }

    pub fn push(&mut self, addr: u16, src: &[u8]) -> Result<(), Error> {
        let data_off = self.data.len();
        if data_off + src.len() > STAGE_DATA_CAP {
            return Err(Error::StagingFull);
        }
        self.data
            .extend_from_slice(src)
            .map_err(|_| Error::StagingFull)?;
        self.entries
            .push(StagedEntry {
                addr,
                len: src.len() as u16,
                data_off: data_off as u16,
            })
            .map_err(|_| {
                self.data.truncate(data_off);
                Error::StagingFull
            })?;
        Ok(())
    }

    pub fn iter_from(&self, snap: &Snapshot) -> impl Iterator<Item = (u16, &[u8])> + '_ {
        self.entries[snap.entries as usize..].iter().map(|e| {
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

/// Overlays staged-but-uncommitted bytes onto live-table reads so validators
/// see the value about to be committed.
pub struct StagedView<'a> {
    router: &'a dyn Router,
    staged: &'a StagedWrites,
    snap: Snapshot,
}

impl<'a> StagedView<'a> {
    pub fn new(router: &'a dyn Router, staged: &'a StagedWrites, snap: Snapshot) -> Self {
        Self {
            router,
            staged,
            snap,
        }
    }

    pub fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), Error> {
        router_read_bytes(self.router, addr, dst)?;
        if dst.is_empty() {
            return Ok(());
        }
        let req_lo = addr as usize;
        let req_hi = req_lo + dst.len();
        for (s_addr, s_data) in self.staged.iter_from(&self.snap) {
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
