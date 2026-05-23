use crate::desc::Error;
use crate::route::{Router, router_read_bytes};

pub const STAGE_DATA_CAP: usize = 128;
pub const STAGE_ENTRY_CAP: usize = 16;

#[derive(Copy, Clone)]
pub(crate) struct StagedEntry {
    addr: u16,
    len: u16,
    data_off: u16,
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

    pub(crate) fn push_chunk(&mut self, addr: u16, src: &[u8]) -> Result<(), Error> {
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

    pub(crate) fn rewind(&mut self, data_len: usize, entry_count: usize) {
        self.data.truncate(data_len);
        self.entries.truncate(entry_count);
    }

    pub(crate) fn iter_from(&self, start_entry: usize) -> impl Iterator<Item = (u16, &[u8])> + '_ {
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

    pub fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), Error> {
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
