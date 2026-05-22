//! Byte-addressed access to the control table.
//!
//! Per-region `&[FieldDesc]` tables map protocol slots to (struct offset, size, access):
//! * Outside any region → `OutOfRange`. Cross-region spans → `OutOfRange`.
//! * Padding gap, reserved slot, or RO byte on write → `AccessError`.
//! * Writes validate the full span before committing; failed write leaves the table untouched.
//!
//! Reads use raw-pointer copies, never form `&RegionT`. Writes form a transient
//! `&mut`-equivalent via `SyncUnsafeCell::get()`; caller must hold the region's
//! single-writer guarantee (CONFIG/CONTROL/CALIB from `services`, TELEMETRY from `kernel`).
//!
//! Convention: struct fields whose name starts with `_rsvd_` (and trailing struct-alignment
//! pad) are absent from `*_FIELDS` and remain gaps. Hand-written tables today; the per-block
//! decomposition is shaped to match a future `#[derive(RegmapBlock)]` proc-macro 1:1.

use crate::ControlTable;
use crate::regions::calib::CALIB_FIELDS;
use crate::regions::config::CONFIG_FIELDS;
use crate::regions::control::CONTROL_FIELDS;
use crate::regions::telemetry::TELEMETRY_FIELDS;
use crate::regions::{
    CALIB_BASE_ADDR, CALIB_REGION_SIZE, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, CONTROL_BASE_ADDR,
    CONTROL_REGION_SIZE, TELEMETRY_BASE_ADDR, TELEMETRY_REGION_SIZE,
};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RegmapError {
    /// Empty, wrapping, or outside every region.
    OutOfRange,
    /// Padding gap, reserved slot, or RO byte on write.
    AccessError,
    /// Staging buffer has no room for this chunk.
    StagingFull,
}

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

/// One entry per host-visible struct field. Future: `#[derive(RegmapBlock)]` will
/// emit these from the struct definition; today they are hand-written but kept
/// in lockstep with that expansion (1:1, `offset_of!` for both addrs).
#[derive(Copy, Clone, Debug)]
pub struct FieldDesc {
    /// Byte offset in the region's protocol address space.
    pub addr_offset: u16,
    pub size: u16,
    /// Byte offset in the region struct.
    pub struct_offset: u16,
    pub access: Access,
}

/// (struct base ptr, region base addr, field table) for the region containing
/// `[addr, end)`. `None` if the range crosses or escapes every region.
struct RegionRef {
    base: *mut u8,
    base_addr: u16,
    fields: &'static [FieldDesc],
}

impl ControlTable {
    pub fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), RegmapError> {
        if dst.is_empty() {
            return Ok(());
        }
        let end = (addr as usize)
            .checked_add(dst.len())
            .ok_or(RegmapError::OutOfRange)?;
        let r = self.region_for(addr, end).ok_or(RegmapError::OutOfRange)?;
        // SAFETY: get() non-null; descriptor sizes verified in `region_structs_fit_regions`.
        unsafe { walk_read(r.base, addr - r.base_addr, dst, r.fields) }
    }

    /// Caller must be the region's sole writer (services for CONFIG/CONTROL/CALIB, kernel for TELEMETRY).
    /// Sync-write: stage + commit + rewind. Buffer unchanged on return.
    pub fn write_bytes(
        &self,
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
        let r = self.region_for(addr, end).ok_or(RegmapError::OutOfRange)?;

        let saved_data = staged.data.len();
        let saved_entries = staged.entries.len();

        let result = stage_write(r.base_addr, addr - r.base_addr, src, r.fields, staged);
        if result.is_ok() {
            // Step 4: field/region validators run here.
            unsafe { self.commit_staged_range(staged, saved_entries) };
        }
        staged.rewind(saved_data, saved_entries);
        result
    }

    /// SAFETY: caller holds the region's single-writer guarantee.
    unsafe fn commit_staged_range(&self, staged: &StagedWrites, start_entry: usize) {
        for (abs_addr, data) in staged.iter_from(start_entry) {
            let end = abs_addr as usize + data.len();
            // stage_write guarantees single-region containment.
            let Some(r) = self.region_for(abs_addr, end) else {
                continue;
            };
            unsafe { commit_chunk(r.base, r.base_addr, abs_addr, data, r.fields) };
        }
    }

    fn region_for(&self, addr: u16, end: usize) -> Option<RegionRef> {
        if range_in(addr, end, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE) {
            Some(RegionRef {
                base: self.config.get() as *mut u8,
                base_addr: CONFIG_BASE_ADDR,
                fields: CONFIG_FIELDS,
            })
        } else if range_in(addr, end, TELEMETRY_BASE_ADDR, TELEMETRY_REGION_SIZE) {
            Some(RegionRef {
                base: self.telemetry.get() as *mut u8,
                base_addr: TELEMETRY_BASE_ADDR,
                fields: TELEMETRY_FIELDS,
            })
        } else if range_in(addr, end, CONTROL_BASE_ADDR, CONTROL_REGION_SIZE) {
            Some(RegionRef {
                base: self.control.get() as *mut u8,
                base_addr: CONTROL_BASE_ADDR,
                fields: CONTROL_FIELDS,
            })
        } else if range_in(addr, end, CALIB_BASE_ADDR, CALIB_REGION_SIZE) {
            Some(RegionRef {
                base: self.calib.get() as *mut u8,
                base_addr: CALIB_BASE_ADDR,
                fields: CALIB_FIELDS,
            })
        } else {
            None
        }
    }
}

fn range_in(addr: u16, end: usize, base: u16, size: usize) -> bool {
    let base = base as usize;
    (addr as usize) >= base && end <= base + size
}

/// Walk fields covering `[offset_in_region, +len)`. Calls `on_chunk(struct_off, buf_off, chunk_len)`
/// for each overlapping byte run. Returns `AccessError` on a gap, or on an RO field if `require_rw`.
/// Descriptors must be sorted by `addr_offset` and non-overlapping.
fn walk_fields(
    offset_in_region: u16,
    len: usize,
    fields: &[FieldDesc],
    require_rw: bool,
    mut on_chunk: impl FnMut(usize, usize, usize),
) -> Result<(), RegmapError> {
    let req_hi = offset_in_region as usize + len;
    let mut req_lo = offset_in_region as usize;
    let mut buf_pos = 0usize;
    for desc in fields {
        if req_lo >= req_hi {
            break;
        }
        let blk_lo = desc.addr_offset as usize;
        let blk_hi = blk_lo + desc.size as usize;
        if blk_hi <= req_lo {
            continue;
        }
        if blk_lo > req_lo || (require_rw && desc.access != Access::Rw) {
            return Err(RegmapError::AccessError);
        }
        let chunk_hi = req_hi.min(blk_hi);
        let chunk_len = chunk_hi - req_lo;
        let struct_off = desc.struct_offset as usize + (req_lo - blk_lo);
        on_chunk(struct_off, buf_pos, chunk_len);
        buf_pos += chunk_len;
        req_lo = chunk_hi;
    }
    if req_lo < req_hi {
        Err(RegmapError::AccessError)
    } else {
        Ok(())
    }
}

unsafe fn walk_read(
    region_base: *const u8,
    offset_in_region: u16,
    dst: &mut [u8],
    fields: &[FieldDesc],
) -> Result<(), RegmapError> {
    let dst_ptr = dst.as_mut_ptr();
    walk_fields(
        offset_in_region,
        dst.len(),
        fields,
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

/// Validate that `src` lies entirely in RW fields with no gaps; stage as one entry.
fn stage_write(
    region_base_addr: u16,
    offset_in_region: u16,
    src: &[u8],
    fields: &[FieldDesc],
    staged: &mut StagedWrites,
) -> Result<(), RegmapError> {
    walk_fields(offset_in_region, src.len(), fields, true, |_, _, _| {})?;
    let abs_addr = region_base_addr.wrapping_add(offset_in_region);
    staged.push_chunk(abs_addr, src)
}

/// SAFETY: `region_base` points to the matching region struct; `[abs_addr, abs_addr+data.len())`
/// is contained in that region and covered by RW fields per `fields`; caller has exclusive access.
unsafe fn commit_chunk(
    region_base: *mut u8,
    region_base_addr: u16,
    abs_addr: u16,
    data: &[u8],
    fields: &[FieldDesc],
) {
    let offset = abs_addr.wrapping_sub(region_base_addr);
    let data_ptr = data.as_ptr();
    // stage_write already validated this range; walk_fields can't error here.
    let _ = walk_fields(
        offset,
        data.len(),
        fields,
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
        let t = fresh();
        // block 1 (offset 32) = comms, 4 B.
        let comms_addr = CONFIG_BASE_ADDR + 32;
        let payload = [0x07u8, 0x03, 0x96, 0x00];
        write(&t, comms_addr, &payload).unwrap();
        let mut buf = [0u8; 4];
        t.read_bytes(comms_addr, &mut buf).unwrap();
        assert_eq!(buf, payload);
        let comms = unsafe { &*t.config.get() }.comms;
        assert_eq!(comms.id, 0x07);
        assert_eq!(comms.baud_rate_idx, 0x03);
        assert_eq!(comms.return_delay_us, 0x0096);
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
        let t = fresh();
        // _rsvd_align gap at offset 2..4 forces a split write.
        write(&t, CONTROL_BASE_ADDR, &[1, 3]).unwrap();
        let mut tail = [0u8; 10];
        tail[0..4].copy_from_slice(&10000i32.to_le_bytes());
        tail[4..8].copy_from_slice(&2000i32.to_le_bytes());
        tail[8..10].copy_from_slice(&320i16.to_le_bytes());
        write(&t, CONTROL_BASE_ADDR + 4, &tail).unwrap();
        let lc = unsafe { &*t.control.get() }.lifecycle;
        assert_eq!(lc.torque_enable, 1);
        assert_eq!(lc.mode, 3);
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
        assert_eq!(s.stream_enable, 1);
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

    fn assert_fields_sorted_and_bounded(fields: &[FieldDesc], region_size: u16) {
        for win in fields.windows(2) {
            let (a, b) = (&win[0], &win[1]);
            assert!(
                a.addr_offset + a.size <= b.addr_offset,
                "fields not sorted/non-overlapping: {:?} then {:?}",
                a,
                b,
            );
        }
        if let Some(last) = fields.last() {
            assert!(
                last.addr_offset + last.size <= region_size,
                "last field runs past region end: {:?}",
                last,
            );
        }
    }

    #[test]
    fn config_fields_sorted_and_bounded() {
        use crate::regions::CONFIG_REGION_SIZE;
        assert_fields_sorted_and_bounded(CONFIG_FIELDS, CONFIG_REGION_SIZE as u16);
    }

    #[test]
    fn telemetry_fields_sorted_and_bounded() {
        use crate::regions::TELEMETRY_REGION_SIZE;
        assert_fields_sorted_and_bounded(TELEMETRY_FIELDS, TELEMETRY_REGION_SIZE as u16);
    }

    #[test]
    fn control_fields_sorted_and_bounded() {
        use crate::regions::CONTROL_REGION_SIZE;
        assert_fields_sorted_and_bounded(CONTROL_FIELDS, CONTROL_REGION_SIZE as u16);
    }

    #[test]
    fn calib_fields_sorted_and_bounded() {
        assert_fields_sorted_and_bounded(CALIB_FIELDS, CALIB_REGION_SIZE as u16);
    }

    #[test]
    fn write_to_padding_gap_fails() {
        // ControlLifecycle _rsvd_align gap at addr_offset 2..4.
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
        let t = fresh();
        let comms_addr = CONFIG_BASE_ADDR + 32;
        write(&t, comms_addr, &[0x07, 0x03, 0x96, 0x00]).unwrap();
        write(&t, comms_addr, &[0x09, 0x05]).unwrap();
        let comms = unsafe { &*t.config.get() }.comms;
        assert_eq!(comms.id, 0x09);
        assert_eq!(comms.baud_rate_idx, 0x05);
        assert_eq!(comms.return_delay_us, 0x0096);
    }
}
