//! Byte-addressed access to the control table.
//!
//! Per-region `&[BlockDesc]` tables map protocol slots to (struct offset, size, access):
//! * Outside any region → `OutOfRange`. Cross-region spans → `OutOfRange`.
//! * Reserved slot, or unused tail of an active block, or RO byte on write → `AccessError`.
//! * Writes validate the full span before committing; failed write leaves the table untouched.
//!
//! Reads use raw-pointer copies, never form `&RegionT`. Writes form a transient
//! `&mut`-equivalent via `SyncUnsafeCell::get()`; caller must hold the region's
//! single-writer guarantee (CONFIG/CONTROL/CALIB from `services`, TELEMETRY from `kernel`).

use crate::ControlTable;
use crate::regions::calib::CALIB_BLOCKS;
use crate::regions::config::CONFIG_BLOCKS;
use crate::regions::control::CONTROL_BLOCKS;
use crate::regions::telemetry::TELEMETRY_BLOCKS;
use crate::regions::{
    CALIB_BASE_ADDR, CALIB_REGION_SIZE, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, CONTROL_BASE_ADDR,
    CONTROL_REGION_SIZE, TELEMETRY_BASE_ADDR, TELEMETRY_REGION_SIZE,
};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RegmapError {
    /// Empty, wrapping, or outside every region.
    OutOfRange,
    /// Reserved slot, unused tail of a block, or RO byte on write.
    AccessError,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Access {
    Ro,
    Rw,
}

#[derive(Copy, Clone, Debug)]
pub struct BlockDesc {
    /// Byte offset in the region's protocol address space.
    pub addr_offset: u16,
    /// Active length. Bytes from `addr_offset+size` up to the next slot are reserved.
    pub size: u16,
    /// Byte offset in the region struct.
    pub struct_offset: u16,
    pub access: Access,
}

impl ControlTable {
    pub fn read_bytes(&self, addr: u16, dst: &mut [u8]) -> Result<(), RegmapError> {
        if dst.is_empty() {
            return Ok(());
        }
        let end = (addr as usize)
            .checked_add(dst.len())
            .ok_or(RegmapError::OutOfRange)?;

        if range_in(addr, end, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE) {
            let base = self.config.get() as *const u8;
            // SAFETY: get() non-null; descriptor sizes verified in `region_structs_fit_regions`.
            unsafe { walk_read(base, addr - CONFIG_BASE_ADDR, dst, CONFIG_BLOCKS) }
        } else if range_in(addr, end, TELEMETRY_BASE_ADDR, TELEMETRY_REGION_SIZE) {
            let base = self.telemetry.get() as *const u8;
            unsafe { walk_read(base, addr - TELEMETRY_BASE_ADDR, dst, TELEMETRY_BLOCKS) }
        } else if range_in(addr, end, CONTROL_BASE_ADDR, CONTROL_REGION_SIZE) {
            let base = self.control.get() as *const u8;
            unsafe { walk_read(base, addr - CONTROL_BASE_ADDR, dst, CONTROL_BLOCKS) }
        } else if range_in(addr, end, CALIB_BASE_ADDR, CALIB_REGION_SIZE) {
            let base = self.calib.get() as *const u8;
            unsafe { walk_read(base, addr - CALIB_BASE_ADDR, dst, CALIB_BLOCKS) }
        } else {
            Err(RegmapError::OutOfRange)
        }
    }

    /// Caller must be the region's sole writer (services for CONFIG/CONTROL/CALIB, kernel for TELEMETRY).
    pub fn write_bytes(&self, addr: u16, src: &[u8]) -> Result<(), RegmapError> {
        if src.is_empty() {
            return Ok(());
        }
        let end = (addr as usize)
            .checked_add(src.len())
            .ok_or(RegmapError::OutOfRange)?;

        if range_in(addr, end, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE) {
            let base = self.config.get() as *mut u8;
            unsafe { walk_write(base, addr - CONFIG_BASE_ADDR, src, CONFIG_BLOCKS) }
        } else if range_in(addr, end, TELEMETRY_BASE_ADDR, TELEMETRY_REGION_SIZE) {
            let base = self.telemetry.get() as *mut u8;
            unsafe { walk_write(base, addr - TELEMETRY_BASE_ADDR, src, TELEMETRY_BLOCKS) }
        } else if range_in(addr, end, CONTROL_BASE_ADDR, CONTROL_REGION_SIZE) {
            let base = self.control.get() as *mut u8;
            unsafe { walk_write(base, addr - CONTROL_BASE_ADDR, src, CONTROL_BLOCKS) }
        } else if range_in(addr, end, CALIB_BASE_ADDR, CALIB_REGION_SIZE) {
            let base = self.calib.get() as *mut u8;
            unsafe { walk_write(base, addr - CALIB_BASE_ADDR, src, CALIB_BLOCKS) }
        } else {
            Err(RegmapError::OutOfRange)
        }
    }
}

fn range_in(addr: u16, end: usize, base: u16, size: usize) -> bool {
    let base = base as usize;
    (addr as usize) >= base && end <= base + size
}

/// Descriptors must be sorted by `addr_offset` and non-overlapping.
unsafe fn walk_read(
    region_base: *const u8,
    offset_in_region: u16,
    dst: &mut [u8],
    blocks: &[BlockDesc],
) -> Result<(), RegmapError> {
    let mut req_lo = offset_in_region as usize;
    let req_hi = req_lo + dst.len();
    let mut dst_pos = 0usize;

    for desc in blocks {
        if req_lo >= req_hi {
            break;
        }
        let blk_lo = desc.addr_offset as usize;
        let blk_hi = blk_lo + desc.size as usize;
        if blk_hi <= req_lo {
            continue;
        }
        if blk_lo > req_lo {
            return Err(RegmapError::AccessError);
        }
        let chunk_hi = req_hi.min(blk_hi);
        let chunk_len = chunk_hi - req_lo;
        let struct_off = desc.struct_offset as usize + (req_lo - blk_lo);
        unsafe {
            core::ptr::copy_nonoverlapping(
                region_base.add(struct_off),
                dst.as_mut_ptr().add(dst_pos),
                chunk_len,
            );
        }
        dst_pos += chunk_len;
        req_lo = chunk_hi;
    }

    if req_lo < req_hi {
        Err(RegmapError::AccessError)
    } else {
        Ok(())
    }
}

unsafe fn walk_write(
    region_base: *mut u8,
    offset_in_region: u16,
    src: &[u8],
    blocks: &[BlockDesc],
) -> Result<(), RegmapError> {
    // Validate before mutating: any uncovered or RO byte aborts the whole write.
    let req_lo0 = offset_in_region as usize;
    let req_hi = req_lo0 + src.len();
    let mut req_lo = req_lo0;
    for desc in blocks {
        if req_lo >= req_hi {
            break;
        }
        let blk_lo = desc.addr_offset as usize;
        let blk_hi = blk_lo + desc.size as usize;
        if blk_hi <= req_lo {
            continue;
        }
        if blk_lo > req_lo || desc.access != Access::Rw {
            return Err(RegmapError::AccessError);
        }
        req_lo = req_hi.min(blk_hi);
    }
    if req_lo < req_hi {
        return Err(RegmapError::AccessError);
    }

    let mut req_lo = req_lo0;
    let mut src_pos = 0usize;
    for desc in blocks {
        if req_lo >= req_hi {
            break;
        }
        let blk_lo = desc.addr_offset as usize;
        let blk_hi = blk_lo + desc.size as usize;
        if blk_hi <= req_lo {
            continue;
        }
        let chunk_hi = req_hi.min(blk_hi);
        let chunk_len = chunk_hi - req_lo;
        let struct_off = desc.struct_offset as usize + (req_lo - blk_lo);
        unsafe {
            core::ptr::copy_nonoverlapping(
                src.as_ptr().add(src_pos),
                region_base.add(struct_off),
                chunk_len,
            );
        }
        src_pos += chunk_len;
        req_lo = chunk_hi;
    }
    Ok(())
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
        t.write_bytes(comms_addr, &payload).unwrap();
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
        let err = t.write_bytes(CONFIG_BASE_ADDR, &payload).unwrap_err();
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
        let payload: [u8; 16] = [
            1, 3, 0, 0, // torque_enable, mode, padding
            0x10, 0x27, 0, 0, // goal_position = 10000
            0xD0, 0x07, 0, 0, // goal_velocity = 2000
            0x40, 0x01, // goal_effort = 320
            0, 0,
        ];
        t.write_bytes(CONTROL_BASE_ADDR, &payload).unwrap();
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
        let err = t.write_bytes(TELEMETRY_BASE_ADDR, &payload).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
    }

    #[test]
    fn telemetry_reads_succeed_for_active_blocks() {
        let t = fresh();
        let mut buf = [0xAAu8; 16];
        t.read_bytes(TELEMETRY_BASE_ADDR, &mut buf).unwrap();
        assert_eq!(buf, [0u8; 16]);
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
        t.write_bytes(bemf_addr, &payload).unwrap();
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
        t.write_bytes(addr, &payload).unwrap();
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
        let mut payload = [0u8; 16];
        payload[0..2].copy_from_slice(&0x0100u16.to_le_bytes());
        payload[2..4].copy_from_slice(&0x0040u16.to_le_bytes());
        payload[4..6].copy_from_slice(&0x0080u16.to_le_bytes());
        payload[8..12].copy_from_slice(&5000i32.to_le_bytes());
        t.write_bytes(addr, &payload).unwrap();
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
        let mut payload = [0u8; 12];
        payload[0] = 1;
        payload[1] = 4;
        payload[2..4].copy_from_slice(&500u16.to_le_bytes());
        payload[4..8].copy_from_slice(&0x0000_00ABu32.to_le_bytes());
        t.write_bytes(addr, &payload).unwrap();
        let s = unsafe { &*t.control.get() }.streaming;
        assert_eq!(s.stream_enable, 1);
        assert_eq!(s.stream_decimation, 4);
        assert_eq!(s.stream_duration_ms, 500);
        assert_eq!(s.stream_field_mask, 0xAB);
    }

    #[test]
    fn write_calib_pot_lut_partial_does_not_smash_neighbours() {
        let t = fresh();
        let addr = CALIB_BASE_ADDR;
        let payload = [0xAA, 0xBB, 0xCC, 0xDD];
        t.write_bytes(addr, &payload).unwrap();
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
        let err = t.write_bytes(addr, &payload).unwrap_err();
        assert_eq!(err, RegmapError::AccessError);
        let stall = unsafe { &*t.config.get() }.safety.stall;
        assert_eq!(stall.stall_motion_threshold_urad, 0);
    }

    #[test]
    fn partial_write_preserves_unwritten_bytes() {
        let t = fresh();
        let comms_addr = CONFIG_BASE_ADDR + 32;
        t.write_bytes(comms_addr, &[0x07, 0x03, 0x96, 0x00])
            .unwrap();
        t.write_bytes(comms_addr, &[0x09, 0x05]).unwrap();
        let comms = unsafe { &*t.config.get() }.comms;
        assert_eq!(comms.id, 0x09);
        assert_eq!(comms.baud_rate_idx, 0x05);
        assert_eq!(comms.return_delay_us, 0x0096);
    }
}
