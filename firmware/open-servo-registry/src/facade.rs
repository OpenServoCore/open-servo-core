//! Facade translation between DXL-compatible RAM registers and canonical vendor registers.
//!
//! This module provides translation for the core 4 fields:
//! - `torque_enable`: dxl 64 <-> vendor 581
//! - `goal_pwm`: dxl 100 <-> vendor 583
//! - `goal_position`: dxl 116 <-> vendor 512 (with position codec)
//! - `present_position`: dxl 132 <-> vendor 516 (RO publish only)
//!
//! # Buffer Limitation
//!
//! Translation buffers are sized to [`MAX_MAP_LEN`] bytes (currently 4).
//! All mapping entries must have `dxl_len` and `vendor_len` <= `MAX_MAP_LEN`.

use open_servo_shadow::KernelView;

use crate::spec::{CodecKind, MapDirection};
use crate::vendor::{CodecCtx, FACADE_MAPPINGS};

// ============================================================================
// Constants
// ============================================================================

/// Maximum byte length for a single mapping entry.
///
/// Translation buffers are sized to this value. All `dxl_len` and `vendor_len`
/// in [`FACADE_MAPPINGS`] must be <= this constant.
pub const MAX_MAP_LEN: usize = 4;

// ============================================================================
// Codec Types
// ============================================================================

/// Servo position mode (bounded vs wrap).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ServoPosKind {
    Bounded = 0,
    Wrap360 = 1,
}

impl From<u8> for ServoPosKind {
    fn from(v: u8) -> Self {
        match v {
            0 => ServoPosKind::Bounded,
            _ => ServoPosKind::Wrap360,
        }
    }
}

// ============================================================================
// Codec Trait and Implementations
// ============================================================================

/// Trait for facade↔vendor codecs.
pub trait Codec {
    /// Decode src (dxl) bytes to dst (vendor) bytes.
    fn decode(src: &[u8], dst: &mut [u8], ctx: &CodecCtx);
    /// Encode src (vendor) bytes to dst (dxl) bytes.
    fn encode(src: &[u8], dst: &mut [u8], ctx: &CodecCtx);
}

/// Identity codec: copy bytes unchanged.
pub struct IdentityCodec;

impl Codec for IdentityCodec {
    fn decode(src: &[u8], dst: &mut [u8], _ctx: &CodecCtx) {
        debug_assert_eq!(
            src.len(),
            dst.len(),
            "Identity codec requires equal lengths"
        );
        dst.copy_from_slice(src);
    }

    fn encode(src: &[u8], dst: &mut [u8], _ctx: &CodecCtx) {
        debug_assert_eq!(
            src.len(),
            dst.len(),
            "Identity codec requires equal lengths"
        );
        dst.copy_from_slice(src);
    }
}

/// Position codec: pulse ↔ centidegree conversion with optional clamping.
pub struct PositionCodec;

impl Codec for PositionCodec {
    fn decode(src: &[u8], dst: &mut [u8], ctx: &CodecCtx) {
        let pulses = i32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let cdeg = decode_position(pulses, ctx);
        dst.copy_from_slice(&cdeg.to_le_bytes());
    }

    fn encode(src: &[u8], dst: &mut [u8], ctx: &CodecCtx) {
        let cdeg = i32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let pulses = encode_position(cdeg, ctx);
        dst.copy_from_slice(&pulses.to_le_bytes());
    }
}

// ============================================================================
// Position Codec Helpers
// ============================================================================

/// Decode pulses (dxl i32) to centidegrees (vendor i32).
///
/// Conversion: 1 pulse = 0.088 degrees = 8.8 cdeg
/// Formula: cdeg = round(pulses * 8.8) = (pulses * 88 + sign * 5) / 10
fn pulses_to_cdeg(pulses: i32) -> i32 {
    let sign = if pulses >= 0 { 5 } else { -5 };
    (pulses * 88 + sign) / 10
}

/// Encode centidegrees (vendor i32) to pulses (dxl i32).
///
/// Formula: pulses = round(cdeg / 8.8) = (cdeg * 10 + sign * 44) / 88
fn cdeg_to_pulses(cdeg: i32) -> i32 {
    let sign = if cdeg >= 0 { 44 } else { -44 };
    (cdeg * 10 + sign) / 88
}

/// Decode pulses to cdeg with optional clamping based on position kind.
fn decode_position(pulses: i32, ctx: &CodecCtx) -> i32 {
    let cdeg = pulses_to_cdeg(pulses);
    if ctx.servo_pos_kind == ServoPosKind::Bounded as u8 {
        cdeg.clamp(ctx.pos_min_limit_cdeg, ctx.pos_max_limit_cdeg)
    } else {
        cdeg
    }
}

/// Encode cdeg to pulses with optional clamping based on position kind.
fn encode_position(cdeg: i32, ctx: &CodecCtx) -> i32 {
    let clamped = if ctx.servo_pos_kind == ServoPosKind::Bounded as u8 {
        cdeg.clamp(ctx.pos_min_limit_cdeg, ctx.pos_max_limit_cdeg)
    } else {
        cdeg
    };
    cdeg_to_pulses(clamped)
}

// ============================================================================
// Codec Dispatch
// ============================================================================

/// Apply decode (dxl → vendor) for the given codec kind.
fn apply_codec_decode(codec: CodecKind, src: &[u8], dst: &mut [u8], ctx: &CodecCtx) {
    match codec {
        CodecKind::Identity => IdentityCodec::decode(src, dst, ctx),
        CodecKind::Position => PositionCodec::decode(src, dst, ctx),
    }
}

/// Apply encode (vendor → dxl) for the given codec kind.
fn apply_codec_encode(codec: CodecKind, src: &[u8], dst: &mut [u8], ctx: &CodecCtx) {
    match codec {
        CodecKind::Identity => IdentityCodec::encode(src, dst, ctx),
        CodecKind::Position => PositionCodec::encode(src, dst, ctx),
    }
}

// ============================================================================
// Translation Functions
// ============================================================================

/// Translate dirty RW dxl writes into canonical vendor registers.
///
/// For each RW dxl field in [`FACADE_MAPPINGS`]:
/// - If dxl range is dirty: read dxl, decode, write vendor, mark vendor dirty, clear dxl dirty
///
/// This function should be called before `kernel.commit_shadow()`.
pub fn translate_dirty_facade_to_vendor(view: &mut KernelView<'_>) {
    let ctx = CodecCtx::from_view(&*view);

    for entry in FACADE_MAPPINGS {
        // Skip entries not in ToVendor direction
        if !matches!(entry.direction, MapDirection::ToVendor | MapDirection::Both) {
            continue;
        }

        // Skip if dxl range is not dirty
        if !view.is_range_dirty(entry.dxl_addr, entry.dxl_len as u16) {
            continue;
        }

        let mut src_buf = [0u8; MAX_MAP_LEN];
        let mut dst_buf = [0u8; MAX_MAP_LEN];
        let src = &mut src_buf[..entry.dxl_len as usize];
        let dst = &mut dst_buf[..entry.vendor_len as usize];

        if view.read(entry.dxl_addr, src).is_ok() {
            apply_codec_decode(entry.codec, src, dst, &ctx);
            // Only mark vendor dirty / clear dxl dirty if write succeeds
            if view.write(entry.vendor_addr, dst).is_ok() {
                view.mark_range_dirty(entry.vendor_addr, entry.vendor_len as u16);
                view.clear_range_dirty(entry.dxl_addr, entry.dxl_len as u16);
            }
        }
    }
}

/// Publish canonical vendor telemetry into RO dxl fields.
///
/// Reads vendor registers, encodes to dxl format, writes dxl.
/// Does NOT mark dirty (these are RO from host perspective).
///
/// Safe write-back: only updates dxl if the dxl range is NOT dirty,
/// avoiding clobber of pending host writes.
///
/// This function should be called after `kernel.publish_telemetry()`.
pub fn publish_vendor_to_facade(view: &mut KernelView<'_>) {
    let ctx = CodecCtx::from_view(&*view);

    for entry in FACADE_MAPPINGS {
        // Skip entries not in ToFacade direction
        if !matches!(entry.direction, MapDirection::ToFacade | MapDirection::Both) {
            continue;
        }

        // Safe write-back: skip if dxl is dirty (pending host write)
        if view.is_range_dirty(entry.dxl_addr, entry.dxl_len as u16) {
            continue;
        }

        let mut src_buf = [0u8; MAX_MAP_LEN];
        let mut dst_buf = [0u8; MAX_MAP_LEN];
        let src = &mut src_buf[..entry.vendor_len as usize]; // read from vendor
        let dst = &mut dst_buf[..entry.dxl_len as usize]; // write to dxl

        if view.read(entry.vendor_addr, src).is_ok() {
            apply_codec_encode(entry.codec, src, dst, &ctx);
            // Write to dxl; ignore errors (RO telemetry, no dirty marking)
            let _ = view.write(entry.dxl_addr, dst);
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::addr as dxl_addr;
    use crate::spec::MapEntry;
    use crate::vendor::addr as vendor_addr;
    use open_servo_shadow::ShadowTable;

    #[test]
    fn test_position_codec_roundtrip_basic() {
        // 2048 pulses -> ~18022 cdeg -> 2048 pulses
        let cdeg = pulses_to_cdeg(2048);
        assert_eq!(cdeg, 18022); // (2048*88+5)/10 = 180229/10 = 18022

        let pulses_back = cdeg_to_pulses(cdeg);
        assert_eq!(pulses_back, 2048); // (18022*10+44)/88 = 180264/88 = 2048
    }

    #[test]
    fn test_position_codec_negative() {
        // Test negative values
        let cdeg = pulses_to_cdeg(-2048);
        assert_eq!(cdeg, -18022); // (-2048*88-5)/10 = -180229/10 = -18022

        let pulses_back = cdeg_to_pulses(cdeg);
        assert_eq!(pulses_back, -2048);
    }

    #[test]
    fn test_position_codec_zero() {
        assert_eq!(pulses_to_cdeg(0), 0);
        assert_eq!(cdeg_to_pulses(0), 0);
    }

    #[test]
    fn test_translate_facade_to_vendor_marks_vendor_dirty_and_clears_facade_dirty() {
        let mut table = ShadowTable::<1024>::new();

        // Seed context: operating_mode=0, pos_kind=0 (Bounded), min=-18000, max=18000
        table
            .write_no_dirty(vendor_addr::OPERATING_MODE, &[0])
            .unwrap();
        table
            .write_no_dirty(vendor_addr::SERVO_POS_KIND, &[0])
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MIN_LIMIT_CDEG, &(-18000i32).to_le_bytes())
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MAX_LIMIT_CDEG, &(18000i32).to_le_bytes())
            .unwrap();

        // Host writes dxl goal_position (marks dirty)
        table
            .write(dxl_addr::GOAL_POSITION, &1024i32.to_le_bytes())
            .unwrap();
        assert!(table.is_range_dirty(dxl_addr::GOAL_POSITION, 4));

        // Translate (scope KernelView to drop before reading table)
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            translate_dirty_facade_to_vendor(&mut view);
        }

        // Assert: vendor 512 has expected cdeg
        let mut buf = [0u8; 4];
        table.read(vendor_addr::GOAL_POS_CDEG, &mut buf).unwrap();
        let cdeg = i32::from_le_bytes(buf);
        assert_eq!(cdeg, pulses_to_cdeg(1024)); // ~9011

        // Assert: vendor range dirty, dxl range not dirty
        assert!(table.is_range_dirty(vendor_addr::GOAL_POS_CDEG, 4));
        assert!(!table.is_range_dirty(dxl_addr::GOAL_POSITION, 4));
    }

    #[test]
    fn test_translate_torque_enable() {
        let mut table = ShadowTable::<1024>::new();

        // Host writes dxl torque_enable
        table.write(dxl_addr::TORQUE_ENABLE, &[1]).unwrap();
        assert!(table.is_range_dirty(dxl_addr::TORQUE_ENABLE, 1));

        // Translate
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            translate_dirty_facade_to_vendor(&mut view);
        }

        // Assert: vendor torque_enable has value
        let mut buf = [0u8; 1];
        table.read(vendor_addr::TORQUE_ENABLE, &mut buf).unwrap();
        assert_eq!(buf[0], 1);

        // Assert: vendor dirty, dxl not dirty
        assert!(table.is_range_dirty(vendor_addr::TORQUE_ENABLE, 1));
        assert!(!table.is_range_dirty(dxl_addr::TORQUE_ENABLE, 1));
    }

    #[test]
    fn test_translate_goal_pwm() {
        let mut table = ShadowTable::<1024>::new();

        // Host writes dxl goal_pwm (i16 LE)
        let pwm: i16 = -500;
        table.write(dxl_addr::GOAL_PWM, &pwm.to_le_bytes()).unwrap();
        assert!(table.is_range_dirty(dxl_addr::GOAL_PWM, 2));

        // Translate
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            translate_dirty_facade_to_vendor(&mut view);
        }

        // Assert: vendor goal_pwm has value
        let mut buf = [0u8; 2];
        table.read(vendor_addr::GOAL_PWM, &mut buf).unwrap();
        assert_eq!(i16::from_le_bytes(buf), -500);

        // Assert: vendor dirty, dxl not dirty
        assert!(table.is_range_dirty(vendor_addr::GOAL_PWM, 2));
        assert!(!table.is_range_dirty(dxl_addr::GOAL_PWM, 2));
    }

    #[test]
    fn test_publish_vendor_to_facade_writes_present_position() {
        let mut table = ShadowTable::<1024>::new();

        // Seed context
        table
            .write_no_dirty(vendor_addr::SERVO_POS_KIND, &[0])
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MIN_LIMIT_CDEG, &(-18000i32).to_le_bytes())
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MAX_LIMIT_CDEG, &(18000i32).to_le_bytes())
            .unwrap();

        // Seed vendor present_pos_cdeg
        table
            .write_no_dirty(vendor_addr::PRESENT_POS_CDEG, &(9000i32).to_le_bytes())
            .unwrap();

        // Publish (scope KernelView)
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            publish_vendor_to_facade(&mut view);
        }

        // Assert: dxl 132 has expected pulses
        let mut buf = [0u8; 4];
        table.read(dxl_addr::PRESENT_POSITION, &mut buf).unwrap();
        let pulses = i32::from_le_bytes(buf);
        assert_eq!(pulses, cdeg_to_pulses(9000)); // ~1023

        // Assert: dxl range is NOT dirty
        assert!(!table.is_range_dirty(dxl_addr::PRESENT_POSITION, 4));
    }

    #[test]
    fn test_position_clamping_bounded() {
        let ctx = CodecCtx {
            operating_mode: 0,
            servo_pos_kind: ServoPosKind::Bounded as u8,
            pos_min_limit_cdeg: -9000,
            pos_max_limit_cdeg: 9000,
        };

        // Value within limits - no clamping
        let cdeg = decode_position(1000, &ctx);
        assert_eq!(cdeg, pulses_to_cdeg(1000));

        // Value exceeds max - should clamp
        let cdeg = decode_position(2000, &ctx); // 2000 pulses = 17600 cdeg > 9000
        assert_eq!(cdeg, 9000);

        // Value below min - should clamp
        let cdeg = decode_position(-2000, &ctx);
        assert_eq!(cdeg, -9000);
    }

    #[test]
    fn test_position_no_clamping_wrap360() {
        let ctx = CodecCtx {
            operating_mode: 0,
            servo_pos_kind: ServoPosKind::Wrap360 as u8,
            pos_min_limit_cdeg: -9000,
            pos_max_limit_cdeg: 9000,
        };

        // Should NOT clamp even if outside limits
        let cdeg = decode_position(2000, &ctx);
        assert_eq!(cdeg, pulses_to_cdeg(2000)); // ~17600, not clamped
    }

    #[test]
    fn test_publish_vendor_to_facade_skips_when_facade_dirty() {
        let mut table = ShadowTable::<1024>::new();

        // Seed context (all fields read by CodecCtx::from_view)
        table
            .write_no_dirty(vendor_addr::OPERATING_MODE, &[0])
            .unwrap();
        table
            .write_no_dirty(vendor_addr::SERVO_POS_KIND, &[0])
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MIN_LIMIT_CDEG, &(-18000i32).to_le_bytes())
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MAX_LIMIT_CDEG, &(18000i32).to_le_bytes())
            .unwrap();

        // Seed vendor present_pos_cdeg (kernel telemetry)
        table
            .write_no_dirty(vendor_addr::PRESENT_POS_CDEG, &(9000i32).to_le_bytes())
            .unwrap();

        // Host writes to dxl addr 132 (marks dirty) - simulates pending write
        let host_bytes = 0xDEADBEEFu32.to_le_bytes();
        table
            .write(dxl_addr::PRESENT_POSITION, &host_bytes)
            .unwrap();
        assert!(table.is_range_dirty(dxl_addr::PRESENT_POSITION, 4));

        // Publish (should skip write because dxl is dirty)
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            publish_vendor_to_facade(&mut view);
        }

        // Assert: dxl 132 remains the host-written bytes (NOT overwritten)
        let mut buf = [0u8; 4];
        table.read(dxl_addr::PRESENT_POSITION, &mut buf).unwrap();
        assert_eq!(buf, host_bytes, "dxl should retain host-written bytes");

        // Assert: dirty bit remains set
        assert!(table.is_range_dirty(dxl_addr::PRESENT_POSITION, 4));
    }

    // ========================================================================
    // Tests for FACADE_MAPPINGS from vendor
    // ========================================================================

    #[test]
    fn test_mapping_table_has_expected_entries() {
        // Verify the generated mapping table has 4 entries with correct addresses
        assert_eq!(FACADE_MAPPINGS.len(), 4);

        // Find entries by name to be order-independent
        fn find_entry(name: &str) -> &'static MapEntry {
            FACADE_MAPPINGS.iter().find(|e| e.name == name).expect(name)
        }

        // Check goal_pos_cdeg (vendor) maps to goal_position (dxl)
        let goal_pos = find_entry("goal_pos_cdeg");
        assert_eq!(goal_pos.dxl_addr, dxl_addr::GOAL_POSITION);
        assert_eq!(goal_pos.vendor_addr, vendor_addr::GOAL_POS_CDEG);
        assert_eq!(goal_pos.dxl_len, 4);
        assert_eq!(goal_pos.vendor_len, 4);
        assert_eq!(goal_pos.direction, MapDirection::Both); // RW
        assert_eq!(goal_pos.codec, CodecKind::Position);

        // Check present_pos_cdeg (vendor) maps to present_position (dxl)
        let present_pos = find_entry("present_pos_cdeg");
        assert_eq!(present_pos.dxl_addr, dxl_addr::PRESENT_POSITION);
        assert_eq!(present_pos.vendor_addr, vendor_addr::PRESENT_POS_CDEG);
        assert_eq!(present_pos.dxl_len, 4);
        assert_eq!(present_pos.vendor_len, 4);
        assert_eq!(present_pos.direction, MapDirection::ToFacade); // RO
        assert_eq!(present_pos.codec, CodecKind::Position);

        // Check torque_enable
        let torque = find_entry("torque_enable");
        assert_eq!(torque.dxl_addr, dxl_addr::TORQUE_ENABLE);
        assert_eq!(torque.vendor_addr, vendor_addr::TORQUE_ENABLE);
        assert_eq!(torque.dxl_len, 1);
        assert_eq!(torque.vendor_len, 1);
        assert_eq!(torque.direction, MapDirection::Both); // RW
        assert_eq!(torque.codec, CodecKind::Identity);

        // Check goal_pwm
        let goal_pwm = find_entry("goal_pwm");
        assert_eq!(goal_pwm.dxl_addr, dxl_addr::GOAL_PWM);
        assert_eq!(goal_pwm.vendor_addr, vendor_addr::GOAL_PWM);
        assert_eq!(goal_pwm.dxl_len, 2);
        assert_eq!(goal_pwm.vendor_len, 2);
        assert_eq!(goal_pwm.direction, MapDirection::Both); // RW
        assert_eq!(goal_pwm.codec, CodecKind::Identity);
    }

    #[test]
    fn test_ctx_builder_bounded_vs_wrap360_affects_clamping() {
        let mut table = ShadowTable::<1024>::new();

        // Test 1: Bounded mode - should clamp
        table
            .write_no_dirty(vendor_addr::OPERATING_MODE, &[0])
            .unwrap();
        table
            .write_no_dirty(vendor_addr::SERVO_POS_KIND, &[0]) // Bounded
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MIN_LIMIT_CDEG, &(-9000i32).to_le_bytes())
            .unwrap();
        table
            .write_no_dirty(vendor_addr::POS_MAX_LIMIT_CDEG, &(9000i32).to_le_bytes())
            .unwrap();

        // Write a large pulse value that exceeds limits
        table
            .write(dxl_addr::GOAL_POSITION, &2000i32.to_le_bytes()) // ~17600 cdeg
            .unwrap();

        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            translate_dirty_facade_to_vendor(&mut view);
        }

        let mut buf = [0u8; 4];
        table.read(vendor_addr::GOAL_POS_CDEG, &mut buf).unwrap();
        let cdeg_bounded = i32::from_le_bytes(buf);
        assert_eq!(cdeg_bounded, 9000, "Bounded mode should clamp to max");

        // Test 2: Wrap360 mode - should NOT clamp
        table
            .write_no_dirty(vendor_addr::SERVO_POS_KIND, &[1]) // Wrap360
            .unwrap();

        // Write the same large pulse value again
        table
            .write(dxl_addr::GOAL_POSITION, &2000i32.to_le_bytes())
            .unwrap();

        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            translate_dirty_facade_to_vendor(&mut view);
        }

        table.read(vendor_addr::GOAL_POS_CDEG, &mut buf).unwrap();
        let cdeg_wrap = i32::from_le_bytes(buf);
        assert_eq!(
            cdeg_wrap,
            pulses_to_cdeg(2000),
            "Wrap360 mode should NOT clamp"
        );
    }
}
