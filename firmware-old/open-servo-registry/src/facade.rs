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
//!
//! # Translation Architecture
//!
//! - `translate_dirty_facade_to_vendor()` - DELETED (merged into kernel's commit_shadow)
//! - `publish_vendor_to_facade()` - Publishes RO telemetry from vendor to facade
//! - Position codec helpers - Convert between pulses and centidegrees

use embedded_shadow::view::KernelView;

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
pub fn pulses_to_cdeg(pulses: i32) -> i32 {
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

// NOTE: translate_dirty_facade_to_vendor() has been DELETED.
// Facade-to-vendor translation is now merged into the kernel's commit_shadow()
// method, which checks both facade and vendor dirty bits inline.

/// Publish canonical vendor telemetry into RO dxl fields.
///
/// Reads vendor registers, encodes to dxl format, writes dxl.
/// Does NOT mark dirty (these are RO from host perspective).
///
/// Safe write-back: only updates dxl if the dxl range is NOT dirty,
/// avoiding clobber of pending host writes.
///
/// This function should be called after `kernel.publish_telemetry()`.
pub fn publish_vendor_to_facade<const TS: usize, const BS: usize, const BC: usize>(
    view: &mut KernelView<'_, TS, BS, BC>,
) where
    bitmaps::BitsImpl<BC>: bitmaps::Bits,
{
    let ctx = CodecCtx::from_view(view);

    for entry in FACADE_MAPPINGS {
        // Skip entries not in ToFacade direction
        if !matches!(entry.direction, MapDirection::ToFacade | MapDirection::Both) {
            continue;
        }

        // Safe write-back: skip if dxl is dirty (pending host write)
        if view.is_dirty(entry.dxl_addr, entry.dxl_len as usize).unwrap_or(false) {
            continue;
        }

        let mut src_buf = [0u8; MAX_MAP_LEN];
        let mut dst_buf = [0u8; MAX_MAP_LEN];
        let src = &mut src_buf[..entry.vendor_len as usize]; // read from vendor
        let dst = &mut dst_buf[..entry.dxl_len as usize]; // write to dxl

        if view.read_range(entry.vendor_addr, src).is_ok() {
            apply_codec_encode(entry.codec, src, dst, &ctx);
            // Write to dxl; ignore errors (RO telemetry, no dirty marking)
            let _ = view.write_range(entry.dxl_addr, dst);
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
    use crate::vendor::{addr as vendor_addr, CodecCtx};

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

    // NOTE: Tests for translate_dirty_facade_to_vendor() removed.
    // Facade translation is now merged into the kernel's commit_shadow().

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

    // NOTE: Integration tests for publish_vendor_to_facade() have been temporarily
    // removed during the embedded-shadow migration. They will be re-added when
    // test infrastructure using the new storage types is in place.

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
}
