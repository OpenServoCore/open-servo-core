//! Facade translation between DXL-compatible RAM registers and canonical vendor registers.
//!
//! This module provides translation for the core 4 fields:
//! - `torque_enable`: facade 64 <-> vendor 581
//! - `goal_pwm`: facade 100 <-> vendor 583
//! - `goal_position`: facade 116 <-> vendor 512 (with position codec)
//! - `present_position`: facade 132 <-> vendor 516 (RO publish only)
//!
//! # Buffer Limitation
//!
//! Translation buffers are sized to [`MAX_MAP_LEN`] bytes (currently 4).
//! All mapping entries must have `src_len` and `dst_len` <= `MAX_MAP_LEN`.

use open_servo_macros::FacadeMap;
use open_servo_shadow::KernelView;

use crate::ram::addr as ram_addr;
use crate::vendor::addr as vendor_addr;

// ============================================================================
// Constants
// ============================================================================

/// Maximum byte length for a single mapping entry.
///
/// Translation buffers are sized to this value. All `src_len` and `dst_len`
/// in [`FACADE_MAPPINGS`] must be <= this constant.
pub const MAX_MAP_LEN: usize = 4;

// ============================================================================
// Mapping Types
// ============================================================================

/// Direction for facade↔vendor translation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MapDirection {
    /// Facade → Vendor on dirty write.
    ToVendor,
    /// Vendor → Facade on publish (RO telemetry).
    ToFacade,
    /// Bidirectional (RW with read-back).
    Both,
}

/// Codec kind for translation dispatch.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CodecKind {
    /// Copy bytes unchanged (requires equal src/dst lengths).
    Identity,
    /// Pulse ↔ centidegree conversion with optional clamping.
    Position,
}

/// A single facade↔vendor mapping entry.
#[derive(Clone, Copy, Debug)]
pub struct MapEntry {
    /// Field name for debugging.
    pub name: &'static str,
    /// Source address (facade for ToVendor, vendor for ToFacade).
    pub src_addr: u16,
    /// Source byte length.
    pub src_len: u8,
    /// Destination address (vendor for ToVendor, facade for ToFacade).
    pub dst_addr: u16,
    /// Destination byte length.
    pub dst_len: u8,
    /// Translation direction.
    pub direction: MapDirection,
    /// Codec to apply during translation.
    pub codec: CodecKind,
}

// ============================================================================
// Generated Mapping Table
// ============================================================================

/// Facade↔Vendor mapping declarations.
///
/// This struct is only used for the derive macro. The actual mappings are
/// in [`FACADE_MAPPINGS`].
#[derive(FacadeMap)]
#[facademap(table = "FACADE_MAPPINGS")]
#[allow(dead_code)]
struct FacadeMappings {
    #[map(
        src = ram_addr::TORQUE_ENABLE,
        src_len = 1,
        dst = vendor_addr::TORQUE_ENABLE,
        dst_len = 1,
        dir = Both,
        codec = Identity
    )]
    torque_enable: (),

    #[map(
        src = ram_addr::GOAL_PWM,
        src_len = 2,
        dst = vendor_addr::GOAL_PWM,
        dst_len = 2,
        dir = ToVendor,
        codec = Identity
    )]
    goal_pwm: (),

    #[map(
        src = ram_addr::GOAL_POSITION,
        src_len = 4,
        dst = vendor_addr::GOAL_POS_CDEG,
        dst_len = 4,
        dir = ToVendor,
        codec = Position
    )]
    goal_position: (),

    #[map(
        src = ram_addr::PRESENT_POSITION,
        src_len = 4,
        dst = vendor_addr::PRESENT_POS_CDEG,
        dst_len = 4,
        dir = ToFacade,
        codec = Position
    )]
    present_position: (),
}

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

/// Codec context read from vendor registers.
pub struct CodecCtx {
    #[allow(dead_code)]
    pub operating_mode: u8,
    pub servo_pos_kind: ServoPosKind,
    pub pos_min_cdeg: i32,
    pub pos_max_cdeg: i32,
}

impl CodecCtx {
    /// Read codec context from vendor registers via KernelView.
    pub fn from_view(view: &KernelView<'_>) -> Self {
        let mut buf1 = [0u8; 1];
        let mut buf4 = [0u8; 4];

        let operating_mode = if view.read(vendor_addr::OPERATING_MODE, &mut buf1).is_ok() {
            buf1[0]
        } else {
            0
        };

        let servo_pos_kind = if view.read(vendor_addr::SERVO_POS_KIND, &mut buf1).is_ok() {
            ServoPosKind::from(buf1[0])
        } else {
            ServoPosKind::Bounded
        };

        let pos_min_cdeg = if view
            .read(vendor_addr::POS_MIN_LIMIT_CDEG, &mut buf4)
            .is_ok()
        {
            i32::from_le_bytes(buf4)
        } else {
            i32::MIN
        };

        let pos_max_cdeg = if view
            .read(vendor_addr::POS_MAX_LIMIT_CDEG, &mut buf4)
            .is_ok()
        {
            i32::from_le_bytes(buf4)
        } else {
            i32::MAX
        };

        Self {
            operating_mode,
            servo_pos_kind,
            pos_min_cdeg,
            pos_max_cdeg,
        }
    }
}

// ============================================================================
// Codec Trait and Implementations
// ============================================================================

/// Trait for facade↔vendor codecs.
pub trait Codec {
    /// Decode src (facade) bytes to dst (vendor) bytes.
    fn decode(src: &[u8], dst: &mut [u8], ctx: &CodecCtx);
    /// Encode src (vendor) bytes to dst (facade) bytes.
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

/// Decode pulses (facade i32) to centidegrees (vendor i32).
///
/// Conversion: 1 pulse = 0.088 degrees = 8.8 cdeg
/// Formula: cdeg = round(pulses * 8.8) = (pulses * 88 + sign * 5) / 10
fn pulses_to_cdeg(pulses: i32) -> i32 {
    let sign = if pulses >= 0 { 5 } else { -5 };
    (pulses * 88 + sign) / 10
}

/// Encode centidegrees (vendor i32) to pulses (facade i32).
///
/// Formula: pulses = round(cdeg / 8.8) = (cdeg * 10 + sign * 44) / 88
fn cdeg_to_pulses(cdeg: i32) -> i32 {
    let sign = if cdeg >= 0 { 44 } else { -44 };
    (cdeg * 10 + sign) / 88
}

/// Decode pulses to cdeg with optional clamping based on position kind.
fn decode_position(pulses: i32, ctx: &CodecCtx) -> i32 {
    let cdeg = pulses_to_cdeg(pulses);
    if ctx.servo_pos_kind == ServoPosKind::Bounded {
        cdeg.clamp(ctx.pos_min_cdeg, ctx.pos_max_cdeg)
    } else {
        cdeg
    }
}

/// Encode cdeg to pulses with optional clamping based on position kind.
fn encode_position(cdeg: i32, ctx: &CodecCtx) -> i32 {
    let clamped = if ctx.servo_pos_kind == ServoPosKind::Bounded {
        cdeg.clamp(ctx.pos_min_cdeg, ctx.pos_max_cdeg)
    } else {
        cdeg
    };
    cdeg_to_pulses(clamped)
}

// ============================================================================
// Codec Dispatch
// ============================================================================

/// Apply decode (facade → vendor) for the given codec kind.
fn apply_codec_decode(codec: CodecKind, src: &[u8], dst: &mut [u8], ctx: &CodecCtx) {
    match codec {
        CodecKind::Identity => IdentityCodec::decode(src, dst, ctx),
        CodecKind::Position => PositionCodec::decode(src, dst, ctx),
    }
}

/// Apply encode (vendor → facade) for the given codec kind.
fn apply_codec_encode(codec: CodecKind, src: &[u8], dst: &mut [u8], ctx: &CodecCtx) {
    match codec {
        CodecKind::Identity => IdentityCodec::encode(src, dst, ctx),
        CodecKind::Position => PositionCodec::encode(src, dst, ctx),
    }
}

// ============================================================================
// Translation Functions
// ============================================================================

/// Translate dirty RW facade writes into canonical vendor registers.
///
/// For each RW facade field in [`FACADE_MAPPINGS`]:
/// - If facade range is dirty: read facade, decode, write vendor, mark vendor dirty, clear facade dirty
///
/// This function should be called before `kernel.commit_shadow()`.
pub fn translate_dirty_facade_to_vendor(view: &mut KernelView<'_>) {
    let ctx = CodecCtx::from_view(&*view);

    for entry in FACADE_MAPPINGS {
        // Skip entries not in ToVendor direction
        if !matches!(entry.direction, MapDirection::ToVendor | MapDirection::Both) {
            continue;
        }

        // Skip if facade range is not dirty
        if !view.is_range_dirty(entry.src_addr, entry.src_len as u16) {
            continue;
        }

        let mut src_buf = [0u8; MAX_MAP_LEN];
        let mut dst_buf = [0u8; MAX_MAP_LEN];
        let src = &mut src_buf[..entry.src_len as usize];
        let dst = &mut dst_buf[..entry.dst_len as usize];

        if view.read(entry.src_addr, src).is_ok() {
            apply_codec_decode(entry.codec, src, dst, &ctx);
            // Only mark vendor dirty / clear facade dirty if write succeeds
            if view.write(entry.dst_addr, dst).is_ok() {
                view.mark_range_dirty(entry.dst_addr, entry.dst_len as u16);
                view.clear_range_dirty(entry.src_addr, entry.src_len as u16);
            }
        }
    }
}

/// Publish canonical vendor telemetry into RO facade fields.
///
/// Reads vendor registers, encodes to facade format, writes facade.
/// Does NOT mark dirty (these are RO from host perspective).
///
/// Safe write-back: only updates facade if the facade range is NOT dirty,
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

        // Safe write-back: skip if facade is dirty (pending host write)
        if view.is_range_dirty(entry.src_addr, entry.src_len as u16) {
            continue;
        }

        let mut src_buf = [0u8; MAX_MAP_LEN];
        let mut dst_buf = [0u8; MAX_MAP_LEN];
        let src = &mut src_buf[..entry.dst_len as usize]; // read from vendor (dst_len)
        let dst = &mut dst_buf[..entry.src_len as usize]; // write to facade (src_len)

        if view.read(entry.dst_addr, src).is_ok() {
            apply_codec_encode(entry.codec, src, dst, &ctx);
            // Write to facade; ignore errors (RO telemetry, no dirty marking)
            let _ = view.write(entry.src_addr, dst);
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
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

        // Host writes facade goal_position (marks dirty)
        table
            .write(ram_addr::GOAL_POSITION, &1024i32.to_le_bytes())
            .unwrap();
        assert!(table.is_range_dirty(ram_addr::GOAL_POSITION, 4));

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

        // Assert: vendor range dirty, facade range not dirty
        assert!(table.is_range_dirty(vendor_addr::GOAL_POS_CDEG, 4));
        assert!(!table.is_range_dirty(ram_addr::GOAL_POSITION, 4));
    }

    #[test]
    fn test_translate_torque_enable() {
        let mut table = ShadowTable::<1024>::new();

        // Host writes facade torque_enable
        table.write(ram_addr::TORQUE_ENABLE, &[1]).unwrap();
        assert!(table.is_range_dirty(ram_addr::TORQUE_ENABLE, 1));

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

        // Assert: vendor dirty, facade not dirty
        assert!(table.is_range_dirty(vendor_addr::TORQUE_ENABLE, 1));
        assert!(!table.is_range_dirty(ram_addr::TORQUE_ENABLE, 1));
    }

    #[test]
    fn test_translate_goal_pwm() {
        let mut table = ShadowTable::<1024>::new();

        // Host writes facade goal_pwm (i16 LE)
        let pwm: i16 = -500;
        table.write(ram_addr::GOAL_PWM, &pwm.to_le_bytes()).unwrap();
        assert!(table.is_range_dirty(ram_addr::GOAL_PWM, 2));

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

        // Assert: vendor dirty, facade not dirty
        assert!(table.is_range_dirty(vendor_addr::GOAL_PWM, 2));
        assert!(!table.is_range_dirty(ram_addr::GOAL_PWM, 2));
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

        // Assert: facade 132 has expected pulses
        let mut buf = [0u8; 4];
        table.read(ram_addr::PRESENT_POSITION, &mut buf).unwrap();
        let pulses = i32::from_le_bytes(buf);
        assert_eq!(pulses, cdeg_to_pulses(9000)); // ~1023

        // Assert: facade range is NOT dirty
        assert!(!table.is_range_dirty(ram_addr::PRESENT_POSITION, 4));
    }

    #[test]
    fn test_position_clamping_bounded() {
        let ctx = CodecCtx {
            operating_mode: 0,
            servo_pos_kind: ServoPosKind::Bounded,
            pos_min_cdeg: -9000,
            pos_max_cdeg: 9000,
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
            servo_pos_kind: ServoPosKind::Wrap360,
            pos_min_cdeg: -9000,
            pos_max_cdeg: 9000,
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

        // Host writes to facade addr 132 (marks dirty) - simulates pending write
        let host_bytes = 0xDEADBEEFu32.to_le_bytes();
        table
            .write(ram_addr::PRESENT_POSITION, &host_bytes)
            .unwrap();
        assert!(table.is_range_dirty(ram_addr::PRESENT_POSITION, 4));

        // Publish (should skip write because facade is dirty)
        {
            let (bytes, dirty) = table.as_mut_slices();
            let mut view = KernelView::new(bytes, dirty);
            publish_vendor_to_facade(&mut view);
        }

        // Assert: facade 132 remains the host-written bytes (NOT overwritten)
        let mut buf = [0u8; 4];
        table.read(ram_addr::PRESENT_POSITION, &mut buf).unwrap();
        assert_eq!(buf, host_bytes, "facade should retain host-written bytes");

        // Assert: dirty bit remains set
        assert!(table.is_range_dirty(ram_addr::PRESENT_POSITION, 4));
    }

    // ========================================================================
    // New tests for Ticket 6
    // ========================================================================

    #[test]
    fn test_mapping_table_has_expected_entries() {
        // Verify the generated mapping table has 4 entries with correct addresses
        assert_eq!(FACADE_MAPPINGS_COUNT, 4);
        assert_eq!(FACADE_MAPPINGS.len(), 4);

        // Check each entry
        let torque = &FACADE_MAPPINGS[0];
        assert_eq!(torque.name, "torque_enable");
        assert_eq!(torque.src_addr, ram_addr::TORQUE_ENABLE);
        assert_eq!(torque.dst_addr, vendor_addr::TORQUE_ENABLE);
        assert_eq!(torque.src_len, 1);
        assert_eq!(torque.dst_len, 1);
        assert_eq!(torque.direction, MapDirection::Both);
        assert_eq!(torque.codec, CodecKind::Identity);

        let goal_pwm = &FACADE_MAPPINGS[1];
        assert_eq!(goal_pwm.name, "goal_pwm");
        assert_eq!(goal_pwm.src_addr, ram_addr::GOAL_PWM);
        assert_eq!(goal_pwm.dst_addr, vendor_addr::GOAL_PWM);
        assert_eq!(goal_pwm.src_len, 2);
        assert_eq!(goal_pwm.dst_len, 2);
        assert_eq!(goal_pwm.direction, MapDirection::ToVendor);
        assert_eq!(goal_pwm.codec, CodecKind::Identity);

        let goal_pos = &FACADE_MAPPINGS[2];
        assert_eq!(goal_pos.name, "goal_position");
        assert_eq!(goal_pos.src_addr, ram_addr::GOAL_POSITION);
        assert_eq!(goal_pos.dst_addr, vendor_addr::GOAL_POS_CDEG);
        assert_eq!(goal_pos.src_len, 4);
        assert_eq!(goal_pos.dst_len, 4);
        assert_eq!(goal_pos.direction, MapDirection::ToVendor);
        assert_eq!(goal_pos.codec, CodecKind::Position);

        let present_pos = &FACADE_MAPPINGS[3];
        assert_eq!(present_pos.name, "present_position");
        assert_eq!(present_pos.src_addr, ram_addr::PRESENT_POSITION);
        assert_eq!(present_pos.dst_addr, vendor_addr::PRESENT_POS_CDEG);
        assert_eq!(present_pos.src_len, 4);
        assert_eq!(present_pos.dst_len, 4);
        assert_eq!(present_pos.direction, MapDirection::ToFacade);
        assert_eq!(present_pos.codec, CodecKind::Position);
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
            .write(ram_addr::GOAL_POSITION, &2000i32.to_le_bytes()) // ~17600 cdeg
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
            .write(ram_addr::GOAL_POSITION, &2000i32.to_le_bytes())
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
