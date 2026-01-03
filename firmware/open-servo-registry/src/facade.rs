//! Facade translation between DXL-compatible RAM registers and canonical vendor registers.
//!
//! This module provides translation for the core 4 fields:
//! - `torque_enable`: facade 64 <-> vendor 581
//! - `goal_pwm`: facade 100 <-> vendor 583
//! - `goal_position`: facade 116 <-> vendor 512 (with position codec)
//! - `present_position`: facade 132 <-> vendor 516 (RO publish only)

use open_servo_shadow::KernelView;

// ============================================================================
// Address Constants
// ============================================================================

/// Facade (DXL-compatible RAM) addresses.
mod facade_addr {
    pub const TORQUE_ENABLE: u16 = 64;
    pub const GOAL_PWM: u16 = 100;
    pub const GOAL_POSITION: u16 = 116;
    pub const PRESENT_POSITION: u16 = 132;
}

/// Vendor register addresses.
mod vendor_addr {
    pub const GOAL_POS_CDEG: u16 = 512;
    pub const PRESENT_POS_CDEG: u16 = 516;
    pub const POS_MIN_LIMIT_CDEG: u16 = 530;
    pub const POS_MAX_LIMIT_CDEG: u16 = 534;
    pub const TORQUE_ENABLE: u16 = 581;
    pub const OPERATING_MODE: u16 = 582;
    pub const GOAL_PWM: u16 = 583;
    pub const SERVO_POS_KIND: u16 = 625;
}

// ============================================================================
// Types
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
struct CodecCtx {
    #[allow(dead_code)]
    operating_mode: u8,
    servo_pos_kind: ServoPosKind,
    pos_min_cdeg: i32,
    pos_max_cdeg: i32,
}

impl CodecCtx {
    /// Read codec context from vendor registers via KernelView.
    fn from_view(view: &KernelView<'_>) -> Self {
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
// Position Codec
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
// Translation Functions
// ============================================================================

/// Translate dirty RW facade writes into canonical vendor registers.
///
/// For each RW facade field:
/// - If facade range is dirty: read facade, decode, write vendor, mark vendor dirty, clear facade dirty
///
/// This function should be called before `kernel.commit_shadow()`.
pub fn translate_dirty_facade_to_vendor(view: &mut KernelView<'_>) {
    let ctx = CodecCtx::from_view(&*view);

    // 1. torque_enable (facade 64 -> vendor 581, len 1, Bool/identity)
    if view.is_range_dirty(facade_addr::TORQUE_ENABLE, 1) {
        let mut buf = [0u8; 1];
        if view.read(facade_addr::TORQUE_ENABLE, &mut buf).is_ok() {
            let _ = view.write(vendor_addr::TORQUE_ENABLE, &buf);
            view.mark_range_dirty(vendor_addr::TORQUE_ENABLE, 1);
            view.clear_range_dirty(facade_addr::TORQUE_ENABLE, 1);
        }
    }

    // 2. goal_pwm (facade 100 -> vendor 583, len 2, Identity i16 LE)
    if view.is_range_dirty(facade_addr::GOAL_PWM, 2) {
        let mut buf = [0u8; 2];
        if view.read(facade_addr::GOAL_PWM, &mut buf).is_ok() {
            let _ = view.write(vendor_addr::GOAL_PWM, &buf);
            view.mark_range_dirty(vendor_addr::GOAL_PWM, 2);
            view.clear_range_dirty(facade_addr::GOAL_PWM, 2);
        }
    }

    // 3. goal_position (facade 116 -> vendor 512, len 4, Position codec)
    if view.is_range_dirty(facade_addr::GOAL_POSITION, 4) {
        let mut buf = [0u8; 4];
        if view.read(facade_addr::GOAL_POSITION, &mut buf).is_ok() {
            let pulses = i32::from_le_bytes(buf);
            let cdeg = decode_position(pulses, &ctx);
            let _ = view.write(vendor_addr::GOAL_POS_CDEG, &cdeg.to_le_bytes());
            view.mark_range_dirty(vendor_addr::GOAL_POS_CDEG, 4);
            view.clear_range_dirty(facade_addr::GOAL_POSITION, 4);
        }
    }
}

/// Publish canonical vendor telemetry into RO facade fields.
///
/// Reads vendor registers, encodes to facade format, writes facade.
/// Does NOT mark dirty (these are RO from host perspective).
///
/// This function should be called after `kernel.publish_telemetry()`.
pub fn publish_vendor_to_facade(view: &mut KernelView<'_>) {
    let ctx = CodecCtx::from_view(&*view);

    // present_position (vendor 516 -> facade 132, len 4, Position codec)
    let mut buf = [0u8; 4];
    if view.read(vendor_addr::PRESENT_POS_CDEG, &mut buf).is_ok() {
        let cdeg = i32::from_le_bytes(buf);
        let pulses = encode_position(cdeg, &ctx);
        let _ = view.write(facade_addr::PRESENT_POSITION, &pulses.to_le_bytes());
        // NOTE: Do NOT mark dirty - this is RO telemetry
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
            .write(facade_addr::GOAL_POSITION, &1024i32.to_le_bytes())
            .unwrap();
        assert!(table.is_range_dirty(facade_addr::GOAL_POSITION, 4));

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
        assert!(!table.is_range_dirty(facade_addr::GOAL_POSITION, 4));
    }

    #[test]
    fn test_translate_torque_enable() {
        let mut table = ShadowTable::<1024>::new();

        // Host writes facade torque_enable
        table.write(facade_addr::TORQUE_ENABLE, &[1]).unwrap();
        assert!(table.is_range_dirty(facade_addr::TORQUE_ENABLE, 1));

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
        assert!(!table.is_range_dirty(facade_addr::TORQUE_ENABLE, 1));
    }

    #[test]
    fn test_translate_goal_pwm() {
        let mut table = ShadowTable::<1024>::new();

        // Host writes facade goal_pwm (i16 LE)
        let pwm: i16 = -500;
        table
            .write(facade_addr::GOAL_PWM, &pwm.to_le_bytes())
            .unwrap();
        assert!(table.is_range_dirty(facade_addr::GOAL_PWM, 2));

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
        assert!(!table.is_range_dirty(facade_addr::GOAL_PWM, 2));
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
        table.read(facade_addr::PRESENT_POSITION, &mut buf).unwrap();
        let pulses = i32::from_le_bytes(buf);
        assert_eq!(pulses, cdeg_to_pulses(9000)); // ~1023

        // Assert: facade range is NOT dirty
        assert!(!table.is_range_dirty(facade_addr::PRESENT_POSITION, 4));
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
}
