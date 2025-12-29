//! Shadow table field definitions for ServoKernel.
//!
//! These offsets define the byte layout within shadow table regions.

use crate::spec::{Encoding, RegSpec};

/// Mode enum values for encoding.
pub const MODE_NAMES: &[&str] = &["Position", "OpenLoop"];

/// All control region fields.
pub const CTRL_FIELDS: &[RegSpec] = &[
    RegSpec::new("ctrl.engaged", 0x80, 1, Encoding::Bool, true),
    RegSpec::new("ctrl.mode", 0x81, 1, Encoding::Enum(MODE_NAMES), true),
    RegSpec::new("ctrl.goal_pos", 0x84, 4, Encoding::I32Le, true),
    RegSpec::new("ctrl.ol_effort", 0x88, 2, Encoding::I16Le, true),
];

/// All telemetry region fields.
pub const TELEM_FIELDS: &[RegSpec] = &[
    RegSpec::new("telem.pos", 0x00, 4, Encoding::I32Le, false),
    RegSpec::new("telem.effort", 0x04, 2, Encoding::I16Le, false),
    RegSpec::new("telem.engaged", 0x06, 1, Encoding::Bool, false),
    RegSpec::new("telem.mode", 0x07, 1, Encoding::Enum(MODE_NAMES), false),
    RegSpec::new("telem.fault", 0x08, 4, Encoding::U32Le, false),
    RegSpec::new("telem.gate", 0x0C, 1, Encoding::U8, false),
];

/// All fields combined.
pub const ALL_FIELDS: &[RegSpec] = &[
    // ctrl
    RegSpec::new("ctrl.engaged", 0x80, 1, Encoding::Bool, true),
    RegSpec::new("ctrl.mode", 0x81, 1, Encoding::Enum(MODE_NAMES), true),
    RegSpec::new("ctrl.goal_pos", 0x84, 4, Encoding::I32Le, true),
    RegSpec::new("ctrl.ol_effort", 0x88, 2, Encoding::I16Le, true),
    // telem
    RegSpec::new("telem.pos", 0x00, 4, Encoding::I32Le, false),
    RegSpec::new("telem.effort", 0x04, 2, Encoding::I16Le, false),
    RegSpec::new("telem.engaged", 0x06, 1, Encoding::Bool, false),
    RegSpec::new("telem.mode", 0x07, 1, Encoding::Enum(MODE_NAMES), false),
    RegSpec::new("telem.fault", 0x08, 4, Encoding::U32Le, false),
    RegSpec::new("telem.gate", 0x0C, 1, Encoding::U8, false),
];

/// Lookup field by name (case-insensitive prefix match).
pub fn find(name: &str) -> Option<&'static RegSpec> {
    // Simple linear search for prefix match (case-insensitive)
    for field in ALL_FIELDS {
        if starts_with_ignore_case(field.name, name) {
            return Some(field);
        }
    }
    None
}

/// Check if `haystack` starts with `needle` (ASCII case-insensitive).
fn starts_with_ignore_case(haystack: &str, needle: &str) -> bool {
    if needle.len() > haystack.len() {
        return false;
    }
    haystack
        .as_bytes()
        .iter()
        .zip(needle.as_bytes().iter())
        .all(|(&h, &n)| h.to_ascii_lowercase() == n.to_ascii_lowercase())
}

/// FieldDesc aliases for kernel compatibility.
///
/// These match the original `shadow_fields.rs` API so kernel can import
/// from here without changing its internal code.
pub mod compat {
    use open_servo_kernel_api::shadow::FieldDesc;

    /// Control region field descriptors.
    pub mod ctrl {
        use super::FieldDesc;

        /// Engaged flag (u8: 0=disengaged, 1=engaged).
        pub const ENGAGED: FieldDesc = FieldDesc::new(0x80, 1);

        /// Operating mode (u8: 0=Position, 1=OpenLoop).
        pub const MODE: FieldDesc = FieldDesc::new(0x81, 1);

        /// Position setpoint (i32 LE, centi-degrees).
        pub const GOAL_POS: FieldDesc = FieldDesc::new(0x84, 4);

        /// Open-loop effort (i16 LE, raw effort).
        pub const OPEN_LOOP_EFFORT: FieldDesc = FieldDesc::new(0x88, 2);
    }

    /// Telemetry region field offsets.
    pub mod telem {
        /// Current position (i32 LE, centi-degrees).
        pub const POS_CDEG32: u16 = 0x00;

        /// Last commanded effort (i16 LE, raw).
        pub const EFFORT_RAW: u16 = 0x04;

        /// Engaged state mirror (u8).
        pub const ENGAGED: u16 = 0x06;

        /// Operating mode mirror (u8).
        pub const MODE: u16 = 0x07;

        /// Fault mask (u32 LE, bitfield).
        pub const FAULT_MASK: u16 = 0x08;

        /// Gate reason (u8).
        pub const GATE_REASON: u16 = 0x0C;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_exact() {
        let spec = find("ctrl.engaged").unwrap();
        assert_eq!(spec.offset, 0x80);
        assert_eq!(spec.len, 1);
    }

    #[test]
    fn test_find_prefix() {
        let spec = find("ctrl.goal").unwrap();
        assert_eq!(spec.name, "ctrl.goal_pos");
        assert_eq!(spec.offset, 0x84);
    }

    #[test]
    fn test_find_case_insensitive() {
        let spec = find("TELEM.POS").unwrap();
        assert_eq!(spec.name, "telem.pos");
    }

    #[test]
    fn test_find_not_found() {
        assert!(find("nonexistent").is_none());
    }
}
