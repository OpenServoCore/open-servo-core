//! Vendor-specific register definitions (0x200+ / 512+).
//!
//! These are the native high-resolution registers used internally by the kernel.
//! Standard Dynamixel registers (0-251) are a compatibility translation layer.

use crate::spec::{Access, Encoding, RegSpec};

/// Operating mode enum values.
pub const MODE_NAMES: &[&str] = &["Position", "OpenLoop"];

// ============================================================================
// Vendor Register Addresses (from REGISTER_MAP.md)
// ============================================================================

/// High-resolution position/velocity registers.
pub mod addr {
    // High-Resolution Access (512-539)
    pub const GOAL_POS_CDEG: u16 = 512;
    pub const PRESENT_POS_CDEG: u16 = 516;
    pub const GOAL_VEL_DPS10: u16 = 520;
    pub const PRESENT_TEMP_CENTIC: u16 = 524;
    pub const PRESENT_VOLTAGE_MV: u16 = 526;
    pub const PRESENT_CURRENT_MA: u16 = 528;
    pub const POS_MIN_LIMIT_CDEG: u16 = 530;
    pub const POS_MAX_LIMIT_CDEG: u16 = 534;
    pub const CONTROL_FLAGS: u16 = 538;

    // Compliance Control (540-562)
    pub const COMPLIANCE_MODE: u16 = 540;
    pub const COMPLIANCE_LIMITED: u16 = 541;
    pub const MOVE_CURRENT_LIMIT: u16 = 542;
    pub const HOLD_CURRENT_LIMIT: u16 = 544;
    pub const COMPLIANCE_HYSTERESIS: u16 = 546;
    pub const COMPLIANCE_DEGLITCH: u16 = 548;
    pub const BACKOFF_FACTOR: u16 = 549;
    pub const RECOVERY_RATE: u16 = 551;
    pub const HOLD_ENTER_ERROR: u16 = 553;
    pub const HOLD_EXIT_ERROR: u16 = 555;
    pub const HOLD_ENTER_VELOCITY: u16 = 557;
    pub const HOLD_EXIT_VELOCITY: u16 = 559;
    pub const BACKDRIVE_VELOCITY: u16 = 561;

    // Debug/Telemetry (563-580)
    pub const CONTROL_LOOP_COUNTER: u16 = 563;
    pub const CONTROL_OUTPUT: u16 = 567;
    pub const INTEGRAL_TERM: u16 = 569;
    pub const DERIVATIVE_TERM: u16 = 571;
    pub const LOOP_TIME: u16 = 573;
    pub const FAULT_STATUS: u16 = 575;
    pub const FAULT_COUNT: u16 = 576;
    pub const FAULT_HISTORY: u16 = 577;

    // Kernel-specific extensions (581+)
    pub const TORQUE_ENABLE: u16 = 581;
    pub const OPERATING_MODE: u16 = 582;
    pub const GOAL_PWM: u16 = 583;
    pub const ENGAGED_MIRROR: u16 = 585;
    pub const MODE_MIRROR: u16 = 586;
    pub const GATE_REASON: u16 = 587;
}

// ============================================================================
// Register Specifications
// ============================================================================

/// All vendor region fields (for REPL/debug introspection).
pub const VENDOR_FIELDS: &[RegSpec] = &[
    // High-Resolution Access
    RegSpec::new(
        "goal_pos_cdeg",
        addr::GOAL_POS_CDEG,
        Encoding::I32Le,
        Access::Rw,
    ),
    RegSpec::new(
        "present_pos_cdeg",
        addr::PRESENT_POS_CDEG,
        Encoding::I32Le,
        Access::R,
    ),
    RegSpec::new(
        "goal_vel_dps10",
        addr::GOAL_VEL_DPS10,
        Encoding::I32Le,
        Access::Rw,
    ),
    RegSpec::new(
        "present_temp_centic",
        addr::PRESENT_TEMP_CENTIC,
        Encoding::I16Le,
        Access::R,
    ),
    RegSpec::new(
        "present_voltage_mv",
        addr::PRESENT_VOLTAGE_MV,
        Encoding::U16Le,
        Access::R,
    ),
    RegSpec::new(
        "present_current_ma",
        addr::PRESENT_CURRENT_MA,
        Encoding::I16Le,
        Access::R,
    ),
    RegSpec::new(
        "pos_min_limit_cdeg",
        addr::POS_MIN_LIMIT_CDEG,
        Encoding::I32Le,
        Access::Rw,
    ),
    RegSpec::new(
        "pos_max_limit_cdeg",
        addr::POS_MAX_LIMIT_CDEG,
        Encoding::I32Le,
        Access::Rw,
    ),
    RegSpec::new(
        "control_flags",
        addr::CONTROL_FLAGS,
        Encoding::U16Le,
        Access::R,
    ),
    // Compliance Control
    RegSpec::new(
        "compliance_mode",
        addr::COMPLIANCE_MODE,
        Encoding::U8,
        Access::R,
    ),
    RegSpec::new(
        "compliance_limited",
        addr::COMPLIANCE_LIMITED,
        Encoding::U8,
        Access::R,
    ),
    RegSpec::new(
        "move_current_limit",
        addr::MOVE_CURRENT_LIMIT,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "hold_current_limit",
        addr::HOLD_CURRENT_LIMIT,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "compliance_hysteresis",
        addr::COMPLIANCE_HYSTERESIS,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "compliance_deglitch",
        addr::COMPLIANCE_DEGLITCH,
        Encoding::U8,
        Access::Rw,
    ),
    RegSpec::new(
        "backoff_factor",
        addr::BACKOFF_FACTOR,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "recovery_rate",
        addr::RECOVERY_RATE,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "hold_enter_error",
        addr::HOLD_ENTER_ERROR,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "hold_exit_error",
        addr::HOLD_EXIT_ERROR,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "hold_enter_velocity",
        addr::HOLD_ENTER_VELOCITY,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "hold_exit_velocity",
        addr::HOLD_EXIT_VELOCITY,
        Encoding::U16Le,
        Access::Rw,
    ),
    RegSpec::new(
        "backdrive_velocity",
        addr::BACKDRIVE_VELOCITY,
        Encoding::U16Le,
        Access::Rw,
    ),
    // Debug/Telemetry
    RegSpec::new(
        "control_loop_counter",
        addr::CONTROL_LOOP_COUNTER,
        Encoding::U32Le,
        Access::R,
    ),
    RegSpec::new(
        "control_output",
        addr::CONTROL_OUTPUT,
        Encoding::I16Le,
        Access::R,
    ),
    RegSpec::new(
        "integral_term",
        addr::INTEGRAL_TERM,
        Encoding::I16Le,
        Access::R,
    ),
    RegSpec::new(
        "derivative_term",
        addr::DERIVATIVE_TERM,
        Encoding::I16Le,
        Access::R,
    ),
    RegSpec::new("loop_time", addr::LOOP_TIME, Encoding::U16Le, Access::R),
    RegSpec::new("fault_status", addr::FAULT_STATUS, Encoding::U8, Access::R),
    RegSpec::new("fault_count", addr::FAULT_COUNT, Encoding::U8, Access::R),
    RegSpec::new(
        "fault_history",
        addr::FAULT_HISTORY,
        Encoding::U32Le,
        Access::R,
    ),
    // Kernel-specific extensions
    RegSpec::new(
        "torque_enable",
        addr::TORQUE_ENABLE,
        Encoding::Bool,
        Access::Rw,
    ),
    RegSpec::new(
        "operating_mode",
        addr::OPERATING_MODE,
        Encoding::Enum(MODE_NAMES),
        Access::Rw,
    ),
    RegSpec::new("goal_pwm", addr::GOAL_PWM, Encoding::I16Le, Access::Rw),
    RegSpec::new(
        "engaged_mirror",
        addr::ENGAGED_MIRROR,
        Encoding::Bool,
        Access::R,
    ),
    RegSpec::new(
        "mode_mirror",
        addr::MODE_MIRROR,
        Encoding::Enum(MODE_NAMES),
        Access::R,
    ),
    RegSpec::new("gate_reason", addr::GATE_REASON, Encoding::U8, Access::R),
];

// ============================================================================
// FieldDesc Aliases for Kernel
// ============================================================================

/// Control fields (kernel writes host-provided values).
pub mod ctrl {
    use super::addr;
    use open_servo_kernel_api::shadow::FieldDesc;

    /// Torque enable (u8: 0=disengaged, 1=engaged).
    pub const ENGAGED: FieldDesc = FieldDesc::new(addr::TORQUE_ENABLE, 1);

    /// Operating mode (u8: 0=Position, 1=OpenLoop).
    pub const MODE: FieldDesc = FieldDesc::new(addr::OPERATING_MODE, 1);

    /// Position setpoint (i32 LE, centi-degrees).
    pub const GOAL_POS: FieldDesc = FieldDesc::new(addr::GOAL_POS_CDEG, 4);

    /// Open-loop effort / PWM (i16 LE).
    pub const OPEN_LOOP_EFFORT: FieldDesc = FieldDesc::new(addr::GOAL_PWM, 2);
}

/// Telemetry fields (kernel publishes state).
pub mod telem {
    use super::addr;

    /// Current position (i32 LE, centi-degrees).
    pub const POS_CDEG32: u16 = addr::PRESENT_POS_CDEG;

    /// Last commanded effort (i16 LE, raw).
    pub const EFFORT_RAW: u16 = addr::CONTROL_OUTPUT;

    /// Engaged state mirror (u8).
    pub const ENGAGED: u16 = addr::ENGAGED_MIRROR;

    /// Operating mode mirror (u8).
    pub const MODE: u16 = addr::MODE_MIRROR;

    /// Fault mask (u32 LE, bitfield).
    pub const FAULT_MASK: u16 = addr::FAULT_HISTORY;

    /// Gate reason (u8).
    pub const GATE_REASON: u16 = addr::GATE_REASON;
}

// ============================================================================
// Lookup
// ============================================================================

/// Lookup field by name (case-insensitive prefix match).
pub fn find(name: &str) -> Option<&'static RegSpec> {
    for field in VENDOR_FIELDS {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_exact() {
        let spec = find("goal_pos_cdeg").unwrap();
        assert_eq!(spec.address, 512);
        assert_eq!(spec.len(), 4);
    }

    #[test]
    fn test_find_prefix() {
        let spec = find("present_pos").unwrap();
        assert_eq!(spec.name, "present_pos_cdeg");
        assert_eq!(spec.address, 516);
    }

    #[test]
    fn test_find_case_insensitive() {
        let spec = find("FAULT_HISTORY").unwrap();
        assert_eq!(spec.name, "fault_history");
    }

    #[test]
    fn test_ctrl_field_addresses() {
        assert_eq!(ctrl::ENGAGED.offset, 581);
        assert_eq!(ctrl::GOAL_POS.offset, 512);
    }

    #[test]
    fn test_telem_field_addresses() {
        assert_eq!(telem::POS_CDEG32, 516);
        assert_eq!(telem::EFFORT_RAW, 567);
        assert_eq!(telem::FAULT_MASK, 577);
    }
}
