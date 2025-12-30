//! Vendor-specific register definitions (0x200+ / 512+).
//!
//! These are the native high-resolution registers used internally by the kernel.
//! Standard Dynamixel registers (0-251) are a compatibility translation layer.

use crate::spec::RegSpec;
use open_servo_macros::RegMap;

/// Operating mode enum values.
pub const MODE_NAMES: &[&str] = &["Position", "OpenLoop"];

// ============================================================================
// Register Map Definition
// ============================================================================

#[derive(RegMap)]
#[regmap(base = 512, fields = "VENDOR_FIELDS")]
#[allow(dead_code)]
struct VendorRegs {
    // High-Resolution Access (512-539)
    #[reg(RW)]
    goal_pos_cdeg: i32,         // 512
    #[reg(RO)]
    present_pos_cdeg: i32,      // 516
    #[reg(RW)]
    goal_vel_dps10: i32,        // 520
    #[reg(RO)]
    present_temp_centic: i16,   // 524
    #[reg(RO)]
    present_voltage_mv: i16,    // 526
    #[reg(RO)]
    present_current_ma: i16,    // 528
    #[reg(RW)]
    pos_min_limit_cdeg: i32,    // 530
    #[reg(RW)]
    pos_max_limit_cdeg: i32,    // 534
    #[reg(RO)]
    control_flags: u16,         // 538

    // Compliance Control (540-562)
    #[reg(RO)]
    compliance_mode: u8,        // 540
    #[reg(RO)]
    compliance_limited: u8,     // 541
    #[reg(RW)]
    move_current_limit: u16,    // 542
    #[reg(RW)]
    hold_current_limit: u16,    // 544
    #[reg(RW)]
    compliance_hysteresis: u16, // 546
    #[reg(RW)]
    compliance_deglitch: u8,    // 548
    #[reg(RW)]
    backoff_factor: u16,        // 549
    #[reg(RW)]
    recovery_rate: u16,         // 551
    #[reg(RW)]
    hold_enter_error: u16,      // 553
    #[reg(RW)]
    hold_exit_error: u16,       // 555
    #[reg(RW)]
    hold_enter_velocity: u16,   // 557
    #[reg(RW)]
    hold_exit_velocity: u16,    // 559
    #[reg(RW)]
    backdrive_velocity: u16,    // 561

    // Debug/Telemetry (563-580)
    #[reg(RO)]
    control_loop_counter: u32,  // 563
    #[reg(RO)]
    control_output: i16,        // 567
    #[reg(RO)]
    integral_term: i16,         // 569
    #[reg(RO)]
    derivative_term: i16,       // 571
    #[reg(RO)]
    loop_time: u16,             // 573
    #[reg(RO)]
    fault_status: u8,           // 575
    #[reg(RO)]
    fault_count: u8,            // 576
    #[reg(RO)]
    fault_history: u32,         // 577

    // Kernel-specific extensions (581+)
    #[reg(RW)]
    torque_enable: bool,        // 581
    #[reg(RW)]
    operating_mode: u8,         // 582
    #[reg(RW)]
    goal_pwm: i16,              // 583
    #[reg(RO)]
    engaged_mirror: bool,       // 585
    #[reg(RO)]
    mode_mirror: u8,            // 586
    #[reg(RO)]
    gate_reason: u8,            // 587

    // Motor-specific telemetry (588+)
    #[reg(RO)]
    motor_temp_centic: i16,     // 588
    #[reg(RO)]
    motor_vplus_mv: i16,        // 590
    #[reg(RO)]
    motor_vminus_mv: i16,       // 592
}

// ============================================================================
// FieldDesc Aliases for Kernel
// ============================================================================

/// Control fields (kernel writes host-provided values).
pub mod ctrl {
    use super::addr;
    use open_servo_kernel_api::shadow::FieldDesc;

    pub const ENGAGED: FieldDesc = FieldDesc::new(addr::TORQUE_ENABLE, 1);
    pub const MODE: FieldDesc = FieldDesc::new(addr::OPERATING_MODE, 1);
    pub const GOAL_POS: FieldDesc = FieldDesc::new(addr::GOAL_POS_CDEG, 4);
    pub const OPEN_LOOP_EFFORT: FieldDesc = FieldDesc::new(addr::GOAL_PWM, 2);
}

/// Telemetry fields (kernel publishes state).
pub mod telem {
    use super::addr;

    pub const POS_CDEG32: u16 = addr::PRESENT_POS_CDEG;
    pub const EFFORT_RAW: u16 = addr::CONTROL_OUTPUT;
    pub const ENGAGED: u16 = addr::ENGAGED_MIRROR;
    pub const MODE: u16 = addr::MODE_MIRROR;
    pub const FAULT_MASK: u16 = addr::FAULT_HISTORY;
    pub const GATE_REASON: u16 = addr::GATE_REASON;
    pub const AMBIENT_TEMP: u16 = addr::PRESENT_TEMP_CENTIC;
    pub const MCU_VDD_MV: u16 = addr::PRESENT_VOLTAGE_MV;
    pub const MOTOR_CURRENT_MA: u16 = addr::PRESENT_CURRENT_MA;
    pub const MOTOR_TEMP: u16 = addr::MOTOR_TEMP_CENTIC;
    pub const MOTOR_VPLUS_MV: u16 = addr::MOTOR_VPLUS_MV;
    pub const MOTOR_VMINUS_MV: u16 = addr::MOTOR_VMINUS_MV;
}

// ============================================================================
// Lookup
// ============================================================================

pub fn find(name: &str) -> Option<&'static RegSpec> {
    for field in VENDOR_FIELDS {
        if starts_with_ignore_case(field.name, name) {
            return Some(field);
        }
    }
    None
}

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

    #[test]
    fn test_addresses_computed() {
        assert_eq!(addr::GOAL_POS_CDEG, 512);
        assert_eq!(addr::PRESENT_POS_CDEG, 516);
        assert_eq!(addr::PRESENT_TEMP_CENTIC, 524);
        assert_eq!(addr::TORQUE_ENABLE, 581);
        assert_eq!(addr::MOTOR_TEMP_CENTIC, 588);
    }
}
