//! Minimal host-plane register mapping (Stage-0).
//!
//! This is *not* the full registry/regmap architecture.
//! It’s just enough to wire Dynamixel service to kernel controls cleanly.
//!
//! When you later introduce a real regmap with dirty flags, you can replace this
//! module without changing the realtime tick pipeline.

use open_servo_kernel_api::mode::OperatingMode;
use open_servo_kernel_api::regs::{RegError, RegValue};
use open_servo_units::{CentiDeg32, Effort};

/// Stage-0 register addresses (project-local).
///
/// You can later replace these with your REGISTER_MAP.md mapping.
pub mod addr {
    use open_servo_kernel_api::regs::RegAddr;

    pub const ENGAGED: RegAddr = 0x0001;
    pub const MODE: RegAddr = 0x0002;

    pub const POS_SP_CDEG32: RegAddr = 0x0010;
    pub const OPEN_LOOP_EFFORT_RAW: RegAddr = 0x0011;

    // Debug
    pub const LAST_GATE: RegAddr = 0x0020;
    pub const LAST_EFFORT_RAW: RegAddr = 0x0021;
    pub const LAST_POS_CDEG32: RegAddr = 0x0022;
}

#[inline]
pub fn reg_read_engaged(engaged: bool) -> RegValue {
    RegValue::U32(if engaged { 1 } else { 0 })
}

#[inline]
pub fn reg_write_engaged(v: RegValue) -> Result<bool, RegError> {
    match v {
        RegValue::U32(x) => Ok(x != 0),
        _ => Err(RegError::InvalidValue),
    }
}

#[inline]
pub fn reg_read_mode(mode: OperatingMode) -> RegValue {
    RegValue::U32(mode as u32)
}

#[inline]
pub fn reg_write_mode(v: RegValue) -> Result<OperatingMode, RegError> {
    match v {
        RegValue::U32(0) => Ok(OperatingMode::Position),
        RegValue::U32(1) => Ok(OperatingMode::OpenLoop),
        _ => Err(RegError::InvalidValue),
    }
}

#[inline]
pub fn reg_read_pos_sp(sp: CentiDeg32) -> RegValue {
    RegValue::I32(sp.as_cdeg())
}

#[inline]
pub fn reg_write_pos_sp(v: RegValue) -> Result<CentiDeg32, RegError> {
    match v {
        RegValue::I32(x) => Ok(CentiDeg32::from_cdeg(x)),
        _ => Err(RegError::InvalidValue),
    }
}

#[inline]
pub fn reg_read_effort(e: Effort) -> RegValue {
    RegValue::I32(e.as_raw() as i32)
}

#[inline]
pub fn reg_write_effort(v: RegValue) -> Result<Effort, RegError> {
    match v {
        RegValue::I32(x) => {
            let raw = x.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            Ok(Effort::from_raw(raw))
        }
        _ => Err(RegError::InvalidValue),
    }
}
