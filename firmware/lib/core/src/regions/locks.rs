use crate::regions::control;
use crate::regmap::{RegmapError, StagedView, ValidationKind};

pub fn torque_locked(view: &StagedView) -> Result<(), RegmapError> {
    let mut b = [0u8; 1];
    view.read_bytes(control::FIELD_TORQUE_ENABLE.addr, &mut b)?;
    if b[0] != 0 {
        Err(RegmapError::ValidationError(ValidationKind::Locked))
    } else {
        Ok(())
    }
}
