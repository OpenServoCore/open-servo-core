use crate::regions::control;
use control_table::{RegmapError, StagedView, ValidationKind};

pub fn torque_locked(view: &StagedView) -> Result<(), RegmapError> {
    let mut b = [0u8; 1];
    view.read_bytes(control::addr::lifecycle::TORQUE_ENABLE, &mut b)?;
    if b[0] != 0 {
        Err(RegmapError::ValidationError(ValidationKind::Locked))
    } else {
        Ok(())
    }
}
