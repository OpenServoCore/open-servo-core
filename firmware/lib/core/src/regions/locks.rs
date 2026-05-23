use crate::regions::control;
use control_table::{StagedView, ValidationKind};

pub fn torque_locked(view: &StagedView) -> Result<(), control_table::Error> {
    let mut b = [0u8; 1];
    view.read_bytes(control::addr::lifecycle::TORQUE_ENABLE, &mut b)?;
    if b[0] != 0 {
        Err(control_table::Error::ValidationError(
            ValidationKind::Locked,
        ))
    } else {
        Ok(())
    }
}
