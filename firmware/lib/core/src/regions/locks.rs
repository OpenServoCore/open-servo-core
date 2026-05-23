use crate::regions::control;
use crate::{Error, StagedView, ValidationKind};

pub fn torque_locked(view: &StagedView) -> Result<(), Error> {
    let mut b = [0u8; 1];
    view.read_bytes(control::addr::lifecycle::TORQUE_ENABLE, &mut b)?;
    if b[0] != 0 {
        Err(Error::ValidationError(ValidationKind::Locked))
    } else {
        Ok(())
    }
}
