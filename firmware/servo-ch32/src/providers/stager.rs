//! Stager-wake provider - binds `StagerWake` to the SW vector, whose body
//! is the TEL stager (`runtime::isr::on_tel_stage`).

use osc_servo_drivers::traits::bus;

use crate::hal::pfic;

#[derive(Copy, Clone)]
pub struct Stager;

impl bus::StagerWake for Stager {
    #[inline(always)]
    fn wake(&self) {
        pfic::pend_software();
    }
}
