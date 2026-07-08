//! Consumer/sequence wake providers (osc-servo-transport §6, A3(b)): the
//! dispatch handoff's cross-priority signals, as PFIC software pends.
//!
//! Lane → vector encodes the locked interleave policy through same-class
//! arbitration (lowest number first): a live request pends the core
//! software interrupt (14), beating the kernel's DMA1_CH1 (22); a backlog
//! frame pends the unused I2C1_EV (30), letting a pending kernel tick slot
//! in between queued requests. The completed record pends SysTick (12) —
//! the HIGH transport re-enters `on_deadline` and adopts.

use crate::hal::pfic;
use osc_drivers::traits::bus;
use osc_drivers::traits::bus::Lane;

/// HIGH-side: wake the LOW dispatch consumer for a published job.
pub struct DispatchWake;

impl bus::DispatchWake for DispatchWake {
    fn job_ready(&mut self, lane: Lane) {
        match lane {
            Lane::Live => pfic::pend_software(),
            Lane::Queued => pfic::pend(pfic::Interrupt::I2C1_EV),
        }
    }
}

/// LOW-side: wake the HIGH transport to adopt the completed record.
pub struct SequenceWake;

impl bus::SequenceWake for SequenceWake {
    fn reply_ready(&mut self) {
        pfic::pend_systick();
    }
}
