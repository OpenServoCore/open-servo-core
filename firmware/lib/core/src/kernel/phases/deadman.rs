//! The bus dead-man (kernel sec 2.7): the countdown of `bus_deadman_ticks`
//! both medium halves carry, the experiment image's and the production
//! image's built with `deadman`.

use core::sync::atomic::{Ordering, compiler_fence};

use super::loops::faults::{self, FaultLatch};
use crate::Shared;
use crate::kernel::DECIM_MED;

/// The bus dead-man countdown of `bus_deadman_ticks`, stepped once per
/// medium period on the RAIL phase.
pub(in crate::kernel) struct Deadman {
    /// Ticks left; 0 while off, lapsed or torque off.
    ticks: u16,
    /// `Shared::bus_gen` at the last reload.
    bus_gen: u8,
    /// `Shared::bus_gen` as the CONTROL phase read it, ahead of the
    /// lifecycle.
    seen: u8,
}

impl Deadman {
    pub(in crate::kernel) const fn new() -> Self {
        Self {
            ticks: 0,
            bus_gen: 0,
            seen: 0,
        }
    }

    /// CONTROL, ahead of the lifecycle read.
    #[inline(always)]
    pub(in crate::kernel) fn see(&mut self, shared: &Shared) {
        // dispatch bumps the generation behind the write it reports, so a
        // step seen here is in the lifecycle read that follows
        self.seen = shared.bus_gen();
        compiler_fence(Ordering::Acquire);
    }

    /// The torque enable edge: the countdown starts from the field.
    #[inline(always)]
    pub(in crate::kernel) fn start(&mut self, field: u16) {
        self.ticks = field;
        self.bus_gen = self.seen;
    }

    /// RAIL: the period. Torque off clears; an instruction the CONTROL read
    /// saw dispatched since the last reload reloads the countdown from the
    /// field it read; a streaming burst holds it; else a loaded countdown
    /// counts the period down and latches the fault at zero. Zero never
    /// counts: the field is off, or a write the next CONTROL carries.
    #[inline(always)]
    pub(in crate::kernel) fn step(
        &mut self,
        torque: bool,
        field: u16,
        streaming: bool,
        faults: &mut FaultLatch,
    ) {
        if !torque {
            self.ticks = 0;
        } else if self.bus_gen != self.seen {
            self.bus_gen = self.seen;
            self.ticks = field;
        } else if !streaming && self.ticks != 0 {
            if self.ticks > DECIM_MED as u16 {
                self.ticks -= DECIM_MED as u16;
            } else {
                self.ticks = 0;
                faults.raise(faults::BIT_DEADMAN, faults::CODE_DEADMAN);
            }
        }
    }

    #[cfg(feature = "experiment")]
    pub(in crate::kernel) fn loaded(&self) -> bool {
        self.ticks != 0
    }
}
