use core::sync::atomic::Ordering;

use osc_core::{BootMode, Event, ServiceEvents};

use crate::hal::{flash, pfic};
use crate::legacy::dxl::statics::{
    DXL_BAUD_PENDING_BRR, DXL_CLOCK_FINE_TRIM_PENDING, DXL_CLOCK_TRIM_PENDING, DXL_REBOOT_PENDING,
    DXL_TX_BUF,
};
use crate::legacy::dxl::timing::brr;

pub struct Ch32Events;

impl Ch32Events {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for Ch32Events {
    fn default() -> Self {
        Self::new()
    }
}

impl ServiceEvents for Ch32Events {
    fn send(&mut self, event: Event) {
        match event {
            Event::SetDxlBaud(rate) => {
                DXL_BAUD_PENDING_BRR.store(brr(rate), Ordering::Release);
            }
            Event::SetClockTrim(delta) => {
                DXL_CLOCK_TRIM_PENDING.store(delta as i16, Ordering::Release);
            }
            Event::SetClockFineTrimUs(q88_us) => {
                DXL_CLOCK_FINE_TRIM_PENDING.store(q88_us as i32, Ordering::Release);
            }
            Event::Reboot(mode) => {
                flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
                DXL_REBOOT_PENDING.store(true, Ordering::Release);
                // SAFETY: read-only check of TX buf occupancy; ISR may push
                // past us but never shrinks. Worst case: deferred reset, TC
                // fires it.
                let tx_empty = unsafe { (*DXL_TX_BUF.get()).is_empty() };
                if tx_empty {
                    pfic::software_reset();
                }
            }
        }
    }
}
