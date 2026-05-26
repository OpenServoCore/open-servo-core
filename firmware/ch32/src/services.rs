use core::sync::atomic::Ordering;

use heapless::Vec;
use osc_core::{BootMode, DeviceControl, DxlBus, RxSnapshot, ServicesIo};

use crate::dxl_fast;
use crate::hal::{flash, pfic};
use crate::idle_ring;
use crate::statics::{
    DXL_REBOOT_PENDING, DXL_RX_BUF, DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_BUF_LEN,
};

/// Single &mut writer: the main loop holding the `Services` struct.
pub struct Ch32Bus {
    /// SysTick at the request's trailing IDLE, stashed by `request_complete`
    /// and consumed by the next `send_after*` to schedule the deadline.
    idle_tick: Option<u32>,
}

impl Ch32Bus {
    pub const fn new() -> Self {
        Self { idle_tick: None }
    }
}

impl Default for Ch32Bus {
    fn default() -> Self {
        Self::new()
    }
}

impl DxlBus for Ch32Bus {
    type ReplyBuffer = Vec<u8, DXL_TX_BUF_LEN>;

    fn received(&self) -> RxSnapshot<'_> {
        let write_pos = DXL_RX_WRITE_POS.load(Ordering::Acquire);
        // SAFETY: DMA writes DXL_RX_BUF circularly; we only read indices
        // below the IDLE-published write_pos.
        let ring = unsafe { &*DXL_RX_BUF.get() };
        RxSnapshot::new(ring, write_pos)
    }

    fn reply_buffer(&mut self) -> &mut Self::ReplyBuffer {
        // SAFETY: &mut self proves sole-writer; USART1 TC ISR only clears
        // after a send cycle this struct initiated.
        unsafe { &mut *DXL_TX_BUF.get() }
    }

    fn request_complete(&mut self, request_end: u32) -> bool {
        match idle_ring::pop_matching(request_end) {
            Some(tick) => {
                self.idle_tick = Some(tick);
                true
            }
            None => {
                self.idle_tick = None;
                false
            }
        }
    }

    fn send(&mut self) {
        if dxl_fast::arm_tx() {
            dxl_fast::fire_now();
        }
    }

    fn send_after(&mut self, delay_us: u32) {
        let Some(idle_tick) = self.idle_tick.take() else {
            return;
        };
        dxl_fast::start_plain_after(idle_tick, delay_us);
    }

    fn send_with_snoop_crc(&mut self, delay_us: u32, snoop_from: Option<u32>) {
        let Some(idle_tick) = self.idle_tick.take() else {
            return;
        };
        dxl_fast::start_fast_after(idle_tick, delay_us, snoop_from);
    }
}

pub struct Ch32Device;

impl Ch32Device {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for Ch32Device {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceControl for Ch32Device {
    fn reboot(&mut self, mode: BootMode) {
        flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
        DXL_REBOOT_PENDING.store(true, Ordering::Release);
        // SAFETY: read-only check of TX buf occupancy; ISR may push past us
        // but never shrinks. Worst case: deferred reset, then TC fires it.
        let tx_empty = unsafe { (*DXL_TX_BUF.get()).is_empty() };
        if tx_empty {
            pfic::software_reset();
        }
    }
}

pub struct Ch32ServicesIo {
    pub bus: Ch32Bus,
    pub device: Ch32Device,
}

impl Ch32ServicesIo {
    pub const fn new() -> Self {
        Self {
            bus: Ch32Bus::new(),
            device: Ch32Device::new(),
        }
    }
}

impl Default for Ch32ServicesIo {
    fn default() -> Self {
        Self::new()
    }
}

impl ServicesIo for Ch32ServicesIo {
    type Bus = Ch32Bus;
    type Device = Ch32Device;

    fn parts(&mut self) -> (&mut Ch32Bus, &mut Ch32Device) {
        (&mut self.bus, &mut self.device)
    }
}
