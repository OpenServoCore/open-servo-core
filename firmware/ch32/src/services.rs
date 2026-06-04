use core::sync::atomic::Ordering;

use heapless::Vec;
use osc_core::{BootMode, DxlBus, Event, RxSnapshot, ServiceEvents, ServicesIo};

use crate::dxl;
use crate::dxl::statics::{
    DXL_BAUD_PENDING_BRR, DXL_CLOCK_FINE_TRIM_PENDING, DXL_CLOCK_TRIM_PENDING, DXL_REBOOT_PENDING,
    DXL_RX_BUF, DXL_TX_BUF, DXL_TX_BUF_LEN, RX_MASK_U32,
};
use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;
use crate::hal::{flash, pfic};
use crate::idle_anchor::{self, IdleAnchor};

/// Single &mut writer: the main loop holding the `Services` struct.
pub struct Ch32Bus {
    /// Latest IDLE anchor consumed by `rx_poll`; `tick` feeds the following
    /// `send_*` call.
    anchor: IdleAnchor,
}

impl Ch32Bus {
    pub const fn new() -> Self {
        Self {
            anchor: IdleAnchor::empty(),
        }
    }
}

impl Default for Ch32Bus {
    fn default() -> Self {
        Self::new()
    }
}

impl DxlBus for Ch32Bus {
    type TxBuffer = Vec<u8, DXL_TX_BUF_LEN>;

    fn rx_poll(&mut self) -> Option<RxSnapshot<'_>> {
        let fresh = idle_anchor::snapshot();
        if fresh.seq == self.anchor.seq {
            return None;
        }
        self.anchor = fresh;
        // SAFETY: DMA writes DXL_RX_BUF circularly; we only read indices
        // below the IDLE-published wire-end position.
        let ring = unsafe { &*DXL_RX_BUF.get() };
        let write_pos = (fresh.bytes & RX_MASK_U32) as u16;
        Some(RxSnapshot::new(ring, write_pos))
    }

    fn tx_buffer(&mut self) -> &mut Self::TxBuffer {
        // SAFETY: &mut self proves sole-writer; USART1 TC ISR only clears
        // after a send cycle this struct initiated.
        unsafe { &mut *DXL_TX_BUF.get() }
    }

    fn send_after(&mut self, delay_us: u32) {
        // Flush any stale slot setup: an unfired SysTick CMP from a prior
        // Sync/Fast op would otherwise re-fire DMA and patch CRC over this
        // reply's buffer.
        dxl::cancel();
        dxl::start_plain_after(self.anchor.tick, delay_us);
    }

    fn send_with_snoop_crc(&mut self, delay_q88_us: u32, snoop_from: Option<u32>) {
        dxl::start_fast_after(self.anchor.tick, delay_q88_us, snoop_from);
    }
}

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
                let brr = usart::brr(PCLK_HZ, rate.as_hz());
                DXL_BAUD_PENDING_BRR.store(brr, Ordering::Release);
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

pub struct Ch32ServicesIo {
    pub bus: Ch32Bus,
    pub events: Ch32Events,
}

impl Ch32ServicesIo {
    pub const fn new() -> Self {
        Self {
            bus: Ch32Bus::new(),
            events: Ch32Events::new(),
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
    type Events = Ch32Events;

    fn parts(&mut self) -> (&mut Ch32Bus, &mut Ch32Events) {
        (&mut self.bus, &mut self.events)
    }
}
