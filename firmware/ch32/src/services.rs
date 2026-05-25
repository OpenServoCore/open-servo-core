use core::sync::atomic::Ordering;

use heapless::Vec;
use osc_core::{BootMode, DxlIo, RxSnapshot};

use crate::dxl_fast;
use crate::hal::{flash, pfic};
use crate::idle_ring;
use crate::statics::{
    DXL_REBOOT_PENDING, DXL_RX_BUF, DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_BUF_LEN,
};

pub struct Ch32DxlIo;

impl Ch32DxlIo {
    pub const fn new() -> Self {
        Self
    }
}

impl Default for Ch32DxlIo {
    fn default() -> Self {
        Self::new()
    }
}

impl DxlIo for Ch32DxlIo {
    type TxBuf = Vec<u8, DXL_TX_BUF_LEN>;

    fn rx_snapshot(&self) -> RxSnapshot<'_> {
        let write_pos = DXL_RX_WRITE_POS.load(Ordering::Acquire);
        // SAFETY: DMA writes DXL_RX_BUF circularly; we only read indices
        // below the IDLE-published write_pos.
        let ring = unsafe { &*DXL_RX_BUF.get() };
        RxSnapshot::new(ring, write_pos)
    }

    fn tx_buf(&mut self) -> &mut Self::TxBuf {
        // SAFETY: &mut self proves sole-writer; USART1 TC ISR only clears
        // after a start_tx cycle this struct initiated.
        unsafe { &mut *DXL_TX_BUF.get() }
    }

    fn start_tx(&mut self) {
        if dxl_fast::arm_tx() {
            dxl_fast::fire_now();
        }
    }

    fn start_tx_after(&mut self, idle_tick: u32, delay_us: u32) {
        dxl_fast::start_plain_after(idle_tick, delay_us);
    }

    fn idle_for(&self, parsed_end: u32) -> Option<u32> {
        idle_ring::pop_matching(parsed_end)
    }

    fn start_fast_tx_after(
        &mut self,
        idle_tick: u32,
        switch_us: u32,
        fire_us: u32,
        frame_end: u32,
    ) {
        dxl_fast::start_fast_after(idle_tick, switch_us, fire_us, frame_end);
    }

    fn request_reboot(&mut self, mode: BootMode) {
        flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
        DXL_REBOOT_PENDING.store(true, Ordering::Release);
        if self.tx_buf().is_empty() {
            pfic::software_reset();
        }
    }
}
