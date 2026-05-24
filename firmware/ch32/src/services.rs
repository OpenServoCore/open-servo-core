use core::sync::atomic::Ordering;

use ch32_metapac::USART1;
use heapless::Vec;
use osc_core::{BootMode, DxlIo, RxSnapshot};

use crate::hal::{dma, flash, gpio, pfic, systick, usart};
use crate::statics::{
    DXL_IDLE_HEAD, DXL_IDLE_RING, DXL_IDLE_RING_LEN, DXL_IDLE_TAIL, DXL_REBOOT_PENDING, DXL_RX_BUF,
    DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_BUF_LEN, DXL_TX_EN,
};

pub struct Ch32DxlIo;

impl Ch32DxlIo {
    pub const fn new() -> Self {
        Self
    }

    fn arm_tx(&mut self) -> bool {
        let len = self.tx_buf().len();
        if len == 0 {
            return false;
        }
        if let Some(t) = unsafe { *DXL_TX_EN.get() } {
            gpio::set_level(t.pin, t.tx_level);
        }
        dma::set_count(dma::Channel::CH4, len as u16);
        usart::clear_tc(USART1);
        usart::set_dma_tx(USART1, true);
        usart::set_tc_irq(USART1, true);
        true
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
        if !self.arm_tx() {
            return;
        }
        dma::enable(dma::Channel::CH4);
    }

    fn start_tx_after(&mut self, idle_tick: u32, delay_us: u32) {
        systick::set_irq(false);
        systick::clear_match();

        let needed = delay_us.saturating_mul(systick::TICKS_PER_US);

        if systick::ticks().wrapping_sub(idle_tick) >= needed {
            self.start_tx();
            return;
        }

        if !self.arm_tx() {
            return;
        }

        systick::set_cmp(idle_tick.wrapping_add(needed));
        systick::set_irq(true);

        // CNTIF latches only on CNT==CMP up-count; if CNT crossed the deadline
        // before CMP was written, the next match is ~89 s away on wrap.
        if systick::ticks().wrapping_sub(idle_tick) >= needed {
            crate::irq::on_systick_match();
        }
    }

    fn idle_for(&self, parsed_end: u32) -> Option<u32> {
        // Mask IRQs so the producer can't push or evict while we walk the ring.
        critical_section::with(|_| {
            let mask = (DXL_IDLE_RING_LEN as u8) - 1;
            let head = DXL_IDLE_HEAD.load(Ordering::Relaxed);
            let mut tail = DXL_IDLE_TAIL.load(Ordering::Relaxed);
            let mut hit = None;
            while tail != head {
                // SAFETY: producer is the USART1 IDLE ISR, masked by the CS.
                let stamp = unsafe { (*DXL_IDLE_RING.get())[tail as usize] };
                let delta = stamp.bytes.wrapping_sub(parsed_end) as i32;
                if delta > 0 {
                    // Future stamp — leave for a later call.
                    break;
                }
                tail = (tail + 1) & mask;
                if delta == 0 {
                    hit = Some(stamp.tick);
                    break;
                }
            }
            DXL_IDLE_TAIL.store(tail, Ordering::Relaxed);
            hit
        })
    }

    fn request_reboot(&mut self, mode: BootMode) {
        flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
        DXL_REBOOT_PENDING.store(true, Ordering::Release);
        if self.tx_buf().is_empty() {
            pfic::software_reset();
        }
    }
}
