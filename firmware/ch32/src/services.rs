use core::sync::atomic::Ordering;

use heapless::Vec;
use osc_core::{BaudRate, BootMode, DeviceControl, DxlBus, RegionStorage, RxSnapshot, ServicesIo};

use crate::dxl_fast;
use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;
use crate::hal::{flash, pfic, rcc};
use crate::idle_ring;
use crate::statics::{
    DXL_BAUD_PENDING_BRR, DXL_BYTE_TIME_TICKS, DXL_REBOOT_PENDING, DXL_RX_BUF, DXL_RX_BYTE_COUNT,
    DXL_RX_STAMP_FIRST, DXL_RX_STAMP_LAST, DXL_RX_WRITE_POS, DXL_TX_BUF, DXL_TX_BUF_LEN, SHARED,
};

/// Single &mut writer: the main loop holding the `Services` struct.
pub struct Ch32Bus {
    /// SysTick at the request's stop-bit completion, stashed by
    /// `request_complete` and consumed by the next `send_after*` to schedule
    /// the deadline.
    request_end_tick: Option<u32>,
}

impl Ch32Bus {
    pub const fn new() -> Self {
        Self {
            request_end_tick: None,
        }
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
                self.request_end_tick = Some(tick);
                true
            }
            None => {
                self.request_end_tick = None;
                false
            }
        }
    }

    fn send_after(&mut self, delay_us: u32) {
        // Flush any stale slot setup: an unfired SysTick CMP from a prior
        // Sync/Fast op would otherwise re-fire DMA and patch CRC over this
        // reply's buffer.
        dxl_fast::cancel();
        match self.request_end_tick.take() {
            Some(request_end_tick) => dxl_fast::start_plain_after(request_end_tick, delay_us),
            None => {
                if dxl_fast::arm_tx() {
                    dxl_fast::fire_now();
                }
            }
        }
    }

    fn send_with_snoop_crc(&mut self, delay_us: u32, snoop_from: Option<u32>) {
        let Some(request_end_tick) = self.request_end_tick.take() else {
            return;
        };
        dxl_fast::start_fast_after(request_end_tick, delay_us, snoop_from);
    }

    fn set_baud(&mut self, rate: BaudRate) {
        let brr = usart::brr(PCLK_HZ, rate.as_hz());
        DXL_BAUD_PENDING_BRR.store(brr, Ordering::Release);
    }

    fn trigger_clock_cal(&mut self) {
        // RX byte-stamp snapshot. COUNT load is Acquire; pairs with the IDLE
        // ISR's Release store and orders the Relaxed FIRST/LAST reads.
        let count = DXL_RX_BYTE_COUNT.load(Ordering::Acquire);
        // 4 is the floor where (count - 1) is large enough for the divide to
        // yield anything meaningful; in practice the bench cal packet is the
        // 10-byte CAL frame itself, so we always have ≥ 9 intervals.
        if count < 4 {
            return;
        }
        let first = DXL_RX_STAMP_FIRST.load(Ordering::Relaxed);
        let last = DXL_RX_STAMP_LAST.load(Ordering::Relaxed);
        let nominal = DXL_BYTE_TIME_TICKS.load(Ordering::Relaxed);
        if nominal == 0 {
            return;
        }

        // FIRST = end-of-byte-1 grabbed inside RXNE ISR; LAST = IDLE backdate
        // = end-of-byte-N. Span covers (count - 1) byte times.
        //
        // FIRST carries the RXNE-ISR entry latency as a constant offset, so
        // observed reads SHORTER than reality by `isr_latency / (count - 1)`.
        // At 3 Mbaud with a 10-byte cal packet that's ~17000 ppm of error —
        // larger than the trim step. Bench-side mitigation: run CAL at low
        // baud (1 Mbaud or below) so the offset is well under one step.
        // Compensation lives in a follow-up; for now we just compute and
        // apply.
        //
        // Math, derived to one soft divide. Direct form:
        //   observed = span / intervals
        //   step     = (observed - nominal) * scale / nominal     (rounded)
        // Collapsed:
        //   step     = (span - intervals * nominal) * scale
        //              ---------------------------------------    (rounded)
        //                       intervals * nominal
        // V006 has no hardware divide; one soft div in main-loop context
        // (CAL contract: torque off, kernel hot path not live) is fine.
        // A8.2's per-shot runtime drift correction is where Q-format
        // reciprocals will land.
        let span = last.wrapping_sub(first);
        let intervals = (count - 1) as u32;
        let scale = (1_000_000 / rcc::CLOCK_TRIM_PPM_PER_STEP) as i32;
        let denom = (intervals as i32).saturating_mul(nominal as i32);
        if denom <= 0 {
            return;
        }
        let numerator = (span as i32).saturating_sub(denom).saturating_mul(scale);
        let round = if numerator >= 0 {
            denom / 2
        } else {
            -(denom / 2)
        };
        let step = numerator.saturating_add(round) / denom;
        if step == 0 {
            return;
        }

        let current = SHARED.table.config.with(|c| c.comms.clock_trim) as i32;
        let new_delta = (current + step).clamp(
            rcc::CLOCK_TRIM_DELTA_MIN as i32,
            rcc::CLOCK_TRIM_DELTA_MAX as i32,
        ) as i8;
        if new_delta as i32 == current {
            return;
        }
        rcc::apply_clock_trim_delta(new_delta);
        SHARED
            .table
            .config
            .with_mut(|c| c.comms.clock_trim = new_delta);
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
