use core::sync::atomic::Ordering;

use dxl_protocol::prelude::*;
use osc_core::{BootMode, DxlBus, Event, Schedule, ServiceEvents, ServicesIo};

use crate::dxl;
use crate::dxl::Ch32DxlCrc;
use crate::dxl::statics::{
    DXL_BAUD_PENDING_BRR, DXL_CLOCK_FINE_TRIM_PENDING, DXL_CLOCK_TRIM_PENDING, DXL_REBOOT_PENDING,
    DXL_RX_BUF, DXL_RX_SCRATCH, DXL_TX_BUF, RX_MASK_U32,
};
use crate::dxl::timing::{SLOT_MARGIN, bytes_to_us, bytes_to_us_q88};
use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;
use crate::hal::{flash, pfic};
use crate::idle_anchor::{self, IdleAnchor};

type Wire = Codec<Ch32DxlCrc>;

/// Single &mut writer: the main loop holding the `Services` struct.
pub struct Ch32Bus {
    /// Latest IDLE anchor consumed by `poll`. `tick` feeds the following
    /// `send` call; `bytes` carries the cumulative wire-end cursor used
    /// for both the next slice-range diff and the snoop-from origin.
    anchor: IdleAnchor,
}

impl Ch32Bus {
    pub const fn new() -> Self {
        Self {
            anchor: IdleAnchor::empty(),
        }
    }

    /// Stitch the latest IDLE-anchored RX window into [`DXL_RX_SCRATCH`] and
    /// return a 'static slice over it. Returns `None` when no fresh anchor or
    /// the burst is too big to be a valid DXL frame.
    fn stitch_window(&mut self) -> Option<&'static [u8]> {
        let fresh = idle_anchor::snapshot();
        if fresh.seq == self.anchor.seq {
            return None;
        }
        let prev_bytes = self.anchor.bytes;
        self.anchor = fresh;

        // SAFETY: DMA writes DXL_RX_BUF circularly; we only read indices
        // below the IDLE-published wire-end position.
        let ring = unsafe { &*DXL_RX_BUF.get() };
        let cap = ring.len();

        // Clamp to ring capacity: a length > cap means earlier bursts were
        // overwritten before we polled — present the most recent `cap`
        // bytes and let the parser resync.
        let length = (fresh.bytes.wrapping_sub(prev_bytes) as usize).min(cap);
        if length == 0 {
            return None;
        }

        // SAFETY: sole writer; the services layer drops the previous slice
        // before re-polling, so overwriting the scratch here is safe.
        let scratch = unsafe { &mut *DXL_RX_SCRATCH.get() };
        if length > scratch.capacity() {
            return None;
        }
        scratch.clear();

        let end = (fresh.bytes & RX_MASK_U32) as usize;
        let start = (end + cap - length) % cap;
        if start + length <= cap {
            scratch
                .extend_from_slice(&ring[start..start + length])
                .ok()?;
        } else {
            let head_len = cap - start;
            scratch.extend_from_slice(&ring[start..]).ok()?;
            scratch.extend_from_slice(&ring[..length - head_len]).ok()?;
        }

        // SAFETY: DXL_RX_SCRATCH is 'static; the slice stays valid until the
        // next poll overwrites it.
        Some(unsafe { core::slice::from_raw_parts(scratch.as_ptr(), scratch.len()) })
    }
}

impl Default for Ch32Bus {
    fn default() -> Self {
        Self::new()
    }
}

impl DxlBus for Ch32Bus {
    fn poll(&mut self) -> Option<Packet<'static>> {
        let window = self.stitch_window()?;
        // Walk forward — dispatch the frame ending exactly at the wire-end;
        // earlier frames are pre-IDLE traffic the master has moved on from,
        // parsed only to advance the cursor and dropped.
        let n = window.len();
        let mut offset = 0;
        while offset < n {
            match Wire::parse_one(&window[offset..]) {
                Ok((packet, used)) => {
                    if offset + used == n {
                        return Some(packet);
                    }
                    offset += used;
                }
                Err(ParseError::Incomplete) => return None,
                Err(ParseError::Resync { skip })
                | Err(ParseError::BadCrc { skip })
                | Err(ParseError::BadInstruction { skip })
                | Err(ParseError::BadLength { skip }) => {
                    offset = (offset + skip).min(n);
                }
            }
        }
        None
    }

    fn send(&mut self, reply: StatusReply<'_>, schedule: Schedule) {
        // SAFETY: &mut self proves sole-writer; USART1 TC ISR only clears
        // after a send cycle this struct initiated.
        let buf = unsafe { &mut *DXL_TX_BUF.get() };
        buf.truncate(0);
        if Wire::write_status_reply(buf, &reply).is_err() {
            buf.truncate(0);
            return;
        }

        // Pick fire mechanism from the reply variant. Fast Last patches the
        // chain CRC at fire-time via snoop; Fast Only computes the CRC
        // locally and fires as a plain Status frame; everything else fires
        // plain at `wire_end + rdt + bytes_to_us(bytes_before + margin)`.
        match reply {
            StatusReply::FastSyncRead { position, .. }
            | StatusReply::FastBulkRead { position, .. }
            | StatusReply::FastError { position, .. } => {
                self.fire_fast(position, schedule);
            }
            _ => self.fire_plain(schedule),
        }
    }
}

impl Ch32Bus {
    fn fire_plain(&mut self, schedule: Schedule) {
        // Flush any stale slot setup: an unfired SysTick CMP from a prior
        // Sync/Fast op would otherwise re-fire DMA and patch CRC over this
        // reply's buffer.
        dxl::cancel();
        let bytes = schedule.bytes_before + (schedule.slot_index as u32) * SLOT_MARGIN;
        let delay_us = schedule.rdt_us + bytes_to_us(bytes);
        dxl::start_plain_after(self.anchor.tick, delay_us);
    }

    fn fire_fast(&mut self, position: FastPosition, schedule: Schedule) {
        match position {
            FastPosition::Only { .. } => {
                dxl::cancel();
                dxl::start_plain_after(self.anchor.tick, schedule.rdt_us);
            }
            FastPosition::First { .. } | FastPosition::Middle => {
                dxl::cancel();
                let delay_us = schedule.rdt_us + bytes_to_us(schedule.bytes_before);
                dxl::start_plain_after(self.anchor.tick, delay_us);
            }
            FastPosition::Last => {
                let fire_q88_us = (schedule.rdt_us << 8) + bytes_to_us_q88(schedule.bytes_before);
                dxl::start_fast_after(self.anchor.tick, fire_q88_us, Some(self.anchor.bytes));
            }
        }
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
