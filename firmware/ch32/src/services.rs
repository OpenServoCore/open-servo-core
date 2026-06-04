use core::sync::atomic::Ordering;

use dxl_protocol::prelude::*;
use osc_core::{BootMode, DxlBus, Event, Schedule, ServiceEvents, ServicesIo};

use crate::dxl;
use crate::dxl::Ch32DxlCrc;
use crate::dxl::statics::{
    DXL_BAUD_PENDING_BRR, DXL_CLOCK_FINE_TRIM_PENDING, DXL_CLOCK_TRIM_PENDING, DXL_REBOOT_PENDING,
    DXL_RX_BUF, DXL_RX_BUF_LEN, DXL_TX_BUF, RX_MASK_U32,
};
use crate::dxl::timing::{SLOT_MARGIN, bytes_to_us, bytes_to_us_q88};
use crate::hal::clocks::PCLK_HZ;
use crate::hal::usart;
use crate::hal::{dma, flash, pfic};
use crate::idle_anchor::{self, IdleAnchor};

type Wire = Codec<Ch32DxlCrc>;

/// Single &mut writer: the main loop holding the `Services` struct.
pub struct Ch32Bus {
    /// Latest IDLE anchor consumed by `poll`. `tick` feeds the following
    /// `send` call; `bytes` carries the cumulative wire-end cursor.
    anchor: IdleAnchor,
    /// Wire-byte length of the window `poll` last returned. `send` compares
    /// this against current DMA position to detect a ring overrun while
    /// dispatch was in flight (the parsed slices live in the ring; if DMA
    /// wrapped past the parsed range before the reply was scheduled, the
    /// slices contain garbage and the reply must be aborted).
    parsed_length: usize,
}

impl Ch32Bus {
    pub const fn new() -> Self {
        Self {
            anchor: IdleAnchor::empty(),
            parsed_length: 0,
        }
    }

    /// Compute the (head, tail) slices for the latest IDLE-anchored RX
    /// window directly over the ring. Returns `None` when no fresh anchor
    /// or the burst is too big to be a valid DXL frame.
    fn extract_window(&mut self) -> Option<(&'static [u8], &'static [u8])> {
        let fresh = idle_anchor::snapshot();
        if fresh.seq == self.anchor.seq {
            return None;
        }
        let prev_bytes = self.anchor.bytes;
        self.anchor = fresh;

        // SAFETY: read-only access to a static ring; DMA writes circularly,
        // but we only read indices below the IDLE-published wire-end.
        let ring: &'static [u8] =
            unsafe { core::slice::from_raw_parts((*DXL_RX_BUF.get()).as_ptr(), DXL_RX_BUF_LEN) };
        let cap = ring.len();

        // Clamp to ring capacity: length > cap means earlier bursts were
        // overwritten before we polled — present the most recent `cap`
        // bytes and let the parser resync.
        let length = (fresh.bytes.wrapping_sub(prev_bytes) as usize).min(cap);
        if length == 0 {
            return None;
        }
        self.parsed_length = length;

        let end = (fresh.bytes & RX_MASK_U32) as usize;
        let start = (end + cap - length) % cap;
        if start + length <= cap {
            Some((&ring[start..start + length], &[]))
        } else {
            let head_len = cap - start;
            Some((&ring[start..], &ring[..length - head_len]))
        }
    }

    /// True if DMA has advanced enough since the IDLE anchor that it has
    /// wrapped past the start of the parsed range, meaning any borrowed
    /// slices into that range now point at fresh wire bytes (garbage from
    /// the dispatcher's perspective).
    fn parsed_window_overrun(&self) -> bool {
        let cap = DXL_RX_BUF_LEN;
        if self.parsed_length == 0 {
            return false;
        }
        // Bytes DMA has written into the ring since IDLE captured wire-end.
        // CH5 NDTR counts down; (cap - remaining) is the write index modulo cap.
        let remaining = dma::remaining(dma::Channel::CH5) as usize;
        let current_idx = (cap - remaining) & (cap - 1);
        let wire_end_idx = (self.anchor.bytes & RX_MASK_U32) as usize;
        let new_bytes = (current_idx + cap - wire_end_idx) % cap;
        new_bytes >= cap - self.parsed_length
    }
}

impl Default for Ch32Bus {
    fn default() -> Self {
        Self::new()
    }
}

impl DxlBus for Ch32Bus {
    fn poll(&mut self) -> Option<Packet<'static>> {
        let (mut head, mut tail) = self.extract_window()?;
        // Walk forward — dispatch the frame ending exactly at the wire-end;
        // earlier frames are pre-IDLE traffic the master has moved on from,
        // parsed only to advance the cursor and dropped.
        loop {
            let total = head.len() + tail.len();
            if total == 0 {
                return None;
            }
            match Wire::parse_one(head, tail) {
                Ok((packet, used)) => {
                    if used == total {
                        return Some(packet);
                    }
                    (head, tail) = advance(head, tail, used);
                }
                Err(ParseError::Incomplete) => return None,
                Err(ParseError::Resync { skip })
                | Err(ParseError::BadCrc { skip })
                | Err(ParseError::BadInstruction { skip })
                | Err(ParseError::BadLength { skip }) => {
                    let skip = skip.min(total);
                    (head, tail) = advance(head, tail, skip);
                }
            }
        }
    }

    fn send(&mut self, reply: StatusReply<'_>, schedule: Schedule) {
        // Defense-in-depth: if DMA wrapped past the parsed range during
        // dispatch, the request data we just acted on may have been garbage.
        // Abort the reply and surface the fault — master will see the timeout
        // and the link's dma_overrun counter increment.
        if self.parsed_window_overrun() {
            dxl::report_dma_overrun();
            return;
        }

        // SAFETY: &mut self proves sole-writer; USART1 TC ISR only clears
        // after a send cycle this struct initiated.
        let buf = unsafe { &mut *DXL_TX_BUF.get() };
        buf.truncate(0);
        if Wire::write_status_reply(buf, &reply).is_err() {
            buf.truncate(0);
            return;
        }

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

/// Advance a (head, tail) ring window past `by` virtual bytes. Once `head`
/// is fully consumed, the unconsumed tail becomes the new head with no tail.
fn advance<'a>(head: &'a [u8], tail: &'a [u8], by: usize) -> (&'a [u8], &'a [u8]) {
    if by >= head.len() {
        (&tail[by - head.len()..], &[])
    } else {
        (&head[by..], tail)
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
