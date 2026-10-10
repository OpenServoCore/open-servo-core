//! TEL burst sender: puts the kernel-encoded frame buffers on the wire as
//! `Stream` status frames (protocol sec 5.3). A committed nonzero
//! `tel_count` write arms it; any host break kills it (`on_break`:
//! reclaiming the line IS the abort lever). ALERT carries the batch's
//! fault-OR (the fault contract), already folded by the encoder.
//!
//! A banked buffer gets its header and a CRC engine run while the frame
//! ahead of it still streams; sealed, it leaves as one arm straight from the
//! buffer, chained from the previous frame's TC. Between two frames the wire
//! waits for one TC body and a trigger only, which a kernel above the bus
//! delays by at most one body (DES pin `tel_six_fields_keep_up_while_stepping`).

use osc_protocol::frame::Header;
use osc_protocol::wire::{self, Inst, ResultCode};
use osc_servo_core::tel::STREAM_PAYLOAD_MAX;

use super::ServoBus;
use crate::tel::{BufMeta, CRC_LEN, TelDrain, next_buf};
use crate::traits::bus::{CrcEngine, Providers};

const _: () = assert!(STREAM_PAYLOAD_MAX <= wire::MAX_PAYLOAD as usize);

/// The oldest unsent buffer on its way to the wire.
#[derive(Copy, Clone)]
enum Next {
    /// Not banked, or banked and waiting for the CRC engine.
    Open,
    /// Header written, covered span in the CRC engine.
    Crc(BufMeta),
    /// CRC written: the frame is whole.
    Sealed(BufMeta),
}

pub(super) struct TelBurst {
    drain: Option<TelDrain>,
    /// Arm epoch this burst sends under (`send_arm`'s tag).
    epoch: u8,
    /// The oldest unsent buffer; the kernel fills 0 first after every arm.
    next: usize,
    state: Next,
    active: bool,
    /// The buffer on the wire, and whether it is the LAST frame.
    streaming: Option<(usize, bool)>,
}

impl TelBurst {
    pub(super) const fn new() -> Self {
        Self {
            drain: None,
            epoch: 0,
            next: 0,
            state: Next::Open,
            active: false,
            streaming: None,
        }
    }

    pub(super) fn attach(&mut self, drain: TelDrain) {
        self.drain = Some(drain);
    }

    pub(super) fn active(&self) -> bool {
        self.active
    }

    /// `Reply::tel_arm`: count 0 disarms; a fresh arm simply overwrites a
    /// live burst (stale buffers die by epoch, the encoder restarts).
    pub(super) fn arm(&mut self, mask: u16, count: u16) {
        let Some(d) = self.drain.as_mut() else {
            return;
        };
        self.epoch = d.send_arm(mask, count);
        if count == 0 {
            self.deactivate();
            return;
        }
        d.clear_ready();
        d.set_active(true);
        self.next = 0;
        self.state = Next::Open;
        self.streaming = None;
        self.active = true;
    }

    pub(super) fn abort(&mut self) {
        self.deactivate();
    }

    /// TX release notice: frees the streamed buffer; after the LAST frame
    /// the line simply goes quiet.
    pub(super) fn on_tx_released(&mut self) {
        let Some((idx, last)) = self.streaming.take() else {
            return;
        };
        if let Some(d) = self.drain.as_mut() {
            d.release(idx);
        }
        if last {
            self.deactivate();
        }
    }

    fn deactivate(&mut self) {
        self.active = false;
        self.state = Next::Open;
        self.streaming = None;
        if let Some(d) = self.drain.as_mut() {
            d.set_active(false);
            d.clear_ready();
        }
    }
}

impl<P: Providers> ServoBus<P> {
    /// TEL poll, at the bus level: from the SW vector every kernel tick
    /// pends during a burst, and from the TC that frees the wire. Seals the
    /// oldest unsent buffer once the CRC engine is done, sends it when the
    /// wire is ours (TX idle, no frame mid-verdict), and starts the CRC of
    /// the next banked one. The arm's break-silence contract makes the
    /// immediate trigger safe: any host traffic would have killed the burst
    /// in `on_break` before this poll ran.
    pub fn poll_tel(&mut self) {
        if !self.burst.active || self.pending.is_some() {
            return;
        }
        let Some(d) = self.burst.drain.as_mut() else {
            return;
        };
        let idx = self.burst.next;
        if let Next::Crc(meta) = self.burst.state {
            let Some(crc) = self.crc.result() else {
                return;
            };
            let at = Header::SIZE + meta.len as usize;
            let Some(tail) = d
                .frame(idx)
                .and_then(|f| f.get_mut(at..))
                .and_then(|t| t.first_chunk_mut::<CRC_LEN>())
            else {
                return;
            };
            *tail = crc.to_le_bytes();
            self.burst.state = Next::Sealed(meta);
        }
        if let Next::Sealed(meta) = self.burst.state
            && !self.tx.busy()
        {
            let end = Header::SIZE + meta.len as usize + CRC_LEN;
            let Some(span) = d.frame(idx).and_then(|f| f.get(1..end)) else {
                return;
            };
            if self.tx.send_sealed(span).is_ok() {
                self.burst.streaming = Some((idx, meta.last));
                self.burst.next = next_buf(idx);
                self.burst.state = Next::Open;
            }
        }
        // The engine is the bus's: free unless a non-TEL reply streams.
        if matches!(self.burst.state, Next::Open)
            && (!self.tx.busy() || self.burst.streaming.is_some())
        {
            let idx = self.burst.next;
            let Some(meta) = d.ready(idx, self.burst.epoch) else {
                return;
            };
            let cov = Header::SIZE + meta.len as usize;
            let Some(covered) = d.frame(idx).and_then(|f| f.get_mut(..cov)) else {
                return;
            };
            if let Some(head) = covered.first_chunk_mut::<{ Header::SIZE }>() {
                *head = [
                    wire::ALIGN_BYTE,
                    self.id,
                    wire::len_for(meta.len as u8),
                    Inst::status(ResultCode::Stream, meta.alert).0,
                ];
            }
            self.crc.reset();
            self.crc.feed(covered);
            self.burst.state = Next::Crc(meta);
        }
    }
}
