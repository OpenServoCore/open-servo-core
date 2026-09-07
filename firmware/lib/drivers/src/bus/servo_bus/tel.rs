//! TEL burst stager: puts the kernel-encoded ping-pong buffers on the wire
//! as `Stream` status frames (protocol sec 5.3). A committed nonzero
//! `tel_count` write arms it; the main-loop poll stages one ready buffer per
//! frame whenever the wire is ours; any host break kills it (`on_break` --
//! reclaiming the line IS the abort lever). ALERT carries the batch's
//! fault-OR (the fault contract), already folded by the encoder.

use osc_protocol::wire::{self, ResultCode};
use osc_servo_core::tel::STREAM_PAYLOAD_MAX;

use super::ServoBus;
use crate::tel::{BufMeta, TelDrain};
use crate::traits::bus::Providers;

const _: () = assert!(STREAM_PAYLOAD_MAX <= wire::MAX_PAYLOAD as usize);

pub(super) struct TelBurst {
    drain: Option<TelDrain>,
    /// Arm epoch this burst stages under (`send_arm`'s tag).
    epoch: u8,
    /// Next buffer to stage; the kernel fills 0 first after every arm.
    next: usize,
    active: bool,
    /// A burst frame is streaming; its TX release frees the buffer.
    in_flight: bool,
    /// The streaming frame is the LAST; its release ends the burst.
    last_in_flight: bool,
}

impl TelBurst {
    pub(super) const fn new() -> Self {
        Self {
            drain: None,
            epoch: 0,
            next: 0,
            active: false,
            in_flight: false,
            last_in_flight: false,
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
        self.active = true;
        self.in_flight = false;
        self.last_in_flight = false;
    }

    pub(super) fn abort(&mut self) {
        self.deactivate();
    }

    /// TX release notice: frees the streamed buffer; after the LAST frame
    /// the line simply goes quiet.
    pub(super) fn on_tx_released(&mut self) {
        if !self.in_flight {
            return;
        }
        self.in_flight = false;
        if let Some(d) = self.drain.as_mut() {
            d.release(self.next);
        }
        self.next ^= 1;
        if self.last_in_flight {
            self.deactivate();
        }
    }

    fn deactivate(&mut self) {
        self.active = false;
        self.in_flight = false;
        self.last_in_flight = false;
        if let Some(d) = self.drain.as_mut() {
            d.set_active(false);
            d.clear_ready();
        }
    }

    fn next_ready(&mut self) -> Option<(usize, BufMeta)> {
        if !self.active || self.in_flight {
            return None;
        }
        let (next, epoch) = (self.next, self.epoch);
        let meta = self.drain.as_mut()?.ready(next, epoch)?;
        Some((next, meta))
    }
}

impl<P: Providers> ServoBus<P> {
    /// Main-loop TEL poll, ISRs masked by the caller (the `take_reboot`
    /// idiom): when a burst is live, a batch is banked, and the wire is
    /// ours -- TX idle, no frame mid-verdict -- stage the next `Stream`
    /// status frame and put it on the wire. The arm's break-silence contract
    /// makes the immediate trigger safe: any host traffic would have killed
    /// the burst in `on_break` before this poll ran.
    pub fn poll_tel(&mut self) {
        if self.tx.busy() || self.pending.is_some() {
            return;
        }
        let Some((idx, meta)) = self.burst.next_ready() else {
            return;
        };
        let Some(d) = self.burst.drain.as_ref() else {
            return;
        };
        let len = (meta.len as usize).min(STREAM_PAYLOAD_MAX);
        let payload = &d.payload(idx)[..len];
        if self
            .tx
            .stage(
                &mut self.crc,
                self.id,
                ResultCode::Stream,
                meta.alert,
                payload,
            )
            .is_ok()
        {
            self.tx.trigger(&mut self.crc, None);
            self.burst.in_flight = true;
            self.burst.last_in_flight = meta.last;
        }
    }
}
