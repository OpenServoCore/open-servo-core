//! The reply surface: `ReplyHandle` over disjoint bus fields and its builders.

use osc_protocol::wire::ResultCode;
use osc_servo_core::traits::{Reply, SendError, Status};
use osc_servo_core::{BaudRate, BootMode};

use super::super::tx::TxEngine;
use super::ServoBus;
use crate::traits::bus::{CrcEngine, Providers, TxWire};

/// Reply surface over disjoint `ServoBus` fields (driver-pattern sec 4.3): the
/// decoded request still borrows the ring while the dispatcher stages the
/// reply into the TX engine and the deferred-config fields.
pub(super) struct ReplyHandle<'a, W: TxWire, C: CrcEngine> {
    pub(super) tx: &'a mut TxEngine<W>,
    pub(super) crc: &'a mut C,
    /// Write-through to the bus id: `set_id` applies here so a status staged
    /// after it already carries the new id (the ASSIGN ack, sec 9.2).
    pub(super) id: &'a mut u8,
    pub(super) pending_id: &'a mut Option<u8>,
    pub(super) pending_baud: &'a mut Option<BaudRate>,
    pub(super) pending_reboot: &'a mut Option<BootMode>,
    pub(super) pending_cal: &'a mut Option<(u16, u8)>,
    pub(super) response_deadline_us: &'a mut u16,
    pub(super) staged: bool,
    /// sec 9.2: the request is a broadcast ENUM -- a reply staged through this
    /// handle is marked collision-tolerant, keyed off its own payload (the
    /// responder's UID) for the slot draw.
    pub(super) tolerant: bool,
}

/// sec 9.2 slot key: the reply payload IS the responder's UID for ENUM -- fold
/// its osc-CRC to a byte. The CRC mixing keeps same-reel sequential serials
/// apart; sequencing XORs in the live tick, so equal keys still re-roll.
fn slot_key(payload: &[u8]) -> u8 {
    let crc = osc_protocol::crc::osc_crc(payload);
    (crc ^ (crc >> 8)) as u8
}

impl<P: Providers> ServoBus<P> {
    /// A reply surface over the deferred-config fields, with no request decode --
    /// used at verdict commit, where hooks stage baud/id but no frame is
    /// re-parsed (the ring isn't borrowed here).
    pub(super) fn reply_handle(&mut self) -> ReplyHandle<'_, P::Tx, P::Crc> {
        ReplyHandle {
            tx: &mut self.tx,
            crc: &mut self.crc,
            id: &mut self.id,
            pending_id: &mut self.pending_id,
            pending_baud: &mut self.pending_baud,
            pending_reboot: &mut self.pending_reboot,
            pending_cal: &mut self.clock.pending_cal,
            response_deadline_us: &mut self.response_deadline_us,
            staged: false,
            tolerant: false,
        }
    }
}

impl<W: TxWire, C: CrcEngine> Reply for ReplyHandle<'_, W, C> {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError> {
        let r = self
            .tx
            .stage(self.crc, *self.id, status.result, status.alert, status.data);
        if r.is_ok() {
            self.staged = true;
            if self.tolerant {
                self.tx.mark_collision_tolerant(slot_key(status.data));
            }
        }
        r
    }

    fn send_status_gather(
        &mut self,
        result: ResultCode,
        alert: bool,
        spans: &[&[u8]],
    ) -> Result<(), SendError> {
        let r = self
            .tx
            .stage_gather(self.crc, *self.id, result, alert, spans);
        if r.is_ok() {
            self.staged = true;
            if self.tolerant {
                let crc = spans
                    .iter()
                    .fold(0, |c, s| osc_protocol::crc::osc_crc_continue(c, s));
                self.tx.mark_collision_tolerant((crc ^ (crc >> 8)) as u8);
            }
        }
        r
    }

    fn stage_id(&mut self, id: u8) {
        *self.pending_id = Some(id);
    }

    fn set_id(&mut self, id: u8) {
        *self.id = id;
    }

    fn stage_baud(&mut self, baud: BaudRate) {
        *self.pending_baud = Some(baud);
    }

    fn set_response_deadline(&mut self, us: u16) {
        *self.response_deadline_us = us;
    }

    fn stage_reboot(&mut self, mode: BootMode) {
        *self.pending_reboot = Some(mode);
    }

    fn begin_clock_cal(&mut self, gap_us: u16, gaps: u8) {
        *self.pending_cal = Some((gap_us, gaps));
    }
}
