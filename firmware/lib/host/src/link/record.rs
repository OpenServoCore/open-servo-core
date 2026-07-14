//! Pipe record formats: the adapter <-> client vocabulary over a reliable
//! ordered byte pipe. USB bulk is the first carrier, not the definition --
//! records are length-prefixed (`len(2 LE)` covering `type(1) + body`), so
//! carrier transfer boundaries carry no meaning and the format ports to any
//! ordered pipe.
//!
//! Contract (host-band settled decisions): each SUBMIT names a client
//! `seq`; exactly one REJECTED-or-TERMINAL answers it; STATUS records
//! stream before their terminal, `(seq, slot)`-tagged. NOREPLY commands
//! still terminate ("sent at tick"). Ticks are raw engine ticks; INFO
//! advertises the tick rate once instead of converting per record.

use crate::engine::shape::InvalidReason;
use crate::engine::{Outcome, Terminal};

/// Record type bytes. `0x00..=0x3F` client->adapter, `0x80..=0xBF`
/// adapter->client. Reserved families: `0x40..=0x4F` firmware update
/// (client->adapter `0x40..=0x47`, adapter->client `0x48..=0x4F`) and
/// `0x60..=0x6F` bench (the IF1 personality; deliberate protocol-breaking
/// lives only there).
pub const REC_HELLO: u8 = 0x00;
pub const REC_SUBMIT: u8 = 0x01;
/// Hand the adapter to the resident bootloader (the mandatory
/// revert-to-stock path, wlink-iap flow). No seq, never refused; quiescing
/// the bus first is the client's job. The server acks, then the chip drains
/// the queued [`AdapterRequest`](crate::link::AdapterRequest), flushes the
/// pipe, disarms the loader, and resets -- the ack is the last record the
/// port delivers.
pub const REC_ENTER_BOOTLOADER: u8 = 0x40;
/// Bench family: drive the adapter's DUT power rails (the adapter feeds
/// the bus side's 3V3; 5V is a bench convenience). Body: one state byte,
/// bit0 = 3V3 on, bit1 = 5V on -- absolute state, so queue last-wins is
/// sound. Answered by [`REC_RAILS_ACK`] echoing the applied state.
pub const REC_SET_RAILS: u8 = 0x60;
pub const REC_INFO: u8 = 0x80;
pub const REC_STATUS: u8 = 0x81;
pub const REC_TERMINAL: u8 = 0x82;
pub const REC_REJECTED: u8 = 0x83;
/// Adapter's answer to a record type it does not serve.
pub const REC_UNKNOWN: u8 = 0x84;
pub const REC_BOOTLOADER_ACK: u8 = 0x48;
pub const REC_RAILS_ACK: u8 = 0x68;

/// SUBMIT verb byte (body: `seq(2 LE) verb(1) args`).
pub const VERB_EXCHANGE: u8 = 0x00; // args: id(1) inst(1) payload(rest)
pub const VERB_RESCUE: u8 = 0x01; // no args
pub const VERB_HOST_BAUD: u8 = 0x02; // args: rate idx(1)
pub const VERB_SET_RESPONSE_DEADLINE: u8 = 0x03; // args: us(2 LE)

/// REJECTED reason byte.
pub const REASON_BAD_ID: u8 = 0x00;
pub const REASON_BAD_INST: u8 = 0x01;
pub const REASON_BAD_PAYLOAD: u8 = 0x02;
pub const REASON_TOO_LONG: u8 = 0x03;
pub const REASON_BUSY: u8 = 0x04;
/// SUBMIT body itself did not parse (short, bad verb, bad args).
pub const REASON_MALFORMED: u8 = 0x05;

/// TERMINAL outcome byte.
pub const OUTCOME_SENT: u8 = 0x00;
pub const OUTCOME_COMPLETE: u8 = 0x01;
pub const OUTCOME_TIMEOUT: u8 = 0x02;

/// TERMINAL flags byte.
pub const FLAG_GARBLE_AFTER_LAST_FRAME: u8 = 1 << 0;

pub const LINK_VERSION: u8 = 1;

/// The `seq` a record carries when no command owns it (a status or
/// terminal surfacing outside any active seq -- a server invariant breach
/// made visible rather than silent).
pub const SEQ_NONE: u16 = 0xFFFF;

pub fn reason_code(r: InvalidReason) -> u8 {
    match r {
        InvalidReason::BadId => REASON_BAD_ID,
        InvalidReason::BadInst => REASON_BAD_INST,
        InvalidReason::BadPayload => REASON_BAD_PAYLOAD,
        InvalidReason::TooLong => REASON_TOO_LONG,
    }
}

/// Prefix `dst` with the record length and return the full record span:
/// `body_len` counts type + body bytes already written at `dst[2..]`.
fn sealed(dst: &mut [u8], body_len: usize) -> &[u8] {
    dst[..2].copy_from_slice(&(body_len as u16).to_le_bytes());
    &dst[..2 + body_len]
}

pub fn info(dst: &mut [u8], ticks_per_us: u32) -> &[u8] {
    dst[2] = REC_INFO;
    dst[3] = LINK_VERSION;
    dst[4..8].copy_from_slice(&ticks_per_us.to_le_bytes());
    sealed(dst, 6)
}

pub fn rejected(dst: &mut [u8], seq: u16, reason: u8) -> &[u8] {
    dst[2] = REC_REJECTED;
    dst[3..5].copy_from_slice(&seq.to_le_bytes());
    dst[5] = reason;
    sealed(dst, 4)
}

pub fn unknown(dst: &mut [u8], rtype: u8) -> &[u8] {
    dst[2] = REC_UNKNOWN;
    dst[3] = rtype;
    sealed(dst, 2)
}

pub fn bootloader_ack(dst: &mut [u8]) -> &[u8] {
    dst[2] = REC_BOOTLOADER_ACK;
    sealed(dst, 1)
}

pub fn rails_ack(dst: &mut [u8], state: u8) -> &[u8] {
    dst[2] = REC_RAILS_ACK;
    dst[3] = state;
    sealed(dst, 2)
}

/// STATUS: `seq(2) slot(1) id(1) inst(1) payload`. The payload arrives as
/// the ring view's two segments.
pub fn status<'a>(
    dst: &'a mut [u8],
    seq: u16,
    slot: u8,
    id: u8,
    inst: u8,
    payload: (&[u8], &[u8]),
) -> &'a [u8] {
    dst[2] = REC_STATUS;
    dst[3..5].copy_from_slice(&seq.to_le_bytes());
    dst[5] = slot;
    dst[6] = id;
    dst[7] = inst;
    let n = payload.0.len();
    dst[8..8 + n].copy_from_slice(payload.0);
    dst[8 + n..8 + n + payload.1.len()].copy_from_slice(payload.1);
    sealed(dst, 6 + n + payload.1.len())
}

/// TERMINAL: `seq(2) outcome(1) slot(1) tick(4 LE) statuses(1) garble(2 LE)
/// flags(1)`.
pub fn terminal<'a>(dst: &'a mut [u8], seq: u16, t: &Terminal) -> &'a [u8] {
    let (outcome, slot) = match t.outcome {
        Outcome::Sent => (OUTCOME_SENT, 0),
        Outcome::Complete => (OUTCOME_COMPLETE, 0),
        Outcome::Timeout { slot } => (OUTCOME_TIMEOUT, slot),
    };
    dst[2] = REC_TERMINAL;
    dst[3..5].copy_from_slice(&seq.to_le_bytes());
    dst[5] = outcome;
    dst[6] = slot;
    dst[7..11].copy_from_slice(&t.tick.to_le_bytes());
    dst[11] = t.evidence.statuses;
    dst[12..14].copy_from_slice(&t.evidence.garble.to_le_bytes());
    dst[14] = if t.evidence.garble_after_last_frame {
        FLAG_GARBLE_AFTER_LAST_FRAME
    } else {
        0
    };
    sealed(dst, 13)
}
