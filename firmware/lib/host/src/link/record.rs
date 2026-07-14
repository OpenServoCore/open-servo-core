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
/// `0x60..=0x6F` the instrument family (client->adapter `0x60..=0x67`,
/// adapter->client `0x68..=0x6F`) -- rails, raw wire verbs, edge-capture
/// drains. Unconditional in every build; deliberate protocol-breaking
/// lives only here, never in SUBMIT (whose Shape validation stands).
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
/// Instrument raw TX: one law break + the body's raw bytes, engine command
/// pipeline bypassed (no Shape validation -- malformed frames are the
/// point). Body: `seq(2 LE) bytes(1..)`. Answered by [`REC_WIRE_DONE`] at
/// wire release, or REJECTED (busy while a SUBMIT command or another wire
/// op is outstanding, and vice versa -- one wire owner at a time).
pub const REC_WIRE_SEND: u8 = 0x61;
/// Instrument burst: length-prefixed frames fired back-to-back, each
/// break-framed, chained on TC (gap = one TC service, under a character
/// time at every operational baud). Body: `seq(2 LE) [len(1) bytes]+`.
pub const REC_WIRE_BURST: u8 = 0x62;
/// Instrument dominant-low pulse of arbitrary width (the engine's rescue
/// verb owns the protocol-shaped pulse; this is the sweepable one).
/// Body: `seq(2 LE) us(2 LE)`.
pub const REC_WIRE_PULSE: u8 = 0x63;
/// Drain captured wire edges (both polarities, capture order). Body:
/// `max(1)` = per-ring entry cap, clamped to [`DRAIN_MAX`]. Answered by
/// [`REC_EDGES`]. Consumer-paced: drain at least every few ms of wire
/// activity or the overflow flag goes up (and stays, sticky).
pub const REC_EDGE_DRAIN: u8 = 0x64;
/// Drop undrained captures + the overflow flag. Answered by
/// [`REC_CAPTURE_ACK`].
pub const REC_CAPTURE_RESET: u8 = 0x65;
pub const REC_INFO: u8 = 0x80;
pub const REC_STATUS: u8 = 0x81;
pub const REC_TERMINAL: u8 = 0x82;
pub const REC_REJECTED: u8 = 0x83;
/// Adapter's answer to a record type it does not serve.
pub const REC_UNKNOWN: u8 = 0x84;
pub const REC_BOOTLOADER_ACK: u8 = 0x48;
pub const REC_RAILS_ACK: u8 = 0x68;
/// A wire op (send/burst/pulse) finished. Body: `seq(2 LE) tick(4 LE)` --
/// the wire-release engine tick.
pub const REC_WIRE_DONE: u8 = 0x69;
/// Edge drain reply. Body: `flags(1: bit0 overflow) now(4 LE)
/// fall_n(1) rise_n(1) falls(fall_n x u16 LE) rises(rise_n x u16 LE)`.
/// Edge ticks are the engine tick domain's low 16 bits; `now` is the
/// 32-bit drain moment, the client's unwrap reference (unwrap ambiguity
/// resets across gaps longer than one u16 wrap -- irrelevant within an
/// exchange, which is all the instrument measures).
pub const REC_EDGES: u8 = 0x6C;
pub const REC_CAPTURE_ACK: u8 = 0x6D;

/// Per-ring entry cap on one edge drain (keeps the reply inside the
/// outbound scratch; the BBATCH-style polled-drain precedent).
pub const DRAIN_MAX: usize = 64;

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

pub fn wire_done(dst: &mut [u8], seq: u16, tick: u32) -> &[u8] {
    dst[2] = REC_WIRE_DONE;
    dst[3..5].copy_from_slice(&seq.to_le_bytes());
    dst[5..9].copy_from_slice(&tick.to_le_bytes());
    sealed(dst, 7)
}

pub fn edges<'a>(
    dst: &'a mut [u8],
    overflow: bool,
    now: u32,
    falls: &[u16],
    rises: &[u16],
) -> &'a [u8] {
    dst[2] = REC_EDGES;
    dst[3] = overflow as u8;
    dst[4..8].copy_from_slice(&now.to_le_bytes());
    dst[8] = falls.len() as u8;
    dst[9] = rises.len() as u8;
    let mut at = 10;
    for &t in falls.iter().chain(rises) {
        dst[at..at + 2].copy_from_slice(&t.to_le_bytes());
        at += 2;
    }
    sealed(dst, at - 2)
}

pub fn capture_ack(dst: &mut [u8]) -> &[u8] {
    dst[2] = REC_CAPTURE_ACK;
    sealed(dst, 1)
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
