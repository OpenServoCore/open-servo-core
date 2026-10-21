//! Link records (link sec 3): the adapter <-> client vocabulary over a
//! reliable ordered byte pipe. Every record, either direction, is
//! `len(2 LE) kind(1) seq(2 LE) body`, `len` counting every byte after
//! itself, so carrier transfer boundaries carry no meaning. Requests use
//! kinds `0x00..=0x7F`, responses `0x80..=0xFF`; every request draws
//! exactly one DONE or one ERROR on its `seq`, a bus exchange streaming
//! FRAMEs first. Both directions' codecs live here: the adapter builds,
//! clients parse, from one definition.

use crate::engine::{Outcome, Terminal};

/// USB carrier, not a record (link sec 8.2): vendor OUT control request on
/// EP0 (bmRequestType `0x41`, interface 0, no data) that opens a session.
/// A carrier signal rather than SET_CONFIGURATION, which WebUSB skips when
/// the configuration is already active, and rather than a record, because
/// the pipe it would ride may still hold the old session's bytes.
pub const USB_REQ_OPEN_SESSION: u8 = 0x01;

// Request kinds (link sec 4 to 9).
pub const GET_INFO: u8 = 0x00;
pub const RAILS: u8 = 0x01;
pub const ADAPTER_IAP: u8 = 0x02;
pub const SET_BUS: u8 = 0x10;
pub const OSC_EXCHANGE: u8 = 0x20;
pub const OSC_RESCUE: u8 = 0x21;
pub const OSC_DEADLINE: u8 = 0x22;
pub const TB_EXCHANGE: u8 = 0x30;
pub const WIRE_SEND: u8 = 0x60;
pub const WIRE_BURST: u8 = 0x61;
pub const WIRE_PULSE: u8 = 0x62;
pub const WIRE_TRAIN: u8 = 0x63;
pub const WIRE_BAUD: u8 = 0x64;
pub const EDGE_DRAIN: u8 = 0x65;
pub const CAPTURE_RESET: u8 = 0x66;

// Response kinds (link sec 3.2).
pub const FRAME: u8 = 0x80;
pub const DONE: u8 = 0x81;
pub const ERROR: u8 = 0x82;

// ERROR codes (link sec 3.2).
pub const ERR_MALFORMED: u8 = 0x00;
pub const ERR_UNKNOWN_KIND: u8 = 0x01;
pub const ERR_UNSUPPORTED: u8 = 0x02;
pub const ERR_WRONG_MODE: u8 = 0x03;
pub const ERR_BUSY: u8 = 0x04;
pub const ERR_BAD_RATE: u8 = 0x05;
pub const ERR_BAD_ID: u8 = 0x06;
pub const ERR_BAD_INST: u8 = 0x07;
pub const ERR_BAD_PAYLOAD: u8 = 0x08;
pub const ERR_TOO_LONG: u8 = 0x09;
pub const ERR_FRAMING: u8 = 0x0A;

/// The `seq` of a response no request owns (link sec 3.3).
pub const SEQ_NONE: u16 = 0xFFFF;
/// The echoed kind of a response no request owns.
pub const KIND_NONE: u8 = 0xFF;

// INFO `features` bits (link sec 4.1).
pub const FEATURE_INSTRUMENT: u16 = 1 << 0;
pub const FEATURE_RAILS: u16 = 1 << 1;
pub const FEATURE_ADAPTER_IAP: u16 = 1 << 2;
pub const FEATURE_TINYBOOT: u16 = 1 << 3;

// Bus modes (link sec 5).
pub const MODE_OSC: u8 = 0;
pub const MODE_TINYBOOT: u8 = 1;

/// OSC_EXCHANGE `flags` bit 0: collect a servo telemetry burst.
pub const EXCHANGE_STREAM: u8 = 1 << 0;

// TB_EXCHANGE `expect` (link sec 7).
pub const EXPECT_NONE: u8 = 0;
pub const EXPECT_ONE: u8 = 1;
pub const EXPECT_COLLECT: u8 = 2;

/// Largest request the adapter accepts, `len` included: comfortably past
/// an OSC_EXCHANGE with a 252-byte payload.
pub const MAX_REQUEST: usize = 512;
/// Largest response the adapter sends, `len` included: a FRAME carrying a
/// full osc status, an edge drain at its cap.
pub const MAX_RESPONSE: usize = 300;
/// `len(2) kind(1) seq(2)`.
pub const HEADER: usize = 5;
/// Largest WIRE_BURST frame train: a whole request less its header.
pub const WIRE_BURST_STREAM_MAX: usize = MAX_REQUEST - HEADER;
/// Per-polarity entry cap on one edge drain (keeps the result inside the
/// largest response).
pub const DRAIN_MAX: usize = 64;

// Exchange result outcomes (link sec 6.2).
pub const OUTCOME_SENT: u8 = 0x00;
pub const OUTCOME_COMPLETE: u8 = 0x01;
pub const OUTCOME_TIMEOUT: u8 = 0x02;

/// Exchange result `flags` bit 0: garble after the last clean frame.
pub const FLAG_GARBLE_AFTER_LAST_FRAME: u8 = 1 << 0;

/// Rails state out of adapter boot: both ON (the pins provider asserts
/// them before anything else), so a bare adapter powers its bus.
pub const RAILS_BOOT_STATE: u8 = 0b11;

/// INFO `reset` byte: the chip's reset-cause flags, read and cleared at
/// adapter boot (so each boot reports only the resets since the last).
pub const RESET_PIN: u8 = 1 << 0;
pub const RESET_POR: u8 = 1 << 1;
pub const RESET_SFT: u8 = 1 << 2;
pub const RESET_IWDG: u8 = 1 << 3;
pub const RESET_WWDG: u8 = 1 << 4;
pub const RESET_LPWR: u8 = 1 << 5;

/// INFO `phase` byte: where the adapter's main loop stood when the last
/// reset hit (kept in reset-surviving RAM). Names an IWDG death's location
/// the way mcause/mepc name an exception's.
pub const PHASE_NONE: u8 = 0;
pub const PHASE_BRINGUP: u8 = 1;
pub const PHASE_USB_POLL: u8 = 2;
pub const PHASE_PIPE: u8 = 3;
pub const PHASE_ADAPTER_REQ: u8 = 4;
pub const PHASE_PUMP: u8 = 5;
pub const PHASE_USB_TX: u8 = 6;
pub const PHASE_HSE_FAIL: u8 = 7;

/// Adapter diagnostics, the INFO tail: `reset(1) phase(1) crash_seq(4)
/// mcause(4) mepc(4) mtval(4) hse_fail(4) resets(4) uptime_ms(4)`. The
/// crash fields describe the last synchronous exception the adapter
/// trapped (they persist across resets until the next exception
/// overwrites them; `crash_seq` counts them, 0 = none on record);
/// `hse_fail` counts crystal-start failures; `resets` counts resets since
/// the last power-on. `uptime_ms` is live, stamped per INFO (wraps at ~49
/// days): shorter than the time since a USB drop means the adapter
/// restarted. A chip-less server (DES sim) reports all zeros.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Diag {
    pub reset: u8,
    pub phase: u8,
    pub crash_seq: u32,
    pub mcause: u32,
    pub mepc: u32,
    pub mtval: u32,
    pub hse_fail: u32,
    pub resets: u32,
    pub uptime_ms: u32,
}

/// Byte length of the INFO tail.
pub const DIAG_LEN: usize = 30;

/// GET_INFO's result (link sec 4.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Info {
    pub features: u16,
    pub ticks_per_us: u32,
    pub max_request: u16,
    pub max_response: u16,
    pub max_tb_payload: u16,
    pub mode: u8,
    pub bps: u32,
    pub deadline_us: u16,
    pub diag: Diag,
}

/// Byte length of the INFO result.
pub const INFO_LEN: usize = 19 + DIAG_LEN;

impl Info {
    pub fn write(&self, dst: &mut [u8]) -> usize {
        let mut w = Writer(dst, 0);
        w.u16(self.features);
        w.u32(self.ticks_per_us);
        w.u16(self.max_request);
        w.u16(self.max_response);
        w.u16(self.max_tb_payload);
        w.u8(self.mode);
        w.u32(self.bps);
        w.u16(self.deadline_us);
        let d = &self.diag;
        w.u8(d.reset);
        w.u8(d.phase);
        for v in [
            d.crash_seq,
            d.mcause,
            d.mepc,
            d.mtval,
            d.hse_fail,
            d.resets,
            d.uptime_ms,
        ] {
            w.u32(v);
        }
        w.1
    }

    /// `None` unless `src` is exactly one INFO result.
    pub fn parse(src: &[u8]) -> Option<Self> {
        if src.len() != INFO_LEN {
            return None;
        }
        let mut r = Reader(src, 0);
        Some(Self {
            features: r.u16(),
            ticks_per_us: r.u32(),
            max_request: r.u16(),
            max_response: r.u16(),
            max_tb_payload: r.u16(),
            mode: r.u8(),
            bps: r.u32(),
            deadline_us: r.u16(),
            diag: Diag {
                reset: r.u8(),
                phase: r.u8(),
                crash_seq: r.u32(),
                mcause: r.u32(),
                mepc: r.u32(),
                mtval: r.u32(),
                hse_fail: r.u32(),
                resets: r.u32(),
                uptime_ms: r.u32(),
            },
        })
    }
}

/// An exchange's result (link sec 6.2): `outcome(1) slot(1) tick(4)
/// frames(2) garble(2) flags(1) laps(2)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExchangeResult {
    pub outcome: Outcome,
    pub tick: u32,
    pub frames: u16,
    pub garble: u16,
    /// Garble after the last clean frame.
    pub trailing: bool,
    pub laps: u16,
}

pub const EXCHANGE_LEN: usize = 13;

impl ExchangeResult {
    pub fn of(t: &Terminal) -> Self {
        Self {
            outcome: t.outcome,
            tick: t.tick,
            frames: t.evidence.statuses,
            garble: t.evidence.garble,
            trailing: t.evidence.garble_after_last_frame,
            laps: t.evidence.laps,
        }
    }

    pub fn write(&self, dst: &mut [u8]) -> usize {
        let (outcome, slot) = match self.outcome {
            Outcome::Sent => (OUTCOME_SENT, 0),
            Outcome::Complete => (OUTCOME_COMPLETE, 0),
            Outcome::Timeout { slot } => (OUTCOME_TIMEOUT, slot),
        };
        let mut w = Writer(dst, 0);
        w.u8(outcome);
        w.u8(slot);
        w.u32(self.tick);
        w.u16(self.frames);
        w.u16(self.garble);
        w.u8(if self.trailing {
            FLAG_GARBLE_AFTER_LAST_FRAME
        } else {
            0
        });
        w.u16(self.laps);
        w.1
    }

    /// `None` unless `src` is exactly one exchange result.
    pub fn parse(src: &[u8]) -> Option<Self> {
        if src.len() != EXCHANGE_LEN {
            return None;
        }
        let mut r = Reader(src, 0);
        let outcome = r.u8();
        let slot = r.u8();
        let outcome = match outcome {
            OUTCOME_SENT => Outcome::Sent,
            OUTCOME_COMPLETE => Outcome::Complete,
            OUTCOME_TIMEOUT => Outcome::Timeout { slot },
            _ => return None,
        };
        Some(Self {
            outcome,
            tick: r.u32(),
            frames: r.u16(),
            garble: r.u16(),
            trailing: r.u8() & FLAG_GARBLE_AFTER_LAST_FRAME != 0,
            laps: r.u16(),
        })
    }
}

/// Seal a response whose body (after the header) is `body_len` bytes.
fn sealed(dst: &mut [u8], kind: u8, seq: u16, body_len: usize) -> &[u8] {
    let len = HEADER - 2 + body_len;
    dst[..2].copy_from_slice(&(len as u16).to_le_bytes());
    dst[2] = kind;
    dst[3..5].copy_from_slice(&seq.to_le_bytes());
    &dst[..2 + len]
}

/// DONE echoing `req`, its result written by `result` into the slice it is
/// handed (returning the length).
pub fn done(dst: &mut [u8], seq: u16, req: u8, result: impl FnOnce(&mut [u8]) -> usize) -> &[u8] {
    dst[HEADER] = req;
    let n = result(&mut dst[HEADER + 1..]);
    sealed(dst, DONE, seq, 1 + n)
}

pub fn error(dst: &mut [u8], seq: u16, req: u8, code: u8) -> &[u8] {
    dst[HEADER] = req;
    dst[HEADER + 1] = code;
    sealed(dst, ERROR, seq, 2)
}

/// FRAME: `slot(1) frame`, the frame arriving as the ring view's two
/// segments.
pub fn frame<'a>(dst: &'a mut [u8], seq: u16, slot: u8, frame: (&[u8], &[u8])) -> &'a [u8] {
    dst[HEADER] = slot;
    let at = HEADER + 1;
    let (a, b) = frame;
    dst[at..at + a.len()].copy_from_slice(a);
    dst[at + a.len()..at + a.len() + b.len()].copy_from_slice(b);
    sealed(dst, FRAME, seq, 1 + a.len() + b.len())
}

/// An osc FRAME: `slot(1) id(1) inst(1) payload`.
pub fn osc_frame<'a>(
    dst: &'a mut [u8],
    seq: u16,
    slot: u8,
    id: u8,
    inst: u8,
    payload: (&[u8], &[u8]),
) -> &'a [u8] {
    dst[HEADER] = slot;
    dst[HEADER + 1] = id;
    dst[HEADER + 2] = inst;
    let at = HEADER + 3;
    let (a, b) = payload;
    dst[at..at + a.len()].copy_from_slice(a);
    dst[at + a.len()..at + a.len() + b.len()].copy_from_slice(b);
    sealed(dst, FRAME, seq, 3 + a.len() + b.len())
}

/// EDGE_DRAIN's result: `flags(1: bit0 overflow) now(4) fall_n(1)
/// rise_n(1) falls(fall_n x 2) rises(rise_n x 2)`. Edge ticks are the
/// adapter tick's low 16 bits, each ring chronological; `now` is the
/// drain's 32-bit tick.
pub fn write_edges(
    dst: &mut [u8],
    overflow: bool,
    now: u32,
    falls: &[u16],
    rises: &[u16],
) -> usize {
    let mut w = Writer(dst, 0);
    w.u8(overflow as u8);
    w.u32(now);
    w.u8(falls.len() as u8);
    w.u8(rises.len() as u8);
    for &t in falls.iter().chain(rises) {
        w.u16(t);
    }
    w.1
}

struct Writer<'a>(&'a mut [u8], usize);

impl Writer<'_> {
    fn bytes(&mut self, b: &[u8]) {
        self.0[self.1..self.1 + b.len()].copy_from_slice(b);
        self.1 += b.len();
    }
    fn u8(&mut self, v: u8) {
        self.bytes(&[v]);
    }
    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
}

/// Reads a span whose length the caller checked.
struct Reader<'a>(&'a [u8], usize);

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut b = [0; N];
        b.copy_from_slice(&self.0[self.1..self.1 + N]);
        self.1 += N;
        b
    }
    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn info_round_trips() {
        let info = Info {
            features: FEATURE_RAILS | FEATURE_INSTRUMENT,
            ticks_per_us: 18,
            max_request: MAX_REQUEST as u16,
            max_response: MAX_RESPONSE as u16,
            max_tb_payload: 240,
            mode: MODE_TINYBOOT,
            bps: 500_000,
            deadline_us: 1000,
            diag: Diag {
                reset: RESET_IWDG,
                phase: PHASE_PUMP,
                crash_seq: 3,
                mcause: 5,
                mepc: 0x2a1c,
                mtval: 0x2000_4000,
                hse_fail: 1,
                resets: 4,
                uptime_ms: 3_733_000,
            },
        };
        let mut buf = [0u8; INFO_LEN];
        assert_eq!(info.write(&mut buf), INFO_LEN);
        assert_eq!(Info::parse(&buf), Some(info));
        assert_eq!(Info::parse(&buf[1..]), None);
    }

    #[test]
    fn exchange_result_round_trips() {
        let r = ExchangeResult {
            outcome: Outcome::Timeout { slot: 2 },
            tick: 7,
            frames: 12,
            garble: 188,
            trailing: true,
            laps: 2,
        };
        let mut buf = [0u8; EXCHANGE_LEN];
        assert_eq!(r.write(&mut buf), EXCHANGE_LEN);
        assert_eq!(buf[..2], [OUTCOME_TIMEOUT, 2]);
        assert_eq!(ExchangeResult::parse(&buf), Some(r));
    }

    #[test]
    fn responses_carry_the_envelope() {
        let mut buf = [0u8; 32];
        assert_eq!(
            error(&mut buf, 0x1234, OSC_EXCHANGE, ERR_BUSY),
            [5, 0, ERROR, 0x34, 0x12, OSC_EXCHANGE, ERR_BUSY]
        );
        assert_eq!(
            done(&mut buf, 7, RAILS, |r| {
                r[0] = 0b01;
                1
            }),
            [5, 0, DONE, 7, 0, RAILS, 0b01]
        );
        assert_eq!(
            osc_frame(&mut buf, 7, 0, 5, 0x80, (&[1, 2], &[3])),
            [9, 0, FRAME, 7, 0, 0, 5, 0x80, 1, 2, 3]
        );
    }
}
