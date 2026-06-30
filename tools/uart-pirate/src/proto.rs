//! ASCII line protocol over USB-CDC. The high-throughput per-byte stamp
//! drain (`BBATCH`) goes binary because at 3 Mbaud the byte rate
//! (~300 kB/s after framing) won't survive ASCII per-line framing; the
//! binary handler lives in `usb_cdc::serve` since the frame spans
//! multiple CDC bulk packets.
//!
//! Host → device
//!   `FIRE bytes=<hex> at=<u32>`
//!       Arm `inject` to push `bytes` onto the wire when tick32 == `at`.
//!   `ARM bytes=<hex> after_idle=<u32>`
//!       Same TX, but fire `after_idle` tick32 ticks after the next
//!       USART IDLE assertion (≈ 1 char-time after wire-end).
//!   `MASTER bytes=<hex>`
//!       Fire `bytes` now as the bus master. Host derives the request-
//!       end stamp from the per-byte stream.
//!   `BDRAIN`
//!       Pop one byte record. Reply: `BSTAMP <tick> <byte> <flags>` or
//!       `EMPTY`. ASCII; debug only — use `BBATCH` for throughput.
//!   `BBATCH <count:u16>`
//!       Drain up to `count` byte records as a binary frame. Handled
//!       directly in `usb_cdc::serve` — see that file for wire format.
//!   `BTRACE`
//!       Pop one walker-ISR trace record. Reply: `BTRACE <phase>
//!       <cnt_entry> <cnt_exit> <pending> <edges> <bytes> <falling_total>
//!       <rx_total>` (decimal) or `EMPTY`.
//!   `BTRACECLEAR`
//!       Reset the trace ring tail. Use before a stress run.
//!   `STATUS`
//!       `STATUS <baud> <avail> <desynced> <cause> <last_tick>`. Always
//!       responds — even mid-desync — so the host has a one-shot health
//!       probe that can't be masked by walker state.
//!   `RESET`
//!       Clear `DESYNCED` + cause, drain stamp/IC rings, re-arm walker.
//!       Keeps baud. The routine recovery for any desync cause.
//!   `BAUD <bps>` → `OK` / `ERR baud`. Implicit RESET. Caller must
//!       quiesce the bus.
//!   `COMP?`      → `COMP pipe=<u32> bit_q4=<u32>`. Reads the TX-comp
//!       tunables — static HCLK-domain pipeline ticks and the bit-clock
//!       multiplier in Q4 (16 = 1.0 × brr).
//!   `COMP pipe=<u32> bit_q4=<u32>`
//!       Set one or both tunables (missing key leaves that side
//!       unchanged); `FIRE_COMP_TICKS` is recomputed for the current
//!       baud immediately. Bypasses the desync guard so bench can
//!       calibrate after a trip.
//!   `TICK?`      → `TICK <tick32:u32>`
//!   `LAST?`      → `LAST <tick32:u32>` last fire kickoff
//!   `HZ`         → `HZ <u32>` tick32 ticks per microsecond
//!
//! Device → host
//!   `OK`, `ERR <reason>`, plus the per-command replies above. Newlines
//!   are appended by the CDC writer.
//!
//! Desync handling: `STATUS`, `RESET`, and `BAUD` are processed
//! unconditionally so the host always has a recovery path. Every other
//! command returns `ERR desync <cause>` while the flag is set.

use core::str;

use uart_pirate::parse::decode_hex;

use crate::capture::{self, DesyncCause};
use crate::inject::{self, TX_BUF_LEN};
use crate::led;

pub enum Reply {
    Ok,
    Err(&'static str),
    Tick(u32),
    Last(u32),
    BStamp {
        tick: u32,
        byte: u8,
        flags: u8,
    },
    BTrace(capture::WalkerTrace),
    Empty,
    Status {
        baud: u32,
        avail: u32,
        cause: Option<DesyncCause>,
        last_tick: u32,
    },
    HzPerUs(u32),
    Comp {
        pipe: u32,
        bit_q4: u32,
    },
}

pub struct BatchRequest {
    pub count: u16,
}

pub fn parse_batch(rest: &[u8]) -> Result<BatchRequest, &'static str> {
    let Ok(rest) = str::from_utf8(rest) else {
        return Err("utf8");
    };
    let count: u16 = rest.trim().parse().map_err(|_| "count")?;
    Ok(BatchRequest { count })
}

/// Returns the static error string for a desync cause. Format used by
/// every command that errors out on DESYNCED, plus by `usb_cdc::handle_batch`
/// for the BBATCH error path.
pub fn desync_err_for(cause: DesyncCause) -> &'static str {
    match cause {
        DesyncCause::IcOverrun => "desync ic_overrun",
        DesyncCause::StampOverflow => "desync stamp_overflow",
    }
}

pub fn handle_line(line: &[u8]) -> Reply {
    let Ok(line) = str::from_utf8(line) else {
        return Reply::Err("utf8");
    };
    let line = line.trim();

    // STATUS, RESET, BAUD bypass the desync guard so the host always has
    // a recovery path (and a probe that surfaces the current state).
    // BTRACE / BTRACECLEAR also bypass — they're read-only diagnostics
    // against the walker trace ring and surfacing them post-desync lets
    // the host inspect what the walker was doing at the moment of trip.
    if line == "STATUS" {
        return status();
    }
    if line == "RESET" {
        capture::reset_walker();
        return Reply::Ok;
    }
    if let Some(rest) = line.strip_prefix("BAUD ") {
        return baud(rest);
    }
    if line == "COMP?" {
        let (pipe, bit_q4) = inject::tx_comp();
        return Reply::Comp { pipe, bit_q4 };
    }
    if let Some(rest) = line.strip_prefix("COMP ") {
        return comp(rest);
    }
    if line == "BTRACE" {
        return match capture::trace_drain() {
            Some(r) => Reply::BTrace(r),
            None => Reply::Empty,
        };
    }
    if line == "BTRACECLEAR" {
        capture::trace_clear();
        return Reply::Ok;
    }

    if let Some(cause) = capture::desync_cause() {
        return Reply::Err(desync_err_for(cause));
    }

    if let Some(rest) = line.strip_prefix("FIRE ") {
        return fire(rest);
    }
    if let Some(rest) = line.strip_prefix("ARM ") {
        return arm(rest);
    }
    if let Some(rest) = line.strip_prefix("MASTER ") {
        return master(rest);
    }

    match line {
        "TICK?" => Reply::Tick(inject::read_tick32()),
        "LAST?" => Reply::Last(inject::last_fired_tick()),
        "HZ" => Reply::HzPerUs(inject::wire_ticks_per_us()),
        "BDRAIN" => match capture::drain_byte() {
            Some(r) => Reply::BStamp {
                tick: r.tick,
                byte: r.byte,
                flags: r.flags,
            },
            None => Reply::Empty,
        },
        _ => Reply::Err("unknown"),
    }
}

fn status() -> Reply {
    Reply::Status {
        baud: capture::current_baud(),
        avail: capture::stamps_available(),
        cause: capture::desync_cause(),
        last_tick: inject::last_fired_tick(),
    }
}

fn comp(rest: &str) -> Reply {
    let mut pipe: Option<u32> = None;
    let mut bit_q4: Option<u32> = None;

    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "pipe" => match v.parse() {
                Ok(n) => pipe = Some(n),
                Err(_) => return Reply::Err("pipe"),
            },
            "bit_q4" => match v.parse() {
                Ok(n) => bit_q4 = Some(n),
                Err(_) => return Reply::Err("bit_q4"),
            },
            _ => return Reply::Err("key"),
        }
    }
    if pipe.is_none() && bit_q4.is_none() {
        return Reply::Err("missing");
    }
    inject::set_tx_comp(pipe, bit_q4);
    Reply::Ok
}

fn baud(rest: &str) -> Reply {
    let Ok(bps) = rest.trim().parse::<u32>() else {
        return Reply::Err("baud");
    };
    match inject::set_baud(bps) {
        Ok(()) => {}
        Err(inject::BaudError::OutOfRange) => return Reply::Err("baud"),
        Err(inject::BaudError::Busy) => return Reply::Err("busy"),
    }
    if capture::set_baud(bps).is_err() {
        return Reply::Err("baud");
    }
    Reply::Ok
}

fn fire(rest: &str) -> Reply {
    let mut at: Option<u32> = None;
    let mut buf = [0u8; TX_BUF_LEN];
    let mut len: Option<usize> = None;

    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "at" => at = v.parse().ok(),
            "bytes" => len = decode_hex(v, &mut buf),
            _ => return Reply::Err("key"),
        }
    }
    let (Some(at), Some(len)) = (at, len) else {
        return Reply::Err("missing");
    };

    match inject::schedule_fire(&buf[..len], at) {
        Ok(()) => {
            led::signal();
            Reply::Ok
        }
        Err(inject::ArmError::TooLong) => Reply::Err("toolong"),
        Err(inject::ArmError::Busy) => Reply::Err("busy"),
    }
}

fn arm(rest: &str) -> Reply {
    let mut after: Option<u32> = None;
    let mut buf = [0u8; TX_BUF_LEN];
    let mut len: Option<usize> = None;

    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "after_idle" => after = v.parse().ok(),
            "bytes" => len = decode_hex(v, &mut buf),
            _ => return Reply::Err("key"),
        }
    }
    let (Some(after), Some(len)) = (after, len) else {
        return Reply::Err("missing");
    };

    match inject::schedule_fire_after_idle(&buf[..len], after) {
        Ok(()) => {
            led::signal();
            Reply::Ok
        }
        Err(inject::ArmError::TooLong) => Reply::Err("toolong"),
        Err(inject::ArmError::Busy) => Reply::Err("busy"),
    }
}

fn master(rest: &str) -> Reply {
    let mut buf = [0u8; TX_BUF_LEN];
    let mut len: Option<usize> = None;

    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "bytes" => len = decode_hex(v, &mut buf),
            _ => return Reply::Err("key"),
        }
    }
    let Some(len) = len else {
        return Reply::Err("missing");
    };

    match inject::fire_now_master(&buf[..len]) {
        Ok(()) => {
            led::signal();
            Reply::Ok
        }
        Err(inject::ArmError::TooLong) => Reply::Err("toolong"),
        Err(inject::ArmError::Busy) => Reply::Err("busy"),
    }
}
