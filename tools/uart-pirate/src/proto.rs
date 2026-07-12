//! ASCII line protocol over USB-CDC. The high-throughput per-byte stamp
//! drain (`BBATCH`) goes binary because at 3 Mbaud the byte rate
//! (~300 kB/s after framing) won't survive ASCII per-line framing; the
//! binary handler lives in `usb_cdc::serve` since the frame spans
//! multiple CDC bulk packets.
//!
//! Host → device
//!   `SEND bytes=<hex>`
//!       Push `bytes` onto the wire now (no precise timing — wire-start
//!       lands a few µs after the command).
//!   `SEND bytes=<hex> at=<u32>`
//!       Push `bytes` onto the wire when tick32 == `at`.
//!   `SEND bytes=<hex> after_idle=<u32>`
//!       Push `bytes` `after_idle` tick32 ticks after the next USART
//!       IDLE assertion (≈ 1 char-time after wire-end).
//!   `BDRAIN`
//!       Pop one byte record. Reply: `BSTAMP <tick> <byte> <flags>` or
//!       `EMPTY`. ASCII; debug only — use `BBATCH` for throughput.
//!       Ticks are break-boundary-anchored (`rx::stamp`): breaks carry
//!       real capture ticks, interior bytes nominal bit-time strides.
//!   `BBATCH <count:u16>`
//!       Drain up to `count` byte records as a binary frame. Handled
//!       directly in `usb_cdc::serve` — see that file for wire format.
//!   `STATUS`
//!       `STATUS <baud> <avail> <desynced> <cause> <last_tick>`. Always
//!       responds — even mid-desync — so the host has a one-shot health
//!       probe.
//!   `RESET`
//!       Clear the desync flag and drop undrained capture state. Keeps
//!       baud. The routine recovery for a desync.
//!   `BAUD <bps>` → `OK` / `ERR baud`. Implicit RESET. Caller must
//!       quiesce the bus.
//!   `COMP?`      → `COMP pipe=<u32> bit_q4=<u32>`. Reads the TX-comp
//!       tunables — static HCLK-domain pipeline ticks and the bit-clock
//!       multiplier in Q4 (16 = 1.0 × brr).
//!   `COMP pipe=<u32> bit_q4=<u32>`
//!       Set one or both tunables (missing key leaves that side
//!       unchanged); `TX_COMP_TICKS` is recomputed for the current baud
//!       immediately. Bypasses the desync guard so bench can calibrate
//!       after a trip.
//!   `BREAK [n=<count>] [gap_us=<us>]`
//!       Send `n` (default 1) UART breaks via SBK, `gap_us` (default
//!       1000) apart. Blocks until the last break has shifted out.
//!       Break-framing spike command.
//!   `BRKSEND bytes=<hex>`
//!       One 10-bit-exact break (a 9-bit 0x00 frame, not SBK) followed
//!       back-to-back by `bytes` (poll-fed, ≤ 272 B) — the osc-native
//!       host send primitive, same break shape as a servo reply, and
//!       the shape LIN break detection (LBDL=0) keys on.
//!   `BURST bytes=<hex>`
//!       Zero-gap multi-frame bombardment. `bytes` is length-prefixed
//!       frames (`[len_0][frame_0][len_1][frame_1]…`, ≤ 640 B total);
//!       each frame goes out as one 10-bit-exact break + its bytes,
//!       back-to-back with sub-byte spacing throughout.
//!   `CAL bytes=<hex> gap_us=<us> breaks=<n>`
//!       Break-framed `bytes` (the announce), then `n` bare breaks whose
//!       start edges sit on an exact `gap_us` grid, tick32-paced against
//!       the crystal — the osc-native MGMT CAL train (§9.3).
//!   `LOWPULSE us=<n>`
//!       Drive TX low as a GPIO for `n` µs (≤ 100 ms), then restore AF.
//!       The osc-native "rescue break" shape — detectable at any baud.
//!   `FEINJ pre=<hex> bad=<hex> post=<hex>`
//!       Back-to-back stream where the `bad` bytes go out as 9-bit
//!       frames with bit 8 = 0 — a mid-stream framing error at an 8N1
//!       receiver, with real data levels (not a break). Any of the
//!       three keys may be omitted (empty).
//!   `TICK?`      → `TICK <tick32:u32>`
//!   `LAST?`      → `LAST <tick32:u32>` last send kickoff
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

use crate::parse::decode_hex;

use crate::led;
use crate::rx::{self, DesyncCause};
use crate::tick;
use crate::tx::{self, TX_BUF_LEN};

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
    if line == "STATUS" {
        return status();
    }
    if line == "RESET" {
        rx::reset();
        return Reply::Ok;
    }
    if let Some(rest) = line.strip_prefix("BAUD ") {
        return baud(rest);
    }
    if line == "COMP?" {
        let (pipe, bit_q4) = tx::tx_comp();
        return Reply::Comp { pipe, bit_q4 };
    }
    if let Some(rest) = line.strip_prefix("COMP ") {
        return comp(rest);
    }

    if let Some(cause) = rx::desync_cause() {
        return Reply::Err(desync_err_for(cause));
    }

    if let Some(rest) = line.strip_prefix("SEND ") {
        return send(rest);
    }
    if line == "BREAK" {
        return brk("");
    }
    if let Some(rest) = line.strip_prefix("BREAK ") {
        return brk(rest);
    }
    if let Some(rest) = line.strip_prefix("BRKSEND ") {
        return brksend(rest);
    }
    if let Some(rest) = line.strip_prefix("BURST ") {
        return burst(rest);
    }
    if let Some(rest) = line.strip_prefix("CAL ") {
        return cal(rest);
    }
    if let Some(rest) = line.strip_prefix("LOWPULSE ") {
        return lowpulse(rest);
    }
    if let Some(rest) = line.strip_prefix("FEINJ ") {
        return feinj(rest);
    }

    match line {
        "TICK?" => Reply::Tick(tick::read_tick32()),
        "LAST?" => Reply::Last(tx::last_send_tick()),
        "HZ" => Reply::HzPerUs(tick::wire_ticks_per_us()),
        "BDRAIN" => match rx::drain_byte() {
            Ok(Some(r)) => Reply::BStamp {
                tick: r.tick,
                byte: r.byte,
                flags: r.flags,
            },
            Ok(None) => Reply::Empty,
            Err(cause) => Reply::Err(desync_err_for(cause)),
        },
        _ => Reply::Err("unknown"),
    }
}

fn status() -> Reply {
    Reply::Status {
        baud: rx::current_baud(),
        avail: rx::stamps_available(),
        cause: rx::desync_cause(),
        last_tick: tx::last_send_tick(),
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
    tx::set_tx_comp(pipe, bit_q4);
    Reply::Ok
}

fn baud(rest: &str) -> Reply {
    let Ok(bps) = rest.trim().parse::<u32>() else {
        return Reply::Err("baud");
    };
    match tx::set_baud(bps) {
        Ok(()) => {}
        Err(tx::BaudError::OutOfRange) => return Reply::Err("baud"),
        Err(tx::BaudError::Busy) => return Reply::Err("busy"),
    }
    if rx::set_baud(bps).is_err() {
        return Reply::Err("baud");
    }
    Reply::Ok
}

fn brk(rest: &str) -> Reply {
    let mut n: u32 = 1;
    let mut gap_us: u32 = 1000;
    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "n" => match v.parse() {
                Ok(x) => n = x,
                Err(_) => return Reply::Err("n"),
            },
            "gap_us" => match v.parse() {
                Ok(x) => gap_us = x,
                Err(_) => return Reply::Err("gap_us"),
            },
            _ => return Reply::Err("key"),
        }
    }
    match tx::send_breaks(n, gap_us) {
        Ok(()) => Reply::Ok,
        Err(tx::SendError::TooLong) => Reply::Err("range"),
        Err(tx::SendError::Busy) => Reply::Err("busy"),
    }
}

fn brksend(rest: &str) -> Reply {
    let mut buf = [0u8; tx::BRK_PAYLOAD_MAX];
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
    match tx::send_break_then(&buf[..len]) {
        Ok(()) => Reply::Ok,
        Err(tx::SendError::TooLong) => Reply::Err("toolong"),
        Err(tx::SendError::Busy) => Reply::Err("busy"),
    }
}

fn burst(rest: &str) -> Reply {
    let mut buf = [0u8; tx::BURST_STREAM_MAX];
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
    match tx::send_burst(&buf[..len]) {
        Ok(()) => Reply::Ok,
        Err(tx::SendError::TooLong) => Reply::Err("toolong"),
        Err(tx::SendError::Busy) => Reply::Err("busy"),
    }
}

fn feinj(rest: &str) -> Reply {
    let mut pre = [0u8; tx::FE_INJECT_MAX];
    let mut bad = [0u8; tx::FE_INJECT_MAX];
    let mut post = [0u8; tx::FE_INJECT_MAX];
    let (mut n_pre, mut n_bad, mut n_post) = (0usize, 0usize, 0usize);
    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        let n = match k {
            "pre" => decode_hex(v, &mut pre).map(|n| n_pre = n),
            "bad" => decode_hex(v, &mut bad).map(|n| n_bad = n),
            "post" => decode_hex(v, &mut post).map(|n| n_post = n),
            _ => return Reply::Err("key"),
        };
        if n.is_none() {
            return Reply::Err("hex");
        }
    }
    match tx::send_fe_inject(&pre[..n_pre], &bad[..n_bad], &post[..n_post]) {
        Ok(()) => Reply::Ok,
        Err(tx::SendError::TooLong) => Reply::Err("toolong"),
        Err(tx::SendError::Busy) => Reply::Err("busy"),
    }
}

fn cal(rest: &str) -> Reply {
    let mut buf = [0u8; tx::BRK_PAYLOAD_MAX];
    let mut len: Option<usize> = None;
    let mut gap_us: Option<u32> = None;
    let mut breaks: Option<u32> = None;
    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "bytes" => len = decode_hex(v, &mut buf),
            "gap_us" => match v.parse() {
                Ok(x) => gap_us = Some(x),
                Err(_) => return Reply::Err("gap_us"),
            },
            "breaks" => match v.parse() {
                Ok(x) => breaks = Some(x),
                Err(_) => return Reply::Err("breaks"),
            },
            _ => return Reply::Err("key"),
        }
    }
    let (Some(len), Some(gap_us), Some(breaks)) = (len, gap_us, breaks) else {
        return Reply::Err("missing");
    };
    match tx::send_cal(&buf[..len], gap_us, breaks) {
        Ok(()) => Reply::Ok,
        Err(tx::SendError::TooLong) => Reply::Err("range"),
        Err(tx::SendError::Busy) => Reply::Err("busy"),
    }
}

fn lowpulse(rest: &str) -> Reply {
    let mut us: Option<u32> = None;
    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "us" => us = v.parse().ok(),
            _ => return Reply::Err("key"),
        }
    }
    let Some(us) = us else {
        return Reply::Err("missing");
    };
    match tx::low_pulse_us(us) {
        Ok(()) => Reply::Ok,
        Err(tx::SendError::TooLong) => Reply::Err("range"),
        Err(tx::SendError::Busy) => Reply::Err("busy"),
    }
}

fn send(rest: &str) -> Reply {
    let mut at: Option<u32> = None;
    let mut after_idle: Option<u32> = None;
    let mut buf = [0u8; TX_BUF_LEN];
    let mut len: Option<usize> = None;

    for tok in rest.split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Reply::Err("kv");
        };
        match k {
            "at" => at = v.parse().ok(),
            "after_idle" => after_idle = v.parse().ok(),
            "bytes" => len = decode_hex(v, &mut buf),
            _ => return Reply::Err("key"),
        }
    }
    let Some(len) = len else {
        return Reply::Err("missing");
    };
    // SAFETY: (Some, Some) returns "conflict" up-front, so the match
    // below never sees both keys.
    if at.is_some() && after_idle.is_some() {
        return Reply::Err("conflict");
    }

    let result = match (at, after_idle) {
        (Some(at), None) => {
            // Commanded-tick send displaces a stale idle arm; a plain
            // `SEND` (send_now) coexists with one — see `cancel_idle_send`.
            tx::cancel_idle_send();
            tx::schedule_send_at(&buf[..len], at)
        }
        (None, Some(after)) => tx::schedule_send_after_idle(&buf[..len], after),
        (None, None) => tx::send_now(&buf[..len]),
        (Some(_), Some(_)) => return Reply::Err("conflict"),
    };
    match result {
        Ok(()) => {
            led::signal();
            Reply::Ok
        }
        Err(tx::SendError::TooLong) => Reply::Err("toolong"),
        Err(tx::SendError::Busy) => Reply::Err("busy"),
    }
}
