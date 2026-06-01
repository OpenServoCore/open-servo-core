//! ASCII line protocol over USB-CDC. Greppable in pytest logs; deliberately
//! tolerant of whitespace, intolerant of anything else.
//!
//! The injector is intentionally protocol-agnostic on the TX side: callers
//! supply the exact wire bytes via `bytes=<hex>`. That lets the host stay in
//! charge of DXL framing (any FastSlot variant, plain Status, fuzz junk, …)
//! without bloating the firmware with encoders for every shape we want to
//! emit.
//!
//! Host → device
//!   `FIRE bytes=<hex> at=<u64>`
//!       Arm `inject` to push `bytes` onto the wire when SysTick.CNT == `at`.
//!       `at` is in HCLK/8 ticks (18 MHz at 144 MHz).
//!   `ARM bytes=<hex> after_idle=<u32>`
//!       Same TX, but fire `after_idle` ticks after the wire returns to idle.
//!       Listener backdates IDLE-ISR entry by one char-time, so this is the
//!       spec-natural "ticks after end-of-frame" — no compensation needed.
//!   `MASTER bytes=<hex>`
//!       Fire `bytes` now as the bus master; USART1 TC IRQ captures wire-end
//!       (T_request_end). Listener suppresses the IDLE stamp from our own
//!       TX echo so the slave-side stamp ring isn't polluted.
//!   `XFER bytes=<hex> reply_us=<u32>`
//!       MASTER + wait up to `reply_us` for the slave-reply end-of-frame IDLE.
//!       Streams `REPLY <hex>\n` with the slave's reply bytes (extracted from
//!       the RX DMA ring) on success, `NOREPLY\n` on timeout. Drains the stamp
//!       ring first so a stale Round/Plain can't masquerade as this trip's
//!       reply. Handled directly in `usb_cdc::serve` so the hex stream can
//!       span multiple CDC bulk packets.
//!   `RX from=<u32> len=<u16>`
//!       Stream `REPLY <hex>\n` with `len` bytes from the RX DMA ring starting
//!       at absolute byte-count address `from`. Caller must keep `from` within
//!       the last `RX_BUF_LEN` of `BYTES` or the bytes will have been
//!       overwritten. For MASTER+ARM chains where the reply isn't a single
//!       Round trip — split tx echo and arm-emitted bytes apart by offset.
//!   `TICK?`      → `TICK <u64>`           current SysTick.CNT
//!   `LAST?`      → `LAST <u32>`           last `inject` kickoff tick (low half)
//!   `REQ?`       → `REQ <u32>`            last master TC stamp (low half)
//!   `FIRST?`     → `FIRST <u32>`          last slave-reply T0 stamp (low half)
//!   `FIREFIRST?` → `FIREFIRST <u32>`      last FIRE self-echo T0 stamp (low half)
//!   `DRAIN`      → one entry from the listen ring:
//!                  `STAMP <tick> <head>`                     plain bus IDLE
//!                  `ROUND <req> <first> <last> <head>`       master round-trip
//!                  `EMPTY`                                   ring empty
//!   `BYTES`      → `BYTES <u32>`          total RX bytes since boot
//!   `HZ`         → `HZ <u32>`             SysTick ticks per microsecond
//!   `BAUD <bps>` → `OK` or `ERR baud`     retune both USART1 TX + USART3 RX
//!
//! Device → host
//!   `OK`, `ERR <reason>`, plus the per-command replies above. Newlines are
//!   appended by the CDC writer.

use core::str;

use dxl_pirate::parse::decode_hex;

use crate::inject::TX_BUF_LEN;
use crate::{inject, led, listen};

pub enum Reply {
    Ok,
    Err(&'static str),
    Tick(u64),
    Last(u32),
    Req(u32),
    First(u32),
    FireFirst(u32),
    Stamp {
        tick: u32,
        head: u16,
    },
    Round {
        req: u32,
        first: u32,
        last: u32,
        head: u16,
    },
    Empty,
    Bytes(u32),
    HzPerUs(u32),
}

pub struct XferRequest {
    pub payload: [u8; TX_BUF_LEN],
    pub len: usize,
    pub reply_us: u32,
}

pub fn parse_xfer(rest: &[u8]) -> Result<XferRequest, &'static str> {
    let Ok(rest) = str::from_utf8(rest) else {
        return Err("utf8");
    };
    let mut payload = [0u8; TX_BUF_LEN];
    let mut len: Option<usize> = None;
    let mut reply_us: Option<u32> = None;
    for tok in rest.trim().split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Err("kv");
        };
        match k {
            "bytes" => len = decode_hex(v, &mut payload),
            "reply_us" => reply_us = v.parse().ok(),
            _ => return Err("key"),
        }
    }
    let (Some(len), Some(reply_us)) = (len, reply_us) else {
        return Err("missing");
    };
    Ok(XferRequest {
        payload,
        len,
        reply_us,
    })
}

pub struct RxRequest {
    pub from: u32,
    pub len: u16,
}

pub fn parse_rx(rest: &[u8]) -> Result<RxRequest, &'static str> {
    let Ok(rest) = str::from_utf8(rest) else {
        return Err("utf8");
    };
    let mut from: Option<u32> = None;
    let mut len: Option<u16> = None;
    for tok in rest.trim().split_ascii_whitespace() {
        let Some((k, v)) = tok.split_once('=') else {
            return Err("kv");
        };
        match k {
            "from" => from = v.parse().ok(),
            "len" => len = v.parse().ok(),
            _ => return Err("key"),
        }
    }
    let (Some(from), Some(len)) = (from, len) else {
        return Err("missing");
    };
    Ok(RxRequest { from, len })
}

pub fn handle_line(line: &[u8]) -> Reply {
    let Ok(line) = str::from_utf8(line) else {
        return Reply::Err("utf8");
    };
    let line = line.trim();

    if let Some(rest) = line.strip_prefix("FIRE ") {
        return fire(rest);
    }
    if let Some(rest) = line.strip_prefix("ARM ") {
        return arm(rest);
    }
    if let Some(rest) = line.strip_prefix("MASTER ") {
        return master(rest);
    }
    if let Some(rest) = line.strip_prefix("BAUD ") {
        return baud(rest);
    }

    match line {
        "TICK?" => Reply::Tick(inject::read_systick_cnt()),
        "LAST?" => Reply::Last(inject::last_fired_tick()),
        "REQ?" => Reply::Req(inject::last_master_request_end()),
        "FIRST?" => Reply::First(listen::last_t_first()),
        "FIREFIRST?" => Reply::FireFirst(listen::last_fire_t_first()),
        "BYTES" => Reply::Bytes(listen::byte_count()),
        "HZ" => Reply::HzPerUs(inject::ticks_per_us()),
        "DRAIN" => match listen::drain_stamp() {
            Some(listen::IdleStamp::Plain { tick, head }) => Reply::Stamp { tick, head },
            Some(listen::IdleStamp::Round {
                req,
                first,
                last,
                head,
            }) => Reply::Round {
                req,
                first,
                last,
                head,
            },
            None => Reply::Empty,
        },
        _ => Reply::Err("unknown"),
    }
}

fn baud(rest: &str) -> Reply {
    let Ok(bps) = rest.trim().parse::<u32>() else {
        return Reply::Err("baud");
    };
    // Caller must quiesce the bus first — both calls bounce UE around a BRR
    // write, which garbles any byte in flight on either USART.
    if inject::set_baud(bps).is_err() {
        return Reply::Err("baud");
    }
    if listen::set_baud(bps).is_err() {
        return Reply::Err("baud");
    }
    Reply::Ok
}

fn fire(rest: &str) -> Reply {
    let mut at: Option<u64> = None;
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

    match inject::arm(&buf[..len], at) {
        Ok(()) => {
            led::signal();
            Reply::Ok
        }
        Err(inject::ArmError::TooLong) => Reply::Err("toolong"),
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

    match inject::arm_after_idle(&buf[..len], after) {
        Ok(()) => {
            led::signal();
            Reply::Ok
        }
        Err(inject::ArmError::TooLong) => Reply::Err("toolong"),
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

    match inject::master_send(&buf[..len]) {
        Ok(()) => {
            led::signal();
            Reply::Ok
        }
        Err(inject::ArmError::TooLong) => Reply::Err("toolong"),
    }
}
