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
//!   `TICK?`      → `TICK <u64>`           current SysTick.CNT
//!   `LAST?`      → `LAST <u32>`           last `inject` kickoff tick (low half)
//!   `DRAIN`      → `STAMP <tick> <head>`  one entry from the listen ring, or
//!                  `EMPTY`                if empty
//!   `BYTES`      → `BYTES <u32>`          total RX bytes since boot
//!   `HZ`         → `HZ <u32>`             SysTick ticks per microsecond
//!   `BAUD <bps>` → `OK` or `ERR baud`     retune both USART2 TX + USART3 RX
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
    Stamp { tick: u32, head: u16 },
    Empty,
    Bytes(u32),
    HzPerUs(u32),
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
    if let Some(rest) = line.strip_prefix("BAUD ") {
        return baud(rest);
    }

    match line {
        "TICK?" => Reply::Tick(inject::read_systick_cnt()),
        "LAST?" => Reply::Last(inject::last_fired_tick()),
        "BYTES" => Reply::Bytes(listen::byte_count()),
        "HZ" => Reply::HzPerUs(inject::ticks_per_us()),
        "DRAIN" => match listen::drain_stamp() {
            Some(s) => Reply::Stamp {
                tick: s.tick,
                head: s.head,
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
