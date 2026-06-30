//! Raw USB-CDC client for the uart-pirate device.
//!
//! Internal to the bench crate — bins import [`crate::Bus`] (the
//! task-shaped facade) and never touch this module directly. All wire
//! grammar (ASCII command lines, binary BBATCH frames, sticky DESYNCED
//! state) is encapsulated here; the rest of the lib turns those calls
//! into protocol-level operations.

use std::io::{self, Read, Write};
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use serialport::SerialPort;

pub const PIRATE_VID: u16 = 0xC0DE;
pub const PIRATE_PID: u16 = 0xCAFE;

// ---------------------------------------------------------------------------
// Public types (re-exported from lib.rs root)
// ---------------------------------------------------------------------------

/// One walker-emitted stamp: per-byte capture tick + decoded byte + USART
/// flags (PE/FE/NE/ORE).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct BStamp {
    pub tick: u32,
    pub byte: u8,
    pub flags: u8,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DesyncCause {
    IcOverrun,
    StampOverflow,
}

impl DesyncCause {
    pub fn parse(s: &str) -> Option<Self> {
        match s.trim() {
            "ic_overrun" => Some(Self::IcOverrun),
            "stamp_overflow" => Some(Self::StampOverflow),
            _ => None,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Self::IcOverrun => "ic_overrun",
            Self::StampOverflow => "stamp_overflow",
        }
    }
}

impl std::fmt::Display for DesyncCause {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

/// Typed error returned when the pirate replies `ERR desync <cause>`.
/// Wrapped through `anyhow::Error`; callers that need to match can
/// `err.downcast_ref::<PirateDesync>()`.
#[derive(Copy, Clone, Debug)]
pub struct PirateDesync(pub DesyncCause);

impl std::fmt::Display for PirateDesync {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "pirate DESYNCED ({})", self.0)
    }
}

impl std::error::Error for PirateDesync {}

#[derive(Copy, Clone, Debug)]
pub struct PirateStatus {
    pub baud: u32,
    pub avail: u32,
    pub cause: Option<DesyncCause>,
    pub last_tick: u32,
}

/// TX-comp tunables read from the pirate via `COMP?`. See firmware
/// `inject.rs` for the decomposition:
/// `TX_COMP_TICKS = pipe + (bit_q4 × brr) / 16`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TxComp {
    /// Static HCLK-domain pipeline ticks (TIM4 CC2 → USART3.DR write).
    pub pipe: u32,
    /// USART bit-clock multiplier in Q4 (16 = 1.0 × brr).
    pub bit_q4: u32,
}

/// Full capture from one transfer: the raw per-byte stream plus
/// (when a reply arrived) the three load-bearing timing points.
#[derive(Clone, Debug, Default)]
pub struct ReplyCapture {
    pub stamps: Vec<BStamp>,
    pub timing: Option<ReplyTiming>,
}

/// Replacement for the old `Round { req, first, last }`. All ticks are in
/// the pirate's 144 MHz tick32 domain; derive elapsed µs via
/// `Client::hz_per_us`.
#[derive(Copy, Clone, Debug)]
pub struct ReplyTiming {
    /// Last byte of the host TX echo. Equivalent to the old `Round.req`.
    pub req_end: u32,
    /// First byte after the host TX echo. Equivalent to `Round.first`.
    pub reply_first: u32,
    /// Last byte in the captured stream. Equivalent to `Round.last`.
    pub reply_last: u32,
}

/// `BICSNAP` payload. Atomic view of the IC ring (pre-lifted u32 ticks
/// plus walker state) at the moment the pirate served the request.
/// `entries` holds the in-ring window in OLDEST-FIRST order; entries are
/// already lifted to `tick32` on chip via the walker's `IC_TICKS_RING`,
/// so the host can compare them directly against stamp ticks without a
/// host-side lift step.
///
/// `ref_tick` is the walker's "now" reference at its most recent exit —
/// useful for relating the entry window to wall time, not required for
/// using the ticks themselves.
#[derive(Clone, Debug)]
pub struct IcSnapshot {
    pub ref_tick: u32,
    pub falling_total: u32,
    pub walked: u32,
    pub rx_total: u32,
    pub byte_head: u32,
    pub bit_ticks: u32,
    pub cc_filter_delay: u32,
    pub entries: Vec<u32>,
}

// ---------------------------------------------------------------------------
// Client
// ---------------------------------------------------------------------------

pub struct Client {
    port: Box<dyn SerialPort>,
    port_path: String,
    hz_per_us: Option<u32>,
    baud: u32,
}

impl Client {
    /// Open a pirate by USB-CDC path. Asserts DTR/RTS so the pirate's
    /// `wait_connection()` unblocks, queries STATUS, and clears any
    /// leftover DESYNCED state (from a prior crashed run) via RESET.
    pub fn open(port_path: &str, timeout: Duration) -> Result<Self> {
        let mut port = serialport::new(port_path, 115_200)
            .timeout(timeout)
            .open()
            .with_context(|| format!("opening pirate at {port_path}"))?;
        port.write_data_terminal_ready(true)
            .context("asserting DTR")?;
        port.write_request_to_send(true).context("asserting RTS")?;
        let mut client = Self {
            port,
            port_path: port_path.to_string(),
            hz_per_us: None,
            baud: 0,
        };
        let st = client.status()?;
        client.baud = st.baud;
        if st.cause.is_some() {
            client.reset()?;
        }
        Ok(client)
    }

    pub fn port_path(&self) -> &str {
        &self.port_path
    }

    #[allow(dead_code)]
    pub fn current_baud(&self) -> u32 {
        self.baud
    }

    // -----------------------------------------------------------------------
    // Low-level wire I/O
    // -----------------------------------------------------------------------

    fn discard_pending(&mut self) -> Result<()> {
        loop {
            let avail = self.port.bytes_to_read()?;
            if avail == 0 {
                return Ok(());
            }
            let mut tmp = vec![0u8; avail as usize];
            self.port.read_exact(&mut tmp)?;
        }
    }

    fn send_line(&mut self, line: &str) -> Result<()> {
        self.discard_pending()?;
        self.port.write_all(line.as_bytes())?;
        self.port.write_all(b"\n")?;
        self.port.flush()?;
        Ok(())
    }

    fn read_line(&mut self) -> Result<String> {
        let mut buf = Vec::with_capacity(64);
        let mut b = [0u8; 1];
        loop {
            match self.port.read(&mut b) {
                Ok(0) => continue,
                Ok(_) => {
                    if b[0] == b'\n' {
                        break;
                    }
                    if b[0] != b'\r' {
                        buf.push(b[0]);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                    bail!("pirate read timed out on {}", self.port_path);
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok(String::from_utf8_lossy(&buf).into_owned())
    }

    fn read_exact_bytes(&mut self, n: usize) -> Result<Vec<u8>> {
        let mut out = vec![0u8; n];
        self.port.read_exact(&mut out).map_err(|e| match e.kind() {
            io::ErrorKind::TimedOut => {
                anyhow!("pirate read timed out on {} (binary frame)", self.port_path)
            }
            _ => e.into(),
        })?;
        Ok(out)
    }

    fn check_desync_line(&self, line: &str) -> Option<PirateDesync> {
        line.strip_prefix("ERR desync ")
            .and_then(DesyncCause::parse)
            .map(PirateDesync)
    }

    /// Send one command, read one ASCII reply line. Returns `Err` (with
    /// `PirateDesync` in the chain) if the pirate replied with a desync
    /// error.
    pub fn command(&mut self, line: &str) -> Result<String> {
        self.send_line(line)?;
        let reply = self.read_line()?;
        if let Some(desync) = self.check_desync_line(&reply) {
            return Err(anyhow!(desync));
        }
        Ok(reply)
    }

    pub fn expect_ok(&mut self, line: &str) -> Result<()> {
        let reply = self.command(line)?;
        if reply != "OK" {
            bail!("{line:?} → {reply:?}");
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Wire commands
    // -----------------------------------------------------------------------

    pub fn set_baud(&mut self, bps: u32) -> Result<()> {
        self.expect_ok(&format!("BAUD {bps}"))?;
        self.baud = bps;
        Ok(())
    }

    pub fn tick(&mut self) -> Result<u32> {
        let reply = self.command("TICK?")?;
        parse_kv(&reply, "TICK")
    }

    pub fn last_send_tick(&mut self) -> Result<u32> {
        let reply = self.command("LAST?")?;
        parse_kv(&reply, "LAST")
    }

    /// 144 MHz tick32 ticks per microsecond on the pirate (compile-time
    /// firmware constant). Cached after the first query.
    pub fn hz_per_us(&mut self) -> Result<u32> {
        if let Some(v) = self.hz_per_us {
            return Ok(v);
        }
        let reply = self.command("HZ")?;
        let v = parse_kv::<u32>(&reply, "HZ")?;
        self.hz_per_us = Some(v);
        Ok(v)
    }

    pub fn send_now(&mut self, data: &[u8]) -> Result<()> {
        self.expect_ok(&format!("SEND bytes={}", hex(data)))
    }

    /// Stage `data` to send `after_idle_us` microseconds after the next
    /// wire IDLE. Wall-clock units; converts to ticks via cached
    /// `hz_per_us`.
    pub fn schedule_send_after_idle(&mut self, data: &[u8], after_idle_us: u32) -> Result<()> {
        let ticks = after_idle_us * self.hz_per_us()?;
        self.expect_ok(&format!("SEND bytes={} after_idle={ticks}", hex(data)))
    }

    pub fn schedule_send_at(&mut self, data: &[u8], at_tick: u32) -> Result<()> {
        self.expect_ok(&format!("SEND bytes={} at={at_tick}", hex(data)))
    }

    pub fn status(&mut self) -> Result<PirateStatus> {
        let reply = self.command("STATUS")?;
        parse_status(&reply)
    }

    /// Read TX-comp tunables: HCLK-domain pipeline ticks and the bit-clock
    /// multiplier in Q4 (16 = 1.0 × brr). Bypasses the desync guard so
    /// it's safe to call after a stage trip.
    pub fn comp(&mut self) -> Result<TxComp> {
        let reply = self.command("COMP?")?;
        parse_comp(&reply)
    }

    /// Write one or both TX-comp tunables. `bit_q4` clamps to a u32 — the
    /// firmware refuses negative values at the parser layer.
    pub fn set_comp(&mut self, pipe: u32, bit_q4: u32) -> Result<()> {
        self.expect_ok(&format!("COMP pipe={pipe} bit_q4={bit_q4}"))
    }

    pub fn reset(&mut self) -> Result<()> {
        self.expect_ok("RESET")
    }

    /// Pop one byte record via ASCII `BDRAIN`. Debug path; for sustained
    /// drain use [`Self::bbatch`].
    #[allow(dead_code)]
    pub fn drain_byte(&mut self) -> Result<Option<BStamp>> {
        let reply = self.command("BDRAIN")?;
        if reply == "EMPTY" {
            return Ok(None);
        }
        let parts: Vec<&str> = reply.split_whitespace().collect();
        match parts.as_slice() {
            ["BSTAMP", tick, byte, flags] => Ok(Some(BStamp {
                tick: tick.parse()?,
                byte: byte.parse()?,
                flags: flags.parse()?,
            })),
            _ => bail!("unexpected BDRAIN reply: {reply:?}"),
        }
    }

    /// Pull up to `max` records as a binary BBATCH frame:
    /// `[0xA5 0x5A][count:u16 LE][count × (tick:u32 LE | byte | flags)]`.
    /// If the pirate is DESYNCED the reply is an ASCII `ERR desync …`
    /// line instead; the desync is surfaced as `PirateDesync` in the
    /// error chain.
    pub fn bbatch(&mut self, max: u16) -> Result<Vec<BStamp>> {
        self.send_line(&format!("BBATCH {max}"))?;
        // Peek the first byte: 0xA5 → binary frame; anything else → ASCII
        // error line. Both paths must read to end so the socket stays
        // aligned for the next command.
        let first = self.read_exact_bytes(1)?[0];
        if first != 0xA5 {
            let mut buf = vec![first];
            let mut b = [0u8; 1];
            loop {
                self.port.read_exact(&mut b)?;
                if b[0] == b'\n' {
                    break;
                }
                if b[0] != b'\r' {
                    buf.push(b[0]);
                }
            }
            let line = String::from_utf8_lossy(&buf);
            if let Some(desync) = self.check_desync_line(&line) {
                return Err(anyhow!(desync));
            }
            bail!("BBATCH unexpected reply: {line:?}");
        }
        let sync2 = self.read_exact_bytes(1)?[0];
        if sync2 != 0x5A {
            bail!("BBATCH bad sync: 0xA5 0x{sync2:02X}");
        }
        let count_bytes = self.read_exact_bytes(2)?;
        let count = u16::from_le_bytes([count_bytes[0], count_bytes[1]]) as usize;
        if count == 0 {
            return Ok(Vec::new());
        }
        let body = self.read_exact_bytes(count * 6)?;
        let mut out = Vec::with_capacity(count);
        for i in 0..count {
            let off = i * 6;
            out.push(BStamp {
                tick: u32::from_le_bytes([body[off], body[off + 1], body[off + 2], body[off + 3]]),
                byte: body[off + 4],
                flags: body[off + 5],
            });
        }
        Ok(out)
    }

    /// Read the IC ring snapshot frame. Bypasses the pirate's desync guard
    /// — use freely as a post-trip diagnostic.
    pub fn ic_snapshot(&mut self) -> Result<IcSnapshot> {
        self.send_line("BICSNAP")?;
        let first = self.read_exact_bytes(1)?[0];
        if first != 0xA5 {
            let mut buf = vec![first];
            let mut b = [0u8; 1];
            loop {
                self.port.read_exact(&mut b)?;
                if b[0] == b'\n' {
                    break;
                }
                if b[0] != b'\r' {
                    buf.push(b[0]);
                }
            }
            bail!(
                "BICSNAP unexpected reply: {:?}",
                String::from_utf8_lossy(&buf)
            );
        }
        let sync2 = self.read_exact_bytes(1)?[0];
        if sync2 != 0x5C {
            bail!("BICSNAP bad sync: 0xA5 0x{sync2:02X}");
        }
        let hdr = self.read_exact_bytes(30)?;
        let ref_tick = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        let falling_total = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        let walked = u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]);
        let rx_total = u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]);
        let byte_head = u32::from_le_bytes([hdr[16], hdr[17], hdr[18], hdr[19]]);
        let bit_ticks = u32::from_le_bytes([hdr[20], hdr[21], hdr[22], hdr[23]]);
        let cc_filter_delay = u32::from_le_bytes([hdr[24], hdr[25], hdr[26], hdr[27]]);
        let n = u16::from_le_bytes([hdr[28], hdr[29]]) as usize;
        let body = if n > 0 {
            self.read_exact_bytes(n * 4)?
        } else {
            Vec::new()
        };
        let mut entries = Vec::with_capacity(n);
        for i in 0..n {
            let b = i * 4;
            entries.push(u32::from_le_bytes([
                body[b],
                body[b + 1],
                body[b + 2],
                body[b + 3],
            ]));
        }
        Ok(IcSnapshot {
            ref_tick,
            falling_total,
            walked,
            rx_total,
            byte_head,
            bit_ticks,
            cc_filter_delay,
            entries,
        })
    }
}

// ---------------------------------------------------------------------------
// USB autodetect
// ---------------------------------------------------------------------------

/// Find the pirate's USB-CDC port by VID/PID. On macOS the same device
/// shows as both `/dev/cu.*` and `/dev/tty.*`; `/dev/cu.*` is the callout
/// path (non-blocking) which is what we want.
pub fn auto_detect_pirate() -> Result<String> {
    let ports = serialport::available_ports().context("listing serial ports")?;
    let mut hits: Vec<String> = ports
        .into_iter()
        .filter_map(|p| match &p.port_type {
            serialport::SerialPortType::UsbPort(info)
                if info.vid == PIRATE_VID && info.pid == PIRATE_PID =>
            {
                Some(p.port_name)
            }
            _ => None,
        })
        .collect();
    hits.retain(|p| !p.contains("/tty."));
    match hits.len() {
        0 => bail!(
            "no uart-pirate found (VID 0x{PIRATE_VID:04X} PID 0x{PIRATE_PID:04X}); pass --port to override"
        ),
        1 => Ok(hits.into_iter().next().unwrap()),
        n => {
            let list = hits
                .iter()
                .map(|p| format!("  {p}"))
                .collect::<Vec<_>>()
                .join("\n");
            bail!("found {n} pirates; pass --port to pick one:\n{list}");
        }
    }
}

// ---------------------------------------------------------------------------
// Parsing helpers
// ---------------------------------------------------------------------------

fn hex(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 2);
    for b in data {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

fn parse_kv<T: std::str::FromStr>(reply: &str, tag: &str) -> Result<T>
where
    T::Err: std::fmt::Display,
{
    let rest = reply
        .strip_prefix(tag)
        .and_then(|s| s.strip_prefix(' '))
        .ok_or_else(|| anyhow!("{tag}? → {reply:?}"))?;
    rest.parse::<T>()
        .map_err(|e| anyhow!("{tag}? parse {rest:?}: {e}"))
}

fn parse_comp(reply: &str) -> Result<TxComp> {
    let rest = reply
        .strip_prefix("COMP ")
        .ok_or_else(|| anyhow!("COMP reply: {reply:?}"))?;
    let mut pipe: Option<u32> = None;
    let mut bit_q4: Option<u32> = None;
    for tok in rest.split_ascii_whitespace() {
        let (k, v) = tok
            .split_once('=')
            .ok_or_else(|| anyhow!("COMP token: {tok:?}"))?;
        match k {
            "pipe" => pipe = Some(v.parse()?),
            "bit_q4" => bit_q4 = Some(v.parse()?),
            _ => bail!("COMP unknown key: {k:?}"),
        }
    }
    let pipe = pipe.ok_or_else(|| anyhow!("COMP missing pipe"))?;
    let bit_q4 = bit_q4.ok_or_else(|| anyhow!("COMP missing bit_q4"))?;
    Ok(TxComp { pipe, bit_q4 })
}

fn parse_status(reply: &str) -> Result<PirateStatus> {
    let rest = reply
        .strip_prefix("STATUS ")
        .ok_or_else(|| anyhow!("STATUS reply: {reply:?}"))?;
    let parts: Vec<&str> = rest.split_whitespace().collect();
    if parts.len() != 5 {
        bail!("STATUS reply shape: {reply:?}");
    }
    let baud: u32 = parts[0].parse()?;
    let avail: u32 = parts[1].parse()?;
    let desynced: u32 = parts[2].parse()?;
    let cause = if desynced == 0 {
        None
    } else {
        Some(
            DesyncCause::parse(parts[3])
                .ok_or_else(|| anyhow!("STATUS cause token {:?}", parts[3]))?,
        )
    };
    let last_tick: u32 = parts[4].parse()?;
    Ok(PirateStatus {
        baud,
        avail,
        cause,
        last_tick,
    })
}
