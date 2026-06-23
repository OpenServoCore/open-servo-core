//! USB-CDC client for the dxl-pirate.
//!
//! The pirate is the bench's sole bus actor: master TX, listener, and
//! IDLE-stamp ring all in one V203 firmware. See `tools/dxl-pirate/src/proto.rs`
//! for the ASCII grammar this client wraps.

use std::io::{self, Read, Write};
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use dxl_protocol::crc::CrcUmts;
use dxl_protocol::types::Id;
use dxl_protocol::{InstructionEncoder, SoftwareCrcUmts};
use heapless::Vec as HVec;
use serialport::SerialPort;

/// SysTick stamp of a bus-IDLE event, no associated master TX.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Plain {
    pub tick: u32,
    pub head: u32,
}

/// Atomic master-TX + slave-reply round-trip captured by the pirate listener.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Round {
    pub req: u32,
    pub first: u32,
    pub last: u32,
    pub head: u32,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IdleStamp {
    Plain(Plain),
    Round(Round),
}

pub struct PirateClient {
    port: Box<dyn SerialPort>,
    port_path: String,
}

impl PirateClient {
    pub fn port_path(&self) -> &str {
        &self.port_path
    }

    /// Open a pirate over USB-CDC. The nominal 115200 baud is purely cosmetic;
    /// the wire baud is whatever `BAUD <bps>` was last set to.
    ///
    /// DTR is asserted post-open because the pirate firmware blocks at
    /// `CdcAcmClass::wait_connection()` until the host signals connected.
    /// pyserial does this implicitly on every open; serialport-rs doesn't.
    pub fn open(port_path: &str, timeout: Duration) -> Result<Self> {
        let mut port = serialport::new(port_path, 115_200)
            .timeout(timeout)
            .open()
            .with_context(|| format!("opening pirate at {port_path}"))?;
        port.write_data_terminal_ready(true)
            .context("asserting DTR")?;
        port.write_request_to_send(true).context("asserting RTS")?;
        Ok(Self {
            port,
            port_path: port_path.to_string(),
        })
    }

    /// Drain anything the OS has buffered from the pirate. Called before every
    /// command so stale `STAMP`/`ROUND` lines from a prior listen don't poison
    /// the next reply.
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

    fn send(&mut self, line: &str) -> Result<()> {
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

    /// Send one line, return one stripped reply. Use for `OK`/value/`STAMP`/etc.
    /// Don't use for `XFER`/`RX` — their `REPLY <hex>` may span multiple CDC
    /// packets and is concatenated transparently by `read_line`.
    pub fn command(&mut self, line: &str) -> Result<String> {
        self.send(line)?;
        self.read_line()
    }

    pub fn expect_ok(&mut self, line: &str) -> Result<()> {
        let reply = self.command(line)?;
        if reply != "OK" {
            bail!("{line:?} → {reply:?}");
        }
        Ok(())
    }

    pub fn set_baud(&mut self, bps: u32) -> Result<()> {
        self.expect_ok(&format!("BAUD {bps}"))
    }

    pub fn tick(&mut self) -> Result<u64> {
        let reply = self.command("TICK?")?;
        parse_kv(&reply, "TICK")
    }

    pub fn bytes_count(&mut self) -> Result<u32> {
        let reply = self.command("BYTES")?;
        parse_kv::<u32>(&reply, "BYTES")
    }

    pub fn hz_per_us(&mut self) -> Result<u32> {
        let reply = self.command("HZ")?;
        parse_kv::<u32>(&reply, "HZ")
    }

    pub fn master(&mut self, data: &[u8]) -> Result<()> {
        self.expect_ok(&format!("MASTER bytes={}", hex(data)))
    }

    pub fn arm(&mut self, data: &[u8], after_idle_ticks: u32) -> Result<()> {
        self.expect_ok(&format!(
            "ARM bytes={} after_idle={after_idle_ticks}",
            hex(data)
        ))
    }

    pub fn fire(&mut self, data: &[u8], at_tick: u64) -> Result<()> {
        self.expect_ok(&format!("FIRE bytes={} at={at_tick}", hex(data)))
    }

    /// Low-32 SysTick from inject's `FIRED_TICK_LO`. After a FIRE, equals
    /// `at_tick & 0xFFFFFFFF` on the scheduled path, or `now-on-device` if the
    /// TIM4-OPM schedule was missed (immediate-fire fallback).
    pub fn last_fired(&mut self) -> Result<u32> {
        let reply = self.command("LAST?")?;
        parse_kv::<u32>(&reply, "LAST")
    }

    /// Low-32 SysTick from listen's `FIRE_T_FIRST`. RXNE stamp of the first
    /// self-echo byte after a FIRE. Returns 0 if RXNE didn't fire (e.g. bus
    /// disconnected or per-byte ISR missed).
    pub fn last_fire_t_first(&mut self) -> Result<u32> {
        let reply = self.command("FIREFIRST?")?;
        parse_kv::<u32>(&reply, "FIREFIRST")
    }

    /// Fire `data` as master and wait up to `reply_us` for the slave's
    /// end-of-frame IDLE. Returns the slave's reply bytes, or `None` on timeout.
    pub fn xfer(&mut self, data: &[u8], reply_us: u32) -> Result<Option<Vec<u8>>> {
        self.send(&format!("XFER bytes={} reply_us={reply_us}", hex(data)))?;
        let reply = self.read_line()?;
        if reply == "NOREPLY" {
            return Ok(None);
        }
        if let Some(rest) = reply.strip_prefix("REPLY ") {
            return Ok(Some(unhex(rest)?));
        }
        if reply == "REPLY" {
            return Ok(Some(Vec::new()));
        }
        if let Some(err) = reply.strip_prefix("ERR ") {
            bail!("XFER error: {err}");
        }
        bail!("unexpected XFER response: {reply:?}");
    }

    /// Pull `length` raw bytes from the RX DMA ring starting at absolute
    /// byte-count address `from_addr`. Caller stays within the last 256
    /// bytes of the current head.
    pub fn rx_range(&mut self, from_addr: u32, length: u16) -> Result<Vec<u8>> {
        if length == 0 {
            return Ok(Vec::new());
        }
        self.send(&format!("RX from={from_addr} len={length}"))?;
        let reply = self.read_line()?;
        if let Some(rest) = reply.strip_prefix("REPLY ") {
            return unhex(rest);
        }
        if reply == "REPLY" {
            return Ok(Vec::new());
        }
        if let Some(err) = reply.strip_prefix("ERR ") {
            bail!("RX error: {err}");
        }
        bail!("unexpected RX response: {reply:?}");
    }

    pub fn drain_stamps(&mut self) -> Result<Vec<IdleStamp>> {
        let mut out = Vec::new();
        loop {
            let reply = self.command("DRAIN")?;
            if reply == "EMPTY" {
                return Ok(out);
            }
            let parts: Vec<&str> = reply.split_whitespace().collect();
            match parts.as_slice() {
                ["STAMP", tick, head] => {
                    out.push(IdleStamp::Plain(Plain {
                        tick: tick.parse()?,
                        head: head.parse()?,
                    }));
                }
                ["ROUND", req, first, last, head] => {
                    out.push(IdleStamp::Round(Round {
                        req: req.parse()?,
                        first: first.parse()?,
                        last: last.parse()?,
                        head: head.parse()?,
                    }));
                }
                _ => bail!("unexpected DRAIN entry: {reply:?}"),
            }
        }
    }

    /// Block until the bus has been silent (no new RX bytes) for `window`.
    /// Used between probes to absorb stragglers from a previous reply that
    /// arrived after the read window closed.
    pub fn wait_quiet(&mut self, window: Duration) -> Result<()> {
        let deadline = Instant::now() + window * 10;
        let mut prev = self.bytes_count()?;
        let mut silent_since: Option<Instant> = None;
        while Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
            let now = self.bytes_count()?;
            if now != prev {
                prev = now;
                silent_since = None;
            } else if silent_since.is_none() {
                silent_since = Some(Instant::now());
            } else if Instant::now().duration_since(silent_since.unwrap()) >= window {
                return Ok(());
            }
        }
        Ok(())
    }
}

fn hex(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 2);
    for b in data {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

fn unhex(s: &str) -> Result<Vec<u8>> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(Vec::new());
    }
    if !s.len().is_multiple_of(2) {
        bail!("odd-length hex string: {s:?}");
    }
    let mut out = Vec::with_capacity(s.len() / 2);
    for i in (0..s.len()).step_by(2) {
        out.push(u8::from_str_radix(&s[i..i + 2], 16)?);
    }
    Ok(out)
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

// ---------------------------------------------------------------------------
// USB autodetect
// ---------------------------------------------------------------------------

pub const PIRATE_VID: u16 = 0xC0DE;
pub const PIRATE_PID: u16 = 0xCAFE;

/// Find the pirate's USB-CDC port by VID/PID. On macOS the same device shows
/// as both `/dev/cu.*` and `/dev/tty.*`; `/dev/cu.*` is the callout path
/// (non-blocking) and is what we want.
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
            "no dxl-pirate found (VID 0x{PIRATE_VID:04X} PID 0x{PIRATE_PID:04X}); pass --port to override"
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
// DXL encode/decode helpers
// ---------------------------------------------------------------------------

/// `comms::BAUD_RATE_IDX` — see `firmware/lib/core/src/regions/config.rs`.
pub const BAUD_RATE_IDX_ADDR: u16 = 13;

/// Wire bauds supported by the firmware's `BaudRate` enum, in
/// discovery-sweep order: 1M first (default), then most-likely-current.
pub const SUPPORTED_BAUDS: &[u32] = &[1_000_000, 3_000_000, 2_000_000, 115_200, 57_600, 9_600];

/// `BaudRate` enum discriminant for a given wire baud.
pub fn baud_index(bps: u32) -> Result<u8> {
    Ok(match bps {
        9_600 => 0,
        57_600 => 1,
        115_200 => 2,
        1_000_000 => 3,
        2_000_000 => 4,
        3_000_000 => 5,
        _ => bail!("unsupported baud: {bps}"),
    })
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PingInfo {
    pub id: u8,
    pub model: u16,
    pub fw_ver: u8,
    pub error: u8,
}

pub fn build_ping(id: Id) -> Result<HVec<u8, 16>> {
    let mut frame: HVec<u8, 16> = HVec::new();
    InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut frame)
        .ping(id)
        .map_err(|e| anyhow!("encode ping: {e:?}"))?;
    Ok(frame)
}

pub fn build_write(id: Id, addr: u16, data: &[u8]) -> Result<HVec<u8, 64>> {
    let mut frame: HVec<u8, 64> = HVec::new();
    InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut frame)
        .write(id, addr, data)
        .map_err(|e| anyhow!("encode write: {e:?}"))?;
    Ok(frame)
}

/// Hand-decode a Ping status reply. Stuffing inside the 3 param bytes
/// (`model_lo`, `model_hi`, `fw_ver`) requires `model == 0xFFFF` paired with
/// `fw_ver == 0xFD` — physically impossible for any real servo — so we skip
/// the destuffer here and let CRC verification catch any wire corruption.
///
/// Pass `expected_id == None` to accept any id (e.g. broadcast-Ping
/// discovery); the responding servo's id is returned in `PingInfo`.
pub fn decode_ping_status(bytes: &[u8], expected_id: Option<Id>) -> Result<PingInfo> {
    if bytes.len() != 14 {
        bail!(
            "expected 14-byte status reply, got {} bytes: {:02X?}",
            bytes.len(),
            bytes
        );
    }
    if bytes[0..4] != [0xFF, 0xFF, 0xFD, 0x00] {
        bail!("bad header: {:02X?}", &bytes[0..4]);
    }
    if let Some(e) = expected_id
        && bytes[4] != e.as_byte()
    {
        bail!("reply id {} != expected {}", bytes[4], e.as_byte());
    }
    let length = u16::from_le_bytes([bytes[5], bytes[6]]);
    if length != 7 {
        bail!("reply length field {length} != 7 (expected 1 instr + 1 err + 3 params + 2 crc)");
    }
    if bytes[7] != 0x55 {
        bail!("instruction byte 0x{:02X} != 0x55 (Status)", bytes[7]);
    }
    let mut crc = SoftwareCrcUmts::new();
    crc.update(&bytes[..bytes.len() - 2]);
    let got_crc = u16::from_le_bytes([bytes[12], bytes[13]]);
    let want_crc = crc.finalize();
    if want_crc != got_crc {
        bail!("CRC mismatch: computed 0x{want_crc:04X}, got 0x{got_crc:04X}");
    }
    Ok(PingInfo {
        id: bytes[4],
        model: u16::from_le_bytes([bytes[9], bytes[10]]),
        fw_ver: bytes[11],
        error: bytes[8],
    })
}

// ---------------------------------------------------------------------------
// Session prologue
// ---------------------------------------------------------------------------

/// Per DXL 2.0, a servo with id N answering a broadcast Ping waits
/// `N * 14_bytes` of bus time after the request packet end before sending
/// its reply, so multiple servos chain replies without colliding. We don't
/// know the responder's id until we hear from it, so the discovery probe
/// budgets for the max-id worst case at each baud.
fn broadcast_ping_reply_us(bps: u32) -> u32 {
    const MAX_ID: u32 = 253;
    const STATUS_FRAME_BYTES: u32 = 14;
    const SLACK_US: u32 = 5_000;
    let byte_time_us = (10u32 * 1_000_000).div_ceil(bps);
    MAX_ID * STATUS_FRAME_BYTES * byte_time_us + SLACK_US
}

/// Single-target reply window: 14-byte Status at the slowest supported baud
/// (9600) + max RDT (510 µs) + slack. Safe for any unicast Read/Write/Ping
/// at any supported baud.
const UNICAST_REPLY_US: u32 = 50_000;
const BAUD_SETTLE: Duration = Duration::from_millis(50);

#[derive(Clone, Debug)]
pub struct SessionArgs {
    /// Pirate USB-CDC path. `None` → autodetect by VID/PID.
    pub port: Option<String>,
    /// Wire baud to leave the bus at. The servo is switched to this baud if
    /// it's currently at a different one.
    pub target_baud: u32,
}

/// Open pirate + locate servo. After `Session::start` returns, the pirate is
/// running at `baud` and the servo at `id` has been confirmed responsive.
pub struct Session {
    pub pirate: PirateClient,
    pub id: u8,
    pub baud: u32,
}

impl Session {
    pub fn start(args: SessionArgs) -> Result<Self> {
        let port = match args.port {
            Some(p) => p,
            None => auto_detect_pirate()?,
        };
        let mut pirate = PirateClient::open(&port, Duration::from_millis(500))?;
        let target_idx = baud_index(args.target_baud)?;

        // 1. Probe target baud first; if the servo already lives there, done.
        if let Some(id) = probe_baud(&mut pirate, args.target_baud)? {
            return Ok(Self {
                pirate,
                id,
                baud: args.target_baud,
            });
        }

        // 2. Sweep known bauds to find the servo's current home.
        let mut found: Option<(u32, u8)> = None;
        for b in SUPPORTED_BAUDS.iter().copied() {
            if b == args.target_baud {
                continue;
            }
            if let Some(id) = probe_baud(&mut pirate, b)? {
                found = Some((b, id));
                break;
            }
        }
        let (current_baud, id) =
            found.ok_or_else(|| anyhow!("no servo found at any known baud"))?;

        // 3. Tell the servo to switch.
        let frame = build_write(Id::new(id), BAUD_RATE_IDX_ADDR, &[target_idx])?;
        let _ = pirate.xfer(&frame, UNICAST_REPLY_US)?;
        sleep(BAUD_SETTLE);

        // 4. Move pirate to target, reverify. USB-CDC baud reconfig can glitch
        // the pirate TX line into a phantom byte that the chip frames as FE/NE,
        // so the first post-switch ping may legitimately time out — retry a few.
        pirate.set_baud(args.target_baud)?;
        sleep(BAUD_SETTLE);
        let _ = pirate.drain_stamps()?;
        let mut verified = false;
        for _ in 0..3 {
            if probe_unicast(&mut pirate, Id::new(id))?.is_some() {
                verified = true;
                break;
            }
            sleep(Duration::from_millis(20));
        }
        if !verified {
            bail!(
                "servo at id {id} stopped replying after baud switch {current_baud} -> {}",
                args.target_baud,
            );
        }

        Ok(Self {
            pirate,
            id,
            baud: args.target_baud,
        })
    }
}

/// Set pirate baud, drain stamps, broadcast-ping. Returns the responding id
/// (if any) or `None` on timeout.
fn probe_baud(pirate: &mut PirateClient, bps: u32) -> Result<Option<u8>> {
    pirate.set_baud(bps)?;
    sleep(BAUD_SETTLE);
    let _ = pirate.drain_stamps()?;
    let frame = build_ping(Id::BROADCAST)?;
    match pirate.xfer(&frame, broadcast_ping_reply_us(bps))? {
        None => Ok(None),
        Some(bytes) => Ok(Some(decode_ping_status(&bytes, None)?.id)),
    }
}

fn probe_unicast(pirate: &mut PirateClient, id: Id) -> Result<Option<u8>> {
    let frame = build_ping(id)?;
    match pirate.xfer(&frame, UNICAST_REPLY_US)? {
        None => Ok(None),
        Some(bytes) => Ok(Some(decode_ping_status(&bytes, Some(id))?.id)),
    }
}
