//! USB-CDC client for the dxl-pirate.
//!
//! The pirate is the bench's sole bus actor: master TX, listener, and
//! IDLE-stamp ring all in one V203 firmware. See `tools/dxl-pirate/src/proto.rs`
//! for the ASCII grammar this client wraps.

use std::io::{self, Read, Write};
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use dxl_protocol::streaming::{CrcResult, Event, HeaderEvent, Parser, PayloadEvent, StatusPayload};
use dxl_protocol::types::{BulkReadEntry, Id, Slot, SlotPosition, StatusError};
use dxl_protocol::{InstructionEncoder, SlotEncoder, SoftwareCrcUmts};
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

/// A complete, CRC-verified Status reply, decoded via
/// [`dxl_protocol::streaming::Parser`]. `data` is the assembled payload
/// bytes — for Ping replies that's `[model_lo, model_hi, fw_version]`; for
/// Read replies it's the requested register window.
#[derive(Debug, Clone)]
pub struct StatusReply {
    pub id: Id,
    pub error: StatusError,
    pub data: Vec<u8>,
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

pub fn build_read(id: Id, addr: u16, length: u16) -> Result<HVec<u8, 16>> {
    let mut frame: HVec<u8, 16> = HVec::new();
    InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut frame)
        .read(id, addr, length)
        .map_err(|e| anyhow!("encode read: {e:?}"))?;
    Ok(frame)
}

pub fn build_reboot(id: Id) -> Result<HVec<u8, 16>> {
    let mut frame: HVec<u8, 16> = HVec::new();
    InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut frame)
        .reboot(id)
        .map_err(|e| anyhow!("encode reboot: {e:?}"))?;
    Ok(frame)
}

/// Decode a Status reply by running the dxl-protocol streaming parser over
/// it. Returns the verified header + accumulated payload; only `Status`
/// frames are accepted (the parser otherwise routes Instruction frames
/// through a separate event branch).
///
/// Pass `expected_id == None` to accept any id (e.g. broadcast-Ping
/// discovery).
pub fn parse_status_reply(bytes: &[u8], expected_id: Option<Id>) -> Result<StatusReply> {
    let mut parser = Parser::<SoftwareCrcUmts>::new();
    let mut header_id: Option<Id> = None;
    let mut header_err = StatusError::OK;
    let mut data: Vec<u8> = Vec::new();
    let mut done = false;
    for evt in parser.feed(bytes) {
        if done {
            bail!("trailing bytes after Status reply CRC: {bytes:02X?}");
        }
        match evt {
            Event::Sync => {}
            Event::Header(HeaderEvent::Status(h)) => {
                header_id = Some(h.id);
                header_err = h.error;
            }
            Event::Header(HeaderEvent::Instruction(_)) => {
                bail!("expected Status reply, got Instruction frame: {bytes:02X?}");
            }
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) => {
                let lo = offset as usize;
                let hi = lo + length as usize;
                data.extend_from_slice(&bytes[lo..hi]);
            }
            // The streaming parser does not distinguish Ping replies from Read
            // replies at the payload level — both arrive as ReadDataChunk(s).
            // Other StatusPayload variants are reserved for the Fast-Read
            // decoder (see parse_fast_response in B3).
            Event::Payload(_) => {
                bail!("unexpected payload event in Status reply: {bytes:02X?}");
            }
            Event::Crc(CrcResult::Good) => done = true,
            Event::Crc(CrcResult::Bad) => bail!("bad CRC on Status reply: {bytes:02X?}"),
            Event::Resync(kind) => bail!("parser resync ({kind:?}) on: {bytes:02X?}"),
        }
    }
    if !done {
        bail!(
            "incomplete Status reply ({} bytes, no terminal CRC event): {bytes:02X?}",
            bytes.len()
        );
    }
    let id = header_id.ok_or_else(|| anyhow!("no Status header in reply: {bytes:02X?}"))?;
    if let Some(e) = expected_id
        && id.as_byte() != e.as_byte()
    {
        bail!("reply id {} != expected {}", id.as_byte(), e.as_byte());
    }
    Ok(StatusReply {
        id,
        error: header_err,
        data,
    })
}

/// Convenience wrapper for Ping replies. The 3 payload bytes are
/// `[model_lo, model_hi, fw_version]`.
pub fn decode_ping_status(bytes: &[u8], expected_id: Option<Id>) -> Result<PingInfo> {
    let reply = parse_status_reply(bytes, expected_id)?;
    if reply.data.len() != 3 {
        bail!(
            "Ping reply has {} payload bytes, expected 3: {bytes:02X?}",
            reply.data.len()
        );
    }
    Ok(PingInfo {
        id: reply.id.as_byte(),
        model: u16::from_le_bytes([reply.data[0], reply.data[1]]),
        fw_ver: reply.data[2],
        error: reply.error.as_byte(),
    })
}

// ---------------------------------------------------------------------------
// Fast-stress helpers (chain emit + counter probes)
// ---------------------------------------------------------------------------

/// `comms::RETURN_DELAY_2US` — see `firmware/lib/core/src/regions/config.rs`.
/// One unit = 2 µs of slave-side reply delay.
pub const RETURN_DELAY_2US_ADDR: u16 = 14;

/// High-id slot stand-in used by the `--position first` cell: chip emits its
/// First-slot bytes then the foreign id never replies, so the chain CRC
/// patch deadline misses by design. 199 is well outside the usual low-id
/// range a real chip would self-assign.
pub const FOREIGN_ID: u8 = 199;

/// Predecessor slot id for `--position last`: pirate ARMs a synthetic INJ
/// reply so the chip's snoop walk + chain CRC patch fire as in production.
pub const INJ_ID: u8 = 50;

/// Base address of the chip's `TelemetryDxlLink` fault-counter block —
/// 10 × u32, RW. Host writes zero to clear; chip-side `report_fault`
/// increments via raw pointer.
pub const LINK_BASE: u16 = 0x023C;

/// Snapshot of the chip's `TelemetryDxlLink` counter block. Field order
/// matches `firmware/lib/core/src/regions/telemetry.rs::TelemetryDxlLink`
/// — keep them in lockstep.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct LinkCounters {
    pub illegal_transition: u32,
    pub unexpected_byte_count: u32,
    pub previous_slot_timeout: u32,
    pub slot_timing_miss: u32,
    pub crc_patch_deadline_miss: u32,
    pub edge_anchor_miss: u32,
    pub dma_overrun: u32,
    pub parity_error: u32,
    pub framing_error: u32,
    pub noise_error: u32,
}

impl LinkCounters {
    pub const FIELDS: &'static [&'static str] = &[
        "illegal_transition",
        "unexpected_byte_count",
        "previous_slot_timeout",
        "slot_timing_miss",
        "crc_patch_deadline_miss",
        "edge_anchor_miss",
        "dma_overrun",
        "parity_error",
        "framing_error",
        "noise_error",
    ];
    pub const LEN: usize = 4 * Self::FIELDS.len();

    pub fn as_slice(&self) -> [u32; 10] {
        [
            self.illegal_transition,
            self.unexpected_byte_count,
            self.previous_slot_timeout,
            self.slot_timing_miss,
            self.crc_patch_deadline_miss,
            self.edge_anchor_miss,
            self.dma_overrun,
            self.parity_error,
            self.framing_error,
            self.noise_error,
        ]
    }

    /// Non-zero per-field deltas vs `prev`, in field order. Wraps via u32
    /// subtraction — counters are monotonic between clears so a non-wrap
    /// delta is always >= 0 in practice.
    pub fn delta_from(&self, prev: &Self) -> Vec<(&'static str, u32)> {
        let curr = self.as_slice();
        let prev = prev.as_slice();
        Self::FIELDS
            .iter()
            .zip(curr.iter().zip(prev.iter()))
            .filter_map(|(name, (c, p))| {
                let d = c.wrapping_sub(*p);
                (d != 0).then_some((*name, d))
            })
            .collect()
    }

    fn from_le_bytes(buf: &[u8]) -> Result<Self> {
        if buf.len() != Self::LEN {
            bail!(
                "LinkCounters expects {} bytes, got {}",
                Self::LEN,
                buf.len()
            );
        }
        let mut words = [0u32; 10];
        for (i, w) in words.iter_mut().enumerate() {
            let off = i * 4;
            *w = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
        }
        Ok(Self {
            illegal_transition: words[0],
            unexpected_byte_count: words[1],
            previous_slot_timeout: words[2],
            slot_timing_miss: words[3],
            crc_patch_deadline_miss: words[4],
            edge_anchor_miss: words[5],
            dma_overrun: words[6],
            parity_error: words[7],
            framing_error: words[8],
            noise_error: words[9],
        })
    }
}

/// Snapshot the chip's fault counters via a single Read at `LINK_BASE`.
pub fn read_counters(pirate: &mut PirateClient, id: Id) -> Result<LinkCounters> {
    let frame = build_read(id, LINK_BASE, LinkCounters::LEN as u16)?;
    let reply = pirate
        .xfer(&frame, UNICAST_REPLY_US)?
        .ok_or_else(|| anyhow!("counter read: no reply"))?;
    let decoded = parse_status_reply(&reply, Some(id))?;
    if decoded.error.as_byte() != 0 {
        bail!("counter read error byte 0x{:02X}", decoded.error.as_byte());
    }
    LinkCounters::from_le_bytes(&decoded.data)
}

/// Zero the chip's fault counters.
pub fn clear_counters(pirate: &mut PirateClient, id: Id) -> Result<()> {
    let zeros = [0u8; LinkCounters::LEN];
    let frame = build_write(id, LINK_BASE, &zeros)?;
    let reply = pirate
        .xfer(&frame, UNICAST_REPLY_US)?
        .ok_or_else(|| anyhow!("counter clear: no reply"))?;
    let decoded = parse_status_reply(&reply, Some(id))?;
    if decoded.error.as_byte() != 0 {
        bail!("counter clear error byte 0x{:02X}", decoded.error.as_byte());
    }
    Ok(())
}

/// Snapshot counters, swallowing transport / parse failures into `None`.
/// Used as a wedge probe — chip silent or replying garbage both signal a
/// wedged transport.
pub fn try_read_counters(pirate: &mut PirateClient, id: Id) -> Option<LinkCounters> {
    read_counters(pirate, id).ok()
}

/// Read a single u8 from the chip's control table.
pub fn read_ct_u8(pirate: &mut PirateClient, id: Id, addr: u16) -> Result<u8> {
    let frame = build_read(id, addr, 1)?;
    let reply = pirate
        .xfer(&frame, UNICAST_REPLY_US)?
        .ok_or_else(|| anyhow!("read_ct_u8: no reply"))?;
    let decoded = parse_status_reply(&reply, Some(id))?;
    if decoded.error.as_byte() != 0 {
        bail!(
            "read_ct_u8 error byte 0x{:02X} at addr 0x{:04X}",
            decoded.error.as_byte(),
            addr
        );
    }
    if decoded.data.len() != 1 {
        bail!(
            "read_ct_u8 returned {} bytes at addr 0x{:04X}",
            decoded.data.len(),
            addr
        );
    }
    Ok(decoded.data[0])
}

/// Encode a Fast Bulk Read instruction frame via [`InstructionEncoder`].
pub fn build_fast_bulk_read(entries: &[BulkReadEntry]) -> Result<HVec<u8, 64>> {
    let mut frame: HVec<u8, 64> = HVec::new();
    InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut frame)
        .fast_bulk_read(entries)
        .map_err(|e| anyhow!("encode fast_bulk_read: {e:?}"))?;
    Ok(frame)
}

/// Build the on-wire bytes for a synthetic First-slot reply, used to ARM
/// a predecessor in `--position last`. The chip's snoop walker treats these
/// bytes as the prior slot's status frame and computes its own slot's chain
/// CRC against them.
pub fn build_inj_first_bytes(
    slot_id: Id,
    error: StatusError,
    data: &[u8],
    packet_length: u16,
) -> Result<HVec<u8, 64>> {
    let mut frame: HVec<u8, 64> = HVec::new();
    let slot = Slot {
        id: slot_id,
        error,
        data,
    };
    SlotEncoder::<_, SoftwareCrcUmts>::new(&mut frame)
        .emit(&slot, SlotPosition::First { packet_length })
        .map_err(|e| anyhow!("encode inj first slot: {e:?}"))?;
    Ok(frame)
}

/// One parsed slot from a Fast Sync/Bulk Read reply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FastSlot {
    pub id: Id,
    pub error: StatusError,
    pub data: Vec<u8>,
}

/// Decode a coalesced Fast Sync/Bulk Read reply. `slot_lengths` is the
/// per-slot payload length list in chain order. Driven by the dxl-protocol
/// streaming parser like [`parse_status_reply`], but pulls slot
/// metadata + payload bytes from the Fast-Read event taxonomy.
///
/// NOTE: the current streaming parser emits `ReadDataChunk` events for the
/// concatenated body of a Fast reply (the body framing collapses to one
/// payload stream). We slice the body ourselves using `slot_lengths` — the
/// chip's wire output is `(err, id, data..)` per slot in order, with the
/// first slot's `err` lifted to the Status header.
pub fn parse_fast_response(bytes: &[u8], slot_lengths: &[usize]) -> Result<Vec<FastSlot>> {
    let decoded = parse_status_reply(bytes, None)?;
    // First slot's error byte comes from the header; subsequent slots carry
    // their own (err, id, data) prefix in the streamed body.
    let body = &decoded.data;
    let mut slots = Vec::with_capacity(slot_lengths.len());
    let mut cursor = 0usize;
    for (idx, &slot_len) in slot_lengths.iter().enumerate() {
        if idx == 0 {
            // Slot 0: header carries the error byte; body starts with id + data.
            if body.len() < cursor + 1 + slot_len {
                bail!("fast reply too short for slot 0 (len {slot_len}): {bytes:02X?}");
            }
            let id_byte = body[cursor];
            cursor += 1;
            let data = body[cursor..cursor + slot_len].to_vec();
            cursor += slot_len;
            slots.push(FastSlot {
                id: Id::new(id_byte),
                error: decoded.error,
                data,
            });
        } else {
            if body.len() < cursor + 2 + slot_len {
                bail!("fast reply too short for slot {idx} (len {slot_len}): {bytes:02X?}");
            }
            let err_byte = body[cursor];
            let id_byte = body[cursor + 1];
            cursor += 2;
            let data = body[cursor..cursor + slot_len].to_vec();
            cursor += slot_len;
            slots.push(FastSlot {
                id: Id::new(id_byte),
                error: StatusError::from_byte(err_byte),
                data,
            });
        }
    }
    if cursor != body.len() {
        bail!(
            "fast reply has {} trailing body bytes after {} slots: {bytes:02X?}",
            body.len() - cursor,
            slot_lengths.len()
        );
    }
    Ok(slots)
}

/// Pull the lone `Round` stamp emitted by an XFER, if present.
pub fn round_from_stamps(stamps: &[IdleStamp]) -> Option<Round> {
    stamps.iter().find_map(|s| match s {
        IdleStamp::Round(r) => Some(*r),
        IdleStamp::Plain(_) => None,
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
pub const UNICAST_REPLY_US: u32 = 50_000;
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
        let (_current_baud, id) =
            found.ok_or_else(|| anyhow!("no servo found at any known baud"))?;

        // 3. Switch to the target.
        set_chip_baud(&mut pirate, id, args.target_baud)?;

        Ok(Self {
            pirate,
            id,
            baud: args.target_baud,
        })
    }
}

/// Default baud the chip boots at — mirrors `BaudRate::B1000000` (default) in
/// `firmware/lib/core/src/regions/config.rs`.
pub const BOOT_BAUD: u32 = 1_000_000;

/// Write `BAUD_RATE_IDX` on the chip, then move the pirate to match. USB-CDC
/// baud reconfig can glitch the pirate TX line into a phantom byte that the
/// chip's USART frames as FE/NE — the first post-switch ping may legitimately
/// time out, so retry a few times before bailing.
pub fn set_chip_baud(pirate: &mut PirateClient, id: u8, target_baud: u32) -> Result<()> {
    let target_idx = baud_index(target_baud)?;
    let frame = build_write(Id::new(id), BAUD_RATE_IDX_ADDR, &[target_idx])?;
    let _ = pirate.xfer(&frame, UNICAST_REPLY_US)?;
    sleep(BAUD_SETTLE);
    pirate.set_baud(target_baud)?;
    sleep(BAUD_SETTLE);
    let _ = pirate.drain_stamps()?;
    for _ in 0..3 {
        if probe_unicast(pirate, Id::new(id))?.is_some() {
            return Ok(());
        }
        sleep(Duration::from_millis(20));
    }
    bail!("servo {id} stopped replying after baud switch to {target_baud}");
}

/// Reboot the chip and wait for it to come back at `BOOT_BAUD`. The reboot
/// ACK can race the reset, so a missing reply is non-fatal. The pirate is
/// left at `BOOT_BAUD` with stamps drained.
pub fn reboot_chip(pirate: &mut PirateClient, id: u8) -> Result<()> {
    let frame = build_reboot(Id::new(id))?;
    let _ = pirate.xfer(&frame, UNICAST_REPLY_US)?;
    sleep(Duration::from_millis(500));
    pirate.set_baud(BOOT_BAUD)?;
    let _ = pirate.drain_stamps()?;
    Ok(())
}

// ---------------------------------------------------------------------------
// HSI drift probe
// ---------------------------------------------------------------------------

pub mod drift {
    //! Host-side HSI drift probe.
    //!
    //! The chip owns its clock trim — it watches inter-byte timing on every
    //! non-Status packet and nudges HSITRIM toward zero drift autonomously
    //! (see `firmware/lib/drivers/src/dxl/uart/clock.rs`). There is no CAL
    //! round-trip and no readable trim register: the only signal the host
    //! can observe is how long the chip takes to transmit a reply, measured
    //! against the pirate's stable clock.
    //!
    //! [`probe_drift_ppm`] issues long Reads and reads the pirate's `Round`
    //! stamp for the reply: `first`/`last` are the per-byte RXNE timestamps
    //! of the first and last reply bytes. Their span covers `(N-1)`
    //! chip-transmitted byte-times; HSI fast → short span, HSI slow → long
    //! span.

    use anyhow::{Result, anyhow, bail};
    use dxl_protocol::types::Id;

    use super::{IdleStamp, PirateClient, build_read, parse_status_reply};

    /// HSITRIM step size mapped to ppm. Mirrors
    /// `firmware/ch32/src/hal/rcc/v00x.rs::CLOCK_TRIM_PPM_PER_STEP`
    /// (HSI_TRIM_STEP_HZ 60 kHz / HSI_HZ 24 MHz). One step ≈ 2500 ppm.
    pub const DRIFT_STEP_PPM: f64 = 2500.0;

    /// Probe the chip's config region — always present, RO, and long enough
    /// that the per-byte RXNE quantization stays small vs the reply span.
    const PROBE_ADDR: u16 = 0;
    const PROBE_LEN: u16 = 128;

    /// Full Status frame around an N-byte payload: 4 header + id + 2 len +
    /// instr + err + N params + 2 CRC.
    const STATUS_OVERHEAD: usize = 11;

    /// Generous Read window — at the slowest supported baud (9600), a
    /// 139-byte Status frame takes ~145 ms on the wire alone.
    const READ_REPLY_US: u32 = 200_000;

    /// Mean chip-TX drift in ppm over `samples` long-Read probes.
    /// Positive ⇒ chip TX slow (HSI slow); negative ⇒ fast.
    pub fn probe_drift_ppm(
        pirate: &mut PirateClient,
        id: Id,
        baud: u32,
        samples: u32,
    ) -> Result<f64> {
        const RETRIES: u32 = 3;
        let n = STATUS_OVERHEAD + PROBE_LEN as usize;
        let ticks_per_us = pirate.hz_per_us()? as f64;
        // (n - 1) byte-times of span, each 10 bit periods, in pirate ticks.
        let nominal_ticks = (n as f64 - 1.0) * 10.0 / baud as f64 * ticks_per_us * 1_000_000.0;

        let mut spans: Vec<u32> = Vec::with_capacity(samples as usize);
        for _ in 0..samples {
            let mut last_err: Option<anyhow::Error> = None;
            for _ in 0..RETRIES {
                match one_span(pirate, id) {
                    Ok(span) => {
                        spans.push(span);
                        last_err = None;
                        break;
                    }
                    Err(e) => last_err = Some(e),
                }
            }
            if let Some(e) = last_err {
                return Err(e);
            }
        }
        let mean_span: f64 = spans.iter().map(|&s| s as f64).sum::<f64>() / spans.len() as f64;
        Ok((mean_span / nominal_ticks - 1.0) * 1_000_000.0)
    }

    fn one_span(pirate: &mut PirateClient, id: Id) -> Result<u32> {
        let _ = pirate.drain_stamps()?;
        let frame = build_read(id, PROBE_ADDR, PROBE_LEN)?;
        let reply = pirate
            .xfer(&frame, READ_REPLY_US)?
            .ok_or_else(|| anyhow!("no Status reply on drift Read"))?;
        let decoded = parse_status_reply(&reply, Some(id))?;
        if decoded.error.as_byte() != 0 {
            bail!(
                "drift Read returned error byte 0x{:02X}",
                decoded.error.as_byte()
            );
        }
        if decoded.data.len() != PROBE_LEN as usize {
            bail!(
                "drift Read returned {} data bytes, expected {}",
                decoded.data.len(),
                PROBE_LEN
            );
        }
        let stamps = pirate.drain_stamps()?;
        let rounds: Vec<_> = stamps
            .iter()
            .filter_map(|s| match s {
                IdleStamp::Round(r) => Some(*r),
                IdleStamp::Plain(_) => None,
            })
            .collect();
        if rounds.len() != 1 {
            bail!("drift Read produced {} Round stamps", rounds.len());
        }
        Ok(rounds[0].last.wrapping_sub(rounds[0].first))
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
