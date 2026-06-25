//! Bench-side facade over the dxl-pirate. Bins drive [`Bus`]; the raw
//! USB-CDC client (`pirate::Client`) is private to this crate so bins
//! never type a wire command directly. See `tools/dxl-pirate/src/proto.rs`
//! for the underlying grammar.

use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow, bail};
use dxl_protocol::streaming::{CrcResult, Event, HeaderEvent, Parser, PayloadEvent, StatusPayload};
use dxl_protocol::types::{BulkReadEntry, Id, Slot, SlotPosition, StatusError};
use dxl_protocol::{InstructionEncoder, SlotEncoder, SoftwareCrcUmts};
use heapless::Vec as HVec;

mod pirate;

pub use pirate::{
    BStamp, DesyncCause, IcSnapshot, PIRATE_PID, PIRATE_VID, PirateDesync, PirateStatus,
    ReplyCapture, ReplyTiming, auto_detect_pirate, lift_against,
};

// ---------------------------------------------------------------------------
// Constants — wire bauds + DXL register addresses
// ---------------------------------------------------------------------------

/// `comms::BAUD_RATE_IDX` — see `firmware/lib/core/src/regions/config.rs`.
pub const BAUD_RATE_IDX_ADDR: u16 = 13;

/// `comms::RETURN_DELAY_2US` — see `firmware/lib/core/src/regions/config.rs`.
/// One unit = 2 µs of slave-side reply delay.
pub const RETURN_DELAY_2US_ADDR: u16 = 14;

/// Wire bauds supported by the firmware's `BaudRate` enum, in
/// discovery-sweep order: 1M first (default), then most-likely-current.
pub const SUPPORTED_BAUDS: &[u32] = &[1_000_000, 3_000_000, 2_000_000, 115_200, 57_600, 9_600];

/// Default baud the chip boots at — mirrors `BaudRate::B1000000` in
/// `firmware/lib/core/src/regions/config.rs`.
pub const BOOT_BAUD: u32 = 1_000_000;

/// Default bus-silence window for `Bus::xfer` and friends.
///
/// Each non-empty `bbatch` poll resets the silence timer; the helper
/// returns when no new bytes have arrived for `DEFAULT_IDLE_US` of host
/// wall-clock. There is no total-elapsed cap inside the helper — the same
/// constant governs unicast (one reply burst), broadcast (254 servos
/// chaining replies), and `arm_then_master` (predecessor-then-chain) calls.
///
/// Sized above two hardware floors that combine to lag the host's
/// observed `last_byte_time` behind the wire:
///
/// - Pirate walker cadence ~114 µs (§3.2 `tools/dxl-pirate/TIMING.md`):
///   a byte received on the wire is stamped at the next TIM2 quarter-wrap
///   walker tick.
/// - USB-CDC `bbatch` RTT ~1 ms per poll.
///
/// At 5 ms the helper waits ~3.9 ms of actual wire silence after the
/// last byte, which is >> any DXL inter-byte gap up through 3 Mbaud and
/// >> the inter-servo gap in a broadcast Ping chain.
pub const DEFAULT_IDLE_US: u32 = 10_000;

/// Backstop on total wall-clock spent in a single xfer. The idle-window
/// exit handles every normal case (responsive, unresponsive, broadcast);
/// this only catches a malfunctioning pirate that never goes quiet.
const MAX_TOTAL_WAIT: Duration = Duration::from_secs(5);

/// Pause between consecutive *empty* `bbatch` polls. Avoids hammering
/// the pirate's USB-CDC stack at the bare USB-RTT rate (~1 ms/poll =
/// 1000 polls/s), which has been observed to wedge the device on
/// sustained traffic. Non-empty polls return immediately — only the
/// idle-wait branch waits.
const EMPTY_POLL_BACKOFF: Duration = Duration::from_millis(1);

const BAUD_SETTLE: Duration = Duration::from_millis(50);

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

// ---------------------------------------------------------------------------
// DXL frame encoders + Status decoder (unchanged)
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PingInfo {
    pub id: u8,
    pub model: u16,
    pub fw_ver: u8,
    pub error: u8,
}

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

pub fn build_fast_bulk_read(entries: &[BulkReadEntry]) -> Result<HVec<u8, 64>> {
    let mut frame: HVec<u8, 64> = HVec::new();
    InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut frame)
        .fast_bulk_read(entries)
        .map_err(|e| anyhow!("encode fast_bulk_read: {e:?}"))?;
    Ok(frame)
}

/// Encode the synthetic First-slot bytes used by `--position last`. The
/// chip's snoop walker treats these as the prior slot's status frame and
/// computes its own chain CRC against them.
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

/// Decode a Status reply via the dxl-protocol streaming parser. Pass
/// `expected_id == None` to accept any id (e.g. broadcast-Ping discovery).
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
// Fast-stress chain helpers
// ---------------------------------------------------------------------------

pub const FOREIGN_ID: u8 = 199;
pub const INJ_ID: u8 = 50;
pub const LINK_BASE: u16 = 0x023C;
pub const TUNE_BASE: u16 = LINK_BASE + LinkCounters::LEN as u16;

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

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TuneStamps {
    pub tx_start_entry_min: u16,
    pub fast_last_entry_min: u16,
    pub schedule_remaining_max: u16,
    pub _rsvd_align: u16,
}

impl TuneStamps {
    pub const LEN: usize = 8;

    fn from_le_bytes(buf: &[u8]) -> Result<Self> {
        if buf.len() != Self::LEN {
            bail!("TuneStamps expects {} bytes, got {}", Self::LEN, buf.len());
        }
        Ok(Self {
            tx_start_entry_min: u16::from_le_bytes([buf[0], buf[1]]),
            fast_last_entry_min: u16::from_le_bytes([buf[2], buf[3]]),
            schedule_remaining_max: u16::from_le_bytes([buf[4], buf[5]]),
            _rsvd_align: u16::from_le_bytes([buf[6], buf[7]]),
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FastSlot {
    pub id: Id,
    pub error: StatusError,
    pub data: Vec<u8>,
}

pub fn parse_fast_response(bytes: &[u8], slot_lengths: &[usize]) -> Result<Vec<FastSlot>> {
    let decoded = parse_status_reply(bytes, None)?;
    let body = &decoded.data;
    let mut slots = Vec::with_capacity(slot_lengths.len());
    let mut cursor = 0usize;
    for (idx, &slot_len) in slot_lengths.iter().enumerate() {
        if idx == 0 {
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

// ---------------------------------------------------------------------------
// Bus — bin-facing facade
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct BusArgs {
    /// Pirate USB-CDC path. `None` → autodetect by VID/PID.
    pub port: Option<String>,
    /// Wire baud to leave the bus at. The servo is switched to this baud
    /// if it's currently at a different one.
    pub target_baud: u32,
}

pub struct Bus {
    pirate: pirate::Client,
    id: u8,
    baud: u32,
}

impl Bus {
    /// Open pirate + locate servo. After `start` returns, the pirate is
    /// at `target_baud` and `id` has been confirmed responsive.
    pub fn start(args: BusArgs) -> Result<Self> {
        let port = match args.port {
            Some(p) => p,
            None => auto_detect_pirate()?,
        };
        let mut client = pirate::Client::open(&port, Duration::from_millis(500))?;

        if let Some(id) = probe_baud(&mut client, args.target_baud)? {
            return Ok(Self {
                pirate: client,
                id,
                baud: args.target_baud,
            });
        }

        let mut found: Option<(u32, u8)> = None;
        for b in SUPPORTED_BAUDS.iter().copied() {
            if b == args.target_baud {
                continue;
            }
            if let Some(id) = probe_baud(&mut client, b)? {
                found = Some((b, id));
                break;
            }
        }
        let (_current_baud, id) =
            found.ok_or_else(|| anyhow!("no servo found at any known baud"))?;

        set_chip_baud_inner(&mut client, id, args.target_baud)?;
        Ok(Self {
            pirate: client,
            id,
            baud: args.target_baud,
        })
    }

    /// Open the pirate without probing for a servo. For selftest bins that
    /// validate the pirate against its own TX-echo loopback. `id()` returns
    /// `0` as a sentinel; `baud()` reflects whatever the pirate is currently
    /// at (callers typically follow with `pirate_set_baud`).
    pub fn start_pirate_only(port: Option<String>) -> Result<Self> {
        let port = match port {
            Some(p) => p,
            None => auto_detect_pirate()?,
        };
        let client = pirate::Client::open(&port, Duration::from_millis(500))?;
        let baud = client.current_baud();
        Ok(Self {
            pirate: client,
            id: 0,
            baud,
        })
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    pub fn baud(&self) -> u32 {
        self.baud
    }

    pub fn port_path(&self) -> &str {
        self.pirate.port_path()
    }

    pub fn hz_per_us(&mut self) -> Result<u32> {
        self.pirate.hz_per_us()
    }

    // -----------------------------------------------------------------------
    // Transport (master/arm/fire → drain stamps)
    // -----------------------------------------------------------------------

    /// Fire `req` as bus master, drain stamps until the bus has been
    /// silent for `idle_us` of host wall-clock, return the full capture.
    /// Each non-empty `bbatch` poll resets the silence timer, so the
    /// helper naturally accommodates long replies (broadcast Ping, fast
    /// Bulk Read chains) without a per-call timeout knob.
    /// See [`DEFAULT_IDLE_US`] for sizing guidance.
    pub fn xfer(&mut self, req: &[u8], idle_us: u32) -> Result<ReplyCapture> {
        xfer_inner(&mut self.pirate, req, idle_us)
    }

    /// `xfer` + slice off the master TX echo. `None` if no reply arrived.
    pub fn xfer_reply(&mut self, req: &[u8], idle_us: u32) -> Result<Option<Vec<u8>>> {
        let cap = self.xfer(req, idle_us)?;
        if cap.timing.is_none() {
            return Ok(None);
        }
        let reply_bytes: Vec<u8> = cap.stamps.iter().skip(req.len()).map(|s| s.byte).collect();
        Ok(Some(reply_bytes))
    }

    /// `xfer` returning just the three-point timing (`None` if no reply).
    pub fn xfer_round(&mut self, req: &[u8], idle_us: u32) -> Result<Option<ReplyTiming>> {
        Ok(self.xfer(req, idle_us)?.timing)
    }

    /// Stage `inject` to fire after the next IDLE, then send `req`. Used
    /// to emulate a predecessor slot's reply in chain timing tests.
    pub fn arm_then_master(
        &mut self,
        inject: &[u8],
        after_idle_us: u32,
        req: &[u8],
        idle_us: u32,
    ) -> Result<ReplyCapture> {
        drain_all(&mut self.pirate)?;
        self.pirate.arm(inject, after_idle_us)?;
        self.pirate.master(req)?;
        collect_until_silent(&mut self.pirate, req, idle_us)
    }

    // -----------------------------------------------------------------------
    // Register-level convenience
    // -----------------------------------------------------------------------

    pub fn read_register(&mut self, id: Id, addr: u16, len: u16) -> Result<Vec<u8>> {
        let frame = build_read(id, addr, len)?;
        let reply = self
            .xfer_reply(&frame, DEFAULT_IDLE_US)?
            .ok_or_else(|| anyhow!("read {addr:#06X}: no reply"))?;
        let decoded = parse_status_reply(&reply, Some(id))?;
        if decoded.error.as_byte() != 0 {
            bail!(
                "read {addr:#06X} error byte 0x{:02X}",
                decoded.error.as_byte()
            );
        }
        Ok(decoded.data)
    }

    pub fn write_register(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<()> {
        let frame = build_write(id, addr, data)?;
        let reply = self
            .xfer_reply(&frame, DEFAULT_IDLE_US)?
            .ok_or_else(|| anyhow!("write {addr:#06X}: no reply"))?;
        let decoded = parse_status_reply(&reply, Some(id))?;
        if decoded.error.as_byte() != 0 {
            bail!(
                "write {addr:#06X} error byte 0x{:02X}",
                decoded.error.as_byte()
            );
        }
        Ok(())
    }

    pub fn read_counters(&mut self, id: Id) -> Result<LinkCounters> {
        let data = self.read_register(id, LINK_BASE, LinkCounters::LEN as u16)?;
        LinkCounters::from_le_bytes(&data)
    }

    pub fn clear_counters(&mut self, id: Id) -> Result<()> {
        let zeros = [0u8; LinkCounters::LEN];
        self.write_register(id, LINK_BASE, &zeros)
    }

    /// Counter read that swallows transport / parse failures into `None`
    /// — used as a wedge probe (silent chip and garbled reply both signal
    /// a wedged transport).
    pub fn try_read_counters(&mut self, id: Id) -> Option<LinkCounters> {
        self.read_counters(id).ok()
    }

    pub fn read_tune(&mut self, id: Id) -> Result<TuneStamps> {
        let data = self.read_register(id, TUNE_BASE, TuneStamps::LEN as u16)?;
        TuneStamps::from_le_bytes(&data)
    }

    pub fn clear_tune(&mut self, id: Id) -> Result<()> {
        let zeros = [0u8; TuneStamps::LEN];
        self.write_register(id, TUNE_BASE, &zeros)
    }

    pub fn read_ct_u8(&mut self, id: Id, addr: u16) -> Result<u8> {
        let data = self.read_register(id, addr, 1)?;
        if data.len() != 1 {
            bail!(
                "read_ct_u8 returned {} bytes at addr {addr:#06X}",
                data.len()
            );
        }
        Ok(data[0])
    }

    // -----------------------------------------------------------------------
    // Lifecycle helpers
    // -----------------------------------------------------------------------

    pub fn set_chip_baud(&mut self, target_baud: u32) -> Result<()> {
        set_chip_baud_inner(&mut self.pirate, self.id, target_baud)?;
        self.baud = target_baud;
        Ok(())
    }

    /// Reboot the chip and wait for it to come back at `BOOT_BAUD`.
    pub fn reboot_chip(&mut self) -> Result<()> {
        let frame = build_reboot(Id::new(self.id))?;
        let _ = self.xfer_reply(&frame, DEFAULT_IDLE_US)?;
        sleep(Duration::from_millis(500));
        self.pirate.set_baud(BOOT_BAUD)?;
        self.baud = BOOT_BAUD;
        drain_all(&mut self.pirate)?;
        Ok(())
    }

    /// Mean chip-TX drift in ppm over `samples` long-Read probes.
    /// Positive ⇒ chip TX slow (HSI slow); negative ⇒ fast.
    pub fn probe_drift_ppm(&mut self, samples: u32) -> Result<f64> {
        drift::probe_drift_ppm(self, samples)
    }

    /// Pirate health probe. Always succeeds even when DESYNCED.
    pub fn pirate_status(&mut self) -> Result<PirateStatus> {
        self.pirate.status()
    }

    /// Clear DESYNCED + cause, drain stamp/IC rings, re-arm walker.
    pub fn pirate_reset(&mut self) -> Result<()> {
        self.pirate.reset()
    }

    /// Atomic snapshot of the pirate's IC ring + walker counters. Diagnostic
    /// surface — bypasses the desync guard so it stays usable post-trip.
    pub fn ic_snapshot(&mut self) -> Result<IcSnapshot> {
        self.pirate.ic_snapshot()
    }

    /// Direct pirate `MASTER`/`FIRE` access for selftest-style bins that
    /// need to drive bytes without the `xfer`-shaped echo+reply contract.
    /// Avoid for protocol tools.
    pub fn pirate_master(&mut self, data: &[u8]) -> Result<()> {
        self.pirate.master(data)
    }

    /// Drain up to `max` byte stamps as a binary BBATCH frame.
    pub fn pirate_bbatch(&mut self, max: u16) -> Result<Vec<BStamp>> {
        self.pirate.bbatch(max)
    }

    /// Reconfigure the pirate's USART baud (and IC filter). Use for selftest
    /// runs where there is no chip to follow along — `set_chip_baud`
    /// otherwise drives both sides together.
    pub fn pirate_set_baud(&mut self, bps: u32) -> Result<()> {
        self.pirate.set_baud(bps)
    }
}

// ---------------------------------------------------------------------------
// Internal: probing + xfer loop
// ---------------------------------------------------------------------------

/// Drain the pirate's stamp ring until one `bbatch` comes back empty.
/// Pre-call invariant (held by every `Bus` entry point): the bus has
/// already been silent for at least `DEFAULT_IDLE_US`, so a single empty
/// poll genuinely means the ring is clear — there are no bytes still in
/// flight that could slip in between `drain_all` returning and `master`
/// firing.
fn drain_all(client: &mut pirate::Client) -> Result<()> {
    loop {
        if client.bbatch(255)?.is_empty() {
            return Ok(());
        }
    }
}

fn xfer_inner(client: &mut pirate::Client, req: &[u8], idle_us: u32) -> Result<ReplyCapture> {
    drain_all(client)?;
    client.master(req)?;
    collect_until_silent(client, req, idle_us)
}

/// Collect post-fire stamps until `idle_us` of host wall-clock has
/// elapsed since the most recent non-empty `bbatch`. Each new batch
/// resets the silence timer; no explicit "first byte arrived" gate is
/// needed because the timer starts at the call boundary and only an
/// unresponsive bus (no bytes, ever) will trip it before the wire
/// quiesces.
///
/// Two hardware floors shape the lower bound on `idle_us`:
///
/// - Walker cadence ~114 µs (§3.2 of `tools/dxl-pirate/TIMING.md`):
///   stamps lag the wire by up to one quarter-wrap.
/// - `bbatch` USB-CDC RTT ~1 ms per poll.
///
/// `DEFAULT_IDLE_US = 5 ms` clears both with margin; callers can pass
/// larger values to absorb longer expected inter-byte gaps.
///
/// `MAX_TOTAL_WAIT` is a hang-protection backstop only — a healthy
/// pirate never reaches it because `collect_until_silent`'s exit
/// criterion is silence, not elapsed time.
fn collect_until_silent(
    client: &mut pirate::Client,
    req: &[u8],
    idle_us: u32,
) -> Result<ReplyCapture> {
    let idle = Duration::from_micros(idle_us as u64);
    let start = Instant::now();
    let mut stamps: Vec<BStamp> = Vec::new();
    let mut last_byte_time = Instant::now();

    loop {
        let batch = client.bbatch(64)?;
        if batch.is_empty() {
            if last_byte_time.elapsed() >= idle {
                break;
            }
            sleep(EMPTY_POLL_BACKOFF);
        } else {
            stamps.extend(batch);
            last_byte_time = Instant::now();
        }
        if start.elapsed() >= MAX_TOTAL_WAIT {
            bail!(
                "pirate xfer exceeded {} s without bus going quiet — pirate likely wedged",
                MAX_TOTAL_WAIT.as_secs()
            );
        }
    }

    // Echo verification: if we have at least req.len() stamps, the leading
    // window must byte-match the request. Mismatches mean wire-RC trouble
    // or a foreign source on the bus — bail loudly, do not paper over.
    if stamps.len() >= req.len() {
        for (i, &expected) in req.iter().enumerate() {
            if stamps[i].byte != expected {
                bail!(
                    "master echo mismatch at byte {i}: got 0x{:02X}, expected 0x{:02X}",
                    stamps[i].byte,
                    expected
                );
            }
        }
    }

    let timing = if stamps.len() > req.len() {
        Some(ReplyTiming {
            req_end: stamps[req.len() - 1].tick,
            reply_first: stamps[req.len()].tick,
            reply_last: stamps.last().unwrap().tick,
        })
    } else {
        None
    };
    Ok(ReplyCapture { stamps, timing })
}

fn probe_baud(client: &mut pirate::Client, bps: u32) -> Result<Option<u8>> {
    client.set_baud(bps)?;
    sleep(BAUD_SETTLE);
    drain_all(client)?;
    let frame = build_ping(Id::BROADCAST)?;
    // Broadcast Ping needs an idle window that outlasts the worst-case
    // pre-reply silence: per DXL 2.0, a servo with id N delays its reply
    // by `N * 14 * byte_time` after the request packet end. Until that
    // delay elapses no bytes hit the wire — the inter-byte-reset path
    // can't help. Size the initial window for ID=253 (the max), then the
    // normal idle path handles any inter-servo gap once replies start.
    let cap = xfer_inner(client, &frame, broadcast_ping_idle_us(bps))?;
    if cap.timing.is_none() {
        return Ok(None);
    }
    let reply: Vec<u8> = cap
        .stamps
        .iter()
        .skip(frame.len())
        .map(|s| s.byte)
        .collect();
    Ok(Some(decode_ping_status(&reply, None)?.id))
}

fn probe_unicast(client: &mut pirate::Client, id: Id) -> Result<Option<u8>> {
    let frame = build_ping(id)?;
    let cap = xfer_inner(client, &frame, DEFAULT_IDLE_US)?;
    if cap.timing.is_none() {
        return Ok(None);
    }
    let reply: Vec<u8> = cap
        .stamps
        .iter()
        .skip(frame.len())
        .map(|s| s.byte)
        .collect();
    Ok(Some(decode_ping_status(&reply, Some(id))?.id))
}

fn set_chip_baud_inner(client: &mut pirate::Client, id: u8, target_baud: u32) -> Result<()> {
    let target_idx = baud_index(target_baud)?;
    let frame = build_write(Id::new(id), BAUD_RATE_IDX_ADDR, &[target_idx])?;
    let _ = xfer_inner(client, &frame, DEFAULT_IDLE_US)?;
    sleep(BAUD_SETTLE);
    client.set_baud(target_baud)?;
    sleep(BAUD_SETTLE);
    drain_all(client)?;
    // USB-CDC baud reconfig can glitch the pirate TX line into a phantom
    // byte that the chip's USART frames as FE/NE — the first post-switch
    // ping may legitimately time out, so retry before bailing.
    for _ in 0..3 {
        if probe_unicast(client, Id::new(id))?.is_some() {
            return Ok(());
        }
        sleep(Duration::from_millis(20));
    }
    bail!("servo {id} stopped replying after baud switch to {target_baud}");
}

/// Idle window for a broadcast Ping discovery probe at `bps`. Covers the
/// worst-case ID=253 reply delay (`253 * 14 * byte_time`) plus a small
/// margin so the helper doesn't return empty before the chip starts
/// replying. Once any byte arrives the standard idle-reset path takes
/// over.
fn broadcast_ping_idle_us(bps: u32) -> u32 {
    const MAX_ID: u32 = 253;
    const STATUS_FRAME_BYTES: u32 = 14;
    let byte_time_us = (10u32 * 1_000_000).div_ceil(bps);
    MAX_ID * STATUS_FRAME_BYTES * byte_time_us + DEFAULT_IDLE_US
}

// ---------------------------------------------------------------------------
// HSI drift probe
// ---------------------------------------------------------------------------

pub mod drift {
    //! Host-side HSI drift probe.
    //!
    //! The chip owns its clock trim — it watches inter-byte timing on
    //! every non-Status packet and nudges HSITRIM toward zero drift
    //! autonomously (see `firmware/lib/drivers/src/dxl/uart/clock.rs`).
    //! There is no CAL round-trip and no readable trim register: the only
    //! signal the host can observe is how long the chip takes to transmit
    //! a reply, measured against the pirate's stable clock.

    use anyhow::{Result, anyhow, bail};
    use dxl_protocol::types::Id;

    use super::{Bus, DEFAULT_IDLE_US, ReplyTiming, build_read, parse_status_reply};

    /// HSITRIM step size mapped to ppm. Mirrors
    /// `firmware/ch32/src/hal/rcc/v00x.rs::CLOCK_TRIM_PPM_PER_STEP`.
    pub const DRIFT_STEP_PPM: f64 = 2500.0;

    const PROBE_ADDR: u16 = 0;
    const PROBE_LEN: u16 = 128;
    const STATUS_OVERHEAD: usize = 11;

    pub fn probe_drift_ppm(bus: &mut Bus, samples: u32) -> Result<f64> {
        const RETRIES: u32 = 3;
        let id = Id::new(bus.id());
        let baud = bus.baud();
        let n = STATUS_OVERHEAD + PROBE_LEN as usize;
        let ticks_per_us = bus.hz_per_us()? as f64;
        let nominal_ticks = (n as f64 - 1.0) * 10.0 / baud as f64 * ticks_per_us * 1_000_000.0;

        let mut spans: Vec<u32> = Vec::with_capacity(samples as usize);
        for _ in 0..samples {
            let mut last_err: Option<anyhow::Error> = None;
            for _ in 0..RETRIES {
                match one_span(bus, id) {
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

    fn one_span(bus: &mut Bus, id: Id) -> Result<u32> {
        let frame = build_read(id, PROBE_ADDR, PROBE_LEN)?;
        let cap = bus.xfer(&frame, DEFAULT_IDLE_US)?;
        let ReplyTiming {
            reply_first,
            reply_last,
            ..
        } = cap
            .timing
            .ok_or_else(|| anyhow!("no reply on drift Read"))?;
        // Decode + validate the reply bytes too, so a CRC-broken response
        // counts as a retry-able transport failure rather than feeding bad
        // timing into the average.
        let reply: Vec<u8> = cap
            .stamps
            .iter()
            .skip(frame.len())
            .map(|s| s.byte)
            .collect();
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
        Ok(reply_last.wrapping_sub(reply_first))
    }
}
