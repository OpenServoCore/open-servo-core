//! osc-native host library: build instruction wire bytes, and parse a captured
//! pirate stamp stream back into a status frame plus the TURNAROUND metric.
//!
//! TURNAROUND is the project's success metric: the time between the END of the
//! instruction packet (the last byte's stop bit completes) and the BEGINNING of
//! the status packet (the reply break's falling edge). From pirate stamps:
//!
//! ```text
//! turnaround = reply_break_stamp.tick - (last_instruction_echo_stamp.tick + 10 * bit_ticks)
//! ```
//!
//! The `+ 10 * bit_ticks` walks from the last echo byte's capture edge to the
//! end of its stop bit (1 start + 8 data + 1 stop = 10 bit-times).

use osc_protocol::crc::osc_crc;
use osc_protocol::frame::Header;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{Id, Inst, MgmtOp, Opcode, ResultCode, UID_LEN};

use crate::pirate::BStamp;

/// Break byte / CRC prefix — a `0x00` stamp marks a frame anchor on the wire.
const BREAK: u8 = 0x00;
/// Bit-times per byte on the wire: 1 start + 8 data + 1 stop.
const BITS_PER_BYTE: u32 = 10;

/// SAVE/FACTORY settle: the ack lands only after the erase + program stall
/// (§9.4 — measured 5–10 ms; 50 ms is the host-timeout guidance).
pub const SAVE_SETTLE_MS: u64 = 50;
/// Post-reset boot time before the servo answers again, with margin.
pub const REBOOT_SETTLE_MS: u64 = 100;
/// Host rescue pulse (§9.1): the servo confirm threshold is ~300 µs of
/// dominant low; 400 gives margin without meaningfully holding the bus.
// ~3× the servo's 300 µs sampler floor: the main-loop sampler's cadence
// rides the 20 kHz idle tick and jitters under ISR load, and repeats are
// idempotent — a fat pulse costs nothing on a recovery path.
pub const RESCUE_PULSE_US: u32 = 1_000;

/// Build the wire bytes (no CRC-prefix; the physical break carries it) for one
/// host->servo instruction frame.
pub fn build_instruction(id: u8, op: Opcode, flags: u8, payload: &[u8]) -> Vec<u8> {
    let mut b = FrameBuf::<300>::new();
    b.start(Id::new(id), Inst::instruction(op, flags));
    let p = payload.len();
    b.payload_mut()[..p].copy_from_slice(payload);
    b.finish(p as u8);
    b.seal();
    // frame()[0] is the 0x00 CRC-prefix the break replaces on the wire.
    b.frame()[1..].to_vec()
}

pub fn build_ping(id: u8) -> Vec<u8> {
    build_instruction(id, Opcode::Ping, 0, &[])
}

/// READ span: `addr(2), count(2)` little-endian (mirrors `ReadReq`).
pub fn build_read(id: u8, addr: u16, len: u16) -> Vec<u8> {
    let mut payload = [0u8; 4];
    payload[..2].copy_from_slice(&addr.to_le_bytes());
    payload[2..].copy_from_slice(&len.to_le_bytes());
    build_instruction(id, Opcode::Read, 0, &payload)
}

/// PROFILE region base in the flat table (osc-core `regions::PROFILE_BASE_ADDR`;
/// pinned here to keep the heavy core crate out of the bench build).
pub const PROFILE_BASE_ADDR: u16 = 0x280;
/// Span words per profile slot (osc-core `regions::profile::SPANS_PER_SLOT`).
pub const PROFILE_SPANS_PER_SLOT: usize = 8;

/// Packed profile span word `[addr:10][count:6]` (§5.2; mirrors osc-core
/// `regions::profile::span_word`). `count = 0` = word disabled.
pub const fn profile_span_word(addr: u16, count: u8) -> u16 {
    (addr << 6) | (count as u16 & 0x3F)
}

/// Split a packed span word back into `(addr, count)`.
pub const fn profile_span_split(word: u16) -> (u16, u8) {
    (word >> 6, (word & 0x3F) as u8)
}

/// Table byte address of `slot`'s first span word.
pub const fn profile_slot_addr(slot: u8) -> u16 {
    PROFILE_BASE_ADDR + slot as u16 * (PROFILE_SPANS_PER_SLOT as u16 * 2)
}

/// WRITE that configures one whole profile slot: the given spans, remaining
/// words zeroed (disabled).
pub fn build_profile_config(id: u8, slot: u8, spans: &[(u16, u8)]) -> Vec<u8> {
    assert!(spans.len() <= PROFILE_SPANS_PER_SLOT, "spans per slot cap");
    let mut data = Vec::with_capacity(PROFILE_SPANS_PER_SLOT * 2);
    for &(addr, count) in spans {
        data.extend_from_slice(&profile_span_word(addr, count).to_le_bytes());
    }
    data.resize(PROFILE_SPANS_PER_SLOT * 2, 0);
    build_write(id, profile_slot_addr(slot), &data)
}

/// READ + PROFILE: the payload names a slot (§5.2).
pub fn build_read_profile(id: u8, slot: u8) -> Vec<u8> {
    build_instruction(id, Opcode::Read, Inst::FLAG_PROFILE, &[slot])
}

/// Broadcast MGMT ENUM: `prefix_len` bits (0..=128) + `ceil(prefix_len/8)`
/// prefix bytes, LSB-first bit stream (§9.2; mirrors `EnumReq`).
pub fn build_enum(prefix_len: u8, prefix: &[u8]) -> Vec<u8> {
    let mut payload = vec![MgmtOp::Enum as u8, prefix_len];
    payload.extend_from_slice(prefix);
    build_instruction(Id::BROADCAST.as_byte(), Opcode::Mgmt, 0, &payload)
}

/// Broadcast MGMT ASSIGN: `uid(16)` + `new_id` (§9.2; mirrors `AssignReq`).
pub fn build_assign(uid: &[u8; UID_LEN], new_id: u8) -> Vec<u8> {
    let mut payload = vec![MgmtOp::Assign as u8];
    payload.extend_from_slice(uid);
    payload.push(new_id);
    build_instruction(Id::BROADCAST.as_byte(), Opcode::Mgmt, 0, &payload)
}

/// Broadcast MGMT CAL announce (§9.3; mirrors `CalReq`): commits the host to
/// follow with `gaps + 1` bare breaks spaced exactly `gap_us` apart. Draws no
/// reply — an ack's own break would enter the train.
pub fn build_cal(gap_us: u16, gaps: u8) -> Vec<u8> {
    let mut payload = vec![MgmtOp::Cal as u8];
    payload.extend_from_slice(&gap_us.to_le_bytes());
    payload.push(gaps);
    build_instruction(Id::BROADCAST.as_byte(), Opcode::Mgmt, 0, &payload)
}

/// MGMT SAVE (§9.4). The ack arrives only after the erase + program stall —
/// exchange with a SAVE-specific window, not the standard settle.
pub fn build_save(id: u8) -> Vec<u8> {
    build_instruction(id, Opcode::Mgmt, 0, &[MgmtOp::Save as u8])
}

/// MGMT FACTORY (§9.5): wipes both saved slots, acks (same stall caveat as
/// SAVE), then the servo reboots itself.
pub fn build_factory(id: u8) -> Vec<u8> {
    build_instruction(id, Opcode::Mgmt, 0, &[MgmtOp::Factory as u8])
}

/// MGMT REBOOT (§9.5): acks first, then resets once the ack has drained.
pub fn build_reboot(id: u8) -> Vec<u8> {
    build_instruction(id, Opcode::Mgmt, 0, &[MgmtOp::Reboot as u8])
}

/// WRITE: `addr(2)` little-endian then the data bytes (mirrors `WriteReq`).
pub fn build_write(id: u8, addr: u16, data: &[u8]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(2 + data.len());
    payload.extend_from_slice(&addr.to_le_bytes());
    payload.extend_from_slice(data);
    build_instruction(id, Opcode::Write, 0, &payload)
}

/// Uniform GWRITE payload: `addr(2) count(1) [id(1) data(count)]×`, every target
/// sharing `data.len()` (mirrors `osc_protocol::group::GwriteUniform`). `count`
/// is taken from the first entry.
pub fn gwrite_uniform_payload(addr: u16, entries: &[(u8, &[u8])]) -> Vec<u8> {
    let count = entries.first().map_or(0, |(_, data)| data.len());
    let mut p = Vec::with_capacity(3 + entries.len() * (1 + count));
    p.extend_from_slice(&addr.to_le_bytes());
    p.push(count as u8);
    for (id, data) in entries {
        p.push(*id);
        p.extend_from_slice(data);
    }
    p
}

/// Uniform GREAD payload: `addr(2) count(2) id-list` — one span, many servos
/// (mirrors `osc_protocol::group::GreadUniform`).
pub fn gread_uniform_payload(addr: u16, count: u16, ids: &[u8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(4 + ids.len());
    p.extend_from_slice(&addr.to_le_bytes());
    p.extend_from_slice(&count.to_le_bytes());
    p.extend_from_slice(ids);
    p
}

/// A decoded status frame. `payload` excludes the PAD byte.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StatusFrame {
    pub id: u8,
    pub result: Option<ResultCode>,
    pub alert: bool,
    pub payload: Vec<u8>,
}

/// One instruction/status round trip with its TURNAROUND.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Exchange {
    pub status: StatusFrame,
    pub turnaround_ticks: u32,
    /// Stamp index one past the reply frame. Stamps beyond it are energy the
    /// parse ignored — for broadcast ENUM that means a slot-delayed peer's
    /// reply trailing the winner's clean frame (§9.2): a collision, not a
    /// unique match.
    pub stamps_end: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExchangeError {
    /// No `0x00` break followed by the sent wire bytes in the stamp stream.
    NoEcho,
    /// The echo was found but no reply break followed it.
    NoReply,
    /// The reply break arrived but too few stamps followed to form the frame.
    Truncated,
    /// The reply header failed osc-native validation (LEN/ID/opcode).
    BadHeader,
    BadCrc {
        computed: u16,
        wire: u16,
    },
}

impl std::fmt::Display for ExchangeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoEcho => write!(f, "instruction echo not found in stamps"),
            Self::NoReply => write!(f, "no reply break after instruction echo"),
            Self::Truncated => write!(f, "reply truncated before frame end"),
            Self::BadHeader => write!(f, "reply header failed validation"),
            Self::BadCrc { computed, wire } => {
                write!(
                    f,
                    "reply CRC mismatch: computed {computed:#06X}, wire {wire:#06X}"
                )
            }
        }
    }
}

impl std::error::Error for ExchangeError {}

/// Parse one captured exchange: locate the host TX echo, then the reply break,
/// decode + CRC-check the status frame, and compute TURNAROUND (see module docs).
pub fn parse_exchange(
    stamps: &[BStamp],
    sent_wire: &[u8],
    bit_ticks: u32,
) -> Result<Exchange, ExchangeError> {
    let echo_break = find_echo(stamps, sent_wire).ok_or(ExchangeError::NoEcho)?;
    // Last byte of the instruction echo sits `sent_wire.len()` stamps past the
    // echo break (which is at `echo_break`).
    let last_echo = &stamps[echo_break + sent_wire.len()];

    let reply_break_idx = stamps[echo_break + sent_wire.len() + 1..]
        .iter()
        .position(|s| s.byte == BREAK)
        .map(|p| echo_break + sent_wire.len() + 1 + p)
        .ok_or(ExchangeError::NoReply)?;
    let reply = &stamps[reply_break_idx..];

    if reply.len() < Header::SIZE {
        return Err(ExchangeError::Truncated);
    }
    let hdr_bytes = [reply[0].byte, reply[1].byte, reply[2].byte, reply[3].byte];
    let hdr = Header::from_bytes(&hdr_bytes);
    hdr.validate().map_err(|_| ExchangeError::BadHeader)?;

    let footprint = hdr.frame_end();
    if reply.len() < footprint {
        return Err(ExchangeError::Truncated);
    }
    let frame: Vec<u8> = reply[..footprint].iter().map(|s| s.byte).collect();

    let clen = hdr.covered_len();
    let computed = osc_crc(&frame[..clen]);
    let wire = u16::from_le_bytes([frame[clen], frame[clen + 1]]);
    if computed != wire {
        return Err(ExchangeError::BadCrc { computed, wire });
    }

    let p = hdr.payload_len() as usize;
    let status = StatusFrame {
        id: hdr.id.as_byte(),
        result: hdr.inst.result(),
        alert: hdr.inst.alert(),
        payload: frame[4..4 + p].to_vec(),
    };
    let turnaround_ticks = reply[0]
        .tick
        .wrapping_sub(last_echo.tick.wrapping_add(BITS_PER_BYTE * bit_ticks));
    Ok(Exchange {
        status,
        turnaround_ticks,
        stamps_end: reply_break_idx + footprint,
    })
}

/// Index of the echo break: a `0x00` stamp whose following stamps' bytes equal
/// `sent_wire`.
fn find_echo(stamps: &[BStamp], sent_wire: &[u8]) -> Option<usize> {
    stamps.iter().enumerate().position(|(i, s)| {
        s.byte == BREAK
            && stamps.len() - i > sent_wire.len()
            && stamps[i + 1..=i + sent_wire.len()]
                .iter()
                .zip(sent_wire)
                .all(|(st, &b)| st.byte == b)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_ping_matches_wire_vector() {
        // CRC-16/ARC over `01 03 10` = 0xFC50 (osc-native-protocol.md §3.2).
        assert_eq!(build_ping(1), [0x01, 0x03, 0x10, 0x50, 0xFC]);
    }

    #[test]
    fn profile_span_word_round_trips() {
        let w = profile_span_word(0x208, 63);
        assert_eq!(profile_span_split(w), (0x208, 63));
        assert_eq!(profile_span_split(profile_span_word(1023, 1)), (1023, 1));
    }

    #[test]
    fn gwrite_uniform_payload_layout() {
        // addr 0x0184, one target id=1, data = 10i32 LE.
        let p = gwrite_uniform_payload(0x0184, &[(1, &10i32.to_le_bytes())]);
        assert_eq!(p, [0x84, 0x01, 4, 1, 10, 0, 0, 0]);
    }

    #[test]
    fn gread_uniform_payload_layout() {
        // addr 0x0184, span 4 bytes, ids [1, 2].
        let p = gread_uniform_payload(0x0184, 4, &[1, 2]);
        assert_eq!(p, [0x84, 0x01, 4, 0, 1, 2]);
    }

    /// Build a stamp stream at `spacing` ticks/byte starting at `start`, one
    /// `BREAK` stamp then one stamp per byte.
    fn stamps_from(start: u32, spacing: u32, bytes: &[u8]) -> Vec<BStamp> {
        std::iter::once(BREAK)
            .chain(bytes.iter().copied())
            .enumerate()
            .map(|(i, byte)| BStamp {
                tick: start + i as u32 * spacing,
                byte,
                flags: 0,
            })
            .collect()
    }

    #[test]
    fn parses_ping_exchange() {
        // A ping reply carries model(2) + fw(1), status Ok (INST 0x80). Build the
        // frame with a computed CRC-16/ARC (§3.2) so it stays valid across CRC
        // changes; the 0x00 break stamp leads on the wire (init-0 no-op).
        const SPACING: u32 = 1440; // 1 Mbaud @ 144 ticks/bit
        const BIT_TICKS: u32 = 144;
        let sent = build_ping(1);

        // ID, LEN, INST(Ok status), model=0x0042, fw=0x56, then CRC.
        let mut reply = vec![0x01, 0x06, 0x80, 0x42, 0x00, 0x56];
        reply.extend_from_slice(&osc_crc(&reply).to_le_bytes());

        let mut stamps = stamps_from(0, SPACING, &sent);
        let last_echo = stamps.last().unwrap().tick; // 7200
        let reply_break = last_echo + 12_800; // 20000
        stamps.extend(stamps_from(reply_break, SPACING, &reply));

        let ex = parse_exchange(&stamps, &sent, BIT_TICKS).expect("exchange parses");
        assert_eq!(ex.status.id, 1);
        assert_eq!(ex.status.result, Some(ResultCode::Ok));
        assert!(!ex.status.alert);
        assert_eq!(ex.status.payload, [0x42, 0x00, 0x56], "model(2) + fw(1)");
        // 20000 - (7200 + 10*144) = 20000 - 8640
        assert_eq!(ex.turnaround_ticks, 11_360);
    }

    #[test]
    fn missing_reply_is_no_reply() {
        let sent = build_ping(1);
        let stamps = stamps_from(0, 1440, &sent);
        assert_eq!(
            parse_exchange(&stamps, &sent, 144),
            Err(ExchangeError::NoReply)
        );
    }
}
