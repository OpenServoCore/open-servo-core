//! OpenServoCore vendor-extension verbs that ride on the DXL 2.0 wire.
//!
//! Reserves the `0xE0..0xEF` vendor-extension instruction block (currently:
//! `CAL = 0xE0`) — far from Robotis's allocated clusters so a future protocol
//! revision can't quietly collide.
//!
//! Plugs into the new streaming parser via `Packet::Raw`: the chip-side
//! dispatcher calls [`decode_raw`] on the raw byte slice to resolve OSC
//! verbs. The reply path uses the unified `Status<'a>` enum's `Raw` variant
//! — the dispatcher composes payload bytes via [`calibrate_status_bytes`]
//! and emits via `bus.send(Status::Raw { id, error, payload })`.
//!
//! `CAL` is a slave-side timed receive: master streams `count` filler bytes,
//! slave times its own RX between T_first (first byte) and T_last (IDLE-
//! backdated end of last byte), computes drift vs the nominal at the current
//! baud, queues HSI trim updates, and returns the measurement in a single
//! Status reply. No master-side timestamping required.

use dxl_protocol::packet::RawPacket;
use dxl_protocol::{Bytes, CrcUmts, DecodeError, InstructionExt, WriteBuf, WriteError, write_ext};

/// Marker for the OSC request extension, kept so tests can still encode a
/// Calibrate request via the legacy `Packet<'_, OscExt>` + `write_packet`
/// path. Production dispatch goes through `Packet::Raw` → [`decode_raw`].
/// Removed alongside the rest of the legacy typed layer in #134.
#[derive(Copy, Clone, Debug)]
pub struct OscExt;

#[derive(Copy, Clone, Debug)]
pub enum OscVariant {
    Calibrate(CalibratePacket),
}

#[derive(Copy, Clone, Debug)]
pub struct CalibratePacket {
    pub id: u8,
    /// Number of filler bytes the master streams after the `count` field —
    /// the slave times its own RX of this payload to derive HSI drift.
    /// Bounded `1..=MAX_CONTROL_RW` by the dispatcher. Filler content is
    /// irrelevant for timing; the writer emits zeros.
    pub count: u16,
}

impl CalibratePacket {
    pub const fn new(id: u8, count: u16) -> Self {
        Self { id, count }
    }
}

pub const CALIBRATE_INSTRUCTION: u8 = 0xE0;

impl InstructionExt for OscExt {
    type Variant<'a> = OscVariant;

    fn decode<'a>(instr: u8, id: u8, params: Bytes<'a>) -> Option<Result<OscVariant, DecodeError>> {
        match instr {
            CALIBRATE_INSTRUCTION => Some(decode_calibrate(id, params)),
            _ => None,
        }
    }

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(v: &OscVariant, out: &mut W) -> Result<(), WriteError> {
        match *v {
            OscVariant::Calibrate(p) => write_ext::<W, CRC, _>(
                out,
                p.id,
                CALIBRATE_INSTRUCTION,
                p.count
                    .to_le_bytes()
                    .into_iter()
                    .chain(core::iter::repeat_n(0u8, p.count as usize)),
            ),
        }
    }
}

fn decode_calibrate(id: u8, params: Bytes<'_>) -> Result<OscVariant, DecodeError> {
    let mut it = params.iter();
    let lo = it.next().ok_or(DecodeError::BadParams)?;
    let hi = it.next().ok_or(DecodeError::BadParams)?;
    let count = u16::from_le_bytes([lo, hi]);
    let payload_len = it.count();
    if payload_len != count as usize {
        return Err(DecodeError::BadParams);
    }
    Ok(OscVariant::Calibrate(CalibratePacket { id, count }))
}

/// Streaming-decoder dispatch for OSC verbs carried as `Packet::Raw`.
/// Returns `None` for instruction bytes we don't own.
pub fn decode_raw(raw: &RawPacket<'_>) -> Option<OscVariant> {
    let id = raw.header.header.id;
    match raw.header.header.instruction.as_byte() {
        CALIBRATE_INSTRUCTION => {
            if raw.params.len() < 2 {
                return None;
            }
            let count = u16::from_le_bytes([raw.params[0], raw.params[1]]);
            if raw.params.len() - 2 != count as usize {
                return None;
            }
            Some(OscVariant::Calibrate(CalibratePacket { id, count }))
        }
        _ => None,
    }
}

/// Slave-measured calibration result. Wire layout (LE) following the Status
/// error byte: `observed_ticks(4) | nominal_ticks(4) | applied_trim_delta(1)
/// | applied_fine_trim_us(2)` — 11 bytes total.
#[derive(Copy, Clone, Debug)]
pub struct CalibrateStatus {
    pub id: u8,
    /// SysTick ticks between T_first and T_last as measured by the slave.
    /// `µs = observed_ticks / ticks_per_us`.
    pub observed_ticks: u32,
    /// `(count − 1) × byte_time_ticks` at the slave's operating baud — the
    /// expected wire duration with zero clock drift.
    pub nominal_ticks: u32,
    /// HSI coarse-trim delta queued by this cal (signed). Zero when the
    /// algorithm chose not to step coarse trim.
    pub applied_trim_delta: i8,
    /// Absolute Q8.8 µs fine-trim residual written to the control table.
    pub applied_fine_trim_us: i16,
}

/// Pack a [`CalibrateStatus`] into its 11-byte wire payload (the bytes that
/// follow the Status error byte). Dispatcher emits the result via
/// `Status::Raw { payload: &calibrate_status_bytes(&s) }`.
pub fn calibrate_status_bytes(s: &CalibrateStatus) -> [u8; 11] {
    let observed = s.observed_ticks.to_le_bytes();
    let nominal = s.nominal_ticks.to_le_bytes();
    let fine = s.applied_fine_trim_us.to_le_bytes();
    [
        observed[0],
        observed[1],
        observed[2],
        observed[3],
        nominal[0],
        nominal[1],
        nominal[2],
        nominal[3],
        s.applied_trim_delta as u8,
        fine[0],
        fine[1],
    ]
}
