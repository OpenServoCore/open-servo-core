//! OpenServoCore vendor-extension verbs that ride on the DXL 2.0 wire.
//!
//! Reserves the `0xE0..0xEF` vendor-extension instruction block (currently:
//! `CAL = 0xE0`) — far from Robotis's allocated clusters so a future protocol
//! revision can't quietly collide.
//!
//! Plugs into the protocol via the [`InstructionExt`] / [`StatusExt`] traits:
//! bind the chip's [`DxlWire`](../../../osc_ch32/dxl/wire/struct.DxlWire.html)
//! alias over `<OscExt, OscReplyExt>` and the standard `Packet`/`Status` enums
//! grow `Ext` arms carrying [`OscVariant`] / [`OscReplyVariant`].
//!
//! `CAL` is a slave-side timed receive: master streams `count` filler bytes,
//! slave times its own RX between T_first (first byte) and T_last (IDLE-
//! backdated end of last byte), computes drift vs the nominal at the current
//! baud, queues HSI trim updates, and returns the measurement in a single
//! Status reply. No master-side timestamping required.

use dxl_protocol::packet::RawPacket;
use dxl_protocol::{
    Bytes, CrcUmts, DecodeError, Instruction, InstructionExt, RawStatus, StatusError, StatusExt,
    WriteBuf, WriteError, write_ext,
};

/// Marker for the OSC request extension; bind as the `I` parameter of
/// [`parse_packet`](dxl_protocol::parse_packet) /
/// [`write_packet`](dxl_protocol::write_packet).
#[derive(Copy, Clone, Debug)]
pub struct OscExt;

/// OSC request verbs decoded from the wire.
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

const CALIBRATE_INSTRUCTION: u8 = 0xE0;

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

/// Streaming-decoder counterpart to the [`InstructionExt`] decode path.
/// `Packet::Raw` carries unrecognized-instruction frames after the new
/// parser routes them; the dispatcher calls this to resolve OSC verbs from
/// the raw byte slice. Returns `None` for instruction bytes we don't own.
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

/// Marker for the OSC reply extension; bind as the `S` parameter of
/// [`decode_status`](dxl_protocol::decode_status) /
/// [`write_status`](dxl_protocol::write_status).
#[derive(Copy, Clone, Debug)]
pub struct OscReplyExt;

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

/// OSC reply shapes. The chip emits these via `bus.send(Status::Ext(..))`.
#[derive(Copy, Clone, Debug)]
pub enum OscReplyVariant {
    Calibrate(CalibrateStatus),
}

impl StatusExt for OscReplyExt {
    type Variant<'a> = OscReplyVariant;

    fn decode<'a>(_instr: u8, _raw: RawStatus<'a>) -> Option<Result<OscReplyVariant, DecodeError>> {
        // Master-side typed decode lands when a Rust master needs it; PC bench
        // parses the 11 payload bytes directly off the raw Status frame.
        None
    }

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &OscReplyVariant,
        out: &mut W,
    ) -> Result<(), WriteError> {
        match *v {
            OscReplyVariant::Calibrate(s) => {
                let observed = s.observed_ticks.to_le_bytes();
                let nominal = s.nominal_ticks.to_le_bytes();
                let fine = s.applied_fine_trim_us.to_le_bytes();
                write_ext::<W, CRC, _>(
                    out,
                    s.id,
                    Instruction::Status.as_u8(),
                    core::iter::once(StatusError::None.as_u8())
                        .chain(observed)
                        .chain(nominal)
                        .chain(core::iter::once(s.applied_trim_delta as u8))
                        .chain(fine),
                )
            }
        }
    }
}
