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
    /// Requested zero-byte payload length in the slave's Status reply.
    /// Bounded `1..=128` by the dispatcher.
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
            OscVariant::Calibrate(p) => {
                write_ext::<W, CRC, _>(out, p.id, CALIBRATE_INSTRUCTION, p.count.to_le_bytes())
            }
        }
    }
}

fn decode_calibrate(id: u8, params: Bytes<'_>) -> Result<OscVariant, DecodeError> {
    let mut it = params.iter();
    let lo = it.next().ok_or(DecodeError::BadParams)?;
    let hi = it.next().ok_or(DecodeError::BadParams)?;
    if it.next().is_some() {
        return Err(DecodeError::BadParams);
    }
    Ok(OscVariant::Calibrate(CalibratePacket {
        id,
        count: u16::from_le_bytes([lo, hi]),
    }))
}

/// Marker for the OSC reply extension; bind as the `S` parameter of
/// [`decode_status`](dxl_protocol::decode_status) /
/// [`write_status`](dxl_protocol::write_status).
#[derive(Copy, Clone, Debug)]
pub struct OscReplyExt;

/// OSC reply shapes. The chip emits these via `bus.send(Status::Ext(..))`.
#[derive(Copy, Clone, Debug)]
pub enum OscReplyVariant {
    /// Status frame with `zeros_count` zero data bytes — no caller-side
    /// scratch needed; the writer streams the zeros directly.
    Calibrate { id: u8, zeros_count: u16 },
}

impl StatusExt for OscReplyExt {
    type Variant<'a> = OscReplyVariant;

    fn decode<'a>(_instr: u8, _raw: RawStatus<'a>) -> Option<Result<OscReplyVariant, DecodeError>> {
        // OSC currently has no typed master-side status flavor — a Calibrate
        // response is an empty Status that decode_status surfaces via the
        // native success path. Add an arm here when a typed flavor lands.
        None
    }

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &OscReplyVariant,
        out: &mut W,
    ) -> Result<(), WriteError> {
        match *v {
            OscReplyVariant::Calibrate { id, zeros_count } => write_ext::<W, CRC, _>(
                out,
                id,
                Instruction::Status.as_u8(),
                core::iter::once(StatusError::None.as_u8())
                    .chain(core::iter::repeat_n(0u8, zeros_count as usize)),
            ),
        }
    }
}
