//! OpenServoCore vendor-extension verbs that ride on the DXL 2.0 wire.
//!
//! Reserves the `0xE0..0xEF` vendor-extension instruction block (currently:
//! `CAL = 0xE0`) — far from Robotis's allocated clusters so a future protocol
//! revision can't quietly collide.
//!
//! Plugs into [`dxl_protocol::Codec`] via the [`Extension`] / [`ReplyExt`]
//! traits: bind `Codec<CRC, OscExt, OscReplyExt>` and the standard
//! `Packet`/`StatusReply` enums grow `Ext` arms carrying [`OscVariant`] /
//! [`OscReplyVariant`].

use dxl_protocol::prelude::{Extension, ReplyExt};
use dxl_protocol::wire::{Bytes, CrcUmts, RawFrame, WriteBuf, WriteError, write_raw};
use dxl_protocol::{DecodeError, Instruction, StatusError};

/// Marker for the OSC request extension; bind it as the `X` parameter of
/// [`dxl_protocol::Codec`].
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

impl Extension for OscExt {
    type Variant<'a> = OscVariant;

    fn decode<'a>(raw: RawFrame<Bytes<'a>>) -> Option<Result<OscVariant, DecodeError>> {
        match raw.instruction {
            CALIBRATE_INSTRUCTION => Some(decode_calibrate(raw)),
            _ => None,
        }
    }

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(v: &OscVariant, out: &mut W) -> Result<(), WriteError> {
        match *v {
            OscVariant::Calibrate(p) => write_raw::<W, _, CRC>(
                out,
                RawFrame {
                    id: p.id,
                    instruction: CALIBRATE_INSTRUCTION,
                    params: p.count.to_le_bytes(),
                },
            ),
        }
    }
}

fn decode_calibrate(raw: RawFrame<Bytes<'_>>) -> Result<OscVariant, DecodeError> {
    let mut it = raw.params.iter();
    let lo = it.next().ok_or(DecodeError::BadParams)?;
    let hi = it.next().ok_or(DecodeError::BadParams)?;
    if it.next().is_some() {
        return Err(DecodeError::BadParams);
    }
    Ok(OscVariant::Calibrate(CalibratePacket {
        id: raw.id,
        count: u16::from_le_bytes([lo, hi]),
    }))
}

/// Marker for the OSC reply extension; bind it as the `R` parameter of
/// [`dxl_protocol::Codec`].
#[derive(Copy, Clone, Debug)]
pub struct OscReplyExt;

/// OSC reply shapes. The chip emits these via `bus.send(StatusReply::Ext(..))`.
#[derive(Copy, Clone, Debug)]
pub enum OscReplyVariant {
    /// Status frame with `zeros_count` zero data bytes — no caller-side
    /// scratch needed; the writer streams the zeros directly.
    Calibrate { id: u8, zeros_count: u16 },
}

impl ReplyExt for OscReplyExt {
    type Variant<'a> = OscReplyVariant;

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &OscReplyVariant,
        out: &mut W,
    ) -> Result<(), WriteError> {
        match *v {
            OscReplyVariant::Calibrate { id, zeros_count } => write_raw::<W, _, CRC>(
                out,
                RawFrame {
                    id,
                    instruction: Instruction::Status.as_u8(),
                    params: core::iter::once(StatusError::None.as_u8())
                        .chain(core::iter::repeat_n(0u8, zeros_count as usize)),
                },
            ),
        }
    }
}
