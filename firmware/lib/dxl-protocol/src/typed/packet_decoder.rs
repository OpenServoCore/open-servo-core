use crate::wire::Bytes;

use super::ext::InstructionExt;
use super::instruction::Instruction;
use super::packet::{
    ActionPacket, BulkReadPacket, BulkWritePacket, ClearPacket, ControlTableBackupPacket,
    FactoryResetPacket, FastBulkReadPacket, FastSyncReadPacket, Packet, PingPacket, RawStatus,
    ReadPacket, RebootPacket, RegWritePacket, SyncReadPacket, SyncWritePacket, WritePacket,
};

#[derive(Copy, Clone, Debug)]
pub enum DecodeError {
    UnknownInstruction,
    BadParams,
}

/// Decode the standard-DXL portion of a frame. Returns `UnknownInstruction`
/// for bytes outside the standard set — callers binding an [`InstructionExt`]
/// catch that error and try the extension's decoder.
pub(crate) fn decode<'a, X: InstructionExt>(
    instruction: u8,
    id: u8,
    params: Bytes<'a>,
) -> Result<Packet<'a, X>, DecodeError> {
    match Instruction::from_u8(instruction) {
        Instruction::Ext(_) => Err(DecodeError::UnknownInstruction),
        instruction => decode_typed::<X>(instruction, id, params),
    }
}

fn decode_typed<'a, X: InstructionExt>(
    instruction: Instruction,
    id: u8,
    params: Bytes<'a>,
) -> Result<Packet<'a, X>, DecodeError> {
    use Instruction::*;
    match instruction {
        Ping => need_empty(&params).map(|_| Packet::Ping(PingPacket { id })),
        Read => {
            let mut it = params.iter();
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            if it.next().is_some() {
                return Err(DecodeError::BadParams);
            }
            Ok(Packet::Read(ReadPacket {
                id,
                address,
                length,
            }))
        }
        Write => {
            let mut it = params.iter();
            let address = take_u16_le(&mut it)?;
            Ok(Packet::Write(WritePacket {
                id,
                address,
                data: it.rest_bytes(),
            }))
        }
        RegWrite => {
            let mut it = params.iter();
            let address = take_u16_le(&mut it)?;
            Ok(Packet::RegWrite(RegWritePacket {
                id,
                address,
                data: it.rest_bytes(),
            }))
        }
        Action => need_empty(&params).map(|_| Packet::Action(ActionPacket { id })),
        FactoryReset => {
            let mut it = params.iter();
            let mode = it.next().ok_or(DecodeError::BadParams)?;
            if it.next().is_some() {
                return Err(DecodeError::BadParams);
            }
            Ok(Packet::FactoryReset(FactoryResetPacket { id, mode }))
        }
        Reboot => need_empty(&params).map(|_| Packet::Reboot(RebootPacket { id })),
        Clear => Ok(Packet::Clear(ClearPacket { id, body: params })),
        ControlTableBackup => Ok(Packet::ControlTableBackup(ControlTableBackupPacket {
            id,
            body: params,
        })),
        Status => {
            let mut it = params.iter();
            let error = it.next().ok_or(DecodeError::BadParams)?;
            Ok(Packet::Status(RawStatus {
                id,
                error,
                params: it.rest_bytes(),
            }))
        }
        SyncRead => {
            let mut it = params.iter();
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::SyncRead(SyncReadPacket {
                address,
                length,
                ids: it.rest_bytes(),
            }))
        }
        SyncWrite => {
            let mut it = params.iter();
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::SyncWrite(SyncWritePacket {
                address,
                length,
                body: it.rest_bytes(),
            }))
        }
        FastSyncRead => {
            let mut it = params.iter();
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::FastSyncRead(FastSyncReadPacket {
                address,
                length,
                ids: it.rest_bytes(),
            }))
        }
        BulkRead => Ok(Packet::BulkRead(BulkReadPacket { body: params })),
        BulkWrite => Ok(Packet::BulkWrite(BulkWritePacket { body: params })),
        FastBulkRead => Ok(Packet::FastBulkRead(FastBulkReadPacket { body: params })),
        Ext(_) => Err(DecodeError::UnknownInstruction),
    }
}

fn need_empty(b: &Bytes<'_>) -> Result<(), DecodeError> {
    if b.iter().next().is_some() {
        return Err(DecodeError::BadParams);
    }
    Ok(())
}

fn take_u16_le<I: Iterator<Item = u8>>(it: &mut I) -> Result<u16, DecodeError> {
    let lo = it.next().ok_or(DecodeError::BadParams)?;
    let hi = it.next().ok_or(DecodeError::BadParams)?;
    Ok(u16::from_le_bytes([lo, hi]))
}
