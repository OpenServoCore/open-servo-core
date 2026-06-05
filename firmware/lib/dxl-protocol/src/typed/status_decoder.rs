//! Master-side second-stage decode: turn a [`RawStatus`] (yielded by
//! [`parse_packet`](crate::parse_packet) as `Packet::Status`) plus the
//! preceding request instruction into a typed [`Status`].

use super::ext::StatusExt;
use super::instruction::Instruction;
use super::packet::RawStatus;
use super::packet_decoder::DecodeError;
use super::status::{
    ActionStatus, BulkReadStatus, ErrorStatus, FastBulkReadStatus, FastSyncReadStatus, PingStatus,
    ReadStatus, RebootStatus, RegWriteStatus, Status, SyncReadStatus, WriteStatus,
};
use super::status_error::StatusError;

/// Decode a Status-instruction frame into a typed [`Status`], given the
/// instruction of the request that preceded it (callers track this from
/// their own request log; build it with [`Instruction::from_u8`]).
///
/// - Native single-slot instructions (Ping/Read/Write/RegWrite/Action/Reboot/
///   SyncRead/BulkRead) decode to their typed variants. A nonzero error byte
///   demotes any of these to [`Status::Error`].
/// - Native Fast Sync/Bulk Read responses decode to [`Status::FastSyncRead`] /
///   [`Status::FastBulkRead`] — call the variant's `slots()` to walk per-slot
///   data.
/// - [`Instruction::Ext`] routes to [`StatusExt::decode`]; `None` from the
///   extension surfaces as [`DecodeError::UnknownInstruction`].
pub fn decode_status<'a, S: StatusExt>(
    instr: Instruction,
    raw: RawStatus<'a>,
) -> Result<Status<'a, S>, DecodeError> {
    if let Instruction::Ext(b) = instr {
        return match S::decode(b, raw) {
            Some(Ok(v)) => Ok(Status::Ext(v)),
            Some(Err(e)) => Err(e),
            None => Err(DecodeError::UnknownInstruction),
        };
    }

    // Slot 0's error byte is the frame error byte (per spec). Fast Sync/Bulk
    // Read responses defer error handling to the per-slot view, so we keep
    // raw bytes for those and route here.
    if let Some(fast) = decode_fast(instr, raw) {
        return Ok(fast);
    }

    // For non-Fast natives, a nonzero error means the request failed; demote
    // the typed shape to Status::Error with the typed StatusError code.
    if raw.error != StatusError::None.as_u8() {
        let error = StatusError::from_u8(raw.error).ok_or(DecodeError::BadParams)?;
        return Ok(Status::Error(ErrorStatus { id: raw.id, error }));
    }

    decode_success(instr, raw)
}

fn decode_fast<'a, S: StatusExt>(instr: Instruction, raw: RawStatus<'a>) -> Option<Status<'a, S>> {
    match instr {
        Instruction::FastSyncRead => Some(Status::FastSyncRead(FastSyncReadStatus::from_raw(raw))),
        Instruction::FastBulkRead => Some(Status::FastBulkRead(FastBulkReadStatus::from_raw(raw))),
        _ => None,
    }
}

fn decode_success<'a, S: StatusExt>(
    instr: Instruction,
    raw: RawStatus<'a>,
) -> Result<Status<'a, S>, DecodeError> {
    let id = raw.id;
    match instr {
        Instruction::Ping => {
            let mut iter = raw.params.iter();
            let lo = iter.next().ok_or(DecodeError::BadParams)?;
            let hi = iter.next().ok_or(DecodeError::BadParams)?;
            let firmware = iter.next().ok_or(DecodeError::BadParams)?;
            if iter.next().is_some() {
                return Err(DecodeError::BadParams);
            }
            Ok(Status::Ping(PingStatus {
                id,
                model: u16::from_le_bytes([lo, hi]),
                firmware,
            }))
        }
        Instruction::Read => Ok(Status::Read(ReadStatus {
            id,
            data: raw.params,
        })),
        Instruction::SyncRead => Ok(Status::SyncRead(SyncReadStatus {
            id,
            data: raw.params,
        })),
        Instruction::BulkRead => Ok(Status::BulkRead(BulkReadStatus {
            id,
            data: raw.params,
        })),
        Instruction::Write => empty_params(&raw).map(|_| Status::Write(WriteStatus { id })),
        Instruction::RegWrite => {
            empty_params(&raw).map(|_| Status::RegWrite(RegWriteStatus { id }))
        }
        Instruction::Action => empty_params(&raw).map(|_| Status::Action(ActionStatus { id })),
        Instruction::Reboot => empty_params(&raw).map(|_| Status::Reboot(RebootStatus { id })),
        // Fast Sync/Bulk handled by decode_fast above; nothing else here is
        // a known status-producing instruction.
        _ => Err(DecodeError::UnknownInstruction),
    }
}

fn empty_params(raw: &RawStatus<'_>) -> Result<(), DecodeError> {
    if raw.params.iter().next().is_some() {
        return Err(DecodeError::BadParams);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::Bytes;

    #[derive(Copy, Clone, Debug)]
    struct NoExt;
    impl StatusExt for NoExt {
        type Variant<'a> = core::convert::Infallible;
        fn decode<'a>(_: u8, _: RawStatus<'a>) -> Option<Result<Self::Variant<'a>, DecodeError>> {
            None
        }
        fn write<'a, W: crate::wire::WriteBuf, CRC: crate::wire::CrcUmts>(
            v: &Self::Variant<'a>,
            _: &mut W,
        ) -> Result<(), crate::wire::WriteError> {
            match *v {}
        }
    }

    #[test]
    fn ping_decodes_to_typed_ping_status() {
        let raw = RawStatus {
            id: 7,
            error: 0,
            params: Bytes::unstuffed(&[0x10, 0x20, 0x03]),
        };
        let s = decode_status::<NoExt>(Instruction::Ping, raw).unwrap();
        match s {
            Status::Ping(p) => {
                assert_eq!(p.id, 7);
                assert_eq!(p.model, 0x2010);
                assert_eq!(p.firmware, 0x03);
            }
            other => panic!("not Ping: {other:?}"),
        }
    }

    #[test]
    fn read_with_nonzero_error_demotes_to_error_status() {
        let raw = RawStatus {
            id: 7,
            error: StatusError::DataRange.as_u8(),
            params: Bytes::unstuffed(&[]),
        };
        let s = decode_status::<NoExt>(Instruction::Read, raw).unwrap();
        match s {
            Status::Error(e) => {
                assert_eq!(e.id, 7);
                assert_eq!(e.error, StatusError::DataRange);
            }
            other => panic!("not Error: {other:?}"),
        }
    }

    #[test]
    fn read_decodes_to_typed_read_status() {
        let data = [0xAAu8, 0xBB, 0xCC, 0xDD];
        let raw = RawStatus {
            id: 7,
            error: 0,
            params: Bytes::unstuffed(&data),
        };
        let s = decode_status::<NoExt>(Instruction::Read, raw).unwrap();
        match s {
            Status::Read(r) => {
                assert_eq!(r.id, 7);
                let collected: heapless::Vec<u8, 8> = r.data.iter().collect();
                assert_eq!(&collected[..], &data);
            }
            other => panic!("not Read: {other:?}"),
        }
    }

    #[test]
    fn fast_sync_dispatches_to_typed_view() {
        let payload = [10, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 20, 0xB0, 0xB1, 0xB2, 0xB3];
        let raw = RawStatus {
            id: crate::wire::BROADCAST_ID,
            error: 0,
            params: Bytes::unstuffed(&payload),
        };
        let s = decode_status::<NoExt>(Instruction::FastSyncRead, raw).unwrap();
        match s {
            Status::FastSyncRead(fast) => {
                let n = fast.slots(4).count();
                assert_eq!(n, 2);
            }
            other => panic!("not FastSyncRead: {other:?}"),
        }
    }

    #[derive(Copy, Clone, Debug)]
    #[allow(dead_code)]
    enum ExtVariant<'a> {
        Calibrate { id: u8, axis: u8, payload: &'a [u8] },
    }

    #[derive(Copy, Clone, Debug)]
    struct OscExt;
    impl StatusExt for OscExt {
        type Variant<'a> = ExtVariant<'a>;
        fn decode<'a>(
            instr: u8,
            raw: RawStatus<'a>,
        ) -> Option<Result<Self::Variant<'a>, DecodeError>> {
            if instr != 0xE0 {
                return None;
            }
            let mut it = raw.params.iter();
            let axis = match it.next() {
                Some(b) => b,
                None => return Some(Err(DecodeError::BadParams)),
            };
            // Rest of the bytes — for this test we just snapshot len.
            let _ = it.count();
            Some(Ok(ExtVariant::Calibrate {
                id: raw.id,
                axis,
                payload: &[],
            }))
        }
        fn write<'a, W: crate::wire::WriteBuf, CRC: crate::wire::CrcUmts>(
            _: &Self::Variant<'a>,
            _: &mut W,
        ) -> Result<(), crate::wire::WriteError> {
            Ok(())
        }
    }

    #[test]
    fn ext_instruction_dispatches_to_status_ext_decode() {
        let raw = RawStatus {
            id: 9,
            error: 0,
            params: Bytes::unstuffed(&[0x02, 0xFF]),
        };
        let s = decode_status::<OscExt>(Instruction::from_u8(0xE0), raw).unwrap();
        match s {
            Status::Ext(ExtVariant::Calibrate { id, axis, .. }) => {
                assert_eq!(id, 9);
                assert_eq!(axis, 0x02);
            }
            other => panic!("not Ext::Calibrate: {other:?}"),
        }
    }

    #[test]
    fn ext_instruction_unclaimed_surfaces_unknown_instruction() {
        let raw = RawStatus {
            id: 1,
            error: 0,
            params: Bytes::unstuffed(&[]),
        };
        // 0xE1 isn't in OscExt's claimed set; ext returns None → UnknownInstruction.
        let err = decode_status::<OscExt>(Instruction::from_u8(0xE1), raw).unwrap_err();
        assert!(matches!(err, DecodeError::UnknownInstruction));
    }
}
