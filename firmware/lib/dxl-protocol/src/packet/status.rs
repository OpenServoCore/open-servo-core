//! Status reply types — overlay header, error byte, request-context dispatch,
//! and per-instruction reply payload overlays.

#![allow(dead_code)]

use super::header::{Header, U16Le};

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct StatusHeader {
    pub header: Header,
    pub error: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct StatusPacket<'a> {
    pub header: &'a StatusHeader,
    pub params: &'a [u8],
}

// ───── status error byte ─────

/// DXL 2.0 Status error byte. Bit 7 = sticky hardware Alert; bits 0..=6 =
/// error code (only `0..=7` are spec-defined). `#[repr(transparent)]` over
/// `u8` keeps wire layout identical and lets it sit inside `#[repr(C)]`
/// overlay structs.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StatusError(pub u8);

impl StatusError {
    pub const OK: Self = Self(0);
    pub const ALERT: u8 = 0x80;
    pub const CODE_MASK: u8 = 0x7F;

    #[inline]
    pub const fn from_byte(b: u8) -> Self {
        Self(b)
    }

    #[inline]
    pub const fn as_byte(self) -> u8 {
        self.0
    }

    #[inline]
    pub const fn new(alert: bool, code: ErrorCode) -> Self {
        let alert_bit = if alert { Self::ALERT } else { 0 };
        Self(alert_bit | code as u8)
    }

    /// Shorthand for `new(false, code)`.
    #[inline]
    pub const fn code(code: ErrorCode) -> Self {
        Self(code as u8)
    }

    #[inline]
    pub const fn alert(self) -> bool {
        self.0 & Self::ALERT != 0
    }

    #[inline]
    pub const fn raw_code(self) -> u8 {
        self.0 & Self::CODE_MASK
    }

    /// Typed view of bits 0..=6; `None` for reserved/unknown values.
    #[inline]
    pub const fn kind(self) -> Option<ErrorCode> {
        ErrorCode::from_u8(self.raw_code())
    }

    #[inline]
    pub const fn is_ok(self) -> bool {
        self.0 == 0
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ErrorCode {
    None = 0x00,
    Result = 0x01,
    Instruction = 0x02,
    Crc = 0x03,
    DataRange = 0x04,
    DataLength = 0x05,
    DataLimit = 0x06,
    Access = 0x07,
}

impl ErrorCode {
    #[inline]
    pub const fn from_u8(b: u8) -> Option<Self> {
        Some(match b {
            0x00 => Self::None,
            0x01 => Self::Result,
            0x02 => Self::Instruction,
            0x03 => Self::Crc,
            0x04 => Self::DataRange,
            0x05 => Self::DataLength,
            0x06 => Self::DataLimit,
            0x07 => Self::Access,
            _ => return None,
        })
    }
}

// ───── status payload overlays ─────

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PingStatus {
    pub model: U16Le,
    pub fw_version: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadStatus<'a> {
    pub error: StatusError,
    pub payload: &'a [u8],
}

impl<'a> FastSyncReadStatus<'a> {
    pub fn slots(&self, slot_length: u16) -> FastSyncSlotIter<'a> {
        FastSyncSlotIter {
            cursor: self.payload,
            slot_length: slot_length as usize,
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadStatus<'a> {
    pub error: StatusError,
    pub payload: &'a [u8],
}

impl<'a> FastBulkReadStatus<'a> {
    pub fn slots<L: IntoIterator<Item = u16>>(
        &self,
        lengths: L,
    ) -> FastBulkSlotIter<'a, L::IntoIter> {
        FastBulkSlotIter {
            cursor: self.payload,
            lengths: lengths.into_iter(),
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Slot<'a> {
    pub id: u8,
    pub error: StatusError,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncSlotIter<'a> {
    cursor: &'a [u8],
    slot_length: usize,
    slot0_error: StatusError,
    slot_index: usize,
}

impl<'a> Iterator for FastSyncSlotIter<'a> {
    type Item = Slot<'a>;
    fn next(&mut self) -> Option<Slot<'a>> {
        next_slot(
            &mut self.cursor,
            self.slot_length,
            self.slot0_error,
            &mut self.slot_index,
        )
    }
}

pub struct FastBulkSlotIter<'a, L: Iterator<Item = u16>> {
    cursor: &'a [u8],
    lengths: L,
    slot0_error: StatusError,
    slot_index: usize,
}

impl<'a, L: Iterator<Item = u16>> Iterator for FastBulkSlotIter<'a, L> {
    type Item = Slot<'a>;
    fn next(&mut self) -> Option<Slot<'a>> {
        let len = self.lengths.next()? as usize;
        next_slot(
            &mut self.cursor,
            len,
            self.slot0_error,
            &mut self.slot_index,
        )
    }
}

fn next_slot<'a>(
    cursor: &mut &'a [u8],
    slot_length: usize,
    slot0_error: StatusError,
    slot_index: &mut usize,
) -> Option<Slot<'a>> {
    let (id, error) = if *slot_index == 0 {
        if cursor.is_empty() {
            return None;
        }
        let (id, rest) = cursor.split_first()?;
        *cursor = rest;
        (*id, slot0_error)
    } else {
        if cursor.len() < 2 {
            return None;
        }
        let error = StatusError::from_byte(cursor[0]);
        let id = cursor[1];
        *cursor = &cursor[2..];
        (id, error)
    };
    if cursor.len() < slot_length {
        return None;
    }
    let (data, rest) = cursor.split_at(slot_length);
    *cursor = rest;
    *slot_index += 1;
    Some(Slot { id, error, data })
}

// ───── status-context dispatch ─────

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RequestKind {
    Ping,
    Read,
    Write,
    RegWrite,
    Action,
    Reboot,
    FactoryReset,
    SyncRead,
    SyncWrite,
    BulkRead,
    BulkWrite,
    FastSyncRead,
    FastBulkRead,
}

/// Unified Status — same shape for parser output (`StatusPacket::interpret`)
/// and writer input (encode side). `id` and `error` are baked into each
/// variant so the encoder can construct a Status without a parent
/// `StatusHeader` to borrow from.
#[derive(Copy, Clone, Debug)]
pub enum Status<'a> {
    /// Write-style ack — no payload. Used for Write/RegWrite/Action/
    /// Reboot/FactoryReset/SyncWrite/BulkWrite replies; also the
    /// short-payload-Ping fallback. `error == StatusError::OK` on success.
    Empty { id: u8, error: StatusError },

    /// Ping reply — 3-byte fixed-shape payload. Owned (not borrowed) so
    /// the same variant covers encode and decode.
    Ping {
        id: u8,
        error: StatusError,
        status: PingStatus,
    },

    /// Read/SyncRead/BulkRead reply — opaque register bytes.
    Read {
        id: u8,
        error: StatusError,
        data: &'a [u8],
    },

    /// Fast Sync Read coalesced reply (master-side decode). `status` holds
    /// slot 0's error byte and the multi-slot payload; iterate via
    /// `status.slots(slot_length)`. Writer rejects this variant — fast
    /// replies are emitted one slot at a time via `Writer::write_slot`.
    FastSyncRead {
        id: u8,
        status: FastSyncReadStatus<'a>,
    },

    /// Fast Bulk Read counterpart.
    FastBulkRead {
        id: u8,
        status: FastBulkReadStatus<'a>,
    },

    /// Encode-only escape hatch — arbitrary Status frame with chip-defined
    /// payload bytes (e.g. OSC `Calibrate` reply). `interpret()` never
    /// produces this variant.
    Raw {
        id: u8,
        error: StatusError,
        payload: &'a [u8],
    },
}

impl<'a> StatusPacket<'a> {
    #[inline]
    pub fn error(&self) -> StatusError {
        StatusError::from_byte(self.header.error)
    }

    pub fn interpret(self, req: RequestKind) -> Status<'a> {
        let id = self.header.header.id;
        let error = self.error();
        match req {
            RequestKind::Write
            | RequestKind::RegWrite
            | RequestKind::Action
            | RequestKind::Reboot
            | RequestKind::FactoryReset
            | RequestKind::SyncWrite
            | RequestKind::BulkWrite => Status::Empty { id, error },

            RequestKind::Ping => {
                if self.params.len() < core::mem::size_of::<PingStatus>() {
                    Status::Empty { id, error }
                } else {
                    // SAFETY: PingStatus is repr(C), align 1, all u8/U16Le
                    // fields. Length checked above; `self.params` is
                    // initialized for its full length.
                    let p = unsafe { *(self.params.as_ptr() as *const PingStatus) };
                    Status::Ping {
                        id,
                        error,
                        status: p,
                    }
                }
            }

            RequestKind::Read | RequestKind::SyncRead | RequestKind::BulkRead => Status::Read {
                id,
                error,
                data: self.params,
            },

            RequestKind::FastSyncRead => Status::FastSyncRead {
                id,
                status: FastSyncReadStatus {
                    error,
                    payload: self.params,
                },
            },

            RequestKind::FastBulkRead => Status::FastBulkRead {
                id,
                status: FastBulkReadStatus {
                    error,
                    payload: self.params,
                },
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::instruction::Instruction;
    use core::mem::{align_of, offset_of, size_of};

    #[test]
    fn status_layouts() {
        assert_eq!(align_of::<StatusHeader>(), 1);
        assert_eq!(align_of::<PingStatus>(), 1);
        assert_eq!(size_of::<StatusHeader>(), 9);
        assert_eq!(size_of::<PingStatus>(), 3);
    }

    #[test]
    fn status_header_offsets() {
        assert_eq!(offset_of!(StatusHeader, header), 0);
        assert_eq!(offset_of!(StatusHeader, error), 8);
    }

    #[test]
    fn ping_status_offsets() {
        assert_eq!(offset_of!(PingStatus, model), 0);
        assert_eq!(offset_of!(PingStatus, fw_version), 2);
    }

    fn make_status(buf: &[u8]) -> StatusPacket<'_> {
        let header: &StatusHeader = unsafe { &*(buf.as_ptr() as *const StatusHeader) };
        StatusPacket {
            header,
            params: &buf[size_of::<StatusHeader>()..],
        }
    }

    #[test]
    fn interpret_ping_decodes_overlay() {
        let buf: [u8; 9 + 3] = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x06, 0x00, Instruction::Status.as_u8(), 0x00, //
            0xFC, 0x03, 0x2A, // model=1020, fw=0x2A
        ];
        let s = make_status(&buf);
        assert_eq!(s.error(), StatusError::OK);
        match s.interpret(RequestKind::Ping) {
            Status::Ping { id, error, status } => {
                assert_eq!(id, 0x01);
                assert_eq!(error, StatusError::OK);
                assert_eq!(status.model.get(), 1020);
                assert_eq!(status.fw_version, 0x2A);
            }
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn interpret_ping_short_payload_falls_back_to_empty() {
        let buf: [u8; 9 + 1] = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, Instruction::Status.as_u8(), 0x00, 0xFC,
        ];
        let s = make_status(&buf);
        assert!(matches!(
            s.interpret(RequestKind::Ping),
            Status::Empty { .. }
        ));
    }

    #[test]
    fn interpret_read_wraps_payload_slice() {
        let buf: [u8; 9 + 4] = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, Instruction::Status.as_u8(), 0x00, //
            0x01, 0x02, 0x03, 0x04,
        ];
        let s = make_status(&buf);
        for req in [
            RequestKind::Read,
            RequestKind::SyncRead,
            RequestKind::BulkRead,
        ] {
            match s.interpret(req) {
                Status::Read { id, error, data } => {
                    assert_eq!(id, 0x01);
                    assert_eq!(error, StatusError::OK);
                    assert_eq!(data, &[0x01, 0x02, 0x03, 0x04]);
                }
                other => panic!("{req:?} expected Read, got {other:?}"),
            }
        }
    }

    #[test]
    fn interpret_write_family_is_empty() {
        let buf: [u8; 9] = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, Instruction::Status.as_u8(), 0x00,
        ];
        let s = make_status(&buf);
        for req in [
            RequestKind::Write,
            RequestKind::RegWrite,
            RequestKind::Action,
            RequestKind::Reboot,
            RequestKind::FactoryReset,
            RequestKind::SyncWrite,
            RequestKind::BulkWrite,
        ] {
            assert!(
                matches!(s.interpret(req), Status::Empty { .. }),
                "{req:?} should produce Status::Empty"
            );
        }
    }

    #[test]
    fn interpret_fast_sync_carries_error_and_payload() {
        let buf: [u8; 9 + 5] = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, Instruction::Status.as_u8(), 0x07, //
            10, 0xAA, 0xBB, 0xCC, 0xDD,
        ];
        let s = make_status(&buf);
        match s.interpret(RequestKind::FastSyncRead) {
            Status::FastSyncRead { id, status } => {
                assert_eq!(id, 0x01);
                assert_eq!(status.error, StatusError::code(ErrorCode::Access));
                assert_eq!(status.payload, &[10, 0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected FastSyncRead, got {other:?}"),
        }
    }

    #[test]
    fn interpret_fast_bulk_carries_error_and_payload() {
        let buf: [u8; 9 + 5] = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, Instruction::Status.as_u8(), 0x07, //
            10, 0xAA, 0xBB, 0xCC, 0xDD,
        ];
        let s = make_status(&buf);
        match s.interpret(RequestKind::FastBulkRead) {
            Status::FastBulkRead { id, status } => {
                assert_eq!(id, 0x01);
                assert_eq!(status.error, StatusError::code(ErrorCode::Access));
                assert_eq!(status.payload, &[10, 0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected FastBulkRead, got {other:?}"),
        }
    }

    #[test]
    fn fast_sync_status_slots_walk_uniform_length() {
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, //
            0x02, 20, 0xB0, 0xB1, 0xB2, 0xB3, //
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3,
        ];
        let s = FastSyncReadStatus {
            error: StatusError::from_byte(0x01),
            payload: &payload,
        };
        let collected: heapless::Vec<(u8, StatusError, &[u8]), 4> =
            s.slots(4).map(|sl| (sl.id, sl.error, sl.data)).collect();
        assert_eq!(collected.len(), 3);
        assert_eq!(
            collected[0],
            (
                10,
                StatusError::from_byte(0x01),
                &[0xA0, 0xA1, 0xA2, 0xA3][..]
            )
        );
        assert_eq!(
            collected[1],
            (
                20,
                StatusError::from_byte(0x02),
                &[0xB0, 0xB1, 0xB2, 0xB3][..]
            )
        );
        assert_eq!(
            collected[2],
            (30, StatusError::OK, &[0xC0, 0xC1, 0xC2, 0xC3][..])
        );
    }

    #[test]
    fn fast_sync_status_slots_stop_on_truncation() {
        let payload = [10, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 20, 0xB0];
        let s = FastSyncReadStatus {
            error: StatusError::OK,
            payload: &payload,
        };
        assert_eq!(s.slots(4).count(), 1);
    }

    #[test]
    fn fast_bulk_status_slots_varying_lengths() {
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, //
            0x02, 20, 0xB0, 0xB1, //
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,
        ];
        let s = FastBulkReadStatus {
            error: StatusError::from_byte(0x05),
            payload: &payload,
        };
        let collected: heapless::Vec<(u8, StatusError, usize), 4> = s
            .slots([4u16, 2, 6].iter().copied())
            .map(|sl| (sl.id, sl.error, sl.data.len()))
            .collect();
        assert_eq!(collected.len(), 3);
        assert_eq!(collected[0], (10, StatusError::from_byte(0x05), 4));
        assert_eq!(collected[1], (20, StatusError::from_byte(0x02), 2));
        assert_eq!(collected[2], (30, StatusError::OK, 6));
    }
}
