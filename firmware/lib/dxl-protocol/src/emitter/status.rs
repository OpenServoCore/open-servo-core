//! Status reply emitter (slave -> master).

#![allow(dead_code)]

use crate::buf::{WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::packet::{Instruction, Status, StatusError};

use super::emit_frame;

pub struct StatusEmitter<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    crc: CRC,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> StatusEmitter<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            crc: CRC::new(),
        }
    }

    pub fn empty(&mut self, id: u8, error: StatusError) -> Result<(), WriteError> {
        self.frame(id, error, &[])
    }

    pub fn ping(
        &mut self,
        id: u8,
        error: StatusError,
        model: u16,
        fw_version: u8,
    ) -> Result<(), WriteError> {
        let m = model.to_le_bytes();
        let err = [error.as_byte()];
        let fw = [fw_version];
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::Status.as_u8(),
            &[&err, &m, &fw],
        )
    }

    pub fn read(&mut self, id: u8, error: StatusError, data: &[u8]) -> Result<(), WriteError> {
        self.frame(id, error, data)
    }

    /// Vendor-extension escape hatch - Status reply with a chip-defined
    /// payload (e.g. OSC `Calibrate` reply).
    pub fn ext(&mut self, id: u8, error: StatusError, payload: &[u8]) -> Result<(), WriteError> {
        self.frame(id, error, payload)
    }

    /// Dispatch on the unified [`Status`] enum.
    ///
    /// Fast Sync/Bulk Read variants emit one Status frame carrying the
    /// coalesced multi-slot payload (the byte shape a master decodes).
    /// Slaves participating in a chain reply use [`super::SlotEmitter`]
    /// instead; this path is for relays, sniffers, or single-slave-with-
    /// all-data masters.
    pub fn emit(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        match status {
            Status::Empty { id, error } => self.empty(id, error),
            Status::Ping { id, error, status } => {
                self.ping(id, error, status.model.get(), status.fw_version)
            }
            Status::Read { id, error, data } => self.read(id, error, data),
            Status::Raw { id, error, payload } => self.ext(id, error, payload),
            Status::FastSyncRead { id, status } => self.frame(id, status.error, status.payload),
            Status::FastBulkRead { id, status } => self.frame(id, status.error, status.payload),
        }
    }

    fn frame(&mut self, id: u8, error: StatusError, payload: &[u8]) -> Result<(), WriteError> {
        let err = [error.as_byte()];
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::Status.as_u8(),
            &[&err, payload],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::BROADCAST_ID;
    use crate::crc_software::SoftwareCrcUmts;
    use crate::decoder::{Decoder, Step};
    use crate::packet::{FastSyncReadStatus, Packet, PingStatus, RequestKind, U16Le};
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    #[test]
    fn status_empty_round_trips() {
        let mut buf = Buf::new();
        StatusEmitter::<_, Crc>::new(&mut buf)
            .empty(0x01, StatusError::OK)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x01);
                assert_eq!(s.error(), StatusError::OK);
                assert_eq!(s.params, &[]);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_empty_carries_error_byte() {
        let mut buf = Buf::new();
        let err = StatusError::from_byte(0x83);
        StatusEmitter::<_, Crc>::new(&mut buf)
            .empty(0x02, err)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x02);
                assert_eq!(s.error(), err);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_ping_round_trips_through_interpret() {
        let mut buf = Buf::new();
        StatusEmitter::<_, Crc>::new(&mut buf)
            .ping(0x03, StatusError::OK, 0x0203, 0x10)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => match s.interpret(RequestKind::Ping) {
                Status::Ping { id, error, status } => {
                    assert_eq!(id, 0x03);
                    assert_eq!(error, StatusError::OK);
                    assert_eq!(status.model.get(), 0x0203);
                    assert_eq!(status.fw_version, 0x10);
                }
                other => panic!("expected Status::Ping, got {other:?}"),
            },
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_read_round_trips() {
        let mut buf = Buf::new();
        let data = [0x10, 0x20, 0x30, 0x40];
        StatusEmitter::<_, Crc>::new(&mut buf)
            .read(0x04, StatusError::OK, &data)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => match s.interpret(RequestKind::Read) {
                Status::Read { id, error, data: d } => {
                    assert_eq!(id, 0x04);
                    assert_eq!(error, StatusError::OK);
                    assert_eq!(d, &data);
                }
                other => panic!("expected Status::Read, got {other:?}"),
            },
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_ext_round_trips() {
        let mut buf = Buf::new();
        let payload = [0xCA, 0xFE, 0xBA, 0xBE];
        StatusEmitter::<_, Crc>::new(&mut buf)
            .ext(0x05, StatusError::OK, &payload)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x05);
                assert_eq!(s.error(), StatusError::OK);
                assert_eq!(s.params, &payload);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_emit_empty_round_trips() {
        let mut buf = Buf::new();
        let err = StatusError::from_byte(0x83);
        StatusEmitter::<_, Crc>::new(&mut buf)
            .emit(Status::Empty {
                id: 0x05,
                error: err,
            })
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x05);
                assert_eq!(s.error(), err);
                assert_eq!(s.params, &[]);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_emit_ping_round_trips_through_interpret() {
        let mut buf = Buf::new();
        StatusEmitter::<_, Crc>::new(&mut buf)
            .emit(Status::Ping {
                id: 0x07,
                error: StatusError::OK,
                status: PingStatus {
                    model: U16Le::from_u16(0x1234),
                    fw_version: 0x42,
                },
            })
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => match s.interpret(RequestKind::Ping) {
                Status::Ping { id, error, status } => {
                    assert_eq!(id, 0x07);
                    assert_eq!(error, StatusError::OK);
                    assert_eq!(status.model.get(), 0x1234);
                    assert_eq!(status.fw_version, 0x42);
                }
                other => panic!("expected Status::Ping, got {other:?}"),
            },
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_emit_read_round_trips() {
        let mut buf = Buf::new();
        let data = [0x10, 0x20, 0x30, 0x40];
        StatusEmitter::<_, Crc>::new(&mut buf)
            .emit(Status::Read {
                id: 0x09,
                error: StatusError::OK,
                data: &data,
            })
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => match s.interpret(RequestKind::Read) {
                Status::Read { id, data: d, .. } => {
                    assert_eq!(id, 0x09);
                    assert_eq!(d, &data);
                }
                other => panic!("expected Status::Read, got {other:?}"),
            },
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_emit_raw_round_trips() {
        let mut buf = Buf::new();
        let payload = [0xCA, 0xFE, 0xBA, 0xBE];
        StatusEmitter::<_, Crc>::new(&mut buf)
            .emit(Status::Raw {
                id: 0x0B,
                error: StatusError::OK,
                payload: &payload,
            })
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x0B);
                assert_eq!(s.params, &payload);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_emit_fast_sync_read_emits_coalesced_payload() {
        let mut buf = Buf::new();
        let payload = [10, 0xAA, 0xBB, 0x00, 20, 0xCC, 0xDD];
        StatusEmitter::<_, Crc>::new(&mut buf)
            .emit(Status::FastSyncRead {
                id: BROADCAST_ID,
                status: FastSyncReadStatus {
                    error: StatusError::OK,
                    payload: &payload,
                },
            })
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, BROADCAST_ID);
                assert_eq!(s.error(), StatusError::OK);
                assert_eq!(s.params, &payload);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }
}
