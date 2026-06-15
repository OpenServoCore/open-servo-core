//! Status reply encoder (slave -> master).

use crate::buf::{WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::types::{Id, Instruction, Status, StatusError};

use super::emit_frame;

pub struct StatusEncoder<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    crc: CRC,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> StatusEncoder<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            crc: CRC::new(),
        }
    }

    pub fn empty(&mut self, id: Id, error: StatusError) -> Result<(), WriteError> {
        self.frame(id, error, &[])
    }

    pub fn ping(
        &mut self,
        id: Id,
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

    pub fn read(&mut self, id: Id, error: StatusError, data: &[u8]) -> Result<(), WriteError> {
        self.frame(id, error, data)
    }

    /// Vendor-extension escape hatch - Status reply with a chip-defined
    /// payload (e.g. OSC `Calibrate` reply).
    pub fn ext(&mut self, id: Id, error: StatusError, payload: &[u8]) -> Result<(), WriteError> {
        self.frame(id, error, payload)
    }

    /// Dispatch on the unified [`Status`] enum.
    ///
    /// Fast Sync/Bulk Read variants emit one Status frame carrying the
    /// coalesced multi-slot payload (the byte shape a master decodes).
    /// Slaves participating in a chain reply use [`super::SlotEncoder`]
    /// instead; this path is for relays, sniffers, or single-slave-with-
    /// all-data masters.
    pub fn emit(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        match status {
            Status::Empty { id, error } => self.empty(id, error),
            Status::Ping { id, error, status } => {
                self.ping(id, error, status.model, status.fw_version)
            }
            Status::Read { id, error, data } => self.read(id, error, data),
            Status::Raw { id, error, payload } => self.ext(id, error, payload),
            Status::FastSyncRead { id, error, payload } => self.frame(id, error, payload),
            Status::FastBulkRead { id, error, payload } => self.frame(id, error, payload),
        }
    }

    fn frame(&mut self, id: Id, error: StatusError, payload: &[u8]) -> Result<(), WriteError> {
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
extern crate alloc;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::SoftwareCrcUmts;
    use crate::streaming::{
        Event, HeaderEvent, Parser, PayloadEvent, StatusHeader as SH, StatusPayload,
    };
    use crate::types::PingStatus;
    use alloc::vec::Vec as AVec;
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    fn parse(wire: &[u8]) -> AVec<Event> {
        let mut p: Parser<Crc> = Parser::new();
        p.feed(wire).collect()
    }

    fn status_header(events: &[Event]) -> SH {
        events
            .iter()
            .find_map(|e| match e {
                Event::Header(HeaderEvent::Status(h)) => Some(*h),
                _ => None,
            })
            .expect("expected status header event")
    }

    fn read_data(wire: &[u8], events: &[Event]) -> AVec<u8> {
        let mut out = AVec::new();
        for ev in events {
            if let Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) = ev
            {
                out.extend_from_slice(&wire[*offset as usize..(*offset + *length) as usize]);
            }
        }
        out
    }

    fn assert_crc_good(events: &[Event]) {
        assert!(events.iter().any(|e| matches!(e, Event::Crc)));
    }

    #[test]
    fn status_empty_round_trips() {
        let mut buf = Buf::new();
        StatusEncoder::<_, Crc>::new(&mut buf)
            .empty(Id::new(0x01), StatusError::OK)
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x01));
        assert_eq!(h.error, StatusError::OK);
        assert_eq!(h.length, 0);
        assert_crc_good(&evs);
    }

    #[test]
    fn status_empty_carries_error_byte() {
        let mut buf = Buf::new();
        let err = StatusError::from_byte(0x83);
        StatusEncoder::<_, Crc>::new(&mut buf)
            .empty(Id::new(0x02), err)
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x02));
        assert_eq!(h.error, err);
        assert_crc_good(&evs);
    }

    #[test]
    fn status_ping_round_trips() {
        let mut buf = Buf::new();
        StatusEncoder::<_, Crc>::new(&mut buf)
            .ping(Id::new(0x03), StatusError::OK, 0x0203, 0x10)
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x03));
        assert_eq!(h.error, StatusError::OK);
        assert_eq!(read_data(&buf, &evs).as_slice(), &[0x03, 0x02, 0x10]);
        assert_crc_good(&evs);
    }

    #[test]
    fn status_read_round_trips() {
        let mut buf = Buf::new();
        let data = [0x10, 0x20, 0x30, 0x40];
        StatusEncoder::<_, Crc>::new(&mut buf)
            .read(Id::new(0x04), StatusError::OK, &data)
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x04));
        assert_eq!(read_data(&buf, &evs).as_slice(), &data);
        assert_crc_good(&evs);
    }

    #[test]
    fn status_ext_round_trips() {
        let mut buf = Buf::new();
        let payload = [0xCA, 0xFE, 0xBA, 0xBE];
        StatusEncoder::<_, Crc>::new(&mut buf)
            .ext(Id::new(0x05), StatusError::OK, &payload)
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x05));
        assert_eq!(read_data(&buf, &evs).as_slice(), &payload);
        assert_crc_good(&evs);
    }

    #[test]
    fn status_emit_empty_round_trips() {
        let mut buf = Buf::new();
        let err = StatusError::from_byte(0x83);
        StatusEncoder::<_, Crc>::new(&mut buf)
            .emit(Status::Empty {
                id: Id::new(0x05),
                error: err,
            })
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x05));
        assert_eq!(h.error, err);
        assert_eq!(h.length, 0);
        assert_crc_good(&evs);
    }

    #[test]
    fn status_emit_ping_round_trips() {
        let mut buf = Buf::new();
        StatusEncoder::<_, Crc>::new(&mut buf)
            .emit(Status::Ping {
                id: Id::new(0x07),
                error: StatusError::OK,
                status: PingStatus {
                    model: 0x1234,
                    fw_version: 0x42,
                },
            })
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x07));
        assert_eq!(read_data(&buf, &evs).as_slice(), &[0x34, 0x12, 0x42]);
        assert_crc_good(&evs);
    }

    #[test]
    fn status_emit_read_round_trips() {
        let mut buf = Buf::new();
        let data = [0x10, 0x20, 0x30, 0x40];
        StatusEncoder::<_, Crc>::new(&mut buf)
            .emit(Status::Read {
                id: Id::new(0x09),
                error: StatusError::OK,
                data: &data,
            })
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x09));
        assert_eq!(read_data(&buf, &evs).as_slice(), &data);
    }

    #[test]
    fn status_emit_raw_round_trips() {
        let mut buf = Buf::new();
        let payload = [0xCA, 0xFE, 0xBA, 0xBE];
        StatusEncoder::<_, Crc>::new(&mut buf)
            .emit(Status::Raw {
                id: Id::new(0x0B),
                error: StatusError::OK,
                payload: &payload,
            })
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x0B));
        assert_eq!(read_data(&buf, &evs).as_slice(), &payload);
    }

    #[test]
    fn status_emit_fast_sync_read_emits_coalesced_payload() {
        let mut buf = Buf::new();
        let payload = [10, 0xAA, 0xBB, 0x00, 20, 0xCC, 0xDD];
        StatusEncoder::<_, Crc>::new(&mut buf)
            .emit(Status::FastSyncRead {
                id: Id::BROADCAST,
                error: StatusError::OK,
                payload: &payload,
            })
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::BROADCAST);
        assert_eq!(h.error, StatusError::OK);
        assert_eq!(read_data(&buf, &evs).as_slice(), &payload);
        assert_crc_good(&evs);
    }
}
