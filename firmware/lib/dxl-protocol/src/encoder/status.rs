//! Status reply encoder (slave -> master).

use crate::buf::{Chunk, WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::types::{Id, Instruction, Status, StatusError};

use super::{emit_frame, emit_frame_with};

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

    /// Status::Read body from a chunk iterator: skips the dispatcher's
    /// scratch buffer by stuffing each `Chunk::Slice` / `Chunk::Zero` run
    /// straight from its source into the output. Frame layout (header +
    /// error + stuffed body + CRC) matches [`Self::read`].
    pub fn read_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        let err_byte = error.as_byte();
        emit_frame_with(
            self.out,
            &mut self.crc,
            id,
            Instruction::Status.as_u8(),
            |out, stuffer| {
                stuffer.push(out, err_byte)?;
                for chunk in chunks {
                    match chunk {
                        Chunk::Slice(s) => stuffer.push_slice(out, s)?,
                        Chunk::Zero(n) => stuffer.push_zero(out, n)?,
                    }
                }
                Ok(())
            },
        )
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
mod tests {
    use super::*;
    use crate::test_util::{Crc, assert_crc_good, parse, read_data, status_header};
    use crate::types::PingStatus;
    use heapless::Vec;

    type Buf = Vec<u8, 256>;

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
    fn status_read_chunked_matches_data_slice_path() {
        // For any (slice/zero) chunk sequence whose payloads concatenate to
        // the same bytes the `read(&[u8])` path takes, the encoded wire frame
        // must be identical.
        let payload = [0x10, 0x00, 0x00, 0x40, 0xFF, 0xFF, 0xFD];
        let mut by_slice = Buf::new();
        StatusEncoder::<_, Crc>::new(&mut by_slice)
            .read(Id::new(0x04), StatusError::OK, &payload)
            .unwrap();

        let mut by_chunks = Buf::new();
        StatusEncoder::<_, Crc>::new(&mut by_chunks)
            .read_chunked(
                Id::new(0x04),
                StatusError::OK,
                [
                    Chunk::Slice(&payload[..1]),
                    Chunk::Zero(2),
                    Chunk::Slice(&payload[3..]),
                ],
            )
            .unwrap();

        assert_eq!(by_slice.as_slice(), by_chunks.as_slice());
    }

    #[test]
    fn status_read_chunked_round_trips() {
        let mut buf = Buf::new();
        StatusEncoder::<_, Crc>::new(&mut buf)
            .read_chunked(
                Id::new(0x09),
                StatusError::OK,
                [Chunk::Slice(&[0x10, 0x20]), Chunk::Zero(2)],
            )
            .unwrap();
        let evs = parse(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::new(0x09));
        assert_eq!(read_data(&buf, &evs).as_slice(), &[0x10, 0x20, 0, 0]);
        assert_crc_good(&evs);
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
