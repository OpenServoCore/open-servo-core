use dxl_protocol::{
    BROADCAST_ID, Bytes, Packet, ParseError, StatusError, WriteBuf, parse_one, write,
};

use crate::{RegmapError, RingReader, RxSnapshot, Shared};

pub const DXL_SCRATCH_LEN: usize = 256;
pub const MAX_READ: usize = 128;

pub trait DxlIo {
    type TxBuf: WriteBuf;

    fn rx_snapshot(&self) -> RxSnapshot<'_>;
    fn tx_buf(&mut self) -> &mut Self::TxBuf;
    fn start_tx(&mut self);
}

pub struct Dxl {
    reader: RingReader<DXL_SCRATCH_LEN>,
}

impl Dxl {
    pub const fn new() -> Self {
        Self {
            reader: RingReader::new(),
        }
    }

    pub fn poll<D: DxlIo>(&mut self, shared: &Shared, io: &mut D) {
        let snap = io.rx_snapshot();
        self.reader.ingest(snap.ring(), snap.write_pos());

        loop {
            match parse_one(self.reader.peek()) {
                Ok((packet, used)) => {
                    dispatch(shared, io, &packet);
                    self.reader.consume(used);
                }
                Err(ParseError::Incomplete) => break,
                Err(ParseError::Resync { skip })
                | Err(ParseError::BadCrc { skip })
                | Err(ParseError::BadInstruction { skip })
                | Err(ParseError::BadLength { skip }) => {
                    self.reader.consume(skip);
                }
            }
        }
    }
}

impl Default for Dxl {
    fn default() -> Self {
        Self::new()
    }
}

fn dispatch<D: DxlIo>(shared: &Shared, io: &mut D, packet: &Packet<'_>) {
    let our_id = our_id(shared);
    let target_id = packet_id(packet);

    let respond = match (packet, target_id) {
        (Packet::Ping { .. }, Some(id)) => id == our_id || id == BROADCAST_ID,
        (_, Some(id)) => id == our_id,
        (_, None) => false,
    };
    if !respond {
        return;
    }

    match packet {
        Packet::Ping { .. } => send_ping_status(shared, io, our_id),
        Packet::Read {
            address, length, ..
        } => send_read_status(shared, io, our_id, *address, *length),
        _ => send_status(io, our_id, StatusError::Instruction, &[]),
    }
}

fn our_id(shared: &Shared) -> u8 {
    // SAFETY: comms.id is a single byte; ISR contexts read but don't mutate.
    unsafe { (*shared.table.config.get()).comms.id }
}

fn packet_id(packet: &Packet<'_>) -> Option<u8> {
    match *packet {
        Packet::Ping { id }
        | Packet::Read { id, .. }
        | Packet::Write { id, .. }
        | Packet::RegWrite { id, .. }
        | Packet::Action { id }
        | Packet::FactoryReset { id, .. }
        | Packet::Reboot { id }
        | Packet::Clear { id, .. }
        | Packet::ControlTableBackup { id, .. }
        | Packet::Status { id, .. } => Some(id),
        Packet::SyncRead { .. }
        | Packet::SyncWrite { .. }
        | Packet::BulkRead { .. }
        | Packet::BulkWrite { .. }
        | Packet::FastSyncRead { .. }
        | Packet::FastBulkRead { .. } => None,
    }
}

fn send_ping_status<D: DxlIo>(shared: &Shared, io: &mut D, id: u8) {
    // SAFETY: identity is read-only after seed_config_defaults; no concurrent writers.
    let identity = unsafe { (*shared.table.config.get()).identity };
    let model = identity.model_number.to_le_bytes();
    let fw = identity.firmware_version as u8;
    let params = [model[0], model[1], fw];
    send_status(io, id, StatusError::None, &params);
}

fn send_read_status<D: DxlIo>(shared: &Shared, io: &mut D, id: u8, address: u16, length: u16) {
    let len = length as usize;
    if len == 0 || len > MAX_READ {
        send_status(io, id, StatusError::DataRange, &[]);
        return;
    }

    let mut buf = [0u8; MAX_READ];
    match shared.table.read_bytes(address, &mut buf[..len]) {
        Ok(()) => send_status(io, id, StatusError::None, &buf[..len]),
        Err(RegmapError::OutOfRange) => send_status(io, id, StatusError::DataRange, &[]),
        Err(RegmapError::AccessError) => send_status(io, id, StatusError::Access, &[]),
    }
}

fn send_status<D: DxlIo>(io: &mut D, id: u8, error: StatusError, params: &[u8]) {
    let buf = io.tx_buf();
    buf.truncate(0);
    let packet = Packet::Status {
        id,
        error: error.as_u8(),
        params: Bytes::Raw(params),
    };
    if write(buf, &packet).is_err() {
        buf.truncate(0);
        return;
    }
    io.start_tx();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Shared;
    use heapless::Vec;

    struct FakeDxlIo {
        rx_ring: [u8; 256],
        rx_write_pos: u16,
        tx: Vec<u8, 256>,
        start_tx_count: u32,
    }

    impl FakeDxlIo {
        fn new() -> Self {
            Self {
                rx_ring: [0; 256],
                rx_write_pos: 0,
                tx: Vec::new(),
                start_tx_count: 0,
            }
        }

        fn feed(&mut self, bytes: &[u8]) {
            for &b in bytes {
                let idx = (self.rx_write_pos as usize) % self.rx_ring.len();
                self.rx_ring[idx] = b;
                self.rx_write_pos = self.rx_write_pos.wrapping_add(1) % (self.rx_ring.len() as u16);
            }
        }
    }

    impl DxlIo for FakeDxlIo {
        type TxBuf = Vec<u8, 256>;

        fn rx_snapshot(&self) -> RxSnapshot<'_> {
            RxSnapshot::new(&self.rx_ring, self.rx_write_pos)
        }
        fn tx_buf(&mut self) -> &mut Self::TxBuf {
            &mut self.tx
        }
        fn start_tx(&mut self) {
            self.start_tx_count += 1;
        }
    }

    fn encode(packet: &Packet<'_>) -> Vec<u8, 64> {
        let mut buf: Vec<u8, 64> = Vec::new();
        write(&mut buf, packet).unwrap();
        buf
    }

    fn parse_status(bytes: &[u8]) -> (u8, u8, Vec<u8, 64>) {
        let (pkt, used) = parse_one(bytes).unwrap();
        assert_eq!(used, bytes.len());
        match pkt {
            Packet::Status { id, error, params } => {
                let mut out: Vec<u8, 64> = Vec::new();
                for b in params.iter() {
                    out.push(b).unwrap();
                }
                (id, error, out)
            }
            _ => panic!("not a status packet"),
        }
    }

    #[test]
    fn ping_to_our_id_replies() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Ping { id: 0 });
        io.feed(&req);
        h.poll(&shared, &mut io);

        assert_eq!(io.start_tx_count, 1);
        let (id, err, params) = parse_status(&io.tx);
        assert_eq!(id, 0);
        assert_eq!(err, 0);
        assert_eq!(&params[..], &[0, 0, 0]); // model_lo, model_hi, fw — defaults
    }

    #[test]
    fn ping_to_broadcast_replies() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Ping { id: BROADCAST_ID });
        io.feed(&req);
        h.poll(&shared, &mut io);

        assert_eq!(io.start_tx_count, 1);
        let (id, err, _) = parse_status(&io.tx);
        assert_eq!(id, 0); // we reply with our own id, not the broadcast id
        assert_eq!(err, 0);
    }

    #[test]
    fn ping_to_other_id_silent() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Ping { id: 17 });
        io.feed(&req);
        h.poll(&shared, &mut io);

        assert_eq!(io.start_tx_count, 0);
        assert!(io.tx.is_empty());
    }

    #[test]
    fn read_model_number_returns_two_bytes() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Read {
            id: 0,
            address: 0,
            length: 2,
        });
        io.feed(&req);
        h.poll(&shared, &mut io);

        let (id, err, params) = parse_status(&io.tx);
        assert_eq!(id, 0);
        assert_eq!(err, 0);
        assert_eq!(&params[..], &[0, 0]);
    }

    #[test]
    fn read_zero_length_rejects_with_data_range() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Read {
            id: 0,
            address: 0,
            length: 0,
        });
        io.feed(&req);
        h.poll(&shared, &mut io);

        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::DataRange.as_u8());
    }

    #[test]
    fn unsupported_instruction_replies_instruction_error() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Write {
            id: 0,
            address: 0,
            data: Bytes::Raw(&[0x01]),
        });
        io.feed(&req);
        h.poll(&shared, &mut io);

        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::Instruction.as_u8());
    }
}
