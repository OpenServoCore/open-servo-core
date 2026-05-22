use dxl_protocol::prelude::*;

use crate::{RegmapError, RingReader, RxSnapshot, Shared};

pub const DXL_SCRATCH_LEN: usize = 256;
pub const MAX_READ: usize = 128;
pub const MAX_WRITE: usize = 128;

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
    match packet {
        Packet::Ping(p) => handle_ping(shared, io, p),
        Packet::Read(p) => handle_read(shared, io, p),
        Packet::Write(p) => handle_write(shared, io, p),
        Packet::RegWrite(p) => handle_reg_write(shared, io, p),
        Packet::Action(p) => handle_action(shared, io, p),
        Packet::FactoryReset(p) => handle_factory_reset(shared, io, p),
        Packet::Reboot(p) => handle_reboot(shared, io, p),
        Packet::Clear(p) => handle_clear(shared, io, p),
        Packet::ControlTableBackup(p) => handle_control_table_backup(shared, io, p),
        Packet::SyncRead(p) => handle_sync_read(shared, io, p),
        Packet::SyncWrite(p) => handle_sync_write(shared, io, p),
        Packet::BulkRead(p) => handle_bulk_read(shared, io, p),
        Packet::BulkWrite(p) => handle_bulk_write(shared, io, p),
        Packet::FastSyncRead(p) => handle_fast_sync_read(shared, io, p),
        Packet::FastBulkRead(p) => handle_fast_bulk_read(shared, io, p),
        // Status frames are what we *send*; if one shows up inbound it's another
        // device on the bus — drop silently.
        Packet::Status(_) => {}
    }
}

fn addressed(shared: &Shared, target: u8) -> Option<(u8, bool)> {
    // SAFETY: comms.id is a single byte; ISR contexts read but don't mutate.
    let id = unsafe { (*shared.table.config.get()).comms.id };

    if target == id {
        Some((id, true))
    } else if target == BROADCAST_ID {
        Some((id, false))
    } else {
        None
    }
}

fn reply_unsupported<D: DxlIo>(shared: &Shared, io: &mut D, target: u8) {
    if let Some((id, true)) = addressed(shared, target) {
        send_status(io, id, StatusError::Instruction, &[]);
    }
}

fn handle_ping<D: DxlIo>(shared: &Shared, io: &mut D, p: &PingPacket) {
    let Some((id, _)) = addressed(shared, p.id) else {
        return;
    };
    // SAFETY: identity is read-only after seed_config_defaults; no concurrent writers.
    let identity = unsafe { (*shared.table.config.get()).identity };
    let model = identity.model_number.to_le_bytes();
    let fw = identity.firmware_version as u8;
    let params = [model[0], model[1], fw];
    send_status(io, id, StatusError::None, &params);
}

fn handle_read<D: DxlIo>(shared: &Shared, io: &mut D, p: &ReadPacket) {
    let Some((id, true)) = addressed(shared, p.id) else {
        return;
    };
    let len = p.length as usize;
    if len == 0 || len > MAX_READ {
        send_status(io, id, StatusError::DataRange, &[]);
        return;
    }
    let mut buf = [0u8; MAX_READ];
    match shared.table.read_bytes(p.address, &mut buf[..len]) {
        Ok(()) => send_status(io, id, StatusError::None, &buf[..len]),
        Err(RegmapError::OutOfRange) => send_status(io, id, StatusError::DataRange, &[]),
        Err(RegmapError::AccessError) => send_status(io, id, StatusError::Access, &[]),
    }
}

fn handle_write<D: DxlIo>(shared: &Shared, io: &mut D, p: &WritePacket<'_>) {
    let Some((id, direct)) = addressed(shared, p.id) else {
        return;
    };

    let mut buf = [0u8; MAX_WRITE];
    let len = match p.data.copy_into(&mut buf) {
        Ok(n) => n,
        Err(_) => {
            if direct {
                send_status(io, id, StatusError::DataRange, &[]);
            }
            return;
        }
    };

    let result = shared.table.write_bytes(p.address, &buf[..len]);
    if !direct {
        return;
    }
    match result {
        Ok(()) => send_status(io, id, StatusError::None, &[]),
        Err(RegmapError::OutOfRange) => send_status(io, id, StatusError::DataRange, &[]),
        Err(RegmapError::AccessError) => send_status(io, id, StatusError::Access, &[]),
    }
}

fn handle_reg_write<D: DxlIo>(shared: &Shared, io: &mut D, p: &RegWritePacket<'_>) {
    // TODO: stage write into pending buffer; commit on Action.
    reply_unsupported(shared, io, p.id);
}

fn handle_action<D: DxlIo>(shared: &Shared, io: &mut D, p: &ActionPacket) {
    // TODO: commit pending RegWrite. Applies on broadcast too.
    reply_unsupported(shared, io, p.id);
}

fn handle_factory_reset<D: DxlIo>(shared: &Shared, io: &mut D, p: &FactoryResetPacket) {
    // TODO: reset CONFIG (and per-mode CALIB) to defaults, then reboot.
    reply_unsupported(shared, io, p.id);
}

fn handle_reboot<D: DxlIo>(shared: &Shared, io: &mut D, p: &RebootPacket) {
    // TODO: ack with Status, wait for TX TC, then trigger soft reset.
    reply_unsupported(shared, io, p.id);
}

fn handle_clear<D: DxlIo>(shared: &Shared, io: &mut D, p: &ClearPacket<'_>) {
    // TODO: clear multi-turn revolution counter (option 0x01, key "CLR\0").
    reply_unsupported(shared, io, p.id);
}

fn handle_control_table_backup<D: DxlIo>(
    shared: &Shared,
    io: &mut D,
    p: &ControlTableBackupPacket<'_>,
) {
    // TODO: store/restore CONFIG snapshot to a third flash slot (key "CTRL").
    reply_unsupported(shared, io, p.id);
}

fn handle_sync_read<D: DxlIo>(_shared: &Shared, _io: &mut D, _p: &SyncReadPacket<'_>) {
    // TODO: if our id appears in the list, reply in slot order using the
    //       bus-turnaround timer (TIM3-SLTM) so slaves don't talk over each other.
}

fn handle_sync_write<D: DxlIo>(_shared: &Shared, _io: &mut D, _p: &SyncWritePacket<'_>) {
    // TODO: scan (id, length-byte chunk) pairs, apply our chunk silently.
}

fn handle_bulk_read<D: DxlIo>(_shared: &Shared, _io: &mut D, _p: &BulkReadPacket<'_>) {
    // TODO: scan (id, address, length) triples; reply in slot order.
}

fn handle_bulk_write<D: DxlIo>(_shared: &Shared, _io: &mut D, _p: &BulkWritePacket<'_>) {
    // TODO: scan (id, address, length, data) tuples; apply our chunk silently.
}

fn handle_fast_sync_read<D: DxlIo>(_shared: &Shared, _io: &mut D, _p: &FastSyncReadPacket<'_>) {
    // TODO: like sync_read but coalesced single-frame response.
}

fn handle_fast_bulk_read<D: DxlIo>(_shared: &Shared, _io: &mut D, _p: &FastBulkReadPacket<'_>) {
    // TODO: like bulk_read but coalesced single-frame response.
}

fn send_status<D: DxlIo>(io: &mut D, id: u8, error: StatusError, params: &[u8]) {
    let buf = io.tx_buf();
    buf.truncate(0);
    let packet = Packet::Status(StatusPacket::new(id, error.as_u8(), params));
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
            Packet::Status(p) => {
                let mut out: Vec<u8, 64> = Vec::new();
                for b in p.params.iter() {
                    out.push(b).unwrap();
                }
                (p.id, p.error, out)
            }
            _ => panic!("not a status packet"),
        }
    }

    #[test]
    fn ping_to_our_id_replies() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Ping(PingPacket::new(0)));
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

        let req = encode(&Packet::Ping(PingPacket::new(BROADCAST_ID)));
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

        let req = encode(&Packet::Ping(PingPacket::new(17)));
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

        let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
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

        let req = encode(&Packet::Read(ReadPacket::new(0, 0, 0)));
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

        let req = encode(&Packet::Reboot(RebootPacket::new(0)));
        io.feed(&req);
        h.poll(&shared, &mut io);

        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::Instruction.as_u8());
    }

    #[test]
    fn write_to_rw_address_succeeds_and_mutates() {
        use crate::regions::CONTROL_BASE_ADDR;
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
        io.feed(&req);
        h.poll(&shared, &mut io);

        let (id, err, params) = parse_status(&io.tx);
        assert_eq!(id, 0);
        assert_eq!(err, 0);
        assert!(params.is_empty());
        let lc = unsafe { &*shared.table.control.get() }.lifecycle;
        assert_eq!(lc.torque_enable, 1);
    }

    #[test]
    fn write_to_ro_address_replies_access_error() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        // CONFIG addr 0 is identity.model_number (RO).
        let req = encode(&Packet::Write(WritePacket::new(0, 0, &[0xAA, 0xBB])));
        io.feed(&req);
        h.poll(&shared, &mut io);

        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::Access.as_u8());
        let identity = unsafe { &*shared.table.config.get() }.identity;
        assert_eq!(identity.model_number, 0);
    }

    #[test]
    fn write_to_unmapped_address_replies_data_range_error() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Write(WritePacket::new(0, 0xFFFE, &[0x01])));
        io.feed(&req);
        h.poll(&shared, &mut io);

        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::DataRange.as_u8());
    }

    #[test]
    fn write_to_other_id_silent_and_does_not_mutate() {
        use crate::regions::CONTROL_BASE_ADDR;
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Write(WritePacket::new(
            17,
            CONTROL_BASE_ADDR,
            &[1],
        )));
        io.feed(&req);
        h.poll(&shared, &mut io);

        assert_eq!(io.start_tx_count, 0);
        let lc = unsafe { &*shared.table.control.get() }.lifecycle;
        assert_eq!(lc.torque_enable, 0);
    }

    #[test]
    fn broadcast_write_applies_but_silent() {
        use crate::regions::CONTROL_BASE_ADDR;
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Write(WritePacket::new(
            BROADCAST_ID,
            CONTROL_BASE_ADDR,
            &[1],
        )));
        io.feed(&req);
        h.poll(&shared, &mut io);

        assert_eq!(io.start_tx_count, 0);
        let lc = unsafe { &*shared.table.control.get() }.lifecycle;
        assert_eq!(lc.torque_enable, 1);
    }
}
