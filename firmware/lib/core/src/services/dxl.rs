use dxl_protocol::prelude::*;

use crate::{Error, RegionStorage, RingReader, Router, RxSnapshot, Shared, StagedWrites};

pub const DXL_SCRATCH_LEN: usize = 256;
pub const MAX_READ: usize = 128;
pub const MAX_WRITE: usize = 128;

fn error_to_status(e: Error) -> StatusError {
    match e {
        Error::AccessError => StatusError::Access,
        _ => StatusError::DataRange,
    }
}

pub trait DxlIo {
    type TxBuf: WriteBuf;

    fn rx_snapshot(&self) -> RxSnapshot<'_>;
    fn tx_buf(&mut self) -> &mut Self::TxBuf;
    fn start_tx(&mut self);
}

pub struct Dxl {
    reader: RingReader<DXL_SCRATCH_LEN>,
    staged: StagedWrites,
}

impl Dxl {
    pub const fn new() -> Self {
        Self {
            reader: RingReader::new(),
            staged: StagedWrites::new(),
        }
    }

    pub fn poll<D: DxlIo>(&mut self, shared: &Shared, io: &mut D) {
        let snap = io.rx_snapshot();
        self.reader.ingest(snap.ring(), snap.write_pos());

        let mut d = Dispatcher {
            shared,
            io,
            staged: &mut self.staged,
        };
        loop {
            match parse_one(self.reader.peek()) {
                Ok((packet, used)) => {
                    d.dispatch(&packet);
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

struct Dispatcher<'a, D: DxlIo> {
    shared: &'a Shared,
    io: &'a mut D,
    staged: &'a mut StagedWrites,
}

impl<D: DxlIo> Dispatcher<'_, D> {
    fn dispatch(&mut self, packet: &Packet<'_>) {
        match packet {
            Packet::Ping(p) => self.handle_ping(p),
            Packet::Read(p) => self.handle_read(p),
            Packet::Write(p) => self.handle_write(p),
            Packet::RegWrite(p) => self.handle_reg_write(p),
            Packet::Action(p) => self.handle_action(p),
            Packet::FactoryReset(p) => self.handle_factory_reset(p),
            Packet::Reboot(p) => self.handle_reboot(p),
            Packet::Clear(p) => self.handle_clear(p),
            Packet::ControlTableBackup(p) => self.handle_control_table_backup(p),
            Packet::SyncRead(p) => self.handle_sync_read(p),
            Packet::SyncWrite(p) => self.handle_sync_write(p),
            Packet::BulkRead(p) => self.handle_bulk_read(p),
            Packet::BulkWrite(p) => self.handle_bulk_write(p),
            Packet::FastSyncRead(p) => self.handle_fast_sync_read(p),
            Packet::FastBulkRead(p) => self.handle_fast_bulk_read(p),
            // Inbound Status frames originate from another device on the bus; drop.
            Packet::Status(_) => {}
        }
    }

    fn addressed(&self, target: u8) -> Option<(u8, bool)> {
        let id = self.shared.table.config.with(|c| c.comms.id);
        if target == id {
            Some((id, true))
        } else if target == BROADCAST_ID {
            Some((id, false))
        } else {
            None
        }
    }

    fn reply_unsupported(&mut self, target: u8) {
        if let Some((id, true)) = self.addressed(target) {
            self.send_status(id, StatusError::Instruction, &[]);
        }
    }

    fn send_status(&mut self, id: u8, error: StatusError, params: &[u8]) {
        let buf = self.io.tx_buf();
        buf.truncate(0);
        let packet = Packet::Status(StatusPacket::new(id, error.as_u8(), params));
        if write(buf, &packet).is_err() {
            buf.truncate(0);
            return;
        }
        self.io.start_tx();
    }

    fn handle_ping(&mut self, p: &PingPacket) {
        let Some((id, _)) = self.addressed(p.id) else {
            return;
        };
        let identity = self.shared.table.config.with(|c| c.identity);
        let model = identity.model_number.to_le_bytes();
        let fw = identity.firmware_version as u8;
        let params = [model[0], model[1], fw];
        self.send_status(id, StatusError::None, &params);
    }

    fn handle_read(&mut self, p: &ReadPacket) {
        let Some((id, true)) = self.addressed(p.id) else {
            return;
        };
        let len = p.length as usize;
        if len == 0 || len > MAX_READ {
            self.send_status(id, StatusError::DataRange, &[]);
            return;
        }
        let mut buf = [0u8; MAX_READ];
        match self.shared.table.read_bytes(p.address, &mut buf[..len]) {
            Ok(()) => self.send_status(id, StatusError::None, &buf[..len]),
            Err(e) => self.send_status(id, error_to_status(e), &[]),
        }
    }

    fn reply_table_result(&mut self, id: u8, direct: bool, result: Result<(), Error>) {
        if !direct {
            return;
        }
        match result {
            Ok(()) => self.send_status(id, StatusError::None, &[]),
            Err(e) => self.send_status(id, error_to_status(e), &[]),
        }
    }

    fn handle_write(&mut self, p: &WritePacket<'_>) {
        let Some((id, direct)) = self.addressed(p.id) else {
            return;
        };

        let mut buf = [0u8; MAX_WRITE];
        let len = match p.data.copy_into(&mut buf) {
            Ok(n) => n,
            Err(_) => {
                if direct {
                    self.send_status(id, StatusError::DataRange, &[]);
                }
                return;
            }
        };

        // Sync Write wipes pending RegWrite staging per DXL convention.
        self.staged.clear();
        let result = self
            .shared
            .table
            .write_bytes(p.address, &buf[..len], self.staged);
        self.reply_table_result(id, direct, result);
    }

    fn handle_reg_write(&mut self, p: &RegWritePacket<'_>) {
        let Some((id, direct)) = self.addressed(p.id) else {
            return;
        };

        let mut buf = [0u8; MAX_WRITE];
        let len = match p.data.copy_into(&mut buf) {
            Ok(n) => n,
            Err(_) => {
                if direct {
                    self.send_status(id, StatusError::DataRange, &[]);
                }
                return;
            }
        };

        let result = self
            .shared
            .table
            .stage_bytes(p.address, &buf[..len], self.staged);
        self.reply_table_result(id, direct, result);
    }

    fn handle_action(&mut self, p: &ActionPacket) {
        let Some((id, direct)) = self.addressed(p.id) else {
            return;
        };
        self.shared.table.commit_staged(self.staged);
        if direct {
            self.send_status(id, StatusError::None, &[]);
        }
    }

    fn handle_factory_reset(&mut self, p: &FactoryResetPacket) {
        // TODO: reset CONFIG (and per-mode CALIB) to defaults, then reboot.
        self.reply_unsupported(p.id);
    }

    fn handle_reboot(&mut self, p: &RebootPacket) {
        // TODO: ack with Status, wait for TX TC, then trigger soft reset.
        self.reply_unsupported(p.id);
    }

    fn handle_clear(&mut self, p: &ClearPacket<'_>) {
        // TODO: clear multi-turn revolution counter (option 0x01, key "CLR\0").
        self.reply_unsupported(p.id);
    }

    fn handle_control_table_backup(&mut self, p: &ControlTableBackupPacket<'_>) {
        // TODO: store/restore CONFIG snapshot to a third flash slot (key "CTRL").
        self.reply_unsupported(p.id);
    }

    fn handle_sync_read(&mut self, _p: &SyncReadPacket<'_>) {
        // TODO: if our id appears in the list, reply in slot order using the
        //       bus-turnaround timer (TIM3-SLTM) so slaves don't talk over each other.
    }

    fn handle_sync_write(&mut self, _p: &SyncWritePacket<'_>) {
        // TODO: scan (id, length-byte chunk) pairs, apply our chunk silently.
    }

    fn handle_bulk_read(&mut self, _p: &BulkReadPacket<'_>) {
        // TODO: scan (id, address, length) triples; reply in slot order.
    }

    fn handle_bulk_write(&mut self, _p: &BulkWritePacket<'_>) {
        // TODO: scan (id, address, length, data) tuples; apply our chunk silently.
    }

    fn handle_fast_sync_read(&mut self, _p: &FastSyncReadPacket<'_>) {
        // TODO: like sync_read but coalesced single-frame response.
    }

    fn handle_fast_bulk_read(&mut self, _p: &FastBulkReadPacket<'_>) {
        // TODO: like bulk_read but coalesced single-frame response.
    }
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
        let lc = shared.table.control.with(|c| c.lifecycle);
        assert!(lc.torque_enable);
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
        let identity = shared.table.config.with(|c| c.identity);
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
        let lc = shared.table.control.with(|c| c.lifecycle);
        assert!(!lc.torque_enable);
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
        let lc = shared.table.control.with(|c| c.lifecycle);
        assert!(lc.torque_enable);
    }

    #[test]
    fn reg_write_then_action_commits_to_live_table() {
        use crate::regions::CONTROL_BASE_ADDR;
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::RegWrite(RegWritePacket::new(
            0,
            CONTROL_BASE_ADDR,
            &[1],
        )));
        io.feed(&req);
        h.poll(&shared, &mut io);
        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::None.as_u8());
        assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

        io.tx.clear();
        let req = encode(&Packet::Action(ActionPacket::new(0)));
        io.feed(&req);
        h.poll(&shared, &mut io);
        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::None.as_u8());
        assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
    }

    #[test]
    fn reg_write_to_ro_address_replies_access_error_immediately() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::RegWrite(RegWritePacket::new(0, 0, &[0xAA, 0xBB])));
        io.feed(&req);
        h.poll(&shared, &mut io);
        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::Access.as_u8());
    }

    #[test]
    fn reg_write_invalid_value_rejected_at_stage_time() {
        use crate::regions::CONTROL_BASE_ADDR;
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::RegWrite(RegWritePacket::new(
            0,
            CONTROL_BASE_ADDR,
            &[2],
        )));
        io.feed(&req);
        h.poll(&shared, &mut io);
        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::DataRange.as_u8());

        io.tx.clear();
        let req = encode(&Packet::Action(ActionPacket::new(0)));
        io.feed(&req);
        h.poll(&shared, &mut io);
        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::None.as_u8());
        assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
    }

    #[test]
    fn sync_write_clears_pending_reg_write_staging() {
        use crate::regions::CONTROL_BASE_ADDR;
        use crate::regions::control::Mode;
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::RegWrite(RegWritePacket::new(
            0,
            CONTROL_BASE_ADDR,
            &[1],
        )));
        io.feed(&req);
        h.poll(&shared, &mut io);

        io.tx.clear();
        let req = encode(&Packet::Write(WritePacket::new(
            0,
            CONTROL_BASE_ADDR + 1,
            &[1],
        )));
        io.feed(&req);
        h.poll(&shared, &mut io);
        assert_eq!(
            shared.table.control.with(|c| c.lifecycle.mode),
            Mode::PositionPid,
        );

        io.tx.clear();
        let req = encode(&Packet::Action(ActionPacket::new(0)));
        io.feed(&req);
        h.poll(&shared, &mut io);
        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::None.as_u8());
        assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
    }

    #[test]
    fn broadcast_reg_write_and_action_silent_but_commits() {
        use crate::regions::CONTROL_BASE_ADDR;
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::RegWrite(RegWritePacket::new(
            BROADCAST_ID,
            CONTROL_BASE_ADDR,
            &[1],
        )));
        io.feed(&req);
        h.poll(&shared, &mut io);
        assert_eq!(io.start_tx_count, 0);

        let req = encode(&Packet::Action(ActionPacket::new(BROADCAST_ID)));
        io.feed(&req);
        h.poll(&shared, &mut io);
        assert_eq!(io.start_tx_count, 0);
        assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
    }

    #[test]
    fn action_with_empty_staging_replies_ok() {
        let shared = Shared::const_new();
        let mut io = FakeDxlIo::new();
        let mut h = Dxl::new();

        let req = encode(&Packet::Action(ActionPacket::new(0)));
        io.feed(&req);
        h.poll(&shared, &mut io);
        let (_, err, _) = parse_status(&io.tx);
        assert_eq!(err, StatusError::None.as_u8());
    }
}
