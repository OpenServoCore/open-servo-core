use dxl_protocol::prelude::*;
use dxl_protocol::{FastSlot, FastSlotBody, write_fast_slot};

use crate::regions::config;
use crate::traits::{DeviceControl, DxlBus};
use crate::{Error, RegionStorage, Router, Shared, StagedWrites, StatusReturnLevel};

use super::slot::{bulk_slot_delay_us, bytes_to_us, slot_period_us};

const MAX_READ: usize = 128;
const MAX_WRITE: usize = 128;
const MAX_BULK_BODY: usize = 256;
const MAX_FAST_SLOTS: usize = 32;

fn error_to_status(e: Error) -> StatusError {
    match e {
        Error::AccessError => StatusError::Access,
        _ => StatusError::DataRange,
    }
}

fn touches_baud(addr: u16, len: usize) -> bool {
    let baud_addr = config::addr::comms::BAUD_RATE_IDX;
    addr <= baud_addr && (addr as u32) + (len as u32) > baud_addr as u32
}

pub(super) struct Dispatcher<'a, B: DxlBus, D: DeviceControl> {
    shared: &'a Shared,
    bus: &'a mut B,
    device: &'a mut D,
    staged: &'a mut StagedWrites,
}

impl<'a, B: DxlBus, D: DeviceControl> Dispatcher<'a, B, D> {
    pub(super) fn new(
        shared: &'a Shared,
        bus: &'a mut B,
        device: &'a mut D,
        staged: &'a mut StagedWrites,
    ) -> Self {
        Self {
            shared,
            bus,
            device,
            staged,
        }
    }

    pub(super) fn dispatch(&mut self, packet: Packet<'_>, parsed_end: u32) {
        match &packet {
            Packet::Ping(p) => self.handle_ping(p),
            Packet::Read(p) => self.handle_read(p),
            Packet::Write(p) => self.handle_write(p),
            Packet::RegWrite(p) => self.handle_reg_write(p),
            Packet::Action(p) => self.handle_action(p),
            Packet::FactoryReset(p) => self.handle_factory_reset(p),
            Packet::Reboot(p) => self.handle_reboot(p),
            Packet::Clear(p) => self.handle_clear(p),
            Packet::ControlTableBackup(p) => self.handle_control_table_backup(p),
            Packet::SyncRead(p) => self.handle_sync_read(p, parsed_end),
            Packet::SyncWrite(p) => self.handle_sync_write(p),
            Packet::BulkRead(p) => self.handle_bulk_read(p, parsed_end),
            Packet::BulkWrite(p) => self.handle_bulk_write(p),
            Packet::FastSyncRead(p) => self.handle_fast_read(p, parsed_end),
            Packet::FastBulkRead(p) => self.handle_fast_read(p, parsed_end),
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
            self.send_status(id, StatusError::Instruction, &[], StatusReturnLevel::All);
        }
    }

    fn level(&self) -> StatusReturnLevel {
        self.shared
            .table
            .config
            .with(|c| c.comms.status_return_level)
    }

    fn baud(&self) -> crate::BaudRate {
        self.shared.table.config.with(|c| c.comms.baud_rate_idx)
    }

    fn send_status(
        &mut self,
        id: u8,
        error: StatusError,
        params: &[u8],
        min_level: StatusReturnLevel,
    ) {
        if self.level() < min_level {
            return;
        }
        let buf = self.bus.reply_buffer();
        buf.truncate(0);
        let packet = Packet::Status(StatusPacket::new(id, error.as_u8(), params));
        if write(buf, &packet).is_err() {
            buf.truncate(0);
            return;
        }
        self.bus.send();
    }

    fn send_status_after(
        &mut self,
        id: u8,
        error: StatusError,
        params: &[u8],
        min_level: StatusReturnLevel,
        delay_us: u32,
    ) {
        if self.level() < min_level {
            return;
        }
        let buf = self.bus.reply_buffer();
        buf.truncate(0);
        let packet = Packet::Status(StatusPacket::new(id, error.as_u8(), params));
        if write(buf, &packet).is_err() {
            buf.truncate(0);
            return;
        }
        self.bus.send_after(delay_us);
    }

    fn handle_ping(&mut self, p: &PingPacket) {
        let Some((id, _)) = self.addressed(p.id) else {
            return;
        };
        let identity = self.shared.table.config.with(|c| c.identity);
        let model = identity.model_number.to_le_bytes();
        let fw = identity.firmware_version as u8;
        let params = [model[0], model[1], fw];
        self.send_status(id, StatusError::None, &params, StatusReturnLevel::None);
    }

    fn handle_read(&mut self, p: &ReadPacket) {
        let Some((id, true)) = self.addressed(p.id) else {
            return;
        };
        let len = p.length as usize;
        if len == 0 || len > MAX_READ {
            self.send_status(id, StatusError::DataRange, &[], StatusReturnLevel::Read);
            return;
        }
        let mut buf = [0u8; MAX_READ];
        match self.shared.table.read_bytes(p.address, &mut buf[..len]) {
            Ok(()) => self.send_status(id, StatusError::None, &buf[..len], StatusReturnLevel::Read),
            Err(e) => self.send_status(id, error_to_status(e), &[], StatusReturnLevel::Read),
        }
    }

    fn reply_table_result(&mut self, id: u8, direct: bool, result: Result<(), Error>) {
        if !direct {
            return;
        }
        match result {
            Ok(()) => self.send_status(id, StatusError::None, &[], StatusReturnLevel::All),
            Err(e) => self.send_status(id, error_to_status(e), &[], StatusReturnLevel::All),
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
                    self.send_status(id, StatusError::DataRange, &[], StatusReturnLevel::All);
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
        let baud_changed = result.is_ok() && touches_baud(p.address, len);
        self.reply_table_result(id, direct, result);
        if baud_changed {
            // reply queued, bus impl defers retune until TC.
            let new_rate = self.shared.table.config.with(|c| c.comms.baud_rate_idx);
            self.bus.set_baud(new_rate);
        }
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
                    self.send_status(id, StatusError::DataRange, &[], StatusReturnLevel::All);
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
            self.send_status(id, StatusError::None, &[], StatusReturnLevel::All);
        }
    }

    fn handle_factory_reset(&mut self, p: &FactoryResetPacket) {
        // TODO: erase CALIB region via Flash trait, then device.reboot().
        self.reply_unsupported(p.id);
    }

    fn handle_reboot(&mut self, p: &RebootPacket) {
        let Some((id, direct)) = self.addressed(p.id) else {
            return;
        };
        let mode = self.shared.table.control.with(|c| c.system.boot_mode);
        if direct {
            self.send_status(id, StatusError::None, &[], StatusReturnLevel::All);
        }
        self.device.reboot(mode);
    }

    fn handle_clear(&mut self, p: &ClearPacket<'_>) {
        // TODO: verify CLR\0 key, then ask kernel to zero multi-turn revolution count.
        self.reply_unsupported(p.id);
    }

    fn handle_control_table_backup(&mut self, p: &ControlTableBackupPacket<'_>) {
        // TODO: verify CTRL key, then serialize CONFIG to a flash slot via Flash trait.
        self.reply_unsupported(p.id);
    }

    fn handle_sync_read(&mut self, p: &SyncReadPacket<'_>, parsed_end: u32) {
        let our_id = self.shared.table.config.with(|c| c.comms.id);
        let Some(slot) = p.ids.iter().position(|id| id == our_id) else {
            return;
        };
        if !self.bus.request_complete(parsed_end) {
            return;
        }

        let delay_us = (slot as u32) * slot_period_us(self.baud(), p.length);

        let len = p.length as usize;
        if len == 0 || len > MAX_READ {
            self.send_status_after(
                our_id,
                StatusError::DataRange,
                &[],
                StatusReturnLevel::Read,
                delay_us,
            );
            return;
        }

        let mut buf = [0u8; MAX_READ];
        match self.shared.table.read_bytes(p.address, &mut buf[..len]) {
            Ok(()) => self.send_status_after(
                our_id,
                StatusError::None,
                &buf[..len],
                StatusReturnLevel::Read,
                delay_us,
            ),
            Err(e) => self.send_status_after(
                our_id,
                error_to_status(e),
                &[],
                StatusReturnLevel::Read,
                delay_us,
            ),
        }
    }

    fn handle_sync_write(&mut self, _p: &SyncWritePacket<'_>) {
        // TODO: scan (id, length-byte chunk) pairs, apply our chunk silently.
    }

    fn handle_bulk_read(&mut self, p: &BulkReadPacket<'_>, parsed_end: u32) {
        let our_id = self.shared.table.config.with(|c| c.comms.id);

        let mut body = [0u8; MAX_BULK_BODY];
        let Ok(n) = p.body.copy_into(&mut body) else {
            return;
        };
        let body = &body[..n];

        let mut found = None;
        for tup in body.chunks_exact(5) {
            if tup[0] == our_id {
                let address = u16::from_le_bytes([tup[1], tup[2]]);
                let length = u16::from_le_bytes([tup[3], tup[4]]);
                found = Some((address, length));
                break;
            }
        }
        let Some((address, length)) = found else {
            return;
        };

        if !self.bus.request_complete(parsed_end) {
            return;
        }

        let delay_us = bulk_slot_delay_us(body, our_id, self.baud()).unwrap_or(0);

        let len = length as usize;
        if len == 0 || len > MAX_READ {
            self.send_status_after(
                our_id,
                StatusError::DataRange,
                &[],
                StatusReturnLevel::Read,
                delay_us,
            );
            return;
        }

        let mut buf = [0u8; MAX_READ];
        match self.shared.table.read_bytes(address, &mut buf[..len]) {
            Ok(()) => self.send_status_after(
                our_id,
                StatusError::None,
                &buf[..len],
                StatusReturnLevel::Read,
                delay_us,
            ),
            Err(e) => self.send_status_after(
                our_id,
                error_to_status(e),
                &[],
                StatusReturnLevel::Read,
                delay_us,
            ),
        }
    }

    fn handle_bulk_write(&mut self, _p: &BulkWritePacket<'_>) {
        // TODO: scan (id, address, length, data) tuples; apply our chunk silently.
    }

    /// Up-front `level()` bail skips find_slot+read+encode when we'd silence
    /// anyway, unlike Read/Write where `send_status` swallows it later.
    fn handle_fast_read<P: FastReadPacket>(&mut self, p: &P, parsed_end: u32) {
        if self.level() < StatusReturnLevel::Read {
            return;
        }
        let our_id = self.shared.table.config.with(|c| c.comms.id);
        let Some(info) = p.find_slot(our_id, MAX_FAST_SLOTS) else {
            return;
        };
        let len = info.length as usize;
        if len == 0 || len > MAX_READ {
            return;
        }
        if !self.bus.request_complete(parsed_end) {
            return;
        }

        // Zero-fill on error keeps slot length-correct; error byte signals it.
        let mut buf = [0u8; MAX_READ];
        let error = match self.shared.table.read_bytes(info.address, &mut buf[..len]) {
            Ok(()) => StatusError::None,
            Err(e) => {
                buf[..len].fill(0);
                error_to_status(e)
            }
        };

        let body = FastSlotBody {
            error: error.as_u8(),
            id: our_id,
            data: &buf[..len],
        };
        let slot = match info.position() {
            FastSlotPosition::Only => FastSlot::Only {
                packet_length: info.packet_length,
                body,
            },
            FastSlotPosition::First => FastSlot::First {
                packet_length: info.packet_length,
                body,
            },
            FastSlotPosition::Middle => FastSlot::Middle(body),
            FastSlotPosition::Last => FastSlot::Last(body),
        };
        let tx_buf = self.bus.reply_buffer();
        tx_buf.truncate(0);
        if write_fast_slot(tx_buf, &slot).is_err() {
            tx_buf.truncate(0);
            return;
        }

        match info.position() {
            FastSlotPosition::Only => {
                // No predecessors → CRC is fully formed in the buffer and
                // no snoop is needed, but we still owe the host the standard
                // post-request idle before driving the line.
                self.bus.send_after(0);
            }
            FastSlotPosition::Last => {
                let fire_us = bytes_to_us(p.bytes_before(info.our_slot), self.baud());
                self.bus.send_with_snoop_crc(fire_us, Some(parsed_end));
            }
            FastSlotPosition::First | FastSlotPosition::Middle => {
                let fire_us = bytes_to_us(p.bytes_before(info.our_slot), self.baud());
                self.bus.send_after(fire_us);
            }
        }
    }
}
