use dxl_protocol::InstructionPacket;
use dxl_protocol::packet::{
    ActionPacket, BulkReadPacket, BulkWritePacket, ErrorCode, FactoryResetPacket,
    FastBulkReadPacket, FastSlotInfo, FastSyncReadPacket, Id, PingPacket, PingStatus, RawPacket,
    ReadPacket, RebootPacket, Slot, Status, StatusError, SyncReadPacket, SyncWritePacket, U16Le,
    WritePacket,
};

use crate::regions::hooks::ControlTableHooks;
use crate::traits::DxlReply;
use crate::{Error, RegionStorage, Router, Shared, StagedWrites, StatusReturnLevel};

use super::limits::{MAX_CONTROL_RW, MAX_SLAVE_COUNT};

fn error_to_status(e: Error) -> StatusError {
    match e {
        Error::AccessError => StatusError::code(ErrorCode::Access),
        _ => StatusError::code(ErrorCode::DataRange),
    }
}

struct Ctx {
    our_id: Id,
    level: StatusReturnLevel,
}

impl Ctx {
    fn addressed(&self, target: Id) -> Option<(Id, bool)> {
        if target == self.our_id {
            Some((self.our_id, true))
        } else if target.is_broadcast() {
            Some((self.our_id, false))
        } else {
            None
        }
    }
}

pub(super) struct Dispatcher<'a, R: DxlReply + ?Sized> {
    shared: &'a Shared,
    reply: &'a mut R,
    staged: &'a mut StagedWrites,
}

impl<'a, R: DxlReply + ?Sized> Dispatcher<'a, R> {
    pub(super) fn new(shared: &'a Shared, reply: &'a mut R, staged: &'a mut StagedWrites) -> Self {
        Self {
            shared,
            reply,
            staged,
        }
    }

    pub(super) fn dispatch(&mut self, packet: InstructionPacket<'_>) {
        let ctx = self.snapshot_ctx();
        match packet {
            InstructionPacket::Ping(p) => self.handle_ping(&ctx, p),
            InstructionPacket::Read(p) => self.handle_read(&ctx, p),
            InstructionPacket::Write(p) => self.handle_write(&ctx, &p),
            InstructionPacket::RegWrite(p) => self.handle_reg_write(&ctx, &p),
            InstructionPacket::Action(p) => self.handle_action(&ctx, p),
            InstructionPacket::FactoryReset(p) => self.handle_factory_reset(&ctx, p),
            InstructionPacket::Reboot(p) => self.handle_reboot(&ctx, p),
            InstructionPacket::SyncRead(p) => self.handle_sync_read(&ctx, &p),
            InstructionPacket::SyncWrite(p) => self.handle_sync_write(&ctx, &p),
            InstructionPacket::BulkRead(p) => self.handle_bulk_read(&ctx, &p),
            InstructionPacket::BulkWrite(p) => self.handle_bulk_write(&ctx, &p),
            InstructionPacket::FastSyncRead(p) => self.handle_fast_sync_read(&ctx, &p),
            InstructionPacket::FastBulkRead(p) => self.handle_fast_bulk_read(&ctx, &p),
            InstructionPacket::Raw(r) => self.handle_raw(&ctx, &r),
        }
    }

    fn snapshot_ctx(&self) -> Ctx {
        let (our_id, level) = self
            .shared
            .table
            .config
            .with(|c| (c.comms.id, c.comms.status_return_level));
        Ctx {
            our_id: Id::new(our_id),
            level,
        }
    }

    fn send_status(&mut self, status: Status<'_>) {
        // Encoder overflow on a Status reply is a programmer error (the
        // driver's TX buffer is sized to fit every reply variant); swallow
        // here and let bench telemetry surface the case during integration.
        let _ = self.reply.send_status(status);
    }

    fn send_slot(&mut self, slot: &Slot<'_>) {
        let _ = self.reply.send_slot(slot);
    }

    fn reply_unsupported(&mut self, ctx: &Ctx, target: Id) {
        if ctx.level < StatusReturnLevel::All {
            return;
        }
        if let Some((id, true)) = ctx.addressed(target) {
            self.send_status(Status::Empty {
                id,
                error: StatusError::code(ErrorCode::Instruction),
            });
        }
    }

    fn reply_table_result(&mut self, ctx: &Ctx, id: Id, direct: bool, result: Result<(), Error>) {
        if !direct || ctx.level < StatusReturnLevel::All {
            return;
        }
        let reply = match result {
            Ok(()) => Status::Empty {
                id,
                error: StatusError::OK,
            },
            Err(e) => Status::Empty {
                id,
                error: error_to_status(e),
            },
        };
        self.send_status(reply);
    }

    fn handle_ping(&mut self, ctx: &Ctx, p: &PingPacket) {
        let Some((id, _direct)) = ctx.addressed(p.header.id) else {
            return;
        };
        let identity = self.shared.table.config.with(|c| c.identity);
        self.send_status(Status::Ping {
            id,
            error: StatusError::OK,
            status: PingStatus {
                model: U16Le::from_u16(identity.model_number),
                fw_version: identity.firmware_version as u8,
            },
        });
    }

    fn handle_read(&mut self, ctx: &Ctx, p: &ReadPacket) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some((id, true)) = ctx.addressed(p.header.id) else {
            return;
        };
        let len = p.length.get() as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.send_status(Status::Empty {
                id,
                error: StatusError::code(ErrorCode::DataRange),
            });
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let reply = match self.shared.table.read_bytes(p.addr.get(), &mut buf[..len]) {
            Ok(()) => Status::Read {
                id,
                error: StatusError::OK,
                data: &buf[..len],
            },
            Err(e) => Status::Empty {
                id,
                error: error_to_status(e),
            },
        };
        self.send_status(reply);
    }

    fn handle_write(&mut self, ctx: &Ctx, p: &WritePacket<'_>) {
        let id_byte = p.header.header.id;
        let Some((id, direct)) = ctx.addressed(id_byte) else {
            return;
        };

        let mut buf = [0u8; MAX_CONTROL_RW];
        let len = p.data.len();
        if len > MAX_CONTROL_RW {
            if direct && ctx.level >= StatusReturnLevel::All {
                self.send_status(Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::DataRange),
                });
            }
            return;
        }
        buf[..len].copy_from_slice(p.data);

        // Sync Write wipes pending RegWrite staging per DXL convention.
        self.staged.clear();
        let result = self
            .shared
            .table
            .write_bytes(p.header.addr.get(), &buf[..len], self.staged);
        let ok = result.is_ok();
        self.reply_table_result(ctx, id, direct, result);
        if ok {
            let mut hooks = ControlTableHooks::new(self.reply);
            self.shared
                .table
                .dispatch_events(p.header.addr.get(), len as u16, &mut hooks);
        }
    }

    fn handle_reg_write(&mut self, ctx: &Ctx, p: &WritePacket<'_>) {
        let id_byte = p.header.header.id;
        let Some((id, direct)) = ctx.addressed(id_byte) else {
            return;
        };

        let mut buf = [0u8; MAX_CONTROL_RW];
        let len = p.data.len();
        if len > MAX_CONTROL_RW {
            if direct && ctx.level >= StatusReturnLevel::All {
                self.send_status(Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::DataRange),
                });
            }
            return;
        }
        buf[..len].copy_from_slice(p.data);

        let result = self
            .shared
            .table
            .stage_bytes(p.header.addr.get(), &buf[..len], self.staged);
        if !direct || ctx.level < StatusReturnLevel::All {
            return;
        }
        let reply = match result {
            Ok(()) => Status::Empty {
                id,
                error: StatusError::OK,
            },
            Err(e) => Status::Empty {
                id,
                error: error_to_status(e),
            },
        };
        self.send_status(reply);
    }

    fn handle_action(&mut self, ctx: &Ctx, p: &ActionPacket) {
        let Some((id, direct)) = ctx.addressed(p.header.id) else {
            return;
        };
        self.shared.table.commit_staged(self.staged);
        if direct && ctx.level >= StatusReturnLevel::All {
            self.send_status(Status::Empty {
                id,
                error: StatusError::OK,
            });
        }
    }

    fn handle_factory_reset(&mut self, ctx: &Ctx, p: &FactoryResetPacket) {
        // TODO: erase CALIB region via Flash trait, then device.reboot().
        self.reply_unsupported(ctx, p.header.id);
    }

    fn handle_reboot(&mut self, ctx: &Ctx, p: &RebootPacket) {
        let Some((id, direct)) = ctx.addressed(p.header.id) else {
            return;
        };
        let mode = self.shared.table.control.with(|c| c.system.boot_mode);
        if direct && ctx.level >= StatusReturnLevel::All {
            self.send_status(Status::Empty {
                id,
                error: StatusError::OK,
            });
        }
        self.reply.stage_reboot(mode);
    }

    fn handle_raw(&mut self, ctx: &Ctx, r: &RawPacket<'_>) {
        // Unknown standard instructions (Clear 0x10, CTBackup 0x20) and any
        // truly unknown byte: ack as unsupported when addressed directly.
        self.reply_unsupported(ctx, r.header.header.id);
    }

    fn handle_sync_read(&mut self, ctx: &Ctx, p: &SyncReadPacket<'_>) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some(_info) = p.find_slot(ctx.our_id) else {
            return;
        };

        let len = p.header.length.get() as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.send_status(Status::Empty {
                id: ctx.our_id,
                error: StatusError::code(ErrorCode::DataRange),
            });
            return;
        }

        let mut buf = [0u8; MAX_CONTROL_RW];
        let reply = match self
            .shared
            .table
            .read_bytes(p.header.addr.get(), &mut buf[..len])
        {
            Ok(()) => Status::Read {
                id: ctx.our_id,
                error: StatusError::OK,
                data: &buf[..len],
            },
            Err(e) => Status::Empty {
                id: ctx.our_id,
                error: error_to_status(e),
            },
        };
        self.send_status(reply);
    }

    fn handle_sync_write(&mut self, ctx: &Ctx, p: &SyncWritePacket<'_>) {
        let len = p.header.length.get() as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            return;
        }
        let Some(entry) = p.find_entry(ctx.our_id) else {
            return;
        };
        if entry.data.len() < len {
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        buf[..len].copy_from_slice(&entry.data[..len]);
        // Mirrors `handle_write`: Sync Write wipes any pending RegWrite staging.
        self.staged.clear();
        if self
            .shared
            .table
            .write_bytes(p.header.addr.get(), &buf[..len], self.staged)
            .is_ok()
        {
            let mut hooks = ControlTableHooks::new(self.reply);
            self.shared
                .table
                .dispatch_events(p.header.addr.get(), len as u16, &mut hooks);
        }
    }

    fn handle_bulk_read(&mut self, ctx: &Ctx, p: &BulkReadPacket<'_>) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some(info) = p.find_slot(ctx.our_id) else {
            return;
        };

        let len = info.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.send_status(Status::Empty {
                id: ctx.our_id,
                error: StatusError::code(ErrorCode::DataRange),
            });
            return;
        }

        let mut buf = [0u8; MAX_CONTROL_RW];
        let reply = match self.shared.table.read_bytes(info.address, &mut buf[..len]) {
            Ok(()) => Status::Read {
                id: ctx.our_id,
                error: StatusError::OK,
                data: &buf[..len],
            },
            Err(e) => Status::Empty {
                id: ctx.our_id,
                error: error_to_status(e),
            },
        };
        self.send_status(reply);
    }

    fn handle_bulk_write(&mut self, ctx: &Ctx, p: &BulkWritePacket<'_>) {
        let Some(entry) = p.find_entry(ctx.our_id) else {
            return;
        };
        let len = entry.data.len();
        if len == 0 || len > MAX_CONTROL_RW {
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        buf[..len].copy_from_slice(entry.data);
        self.staged.clear();
        if self
            .shared
            .table
            .write_bytes(entry.addr, &buf[..len], self.staged)
            .is_ok()
        {
            let mut hooks = ControlTableHooks::new(self.reply);
            self.shared
                .table
                .dispatch_events(entry.addr, len as u16, &mut hooks);
        }
    }

    fn handle_fast_sync_read(&mut self, ctx: &Ctx, p: &FastSyncReadPacket<'_>) {
        let Some(info) = p.find_slot(ctx.our_id, MAX_SLAVE_COUNT) else {
            return;
        };
        self.fast_read_reply(ctx, info);
    }

    fn handle_fast_bulk_read(&mut self, ctx: &Ctx, p: &FastBulkReadPacket<'_>) {
        let Some(info) = p.find_slot(ctx.our_id, MAX_SLAVE_COUNT) else {
            return;
        };
        self.fast_read_reply(ctx, info);
    }

    fn fast_read_reply(&mut self, ctx: &Ctx, info: FastSlotInfo) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let len = info.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            return;
        }
        // Fast Read failure path: emit `length` zero bytes so the response
        // stays positionally aligned; the error byte carries the code.
        let mut buf = [0u8; MAX_CONTROL_RW];
        let (error, data_len) = match self.shared.table.read_bytes(info.address, &mut buf[..len]) {
            Ok(()) => (StatusError::OK, len),
            Err(e) => {
                for b in &mut buf[..len] {
                    *b = 0;
                }
                (error_to_status(e), len)
            }
        };
        // Sync vs Bulk produce identical wire bytes on the slave; the master
        // disambiguates by remembering which request it sent.
        let slot = Slot {
            id: ctx.our_id,
            error,
            data: &buf[..data_len],
        };
        self.send_slot(&slot);
    }
}
