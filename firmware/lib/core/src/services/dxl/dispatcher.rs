use dxl_protocol::prelude::*;

use crate::regions::hooks::ControlTableHooks;
use crate::traits::{DxlBus, Event, Schedule, ServiceEvents};
use crate::{Error, RegionStorage, Router, Shared, StagedWrites, StatusReturnLevel};

use super::limits::{MAX_CONTROL_RW, MAX_SLAVE_COUNT};
use super::osc::{CalibratePacket, OscExt, OscReplyVariant, OscVariant};

fn error_to_status(e: Error) -> StatusError {
    match e {
        Error::AccessError => StatusError::Access,
        _ => StatusError::DataRange,
    }
}

struct Ctx {
    our_id: u8,
    rdt_us: u32,
    level: StatusReturnLevel,
}

impl Ctx {
    fn addressed(&self, target: u8) -> Option<(u8, bool)> {
        if target == self.our_id {
            Some((self.our_id, true))
        } else if target == BROADCAST_ID {
            Some((self.our_id, false))
        } else {
            None
        }
    }

    fn direct_schedule(&self) -> Schedule {
        Schedule {
            rdt_us: self.rdt_us,
            bytes_before: 0,
            slot_index: 0,
        }
    }
}

pub(super) struct Dispatcher<'a, B: DxlBus, E: ServiceEvents> {
    shared: &'a Shared,
    bus: &'a mut B,
    events: &'a mut E,
    staged: &'a mut StagedWrites,
}

impl<'a, B: DxlBus, E: ServiceEvents> Dispatcher<'a, B, E> {
    pub(super) fn new(
        shared: &'a Shared,
        bus: &'a mut B,
        events: &'a mut E,
        staged: &'a mut StagedWrites,
    ) -> Self {
        Self {
            shared,
            bus,
            events,
            staged,
        }
    }

    pub(super) fn dispatch(&mut self, packet: Packet<'_, OscExt>) {
        let ctx = self.snapshot_ctx();
        match &packet {
            Packet::Ping(p) => self.handle_ping(&ctx, p),
            Packet::Read(p) => self.handle_read(&ctx, p),
            Packet::Write(p) => self.handle_write(&ctx, p),
            Packet::RegWrite(p) => self.handle_reg_write(&ctx, p),
            Packet::Action(p) => self.handle_action(&ctx, p),
            Packet::FactoryReset(p) => self.handle_factory_reset(&ctx, p),
            Packet::Reboot(p) => self.handle_reboot(&ctx, p),
            Packet::Clear(p) => self.handle_clear(&ctx, p),
            Packet::ControlTableBackup(p) => self.handle_control_table_backup(&ctx, p),
            Packet::SyncRead(p) => self.handle_sync_read(&ctx, p),
            Packet::SyncWrite(p) => self.handle_sync_write(&ctx, p),
            Packet::BulkRead(p) => self.handle_bulk_read(&ctx, p),
            Packet::BulkWrite(p) => self.handle_bulk_write(&ctx, p),
            Packet::FastSyncRead(p) => self.handle_fast_read(&ctx, p),
            Packet::FastBulkRead(p) => self.handle_fast_read(&ctx, p),
            Packet::Ext(OscVariant::Calibrate(p)) => self.handle_calibrate(&ctx, p),
            // Inbound Status frames originate from another device on the bus; drop.
            Packet::Status(_) => {}
        }
    }

    fn snapshot_ctx(&self) -> Ctx {
        let (our_id, rdt_us, level) = self.shared.table.config.with(|c| {
            (
                c.comms.id,
                c.comms.return_delay_2us as u32 * 2,
                c.comms.status_return_level,
            )
        });
        Ctx {
            our_id,
            rdt_us,
            level,
        }
    }

    fn reply_unsupported(&mut self, ctx: &Ctx, target: u8) {
        if ctx.level < StatusReturnLevel::All {
            return;
        }
        if let Some((id, true)) = ctx.addressed(target) {
            self.bus.send(
                Status::Error(ErrorStatus {
                    id,
                    error: StatusError::Instruction,
                }),
                ctx.direct_schedule(),
            );
        }
    }

    fn reply_table_result(&mut self, ctx: &Ctx, id: u8, direct: bool, result: Result<(), Error>) {
        if !direct || ctx.level < StatusReturnLevel::All {
            return;
        }
        let reply = match result {
            Ok(()) => Status::Write(WriteStatus { id }),
            Err(e) => Status::Error(ErrorStatus {
                id,
                error: error_to_status(e),
            }),
        };
        self.bus.send(reply, ctx.direct_schedule());
    }

    fn handle_ping(&mut self, ctx: &Ctx, p: &PingPacket) {
        let Some((id, _)) = ctx.addressed(p.id) else {
            return;
        };
        let identity = self.shared.table.config.with(|c| c.identity);
        self.bus.send(
            Status::Ping(PingStatus {
                id,
                model: identity.model_number,
                firmware: identity.firmware_version as u8,
            }),
            ctx.direct_schedule(),
        );
    }

    fn handle_read(&mut self, ctx: &Ctx, p: &ReadPacket) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some((id, true)) = ctx.addressed(p.id) else {
            return;
        };
        let len = p.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.bus.send(
                Status::Error(ErrorStatus {
                    id,
                    error: StatusError::DataRange,
                }),
                ctx.direct_schedule(),
            );
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let reply = match self.shared.table.read_bytes(p.address, &mut buf[..len]) {
            Ok(()) => Status::Read(ReadStatus {
                id,
                data: &buf[..len],
            }),
            Err(e) => Status::Error(ErrorStatus {
                id,
                error: error_to_status(e),
            }),
        };
        self.bus.send(reply, ctx.direct_schedule());
    }

    fn handle_write(&mut self, ctx: &Ctx, p: &WritePacket<'_>) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };

        let mut buf = [0u8; MAX_CONTROL_RW];
        let len = match p.data.copy_to_slice(&mut buf) {
            Ok(n) => n,
            Err(_) => {
                if direct && ctx.level >= StatusReturnLevel::All {
                    self.bus.send(
                        Status::Error(ErrorStatus {
                            id,
                            error: StatusError::DataRange,
                        }),
                        ctx.direct_schedule(),
                    );
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
        let ok = result.is_ok();
        self.reply_table_result(ctx, id, direct, result);
        if ok {
            let mut hooks = ControlTableHooks::new(self.events);
            self.shared
                .table
                .dispatch_events(p.address, len as u16, &mut hooks);
        }
    }

    fn handle_reg_write(&mut self, ctx: &Ctx, p: &RegWritePacket<'_>) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };

        let mut buf = [0u8; MAX_CONTROL_RW];
        let len = match p.data.copy_to_slice(&mut buf) {
            Ok(n) => n,
            Err(_) => {
                if direct && ctx.level >= StatusReturnLevel::All {
                    self.bus.send(
                        Status::Error(ErrorStatus {
                            id,
                            error: StatusError::DataRange,
                        }),
                        ctx.direct_schedule(),
                    );
                }
                return;
            }
        };

        let result = self
            .shared
            .table
            .stage_bytes(p.address, &buf[..len], self.staged);
        // RegWrite ack uses Status::RegWrite on success path; share the
        // table-result helper by translating after the call.
        if !direct || ctx.level < StatusReturnLevel::All {
            return;
        }
        let reply = match result {
            Ok(()) => Status::RegWrite(RegWriteStatus { id }),
            Err(e) => Status::Error(ErrorStatus {
                id,
                error: error_to_status(e),
            }),
        };
        self.bus.send(reply, ctx.direct_schedule());
    }

    fn handle_action(&mut self, ctx: &Ctx, p: &ActionPacket) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };
        self.shared.table.commit_staged(self.staged);
        if direct && ctx.level >= StatusReturnLevel::All {
            self.bus
                .send(Status::Action(ActionStatus { id }), ctx.direct_schedule());
        }
    }

    fn handle_factory_reset(&mut self, ctx: &Ctx, p: &FactoryResetPacket) {
        // TODO: erase CALIB region via Flash trait, then device.reboot().
        self.reply_unsupported(ctx, p.id);
    }

    fn handle_calibrate(&mut self, ctx: &Ctx, p: &CalibratePacket) {
        let Some((id, true)) = ctx.addressed(p.id) else {
            return;
        };
        if p.count == 0 || p.count as usize > MAX_CONTROL_RW {
            self.bus.send(
                Status::Error(ErrorStatus {
                    id,
                    error: StatusError::DataRange,
                }),
                ctx.direct_schedule(),
            );
            return;
        }
        self.bus.send(
            Status::Ext(OscReplyVariant::Calibrate {
                id,
                zeros_count: p.count,
            }),
            ctx.direct_schedule(),
        );
    }

    fn handle_reboot(&mut self, ctx: &Ctx, p: &RebootPacket) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };
        let mode = self.shared.table.control.with(|c| c.system.boot_mode);
        if direct && ctx.level >= StatusReturnLevel::All {
            self.bus
                .send(Status::Reboot(RebootStatus { id }), ctx.direct_schedule());
        }
        self.events.send(Event::Reboot(mode));
    }

    fn handle_clear(&mut self, ctx: &Ctx, p: &ClearPacket<'_>) {
        // TODO: verify CLR\0 key, then ask kernel to zero multi-turn revolution count.
        self.reply_unsupported(ctx, p.id);
    }

    fn handle_control_table_backup(&mut self, ctx: &Ctx, p: &ControlTableBackupPacket<'_>) {
        // TODO: verify CTRL key, then serialize CONFIG to a flash slot via Flash trait.
        self.reply_unsupported(ctx, p.id);
    }

    fn handle_sync_read(&mut self, ctx: &Ctx, p: &SyncReadPacket<'_>) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some(info) = p.find_slot(ctx.our_id) else {
            return;
        };
        let schedule = Schedule {
            rdt_us: ctx.rdt_us,
            bytes_before: info.bytes_before,
            slot_index: info.index as u16,
        };

        let len = p.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.bus.send(
                Status::Error(ErrorStatus {
                    id: ctx.our_id,
                    error: StatusError::DataRange,
                }),
                schedule,
            );
            return;
        }

        let mut buf = [0u8; MAX_CONTROL_RW];
        let reply = match self.shared.table.read_bytes(p.address, &mut buf[..len]) {
            Ok(()) => Status::SyncRead(SyncReadStatus {
                id: ctx.our_id,
                data: &buf[..len],
            }),
            Err(e) => Status::Error(ErrorStatus {
                id: ctx.our_id,
                error: error_to_status(e),
            }),
        };
        self.bus.send(reply, schedule);
    }

    fn handle_sync_write(&mut self, _ctx: &Ctx, _p: &SyncWritePacket<'_>) {
        // TODO: scan (id, length-byte chunk) pairs, apply our chunk silently.
    }

    fn handle_bulk_read(&mut self, ctx: &Ctx, p: &BulkReadPacket<'_>) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some(info) = p.find_slot(ctx.our_id) else {
            return;
        };
        let schedule = Schedule {
            rdt_us: ctx.rdt_us,
            bytes_before: info.bytes_before,
            slot_index: info.index as u16,
        };

        let len = info.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.bus.send(
                Status::Error(ErrorStatus {
                    id: ctx.our_id,
                    error: StatusError::DataRange,
                }),
                schedule,
            );
            return;
        }

        let mut buf = [0u8; MAX_CONTROL_RW];
        let reply = match self.shared.table.read_bytes(info.address, &mut buf[..len]) {
            Ok(()) => Status::BulkRead(BulkReadStatus {
                id: ctx.our_id,
                data: &buf[..len],
            }),
            Err(e) => Status::Error(ErrorStatus {
                id: ctx.our_id,
                error: error_to_status(e),
            }),
        };
        self.bus.send(reply, schedule);
    }

    fn handle_bulk_write(&mut self, _ctx: &Ctx, _p: &BulkWritePacket<'_>) {
        // TODO: scan (id, address, length, data) tuples; apply our chunk silently.
    }

    fn handle_fast_read<P: FastReadPacket>(&mut self, ctx: &Ctx, p: &P) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some(info) = p.find_slot(ctx.our_id, MAX_SLAVE_COUNT) else {
            return;
        };
        let len = info.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            return;
        }
        let position = info.position();
        let schedule = Schedule {
            rdt_us: ctx.rdt_us,
            bytes_before: info.bytes_before,
            slot_index: info.our_slot as u16,
        };

        // Fast Read failure path: emit `length` zero bytes so the response
        // stays positionally aligned; the error byte carries the code.
        // Re-zero the buf since `read_bytes` may have partially modified it.
        let mut buf = [0u8; MAX_CONTROL_RW];
        let (error, data_len) = match self.shared.table.read_bytes(info.address, &mut buf[..len]) {
            Ok(()) => (StatusError::None, len),
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
            error: error.as_u8(),
            data: Bytes::unstuffed(&buf[..data_len]),
        };
        self.bus.send_slot(slot, position, schedule);
    }
}
