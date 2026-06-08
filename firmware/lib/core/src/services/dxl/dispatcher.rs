use dxl_protocol::packet::{
    ActionPacket, BulkReadPacket, BulkWritePacket, ErrorCode, FactoryResetPacket,
    FastBulkReadPacket, FastSlotInfo, FastSyncReadPacket, Id, Packet, PingPacket, PingStatus,
    RawPacket, ReadPacket, RebootPacket, Slot, Status, StatusError, SyncReadPacket,
    SyncWritePacket, U16Le, WritePacket,
};
use dxl_protocol::{CRC_BYTES, RESPONSE_HEADER_BYTES};

use crate::regions::hooks::ControlTableHooks;
use crate::traits::{DxlBus, Event, Schedule, ServiceEvents};
use crate::{Error, RegionStorage, Router, Shared, StagedWrites, StatusReturnLevel};

use super::limits::{MAX_CONTROL_RW, MAX_SLAVE_COUNT};
use super::osc::{
    CalibratePacket, CalibrateStatus, OscVariant, calibrate_status_bytes, decode_raw,
};

fn error_to_status(e: Error) -> StatusError {
    match e {
        Error::AccessError => StatusError::code(ErrorCode::Access),
        _ => StatusError::code(ErrorCode::DataRange),
    }
}

struct Ctx {
    our_id: Id,
    rdt_us: u32,
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

    pub(super) fn dispatch(&mut self, packet: Packet<'_>) {
        let ctx = self.snapshot_ctx();
        match packet {
            Packet::Ping(p) => self.handle_ping(&ctx, p),
            Packet::Read(p) => self.handle_read(&ctx, p),
            Packet::Write(p) => self.handle_write(&ctx, &p),
            Packet::RegWrite(p) => self.handle_reg_write(&ctx, &p),
            Packet::Action(p) => self.handle_action(&ctx, p),
            Packet::FactoryReset(p) => self.handle_factory_reset(&ctx, p),
            Packet::Reboot(p) => self.handle_reboot(&ctx, p),
            Packet::SyncRead(p) => self.handle_sync_read(&ctx, &p),
            Packet::SyncWrite(p) => self.handle_sync_write(&ctx, &p),
            Packet::BulkRead(p) => self.handle_bulk_read(&ctx, &p),
            Packet::BulkWrite(p) => self.handle_bulk_write(&ctx, &p),
            Packet::FastSyncRead(p) => self.handle_fast_sync_read(&ctx, &p),
            Packet::FastBulkRead(p) => self.handle_fast_bulk_read(&ctx, &p),
            // Inbound Status frames originate from another device on the bus; drop.
            Packet::Status(_) => {}
            Packet::Raw(r) => self.handle_raw(&ctx, &r),
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
            our_id: Id::new(our_id),
            rdt_us,
            level,
        }
    }

    fn reply_unsupported(&mut self, ctx: &Ctx, target: Id) {
        if ctx.level < StatusReturnLevel::All {
            return;
        }
        if let Some((id, true)) = ctx.addressed(target) {
            self.bus.send(
                Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::Instruction),
                },
                ctx.direct_schedule(),
            );
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
        self.bus.send(reply, ctx.direct_schedule());
    }

    fn handle_ping(&mut self, ctx: &Ctx, p: &PingPacket) {
        let Some((id, direct)) = ctx.addressed(p.header.id) else {
            return;
        };
        let identity = self.shared.table.config.with(|c| c.identity);
        // DXL 2.0 broadcast Ping convention: each slave fires its reply in an
        // ID-indexed time slot so multiple chips don't collide on the wire.
        // Slot width = one Ping Status frame (header(9) + model_lo + model_hi
        // + firmware + crc(2) = 14 B); the chip layers SLOT_MARGIN per slot
        // on top via `slot_index`.
        let schedule = if direct {
            ctx.direct_schedule()
        } else {
            const PING_STATUS_FRAME_BYTES: u32 =
                RESPONSE_HEADER_BYTES as u32 + 3 + CRC_BYTES as u32;
            Schedule {
                rdt_us: ctx.rdt_us,
                bytes_before: (id.as_byte() as u32) * PING_STATUS_FRAME_BYTES,
                slot_index: id.as_byte() as u16,
            }
        };
        self.bus.send(
            Status::Ping {
                id,
                error: StatusError::OK,
                status: PingStatus {
                    model: U16Le::from_u16(identity.model_number),
                    fw_version: identity.firmware_version as u8,
                },
            },
            schedule,
        );
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
            self.bus.send(
                Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
                ctx.direct_schedule(),
            );
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
        self.bus.send(reply, ctx.direct_schedule());
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
                self.bus.send(
                    Status::Empty {
                        id,
                        error: StatusError::code(ErrorCode::DataRange),
                    },
                    ctx.direct_schedule(),
                );
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
            let mut hooks = ControlTableHooks::new(self.events);
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
                self.bus.send(
                    Status::Empty {
                        id,
                        error: StatusError::code(ErrorCode::DataRange),
                    },
                    ctx.direct_schedule(),
                );
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
        self.bus.send(reply, ctx.direct_schedule());
    }

    fn handle_action(&mut self, ctx: &Ctx, p: &ActionPacket) {
        let Some((id, direct)) = ctx.addressed(p.header.id) else {
            return;
        };
        self.shared.table.commit_staged(self.staged);
        if direct && ctx.level >= StatusReturnLevel::All {
            self.bus.send(
                Status::Empty {
                    id,
                    error: StatusError::OK,
                },
                ctx.direct_schedule(),
            );
        }
    }

    fn handle_factory_reset(&mut self, ctx: &Ctx, p: &FactoryResetPacket) {
        // TODO: erase CALIB region via Flash trait, then device.reboot().
        self.reply_unsupported(ctx, p.header.id);
    }

    fn handle_calibrate(&mut self, ctx: &Ctx, p: &CalibratePacket) {
        let Some((id, true)) = ctx.addressed(p.id) else {
            return;
        };
        if p.count == 0 || p.count as usize > MAX_CONTROL_RW {
            self.bus.send(
                Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
                ctx.direct_schedule(),
            );
            return;
        }
        let Some(snap) = self.bus.cal_snapshot() else {
            self.bus.send(
                Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
                ctx.direct_schedule(),
            );
            return;
        };
        // Chip owns the drift filter — its `cal_snapshot` reports both the
        // most recent observation (this CALIB request's wire-timing, in the
        // common path) and the most recent batched apply. Dispatcher packs
        // the 11-byte measurement payload and emits it as Status::Raw.
        let payload = calibrate_status_bytes(&CalibrateStatus {
            id,
            observed_ticks: snap.observed_ticks,
            nominal_ticks: snap.nominal_ticks,
            applied_trim_delta: snap.applied_trim_delta,
            applied_fine_trim_us: snap.applied_fine_trim_us,
        });
        self.bus.send(
            Status::Raw {
                id,
                error: StatusError::OK,
                payload: &payload,
            },
            ctx.direct_schedule(),
        );
    }

    fn handle_reboot(&mut self, ctx: &Ctx, p: &RebootPacket) {
        let Some((id, direct)) = ctx.addressed(p.header.id) else {
            return;
        };
        let mode = self.shared.table.control.with(|c| c.system.boot_mode);
        if direct && ctx.level >= StatusReturnLevel::All {
            self.bus.send(
                Status::Empty {
                    id,
                    error: StatusError::OK,
                },
                ctx.direct_schedule(),
            );
        }
        self.events.send(Event::Reboot(mode));
    }

    fn handle_raw(&mut self, ctx: &Ctx, r: &RawPacket<'_>) {
        if let Some(variant) = decode_raw(r) {
            match variant {
                OscVariant::Calibrate(p) => self.handle_calibrate(ctx, &p),
            }
            return;
        }
        // Unknown standard instructions (Clear 0x10, CTBackup 0x20) and any
        // truly unknown byte: ack as unsupported when addressed directly.
        self.reply_unsupported(ctx, r.header.header.id);
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

        let len = p.header.length.get() as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.bus.send(
                Status::Empty {
                    id: ctx.our_id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
                schedule,
            );
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
        self.bus.send(reply, schedule);
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
            let mut hooks = ControlTableHooks::new(self.events);
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
        let schedule = Schedule {
            rdt_us: ctx.rdt_us,
            bytes_before: info.bytes_before,
            slot_index: info.index as u16,
        };

        let len = info.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.bus.send(
                Status::Empty {
                    id: ctx.our_id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
                schedule,
            );
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
        self.bus.send(reply, schedule);
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
            let mut hooks = ControlTableHooks::new(self.events);
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
        self.bus.send_slot(slot, position, schedule);
    }
}
