use dxl_protocol::prelude::*;
use dxl_protocol::{BULK_REQUEST_SLOT_BYTES, FastSlot, FastSlotBody, write_fast_slot};

use crate::regions::config;
use crate::traits::{DxlBus, Event, ServiceEvents};
use crate::{BaudRate, Error, RegionStorage, Router, Shared, StagedWrites, StatusReturnLevel};

use super::limits::{MAX_CONTROL_RW, MAX_SLAVE_COUNT};
use super::slot::{bulk_slot_delay_us, bytes_to_us, bytes_to_us_q88, slot_period_us};

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

fn touches_clock_trim(addr: u16, len: usize) -> bool {
    let field_addr = config::addr::comms::CLOCK_TRIM;
    addr <= field_addr && (addr as u32) + (len as u32) > field_addr as u32
}

fn touches_clock_fine_trim_us(addr: u16, len: usize) -> bool {
    let field_addr = config::addr::comms::CLOCK_FINE_TRIM_US;
    let field_end = field_addr as u32 + 2;
    (addr as u32) < field_end && (addr as u32) + (len as u32) > field_addr as u32
}

struct Ctx {
    our_id: u8,
    rdt_us: u32,
    level: StatusReturnLevel,
    baud: BaudRate,
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
            Packet::Calibrate(p) => self.handle_calibrate(&ctx, p),
            // Inbound Status frames originate from another device on the bus; drop.
            Packet::Status(_) => {}
        }
    }

    fn snapshot_ctx(&self) -> Ctx {
        let (our_id, rdt_us, level, baud) = self.shared.table.config.with(|c| {
            (
                c.comms.id,
                c.comms.return_delay_2us as u32 * 2,
                c.comms.status_return_level,
                c.comms.baud_rate_idx,
            )
        });
        Ctx {
            our_id,
            rdt_us,
            level,
            baud,
        }
    }

    fn reply_unsupported(&mut self, ctx: &Ctx, target: u8) {
        if let Some((id, true)) = ctx.addressed(target) {
            self.send_status(
                ctx,
                id,
                StatusError::Instruction,
                &[],
                StatusReturnLevel::All,
                0,
            );
        }
    }

    fn send_status(
        &mut self,
        ctx: &Ctx,
        id: u8,
        error: StatusError,
        params: &[u8],
        min_level: StatusReturnLevel,
        extra_delay_us: u32,
    ) {
        if ctx.level < min_level {
            return;
        }
        let buf = self.bus.tx_buffer();
        buf.truncate(0);
        let packet = Packet::Status(StatusPacket::new(id, error.as_u8(), params));
        if write(buf, &packet).is_err() {
            buf.truncate(0);
            return;
        }
        self.bus.send_after(ctx.rdt_us + extra_delay_us);
    }

    fn reply_table_result(&mut self, ctx: &Ctx, id: u8, direct: bool, result: Result<(), Error>) {
        if !direct {
            return;
        }
        match result {
            Ok(()) => self.send_status(ctx, id, StatusError::None, &[], StatusReturnLevel::All, 0),
            Err(e) => self.send_status(ctx, id, error_to_status(e), &[], StatusReturnLevel::All, 0),
        }
    }

    fn handle_ping(&mut self, ctx: &Ctx, p: &PingPacket) {
        let Some((id, _)) = ctx.addressed(p.id) else {
            return;
        };
        let identity = self.shared.table.config.with(|c| c.identity);
        let model = identity.model_number.to_le_bytes();
        let fw = identity.firmware_version as u8;
        let params = [model[0], model[1], fw];
        self.send_status(
            ctx,
            id,
            StatusError::None,
            &params,
            StatusReturnLevel::None,
            0,
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
            self.send_status(
                ctx,
                id,
                StatusError::DataRange,
                &[],
                StatusReturnLevel::Read,
                0,
            );
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        match self.shared.table.read_bytes(p.address, &mut buf[..len]) {
            Ok(()) => self.send_status(
                ctx,
                id,
                StatusError::None,
                &buf[..len],
                StatusReturnLevel::Read,
                0,
            ),
            Err(e) => {
                self.send_status(ctx, id, error_to_status(e), &[], StatusReturnLevel::Read, 0)
            }
        }
    }

    fn handle_write(&mut self, ctx: &Ctx, p: &WritePacket<'_>) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };

        let mut buf = [0u8; MAX_CONTROL_RW];
        let len = match p.data.copy_into(&mut buf) {
            Ok(n) => n,
            Err(_) => {
                if direct {
                    self.send_status(
                        ctx,
                        id,
                        StatusError::DataRange,
                        &[],
                        StatusReturnLevel::All,
                        0,
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
        let baud_changed = result.is_ok() && touches_baud(p.address, len);
        let clock_trim_changed = result.is_ok() && touches_clock_trim(p.address, len);
        let clock_fine_trim_changed = result.is_ok() && touches_clock_fine_trim_us(p.address, len);
        self.reply_table_result(ctx, id, direct, result);
        if baud_changed {
            // reply queued, events impl defers retune until TC.
            let new_rate = self.shared.table.config.with(|c| c.comms.baud_rate_idx);
            self.events.send(Event::SetDxlBaud(new_rate));
        }
        if clock_trim_changed {
            let new_trim = self.shared.table.config.with(|c| c.comms.clock_trim);
            self.events.send(Event::SetClockTrim(new_trim));
        }
        if clock_fine_trim_changed {
            let new_q88 = self
                .shared
                .table
                .config
                .with(|c| c.comms.clock_fine_trim_us);
            self.events.send(Event::SetClockFineTrimUs(new_q88));
        }
    }

    fn handle_reg_write(&mut self, ctx: &Ctx, p: &RegWritePacket<'_>) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };

        let mut buf = [0u8; MAX_CONTROL_RW];
        let len = match p.data.copy_into(&mut buf) {
            Ok(n) => n,
            Err(_) => {
                if direct {
                    self.send_status(
                        ctx,
                        id,
                        StatusError::DataRange,
                        &[],
                        StatusReturnLevel::All,
                        0,
                    );
                }
                return;
            }
        };

        let result = self
            .shared
            .table
            .stage_bytes(p.address, &buf[..len], self.staged);
        self.reply_table_result(ctx, id, direct, result);
    }

    fn handle_action(&mut self, ctx: &Ctx, p: &ActionPacket) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };
        self.shared.table.commit_staged(self.staged);
        if direct {
            self.send_status(ctx, id, StatusError::None, &[], StatusReturnLevel::All, 0);
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
            self.send_status(
                ctx,
                id,
                StatusError::DataRange,
                &[],
                StatusReturnLevel::None,
                0,
            );
            return;
        }
        let zeros = [0u8; MAX_CONTROL_RW];
        self.send_status(
            ctx,
            id,
            StatusError::None,
            &zeros[..p.count as usize],
            StatusReturnLevel::None,
            0,
        );
    }

    fn handle_reboot(&mut self, ctx: &Ctx, p: &RebootPacket) {
        let Some((id, direct)) = ctx.addressed(p.id) else {
            return;
        };
        let mode = self.shared.table.control.with(|c| c.system.boot_mode);
        if direct {
            self.send_status(ctx, id, StatusError::None, &[], StatusReturnLevel::All, 0);
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
        let Some(slot) = p.ids.iter().position(|id| id == ctx.our_id) else {
            return;
        };

        let extra = (slot as u32) * slot_period_us(ctx.baud, p.length);

        let len = p.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.send_status(
                ctx,
                ctx.our_id,
                StatusError::DataRange,
                &[],
                StatusReturnLevel::Read,
                extra,
            );
            return;
        }

        let mut buf = [0u8; MAX_CONTROL_RW];
        match self.shared.table.read_bytes(p.address, &mut buf[..len]) {
            Ok(()) => self.send_status(
                ctx,
                ctx.our_id,
                StatusError::None,
                &buf[..len],
                StatusReturnLevel::Read,
                extra,
            ),
            Err(e) => self.send_status(
                ctx,
                ctx.our_id,
                error_to_status(e),
                &[],
                StatusReturnLevel::Read,
                extra,
            ),
        }
    }

    fn handle_sync_write(&mut self, _ctx: &Ctx, _p: &SyncWritePacket<'_>) {
        // TODO: scan (id, length-byte chunk) pairs, apply our chunk silently.
    }

    fn handle_bulk_read(&mut self, ctx: &Ctx, p: &BulkReadPacket<'_>) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let mut body = [0u8; (MAX_SLAVE_COUNT * BULK_REQUEST_SLOT_BYTES)];
        let Ok(n) = p.body.copy_into(&mut body) else {
            return;
        };
        let body = &body[..n];

        let mut found = None;
        for tup in body.chunks_exact(5) {
            if tup[0] == ctx.our_id {
                let address = u16::from_le_bytes([tup[1], tup[2]]);
                let length = u16::from_le_bytes([tup[3], tup[4]]);
                found = Some((address, length));
                break;
            }
        }
        let Some((address, length)) = found else {
            return;
        };

        let extra = bulk_slot_delay_us(body, ctx.our_id, ctx.baud).unwrap_or(0);

        let len = length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            self.send_status(
                ctx,
                ctx.our_id,
                StatusError::DataRange,
                &[],
                StatusReturnLevel::Read,
                extra,
            );
            return;
        }

        let mut buf = [0u8; MAX_CONTROL_RW];
        match self.shared.table.read_bytes(address, &mut buf[..len]) {
            Ok(()) => self.send_status(
                ctx,
                ctx.our_id,
                StatusError::None,
                &buf[..len],
                StatusReturnLevel::Read,
                extra,
            ),
            Err(e) => self.send_status(
                ctx,
                ctx.our_id,
                error_to_status(e),
                &[],
                StatusReturnLevel::Read,
                extra,
            ),
        }
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

        // Zero-fill on error keeps slot length-correct; error byte signals it.
        let mut buf = [0u8; MAX_CONTROL_RW];
        let error = match self.shared.table.read_bytes(info.address, &mut buf[..len]) {
            Ok(()) => StatusError::None,
            Err(e) => {
                buf[..len].fill(0);
                error_to_status(e)
            }
        };

        let body = FastSlotBody {
            error: error.as_u8(),
            id: ctx.our_id,
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
        let tx_buf = self.bus.tx_buffer();
        tx_buf.truncate(0);
        if write_fast_slot(tx_buf, &slot).is_err() {
            tx_buf.truncate(0);
            return;
        }

        match info.position() {
            FastSlotPosition::Only => {
                self.bus.send_after(ctx.rdt_us);
            }
            FastSlotPosition::Last => {
                let fire_q88_us = (ctx.rdt_us << 8)
                    + bytes_to_us_q88(p.bytes_before(info.our_slot), ctx.baud);
                self.bus.send_with_snoop_crc(fire_q88_us, true);
            }
            FastSlotPosition::First | FastSlotPosition::Middle => {
                let fire_us = ctx.rdt_us + bytes_to_us(p.bytes_before(info.our_slot), ctx.baud);
                self.bus.send_after(fire_us);
            }
        }
    }
}
