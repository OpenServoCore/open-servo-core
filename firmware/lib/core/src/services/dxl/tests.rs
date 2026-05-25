use dxl_protocol::prelude::*;
use heapless::Vec;

use crate::{BootMode, RegionStorage, RxSnapshot, Shared, StatusReturnLevel};

use super::{Dxl, DxlIo};

struct FakeDxlIo {
    rx_ring: [u8; 256],
    rx_write_pos: u16,
    rx_bytes_at_idle: u32,
    rx_idle_tick: u32,
    tx: Vec<u8, 256>,
    start_tx_count: u32,
    scheduled_count: u32,
    last_scheduled_idle_tick: Option<u32>,
    last_scheduled_delay_us: Option<u32>,
    fast_scheduled_count: u32,
    last_fast_idle_tick: Option<u32>,
    last_fast_switch_us: Option<u32>,
    last_fast_fire_us: Option<u32>,
    last_fast_frame_end: Option<u32>,
    reboot_count: u32,
    reboot_immediate_count: u32,
    last_reboot_mode: Option<BootMode>,
}

impl FakeDxlIo {
    fn new() -> Self {
        Self {
            rx_ring: [0; 256],
            rx_write_pos: 0,
            rx_bytes_at_idle: 0,
            rx_idle_tick: 0,
            tx: Vec::new(),
            start_tx_count: 0,
            scheduled_count: 0,
            last_scheduled_idle_tick: None,
            last_scheduled_delay_us: None,
            fast_scheduled_count: 0,
            last_fast_idle_tick: None,
            last_fast_switch_us: None,
            last_fast_fire_us: None,
            last_fast_frame_end: None,
            reboot_count: 0,
            reboot_immediate_count: 0,
            last_reboot_mode: None,
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        for &b in bytes {
            let idx = (self.rx_write_pos as usize) % self.rx_ring.len();
            self.rx_ring[idx] = b;
            self.rx_write_pos = self.rx_write_pos.wrapping_add(1) % (self.rx_ring.len() as u16);
        }
        self.rx_bytes_at_idle = self.rx_bytes_at_idle.wrapping_add(bytes.len() as u32);
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
    fn start_tx_after(&mut self, idle_tick: u32, delay_us: u32) {
        if delay_us == 0 {
            self.start_tx();
            return;
        }
        self.scheduled_count += 1;
        self.last_scheduled_idle_tick = Some(idle_tick);
        self.last_scheduled_delay_us = Some(delay_us);
    }
    fn idle_for(&self, parsed_end: u32) -> Option<u32> {
        (self.rx_bytes_at_idle == parsed_end).then_some(self.rx_idle_tick)
    }
    fn start_fast_tx_after(
        &mut self,
        idle_tick: u32,
        switch_us: u32,
        fire_us: u32,
        frame_end: u32,
    ) {
        self.fast_scheduled_count += 1;
        self.last_fast_idle_tick = Some(idle_tick);
        self.last_fast_switch_us = Some(switch_us);
        self.last_fast_fire_us = Some(fire_us);
        self.last_fast_frame_end = Some(frame_end);
    }
    fn request_reboot(&mut self, mode: BootMode) {
        self.reboot_count += 1;
        self.last_reboot_mode = Some(mode);
        if self.tx.is_empty() {
            self.reboot_immediate_count += 1;
        }
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FactoryReset(FactoryResetPacket::new(0, 0xFF)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.tx);
    assert_eq!(err, StatusError::Instruction.as_u8());
}

#[test]
fn write_to_rw_address_succeeds_and_mutates() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
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
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.tx);
    assert_eq!(err, StatusError::None.as_u8());
}

#[test]
fn reboot_to_our_id_acks_then_defers_reset_until_tx_drains() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (id, err, params) = parse_status(&io.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert_eq!(io.reboot_count, 1);
    assert_eq!(io.reboot_immediate_count, 0);
    assert_eq!(io.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_broadcast_resets_immediately_when_bus_idle() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert!(io.tx.is_empty());
    assert_eq!(io.reboot_count, 1);
    assert_eq!(io.reboot_immediate_count, 1);
    assert_eq!(io.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_broadcast_defers_when_prior_tx_in_flight() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    io.tx.push(0xAA).unwrap();

    let req = encode(&Packet::Reboot(RebootPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.reboot_count, 1);
    assert_eq!(io.reboot_immediate_count, 0);
    assert_eq!(&io.tx[..], &[0xAA]);
}

#[test]
fn reboot_to_other_id_silent_and_no_request() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(17)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert!(io.tx.is_empty());
    assert_eq!(io.reboot_count, 0);
    assert_eq!(io.reboot_immediate_count, 0);
    assert_eq!(io.last_reboot_mode, None);
}

#[test]
fn reboot_honors_staged_boot_mode() {
    use crate::regions::control::addr::system::BOOT_MODE;
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(
        0,
        BOOT_MODE,
        &[BootMode::Bootloader as u8],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.reboot_count, 1);
    assert_eq!(io.last_reboot_mode, Some(BootMode::Bootloader));
}

fn set_level(shared: &Shared, level: StatusReturnLevel) {
    shared
        .table
        .config
        .with_mut(|c| c.comms.status_return_level = level);
}

#[test]
fn return_level_none_silences_write_ack_but_ping_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.start_tx_count, 0);
    assert!(io.tx.is_empty());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.start_tx_count, 1);
    let (_, err, _) = parse_status(&io.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_read_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.start_tx_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn return_level_read_silences_write_ack_but_read_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.start_tx_count, 0);

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.start_tx_count, 1);
    let (_, err, _) = parse_status(&io.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_write_error_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, 0, &[0xAA, 0xBB])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.start_tx_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn return_level_none_silences_unsupported_instruction_error() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FactoryReset(FactoryResetPacket::new(0, 0xFF)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.start_tx_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn return_level_none_silences_action_ack() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn return_level_none_silences_reboot_ack_but_reboot_still_fires() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert!(io.tx.is_empty());
    assert_eq!(io.reboot_count, 1);
    assert_eq!(io.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn return_level_none_still_replies_to_ping() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 1);
    let (id, err, params) = parse_status(&io.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
}

#[test]
fn return_level_read_replies_to_read_errors() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0xFFFE, 1)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn sync_read_in_slot_zero_replies_immediately() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    // slot 0 → delay 0 → FakeDxlIo short-circuits to start_tx.
    assert_eq!(io.start_tx_count, 1);
    assert_eq!(io.scheduled_count, 0);
    let (id, err, params) = parse_status(&io.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn sync_read_in_later_slot_schedules_with_slot_delay() {
    use super::slot::slot_period_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();
    io.rx_idle_tick = 42;

    // ids = [9, 7, 0] → our slot index = 2.
    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[9, 7, 0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 1);
    assert_eq!(io.last_scheduled_idle_tick, Some(42));
    let expected = 2 * slot_period_us(BaudRate::B1000000, 2);
    assert_eq!(io.last_scheduled_delay_us, Some(expected));
}

#[test]
fn sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[5, 7, 9])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn sync_read_skips_when_idle_for_returns_none() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    // Force idle_for mismatch by advancing the counter past parsed_end.
    io.rx_bytes_at_idle = io.rx_bytes_at_idle.wrapping_add(1);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn sync_read_data_range_error_still_replies_in_slot() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0xFFFE, 1, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 1);
    let (_, err, _) = parse_status(&io.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn bulk_read_in_slot_zero_replies_immediately() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    // (id=0, addr=0, length=2)
    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 1);
    assert_eq!(io.scheduled_count, 0);
    let (id, err, params) = parse_status(&io.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_in_later_slot_sums_preceding_slot_periods() {
    use super::slot::slot_period_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();
    io.rx_idle_tick = 99;

    // Preceding slots: id=9 length=4, id=7 length=8. Our slot: id=0 length=2.
    let body = [9, 0, 0, 4, 0, 7, 0, 0, 8, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 1);
    assert_eq!(io.last_scheduled_idle_tick, Some(99));
    let expected = slot_period_us(BaudRate::B1000000, 4) + slot_period_us(BaudRate::B1000000, 8);
    assert_eq!(io.last_scheduled_delay_us, Some(expected));
}

#[test]
fn bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn bulk_read_skips_when_idle_for_returns_none() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    io.rx_bytes_at_idle = io.rx_bytes_at_idle.wrapping_add(1);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    // Slot 0 (id=9): bogus address 0xFFFE; our slot (id=0): addr=0 len=2 → model_number.
    let body = [9, 0xFE, 0xFF, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, params) = parse_status(&io.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_data_range_error_still_replies_in_slot() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    // Our tuple targets an unmapped address.
    let body = [0, 0xFE, 0xFF, 1, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 1);
    let (_, err, _) = parse_status(&io.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn start_tx_after_zero_short_circuits_to_start_tx() {
    let mut io = FakeDxlIo::new();
    io.start_tx_after(0, 0);
    assert_eq!(io.start_tx_count, 1);
    assert_eq!(io.scheduled_count, 0);
    assert_eq!(io.last_scheduled_idle_tick, None);
    assert_eq!(io.last_scheduled_delay_us, None);
}

#[test]
fn start_tx_after_nonzero_schedules_without_firing() {
    let mut io = FakeDxlIo::new();
    io.start_tx_after(42, 150);
    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 1);
    assert_eq!(io.last_scheduled_idle_tick, Some(42));
    assert_eq!(io.last_scheduled_delay_us, Some(150));
}

#[test]
fn fast_sync_read_only_slot_emits_only_with_crc_placeholder() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();
    io.rx_idle_tick = 42;

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    let parsed_end = req.len() as u32;
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.fast_scheduled_count, 1);
    assert_eq!(io.last_fast_idle_tick, Some(42));
    assert_eq!(io.last_fast_switch_us, Some(0));
    assert_eq!(io.last_fast_fire_us, Some(0));
    assert_eq!(io.last_fast_frame_end, Some(parsed_end));
    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);

    // Only: header(8) + err(1) + id(1) + data(2) + crc placeholder(2) = 14
    assert_eq!(io.tx.len(), 14);
    assert_eq!(&io.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    // length = 3 + 1*(2+2) = 7
    assert_eq!(&io.tx[5..7], &[7, 0]);
    assert_eq!(io.tx[7], 0x55);
    assert_eq!(io.tx[8], 0); // error
    assert_eq!(io.tx[9], 0); // our id
    assert_eq!(&io.tx[10..12], &[0, 0]); // model_number defaults
    assert_eq!(&io.tx[12..14], &[0xAA, 0xBB]); // CRC placeholder
}

#[test]
fn fast_sync_read_first_slot_emits_header_no_crc_via_start_tx() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();
    io.rx_idle_tick = 42;

    // ids = [0, 7] → our slot = 0 of 2 → First (header + body, no CRC)
    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[0, 7],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    // slot 0 → delay 0 → FakeDxlIo short-circuits to start_tx.
    assert_eq!(io.start_tx_count, 1);
    assert_eq!(io.scheduled_count, 0);
    assert_eq!(io.fast_scheduled_count, 0);

    // First: header(8) + err(1) + id(1) + data(2) = 12, no placeholder.
    assert_eq!(io.tx.len(), 12);
    assert_eq!(&io.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    // length = 3 + 2*(2+2) = 11
    assert_eq!(&io.tx[5..7], &[11, 0]);
    assert_eq!(io.tx[7], 0x55);
    assert_eq!(&io.tx[8..12], &[0, 0, 0, 0]); // err, id, model_lo, model_hi
}

#[test]
fn fast_sync_read_middle_slot_emits_body_only_with_offset_delay() {
    use super::slot::fast_slot_delay_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();
    io.rx_idle_tick = 99;

    // ids = [9, 0, 7] → our slot = 1 of 3 → Middle (body only)
    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[9, 0, 7],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.scheduled_count, 1);
    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.fast_scheduled_count, 0);
    assert_eq!(io.last_scheduled_idle_tick, Some(99));
    let expected = fast_slot_delay_us(1, 2, BaudRate::B1000000);
    assert_eq!(io.last_scheduled_delay_us, Some(expected));

    // Middle: err(1) + id(1) + data(2) = 4
    assert_eq!(io.tx.len(), 4);
    assert_eq!(&io.tx[..], &[0, 0, 0, 0]);
}

#[test]
fn fast_sync_read_last_slot_schedules_fast_with_switch_lt_fire() {
    use super::slot::fast_slot_delay_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();
    io.rx_idle_tick = 7;

    // ids = [9, 0] → our slot = 1 of 2 → Last (body + CRC placeholder)
    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[9, 0],
    )));
    let parsed_end = req.len() as u32;
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.fast_scheduled_count, 1);
    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert_eq!(io.last_fast_idle_tick, Some(7));
    let expected_fire = fast_slot_delay_us(1, 2, BaudRate::B1000000);
    let expected_switch = fast_slot_delay_us(0, 2, BaudRate::B1000000);
    assert_eq!(io.last_fast_fire_us, Some(expected_fire));
    assert_eq!(io.last_fast_switch_us, Some(expected_switch));
    assert!(expected_switch < expected_fire);
    assert_eq!(io.last_fast_frame_end, Some(parsed_end));

    // Last: err(1) + id(1) + data(2) + crc placeholder(2) = 6
    assert_eq!(io.tx.len(), 6);
    assert_eq!(&io.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.tx[4..], &[0xAA, 0xBB]);
}

#[test]
fn fast_sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[5, 7, 9],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert_eq!(io.fast_scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn fast_sync_read_skips_when_idle_for_returns_none() {
    let shared = Shared::new();
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    io.rx_bytes_at_idle = io.rx_bytes_at_idle.wrapping_add(1);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert_eq!(io.fast_scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn fast_sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeDxlIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
    assert_eq!(io.fast_scheduled_count, 0);
    assert!(io.tx.is_empty());
}

#[test]
fn start_fast_tx_after_records_args() {
    let mut io = FakeDxlIo::new();
    io.start_fast_tx_after(42, 46, 66, 24);
    assert_eq!(io.fast_scheduled_count, 1);
    assert_eq!(io.last_fast_idle_tick, Some(42));
    assert_eq!(io.last_fast_switch_us, Some(46));
    assert_eq!(io.last_fast_fire_us, Some(66));
    assert_eq!(io.last_fast_frame_end, Some(24));
    assert_eq!(io.start_tx_count, 0);
    assert_eq!(io.scheduled_count, 0);
}
