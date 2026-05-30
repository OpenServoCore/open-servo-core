use dxl_protocol::prelude::*;
use heapless::Vec;

use crate::traits::{DeviceControl, DxlBus, ServicesIo};
use crate::{BootMode, RegionStorage, RxSnapshot, Shared, StatusReturnLevel};

use super::Dxl;

struct FakeBus {
    rx_ring: [u8; 256],
    rx_write_pos: u16,
    rx_bytes_at_idle: u32,
    tx: Vec<u8, 256>,
    send_count: u32,
    after_count: u32,
    last_after_delay_us: Option<u32>,
    snoop_count: u32,
    last_snoop_delay_us: Option<u32>,
    last_snoop_from: Option<Option<u32>>,
    last_request_end: Option<u32>,
}

impl FakeBus {
    fn new() -> Self {
        Self {
            rx_ring: [0; 256],
            rx_write_pos: 0,
            rx_bytes_at_idle: 0,
            tx: Vec::new(),
            send_count: 0,
            after_count: 0,
            last_after_delay_us: None,
            snoop_count: 0,
            last_snoop_delay_us: None,
            last_snoop_from: None,
            last_request_end: None,
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

impl DxlBus for FakeBus {
    type ReplyBuffer = Vec<u8, 256>;

    fn received(&self) -> RxSnapshot<'_> {
        RxSnapshot::new(&self.rx_ring, self.rx_write_pos)
    }

    fn reply_buffer(&mut self) -> &mut Self::ReplyBuffer {
        &mut self.tx
    }

    fn request_complete(&mut self, request_end: u32) -> bool {
        self.last_request_end = Some(request_end);
        self.rx_bytes_at_idle == request_end
    }

    fn send_after(&mut self, delay_us: u32) {
        if delay_us == 0 {
            self.send_count += 1;
            return;
        }
        self.after_count += 1;
        self.last_after_delay_us = Some(delay_us);
    }

    fn send_with_snoop_crc(&mut self, delay_us: u32, snoop_from: Option<u32>) {
        self.snoop_count += 1;
        self.last_snoop_delay_us = Some(delay_us);
        self.last_snoop_from = Some(snoop_from);
    }
}

struct FakeDevice {
    reboot_count: u32,
    last_reboot_mode: Option<BootMode>,
}

impl FakeDevice {
    fn new() -> Self {
        Self {
            reboot_count: 0,
            last_reboot_mode: None,
        }
    }
}

impl DeviceControl for FakeDevice {
    fn reboot(&mut self, mode: BootMode) {
        self.reboot_count += 1;
        self.last_reboot_mode = Some(mode);
    }
}

struct FakeIo {
    bus: FakeBus,
    device: FakeDevice,
}

impl FakeIo {
    fn new() -> Self {
        Self {
            bus: FakeBus::new(),
            device: FakeDevice::new(),
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        self.bus.feed(bytes);
    }
}

impl ServicesIo for FakeIo {
    type Bus = FakeBus;
    type Device = FakeDevice;

    fn parts(&mut self) -> (&mut FakeBus, &mut FakeDevice) {
        (&mut self.bus, &mut self.device)
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
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
}

#[test]
fn ping_to_broadcast_replies() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, _) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn ping_to_other_id_silent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(17)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn read_model_number_returns_two_bytes() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn read_zero_length_rejects_with_data_range() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn unsupported_instruction_replies_instruction_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FactoryReset(FactoryResetPacket::new(0, 0xFF)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::Instruction.as_u8());
}

#[test]
fn write_to_rw_address_succeeds_and_mutates() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn write_to_ro_address_replies_access_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, 0, &[0xAA, 0xBB])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::Access.as_u8());
    let identity = shared.table.config.with(|c| c.identity);
    assert_eq!(identity.model_number, 0);
}

#[test]
fn write_to_unmapped_address_replies_data_range_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, 0xFFFE, &[0x01])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(
        17,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(!lc.torque_enable);
}

#[test]
fn broadcast_write_applies_but_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(
        BROADCAST_ID,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn reg_write_then_action_commits_to_live_table() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    io.bus.tx.clear();
    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn reg_write_to_ro_address_replies_access_error_immediately() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(0, 0, &[0xAA, 0xBB])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::Access.as_u8());
}

#[test]
fn reg_write_invalid_value_rejected_at_stage_time() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[2],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());

    io.bus.tx.clear();
    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_clears_pending_reg_write_staging() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    io.bus.tx.clear();
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

    io.bus.tx.clear();
    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn broadcast_reg_write_and_action_silent_but_commits() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        BROADCAST_ID,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);

    let req = encode(&Packet::Action(ActionPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn action_with_empty_staging_replies_ok() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
}

#[test]
fn reboot_to_our_id_acks_and_calls_device_reboot() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert_eq!(io.device.reboot_count, 1);
    assert_eq!(io.device.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_broadcast_fires_device_reboot_without_ack() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert_eq!(io.device.reboot_count, 1);
    assert_eq!(io.device.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_other_id_silent_and_no_request() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(17)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert_eq!(io.device.reboot_count, 0);
    assert_eq!(io.device.last_reboot_mode, None);
}

#[test]
fn reboot_honors_staged_boot_mode() {
    use crate::regions::control::addr::system::BOOT_MODE;
    let shared = Shared::new();
    let mut io = FakeIo::new();
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

    assert_eq!(io.device.reboot_count, 1);
    assert_eq!(io.device.last_reboot_mode, Some(BootMode::Bootloader));
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
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_read_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn return_level_read_silences_write_ack_but_read_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_write_error_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, 0, &[0xAA, 0xBB])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn return_level_none_silences_unsupported_instruction_error() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FactoryReset(FactoryResetPacket::new(0, 0xFF)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn return_level_none_silences_action_ack() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
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

    assert_eq!(io.bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn return_level_none_silences_reboot_ack_but_reboot_still_fires() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert_eq!(io.device.reboot_count, 1);
    assert_eq!(io.device.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn return_level_none_still_replies_to_ping() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
}

#[test]
fn return_level_read_replies_to_read_errors() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0xFFFE, 1)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn sync_read_in_slot_zero_replies_immediately() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    // slot 0 → delay 0 → FakeBus short-circuits to send.
    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.after_count, 0);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn sync_read_in_later_slot_schedules_with_slot_delay() {
    use super::slot::slot_period_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    // ids = [9, 7, 0] → our slot index = 2.
    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[9, 7, 0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 1);
    let expected = 2 * slot_period_us(BaudRate::B1000000, 2);
    assert_eq!(io.bus.last_after_delay_us, Some(expected));
}

#[test]
fn sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[5, 7, 9])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn sync_read_skips_when_request_incomplete() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    // Force request_complete to return false by advancing the idle counter past parsed_end.
    io.bus.rx_bytes_at_idle = io.bus.rx_bytes_at_idle.wrapping_add(1);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn sync_read_data_range_error_still_replies_in_slot() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0xFFFE, 1, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn bulk_read_in_slot_zero_replies_immediately() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.after_count, 0);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_in_later_slot_sums_preceding_slot_periods() {
    use super::slot::slot_period_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0, 0, 4, 0, 7, 0, 0, 8, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 1);
    let expected = slot_period_us(BaudRate::B1000000, 4) + slot_period_us(BaudRate::B1000000, 8);
    assert_eq!(io.bus.last_after_delay_us, Some(expected));
}

#[test]
fn bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn bulk_read_skips_when_request_incomplete() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    io.bus.rx_bytes_at_idle = io.bus.rx_bytes_at_idle.wrapping_add(1);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0xFE, 0xFF, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, params) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_data_range_error_still_replies_in_slot() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0xFE, 0xFF, 1, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn send_after_zero_short_circuits_to_send() {
    let mut bus = FakeBus::new();
    bus.send_after(0);
    assert_eq!(bus.send_count, 1);
    assert_eq!(bus.after_count, 0);
    assert_eq!(bus.last_after_delay_us, None);
}

#[test]
fn send_after_nonzero_schedules_without_firing() {
    let mut bus = FakeBus::new();
    bus.send_after(150);
    assert_eq!(bus.send_count, 0);
    assert_eq!(bus.after_count, 1);
    assert_eq!(bus.last_after_delay_us, Some(150));
}

#[test]
fn send_with_snoop_crc_records_args() {
    let mut bus = FakeBus::new();
    bus.send_with_snoop_crc(66, Some(24));
    assert_eq!(bus.snoop_count, 1);
    assert_eq!(bus.last_snoop_delay_us, Some(66));
    assert_eq!(bus.last_snoop_from, Some(Some(24)));
}

#[test]
fn fast_sync_read_only_slot_emits_only_via_send_with_computed_crc() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    // Only-slot computes CRC locally and fires through plain send — no
    // snoop, no slot-timed delay.
    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.snoop_count, 0);
    assert_eq!(io.bus.after_count, 0);

    assert_eq!(io.bus.tx.len(), 14);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[7, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(io.bus.tx[8], 0);
    assert_eq!(io.bus.tx[9], 0);
    assert_eq!(&io.bus.tx[10..12], &[0, 0]);
    let expected_crc = dxl_protocol::crc16(&io.bus.tx[..12]).to_le_bytes();
    assert_eq!(&io.bus.tx[12..14], &expected_crc);
}

#[test]
fn fast_sync_read_first_slot_emits_header_no_crc_via_send() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[0, 7],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    // slot 0 → delay 0 → FakeBus short-circuits to send.
    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);

    assert_eq!(io.bus.tx.len(), 12);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[11, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(&io.bus.tx[8..12], &[0, 0, 0, 0]);
}

#[test]
fn fast_sync_read_middle_slot_emits_body_only_with_offset_delay() {
    use super::slot::bytes_to_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let p = FastSyncReadPacket::new(0, 2, &[9, 0, 7]);
    let req = encode(&Packet::FastSyncRead(p));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.after_count, 1);
    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    let expected = bytes_to_us(p.bytes_before(1), BaudRate::B1000000);
    assert_eq!(io.bus.last_after_delay_us, Some(expected));

    assert_eq!(io.bus.tx.len(), 4);
    assert_eq!(&io.bus.tx[..], &[0, 0, 0, 0]);
}

#[test]
fn fast_sync_read_last_slot_schedules_snoop_with_parsed_end() {
    use super::slot::bytes_to_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let p = FastSyncReadPacket::new(0, 2, &[9, 0]);
    let req = encode(&Packet::FastSyncRead(p));
    let parsed_end = req.len() as u32;
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.snoop_count, 1);
    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    let expected_fire = bytes_to_us(p.bytes_before(1), BaudRate::B1000000);
    assert_eq!(io.bus.last_snoop_delay_us, Some(expected_fire));
    assert_eq!(io.bus.last_snoop_from, Some(Some(parsed_end)));

    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0xAA, 0xBB]);
}

#[test]
fn fast_sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[5, 7, 9],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_sync_read_skips_when_request_incomplete() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    io.bus.rx_bytes_at_idle = io.bus.rx_bytes_at_idle.wrapping_add(1);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_only_slot_emits_only_via_send_with_computed_crc() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.snoop_count, 0);
    assert_eq!(io.bus.after_count, 0);

    assert_eq!(io.bus.tx.len(), 14);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[7, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(io.bus.tx[8], 0);
    assert_eq!(io.bus.tx[9], 0);
    assert_eq!(&io.bus.tx[10..12], &[0, 0]);
    let expected_crc = dxl_protocol::crc16(&io.bus.tx[..12]).to_le_bytes();
    assert_eq!(&io.bus.tx[12..14], &expected_crc);
}

#[test]
fn fast_bulk_read_first_slot_emits_header_no_crc_via_send() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0, 7, 0, 0, 4, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);

    assert_eq!(io.bus.tx.len(), 12);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[13, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(&io.bus.tx[8..12], &[0, 0, 0, 0]);
}

#[test]
fn fast_bulk_read_middle_slot_uses_per_slot_lengths_for_delay() {
    use super::slot::bytes_to_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0, 0, 4, 0, 0, 0, 0, 2, 0, 7, 0, 0, 8, 0];
    let p = FastBulkReadPacket::new(&body);
    let req = encode(&Packet::FastBulkRead(p));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.after_count, 1);
    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    let expected = bytes_to_us(p.bytes_before(1), BaudRate::B1000000);
    assert_eq!(io.bus.last_after_delay_us, Some(expected));

    assert_eq!(io.bus.tx.len(), 4);
    assert_eq!(&io.bus.tx[..], &[0, 0, 0, 0]);
}

#[test]
fn fast_bulk_read_last_slot_schedules_snoop_with_parsed_end() {
    use super::slot::bytes_to_us;
    use crate::BaudRate;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0, 0, 4, 0, 0, 0, 0, 2, 0];
    let p = FastBulkReadPacket::new(&body);
    let req = encode(&Packet::FastBulkRead(p));
    let parsed_end = req.len() as u32;
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.snoop_count, 1);
    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    let expected_fire = bytes_to_us(p.bytes_before(1), BaudRate::B1000000);
    assert_eq!(io.bus.last_snoop_delay_us, Some(expected_fire));
    assert_eq!(io.bus.last_snoop_from, Some(Some(parsed_end)));

    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0xAA, 0xBB]);
}

#[test]
fn fast_bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0xFE, 0xFF, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.snoop_count, 1);
    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0xAA, 0xBB]);
}

#[test]
fn fast_bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_skips_when_request_incomplete() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    io.bus.rx_bytes_at_idle = io.bus.rx_bytes_at_idle.wrapping_add(1);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert_eq!(io.bus.snoop_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn calibrate_unicast_replies_with_zero_payload() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Calibrate(CalibratePacket::new(0, 16)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(params.len(), 16);
    assert!(params.iter().all(|&b| b == 0));
}

#[test]
fn calibrate_unicast_count_max_replies() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Calibrate(CalibratePacket::new(0, 128)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let tx = &io.bus.tx[..];
    // hdr(4) + id(1) + len(2) + inst(1) + err(1) + 128 zero bytes + crc(2)
    assert_eq!(tx.len(), 139);
    assert_eq!(&tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0]);
    assert_eq!(&tx[5..7], &[132, 0]);
    assert_eq!(tx[7], 0x55);
    assert_eq!(tx[8], 0);
    assert!(tx[9..137].iter().all(|&b| b == 0));
}

#[test]
fn calibrate_unicast_count_zero_data_range_err() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Calibrate(CalibratePacket::new(0, 0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, params) = parse_status(&io.bus.tx);
    assert_eq!(err, 0x04);
    assert!(params.is_empty());
}

#[test]
fn calibrate_unicast_count_over_max_data_range_err() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Calibrate(CalibratePacket::new(0, 129)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, params) = parse_status(&io.bus.tx);
    assert_eq!(err, 0x04);
    assert!(params.is_empty());
}

#[test]
fn calibrate_broadcast_silently_dropped() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Calibrate(CalibratePacket::new(BROADCAST_ID, 128)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert_eq!(io.bus.after_count, 0);
    assert!(io.bus.tx.is_empty());
}
