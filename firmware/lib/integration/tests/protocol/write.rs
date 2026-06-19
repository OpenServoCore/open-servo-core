use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::addr::{comms, identity},
};
use osc_core::{BaudRate, RegionStorage, StatusReturnLevel};
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, Host, Servo, Sim, SimTime, format_hex, parse_status,
};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_INTRA_GAP_ADDR: u16 = 100;
const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;
const OVER_MAX_CONTROL_RW: usize = 129;
const NEW_ID: u8 = 2;

#[apply(matrix)]
fn write_to_comms_id_returns_ok_and_mutates_table(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_write(Id::new(1), comms::ID, &[NEW_ID]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    insta::assert_snapshot!(
        "write_to_comms_id_returns_ok_and_mutates_table",
        format_hex(&rx)
    );
    assert_eq!(
        parse_status(Instruction::Write, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert_eq!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .config
            .with(|c| c.comms.id),
        NEW_ID,
    );
}

#[apply(matrix)]
fn write_length_over_cap_replies_data_range(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    // 0x55 alternates each bit so every wire byte produces ~7 falling edges;
    // 0x00 produces only the start-bit edge and at 1 Mbaud the edge-ring HT
    // poll cadence can't drain the 64-byte RX ring before it wraps. The cap
    // rejection itself is baud-independent — this is just enough edge density
    // for the parser to receive the whole frame and reach `commit`.
    let payload = vec![0x55u8; OVER_MAX_CONTROL_RW];
    let rx = write_with(Id::new(1), comms::ID, &payload, baud, rdt_us);
    assert_eq!(
        parse_status(Instruction::Write, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::DataRange),
        },
    );
}

#[apply(matrix)]
fn write_to_ro_field_replies_access_error(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_write(Id::new(1), identity::FIRMWARE_VERSION, &[0x42]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert_eq!(
        parse_status(Instruction::Write, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
    assert_eq!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .config
            .with(|c| c.identity.firmware_version),
        DEFAULT_FIRMWARE_VERSION,
    );
}

#[apply(matrix)]
fn write_to_intra_region_gap_replies_access_error(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = write_with(Id::new(1), CONFIG_INTRA_GAP_ADDR, &[0x01], baud, rdt_us);
    assert_eq!(
        parse_status(Instruction::Write, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
}

#[apply(matrix)]
fn write_across_region_boundary_replies_data_range(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = write_with(
        Id::new(1),
        CONFIG_REGION_END_ADDR - 1,
        &[0, 0, 0, 0],
        baud,
        rdt_us,
    );
    assert_eq!(
        parse_status(Instruction::Write, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::DataRange),
        },
    );
}

/// Write at `clock_trim` (RW i8) extending 2 bytes — the second byte lands in
/// the `_rsvd_align` skip-gap inside `ConfigComms`. The whole write must reject
/// (stage_bytes rolls back via internal snapshot) so the underlying RW byte
/// stays at its seed default.
#[apply(matrix)]
fn write_partial_overlap_rw_into_gap_replies_access_error(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_write(Id::new(1), comms::CLOCK_TRIM, &[0x7F, 0x00]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert_eq!(
        parse_status(Instruction::Write, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
    assert_eq!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .config
            .with(|c| c.comms.clock_trim),
        0,
    );
}

#[apply(matrix)]
fn write_under_torque_lock_replies_access_error(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);
    sim.device::<Servo>(servos[0]).set_torque_enabled(true);

    sim.device_mut::<Host>(host)
        .send_write(Id::new(1), comms::ID, &[NEW_ID]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert_eq!(
        parse_status(Instruction::Write, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
    assert_eq!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .config
            .with(|c| c.comms.id),
        1,
    );
}

#[apply(matrix)]
fn write_to_wrong_id_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_write(Id::new(2), comms::ID, &[NEW_ID]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
    assert_eq!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .config
            .with(|c| c.comms.id),
        1,
    );
}

#[apply(matrix)]
fn write_broadcast_mutates_table_silently(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_write(Id::BROADCAST, comms::ID, &[NEW_ID]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(
        rx.is_empty(),
        "broadcast write must not reply, got {:?}",
        rx
    );
    assert_eq!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .config
            .with(|c| c.comms.id),
        NEW_ID,
    );
}

#[apply(matrix)]
fn write_silent_when_srl_is_none_but_table_still_mutates(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let (rx, id_after) =
        write_under_srl(StatusReturnLevel::None, comms::ID, &[NEW_ID], baud, rdt_us);
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
    assert_eq!(id_after, NEW_ID);
}

#[apply(matrix)]
fn write_silent_when_srl_is_read_but_table_still_mutates(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let (rx, id_after) =
        write_under_srl(StatusReturnLevel::Read, comms::ID, &[NEW_ID], baud, rdt_us);
    assert!(rx.is_empty(), "write ack only at SRL=All; got {:?}", rx,);
    assert_eq!(id_after, NEW_ID);
}

fn write_with(target: Id, addr: u16, data: &[u8], baud: BaudRate, rdt_us: u32) -> Vec<u8> {
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);
    sim.device_mut::<Host>(host).send_write(target, addr, data);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device::<Host>(host).rx_bytes()
}

fn write_under_srl(
    level: StatusReturnLevel,
    addr: u16,
    data: &[u8],
    baud: BaudRate,
    rdt_us: u32,
) -> (Vec<u8>, u8) {
    let mut sim = Sim::default();
    let host = sim.add_device(move |id| Host::new(id).with_baud(baud));
    let servo = sim.add_device(move |id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_baud(baud);
            s.set_rdt_us(rdt_us);
            s.set_status_return_level(level);
        })
    });

    sim.device_mut::<Host>(host)
        .send_write(Id::new(1), addr, data);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    let id_after = sim
        .device::<Servo>(servo)
        .shared()
        .table
        .config
        .with(|c| c.comms.id);
    (rx, id_after)
}
