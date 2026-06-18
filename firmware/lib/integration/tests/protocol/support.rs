use dxl_protocol::types::{Id, Instruction, PingStatus, Status, StatusError};
use osc_integration::sim::{DeviceId, Host, Servo, Sim, SimTime, parse_status_stream};

pub struct Setup {
    pub sim: Sim,
    pub host: DeviceId,
    #[allow(dead_code)]
    pub servos: Vec<DeviceId>,
}

/// Build a host + `n_servos` servos at DXL ids `1..=n_servos`.
pub fn setup(n_servos: usize) -> Setup {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    let servos = (1..=n_servos)
        .map(|i| sim.add_device(move |id| Servo::setup(id, |s| s.set_dxl_id(Id::new(i as u8)))))
        .collect();
    Setup { sim, host, servos }
}

/// Clear host logs, broadcast Ping, and assert every servo replied with an
/// OK status in chain order. Used as a recovery check at the end of tests
/// that exercise chain-collapse or error paths to confirm the bus didn't
/// lock up and no servo is stuck mid-parser.
#[allow(dead_code)]
pub fn assert_bus_healthy(sim: &mut Sim, host: DeviceId, servos: &[DeviceId]) {
    sim.device_mut::<Host>(host).clear_logs();
    sim.device_mut::<Host>(host).send_ping(Id::BROADCAST);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    let replies = parse_status_stream(Instruction::Ping, &rx);
    assert_eq!(
        replies.len(),
        servos.len(),
        "expected {} ping replies, got {}: {:?}",
        servos.len(),
        replies.len(),
        replies,
    );
    let expected_ids: Vec<u8> = servos
        .iter()
        .map(|d| sim.device::<Servo>(*d).dxl_id().as_byte())
        .collect();
    for (reply, expected_id) in replies.iter().zip(expected_ids.iter()) {
        match reply {
            Status::Ping {
                id,
                error,
                status: PingStatus { .. },
            } => {
                assert_eq!(
                    id.as_byte(),
                    *expected_id,
                    "out-of-order ping reply: {:?}",
                    reply
                );
                assert_eq!(*error, StatusError::OK, "unhealthy reply: {:?}", reply);
            }
            other => panic!("expected Status::Ping, got {:?}", other),
        }
    }
}
