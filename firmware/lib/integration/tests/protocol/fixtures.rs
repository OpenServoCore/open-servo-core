use dxl_protocol::types::Id;
use osc_core::BaudRate;
use osc_integration::sim::{Clock, DeviceId, Host, Servo, Sim};
use rstest::fixture;

#[allow(dead_code)]
pub struct OneServo {
    pub sim: Sim,
    pub host: DeviceId,
    pub servo: DeviceId,
}

#[fixture]
#[allow(dead_code)]
pub fn one_servo() -> OneServo {
    const CLOCK: Clock = Clock::new(48_000_000);
    const BAUD: BaudRate = BaudRate::B115200;
    let mut sim = Sim::default();
    let host = sim.add_device(|id| Host::new(id, CLOCK, BAUD));
    let servo = sim.add_device(|id| Servo::new(id, CLOCK, BAUD).with_dxl_id(Id::new(0)));
    OneServo { sim, host, servo }
}
