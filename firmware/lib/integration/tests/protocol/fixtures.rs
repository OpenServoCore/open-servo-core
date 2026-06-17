use dxl_protocol::types::Id;
use osc_integration::sim::{DeviceId, Host, Servo, Sim};
use rstest::fixture;

pub struct OneServo {
    pub sim: Sim,
    pub host: DeviceId,
    pub servo: DeviceId,
}

#[fixture]
pub fn one_servo() -> OneServo {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    let servo = sim.add_device(|id| Servo::new(id).with_dxl_id(Id::new(0)));
    OneServo { sim, host, servo }
}
