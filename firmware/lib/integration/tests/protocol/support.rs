use dxl_protocol::types::Id;
use osc_integration::sim::{DeviceId, Host, Servo, Sim};

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
        .map(|i| sim.add_device(move |id| Servo::new(id).with_dxl_id(Id::new(i as u8))))
        .collect();
    Setup { sim, host, servos }
}
