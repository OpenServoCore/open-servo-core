//! Chip-side control-loop layer. Mirrors `services/` for the other half of
//! the osc-core trait surface (`Sensors`, `Motor`, `ControlIo`) — each
//! submodule binds 1:1 to its osc-core trait; the bundle here impls
//! `ControlIo` so the `Kernel` can reach both via `parts()`.

pub mod motor;
pub mod sensors;

pub use motor::Ch32Motor;
pub use sensors::{Ch32Sensors, Scales};

use osc_core::ControlIo;

use crate::cfg::{BoardConfig, Precomputed};
use crate::runtime::init::BringupResult;

pub struct Ch32ControlIo {
    pub sensors: Ch32Sensors,
    pub motor: Ch32Motor,
}

impl Ch32ControlIo {
    pub fn new(cfg: BoardConfig, pre: Precomputed) -> Self {
        crate::log::info!("Ch32ControlIo::new: start");
        let BoardConfig {
            wiring,
            calibration,
            defaults,
        } = cfg;

        crate::log::debug!(
            "scales: vbus_q32={} vmotor_q32={} shunt_q32={}",
            pre.scales.vbus_q32,
            pre.scales.vmotor_q32,
            pre.scales.shunt_q32,
        );

        let in1 = wiring.motor.in1;
        let in2 = wiring.motor.in2;
        let drv_en = wiring.motor.drv_en;

        let BringupResult { shunt_bias_raw } = crate::runtime::bringup(&wiring, &defaults, &pre);

        crate::log::info!("Ch32ControlIo::new: complete");
        Self {
            sensors: Ch32Sensors::new(calibration, shunt_bias_raw, pre.scales),
            motor: Ch32Motor::new(in1, in2, drv_en, pre.pwm_arr),
        }
    }
}

impl ControlIo for Ch32ControlIo {
    type Sensors = Ch32Sensors;
    type Motor = Ch32Motor;

    fn parts(&mut self) -> (&mut Ch32Sensors, &mut Ch32Motor) {
        (&mut self.sensors, &mut self.motor)
    }
}
