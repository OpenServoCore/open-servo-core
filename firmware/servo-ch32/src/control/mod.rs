//! Chip-side control-loop layer. Mirrors `services/` for the other half of
//! the osc-servo-core trait surface (`Sensors`, `Motor`, `ControlIo`) -- each
//! submodule binds 1:1 to its osc-servo-core trait; the bundle here impls
//! `ControlIo` so the `Kernel` can reach both via `parts()`.

pub mod motor;
pub mod sensors;

pub use motor::Ch32Motor;
pub use sensors::Ch32Sensors;

use osc_servo_core::ControlIo;

use crate::cfg::{BoardConfig, Precomputed, chip};

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
            model,
            hw_rev,
        } = cfg;

        let drv_en_pin = wiring.drv_en.pin.pin();
        let drv_en_active = wiring.drv_en.active;

        crate::runtime::bringup(&wiring, &calibration, &defaults, model, hw_rev, &pre);

        crate::log::info!("Ch32ControlIo::new: complete");
        Self {
            sensors: Ch32Sensors::new(),
            motor: Ch32Motor::new(
                chip::MOTOR_IN1_CH,
                chip::MOTOR_IN2_CH,
                drv_en_pin,
                drv_en_active,
                pre.pwm_arr,
            ),
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
