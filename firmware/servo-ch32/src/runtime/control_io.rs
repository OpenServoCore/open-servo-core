use osc_servo_core::ControlIo;

use crate::cfg::{BoardConfig, Precomputed, chip};
use crate::providers::motor::Ch32Motor;
use crate::providers::sensors::Ch32Sensors;

/// The `ControlIo` bundle: the `Kernel` reaches the sensors and the motor
/// through `parts()`.
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
        super::burst::install(pre.burst_v_max_counts);

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
