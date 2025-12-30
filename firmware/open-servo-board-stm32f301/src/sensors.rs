//! Sensor frame construction from ADC DMA buffer.

use core::ptr::addr_of;

use open_servo_hw::v2::io::SensorFrame;
#[cfg(feature = "voltage-sense-motor")]
use open_servo_hw::v2::samples::MotorVoltage;
use open_servo_hw::v2::samples::{MotorCurrent, Sampled};
use open_servo_units::{CentiC, MilliAmp, MilliVolt};

use crate::adc_config::idx;
use crate::calibration::{adc_to_position, vdd_from_vrefint};
#[cfg(feature = "current-sense-bus")]
use crate::calibration::adc_to_current_ma;
#[cfg(feature = "temp-sense-mcu")]
use crate::calibration::adc_to_mcu_temp;
#[cfg(feature = "voltage-sense-motor")]
use crate::calibration::adc_to_motor_mv;
#[cfg(feature = "temp-sense-motor")]
use crate::calibration::adc_to_ntc_temp;
use crate::resources::ADC_DMA_BUF;

/// Read sensor frame from ADC DMA buffer.
///
/// # Safety
/// Must be called from ADC DMA ISR after transfer complete.
/// The DMA buffer is valid and stable at this point.
#[inline]
pub unsafe fn read_sensor_frame() -> SensorFrame {
    // SAFETY: DMA transfer is complete, buffer is stable. Using addr_of to
    // avoid creating a reference to the mutable static.
    let buf = &*addr_of!(ADC_DMA_BUF);

    // Always available: VDD from VREFINT calibration
    let vdd_mv = vdd_from_vrefint(buf[idx::VREFINT]);

    // Always available: position
    let pos = adc_to_position(buf[idx::POSITION]);

    // Motor current (feature-gated)
    #[cfg(feature = "current-sense-bus")]
    let current = MotorCurrent::Bdc(MilliAmp::from_ma(adc_to_current_ma(buf[idx::CURRENT], vdd_mv)));
    #[cfg(not(feature = "current-sense-bus"))]
    let current = MotorCurrent::default();

    // Ambient temperature from MCU internal sensor (feature-gated)
    #[cfg(feature = "temp-sense-mcu")]
    let ambient_temp = CentiC::from_centi_c(adc_to_mcu_temp(buf[idx::MCU_TEMP], vdd_mv));
    #[cfg(not(feature = "temp-sense-mcu"))]
    let ambient_temp = CentiC::from_centi_c(2500); // Fallback 25°C

    // Motor temperature from NTC (feature-gated, optional)
    #[cfg(feature = "temp-sense-motor")]
    let motor_temp = Sampled::Value(CentiC::from_centi_c(adc_to_ntc_temp(buf[idx::MOTOR_TEMP], vdd_mv)));
    #[cfg(not(feature = "temp-sense-motor"))]
    let motor_temp = Sampled::Unavailable;

    // Motor voltage from dividers (feature-gated, optional)
    #[cfg(feature = "voltage-sense-motor")]
    let motor_v = Sampled::Value(MotorVoltage::Bdc {
        a: MilliVolt::from_mv(adc_to_motor_mv(buf[idx::VOLTAGE_A], vdd_mv)),
        b: MilliVolt::from_mv(adc_to_motor_mv(buf[idx::VOLTAGE_B], vdd_mv)),
    });
    #[cfg(not(feature = "voltage-sense-motor"))]
    let motor_v = Sampled::Unavailable;

    SensorFrame {
        pos,
        current,
        mcu_vdd: MilliVolt::from_mv(vdd_mv),
        vsys: Sampled::Unavailable, // Not implemented on this board
        ambient_temp,
        motor_temp,
        motor_pos: Sampled::Unavailable, // No encoder on this board
        motor_v,
        driver_ok: true, // TODO: read DRV8231A fault pin
    }
}
