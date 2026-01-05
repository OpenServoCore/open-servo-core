//! Sensor frame construction from ADC DMA buffer.

use core::ptr::addr_of;

use open_servo_hw::v2::io::SensorFrame;
use open_servo_hw::v2::samples::{MotorCurrent, MotorCurrentRaw, Reading};
#[cfg(feature = "voltage-sense-motor")]
use open_servo_hw::v2::samples::{MotorVoltage, MotorVoltageRaw};
use open_servo_units::{CentiC, MilliAmp, MilliVolt};

use crate::adc_config::Channels;
#[cfg(feature = "current-sense-bus")]
use crate::calibration::adc_to_current_ma;
#[cfg(feature = "temp-sense-mcu")]
use crate::calibration::adc_to_mcu_temp;
#[cfg(feature = "voltage-sense-motor")]
use crate::calibration::adc_to_motor_mv;
#[cfg(feature = "temp-sense-motor")]
use crate::calibration::adc_to_ntc_temp;
use crate::calibration::{adc_to_position, vdd_from_vrefint};
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
    let ch = Channels::new(buf);

    // Always available: VDD from VREFINT calibration
    let vrefint_raw = ch.vrefint();
    let vdd_mv = vdd_from_vrefint(vrefint_raw);

    // Always available: position
    let pos_raw = ch.position();
    let pos = Reading::Valid {
        value: adc_to_position(pos_raw),
        raw: pos_raw,
    };

    // Motor current (feature-gated)
    #[cfg(feature = "current-sense-bus")]
    let current = {
        let raw = ch.current();
        Reading::Valid {
            value: MotorCurrent::Bdc(MilliAmp::from_ma(adc_to_current_ma(raw, vdd_mv))),
            raw: MotorCurrentRaw::Bdc(raw),
        }
    };
    #[cfg(not(feature = "current-sense-bus"))]
    let current = Reading::Invalid {
        raw: MotorCurrentRaw::default(),
    };

    // MCU VDD with VREFINT as raw value
    let mcu_vdd = Reading::Valid {
        value: MilliVolt::from_mv(vdd_mv),
        raw: vrefint_raw,
    };

    // Ambient temperature from MCU internal sensor (feature-gated)
    #[cfg(feature = "temp-sense-mcu")]
    let ambient_temp = {
        let raw = ch.mcu_temp();
        Reading::Valid {
            value: CentiC::from_centi_c(adc_to_mcu_temp(raw, vdd_mv)),
            raw,
        }
    };
    #[cfg(not(feature = "temp-sense-mcu"))]
    let ambient_temp = Reading::Valid {
        value: CentiC::from_centi_c(2500), // Fallback 25°C
        raw: 0,
    };

    // Motor temperature from NTC (feature-gated, optional)
    #[cfg(feature = "temp-sense-motor")]
    let motor_temp = {
        let raw = ch.motor_temp();
        Some(Reading::Valid {
            value: CentiC::from_centi_c(adc_to_ntc_temp(raw, vdd_mv)),
            raw,
        })
    };
    #[cfg(not(feature = "temp-sense-motor"))]
    let motor_temp = None;

    // Motor voltage from dividers (feature-gated, optional)
    #[cfg(feature = "voltage-sense-motor")]
    let motor_v = {
        let raw_a = ch.voltage_a();
        let raw_b = ch.voltage_b();
        Some(Reading::Valid {
            value: MotorVoltage::Bdc {
                a: MilliVolt::from_mv(adc_to_motor_mv(raw_a, vdd_mv)),
                b: MilliVolt::from_mv(adc_to_motor_mv(raw_b, vdd_mv)),
            },
            raw: MotorVoltageRaw::Bdc(raw_a, raw_b),
        })
    };
    #[cfg(not(feature = "voltage-sense-motor"))]
    let motor_v = None;

    SensorFrame {
        pos,
        current,
        mcu_vdd,
        vsys: None, // Not implemented on this board
        ambient_temp,
        motor_temp,
        motor_pos: None, // No encoder on this board
        motor_v,
        driver_ok: true, // TODO: read DRV8231A fault pin
    }
}
