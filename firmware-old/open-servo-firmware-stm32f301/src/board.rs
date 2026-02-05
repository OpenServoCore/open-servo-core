//! Board implementation for STM32F301.
//!
//! Implements the `Board`, `Timebase`, and `BoardConfig` traits,
//! consolidating all hardware abstraction in one place.

use core::ptr::addr_of;

use cortex_m::peripheral::SCB;
use open_servo_hw::config::{
    BoardConfig, BoardKinematicsConfig, BoardPolicyConfig, BoardSafetyConfig, BoardThermalConfig,
    DxlIdentity, EepromDefaults,
};
#[cfg(any(feature = "voltage-sense-motor", feature = "temp-sense-motor"))]
use open_servo_hw::v2::capability::SensorCapability;
use open_servo_hw::v2::capability::{MotorType, SensorCapabilities, ServoPosKind};
use open_servo_hw::v2::io::{DriveMode, MotorCommand, SensorFrame};
use open_servo_hw::v2::samples::{MotorCurrent, MotorCurrentRaw, Reading};
#[cfg(feature = "voltage-sense-motor")]
use open_servo_hw::v2::samples::{MotorVoltage, MotorVoltageRaw};
use open_servo_hw::v2::Board;
use open_servo_hw::Timebase;
use open_servo_math::{CentiDeg, ComplianceConfig, DegPerSec10, Effort};
use open_servo_units::{CentiC, MilliAmp, MilliVolt, TimeStampUs};
use stm32f3::stm32f301::{TIM1, TIM2};

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
use crate::config::PWM_ARR;
use crate::flash::{EepromFlash, EEPROM_FLASH_SIZE};
use crate::resources::ADC_DMA_BUF;

/// Motor polarity reversal. Set true if motor runs backwards.
const REVERSE_MOTOR: bool = false;

/// STM32F301 board implementation.
///
/// This struct implements all hardware abstraction traits:
/// - [`Board`]: sensor reading, motor control, peripherals
/// - [`Timebase`]: monotonic clock
/// - [`BoardConfig`]: DXL identity, safety limits, tuning
pub struct Stm32f301Board {
    /// Flash driver for EEPROM persistence (Option for take_flash support).
    flash: Option<EepromFlash>,
}

impl Stm32f301Board {
    /// Create a new board instance.
    ///
    /// # Safety
    ///
    /// Must be called exactly once. The board takes ownership of:
    /// - Flash peripheral (for EEPROM)
    /// - Assumes exclusive access to ADC DMA buffer, TIM1, TIM2
    pub unsafe fn new() -> Self {
        Self {
            flash: Some(EepromFlash::new()),
        }
    }
}

// =============================================================================
// Board trait implementation
// =============================================================================

impl Board for Stm32f301Board {
    type Uart = DummyUart; // TODO: implement real UART
    type Flash = EepromFlash;

    fn servo_pos_kind(&self) -> ServoPosKind {
        ServoPosKind::Bounded {
            min: CentiDeg::from_cdeg(-9500), // -95°
            max: CentiDeg::from_cdeg(9500),  // +95°
        }
    }

    fn motor_type(&self) -> MotorType {
        MotorType::Bdc
    }

    fn sensor_capabilities(&self) -> SensorCapabilities {
        #[allow(unused_mut)]
        let mut caps = SensorCapabilities::empty();
        #[cfg(feature = "voltage-sense-motor")]
        {
            caps = caps.with(SensorCapability::MotorVoltage);
        }
        #[cfg(feature = "temp-sense-motor")]
        {
            caps = caps.with(SensorCapability::MotorTemp);
        }
        caps
    }

    fn read_sensors(&mut self) -> SensorFrame {
        // SAFETY: Called from ISR after DMA transfer complete.
        // Using addr_of to avoid creating a reference to the mutable static.
        let buf = unsafe { &*addr_of!(ADC_DMA_BUF) };
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
            vsys: None,
            ambient_temp,
            motor_temp,
            motor_pos: None,
            motor_v,
            driver_ok: true, // TODO: read DRV8231A fault pin
        }
    }

    fn write_motor(&mut self, cmd: MotorCommand) {
        // SAFETY: Single-writer from ISR context.
        let tim1 = unsafe { &*TIM1::ptr() };

        if !cmd.driver_en {
            // Driver disabled: coast
            tim1.ccr1().write(|w| w.ccr().bits(0));
            tim1.ccr4().write(|w| w.ccr().bits(0));
            return;
        }

        match cmd.mode {
            DriveMode::Coast => {
                tim1.ccr1().write(|w| w.ccr().bits(0));
                tim1.ccr4().write(|w| w.ccr().bits(0));
            }
            DriveMode::Drive => {
                let mut effort = cmd.effort.as_raw();
                if REVERSE_MOTOR {
                    effort = effort.saturating_neg();
                }

                if effort >= 0 {
                    let duty = effort_to_duty(effort);
                    tim1.ccr1().write(|w| w.ccr().bits(duty));
                    tim1.ccr4().write(|w| w.ccr().bits(0));
                } else {
                    let duty = effort_to_duty(effort.saturating_neg());
                    tim1.ccr1().write(|w| w.ccr().bits(0));
                    tim1.ccr4().write(|w| w.ccr().bits(duty));
                }
            }
            DriveMode::Brake => {
                // Active brake: both high (slow decay)
                // Stage-0: treat as coast for safety
                tim1.ccr1().write(|w| w.ccr().bits(0));
                tim1.ccr4().write(|w| w.ccr().bits(0));
            }
        }
    }

    fn uart(&mut self) -> &mut Self::Uart {
        // TODO: implement real UART
        static mut DUMMY: DummyUart = DummyUart;
        unsafe { &mut *addr_of!(DUMMY).cast_mut() }
    }

    fn flash(&mut self) -> Option<&mut Self::Flash> {
        self.flash.as_mut()
    }

    fn take_flash(&mut self) -> Option<Self::Flash> {
        self.flash.take()
    }

    fn eeprom_flash_range(&self) -> core::ops::Range<u32> {
        0..EEPROM_FLASH_SIZE as u32
    }

    fn reboot(&self) -> ! {
        SCB::sys_reset()
    }
}

// =============================================================================
// Timebase trait implementation
// =============================================================================

impl Timebase for Stm32f301Board {
    fn now_us(&self) -> TimeStampUs {
        // SAFETY: We only read the counter register, which is safe from any context.
        let cnt = unsafe { (*TIM2::ptr()).cnt.read().bits() };
        TimeStampUs(cnt)
    }
}

// =============================================================================
// BoardConfig trait implementation
// =============================================================================

impl BoardConfig for Stm32f301Board {
    fn dxl_identity(&self) -> DxlIdentity {
        DxlIdentity {
            model_number: 0x8001,        // Open-servo SG90 variant
            model_information: 0x010000, // v1.0.0
            firmware_version: 1,
            protocol_type: 2, // DXL Protocol 2.0
        }
    }

    fn eeprom_defaults(&self) -> EepromDefaults {
        EepromDefaults {
            id: 1,
            baud_rate: 1,      // 57600
            secondary_id: 255, // disabled
        }
    }

    fn safety_config(&self) -> BoardSafetyConfig {
        BoardSafetyConfig {
            current_limit_ma: 2000,
            mcu_temp_limit_cc: 8500, // 85°C
            position_max_delta_cdeg: 500,
            sensor_fault_count: 10,
            position_min_cdeg: -9500,
            position_max_cdeg: 9500,
            stall_timeout_ticks: 1000,
            stall_position_tolerance_cdeg: 50,
            position_error_limit_cdeg: 3000,
            position_error_timeout_us: 1_000_000,
        }
    }

    fn move_compliance_config(&self) -> ComplianceConfig {
        ComplianceConfig {
            limit_ma: 2000,
            hysteresis_ma: 200,
            deglitch_samples: 3,
            backoff_factor_q8: 225, // 0.88
            recovery_rate: 3277,    // 10%/sec
        }
    }

    fn hold_compliance_config(&self) -> ComplianceConfig {
        ComplianceConfig {
            limit_ma: 1500, // Lower limit for hold mode
            hysteresis_ma: 150,
            deglitch_samples: 3,
            backoff_factor_q8: 200, // 0.78 (softer)
            recovery_rate: 1638,    // 5%/sec
        }
    }

    fn thermal_config(&self) -> BoardThermalConfig {
        BoardThermalConfig {
            resistance_mohm: 5000,       // 5Ω
            thermal_resistance_cw: 1000, // 10°C/W
            thermal_capacity_cj: 1500,   // 15 J/°C
            max_temp_cdeg: 10000,        // 100°C
            hysteresis_cdeg: 1000,       // 10°C
            default_ambient_cdeg: 2500,  // 25°C
        }
    }

    fn kinematics_config(&self) -> BoardKinematicsConfig {
        BoardKinematicsConfig {
            sensor_raw_min: 0,
            sensor_raw_max: 4095,
            sensor_min_cdeg: 0,
            sensor_max_cdeg: 36000, // 360°
            mechanical_min_cdeg: -9500,
            mechanical_max_cdeg: 9500,
            zero_offset_cdeg: 0,
            reversed: false,
        }
    }

    fn policy_config(&self) -> BoardPolicyConfig {
        BoardPolicyConfig {
            hold_enter_error: CentiDeg::from_cdeg(100),
            hold_exit_error: CentiDeg::from_cdeg(200),
            hold_enter_vel: DegPerSec10::from_dps10(50),
            hold_exit_vel: DegPerSec10::from_dps10(100),
            backdrive_vel_threshold: DegPerSec10::from_dps10(200),
            backdrive_deadband: Effort::from_raw(1000),
            backdrive_persist_us: 50_000,
            yield_alive_effort_max: Effort::from_raw(4000),
            yield_coast_us: 10_000,
            yield_duration_us: 100_000,
            hold_effort_error_start: CentiDeg::from_cdeg(50),
            hold_effort_error_end: CentiDeg::from_cdeg(500),
            hold_effort_min: Effort::from_raw(2000),
            hold_effort_max: Effort::from_raw(16000),
        }
    }

    fn pid_gains(&self) -> (i32, i32, i32) {
        (1280, 0, 1280) // kp=5.0, ki=0, kd=5.0 (Q8.8)
    }
}

// =============================================================================
// Helper functions
// =============================================================================

/// Convert effort (0..32767) to duty cycle (0..PWM_ARR).
#[inline]
fn effort_to_duty(effort: i16) -> u16 {
    let duty = (effort as u32 * PWM_ARR as u32) / 32767;
    duty as u16
}

// =============================================================================
// Dummy UART (placeholder until real implementation)
// =============================================================================

use embedded_io_async::{ErrorType, Read, Write};

/// Placeholder UART type until real async UART is implemented.
pub struct DummyUart;

impl ErrorType for DummyUart {
    type Error = core::convert::Infallible;
}

impl Read for DummyUart {
    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Block forever - real UART will provide data
        core::future::pending().await
    }
}

impl Write for DummyUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(buf.len()) // Discard
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
