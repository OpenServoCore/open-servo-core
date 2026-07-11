//! Board-tunable wiring. Schematic-fixed pieces (USART/TIM2/TIM1 remaps,
//! OPA inputs, TX_EN, STAT, PWM frequency, ADC sample time) live in
//! [`super::chip`]; anything tunable per board (within the analog/digital
//! pin buckets the chip + this board's free-pin set allow) lives here.

use osc_drivers::Level;

use crate::cfg::chip::{AnalogChannel, DigitalPin};
use crate::hal::{Pin, opa};

/// Bus wire wiring — schema shaped by the `wire-buffered` feature (the wire
/// mode is a compile-time board choice; see `providers/tx_wire`).
#[cfg(feature = "wire-buffered")]
#[derive(Copy, Clone)]
pub struct BusWiring {
    /// 74LVC2G241 direction pin: high = the buffer drives TX onto the data
    /// line and mutes the receive path (inverted enable, same signal);
    /// low = wire released, the data line feeds RX.
    pub tx_en: Pin,
}

/// Bus wire wiring — schema shaped by the `wire-buffered` feature (the wire
/// mode is a compile-time board choice; see `providers/tx_wire`).
#[cfg(not(feature = "wire-buffered"))]
#[derive(Copy, Clone)]
pub struct BusWiring {
    /// TX_EN pin to park LOW on a buffer-populated board running the direct
    /// wire (bypassed rev B — the park keeps the buffer off the jumpered
    /// data line); `None` on boards with no buffer (rev C).
    pub tx_en_park: Option<Pin>,
}

impl BusWiring {
    /// The TX_EN pin this board drives, whatever its role (per-TX-window
    /// direction control, or the direct wire's permanent park).
    pub const fn tx_en_pin(&self) -> Option<Pin> {
        #[cfg(feature = "wire-buffered")]
        {
            Some(self.tx_en)
        }
        #[cfg(not(feature = "wire-buffered"))]
        {
            self.tx_en_park
        }
    }
}

#[derive(Copy, Clone)]
pub struct DrvEn {
    pub pin: DigitalPin,
    /// Level driven to enable the H-bridge driver IC.
    pub active: Level,
}

impl DrvEn {
    pub const fn inactive(&self) -> Level {
        match self.active {
            Level::High => Level::Low,
            Level::Low => Level::High,
        }
    }
}

#[derive(Copy, Clone)]
pub struct CurrentSenseConfig {
    pub gain: opa::Gain,
    pub bias: opa::Bias,
}

#[derive(Copy, Clone)]
pub struct AdcPins {
    pub pos: AnalogChannel,
    pub ntc: AnalogChannel,
    pub vbus: AnalogChannel,
    pub vmotor: (AnalogChannel, AnalogChannel),
}

/// `V_adc = V_in · bot_ohm / (top_ohm + bot_ohm)`.
#[derive(Copy, Clone)]
pub struct Divider {
    pub top_ohm: u32,
    pub bot_ohm: u32,
}

/// β-model NTC params: `R_ntc(T) = r0_ohm · exp(beta · (1/T − 1/T₀))`.
#[derive(Copy, Clone)]
pub struct NtcCal {
    pub beta: u16,
    pub r0_ohm: u32,
    /// T₀ in centi-°C (matches `osc_units::CentiCelsius`).
    pub t0_cc: i16,
    pub bias_r_ohm: u32,
}

/// Schematic-derived constants identical across every unit of a PCB design.
#[derive(Copy, Clone)]
pub struct Calibration {
    pub shunt_r_mohm: u16,
    pub vbus_divider: Divider,
    pub vmotor_divider: Divider,
    pub ntc: NtcCal,
}

/// Board-tunable wiring; consumed during `Ch32ControlIo::new` and not retained.
#[derive(Copy, Clone)]
pub struct BoardWiring {
    /// Scope/probe pad; toggled once per DMA-TC ISR.
    pub dbg: DigitalPin,
    pub drv_en: DrvEn,
    pub bus: BusWiring,
    pub current_sense: CurrentSenseConfig,
    pub sensors: AdcPins,
}

impl BoardWiring {
    /// Compile-time call site: `const _: () = WIRING.assert_valid();`
    pub const fn assert_valid(&self) {
        self.assert_scratch_distinct();
        self.assert_bus_distinct();
        self.assert_sensors_distinct();
    }

    const fn assert_scratch_distinct(&self) {
        if (self.dbg as u8) == (self.drv_en.pin as u8) {
            panic!("BoardWiring: dbg and drv_en.pin must not share a DigitalPin");
        }
    }

    const fn assert_bus_distinct(&self) {
        if let Some(tx_en) = self.bus.tx_en_pin()
            && ((tx_en as u8) == (self.dbg.pin() as u8)
                || (tx_en as u8) == (self.drv_en.pin.pin() as u8))
        {
            panic!("BoardWiring: bus TX_EN must not share a pin with dbg or drv_en");
        }
    }

    const fn assert_sensors_distinct(&self) {
        let chs: [AnalogChannel; 5] = [
            self.sensors.pos,
            self.sensors.ntc,
            self.sensors.vbus,
            self.sensors.vmotor.0,
            self.sensors.vmotor.1,
        ];
        let n = chs.len();
        let mut i = 0;
        while i < n {
            let mut j = i + 1;
            while j < n {
                if (chs[i] as u8) == (chs[j] as u8) {
                    panic!("BoardWiring: duplicate sensor AnalogChannel");
                }
                j += 1;
            }
            i += 1;
        }
    }
}
