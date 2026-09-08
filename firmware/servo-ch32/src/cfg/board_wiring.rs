//! Board-tunable wiring. Schematic-fixed pieces (USART/TIM2/TIM1 remaps,
//! STAT, PWM frequency, ADC sample time) live in [`super::chip`]; anything
//! tunable per board (within the analog/digital pin buckets the chip + this
//! board's free-pin set allow) lives here.

use osc_servo_drivers::Level;

use crate::cfg::chip::{AnalogChannel, DigitalPin};
#[cfg(not(feature = "half-duplex"))]
use crate::hal::Pin;
use crate::hal::opa;

/// Bus wire wiring -- exists only on the buffered default wire (the
/// wire mode is a compile-time board choice; see `providers/tx_wire`). The
/// direct wire needs no bus wiring at all: PC0 carries everything, and on a
/// buffer-populated board running the direct wire, the board's TX_EN
/// pull-down is what keeps the buffer released -- no firmware involved.
#[cfg(not(feature = "half-duplex"))]
#[derive(Copy, Clone)]
pub struct BusWiring {
    /// 74LVC2G241 direction pin: high = the buffer drives TX onto the data
    /// line and mutes the receive path (inverted enable, same signal);
    /// low = wire released, the data line feeds RX.
    pub tx_en: Pin,
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
    pub opa: opa::Config,
    /// External feedback network Rf/Rg x1000. The bare op-amp has no gain of
    /// its own, so this is board data, not a chip setting.
    pub gain_milli: u16,
}

impl CurrentSenseConfig {
    /// ADC channel the amplifier output lands on.
    pub const fn current_channel(&self) -> AnalogChannel {
        match self.opa.out {
            opa::Output::PD4 => AnalogChannel::A7,
            // Dead arm: `BoardWiring::assert_valid` rejects a PA5 output at
            // const-eval, because PA5 is not an ADC pin on this package.
            opa::Output::PA5 => AnalogChannel::A7,
        }
    }
}

#[derive(Copy, Clone)]
pub struct AdcPins {
    pub pos: AnalogChannel,
    pub vmotor: (AnalogChannel, AnalogChannel),
    /// Direct supply-rail divider tap.
    pub vbus: AnalogChannel,
    /// NTC divider tap.
    pub ntc: AnalogChannel,
}

impl AdcPins {
    const fn all(&self) -> [AnalogChannel; 5] {
        [self.pos, self.vmotor.0, self.vmotor.1, self.vbus, self.ntc]
    }
}

/// `V_adc = V_in * bot_ohm / (top_ohm + bot_ohm)`.
#[derive(Copy, Clone)]
pub struct Divider {
    pub top_ohm: u32,
    pub bot_ohm: u32,
}

/// NTC to GND under a pull-up to VDD: `V_adc = VDD * R_ntc / (pullup + R_ntc)`.
#[derive(Copy, Clone)]
pub struct Ntc {
    pub pullup_ohm: u32,
    pub r25_ohm: u32,
    pub beta: u16,
}

/// Schematic-derived constants identical across every unit of a PCB design.
#[derive(Copy, Clone)]
pub struct Calibration {
    pub shunt_r_mohm: u16,
    pub vmotor_divider: Divider,
    /// Direct supply-rail divider (`vbus_raw`).
    pub vbus_divider: Divider,
    pub ntc: Ntc,
    /// Nominal bias both motor-terminal dividers return to, ADC counts; the
    /// fallback when the boot measurement's two taps disagree.
    pub vmotor_bias_nom_counts: u16,
    /// DMM-measured VDD at the chip pin; the v006 ADC reference is VDD itself.
    pub vdd_mv: u16,
    /// Shortest drive window (TIM1 ticks) with a valid shunt sample.
    pub i_window_min_ticks: u16,
    /// Shortest drive window (TIM1 ticks) with a valid vmotor sample.
    pub v_window_min_ticks: u16,
}

/// Board-tunable wiring; consumed during `Ch32ControlIo::new` and not retained.
#[derive(Copy, Clone)]
pub struct BoardWiring {
    /// Scope/probe pad; toggled once per DMA-TC ISR.
    pub dbg: DigitalPin,
    pub drv_en: DrvEn,
    #[cfg(not(feature = "half-duplex"))]
    pub bus: BusWiring,
    pub current_sense: CurrentSenseConfig,
    pub sensors: AdcPins,
}

impl BoardWiring {
    /// Compile-time call site: `const _: () = WIRING.assert_valid();`
    pub const fn assert_valid(&self) {
        self.assert_scratch_distinct();
        #[cfg(not(feature = "half-duplex"))]
        self.assert_bus_distinct();
        self.assert_current_output_readable();
        self.assert_sensors_distinct();
        self.assert_sensors_clear_of_opa_inputs();
    }

    const fn assert_current_output_readable(&self) {
        if matches!(self.current_sense.opa.out, opa::Output::PA5) {
            panic!("BoardWiring: OPA output PA5 has no ADC channel on this package");
        }
    }

    const fn assert_scratch_distinct(&self) {
        if (self.dbg as u8) == (self.drv_en.pin as u8) {
            panic!("BoardWiring: dbg and drv_en.pin must not share a DigitalPin");
        }
    }

    #[cfg(not(feature = "half-duplex"))]
    const fn assert_bus_distinct(&self) {
        let tx_en = self.bus.tx_en;
        if (tx_en as u8) == (self.dbg.pin() as u8) || (tx_en as u8) == (self.drv_en.pin.pin() as u8)
        {
            panic!("BoardWiring: bus TX_EN must not share a pin with dbg or drv_en");
        }
    }

    const fn assert_sensors_distinct(&self) {
        let s = self.sensors.all();
        let chs: [AnalogChannel; 6] = [
            self.current_sense.current_channel(),
            s[0],
            s[1],
            s[2],
            s[3],
            s[4],
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

    /// An OPA input pad (PA2 = A0, PA1 = A1, ...) is the amplifier's, not a
    /// sensor's: its analog switch would load the divider.
    const fn assert_sensors_clear_of_opa_inputs(&self) {
        let (opa_pos, opa_neg, _) = self.current_sense.opa.pins();
        let s = self.sensors.all();
        let mut i = 0;
        while i < s.len() {
            let pin = s[i].pin() as u8;
            if pin == opa_pos as u8 || pin == opa_neg as u8 {
                panic!("BoardWiring: sensor AnalogChannel shares a pin with an OPA input");
            }
            i += 1;
        }
    }
}
