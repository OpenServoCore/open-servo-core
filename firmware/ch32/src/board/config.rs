use osc_core::ConfigDefaults;

use crate::hal::{
    Pin, Tim1Mapping, Tim2Mapping, UsartMapping, adc, gpio::Level, gpio::Pull, opa, timer,
};

#[derive(Copy, Clone)]
pub enum Duplex {
    Full,
    Half,
}

#[derive(Copy, Clone)]
pub struct TxEn {
    pub pin: Pin,
    /// Level driven to enable TX; inverse drives RX.
    pub tx_level: Level,
}

impl TxEn {
    pub const fn idle_level(&self) -> Level {
        match self.tx_level {
            Level::High => Level::Low,
            Level::Low => Level::High,
        }
    }
}

#[derive(Copy, Clone)]
pub struct DxlBus {
    pub usart: UsartMapping,
    pub duplex: Duplex,
    pub rx_pull: Pull,
    pub tx_en: Option<TxEn>,
}

#[derive(Copy, Clone)]
pub struct MotorConfig {
    pub tim1: Tim1Mapping,
    pub in1: timer::Channel,
    pub in2: timer::Channel,
    pub drv_en: Pin,
    pub pwm_freq_hz: u32,
    pub polarity: timer::Polarity,
}

impl MotorConfig {
    pub const fn in1_pin(&self) -> Pin {
        tim1_channel_pin(self.tim1, self.in1)
    }

    pub const fn in2_pin(&self) -> Pin {
        tim1_channel_pin(self.tim1, self.in2)
    }
}

const fn tim1_channel_pin(m: Tim1Mapping, c: timer::Channel) -> Pin {
    match c {
        timer::Channel::CH1 => m.ch1_pin(),
        timer::Channel::CH2 => m.ch2_pin(),
        timer::Channel::CH3 => m.ch3_pin(),
        timer::Channel::CH4 => m.ch4_pin(),
    }
}

#[derive(Copy, Clone)]
pub struct CurrentSenseConfig {
    pub opa: opa::Config,
    pub adc_sample_time: adc::SampleTime,
}

#[derive(Copy, Clone)]
pub struct Sensors {
    pub pos: adc::Input,
    pub ntc: adc::Input,
    pub vbus: adc::Input,
    pub vmotor: (adc::Input, adc::Input),
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

/// Schematic-fixed wiring; consumed during `Ch32Board::new` and not retained.
#[derive(Copy, Clone)]
pub struct BoardWiring {
    pub stat_led: Pin,
    /// Scope/probe pad; toggled once per DMA-TC ISR.
    pub dbg: Pin,
    pub tim2_remap: Tim2Mapping,
    pub motor: MotorConfig,
    pub current_sense: CurrentSenseConfig,
    pub sensors: Sensors,
    pub dxl: DxlBus,
}

impl BoardWiring {
    /// Compile-time call site: `const _: () = WIRING.assert_valid();`
    pub const fn assert_valid(&self) {
        self.assert_no_pin_conflicts();
        self.assert_sensors_valid();
    }

    const fn assert_no_pin_conflicts(&self) {
        let pins: [Option<Pin>; 15] = [
            Some(self.stat_led),
            Some(self.dbg),
            Some(self.motor.in1_pin()),
            Some(self.motor.in2_pin()),
            Some(self.motor.drv_en),
            Some(self.current_sense.opa.input.pos().pin()),
            self.current_sense.opa.input.neg_pin(),
            self.sensors.pos.channel.pin(),
            self.sensors.ntc.channel.pin(),
            self.sensors.vbus.channel.pin(),
            self.sensors.vmotor.0.channel.pin(),
            self.sensors.vmotor.1.channel.pin(),
            Some(self.dxl.usart.tx_pin()),
            match self.dxl.duplex {
                Duplex::Full => Some(self.dxl.usart.rx_pin()),
                Duplex::Half => None,
            },
            match self.dxl.tx_en {
                Some(t) => Some(t.pin),
                None => None,
            },
        ];
        let n = pins.len();
        let mut i = 0;
        while i < n {
            let mut j = i + 1;
            while j < n {
                let clash = match (pins[i], pins[j]) {
                    (Some(a), Some(b)) => (a as u8) == (b as u8),
                    _ => false,
                };
                if clash {
                    panic!("BoardWiring: duplicate pin assignment");
                }
                j += 1;
            }
            i += 1;
        }
    }

    const fn assert_sensors_valid(&self) {
        let chs: [adc::Channel; 5] = [
            self.sensors.pos.channel,
            self.sensors.ntc.channel,
            self.sensors.vbus.channel,
            self.sensors.vmotor.0.channel,
            self.sensors.vmotor.1.channel,
        ];
        let n = chs.len();
        let mut i = 0;
        while i < n {
            if chs[i].pin().is_none() {
                panic!("BoardWiring: sensor uses internal ADC channel");
            }
            let mut j = i + 1;
            while j < n {
                if (chs[i] as u8) == (chs[j] as u8) {
                    panic!("BoardWiring: duplicate sensor ADC channel");
                }
                j += 1;
            }
            i += 1;
        }
    }
}

#[derive(Copy, Clone)]
pub struct BoardConfig {
    pub wiring: BoardWiring,
    pub calibration: Calibration,
    pub defaults: ConfigDefaults,
}
