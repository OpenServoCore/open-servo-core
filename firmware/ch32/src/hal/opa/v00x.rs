use ch32_metapac::OPA;

use crate::hal::Pin;

const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Gain {
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
    X32 = 0b110,
}

impl Gain {
    pub const fn factor(self) -> u16 {
        match self {
            Gain::X4 => 4,
            Gain::X8 => 8,
            Gain::X16 => 16,
            Gain::X32 => 32,
        }
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum PositiveInput {
    PA2 = 0b00,
    PD7 = 0b01,
    PD3 = 0b10,
    PD1 = 0b11,
}

impl PositiveInput {
    pub const fn pin(self) -> Pin {
        match self {
            PositiveInput::PA2 => Pin::PA2,
            PositiveInput::PD7 => Pin::PD7,
            PositiveInput::PD3 => Pin::PD3,
            PositiveInput::PD1 => Pin::PD1,
        }
    }
}

/// In PGA differential mode (RM 17.2.1.3), the negative pin is OPA_CHN2 (PA4).
#[derive(Copy, Clone)]
pub enum NegativeInput {
    PA4,
}

impl NegativeInput {
    pub const fn pin(self) -> Pin {
        match self {
            NegativeInput::PA4 => Pin::PA4,
        }
    }
}

#[derive(Copy, Clone)]
pub enum InputMode {
    SingleEnded(PositiveInput),
    Differential {
        pos: PositiveInput,
        neg: NegativeInput,
    },
}

impl InputMode {
    pub const fn pos(self) -> PositiveInput {
        match self {
            InputMode::SingleEnded(p) => p,
            InputMode::Differential { pos, .. } => pos,
        }
    }

    pub const fn neg_pin(self) -> Option<Pin> {
        match self {
            InputMode::SingleEnded(_) => None,
            InputMode::Differential { neg, .. } => Some(neg.pin()),
        }
    }
}

/// PGA output bias (RM 17.2.1.4 / OPA_CTLR1.VBEN + VBSEL).
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Bias {
    /// ~VDD/2 quiescent.
    MidRail = 0,
    /// ~VDD/4 quiescent.
    QuarterRail = 1,
}

impl Bias {
    pub const fn quiescent_raw(self) -> u16 {
        match self {
            Bias::MidRail => 2048,
            Bias::QuarterRail => 1024,
        }
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Output {
    PD4 = 0b00,
    PA5 = 0b01,
    /// CMP2 input + ADC channel 9, no pin.
    Internal = 0b11,
}

pub struct Config {
    pub input: InputMode,
    pub gain: Gain,
    pub bias: Bias,
    pub output: Output,
}

fn unlock() {
    OPA.opa_key().write(|w| w.set_opa_key(KEY1));
    OPA.opa_key().write(|w| w.set_opa_key(KEY2));
}

/// Configures OPA1 as a PGA (RM 17.2.1.2–4). Caller settles before sampling.
pub fn init(cfg: &Config) {
    unlock();
    OPA.ctlr1().write(|w| {
        w.set_opa_hs1(true);
        // VBCMPSEL feeds CMP2 only; 0b11 leaves it unselected.
        w.set_vbcmpsel(0b11);
        w.set_vbsel(matches!(cfg.bias, Bias::QuarterRail));
        w.set_vben(true);
        w.set_pgadif(matches!(cfg.input, InputMode::Differential { .. }));
        w.set_fb_en1(true);
        w.set_nsel1(cfg.gain as u8);
        w.set_psel1(cfg.input.pos() as u8);
        w.set_mode1(cfg.output as u8);
        w.set_opa_en1(true);
    });
}
