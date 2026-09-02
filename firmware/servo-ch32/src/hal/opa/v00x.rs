use ch32_metapac::OPA;

use crate::hal::Pin;

const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

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

/// Pin-selected negative input (OPA_CTLR1.NSEL1).
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum BareNegativeInput {
    PA1 = 0b000,
    PD0 = 0b001,
}

impl BareNegativeInput {
    pub const fn pin(self) -> Pin {
        match self {
            BareNegativeInput::PA1 => Pin::PA1,
            BareNegativeInput::PD0 => Pin::PD0,
        }
    }
}

/// Pin output route (OPA_CTLR1.MODE1). The internal-only route is excluded: an
/// external feedback network needs the amplifier output on a pad.
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum BareOutput {
    PD4 = 0b00,
    PA5 = 0b01,
}

impl BareOutput {
    pub const fn pin(self) -> Pin {
        match self {
            BareOutput::PD4 => Pin::PD4,
            BareOutput::PA5 => Pin::PA5,
        }
    }
}

/// Bare op-amp: both inputs and the output on pins, loop closed externally.
#[derive(Copy, Clone)]
pub struct BareConfig {
    pub pos: PositiveInput,
    pub neg: BareNegativeInput,
    pub out: BareOutput,
}

impl BareConfig {
    /// Pins to configure as analog inputs: (positive, negative, output).
    pub const fn pins(&self) -> (Pin, Pin, Pin) {
        (self.pos.pin(), self.neg.pin(), self.out.pin())
    }
}

fn unlock() {
    OPA.opa_key().write(|w| w.set_opa_key(KEY1));
    OPA.opa_key().write(|w| w.set_opa_key(KEY2));
}

/// Configures OPA1 as a stand-alone op-amp (RM 17.2.1.1): no internal feedback
/// ladder, no bias, gain set by the external resistor network.
pub fn init_bare(cfg: &BareConfig) {
    unlock();
    OPA.ctlr1().write(|w| {
        // HS on: the external-network settle budget needs the bandwidth;
        // the bridge-edge ring is HS-neutral (rev-B bench sweep).
        w.set_opa_hs1(true);
        // VBCMPSEL feeds CMP2 only; 0b11 leaves it unselected.
        w.set_vbcmpsel(0b11);
        w.set_vben(false);
        w.set_pgadif(false);
        w.set_fb_en1(false);
        w.set_nsel1(cfg.neg as u8);
        w.set_psel1(cfg.pos as u8);
        w.set_mode1(cfg.out as u8);
        w.set_opa_en1(true);
    });
}
